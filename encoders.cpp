#include <cstdio>
#include <cmath>
#include <cstdint>

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/sync.h"
#include "hardware/i2c.h"

#include "encoders.h"
#include "camera.h"
#include "sonar.h"
#include "field.h"

// ====================================================================
// PINS
// ====================================================================
static const uint ENC1_A = 11, ENC1_B = 10;
static const uint ENC2_A = 9,  ENC2_B = 8;
static const uint M1_FWD = 15, M1_BWD = 14;
static const uint M2_FWD = 12, M2_BWD = 13;
static const uint MPU_SDA = 0, MPU_SCL = 1;
static const uint INTAKE_FWD = 6, INTAKE_BWD = 7;

// Intake runs on its own PWM slice (slice 3), independent of the drive
// motors' slices. Uses a higher wrap value and 2.0 clkdiv -- your friend's
// tested IntakePWM.cpp settings -- which gives a lower PWM frequency that
// most small DC motors respond better to.
static const uint32_t INTAKE_PWM_WRAP   = 62500;
static const float    INTAKE_PWM_CLKDIV = 2.0f;

// Percent of wrap to run the intake at (0..100). 30% was your friend's
// tested value. Raise this if balls aren't being pulled in reliably.
static const int      INTAKE_DUTY_PCT   = 30;

// ====================================================================
// ROBOT GEOMETRY -- CALIBRATE THESE
// ====================================================================
static const float WHEEL_RADIUS    = 0.034313f;  // meters (1.351 in radius / 2.70 in dia) -- calibrated 3-run avg
static const float WHEELBASE_IN    = 10.0f;      // MEASURE YOURS
static const float WHEELBASE_M     = WHEELBASE_IN / 39.3701f;
static const int   PPR1            = 1945;   // enc1 measured PPR (new encoder)
static const int   PPR2            = 1963;   // enc2 measured PPR (new encoder)
static const float IN_PER_M        = 39.3701f;

static const float WHEEL_SCALE_M1  = 1.0f;   // reset: new encoder counts at full rate
static const float WHEEL_SCALE_M2  = 1.0f;

// Per-motor base PWM for forward driving. Tune M2_BASE_PWM using drive_raw_test()
// until the robot goes straight with no correction running. M1_BASE_PWM is the
// reference; only M2_BASE_PWM needs changing.
static const int16_t M1_BASE_PWM   = 830;
static const int16_t M2_BASE_PWM   = 1000; // lower until robot drives straight

// No longer needed -- balance is now per-motor above.
static const float M2_SPEED_SCALE  = 1.0f;

static const uint16_t PWM_WRAP = 1000;

// ====================================================================
// ENCODERS
// ====================================================================
static volatile int32_t count1 = 0;
static volatile int32_t count2 = 0;
static volatile uint8_t last_a1 = 0, last_a2 = 0;

static void gpio_irq_callback(uint gpio, uint32_t events) {
    if (gpio == ENC1_A) {
        uint8_t a = gpio_get(ENC1_A);
        uint8_t b = gpio_get(ENC1_B);
        if (a != last_a1) {
            if (b != a) count1++; else count1--;   // A/B swapped on new encoder
            last_a1 = a;
        }
    } else if (gpio == ENC2_A) {
        uint8_t a = gpio_get(ENC2_A);
        uint8_t b = gpio_get(ENC2_B);
        if (a != last_a2) {
            if (b != a) count2--; else count2++;
            last_a2 = a;
        }
    }
}

static void init_encoder_pin(uint pin) {
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_IN);
    gpio_pull_up(pin);
}

static int32_t get_count1() {
    uint32_t ints = save_and_disable_interrupts();
    int32_t c = count1;
    restore_interrupts(ints);
    return c;
}

static int32_t get_count2() {
    uint32_t ints = save_and_disable_interrupts();
    int32_t c = count2;
    restore_interrupts(ints);
    return c;
}

static float wheel_distance_m1(int32_t c) {
    const float circ = 2.0f * (float)M_PI * WHEEL_RADIUS;
    return ((float)c / PPR1) * circ * WHEEL_SCALE_M1;
}
static float wheel_distance_m2(int32_t c) {
    const float circ = 2.0f * (float)M_PI * WHEEL_RADIUS;
    return ((float)c / PPR2) * circ * WHEEL_SCALE_M2;
}

// ====================================================================
// MOTORS
// ====================================================================
static void motor_pin_pwm_init(uint pin) {
    gpio_set_function(pin, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(pin);
    pwm_set_wrap(slice, PWM_WRAP);
    pwm_set_gpio_level(pin, 0);
    pwm_set_enabled(slice, true);
}

// Kickstart parameters. When a motor transitions from stopped (last
// command was 0) to a non-zero command, we briefly slam full PWM in
// the commanded direction to overcome stiction. After KICK_DURATION_MS
// we drop to the actual commanded PWM. This rescues motors with worn
// brushes / draggy gearboxes that won't start at moderate PWM but
// run fine once spinning. Tracked PER MOTOR so a kickstart on M1
// doesn't get blocked by M2 still moving (or vice versa).
//
// Trade-offs:
//  - drive() blocks for KICK_DURATION_MS at every start. Control loops
//    that call drive() every 5ms will pause briefly each time the
//    robot accelerates from a stop.
//  - Kickstart fires both motors at full power even if only one needs
//    it. Healthy motors get a tiny extra jolt at startup, which is
//    harmless but slightly noisier on direction reversals.
static const int16_t  KICK_PWM         = (int16_t)PWM_WRAP;  // full power
static const uint32_t KICK_DURATION_MS = 60;
static int16_t s_last_m1 = 0;
static int16_t s_last_m2 = 0;

// ====================================================================
// LEARNED STRAIGHT-DRIVE BALANCE
// ====================================================================
// Layer 0 is the hand-tuned m2 *= 0.87 in drive() below. That handles
// the bulk static asymmetry. This learned offset sits ON TOP and slowly
// corrects whatever residual drift remains. Updated by
// report_drift_correction() whenever a heading-tolerance pivot fires
// during forward drive.
//
// Convention: positive value SLOWS m1 (subtract from m1, leave m2 alone).
//             negative value SLOWS m2 (subtract |x| from m2, leave m1).
// Effect of "slowing" is applied only when the caller commanded
// approximately-symmetric forward drive (both wheels positive, within
// 30 PWM of each other). Any other command (rotations, reversals, mixed
// direction) bypasses the learned trim because it shouldn't apply there.
//
// Persists for the entire match. Match start = trim is 0; over the
// first few drives the value adapts to whatever the actual asymmetry
// is and stays there.
static int16_t s_balance_trim = 0;
static const float BALANCE_LEARN_RATE = 0.4f;  // PWM points per degree of drift

void report_drift_correction(int32_t dc1, int32_t dc2, float drift_deg) {
    // dc1, dc2 are encoder counts traveled during the segment that just
    // drifted. drift_deg is the heading error that triggered the pivot
    // (signed: positive = robot drifted CCW = m2 was faster).
    //
    // Map the drift magnitude to a learning step. Bigger drift = bigger
    // nudge to the trim. Sign comes from drift direction:
    //   drift > 0 (CCW yaw): m2 was too fast -> increase trim toward
    //     negative (slow m2)
    //   drift < 0 (CW yaw): m1 was too fast -> increase trim toward
    //     positive (slow m1)
    int16_t step = (int16_t)(drift_deg * BALANCE_LEARN_RATE);
    s_balance_trim -= step;
    printf("[balance] drift=%.1f deg dc1=%ld dc2=%ld -> trim adjust by %+d -> %d\n",
           (double)drift_deg, (long)dc1, (long)dc2, (int)(-step),
           (int)s_balance_trim);
}

int16_t get_balance_trim() { return s_balance_trim; }

static inline void apply_motor1_pwm(int16_t v) {
    if (v > 0)      { pwm_set_gpio_level(M1_FWD, v);   pwm_set_gpio_level(M1_BWD, 0);  }
    else if (v < 0) { pwm_set_gpio_level(M1_FWD, 0);   pwm_set_gpio_level(M1_BWD, -v); }
    else            { pwm_set_gpio_level(M1_FWD, 0);   pwm_set_gpio_level(M1_BWD, 0);  }
}
static inline void apply_motor2_pwm(int16_t v) {
    if (v > 0)      { pwm_set_gpio_level(M2_FWD, v);   pwm_set_gpio_level(M2_BWD, 0);  }
    else if (v < 0) { pwm_set_gpio_level(M2_FWD, 0);   pwm_set_gpio_level(M2_BWD, -v); }
    else            { pwm_set_gpio_level(M2_FWD, 0);   pwm_set_gpio_level(M2_BWD, 0);  }
}

void drive(int16_t m1, int16_t m2) {
    // Layer 0: hardcoded asymmetry compensation (Layer 0 from earlier
    // debug session). Robot drifts LEFT (CCW); right wheel goes faster
    // than left, so reduce m2 to 0.87x. Stays as the hand-tuned baseline
    // -- we don't want learning to start from scratch every match.
    int32_t m2_scaled = (int32_t)((float)m2 * M2_SPEED_SCALE);
    m2 = (int16_t)m2_scaled;

    // Layer 2: learned balance trim. Only applies on approximately
    // symmetric forward drive -- pivots and asymmetric maneuvers
    // pass through unchanged. Symmetric = both >0 and within 30 PWM
    // of each other.
    int16_t diff = (int16_t)((int32_t)m1 - (int32_t)m2);
    if (diff < 0) diff = -diff;
    bool symmetric_fwd = (m1 > 0 && m2 > 0 && diff < 30);
    if (symmetric_fwd) {
        if (s_balance_trim > 0) {
            // Slow m1
            int32_t adj = (int32_t)m1 - s_balance_trim;
            m1 = (int16_t)adj;
        } else if (s_balance_trim < 0) {
            // Slow m2
            int32_t adj = (int32_t)m2 + s_balance_trim;
            m2 = (int16_t)adj;
        }
    }

    if (m1 >  (int16_t)PWM_WRAP) m1 =  (int16_t)PWM_WRAP;
    if (m1 < -(int16_t)PWM_WRAP) m1 = -(int16_t)PWM_WRAP;
    if (m2 >  (int16_t)PWM_WRAP) m2 =  (int16_t)PWM_WRAP;
    if (m2 < -(int16_t)PWM_WRAP) m2 = -(int16_t)PWM_WRAP;

    bool m1_starting = (s_last_m1 == 0 && m1 != 0);
    bool m2_starting = (s_last_m2 == 0 && m2 != 0);

    if (m1_starting || m2_starting) {
        // Apply kick PWM in the commanded direction for any motor that's
        // starting. Motors that were already moving (or are commanded to
        // stay stopped) get their normal commanded PWM.
        int16_t kick_m1 = m1_starting ? (m1 > 0 ?  KICK_PWM : -KICK_PWM) : m1;
        int16_t kick_m2 = m2_starting ? (m2 > 0 ?  KICK_PWM : -KICK_PWM) : m2;
        apply_motor1_pwm(kick_m1);
        apply_motor2_pwm(kick_m2);
        sleep_ms(KICK_DURATION_MS);
    }

    apply_motor1_pwm(m1);
    apply_motor2_pwm(m2);

    s_last_m1 = m1;
    s_last_m2 = m2;
}

void stop() { drive(0, 0); }

// ====================================================================
// INTAKE
// ====================================================================
void intake_set_percent(int pct) {
    if (pct >  100) pct =  100;
    if (pct < -100) pct = -100;

    uint32_t level = (INTAKE_PWM_WRAP * (uint32_t)(pct < 0 ? -pct : pct)) / 100u;

    if (pct > 0) {
        pwm_set_gpio_level(INTAKE_FWD, level);
        pwm_set_gpio_level(INTAKE_BWD, 0);
    } else if (pct < 0) {
        pwm_set_gpio_level(INTAKE_FWD, 0);
        pwm_set_gpio_level(INTAKE_BWD, level);
    } else {
        pwm_set_gpio_level(INTAKE_FWD, 0);
        pwm_set_gpio_level(INTAKE_BWD, 0);
    }
}

static void init_intake_pwm() {
    gpio_set_function(INTAKE_FWD, GPIO_FUNC_PWM);
    gpio_set_function(INTAKE_BWD, GPIO_FUNC_PWM);

    uint slice = pwm_gpio_to_slice_num(INTAKE_FWD);
    pwm_set_wrap(slice, INTAKE_PWM_WRAP);
    pwm_set_clkdiv(slice, INTAKE_PWM_CLKDIV);
    pwm_set_gpio_level(INTAKE_FWD, 0);
    pwm_set_gpio_level(INTAKE_BWD, 0);
    pwm_set_enabled(slice, true);
}

// ====================================================================
// MPU9250 + AK8963 register map
// ====================================================================
#define MPU_ADDR         0x68
#define AK_ADDR          0x0C

#define MPU_REG_CONFIG        0x1A
#define MPU_REG_GYRO_C        0x1B
#define MPU_REG_ACCEL_C       0x1C
#define MPU_REG_INT_PIN_CFG   0x37
#define MPU_REG_USER_CTRL     0x6A
#define MPU_REG_PWR           0x6B
#define MPU_REG_WHO_AM_I      0x75
#define MPU_REG_GYRO_Z        0x47

#define AK_REG_WHO_AM_I       0x00
#define AK_REG_STATUS1        0x02
#define AK_REG_HXL            0x03
#define AK_REG_STATUS2        0x09
#define AK_REG_CTRL1          0x0A
#define AK_REG_ASA            0x10

#define MPU_GYRO_FS_SEL 0x00
#define MPU_GYRO_SCALE  131.0f

// ====================================================================
// Magnetometer feature flag + hard-iron offsets
// ====================================================================
// MAG_ENABLED 1 = use mag for diagnostics (stage 2A). The values below
// are ESTIMATED from match-run data showing these observed axis ranges:
//   mx range ~  [+2 .. +318]   -> midpoint +160
//   my range ~ [-605 .. -304]  -> midpoint -454
//   mz range ~ [+136 .. +177]  -> midpoint +157
// When you run the formal calibrate_magnetometer() routine, replace
// these three constants with the exact values it prints.
#define MAG_ENABLED 1
#define MAG_OFFSET_X   +20.3f   // measured avg of 3 runs: 21.7, 17.4, 21.7
#define MAG_OFFSET_Y  -280.2f   // measured avg of 3 runs: -276.4, -284.3, -280.0
#define MAG_OFFSET_Z  +156.0f

// Runtime hard-iron offsets -- initialised from the #defines above but
// overwritten by mag_auto_cal() every match start after the 360-deg spin.
static float s_mag_hard_x = MAG_OFFSET_X;
static float s_mag_hard_y = MAG_OFFSET_Y;

// Latest raw (ASA-scaled, pre-offset) mag readings captured by mpu_update().
// mag_auto_cal() reads these instead of calling ak_read_xyz() a second time
// (which would find no new data since mpu_update() already consumed it).
static float s_mag_latest_x = 0.0f;
static float s_mag_latest_y = 0.0f;
static bool  s_mag_latest_valid = false;

float angle_z = 0.0f;
float fused_heading = 0.0f;
bool  g_sonar_skip_next = false;  // set when sonar blocks a drive; cleared by ST_COVERAGE after double-advance
static uint32_t mpu_last_us = 0;
static bool mpu_ok = false;
static float gyro_bias_dps = 0.0f;

static const float ENC_HEADING_SIGN = +1.0f;
static const float ALPHA_GYRO = 0.98f;

// Forward decl: wrap180 is defined later (with motion primitives) but
// mpu_update() needs it for magnetometer outlier checks and pull math.
static float wrap180(float a);

static int32_t last_c1 = 0;
static int32_t last_c2 = 0;
static bool    fusion_primed = false;

// ====================================================================
// MAGNETOMETER FUSION STATE
// ====================================================================
// Offset that aligns raw mag heading (atan2 of calibrated mx,my) with
// the encoder/gyro fused_heading frame. Computed once at init time.
static float s_mag_to_fused_offset = 0.0f;
static bool  s_mag_offset_set      = false;

// Last accepted mag heading (in fused-frame), used for outlier rejection.
static float s_last_mag_fused = 0.0f;
static bool  s_last_mag_valid = false;

// Set true while rotate_to() is actively pivoting. The mag complementary
// filter is SUSPENDED during rotation -- gyros are accurate over short
// timescales (1-2 sec pivots), but the mag's slow pull rate can't keep
// up with fast yaw rates and will lag, causing fused_heading to overshoot
// and rotate_to to land in completely the wrong orientation. After
// rotation ends, the flag clears and the mag catches fused_heading back
// up over the next few seconds.
static bool s_in_rotation = false;

// How fast the mag pulls fused_heading toward absolute truth.
// 0.005 = 0.5% pull per mag read. With mag reads at ~5 Hz, that's a
// time constant of ~40 seconds — slow enough that motor-induced noise
// doesn't whip the heading around, fast enough that gyro drift over a
// 5 min match stays within ~2 degrees.
static const float MAG_PULL_RATE = 0.02f;

// Reject a mag reading if it differs from the last accepted reading
// by more than this. A genuine fast rotation can move heading 30 deg
// between mag samples, but motor-induced glitches usually jump farther.
// During a rotate_to() we'd lose mag corrections, but rotations are
// short and the gyro handles them.
static const float MAG_OUTLIER_DEG = 25.0f;

static bool mpu_write_reg(uint8_t reg, uint8_t val) {
    uint8_t buf[2] = { reg, val };
    return i2c_write_blocking(i2c0, MPU_ADDR, buf, 2, false) == 2;
}
static bool mpu_read_regs(uint8_t reg, uint8_t* dst, size_t len) {
    if (i2c_write_blocking(i2c0, MPU_ADDR, &reg, 1, true) != 1) return false;
    return i2c_read_blocking(i2c0, MPU_ADDR, dst, len, false) == (int)len;
}
static bool mpu_read_gyro_z_dps(float* out_dps) {
    uint8_t buf[2];
    if (!mpu_read_regs(MPU_REG_GYRO_Z, buf, 2)) return false;
    int16_t raw = (int16_t)(((uint16_t)buf[0] << 8) | buf[1]);
    *out_dps = (float)raw / MPU_GYRO_SCALE;
    return true;
}

static bool mpu_init() {
    i2c_init(i2c0, 400 * 1000);
    gpio_set_function(MPU_SDA, GPIO_FUNC_I2C);
    gpio_set_function(MPU_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(MPU_SDA);
    gpio_pull_up(MPU_SCL);

    sleep_ms(100);

    const int RETRIES = 5;

    uint8_t who = 0xFF;
    bool who_ok = false;
    for (int i = 0; i < RETRIES && !who_ok; i++) {
        who_ok = mpu_read_regs(MPU_REG_WHO_AM_I, &who, 1);
        if (!who_ok) sleep_ms(50);
    }
    if (!who_ok) {
        printf("  [mpu_init] FAIL: cannot read WHO_AM_I — check wiring\n");
        return false;
    }
    printf("  [mpu_init] WHO_AM_I = 0x%02X", who);
    if      (who == 0x71 || who == 0x73) printf(" (MPU9250, OK)\n");
    else if (who == 0x70)                printf(" (MPU6500, no mag)\n");
    else if (who == 0x68)                printf(" (MPU6050, no mag)\n");
    else                                 printf(" (UNKNOWN!)\n");

    bool ok = false;
    for (int i = 0; i < RETRIES && !ok; i++) {
        ok = mpu_write_reg(MPU_REG_PWR, 0x80);
        if (!ok) sleep_ms(50);
    }
    if (!ok) { printf("  [mpu_init] FAIL: reset write failed\n"); return false; }
    sleep_ms(100);

    ok = false;
    for (int i = 0; i < RETRIES && !ok; i++) {
        ok = mpu_write_reg(MPU_REG_PWR, 0x01);
        if (!ok) sleep_ms(50);
    }
    if (!ok) { printf("  [mpu_init] FAIL: wake write failed\n"); return false; }
    sleep_ms(20);

    ok = false;
    for (int i = 0; i < RETRIES && !ok; i++) {
        ok = mpu_write_reg(MPU_REG_GYRO_C, MPU_GYRO_FS_SEL);
        if (!ok) sleep_ms(50);
    }
    if (!ok) { printf("  [mpu_init] FAIL: gyro_config write failed\n"); return false; }
    sleep_ms(10);

    uint8_t rb = 0xFF;
    if (!mpu_read_regs(MPU_REG_GYRO_C, &rb, 1)) {
        printf("  [mpu_init] FAIL: readback failed\n");
        return false;
    }
    printf("  [mpu_init] GYRO_CONFIG readback: 0x%02X\n", rb);
    if (rb != MPU_GYRO_FS_SEL) {
        printf("  [mpu_init] FAIL: readback mismatch\n");
        return false;
    }

#if MAG_ENABLED
    mpu_write_reg(MPU_REG_USER_CTRL,   0x00);
    mpu_write_reg(MPU_REG_INT_PIN_CFG, 0x02);
    sleep_ms(10);
    printf("  [mpu_init] bypass enabled for AK8963 access\n");
#endif

    mpu_ok = true;
    mpu_last_us = time_us_32();
    printf("  [mpu_init] success\n");
    return true;
}

// ====================================================================
// AK8963 (magnetometer on MPU9250)
// ====================================================================
#if MAG_ENABLED
static bool ak_ok = false;
static float ak_asa_x = 1.0f, ak_asa_y = 1.0f, ak_asa_z = 1.0f;

static bool ak_write_reg(uint8_t reg, uint8_t val) {
    uint8_t buf[2] = { reg, val };
    return i2c_write_blocking(i2c0, AK_ADDR, buf, 2, false) == 2;
}
static bool ak_read_regs(uint8_t reg, uint8_t* dst, size_t len) {
    if (i2c_write_blocking(i2c0, AK_ADDR, &reg, 1, true) != 1) return false;
    return i2c_read_blocking(i2c0, AK_ADDR, dst, len, false) == (int)len;
}

static bool ak_init() {
    uint8_t who = 0xFF;
    if (!ak_read_regs(AK_REG_WHO_AM_I, &who, 1)) {
        printf("  [ak_init] FAIL: no response from AK8963 at 0x%02X\n", AK_ADDR);
        return false;
    }
    printf("  [ak_init] WHO_AM_I = 0x%02X (want 0x48)\n", who);
    if (who != 0x48) {
        printf("  [ak_init] FAIL: wrong device ID\n");
        return false;
    }

    ak_write_reg(AK_REG_CTRL1, 0x00); sleep_ms(10);
    ak_write_reg(AK_REG_CTRL1, 0x0F); sleep_ms(10);

    uint8_t asa[3];
    if (!ak_read_regs(AK_REG_ASA, asa, 3)) {
        printf("  [ak_init] FAIL: could not read ASA\n");
        return false;
    }
    ak_asa_x = ((float)asa[0] + 128.0f) / 256.0f;
    ak_asa_y = ((float)asa[1] + 128.0f) / 256.0f;
    ak_asa_z = ((float)asa[2] + 128.0f) / 256.0f;
    printf("  [ak_init] ASA raw = %u,%u,%u  scale = %.3f,%.3f,%.3f\n",
           asa[0], asa[1], asa[2],
           (double)ak_asa_x, (double)ak_asa_y, (double)ak_asa_z);

    ak_write_reg(AK_REG_CTRL1, 0x00); sleep_ms(10);
    ak_write_reg(AK_REG_CTRL1, 0x16); sleep_ms(10);

    ak_ok = true;
    printf("  [ak_init] success (continuous 100 Hz, 16-bit)\n");
    return true;
}

static bool ak_read_xyz(float* mx, float* my, float* mz) {
    if (!ak_ok) return false;

    uint8_t st1;
    if (!ak_read_regs(AK_REG_STATUS1, &st1, 1)) return false;
    if (!(st1 & 0x01)) return false;

    uint8_t buf[7];
    if (!ak_read_regs(AK_REG_HXL, buf, 7)) return false;

    int16_t rx = (int16_t)(((uint16_t)buf[1] << 8) | buf[0]);
    int16_t ry = (int16_t)(((uint16_t)buf[3] << 8) | buf[2]);
    int16_t rz = (int16_t)(((uint16_t)buf[5] << 8) | buf[4]);
    uint8_t st2 = buf[6];

    if (st2 & 0x08) return false;

    *mx = (float)rx * ak_asa_x;
    *my = (float)ry * ak_asa_y;
    *mz = (float)rz * ak_asa_z;
    return true;
}

// One-time hard-iron calibration routine. Call this ONCE from init_mpu()
// (uncomment the line at the bottom of init_mpu()), capture the numbers
// it prints, hard-code them into MAG_OFFSET_X/Y/Z above, then comment
// the call back out. Don't run at every boot.
void calibrate_magnetometer() {
    printf("\n");
    printf("==========================================================\n");
    printf("  MAG CALIBRATION: slowly rotate robot for 30 seconds\n");
    printf("  Do at least TWO full turns. Keep robot level.\n");
    printf("==========================================================\n");

    float mx_min =  1e9f, mx_max = -1e9f;
    float my_min =  1e9f, my_max = -1e9f;
    float mz_min =  1e9f, mz_max = -1e9f;

    const uint32_t DURATION_MS = 30000;
    uint32_t t0 = to_ms_since_boot(get_absolute_time());
    uint32_t samples = 0;
    uint32_t last_progress = t0;

    while (to_ms_since_boot(get_absolute_time()) - t0 < DURATION_MS) {
        float mx, my, mz;
        if (ak_read_xyz(&mx, &my, &mz)) {
            if (mx < mx_min) mx_min = mx;
            if (mx > mx_max) mx_max = mx;
            if (my < my_min) my_min = my;
            if (my > my_max) my_max = my;
            if (mz < mz_min) mz_min = mz;
            if (mz > mz_max) mz_max = mz;
            samples++;
        }

        uint32_t now = to_ms_since_boot(get_absolute_time());
        if (now - last_progress > 2000) {
            last_progress = now;
            uint32_t remaining = (DURATION_MS - (now - t0)) / 1000;
            printf("  [cal] %2lus left  samples=%lu  "
                   "x=[%+.0f..%+.0f] y=[%+.0f..%+.0f] z=[%+.0f..%+.0f]\n",
                   (unsigned long)remaining, (unsigned long)samples,
                   (double)mx_min, (double)mx_max,
                   (double)my_min, (double)my_max,
                   (double)mz_min, (double)mz_max);
        }
        sleep_ms(10);
    }

    float ox = (mx_max + mx_min) * 0.5f;
    float oy = (my_max + my_min) * 0.5f;
    float oz = (mz_max + mz_min) * 0.5f;

    float rx = (mx_max - mx_min) * 0.5f;
    float ry = (my_max - my_min) * 0.5f;
    float rz = (mz_max - mz_min) * 0.5f;

    printf("\n");
    printf("==========================================================\n");
    printf("  CALIBRATION COMPLETE — paste these into encoders.cpp:\n");
    printf("==========================================================\n");
    printf("  #define MAG_OFFSET_X %+.1ff\n", (double)ox);
    printf("  #define MAG_OFFSET_Y %+.1ff\n", (double)oy);
    printf("  #define MAG_OFFSET_Z %+.1ff\n", (double)oz);
    printf("\n");
    printf("  // Half-range per axis (for diagnostic reference only):\n");
    printf("  //   X: %.1f  Y: %.1f  Z: %.1f\n",
           (double)rx, (double)ry, (double)rz);
    printf("==========================================================\n");
    printf("\n");

    if (rx < 50.0f || ry < 50.0f) {
        printf("  WARNING: axis range is very small. Either the robot\n");
        printf("  didn't rotate enough, or the mag is saturated/damaged.\n");
        printf("  Expected half-range ~100-300 counts at typical latitudes.\n\n");
    }
}
// Automated hard-iron calibration. Spins the robot one full revolution using
// the motors, tracks raw mx/my min/max via mpu_update(), then computes true
// offsets and stores them in s_mag_hard_x/y. Call after init_mpu() and
// init_motors() but before set_pose(). The gyro (not the mag) determines
// when 360 deg is complete, so no chicken-and-egg dependency.
void mag_auto_cal() {
    if (!ak_ok) {
        printf("[mag_auto_cal] mag not ready, skipping\n");
        return;
    }

    printf("[mag_auto_cal] spinning 360 deg to calibrate hard-iron offsets...\n");
    sleep_ms(500);

    const int16_t  SPIN_PWM   = 350;
    const float    TARGET_DEG = 375.0f;  // slightly over 360 for full coverage
    const uint32_t TIMEOUT_MS = 8000;

    float    mx_min =  1e9f, mx_max = -1e9f;
    float    my_min =  1e9f, my_max = -1e9f;
    uint32_t samples = 0;

    s_in_rotation = true;   // suppress mag pull during spin
    mpu_update();
    float    start_heading = fused_heading;
    float    start_angle   = angle_z;
    uint32_t t0 = to_ms_since_boot(get_absolute_time());

    drive(+SPIN_PWM, -SPIN_PWM);   // CCW pivot

    while (true) {
        mpu_update();   // updates angle_z from gyro; also sets s_mag_latest_*

        if (s_mag_latest_valid) {
            s_mag_latest_valid = false;
            if (s_mag_latest_x < mx_min) mx_min = s_mag_latest_x;
            if (s_mag_latest_x > mx_max) mx_max = s_mag_latest_x;
            if (s_mag_latest_y < my_min) my_min = s_mag_latest_y;
            if (s_mag_latest_y > my_max) my_max = s_mag_latest_y;
            samples++;
        }

        if (fabsf(angle_z - start_angle) >= TARGET_DEG) break;
        if (to_ms_since_boot(get_absolute_time()) - t0 > TIMEOUT_MS) {
            printf("[mag_auto_cal] TIMEOUT at %.1f deg -- using partial data\n",
                   (double)fabsf(angle_z - start_angle));
            break;
        }
        sleep_ms(5);
    }

    stop();
    s_in_rotation = false;

    if (samples < 20 || mx_max <= mx_min || my_max <= my_min) {
        printf("[mag_auto_cal] bad data (samples=%lu), keeping old offsets\n",
               (unsigned long)samples);
        return;
    }

    s_mag_hard_x = (mx_max + mx_min) * 0.5f;
    s_mag_hard_y = (my_max + my_min) * 0.5f;

    // Re-anchor mag fusion with the new offsets on the next mag read.
    s_mag_offset_set = false;
    s_last_mag_valid = false;

    printf("[mag_auto_cal] done  samples=%lu\n", (unsigned long)samples);
    printf("  x [%.0f .. %.0f] -> offset %.1f\n",
           (double)mx_min, (double)mx_max, (double)s_mag_hard_x);
    printf("  y [%.0f .. %.0f] -> offset %.1f\n",
           (double)my_min, (double)my_max, (double)s_mag_hard_y);

    // Rotate back to the pre-cal heading so the robot is physically facing
    // the same direction it was before the spin. rotate_to handles mag suspend/resume.
    printf("[mag_auto_cal] returning to start heading %.1f\n", (double)start_heading);
    s_in_rotation = false;
    rotate_to(start_heading);
}
#endif  // MAG_ENABLED

static float sample_gyro_bias(int N) {
    float sum = 0.0f; int ok = 0;
    for (int i = 0; i < N; i++) {
        float dps;
        if (mpu_read_gyro_z_dps(&dps)) { sum += dps; ok++; }
        sleep_ms(5);
    }
    return (ok > 0) ? sum / (float)ok : 0.0f;
}

void init_mpu() {
    if (!mpu_init()) { printf("MPU FAIL\n"); return; }
    printf("MPU OK, calibrating gyro bias (keep robot still)...\n");
    gyro_bias_dps = sample_gyro_bias(200);
    printf("Gyro bias: %.3f dps\n", (double)gyro_bias_dps);

    angle_z = 0.0f;
    fused_heading = 0.0f;
    mpu_last_us = time_us_32();

#if MAG_ENABLED
    printf("[ak_init] waking magnetometer...\n");
    ak_init();

    // ONE-TIME CALIBRATION:
    // Uncomment the next 4 lines to run the full calibration routine
    // on boot. Flash, power up, slowly rotate the robot for 30 seconds
    // (at least 2 full turns), capture the MAG_OFFSET_X/Y/Z values it
    // prints, paste them into the #defines above, then re-comment these
    // lines so it doesn't run every boot.
    // if (ak_ok) {
    //     sleep_ms(2000);
    //     calibrate_magnetometer();
    // }
#endif
}

void refresh_gyro_bias() {
    if (!mpu_ok) return;
    stop();
    sleep_ms(150);
    float new_bias = sample_gyro_bias(60);
    float delta = new_bias - gyro_bias_dps;
    if (fabsf(delta) < 2.0f) {
        gyro_bias_dps = new_bias;
        printf("[bias refresh] new=%.3f (delta %+.3f)\n",
               (double)new_bias, (double)delta);
    } else {
        printf("[bias refresh] rejected: new=%.3f delta=%+.3f (too large)\n",
               (double)new_bias, (double)delta);
    }
}

void mpu_update() {
    if (!mpu_ok) return;

    // --- Read magnetometer EVERY update (not just for printing) ---
    // Mag data is the absolute-truth anchor for heading. We read it as
    // often as it'll give us data and stash the most recent valid value
    // for the pull step at the end of mpu_update().
#if MAG_ENABLED
    static float    s_mag_h_pending      = 0.0f;
    static bool     s_mag_h_pending_valid = false;
    static uint32_t s_last_mag_print_ms  = 0;
    {
        float mx, my, mz;
        if (ak_read_xyz(&mx, &my, &mz)) {
            s_mag_latest_x     = mx;
            s_mag_latest_y     = my;
            s_mag_latest_valid = true;
            float mxc = mx - s_mag_hard_x;
            float myc = my - s_mag_hard_y;
            // Robot forward = chip -X, robot left = chip -Y.
            float h = atan2f(-myc, -mxc) * 180.0f / (float)M_PI;
            s_mag_h_pending       = h;
            s_mag_h_pending_valid = true;

            // Diagnostic print at 1 Hz so the log doesn't get flooded.
            uint32_t now_ms = to_ms_since_boot(get_absolute_time());
            if (now_ms - s_last_mag_print_ms > 1000) {
                s_last_mag_print_ms = now_ms;
                printf("  [mag] raw x=%+7.1f y=%+7.1f z=%+7.1f  cal_h=%+6.1f\n",
                       (double)mx, (double)my, (double)mz, (double)h);
            }
        }
    }
#endif

    float dps = 0.0f;
    bool gyro_ok = mpu_read_gyro_z_dps(&dps);

    uint32_t now = time_us_32();
    float dt = (float)(now - mpu_last_us) / 1e6f;
    mpu_last_us = now;

    float d_gyro = gyro_ok ? (dps - gyro_bias_dps) * dt : 0.0f;
    angle_z += d_gyro;

    int32_t c1 = get_count1();
    int32_t c2 = get_count2();
    if (!fusion_primed) {
        last_c1 = c1; last_c2 = c2;
        fusion_primed = true;
        return;
    }

    int32_t dc1 = c1 - last_c1;
    int32_t dc2 = c2 - last_c2;
    last_c1 = c1; last_c2 = c2;

    float dd1 = wheel_distance_m1(dc1);
    float dd2 = wheel_distance_m2(dc2);

    float d_enc_rad = ENC_HEADING_SIGN * (dd2 - dd1) / WHEELBASE_M;
    float d_enc_deg = d_enc_rad * 180.0f / (float)M_PI;

    fused_heading += (1.0f - ALPHA_GYRO) * d_enc_deg
                   +         ALPHA_GYRO  * d_gyro;

    // --- Stage B: magnetometer complementary pull ---
    // After gyro+encoder fusion gives us a short-term-accurate heading,
    // gently pull it toward the absolute mag heading. This corrects
    // gyro drift (and any encoder miscalibration) over a long timescale
    // without yanking the heading around on individual noisy samples.
    //
    // First valid mag reading after a set_pose() captures the offset
    // between mag's frame and our fused-heading frame, so subsequent
    // pulls don't snap the robot's heading to mag's reference (e.g. if
    // mag reads -20 when we set pose to 90, offset = +110 and from
    // then on we pull toward (mag + 110), which equals 90 at start).
    //
    // Outlier rejection: if a new mag reading differs from the last
    // accepted one by more than MAG_OUTLIER_DEG, skip it. Real fast
    // rotation can move heading 25 deg between samples; sudden glitches
    // usually jump farther.
#if MAG_ENABLED
    if (s_mag_h_pending_valid) {
        s_mag_h_pending_valid = false;     // consume

        // While rotate_to() is actively pivoting, skip the complementary
        // pull entirely. The mag's slow pull rate lags fast yaw, which
        // makes fused_heading underestimate the rotation and causes
        // rotate_to to exit at the wrong heading. Pure gyro handles short
        // pivots fine.
        if (s_in_rotation) {
            // fall through; sample already consumed
        } else {
        // Outlier check vs last ACCEPTED mag-in-fused-frame reading.
        bool accept = true;
        if (s_last_mag_valid) {
            float jump = wrap180(s_mag_h_pending + s_mag_to_fused_offset
                                 - s_last_mag_fused);
            if (fabsf(jump) > MAG_OUTLIER_DEG) accept = false;
        }

        if (accept) {
            // First valid sample: capture offset that aligns mag-frame
            // with current fused_heading.
            if (!s_mag_offset_set) {
                s_mag_to_fused_offset = wrap180(fused_heading - s_mag_h_pending);
                s_mag_offset_set = true;
                printf("  [mag] anchor captured: fused=%.1f mag=%.1f offset=%.1f\n",
                       (double)fused_heading, (double)s_mag_h_pending,
                       (double)s_mag_to_fused_offset);
            }

            float mag_in_fused = wrap180(s_mag_h_pending + s_mag_to_fused_offset);
            s_last_mag_fused = mag_in_fused;
            s_last_mag_valid = true;

            // Complementary pull: nudge fused_heading toward mag_in_fused.
            // wrap180 the error so we always pull the short way around.
            float err = wrap180(mag_in_fused - fused_heading);
            fused_heading = wrap180(fused_heading + MAG_PULL_RATE * err);
        }
        }   // end of "not in rotation" branch
    }
#endif
}

// ====================================================================
// POSITION
// ====================================================================
static float pos_x = 0, pos_y = 0;
static int32_t pos_last_c1 = 0, pos_last_c2 = 0;
static bool    pos_primed = false;

void position_update() {
    int32_t c1 = get_count1();
    int32_t c2 = get_count2();
    if (!pos_primed) {
        pos_last_c1 = c1; pos_last_c2 = c2;
        pos_primed = true;
        return;
    }
    float dd1 = wheel_distance_m1(c1 - pos_last_c1);
    float dd2 = wheel_distance_m2(c2 - pos_last_c2);
    pos_last_c1 = c1; pos_last_c2 = c2;

    float delta = (dd1 + dd2) * 0.5f;
    float rad = fused_heading * (float)M_PI / 180.0f;
    pos_x += delta * cosf(rad);
    pos_y += delta * sinf(rad);
}

float get_pos_x_in() { return pos_x * IN_PER_M; }
float get_pos_y_in() { return pos_y * IN_PER_M; }

void set_pose(float x_in, float y_in, float heading_deg) {
    pos_x = x_in / IN_PER_M;
    pos_y = y_in / IN_PER_M;
    fused_heading = heading_deg;
    angle_z = heading_deg;

    last_c1 = get_count1();
    last_c2 = get_count2();
    fusion_primed = true;

    pos_last_c1 = get_count1();
    pos_last_c2 = get_count2();
    pos_primed  = true;

    // Reset mag-fusion offset so it re-locks on the next mag read,
    // anchored to the new heading we just set.
    s_mag_offset_set = false;
    s_last_mag_valid = false;

    printf("[set_pose] x=%.1f y=%.1f head=%.1f\n",
           (double)x_in, (double)y_in, (double)heading_deg);
}

// ====================================================================
// MOTION PRIMITIVES
// ====================================================================
static float wrap180(float a) {
    while (a >  180.0f) a -= 360.0f;
    while (a <= -180.0f) a += 360.0f;
    return a;
}

void rotate_to(float target_deg) {
    printf("  [rotate_to] target=%.1f from current=%.1f\n",
        (double)target_deg, (double)fused_heading);
    const float    ANGLE_TOL_DEG  = 1.5f;
    const int16_t  MAX_PWM        = 350;
    const int16_t  MIN_PWM        = 220;
    const float    SLOW_ZONE_DEG  = 30.0f;
    const uint32_t TIMEOUT_MS     = 3000;
    const uint32_t SETTLE_MS      = 100;

    // Suspend mag fusion -- mag's slow pull rate can't track a fast pivot,
    // and fused_heading would lag/overshoot if the filter ran during it.
    // Pure gyro is accurate enough over a 1-2 sec rotate window. Resume
    // fusion in the SETTLE phase below so mag re-anchors before the next
    // drive command.
    s_in_rotation = true;

    uint32_t t0 = to_ms_since_boot(get_absolute_time());

    while (true) {
        mpu_update();

        float err = wrap180(target_deg - fused_heading);
        float abs_err = fabsf(err);
        if (abs_err <= ANGLE_TOL_DEG) break;
        if (to_ms_since_boot(get_absolute_time()) - t0 > TIMEOUT_MS) break;

        int16_t pwm = MAX_PWM;
        if (abs_err < SLOW_ZONE_DEG) {
            float ratio = abs_err / SLOW_ZONE_DEG;
            pwm = (int16_t)(MIN_PWM + (float)(MAX_PWM - MIN_PWM) * ratio);
        }

        if (err > 0.0f) drive(+pwm, -pwm);
        else            drive(-pwm, +pwm);

        sleep_ms(5);
    }

    printf("  [rotate_to] exit, fused=%.1f (target was %.1f)\n",
        (double)fused_heading, (double)target_deg);

    stop();

    // Re-enable mag fusion. The settle window lets the mag pull rate
    // gently correct any gyro drift accumulated during the pivot before
    // the caller starts driving.
    s_in_rotation = false;

    uint32_t settle_end = to_ms_since_boot(get_absolute_time()) + SETTLE_MS;
    while (to_ms_since_boot(get_absolute_time()) < settle_end) {
        mpu_update();
        sleep_ms(5);
    }
}

void rotate_to_slow(float target_deg) {
    printf("  [rotate_to_slow] target=%.1f from current=%.1f\n",
        (double)target_deg, (double)fused_heading);
    const float    ANGLE_TOL_DEG = 2.0f;
    const int16_t  MAX_PWM       = 200;
    const int16_t  MIN_PWM       = 130;
    const float    SLOW_ZONE_DEG = 90.0f;   // always in slow zone
    const uint32_t TIMEOUT_MS    = 5000;
    const uint32_t SETTLE_MS     = 150;

    s_in_rotation = true;
    uint32_t t0 = to_ms_since_boot(get_absolute_time());

    while (true) {
        mpu_update();
        float err     = wrap180(target_deg - fused_heading);
        float abs_err = fabsf(err);
        if (abs_err <= ANGLE_TOL_DEG) break;
        if (to_ms_since_boot(get_absolute_time()) - t0 > TIMEOUT_MS) break;

        float ratio = (abs_err < SLOW_ZONE_DEG) ? (abs_err / SLOW_ZONE_DEG) : 1.0f;
        int16_t pwm = (int16_t)(MIN_PWM + (float)(MAX_PWM - MIN_PWM) * ratio);

        if (err > 0.0f) drive(+pwm, -pwm);
        else            drive(-pwm, +pwm);

        sleep_ms(5);
    }

    stop();
    s_in_rotation = false;

    printf("  [rotate_to_slow] exit, fused=%.1f (target was %.1f)\n",
        (double)fused_heading, (double)target_deg);

    uint32_t settle_end = to_ms_since_boot(get_absolute_time()) + SETTLE_MS;
    while (to_ms_since_boot(get_absolute_time()) < settle_end) {
        mpu_update();
        sleep_ms(5);
    }
}

// Obstacle bypass: rotate toward field center, drive 2 s in that direction,
// then reorient toward (tx, ty) computed from the new position so the
// approach angle to the waypoint changes and avoids the same obstacle.
static void sonar_bypass(float tx, float ty) {
    const float    BYPASS_GAIN = 12.0f;
    const int16_t  BYPASS_MAX  = 80;
    const uint32_t BYPASS_MS   = 2000;

    float dx_c = (FIELD_X_IN * 0.5f) - get_pos_x_in();
    float dy_c = (FIELD_Y_IN * 0.5f) - get_pos_y_in();
    float center_bearing = atan2f(dy_c, dx_c) * 180.0f / (float)M_PI;

    printf("[sonar_bypass] -> center bearing=%.1f, drive 2s\n",
           (double)center_bearing);

    rotate_to(center_bearing);

    s_in_rotation = true;
    apply_motor1_pwm(KICK_PWM);
    apply_motor2_pwm(KICK_PWM);
    sleep_ms(KICK_DURATION_MS);

    uint32_t t0 = to_ms_since_boot(get_absolute_time());
    while (to_ms_since_boot(get_absolute_time()) - t0 < BYPASS_MS) {
        mpu_update();
        float err = wrap180(center_bearing - fused_heading);
        int16_t htrim = (int16_t)(err * BYPASS_GAIN);
        if (htrim >  BYPASS_MAX) htrim =  BYPASS_MAX;
        if (htrim < -BYPASS_MAX) htrim = -BYPASS_MAX;
        int16_t m1 = (int16_t)(M1_BASE_PWM + htrim);
        int16_t m2 = (int16_t)(M2_BASE_PWM - htrim);
        if (m1 > (int16_t)PWM_WRAP) m1 = (int16_t)PWM_WRAP;
        if (m2 > (int16_t)PWM_WRAP) m2 = (int16_t)PWM_WRAP;
        if (m1 < 0) m1 = 0;
        if (m2 < 0) m2 = 0;
        apply_motor1_pwm(m1);
        apply_motor2_pwm(m2);
        sleep_ms(5);
    }
    stop();
    s_in_rotation = false;

    // Recompute bearing from new position to the target waypoint so the
    // approach angle changes — avoids re-hitting the same obstacle.
    float dx_t = tx - get_pos_x_in();
    float dy_t = ty - get_pos_y_in();
    float fresh_bearing = atan2f(dy_t, dx_t) * 180.0f / (float)M_PI;
    printf("[sonar_bypass] new pos=(%.1f,%.1f) fresh bearing=%.1f\n",
           (double)get_pos_x_in(), (double)get_pos_y_in(), (double)fresh_bearing);
    rotate_to(fresh_bearing);
    printf("[sonar_bypass] done -- caller retries waypoint from new pos\n");
}

void drive_distance(float inches) {
    printf("  [drive_distance] target=%.1f in (%.3f m), starting from heading=%.1f\n",
        (double)inches, (double)(inches / IN_PER_M), (double)fused_heading);

    // Suspend mag pull for the entire drive. The mag is anchored to the start
    // heading (90 deg) and pulls fused_heading toward it even when driving at
    // other bearings -- this fights heading corrections and causes HARD to fire
    // in a loop. Gyro+encoder fusion is accurate enough over one waypoint segment.
    s_in_rotation = true;

    const float    DIST_TOL_M      = 0.02f;
    const uint32_t TIMEOUT_MS      = (uint32_t)(fabsf(inches) * 250.0f) + 3000;

    // --- Zone thresholds ---
    // Zone 3 (hard stop + pivot): |err| > HARD_STOP_DEG
    // Zone 2 (soft PWM nudge):    SOFT_DEG < |err| <= HARD_STOP_DEG
    // Zone 1 (rate matching):     |err| <= SOFT_DEG
    const float   HARD_STOP_DEG  = 5.0f;
    const float   SOFT_DEG       = 0.8f;
    const float   UNDERSHOOT_DEG = 1.0f;   // aim short of ref to land on it after overshoot
    const float   SOFT_GAIN      = 12.0f;  // PWM points per degree of error
    const int16_t SOFT_MAX_TRIM  = 80;     // cap on soft heading correction

    // --- Rate matching (zone 1 only) ---
    const uint32_t WHEEL_WINDOW_MS    = 100;
    const float    WHEEL_RATE_TOL     = 0.05f;
    const int16_t  WHEEL_TRIM_STEP    = 15;
    const float    WHEEL_MIN_PWM_FRAC = 0.80f;

    // --- Sonar ---
    const bool     SONAR_ACTIVE      = (inches >= 3.0f);
    const uint32_t SONAR_INTERVAL_MS = 200;
    const float    SONAR_STOP_CM     = 25.40f;  // 10 inches
    const float    SONAR_BACKUP_IN   = -4.0f;
    const uint32_t SONAR_WAIT_MS     = 10000;
    uint32_t       last_sonar        = 0;

    int16_t  pwm_floor    = (int16_t)((float)M1_BASE_PWM * WHEEL_MIN_PWM_FRAC);
    int16_t  m1_rate_trim = 0;
    int16_t  m2_rate_trim = 0;
    int32_t  c1_window    = get_count1();
    int32_t  c2_window    = get_count2();
    uint32_t window_t0    = to_ms_since_boot(get_absolute_time());

    float   target_m = inches / IN_PER_M;
    float   target_abs = fabsf(target_m);
    int16_t base1    = (target_m >= 0.0f) ? M1_BASE_PWM : -M1_BASE_PWM;
    int16_t base2    = (target_m >= 0.0f) ? M2_BASE_PWM : -M2_BASE_PWM;

    mpu_update();
    float ref_heading = fused_heading;

    int32_t  c1_start        = get_count1();
    int32_t  c2_start        = get_count2();
    uint32_t t0              = to_ms_since_boot(get_absolute_time());
    uint32_t hard_corrections = 0;
    uint32_t soft_corrections = 0;

    while (true) {
        mpu_update();
        position_update();

        int32_t c1 = get_count1() - c1_start;
        int32_t c2 = get_count2() - c2_start;
        float d1 = wheel_distance_m1(c1);
        float d2 = wheel_distance_m2(c2);
        float avg_abs = (fabsf(d1) + fabsf(d2)) * 0.5f;

        if (avg_abs >= target_abs - DIST_TOL_M) break;
        if (to_ms_since_boot(get_absolute_time()) - t0 > TIMEOUT_MS) {
            printf("  [drive_distance] TIMEOUT  dist=%.2fin  enc1=%ld enc2=%ld\n",
                   (double)(avg_abs * IN_PER_M),
                   (long)(get_count1() - c1_start),
                   (long)(get_count2() - c2_start));
            break;
        }

        uint32_t now = to_ms_since_boot(get_absolute_time());

        // --- Sonar safety (forward only, unchanged) ---
        if (SONAR_ACTIVE && (now - last_sonar > SONAR_INTERVAL_MS)) {
            last_sonar = now;
            float dist_cm = read_nearest_cm();
            if (dist_cm > 0.0f && dist_cm <= SONAR_STOP_CM) {
                printf("  [sonar] OBSTACLE at %.1f cm during drive_distance! "
                       "stop, poll up to %lums for clearance\n",
                       (double)dist_cm, (unsigned long)SONAR_WAIT_MS);
                stop();

                bool cleared = false;
                uint32_t wait_t0 = to_ms_since_boot(get_absolute_time());
                while (to_ms_since_boot(get_absolute_time()) - wait_t0 < SONAR_WAIT_MS) {
                    for (int i = 0; i < 20; i++) {
                        mpu_update();
                        position_update();
                        sleep_ms(50);
                    }
                    float check_cm = read_nearest_cm();
                    if (check_cm < 0.0f) {
                        printf("  [sonar] poll: no echo (clear)\n");
                        cleared = true; break;
                    } else if (check_cm > SONAR_STOP_CM) {
                        printf("  [sonar] poll: clear (%.1f cm) - resuming\n",
                               (double)check_cm);
                        cleared = true; break;
                    } else {
                        printf("  [sonar] poll: still blocked (%.1f cm)\n",
                               (double)check_cm);
                    }
                }

                if (cleared) { mpu_update(); position_update(); continue; }

                printf("  [sonar] still blocked after %lums - skipping waypoint\n",
                       (unsigned long)SONAR_WAIT_MS);
                g_sonar_skip_next = true;
                return;
            }
        }

        float heading_err = wrap180(ref_heading - fused_heading);
        float abs_err     = fabsf(heading_err);

        if (abs_err > HARD_STOP_DEG) {
            // ---- Zone 3: hard stop + undershoot pivot ----
            hard_corrections++;
            float pivot_target = ref_heading - copysignf(UNDERSHOOT_DEG, heading_err);
            printf("  [drive] HARD#%lu  err=%.1f  pivot->%.1f\n",
                   (unsigned long)hard_corrections,
                   (double)heading_err, (double)pivot_target);

            // Save counts before pivot so the rotation doesn't corrupt
            // the remaining-distance accumulator.
            int32_t pre1 = get_count1();
            int32_t pre2 = get_count2();
            stop();
            sleep_ms(50);
            rotate_to(pivot_target);
            sleep_ms(50);
            // Shift c1_start/c2_start by however much the pivot moved the
            // encoders -- only actual forward travel counts toward distance.
            c1_start += get_count1() - pre1;
            c2_start += get_count2() - pre2;
            // Reset rate-match window after pivot
            c1_window = get_count1();
            c2_window = get_count2();
            window_t0 = to_ms_since_boot(get_absolute_time());

        } else if (abs_err > SOFT_DEG) {
            // ---- Zone 2: soft PWM nudge while driving ----
            soft_corrections++;
            int16_t htrim = (int16_t)(heading_err * SOFT_GAIN);
            if (htrim >  SOFT_MAX_TRIM) htrim =  SOFT_MAX_TRIM;
            if (htrim < -SOFT_MAX_TRIM) htrim = -SOFT_MAX_TRIM;

            int16_t m1 = (int16_t)(base1 + htrim + (base1 >= 0 ? m1_rate_trim : -m1_rate_trim));
            int16_t m2 = (int16_t)(base2 - htrim + (base2 >= 0 ? m2_rate_trim : -m2_rate_trim));
            if (base1 >= 0) {
                if (m1 < pwm_floor) m1 = pwm_floor;
                if (m2 < pwm_floor) m2 = pwm_floor;
            }

            if ((soft_corrections % 20) == 1) {
                printf("  [drive] SOFT#%lu  err=%.1f  htrim=%d  m1=%d m2=%d\n",
                       (unsigned long)soft_corrections,
                       (double)heading_err, (int)htrim, (int)m1, (int)m2);
            }
            drive(m1, m2);
            sleep_ms(5);

        } else {
            // ---- Zone 1: on heading -- rate matching ----
            if (now - window_t0 >= WHEEL_WINDOW_MS) {
                int32_t dc1 = get_count1() - c1_window;
                int32_t dc2 = get_count2() - c2_window;
                float wd1 = fabsf(wheel_distance_m1(dc1));
                float wd2 = fabsf(wheel_distance_m2(dc2));

                if (wd1 > 0.001f && wd2 > 0.001f) {
                    float ratio = wd1 / wd2;
                    if (ratio > 1.0f + WHEEL_RATE_TOL) {
                        if (m1_rate_trim > -(M1_BASE_PWM - pwm_floor)) m1_rate_trim -= WHEEL_TRIM_STEP;
                    } else if (ratio < 1.0f - WHEEL_RATE_TOL) {
                        if (m2_rate_trim > -(M2_BASE_PWM - pwm_floor)) m2_rate_trim -= WHEEL_TRIM_STEP;
                    } else {
                        if (m1_rate_trim < 0) m1_rate_trim += WHEEL_TRIM_STEP / 3;
                        if (m2_rate_trim < 0) m2_rate_trim += WHEEL_TRIM_STEP / 3;
                        if (m1_rate_trim > 0) m1_rate_trim = 0;
                        if (m2_rate_trim > 0) m2_rate_trim = 0;
                    }
                }
                c1_window = get_count1();
                c2_window = get_count2();
                window_t0 = now;
            }

            int16_t m1 = base1 + (base1 >= 0 ? m1_rate_trim : -m1_rate_trim);
            int16_t m2 = base2 + (base2 >= 0 ? m2_rate_trim : -m2_rate_trim);
            drive(m1, m2);
            sleep_ms(5);
        }
    }

    s_in_rotation = false;  // re-enable mag pull between waypoints

    int32_t total_dc1 = get_count1() - c1_start;
    int32_t total_dc2 = get_count2() - c2_start;
    printf("  [drive_distance] done  hard=%lu soft=%lu  enc1=%ld enc2=%ld  "
           "dist1=%.2fin dist2=%.2fin\n",
           (unsigned long)hard_corrections, (unsigned long)soft_corrections,
           (long)total_dc1, (long)total_dc2,
           (double)(wheel_distance_m1(total_dc1) * IN_PER_M),
           (double)(wheel_distance_m2(total_dc2) * IN_PER_M));
    stop();
}

void drive_forward_until_lost() {
    const int16_t BASE_PWM = 500;
    const uint32_t CAM_INTERVAL_MS = 100;
    const int     LOST_LIMIT = 2;
    const float   POST_LOST_IN = 8.0f;
    // Blind-roll heading control: rate-match wheels, pivot back if drift > tol.
    const uint32_t WHEEL_WINDOW_MS    = 100;
    const float    WHEEL_RATE_TOL     = 0.05f;
    const int16_t  WHEEL_TRIM_STEP    = 15;
    const float    WHEEL_MIN_PWM_FRAC = 0.80f;
    const float    HEADING_TOL_DEG    = 5.0f;

    int lost_count = 0;
    uint32_t last_cam = 0;
    uint8_t  dir = CAM_NONE;

    printf("  [collect] phase A (visual approach)\n");
    while (true) {
        mpu_update();
        position_update();

        uint32_t now = to_ms_since_boot(get_absolute_time());
        if (now - last_cam > CAM_INTERVAL_MS) {
            dir = get_camera_direction();
            last_cam = now;
            if (dir == CAM_NONE) lost_count++;
            else                 lost_count = 0;
        }
        if (lost_count >= LOST_LIMIT) break;

        int16_t m1 = BASE_PWM, m2 = BASE_PWM;
        if      (dir == CAM_LEFT)  { m1 = (int16_t)(BASE_PWM * 0.7f); m2 = BASE_PWM; }
        else if (dir == CAM_RIGHT) { m1 = BASE_PWM; m2 = (int16_t)(BASE_PWM * 0.7f); }

        drive(m1, m2);
        sleep_ms(5);
    }

    printf("  [collect] phase B (blind roll %.1f in)\n", (double)POST_LOST_IN);

    mpu_update();
    float ref_heading = fused_heading;
    float target_m    = POST_LOST_IN / IN_PER_M;
    int32_t c1_blind  = get_count1();
    int32_t c2_blind  = get_count2();
    uint32_t t_blind  = to_ms_since_boot(get_absolute_time());
    const uint32_t BLIND_TIMEOUT_MS = 2000;

    // Sonar safety during blind roll. LOW threshold (15cm) so we don't
    // false-trigger on the target ball just ahead of the intake.
    const uint32_t SONAR_INTERVAL_MS = 200;
    const float    SONAR_STOP_CM     = 25.40f;  // 10 inches
    uint32_t       last_sonar        = 0;

    // Wheel-rate match state for blind roll
    int16_t  pwm_floor   = (int16_t)((float)BASE_PWM * WHEEL_MIN_PWM_FRAC);
    int16_t  m1_trim     = 0;
    int16_t  m2_trim     = 0;
    int32_t  c1_window   = get_count1();
    int32_t  c2_window   = get_count2();
    uint32_t window_t0   = to_ms_since_boot(get_absolute_time());

    while (true) {
        mpu_update();
        position_update();

        int32_t c1 = get_count1() - c1_blind;
        int32_t c2 = get_count2() - c2_blind;
        float d1 = wheel_distance_m1(c1);
        float d2 = wheel_distance_m2(c2);
        float avg_abs = (fabsf(d1) + fabsf(d2)) * 0.5f;
        if (avg_abs >= target_m) break;
        if (to_ms_since_boot(get_absolute_time()) - t_blind > BLIND_TIMEOUT_MS) break;

        uint32_t now = to_ms_since_boot(get_absolute_time());
        if (now - last_sonar > SONAR_INTERVAL_MS) {
            last_sonar = now;
            float dist_cm = read_nearest_cm();
            if (dist_cm > 0.0f && dist_cm <= SONAR_STOP_CM) {
                printf("  [sonar] OBSTACLE at %.1f cm during blind roll! aborting collection\n",
                       (double)dist_cm);
                stop();
                // No back-up here; let chase logic move on.
                return;
            }
        }

        // Layer 2: heading-tolerance backup pivot
        float heading_err = wrap180(ref_heading - fused_heading);
        if (fabsf(heading_err) > HEADING_TOL_DEG) {
            int32_t seg_dc1 = get_count1() - c1_blind;
            int32_t seg_dc2 = get_count2() - c2_blind;
            report_drift_correction(seg_dc1, seg_dc2, -heading_err);

            printf("  [blind-roll] heading drift %.1f deg > %.1f tol -- pivot back\n",
                   (double)heading_err, (double)HEADING_TOL_DEG);
            stop();
            sleep_ms(50);
            rotate_to(ref_heading);
            sleep_ms(50);
            c1_window = get_count1();
            c2_window = get_count2();
            window_t0 = to_ms_since_boot(get_absolute_time());
            continue;
        }

        // Layer 1: continuous wheel-rate matching
        if (now - window_t0 >= WHEEL_WINDOW_MS) {
            int32_t dc1 = get_count1() - c1_window;
            int32_t dc2 = get_count2() - c2_window;
            float wd1 = fabsf(wheel_distance_m1(dc1));
            float wd2 = fabsf(wheel_distance_m2(dc2));
            if (wd1 > 0.001f && wd2 > 0.001f) {
                float ratio = wd1 / wd2;
                if (ratio > 1.0f + WHEEL_RATE_TOL) {
                    if (m1_trim > -(BASE_PWM - pwm_floor)) m1_trim -= WHEEL_TRIM_STEP;
                } else if (ratio < 1.0f - WHEEL_RATE_TOL) {
                    if (m2_trim > -(BASE_PWM - pwm_floor)) m2_trim -= WHEEL_TRIM_STEP;
                } else {
                    if (m1_trim < 0) m1_trim += WHEEL_TRIM_STEP / 3;
                    if (m2_trim < 0) m2_trim += WHEEL_TRIM_STEP / 3;
                    if (m1_trim > 0) m1_trim = 0;
                    if (m2_trim > 0) m2_trim = 0;
                }
            }
            c1_window = get_count1();
            c2_window = get_count2();
            window_t0 = now;
        }

        drive(BASE_PWM + m1_trim, BASE_PWM + m2_trim);
        sleep_ms(5);
    }

    printf("  [collect] done\n");
    stop();
}

// ====================================================================
// WAYPOINT NAVIGATION
// ====================================================================
void go_to_xy(float x_in, float y_in) {
    mpu_update();
    position_update();

    float dx = x_in - get_pos_x_in();
    float dy = y_in - get_pos_y_in();
    float dist = sqrtf(dx*dx + dy*dy);

    if (dist < 1.0f) {
        printf("[go_to_xy] already at (%.1f, %.1f), dist=%.2f in\n",
               (double)x_in, (double)y_in, (double)dist);
        return;
    }

    float bearing_deg = atan2f(dy, dx) * 180.0f / (float)M_PI;

    printf("[go_to_xy] from (%.1f, %.1f) to (%.1f, %.1f)  dist=%.1f  bearing=%.1f\n",
           (double)get_pos_x_in(), (double)get_pos_y_in(),
           (double)x_in, (double)y_in,
           (double)dist, (double)bearing_deg);

    printf("[go_to_xy_scanning] calling rotate_to(%.1f)\n", (double)bearing_deg);
    rotate_to(bearing_deg);
    printf("[go_to_xy_scanning] rotate_to returned, beginning forward drive\n");
    drive_distance(dist);
}

// Drive to within tol_in inches of (x_in, y_in), rotating slowly first.
// Use this for coverage waypoints where precision matters less than smoothness.
void go_to_xy_approx(float x_in, float y_in, float tol_in) {
    mpu_update();
    position_update();

    float dx   = x_in - get_pos_x_in();
    float dy   = y_in - get_pos_y_in();
    float dist = sqrtf(dx*dx + dy*dy);

    float drive_dist = dist - tol_in;
    if (drive_dist < 1.0f) {
        printf("[go_to_xy_approx] already within tolerance of (%.1f, %.1f)\n",
               (double)x_in, (double)y_in);
        return;
    }

    float bearing_deg = atan2f(dy, dx) * 180.0f / (float)M_PI;

    printf("[go_to_xy_approx] from (%.1f, %.1f) to (%.1f, %.1f)  dist=%.1f  tol=%.1f  bearing=%.1f\n",
           (double)get_pos_x_in(), (double)get_pos_y_in(),
           (double)x_in, (double)y_in,
           (double)dist, (double)tol_in, (double)bearing_deg);

    // Use fast rotate_to for large turns -- rotate_to_slow times out on angles >~60°.
    float angle_diff = fabsf(wrap180(bearing_deg - fused_heading));
    if (angle_diff > 45.0f) rotate_to(bearing_deg);
    else                     rotate_to_slow(bearing_deg);
    drive_distance(drive_dist);
}

void go_to_xy_scanning(float x_in, float y_in, bool* ball_seen) {
    if (ball_seen) *ball_seen = false;

    mpu_update();
    position_update();

    float dx = x_in - get_pos_x_in();
    float dy = y_in - get_pos_y_in();
    float dist_total = sqrtf(dx*dx + dy*dy);

    if (dist_total < 1.0f) return;

    float bearing_deg = atan2f(dy, dx) * 180.0f / (float)M_PI;

    printf("[go_to_xy_scanning] from (%.1f, %.1f) to (%.1f, %.1f)  dist=%.1f  bearing=%.1f\n",
           (double)get_pos_x_in(), (double)get_pos_y_in(),
           (double)x_in, (double)y_in,
           (double)dist_total, (double)bearing_deg);

    rotate_to(bearing_deg);

    const int16_t  BASE_PWM        = 500;
    const uint32_t CAM_INTERVAL_MS = 100;
    // 250 ms per inch (covers slow surfaces and gentle ramp-up) plus
    // a fixed 3 sec floor for short drives. A 60-inch drive now has
    // 18 sec of budget instead of 8 — way more headroom for actual
    // mechanical reality.
    const uint32_t TIMEOUT_MS      = (uint32_t)(dist_total * 250.0f) + 3000;
    const float    DIST_TOL_M      = 0.02f;
    // Wheel-rate match + heading-tolerance backup
    const uint32_t WHEEL_WINDOW_MS    = 100;
    const float    WHEEL_RATE_TOL     = 0.05f;
    const int16_t  WHEEL_TRIM_STEP    = 15;
    const float    WHEEL_MIN_PWM_FRAC = 0.80f;
    const float    HEADING_TOL_DEG    = 5.0f;

    float target_m = dist_total / IN_PER_M;

    mpu_update();
    // Use the ORIGINAL target bearing as the reference, not fused_heading
    // after rotation. rotate_to is allowed to exit up to ANGLE_TOL_DEG off
    // target, so capturing fused_heading would lock in that residual error
    // for the whole drive. Driving toward bearing_deg directly means the
    // heading corrector immediately steers toward the true target.
    float ref_heading = bearing_deg;

    // Sonar safety: stop when something within 30cm, poll for clearance
    // up to 10s, back up 4in if still blocked, and exit so the caller
    // retries the waypoint from the new (slightly-backed-up) position.
    const uint32_t SONAR_INTERVAL_MS = 200;
    const float    SONAR_STOP_CM     = 25.40f;  // 10 inches
    const float    SONAR_BACKUP_IN   = -4.0f;
    const uint32_t SONAR_WAIT_MS     = 10000;
    uint32_t       last_sonar        = 0;

    int32_t c1_start = get_count1();
    int32_t c2_start = get_count2();
    uint32_t t0 = to_ms_since_boot(get_absolute_time());
    uint32_t last_cam = 0;
    uint32_t last_dbg = 0;

    // Rate-match state
    int16_t  pwm_floor = (int16_t)((float)BASE_PWM * WHEEL_MIN_PWM_FRAC);
    int16_t  m1_trim   = 0;
    int16_t  m2_trim   = 0;
    int32_t  c1_window = get_count1();
    int32_t  c2_window = get_count2();
    uint32_t window_t0 = to_ms_since_boot(get_absolute_time());

    // Ball-confirmation while driving. We do NOT stop on the first
    // detection (that caused the robot to wander every time vision
    // produced a phantom). Instead we require N consecutive non-NONE
    // camera frames before declaring ball_seen and aborting the drive.
    // Phantoms are by definition non-persistent — a single detection
    // followed by silence won't trip this. Real balls show up across
    // many frames in a row.
    const int BALL_CONFIRM_REQUIRED = 3;
    int       ball_streak = 0;

    while (true) {
        mpu_update();
        position_update();

        int32_t c1 = get_count1() - c1_start;
        int32_t c2 = get_count2() - c2_start;
        float d1 = wheel_distance_m1(c1);
        float d2 = wheel_distance_m2(c2);
        float avg_abs = (fabsf(d1) + fabsf(d2)) * 0.5f;
        if (avg_abs >= target_m - DIST_TOL_M) break;
        if (to_ms_since_boot(get_absolute_time()) - t0 > TIMEOUT_MS) break;

        uint32_t now = to_ms_since_boot(get_absolute_time());

        // --- Sonar safety ---
        if (now - last_sonar > SONAR_INTERVAL_MS) {
            last_sonar = now;
            float dist_cm = read_nearest_cm();
            if (dist_cm > 0.0f && dist_cm <= SONAR_STOP_CM) {
                printf("[sonar] OBSTACLE at %.1f cm! stop, poll up to %lums for clearance\n",
                       (double)dist_cm, (unsigned long)SONAR_WAIT_MS);
                stop();

                bool cleared = false;
                uint32_t wait_t0 = to_ms_since_boot(get_absolute_time());
                while (to_ms_since_boot(get_absolute_time()) - wait_t0
                       < SONAR_WAIT_MS) {
                    // Keep mpu_update() running during the sonar wait so
                    // gyro integration and mag print don't freeze.
                    for (int i = 0; i < 20; i++) {
                        mpu_update();
                        position_update();
                        sleep_ms(50);
                    }
                    float check_cm = read_nearest_cm();
                    if (check_cm < 0.0f) {
                        printf("[sonar] poll: no echo (clear)\n");
                        cleared = true;
                        break;
                    } else if (check_cm > SONAR_STOP_CM) {
                        printf("[sonar] poll: clear (%.1f cm) - resuming\n",
                               (double)check_cm);
                        cleared = true;
                        break;
                    } else {
                        printf("[sonar] poll: still blocked (%.1f cm)\n",
                               (double)check_cm);
                    }
                }

                if (cleared) {
                    mpu_update();
                    position_update();
                    continue;
                }

                printf("[sonar] still blocked after %lums - skipping waypoint\n",
                       (unsigned long)SONAR_WAIT_MS);
                g_sonar_skip_next = true;
                return;
            }
        }

        if (now - last_cam > CAM_INTERVAL_MS) {
            last_cam = now;
            uint8_t dir = get_camera_direction();
            if (dir != CAM_NONE) {
                ball_streak++;
                printf("  [scan-confirm] hit %d/%d (dir=%u)\n",
                       ball_streak, BALL_CONFIRM_REQUIRED, dir);
                if (ball_streak >= BALL_CONFIRM_REQUIRED) {
                    if (ball_seen) *ball_seen = true;
                    printf("[go_to_xy_scanning] ball CONFIRMED mid-drive, "
                           "streak=%d, dir=%u\n", ball_streak, dir);
                    stop();
                    return;
                }
                // Keep driving while we accumulate confirmation. Robot
                // stays on planned path until streak completes.
            } else {
                if (ball_streak > 0) {
                    printf("  [scan-confirm] streak broken at %d (phantom)\n",
                           ball_streak);
                }
                ball_streak = 0;
            }
        }

        // Layer 2: heading-tolerance backup pivot
        float heading_err = wrap180(ref_heading - fused_heading);
        if (fabsf(heading_err) > HEADING_TOL_DEG) {
            int32_t seg_dc1 = get_count1() - c1_start;
            int32_t seg_dc2 = get_count2() - c2_start;
            report_drift_correction(seg_dc1, seg_dc2, -heading_err);

            printf("  [go_to_xy] heading drift %.1f deg > %.1f tol -- pivot back\n",
                   (double)heading_err, (double)HEADING_TOL_DEG);
            stop();
            sleep_ms(50);
            rotate_to(ref_heading);
            sleep_ms(50);
            c1_window = get_count1();
            c2_window = get_count2();
            window_t0 = to_ms_since_boot(get_absolute_time());
            continue;
        }

        // Layer 1: continuous wheel-rate matching
        if (now - window_t0 >= WHEEL_WINDOW_MS) {
            int32_t dc1 = get_count1() - c1_window;
            int32_t dc2 = get_count2() - c2_window;
            float wd1 = fabsf(wheel_distance_m1(dc1));
            float wd2 = fabsf(wheel_distance_m2(dc2));
            if (wd1 > 0.001f && wd2 > 0.001f) {
                float ratio = wd1 / wd2;
                if (ratio > 1.0f + WHEEL_RATE_TOL) {
                    if (m1_trim > -(BASE_PWM - pwm_floor)) m1_trim -= WHEEL_TRIM_STEP;
                } else if (ratio < 1.0f - WHEEL_RATE_TOL) {
                    if (m2_trim > -(BASE_PWM - pwm_floor)) m2_trim -= WHEEL_TRIM_STEP;
                } else {
                    if (m1_trim < 0) m1_trim += WHEEL_TRIM_STEP / 3;
                    if (m2_trim < 0) m2_trim += WHEEL_TRIM_STEP / 3;
                    if (m1_trim > 0) m1_trim = 0;
                    if (m2_trim > 0) m2_trim = 0;
                }
            }
            c1_window = get_count1();
            c2_window = get_count2();
            window_t0 = now;
        }

        int16_t m1 = BASE_PWM + m1_trim;
        int16_t m2 = BASE_PWM + m2_trim;
        drive(m1, m2);

        if (now - last_dbg > 1000) {
            last_dbg = now;
            printf("  [drive] head=%.1f ref=%.1f err=%+.1f m1_trim=%+d m2_trim=%+d  c1=%ld c2=%ld  d1=%.3f d2=%.3f\n",
                   (double)fused_heading, (double)ref_heading,
                   (double)heading_err, (int)m1_trim, (int)m2_trim,
                   (long)c1, (long)c2, (double)d1, (double)d2);
        }

        sleep_ms(5);
    }
    stop();
}

// ====================================================================
// INIT
// ====================================================================
void init_encoders() {
    init_encoder_pin(ENC1_A);
    init_encoder_pin(ENC1_B);
    init_encoder_pin(ENC2_A);
    init_encoder_pin(ENC2_B);
    last_a1 = gpio_get(ENC1_A);
    last_a2 = gpio_get(ENC2_A);

    gpio_set_irq_enabled_with_callback(
        ENC1_A, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true,
        &gpio_irq_callback);
    gpio_set_irq_enabled(
        ENC2_A, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
}

// Diagnostic: motors stay off, prints c1/c2 every 200ms for 20 seconds
// so the user can manually spin each wheel and see which encoder is
// attached to which physical wheel. Print is suppressed when neither
// count has changed to keep the log clean.
// Streams live encoder deltas for both wheels so you can measure counts
// per single rotation. Mark the wheel, spin it exactly one full turn by
// hand, and read the printed delta. Update PPR in encoders.cpp to match.
// Motors must be OFF. Runs for 60 seconds then prints a summary.
// Uncomment the call in main.cpp to run, re-comment when done.
void encoder_rotation_test() {
    const uint32_t DURATION_MS = 60000;
    const uint32_t PRINT_MS    = 150;

    printf("\n==========================================================\n");
    printf("  ENCODER ROTATION TEST  (PPR1=%d  PPR2=%d)\n", PPR1, PPR2);
    printf("  Motors OFF. Mark each wheel, spin exactly ONE full turn,\n");
    printf("  read the delta. That number is your true PPR.\n");
    printf("  Running for %lu seconds...\n", (unsigned long)(DURATION_MS / 1000));
    printf("  Spin enc2 first, then enc1 -- each one full turn.\n");
    printf("==========================================================\n\n");

    int32_t base1 = get_count1();
    int32_t base2 = get_count2();
    int32_t last1 = base1, last2 = base2;
    uint32_t t0 = to_ms_since_boot(get_absolute_time());

    while (to_ms_since_boot(get_absolute_time()) - t0 < DURATION_MS) {
        sleep_ms(PRINT_MS);
        int32_t c1 = get_count1();
        int32_t c2 = get_count2();
        if (c1 != last1 || c2 != last2) {
            printf("  enc1=%+ld   enc2=%+ld\n",
                   (long)(c1 - base1), (long)(c2 - base2));
            last1 = c1;
            last2 = c2;
        }
    }

    printf("\n  FINAL after 30s:  enc1=%+ld   enc2=%+ld\n",
           (long)(get_count1() - base1), (long)(get_count2() - base2));
    printf("  Update PPR in encoders.cpp line ~42 to match one full rotation.\n");
    printf("==========================================================\n\n");
}

// Drive forward for 3 seconds at M1_BASE_PWM / M2_BASE_PWM with ZERO correction.
// Watch whether the robot curves. If it curves left, lower M2_BASE_PWM (or raise
// M1_BASE_PWM). If it curves right, raise M2_BASE_PWM (or lower M1_BASE_PWM).
// Tune until it goes straight, then re-comment. Call after init_mpu().
// 10-second straight drive with hard + soft heading correction.
// Hard pivot (stop + rotate_to) fires when |err| > HARD_DEG.
// Soft PWM trim handles anything within that band, targeting +-2 deg.
// Kick the robot sideways mid-run to test both paths.
void drive_raw_test() {
    const uint32_t RUN_MS    = 10000;
    const float    HARD_DEG  = 5.0f;
    const float    SOFT_GAIN = 12.0f;
    const int16_t  SOFT_MAX  = 80;

    printf("\n[drive_correction_test] M1=%d M2=%d -- 10s, hard>%.0fdeg / soft trim\n",
           (int)M1_BASE_PWM, (int)M2_BASE_PWM, (double)HARD_DEG);
    printf("  Kick robot to test both correction paths. Starting in 2s...\n");
    sleep_ms(2000);

    s_in_rotation = true;

    apply_motor1_pwm(KICK_PWM);
    apply_motor2_pwm(KICK_PWM);
    sleep_ms(KICK_DURATION_MS);

    mpu_update();
    float    ref      = fused_heading;
    int32_t  c1_start = get_count1();
    int32_t  c2_start = get_count2();
    uint32_t t0       = to_ms_since_boot(get_absolute_time());
    uint32_t last_rep = t0;

    while (to_ms_since_boot(get_absolute_time()) - t0 < RUN_MS) {
        mpu_update();

        float err = wrap180(ref - fused_heading);

        if (fabsf(err) > HARD_DEG) {
            apply_motor1_pwm(0);
            apply_motor2_pwm(0);
            sleep_ms(50);
            printf("  [HARD] err=%+.1f -> pivot to ref=%.1f\n",
                   (double)err, (double)ref);
            rotate_to(ref);          // clears s_in_rotation on exit
            s_in_rotation = true;    // re-suspend mag for remainder of drive
            sleep_ms(50);
            apply_motor1_pwm(KICK_PWM);
            apply_motor2_pwm(KICK_PWM);
            sleep_ms(KICK_DURATION_MS);
            mpu_update();
            last_rep = to_ms_since_boot(get_absolute_time());
            continue;
        }

        int16_t htrim = (int16_t)(err * SOFT_GAIN);
        if (htrim >  SOFT_MAX) htrim =  SOFT_MAX;
        if (htrim < -SOFT_MAX) htrim = -SOFT_MAX;

        int16_t m1 = M1_BASE_PWM + htrim;
        int16_t m2 = M2_BASE_PWM - htrim;
        if (m1 > (int16_t)PWM_WRAP) m1 = (int16_t)PWM_WRAP;
        if (m2 > (int16_t)PWM_WRAP) m2 = (int16_t)PWM_WRAP;
        if (m1 < 0) m1 = 0;
        if (m2 < 0) m2 = 0;
        apply_motor1_pwm(m1);
        apply_motor2_pwm(m2);

        uint32_t now = to_ms_since_boot(get_absolute_time());
        if (now - last_rep >= 500) {
            last_rep = now;
            printf("  t=%lus  err=%+.1f  htrim=%+d  enc1=%ld enc2=%ld\n",
                   (unsigned long)((now - t0) / 1000),
                   (double)err, (int)htrim,
                   (long)(get_count1() - c1_start),
                   (long)(get_count2() - c2_start));
        }
        sleep_ms(5);
    }

    apply_motor1_pwm(0);
    apply_motor2_pwm(0);
    s_last_m1 = 0;
    s_last_m2 = 0;
    s_in_rotation = false;
    printf("  [drive_correction_test] done\n");
}

// Drive forward a fixed distance and report what each encoder measured.
// Use a tape measure to check actual distance traveled vs commanded.
// Call after init_encoders() + init_motors() + init_mpu().
// Uncomment in main.cpp to run, re-comment when done.
void drive_forward_test() {
    const float    DIST_IN   = 24.0f;
    const float    circ_in   = 2.0f * (float)M_PI * (WHEEL_RADIUS * IN_PER_M);
    const int      exp1      = (int)(DIST_IN / circ_in * PPR1);
    const int      exp2      = (int)(DIST_IN / circ_in * PPR2);

    printf("\n==========================================================\n");
    printf("  DRIVE FORWARD TEST  --  commanding %.0f inches\n", (double)DIST_IN);
    printf("  Wheel circumference: %.2f in\n", (double)circ_in);
    printf("  Expected enc1=%d  enc2=%d\n", exp1, exp2);
    printf("  Place robot on the ground, mark start position.\n");
    printf("==========================================================\n\n");

    sleep_ms(2000);   // time to set robot down

    int32_t c1_before = get_count1();
    int32_t c2_before = get_count2();

    drive_distance(DIST_IN);

    int32_t dc1 = get_count1() - c1_before;
    int32_t dc2 = get_count2() - c2_before;

    float actual1_in = wheel_distance_m1(dc1) * IN_PER_M;
    float actual2_in = wheel_distance_m2(dc2) * IN_PER_M;

    printf("\n==========================================================\n");
    printf("  RESULTS:\n");
    printf("  enc1: %ld counts  (expected %d)  -> calculated %.2f in\n",
           (long)dc1, exp1, (double)actual1_in);
    printf("  enc2: %ld counts  (expected %d)  -> calculated %.2f in\n",
           (long)dc2, exp2, (double)actual2_in);
    printf("\n  Measure actual distance with a tape measure.\n");
    printf("  If actual != %.0f in, adjust WHEEL_RADIUS in encoders.cpp.\n", (double)DIST_IN);
    printf("  If enc1 != enc2 counts, robot is drifting -- check motors.\n");
    printf("==========================================================\n\n");
}

// Three-phase hand-spin test for encoder 1 only.
// Phase 1: keep wheel still -- checks for noise/spurious counts.
// Phase 2: spin wheel FORWARD by hand -- checks positive direction.
// Phase 3: spin wheel BACKWARD by hand -- checks negative direction.
// Motors must be OFF (call before init_motors, or just don't call drive()).
// Uncomment the call in main.cpp to run, re-comment when done.
void encoder1_test() {
    const uint32_t NOISE_MS    = 3000;
    const uint32_t SPIN_MS     = 8000;
    const uint32_t PRINT_MS    = 200;
    const int32_t  NOISE_LIMIT = 5;    // counts allowed to drift when still
    const int32_t  MIN_COUNTS  = 30;   // minimum counts expected per spin phase

    printf("\n==========================================================\n");
    printf("  ENCODER 1 TEST  (ENC1_A=GPIO%u  ENC1_B=GPIO%u)\n", ENC1_A, ENC1_B);
    printf("  PPR1=%d PPR2=%d  --  motors must be OFF\n", PPR1, PPR2);
    printf("==========================================================\n\n");

    // ---- Phase 1: noise ----
    printf("  [Phase 1 / 3s]  Keep wheel 1 COMPLETELY STILL...\n");
    int32_t base = get_count1();
    int32_t last  = base;
    int32_t noise_peak = 0;
    uint32_t t0 = to_ms_since_boot(get_absolute_time());
    while (to_ms_since_boot(get_absolute_time()) - t0 < NOISE_MS) {
        sleep_ms(PRINT_MS);
        int32_t d = get_count1() - base;
        if (d != (last - base)) {
            printf("    spurious count! delta=%ld\n", (long)d);
            last = get_count1();
        }
        int32_t abs_d = d < 0 ? -d : d;
        if (abs_d > noise_peak) noise_peak = abs_d;
    }
    bool noise_ok = noise_peak <= NOISE_LIMIT;
    printf("  Phase 1 result: peak drift=%ld counts  ->  %s\n\n",
           (long)noise_peak, noise_ok ? "PASS" : "FAIL (encoder counting on its own -- check wiring/noise)");

    // ---- Phase 2: forward ----
    printf("  [Phase 2 / 8s]  Spin wheel 1 FORWARD by hand...\n");
    base = get_count1();
    last = base;
    int32_t fwd_peak = 0;
    t0 = to_ms_since_boot(get_absolute_time());
    while (to_ms_since_boot(get_absolute_time()) - t0 < SPIN_MS) {
        sleep_ms(PRINT_MS);
        int32_t c = get_count1();
        if (c != last) {
            int32_t d = c - base;
            printf("    delta=%+ld\n", (long)d);
            last = c;
            if (d > fwd_peak) fwd_peak = d;
        }
    }
    bool fwd_ok = fwd_peak >= MIN_COUNTS;
    printf("  Phase 2 result: max forward delta=%ld  ->  %s\n\n",
           (long)fwd_peak,
           fwd_ok ? "PASS" : "FAIL (no forward counts -- dead signal or disconnected)");

    // ---- Phase 3: backward ----
    printf("  [Phase 3 / 8s]  Spin wheel 1 BACKWARD by hand...\n");
    base = get_count1();
    last = base;
    int32_t bwd_peak = 0;
    t0 = to_ms_since_boot(get_absolute_time());
    while (to_ms_since_boot(get_absolute_time()) - t0 < SPIN_MS) {
        sleep_ms(PRINT_MS);
        int32_t c = get_count1();
        if (c != last) {
            int32_t d = c - base;
            printf("    delta=%+ld\n", (long)d);
            last = c;
            if (d < bwd_peak) bwd_peak = d;
        }
    }
    bool bwd_ok = bwd_peak <= -MIN_COUNTS;
    printf("  Phase 3 result: max backward delta=%ld  ->  %s\n\n",
           (long)bwd_peak,
           bwd_ok ? "PASS" : "FAIL (no backward counts -- check A/B wiring or direction)");

    printf("==========================================================\n");
    printf("  ENCODER 1 SUMMARY:  noise=%s  forward=%s  backward=%s\n",
           noise_ok ? "OK" : "BAD",
           fwd_ok   ? "OK" : "BAD",
           bwd_ok   ? "OK" : "BAD");
    printf("==========================================================\n\n");
}

void encoder_id_test() {
    printf("\n");
    printf("==========================================================\n");
    printf("  ENCODER ID TEST: motors are OFF, spin wheels by hand.\n");
    printf("  Spin LEFT wheel first, then RIGHT wheel. 20 seconds.\n");
    printf("==========================================================\n");

    int32_t base_c1 = get_count1();
    int32_t base_c2 = get_count2();
    int32_t last_c1 = base_c1;
    int32_t last_c2 = base_c2;

    const uint32_t DURATION_MS = 20000;
    uint32_t t0 = to_ms_since_boot(get_absolute_time());

    while (to_ms_since_boot(get_absolute_time()) - t0 < DURATION_MS) {
        int32_t c1 = get_count1();
        int32_t c2 = get_count2();
        if (c1 != last_c1 || c2 != last_c2) {
            int32_t d1 = c1 - base_c1;
            int32_t d2 = c2 - base_c2;
            printf("  c1=%+ld  c2=%+ld\n", (long)d1, (long)d2);
            last_c1 = c1;
            last_c2 = c2;
        }
        sleep_ms(200);
    }

    int32_t final_c1 = get_count1() - base_c1;
    int32_t final_c2 = get_count2() - base_c2;
    printf("==========================================================\n");
    printf("  FINAL: c1=%+ld  c2=%+ld\n", (long)final_c1, (long)final_c2);
    printf("==========================================================\n\n");
}

// ====================================================================
// MAG NOISE TEST (Stage 2)
// ====================================================================
// Spins both motors forward at 50% PWM for 10 seconds while the user
// holds the robot OFF the ground (wheels spin freely, no load on the
// chassis). Mag readings are printed at 5 Hz. If the chip is mounted
// far enough from the motors and well shielded, readings should stay
// within a tight ~5-10 deg band of stationary. If readings jump or
// drift heavily while the motors run, the chip is too close to the
// motors or wires are coupling magnetic noise.
//
// What to look for:
//   STATIONARY phase (motors off, 3 sec):
//     cal_h should be steady within ~3 deg (your normal noise floor)
//   MOTORS RUNNING phase (motors at 500 PWM, 10 sec):
//     If cal_h still in ~3 deg band -> mag is clean under motor load
//     If cal_h jumps by 10-30 deg -> some motor coupling, probably
//       still usable for fusion with conservative pull rate
//     If cal_h jumps by >30 deg or wanders unboundedly -> chip needs
//       to be relocated before mag fusion is viable
//
// IMPORTANT: hold the robot OFF the ground throughout. If wheels touch
// down the robot will lurch and that's not what we're testing.
#if MAG_ENABLED
void mag_noise_test() {
    printf("\n");
    printf("==========================================================\n");
    printf("  MAG NOISE TEST (Stage 2)\n");
    printf("  HOLD ROBOT OFF GROUND -- wheels will spin freely.\n");
    printf("  Phase 1: STATIONARY 3s (baseline)\n");
    printf("  Phase 2: MOTORS ON 10s (noise check)\n");
    printf("  Phase 3: STOP 2s (recovery)\n");
    printf("==========================================================\n");

    auto print_mag = [](const char* tag) {
        float mx, my, mz;
        if (ak_read_xyz(&mx, &my, &mz)) {
            float mxc = mx - MAG_OFFSET_X;
            float myc = my - MAG_OFFSET_Y;
            float h   = atan2f(-myc, -mxc) * 180.0f / (float)M_PI;
            printf("  [%s] mx=%+7.1f my=%+7.1f mz=%+7.1f  cal_h=%+6.1f\n",
                   tag, (double)mx, (double)my, (double)mz, (double)h);
        } else {
            printf("  [%s] no data\n", tag);
        }
    };

    // Phase 1: stationary
    printf("-- PHASE 1: stationary baseline --\n");
    uint32_t t0 = to_ms_since_boot(get_absolute_time());
    while (to_ms_since_boot(get_absolute_time()) - t0 < 3000) {
        print_mag("stat");
        sleep_ms(200);
    }

    // Phase 2: motors on
    printf("-- PHASE 2: motors ON at 500 PWM --\n");
    drive(500, 500);
    t0 = to_ms_since_boot(get_absolute_time());
    while (to_ms_since_boot(get_absolute_time()) - t0 < 10000) {
        print_mag("run ");
        sleep_ms(200);
    }

    // Phase 3: stop
    stop();
    printf("-- PHASE 3: motors STOPPED, recovery --\n");
    t0 = to_ms_since_boot(get_absolute_time());
    while (to_ms_since_boot(get_absolute_time()) - t0 < 2000) {
        print_mag("stop");
        sleep_ms(200);
    }

    printf("==========================================================\n");
    printf("  MAG NOISE TEST COMPLETE\n");
    printf("==========================================================\n\n");
}
#else
void mag_noise_test() {
    printf("[mag_noise_test] MAG_ENABLED is 0 -- test skipped\n");
}
#endif

// Distance calibration: drives forward at M1/M2_BASE_PWM for 5 seconds
// with soft heading correction (matching real match behavior), then pauses
// 60 seconds so the user can measure physical distance with a tape measure.
// Run 3 times and average the results for accurate WHEEL_RADIUS adjustment.
//
// Adjustment formula (printed at the end of each run):
//   new_radius = old_radius * (actual_inches / computed_inches)
void distance_calibration_test() {
    const uint32_t DRIVE_MS   = 5000;
    const uint32_t PAUSE_MS   = 60000;
    const float    SOFT_GAIN  = 12.0f;
    const int16_t  SOFT_MAX   = 80;

    printf("\n");
    printf("==========================================================\n");
    printf("  DISTANCE CALIBRATION: 5s at M1=%d M2=%d\n",
           (int)M1_BASE_PWM, (int)M2_BASE_PWM);
    printf("  Mark the start position NOW. Starting in 3s...\n");
    printf("==========================================================\n");
    sleep_ms(3000);

    mpu_update();
    float   ref      = fused_heading;
    int32_t base_c1  = get_count1();
    int32_t base_c2  = get_count2();

    s_in_rotation = true;
    apply_motor1_pwm(KICK_PWM);
    apply_motor2_pwm(KICK_PWM);
    sleep_ms(KICK_DURATION_MS);

    uint32_t t0 = to_ms_since_boot(get_absolute_time());
    while (to_ms_since_boot(get_absolute_time()) - t0 < DRIVE_MS) {
        mpu_update();

        float   err   = wrap180(ref - fused_heading);
        int16_t htrim = (int16_t)(err * SOFT_GAIN);
        if (htrim >  SOFT_MAX) htrim =  SOFT_MAX;
        if (htrim < -SOFT_MAX) htrim = -SOFT_MAX;

        int16_t m1 = M1_BASE_PWM + htrim;
        int16_t m2 = M2_BASE_PWM - htrim;
        if (m1 > (int16_t)PWM_WRAP) m1 = (int16_t)PWM_WRAP;
        if (m2 > (int16_t)PWM_WRAP) m2 = (int16_t)PWM_WRAP;
        if (m1 < 0) m1 = 0;
        if (m2 < 0) m2 = 0;
        apply_motor1_pwm(m1);
        apply_motor2_pwm(m2);
        sleep_ms(5);
    }
    stop();
    s_in_rotation = false;

    int32_t d_c1 = get_count1() - base_c1;
    int32_t d_c2 = get_count2() - base_c2;

    float dist_m1_in = wheel_distance_m1(d_c1) * IN_PER_M;
    float dist_m2_in = wheel_distance_m2(d_c2) * IN_PER_M;
    float avg_in     = (dist_m1_in + dist_m2_in) * 0.5f;

    printf("==========================================================\n");
    printf("  STOPPED. Results:\n");
    printf("    enc1 (left)  = %+ld ticks  -> %.2f in\n", (long)d_c1, (double)dist_m1_in);
    printf("    enc2 (right) = %+ld ticks  -> %.2f in\n", (long)d_c2, (double)dist_m2_in);
    printf("    computed avg = %.2f in\n", (double)avg_in);
    printf("\n");
    printf("  MEASURE actual distance now (tape from start mark to front of robot).\n");
    printf("  Then: new_radius = %.6f * (actual_in / %.2f)\n",
           (double)WHEEL_RADIUS, (double)avg_in);
    printf("\n");
    printf("  Pausing 60s to measure...\n");
    printf("==========================================================\n");

    sleep_ms(PAUSE_MS);

    printf("  [distance_cal] pause over -- reposition and run again\n\n");
}

void init_motors() {
    gpio_init(M1_FWD); gpio_set_dir(M1_FWD, GPIO_OUT);
    gpio_init(M1_BWD); gpio_set_dir(M1_BWD, GPIO_OUT);
    gpio_init(M2_FWD); gpio_set_dir(M2_FWD, GPIO_OUT);
    gpio_init(M2_BWD); gpio_set_dir(M2_BWD, GPIO_OUT);
    motor_pin_pwm_init(M1_FWD);
    motor_pin_pwm_init(M1_BWD);
    motor_pin_pwm_init(M2_FWD);
    motor_pin_pwm_init(M2_BWD);

    init_intake_pwm();

    stop();
    intake_set_percent(INTAKE_DUTY_PCT);
    printf("[intake] started at %d%% forward\n", INTAKE_DUTY_PCT);
}

// Measures the raw speed ratio between M1 and M2 at equal PWM so you can
// set M2_SPEED_SCALE accurately. Bypasses drive() to apply identical raw PWM
// to both motors. Robot must be on the ground and free to roll.
// Run once, read the suggested M2_SPEED_SCALE, update the constant, re-flash.
void motor_balance_cal() {
    const int16_t  CAL_PWM  = 500;
    const uint32_t RUN_MS   = 3000;
    const float    circ_in  = 2.0f * (float)M_PI * (WHEEL_RADIUS * IN_PER_M);

    printf("\n==========================================================\n");
    printf("  MOTOR BALANCE CALIBRATION\n");
    printf("  Both motors raw %d PWM for %lus -- no M2_SPEED_SCALE applied.\n",
           CAL_PWM, (unsigned long)(RUN_MS / 1000));
    printf("  Current M2_SPEED_SCALE = %.3f\n", (double)M2_SPEED_SCALE);
    printf("  Place robot on ground. Starting in 2s...\n");
    printf("==========================================================\n");
    sleep_ms(2000);

    // Kickstart to overcome stiction, then drop to CAL_PWM for measurement.
    apply_motor1_pwm(KICK_PWM);
    apply_motor2_pwm(KICK_PWM);
    sleep_ms(KICK_DURATION_MS);
    apply_motor1_pwm(CAL_PWM);
    apply_motor2_pwm(CAL_PWM);

    int32_t c1_start = get_count1();
    int32_t c2_start = get_count2();
    sleep_ms(RUN_MS);
    int32_t dc1 = get_count1() - c1_start;
    int32_t dc2 = get_count2() - c2_start;

    stop();

    float d1_in = wheel_distance_m1(dc1) * IN_PER_M;
    float d2_in = wheel_distance_m2(dc2) * IN_PER_M;

    printf("\n  enc1 = %ld  (%.2f in)\n", (long)dc1, (double)d1_in);
    printf("  enc2 = %ld  (%.2f in)\n", (long)dc2, (double)d2_in);

    if (dc1 > 100 && dc2 > 100) {
        // If M2 spins faster, ratio > 1 -- scale M2 down by that ratio.
        // If M1 spins faster, ratio < 1 -- M2_SPEED_SCALE > 1 to compensate.
        float ratio = (float)dc2 / (float)dc1;  // M2 speed relative to M1
        float suggested = 1.0f / ratio;
        printf("\n  M2/M1 raw speed ratio: %.4f\n", (double)ratio);
        printf("  Suggested M2_SPEED_SCALE = %.4f\n", (double)suggested);
        printf("  Update M2_SPEED_SCALE in encoders.cpp and re-flash.\n");
    } else {
        printf("\n  ERROR: one or both motors showed near-zero counts.\n");
        printf("  Check connections and re-run.\n");
    }
    printf("==========================================================\n\n");
}

// Spin each drive motor individually and sample the encoder every 100ms.
// Prints a live count table so you can see movement even if the robot is
// constrained. Robot should be lifted or on a frictionless surface.
// Call after init_encoders() + init_motors().
void motor_power_test() {
    const int16_t  TEST_PWM   = 500;
    const uint32_t SAMPLE_MS  = 100;
    const uint32_t SAMPLES    = 6;     // 600ms total per motor
    const int32_t  MIN_COUNTS = 10;

    printf("\n=== MOTOR POWER TEST (PWM=%d, %lu samples x %lums) ===\n",
           TEST_PWM, (unsigned long)SAMPLES, (unsigned long)SAMPLE_MS);
    printf("  Lift the robot so wheels can spin freely.\n\n");

    // Kick then run: mirrors what drive() does so the test reflects real behavior.
    #define TEST_MOTOR(label, fwd_pin, bwd_pin, apply_fn, get_count_fn)      \
    do {                                                                      \
        printf("  " label " (FWD=GPIO%u BWD=GPIO%u):\n", fwd_pin, bwd_pin);  \
        int32_t c_start = get_count_fn();                                    \
        apply_fn((int16_t)PWM_WRAP);   /* kickstart at full power */         \
        sleep_ms(KICK_DURATION_MS);                                          \
        apply_fn(TEST_PWM);            /* drop to run PWM */                 \
        int32_t c_max_abs = 0;                                               \
        for (uint32_t i = 1; i <= SAMPLES; i++) {                            \
            sleep_ms(SAMPLE_MS);                                             \
            int32_t d = get_count_fn() - c_start;                            \
            int32_t abs_d = d < 0 ? -d : d;                                  \
            if (abs_d > c_max_abs) c_max_abs = abs_d;                        \
            printf("    %3lums  enc delta=%ld\n",                             \
                   (unsigned long)(KICK_DURATION_MS + i * SAMPLE_MS), (long)d); \
        }                                                                    \
        apply_fn(0);                                                         \
        sleep_ms(200);                                                       \
        printf("  " label " -> peak delta=%ld  %s\n\n",                      \
               (long)c_max_abs,                                              \
               c_max_abs >= MIN_COUNTS ? "PASS" : "FAIL (motor not turning -- stall or wiring)"); \
    } while (0)

    TEST_MOTOR("M1", M1_FWD, M1_BWD, apply_motor1_pwm, get_count1);
    TEST_MOTOR("M2", M2_FWD, M2_BWD, apply_motor2_pwm, get_count2);

    #undef TEST_MOTOR

    printf("=== END MOTOR POWER TEST ===\n\n");
    stop();
}