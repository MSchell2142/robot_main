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
static const float WHEEL_RADIUS    = 0.0281f;    // meters
static const float WHEELBASE_IN    = 10.0f;      // MEASURE YOURS
static const float WHEELBASE_M     = WHEELBASE_IN / 39.3701f;
static const int   PPR             = 1920;
static const float IN_PER_M        = 39.3701f;

// Encoder calibration. Verified by spinning each wheel the same physical
// distance: c1 reads 783 ticks while c2 reads 1908 ticks. Ratio is 2.44,
// meaning encoder 1 counts at ~41% the rate of encoder 2 for identical
// rotation. We trust c2 as canonical (likely full quadrature) and scale
// c1's distance up by 2.44 to match. If encoder 1 is later repaired or
// replaced, set both back to 1.0f.
static const float WHEEL_SCALE_M1  = 2.44f;
static const float WHEEL_SCALE_M2  = 1.0f;

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
            if (b != a) count1--; else count1++;
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
    return ((float)c / PPR) * circ * WHEEL_SCALE_M1;
}
static float wheel_distance_m2(int32_t c) {
    const float circ = 2.0f * (float)M_PI * WHEEL_RADIUS;
    return ((float)c / PPR) * circ * WHEEL_SCALE_M2;
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

void drive(int16_t m1, int16_t m2) {
    if (m1 > (int16_t)PWM_WRAP) m1 = (int16_t)PWM_WRAP;
    if (m1 < -(int16_t)PWM_WRAP) m1 = -(int16_t)PWM_WRAP;

    if (m1 > 0) { pwm_set_gpio_level(M1_FWD, m1); pwm_set_gpio_level(M1_BWD, 0); }
    else if (m1 < 0) { pwm_set_gpio_level(M1_FWD, 0); pwm_set_gpio_level(M1_BWD, -m1); }
    else { pwm_set_gpio_level(M1_FWD, 0); pwm_set_gpio_level(M1_BWD, 0); }

    if (m2 > (int16_t)PWM_WRAP) m2 = (int16_t)PWM_WRAP;
    if (m2 < -(int16_t)PWM_WRAP) m2 = -(int16_t)PWM_WRAP;

    if (m2 > 0) { pwm_set_gpio_level(M2_FWD, m2); pwm_set_gpio_level(M2_BWD, 0); }
    else if (m2 < 0) { pwm_set_gpio_level(M2_FWD, 0); pwm_set_gpio_level(M2_BWD, -m2); }
    else { pwm_set_gpio_level(M2_FWD, 0); pwm_set_gpio_level(M2_BWD, 0); }
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
#define MAG_OFFSET_X  +170.2f
#define MAG_OFFSET_Y  -458.7f
#define MAG_OFFSET_Z  +156.0f

float angle_z = 0.0f;
float fused_heading = 0.0f;
static uint32_t mpu_last_us = 0;
static bool mpu_ok = false;
static float gyro_bias_dps = 0.0f;

static const float ENC_HEADING_SIGN = +1.0f;
static const float ALPHA_GYRO = 0.50f;

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

// How fast the mag pulls fused_heading toward absolute truth.
// 0.005 = 0.5% pull per mag read. With mag reads at ~5 Hz, that's a
// time constant of ~40 seconds — slow enough that motor-induced noise
// doesn't whip the heading around, fast enough that gyro drift over a
// 5 min match stays within ~2 degrees.
static const float MAG_PULL_RATE = 0.005f;

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

    // --- Stage 2A diagnostic: print raw mag + calibrated heading 1/sec ---
#if MAG_ENABLED
    {
        static uint32_t s_last_mag_print_ms = 0;
        uint32_t now_ms = to_ms_since_boot(get_absolute_time());
        if (now_ms - s_last_mag_print_ms > 1000) {
            s_last_mag_print_ms = now_ms;
            float mx, my, mz;
            if (ak_read_xyz(&mx, &my, &mz)) {
                // Apply hard-iron calibration offsets, then compute heading.
                // Robot forward = chip -X, robot left = chip -Y.
                float mxc = mx - MAG_OFFSET_X;
                float myc = my - MAG_OFFSET_Y;
                float h = atan2f(-myc, -mxc) * 180.0f / (float)M_PI;
                printf("  [mag] raw x=%+7.1f y=%+7.1f z=%+7.1f  cal_h=%+6.1f\n",
                       (double)mx, (double)my, (double)mz, (double)h);
            } else {
                printf("  [mag] no data this tick\n");
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

    uint32_t settle_end = to_ms_since_boot(get_absolute_time()) + SETTLE_MS;
    while (to_ms_since_boot(get_absolute_time()) < settle_end) {
        mpu_update();
        sleep_ms(5);
    }
}

void drive_distance(float inches) {
    printf("  [drive_distance] target=%.1f in (%.3f m), starting from heading=%.1f\n",
        (double)inches, (double)(inches / IN_PER_M), (double)fused_heading);
    const float    DIST_TOL_M     = 0.02f;
    const int16_t  BASE_PWM       = 500;
    // Scale timeout to commanded distance (250 ms per inch + 3 sec floor).
    const uint32_t TIMEOUT_MS = (uint32_t)(fabsf(inches) * 250.0f) + 3000;
    const float    HEADING_GAIN   = 15.0f;
    const int16_t  MAX_STEER      = 250;

    float target_m   = inches / IN_PER_M;
    float target_abs = fabsf(target_m);
    int16_t base     = (target_m >= 0.0f) ? BASE_PWM : -BASE_PWM;

    // Sonar safety, only on FORWARD drives. Skip for short jogs (<3in)
    // like chase nudges, which would thrash if a ball is dead ahead.
    const bool     SONAR_ACTIVE      = (inches >= 3.0f);
    const uint32_t SONAR_INTERVAL_MS = 200;
    const float    SONAR_STOP_CM     = 30.0f;
    const float    SONAR_BACKUP_IN   = -4.0f;
    const uint32_t SONAR_WAIT_MS     = 10000;
    uint32_t       last_sonar        = 0;

    mpu_update();
    float ref_heading = fused_heading;

    int32_t c1_start = get_count1();
    int32_t c2_start = get_count2();
    uint32_t t0 = to_ms_since_boot(get_absolute_time());

    while (true) {
        mpu_update();
        position_update();

        int32_t c1 = get_count1() - c1_start;
        int32_t c2 = get_count2() - c2_start;
        float d1 = wheel_distance_m1(c1);
        float d2 = wheel_distance_m2(c2);
        float avg_abs = (fabsf(d1) + fabsf(d2)) * 0.5f;

        if (avg_abs >= target_abs - DIST_TOL_M) break;
        if (to_ms_since_boot(get_absolute_time()) - t0 > TIMEOUT_MS) break;

        uint32_t now = to_ms_since_boot(get_absolute_time());

        // --- Sonar safety (forward only) ---
        if (SONAR_ACTIVE && (now - last_sonar > SONAR_INTERVAL_MS)) {
            last_sonar = now;
            float dist_cm = read_distance_cm();
            if (dist_cm > 0.0f && dist_cm <= SONAR_STOP_CM) {
                printf("  [sonar] OBSTACLE at %.1f cm during drive_distance! "
                       "stop, poll up to %lums for clearance\n",
                       (double)dist_cm, (unsigned long)SONAR_WAIT_MS);
                stop();

                // Poll once per second. Exit early as soon as path clears.
                bool cleared = false;
                uint32_t wait_t0 = to_ms_since_boot(get_absolute_time());
                while (to_ms_since_boot(get_absolute_time()) - wait_t0
                       < SONAR_WAIT_MS) {
                    sleep_ms(1000);
                    float check_cm = read_distance_cm();
                    if (check_cm < 0.0f) {
                        printf("  [sonar] poll: no echo (clear)\n");
                        cleared = true;
                        break;
                    } else if (check_cm > SONAR_STOP_CM) {
                        printf("  [sonar] poll: clear (%.1f cm) - resuming\n",
                               (double)check_cm);
                        cleared = true;
                        break;
                    } else {
                        printf("  [sonar] poll: still blocked (%.1f cm)\n",
                               (double)check_cm);
                    }
                }

                if (cleared) {
                    mpu_update();
                    position_update();
                    continue;
                }

                // Still blocked. Direct motor backup (avoid recursion).
                printf("  [sonar] still blocked after %lums - backing up\n",
                       (unsigned long)SONAR_WAIT_MS);
                int32_t bc1_start = get_count1();
                int32_t bc2_start = get_count2();
                float backup_target_m = fabsf(SONAR_BACKUP_IN) / IN_PER_M;
                uint32_t backup_t0 = to_ms_since_boot(get_absolute_time());
                while (true) {
                    int32_t bc1 = get_count1() - bc1_start;
                    int32_t bc2 = get_count2() - bc2_start;
                    float bd1 = fabsf(wheel_distance_m1(bc1));
                    float bd2 = fabsf(wheel_distance_m2(bc2));
                    float bavg = (bd1 + bd2) * 0.5f;
                    if (bavg >= backup_target_m) break;
                    if (to_ms_since_boot(get_absolute_time()) - backup_t0 > 2000) break;
                    drive(-BASE_PWM, -BASE_PWM);
                    sleep_ms(5);
                }
                stop();
                printf("  [sonar] back-up complete -- aborting drive_distance\n");
                return;
            }
        }

        float heading_err = wrap180(ref_heading - fused_heading);
        float steer_f     = HEADING_GAIN * heading_err;
        if (steer_f >  (float)MAX_STEER) steer_f =  (float)MAX_STEER;
        if (steer_f < -(float)MAX_STEER) steer_f = -(float)MAX_STEER;
        int16_t steer = (int16_t)steer_f;

        int16_t m1 = base + steer;
        int16_t m2 = base - steer;
        drive(m1, m2);

        sleep_ms(5);
    }
    
    printf("  [drive_distance] done\n");
    stop();
}

void drive_forward_until_lost() {
    const int16_t BASE_PWM = 500;
    const uint32_t CAM_INTERVAL_MS = 100;
    const int     LOST_LIMIT = 2;
    const float   POST_LOST_IN = 8.0f;
    const float   HEADING_GAIN = 15.0f;
    const int16_t MAX_STEER    = 250;

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
    const float    SONAR_STOP_CM     = 15.0f;
    uint32_t       last_sonar        = 0;

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
            float dist_cm = read_distance_cm();
            if (dist_cm > 0.0f && dist_cm <= SONAR_STOP_CM) {
                printf("  [sonar] OBSTACLE at %.1f cm during blind roll! aborting collection\n",
                       (double)dist_cm);
                stop();
                // No back-up here; let chase logic move on.
                return;
            }
        }

        float heading_err = wrap180(ref_heading - fused_heading);
        float steer_f     = HEADING_GAIN * heading_err;
        if (steer_f >  (float)MAX_STEER) steer_f =  (float)MAX_STEER;
        if (steer_f < -(float)MAX_STEER) steer_f = -(float)MAX_STEER;
        int16_t steer = (int16_t)steer_f;

        drive(BASE_PWM + steer, BASE_PWM - steer);
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
    const float    HEADING_GAIN    = 15.0f;
    const int16_t  MAX_STEER       = 250;

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
    const float    SONAR_STOP_CM     = 30.0f;
    const float    SONAR_BACKUP_IN   = -4.0f;
    const uint32_t SONAR_WAIT_MS     = 10000;
    uint32_t       last_sonar        = 0;

    int32_t c1_start = get_count1();
    int32_t c2_start = get_count2();
    uint32_t t0 = to_ms_since_boot(get_absolute_time());
    uint32_t last_cam = 0;
    uint32_t last_dbg = 0;

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
            float dist_cm = read_distance_cm();
            if (dist_cm > 0.0f && dist_cm <= SONAR_STOP_CM) {
                printf("[sonar] OBSTACLE at %.1f cm! stop, poll up to %lums for clearance\n",
                       (double)dist_cm, (unsigned long)SONAR_WAIT_MS);
                stop();

                bool cleared = false;
                uint32_t wait_t0 = to_ms_since_boot(get_absolute_time());
                while (to_ms_since_boot(get_absolute_time()) - wait_t0
                       < SONAR_WAIT_MS) {
                    sleep_ms(1000);
                    float check_cm = read_distance_cm();
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

                printf("[sonar] still blocked after %lums - backing up\n",
                       (unsigned long)SONAR_WAIT_MS);
                drive_distance(SONAR_BACKUP_IN);
                printf("[sonar] back-up complete -- exiting drive, will retry waypoint\n");
                return;
            }
        }

        if (now - last_cam > CAM_INTERVAL_MS) {
            last_cam = now;
            uint8_t dir = get_camera_direction();
            if (dir != CAM_NONE) {
                if (ball_seen) *ball_seen = true;
                printf("[go_to_xy_scanning] ball seen mid-drive, dir=%u\n", dir);
                stop();
                return;
            }
        }

        float heading_err = wrap180(ref_heading - fused_heading);
        float steer_f     = HEADING_GAIN * heading_err;
        if (steer_f >  (float)MAX_STEER) steer_f =  (float)MAX_STEER;
        if (steer_f < -(float)MAX_STEER) steer_f = -(float)MAX_STEER;
        int16_t steer = (int16_t)steer_f;

        int16_t m1 = BASE_PWM + steer;
        int16_t m2 = BASE_PWM - steer;
        drive(m1, m2);

        if (now - last_dbg > 1000) {
            last_dbg = now;
            printf("  [drive] head=%.1f ref=%.1f err=%+.1f steer=%+d  c1=%ld c2=%ld  d1=%.3f d2=%.3f\n",
                   (double)fused_heading, (double)ref_heading,
                   (double)heading_err, (int)steer,
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