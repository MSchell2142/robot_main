#include "sonar.h"

#include <cstdio>

#include "pico/stdlib.h"
#include "hardware/gpio.h"

// ============================================================================
// PINS
// ============================================================================
static const uint TRIG_PIN[2] = { 3,  5 };   // [SONAR_FRONT, SONAR_REAR]
static const uint ECHO_PIN[2] = { 2,  4 };

// ============================================================================
// TIMEOUTS
// ============================================================================
static const uint64_t WAIT_RISE_TIMEOUT_US = 50000;  // 50ms
static const uint64_t WAIT_FALL_TIMEOUT_US = 50000;  // 50ms

static const float CM_PER_INCH = 2.54f;

// ============================================================================
// INIT
// ============================================================================
void init_sonar() {
    for (int i = 0; i < 2; i++) {
        gpio_init(TRIG_PIN[i]);
        gpio_set_dir(TRIG_PIN[i], GPIO_OUT);
        gpio_put(TRIG_PIN[i], 0);

        gpio_init(ECHO_PIN[i]);
        gpio_set_dir(ECHO_PIN[i], GPIO_IN);
        gpio_pull_down(ECHO_PIN[i]);
    }

    sleep_ms(50);

    printf("[sonar] initialized  front(TRIG=GPIO%d ECHO=GPIO%d)  rear(TRIG=GPIO%d ECHO=GPIO%d)\n",
           TRIG_PIN[SONAR_FRONT], ECHO_PIN[SONAR_FRONT],
           TRIG_PIN[SONAR_REAR],  ECHO_PIN[SONAR_REAR]);
}

// ============================================================================
// INDEXED READ
// ============================================================================
float read_distance_cm(int sonar) {
    if (sonar < 0 || sonar > 1) return -1.0f;

    gpio_put(TRIG_PIN[sonar], 0);
    sleep_us(5);
    gpio_put(TRIG_PIN[sonar], 1);
    sleep_us(10);
    gpio_put(TRIG_PIN[sonar], 0);

    uint64_t t0 = time_us_64();
    while (gpio_get(ECHO_PIN[sonar]) == 0) {
        if (time_us_64() - t0 > WAIT_RISE_TIMEOUT_US) return -1.0f;
    }

    uint64_t pulse_start = time_us_64();
    while (gpio_get(ECHO_PIN[sonar]) == 1) {
        if (time_us_64() - pulse_start > WAIT_FALL_TIMEOUT_US) return -1.0f;
    }
    uint64_t pulse_end = time_us_64();

    return ((float)(pulse_end - pulse_start) * 0.0343f) * 0.5f;
}

float read_distance_in(int sonar) {
    float d = read_distance_cm(sonar);
    return (d < 0.0f) ? -1.0f : d / CM_PER_INCH;
}

bool sonar_obstacle_within(float threshold_cm, int sonar) {
    float d = read_distance_cm(sonar);
    if (d < 0.0f) return false;
    return d <= threshold_cm;
}

bool sonar_obstacle_within_in(float threshold_in, int sonar) {
    return sonar_obstacle_within(threshold_in * CM_PER_INCH, sonar);
}

float read_nearest_cm() {
    float f = read_distance_cm(SONAR_FRONT);
    float r = read_distance_cm(SONAR_REAR);
    if (f < 0.0f && r < 0.0f) return -1.0f;
    if (f < 0.0f) return r;
    if (r < 0.0f) return f;
    return (f < r) ? f : r;
}

float read_nearest_in() {
    float d = read_nearest_cm();
    return (d < 0.0f) ? -1.0f : d / CM_PER_INCH;
}

// ============================================================================
// CONVENIENCE WRAPPERS
// ============================================================================
void sonar_validation_test() {
    const uint32_t RUN_MS      = 30000;
    const uint32_t INTERVAL_MS = 200;

    printf("\n==========================================================\n");
    printf("  SONAR VALIDATION (30s) -- wave hand in front of each sensor\n");
    printf("  front: TRIG=GPIO%u ECHO=GPIO%u\n", TRIG_PIN[SONAR_FRONT], ECHO_PIN[SONAR_FRONT]);
    printf("  rear:  TRIG=GPIO%u ECHO=GPIO%u\n", TRIG_PIN[SONAR_REAR],  ECHO_PIN[SONAR_REAR]);
    printf("  NO_ECHO = sensor not responding or nothing in range\n");
    printf("==========================================================\n");

    uint32_t t0   = to_ms_since_boot(get_absolute_time());
    uint32_t last = t0;

    while (to_ms_since_boot(get_absolute_time()) - t0 < RUN_MS) {
        uint32_t now = to_ms_since_boot(get_absolute_time());
        if (now - last < INTERVAL_MS) { sleep_ms(5); continue; }
        last = now;

        float f = read_distance_cm(SONAR_FRONT);
        float r = read_distance_cm(SONAR_REAR);
        unsigned long t_s = (unsigned long)((now - t0) / 1000);

        if (f >= 0.0f)
            printf("  t=%3lus  FRONT: %5.1f cm (%4.1f in)\n",
                   t_s, (double)f, (double)(f / CM_PER_INCH));
        else
            printf("  t=%3lus  FRONT: NO_ECHO\n", t_s);

        if (r >= 0.0f)
            printf("  t=%3lus  REAR:  %5.1f cm (%4.1f in)\n",
                   t_s, (double)r, (double)(r / CM_PER_INCH));
        else
            printf("  t=%3lus  REAR:  NO_ECHO\n", t_s);
    }

    printf("==========================================================\n");
    printf("  SONAR VALIDATION DONE\n");
    printf("==========================================================\n\n");
}

bool sonar_should_stop_front() {
    return sonar_obstacle_within_in(SONAR_STOP_DISTANCE_IN, SONAR_FRONT);
}

bool sonar_should_stop_rear() {
    return sonar_obstacle_within_in(SONAR_STOP_DISTANCE_IN, SONAR_REAR);
}

bool sonar_should_stop() {
    return sonar_should_stop_front() || sonar_should_stop_rear();
}
