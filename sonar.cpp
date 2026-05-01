#include "sonar.h"

#include <cstdio>

#include "pico/stdlib.h"
#include "hardware/gpio.h"

// ============================================================================
// PINS
// ============================================================================
static const uint TRIG_PIN = 3;
static const uint ECHO_PIN = 2;

// ============================================================================
// TIMEOUTS
// ============================================================================
// HC-SR04 echo pin pulses HIGH for the round-trip time of the ultrasonic
// burst. At max range (~400cm) the pulse is ~23ms. We use 50ms timeouts
// for both the wait-for-rising-edge and wait-for-falling-edge phases.
// That gives plenty of headroom and bounds the worst-case blocking time
// of read_distance_cm() to ~100ms.
static const uint64_t WAIT_RISE_TIMEOUT_US  = 50000;  // 50ms
static const uint64_t WAIT_FALL_TIMEOUT_US  = 50000;  // 50ms

void init_sonar() {
    gpio_init(TRIG_PIN);
    gpio_set_dir(TRIG_PIN, GPIO_OUT);
    gpio_put(TRIG_PIN, 0);

    gpio_init(ECHO_PIN);
    gpio_set_dir(ECHO_PIN, GPIO_IN);
    gpio_pull_down(ECHO_PIN);

    // Let the sensor settle after power-up.
    sleep_ms(50);

    printf("[sonar] initialized (TRIG=GPIO%d ECHO=GPIO%d)\n",
           TRIG_PIN, ECHO_PIN);
}

float read_distance_cm() {
    // Send 10us TRIG pulse.
    gpio_put(TRIG_PIN, 0);
    sleep_us(5);
    gpio_put(TRIG_PIN, 1);
    sleep_us(10);
    gpio_put(TRIG_PIN, 0);

    // Wait for ECHO to go HIGH (start of return pulse).
    uint64_t t0 = time_us_64();
    while (gpio_get(ECHO_PIN) == 0) {
        if (time_us_64() - t0 > WAIT_RISE_TIMEOUT_US) {
            return -1.0f;
        }
    }

    // ECHO is now HIGH. Time how long it stays high.
    uint64_t pulse_start = time_us_64();
    while (gpio_get(ECHO_PIN) == 1) {
        if (time_us_64() - pulse_start > WAIT_FALL_TIMEOUT_US) {
            return -1.0f;
        }
    }
    uint64_t pulse_end = time_us_64();

    uint64_t duration_us = pulse_end - pulse_start;

    // Speed of sound = 343 m/s = 0.0343 cm/us. Round-trip, so divide by 2.
    return ((float)duration_us * 0.0343f) * 0.5f;
}

bool sonar_obstacle_within(float threshold_cm) {
    float d = read_distance_cm();
    if (d < 0.0f) return false;          // no echo - assume clear
    return (d > 0.0f) && (d <= threshold_cm);
}