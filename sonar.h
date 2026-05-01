// sonar.h - HC-SR04 ultrasonic sensor on GPIO 2/3
#pragma once
#include <cstdint>

// ============================================================================
// PINS
// ============================================================================
// TRIG = GPIO 3  (output, 10us pulse to start a measurement)
// ECHO = GPIO 2  (input, pulse-width is proportional to distance)
//
// HC-SR04 timing:
//   - 10us TRIG pulse
//   - ECHO goes high once ultrasonic burst is sent
//   - ECHO stays high until echo received (or ~30ms timeout)
//   - distance_cm = pulse_us * 0.0343 / 2
//
// Max useful range ~400 cm. Min ~2 cm. We use 50ms timeouts which
// caps measurable range at ~857 cm — far beyond useful — so any
// non-error reading is real.
// ============================================================================

void init_sonar();

// Returns distance in cm (positive). Returns -1.0f if no echo received
// within timeout (sensor hung, no obstacle in range, or echo lost).
//
// IMPORTANT: this is a BLOCKING call. Worst case ~100 ms (50 ms wait
// for echo start + 50 ms wait for echo end). Don't call it inside any
// tight motor-control loop — call it on a slow cadence (~200 ms) and
// stash the result.
float read_distance_cm();

// Convenience: returns true if an obstacle is within 'threshold_cm'.
// Treats no-echo (-1.0f) as "no obstacle" so the robot doesn't panic
// when the sensor briefly misses a reading.
bool sonar_obstacle_within(float threshold_cm);