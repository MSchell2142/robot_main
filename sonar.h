// sonar.h - dual HC-SR04 ultrasonic sensors
//
// Sonar 0 (FRONT): TRIG=GPIO3  ECHO=GPIO2
// Sonar 1 (REAR):  TRIG=GPIO27 ECHO=GPIO26
//
// HC-SR04 timing:
//   - 10us TRIG pulse
//   - ECHO goes high once ultrasonic burst is sent
//   - ECHO stays high until echo received (or ~30ms timeout)
//   - distance_cm = pulse_us * 0.0343 / 2
//
// Max useful range ~400 cm. Min ~2 cm. 50ms timeouts cap worst-case
// blocking time at ~100ms per read. Don't call inside tight motor loops.
#pragma once
#include <cstdint>

#define SONAR_FRONT  0
#define SONAR_REAR   1

// Initialises both sonars.
void init_sonar();

// Prints front and rear distance readings every 200ms for 30 seconds.
// Use to verify wiring and basic sensor function before match use.
void sonar_validation_test();

// ============================================================================
// INDEXED API — use SONAR_FRONT or SONAR_REAR
// ============================================================================

// Returns distance in cm. Returns -1.0f on timeout / no obstacle in range.
float read_distance_cm(int sonar);

// Returns distance in inches. Returns -1.0f on timeout.
float read_distance_in(int sonar);

// Returns true if an obstacle is within threshold_cm on the given sonar.
// Treats no-echo as "clear" — a missed reading never triggers a false stop.
bool sonar_obstacle_within(float threshold_cm, int sonar);

// Same as above, threshold in inches.
bool sonar_obstacle_within_in(float threshold_in, int sonar);

// Returns the closer of the two sonar readings (smallest positive value).
// Returns -1.0f only if both sensors return no-echo.
// Use this in drive loops instead of reading each sensor separately.
float read_nearest_cm();
float read_nearest_in();

// ============================================================================
// CONVENIENCE WRAPPERS — default stop distance 6 inches
// ============================================================================
#define SONAR_STOP_DISTANCE_IN  10.0f

bool sonar_should_stop_front();   // front sonar within 6 in
bool sonar_should_stop_rear();    // rear  sonar within 6 in
bool sonar_should_stop();         // either sonar within 6 in (drop-in replacement)
