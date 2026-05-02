// encoders.h
#pragma once
#include <cstdint>

// ===== INIT =====
void init_encoders();
void init_motors();
void init_mpu();

// ===== DIAGNOSTIC =====
// Motors-off test: prints c1/c2 deltas while user hand-spins wheels,
// so we can verify which physical wheel maps to which encoder. Call
// after init_encoders() but before init_motors() so intake stays off.
// Currently disabled -- uncomment in main.cpp if encoders need re-verifying.
void encoder_id_test();
void encoder_rotation_test();  // spin one full turn by hand, read delta = true PPR
void drive_raw_test();              // 10s drive with hard+soft heading correction -- kick test
void intake_test();                 // cycles intake in/out/off in a loop -- comment out when done
void drive_forward_test();          // drive 24 in, compare enc counts to expected and tape measure
void distance_calibration_test();   // 5s drive at match PWM, pause 60s to measure -- run 3x

// Focused 3-phase test for encoder 1: noise check, forward spin, backward spin.
// Run with motors OFF. Uncomment in main.cpp, re-comment when done.
void encoder1_test();

// Motor balance calibration: runs both motors at equal raw PWM for 3s and
// measures enc1 vs enc2 counts. Prints the suggested M2_SPEED_SCALE to set
// in encoders.cpp so both wheels spin at the same speed before any correction.
// Run once on the ground, update the constant, re-flash.
void motor_balance_cal();

// Motor power test: spins each drive motor at 400 PWM for 300ms and checks
// whether its encoder moved. Prints PASS/FAIL per motor. Call after both
// init_encoders() and init_motors(). Robot must be free to roll.
void motor_power_test();

// Automated mag hard-iron calibration: spins robot 360 deg using motors,
// tracks mx/my min/max, computes true offsets and stores them in RAM.
// Call after init_mpu() + init_motors(), before set_pose().
void mag_auto_cal();

// Mag noise test (Stage 2 validation): hold the robot OFF the ground.
// Spins both wheels at 500 PWM for 10 seconds and prints magnetometer
// readings throughout. Lets you see whether motor magnetic fields
// disturb the mag chip. Call after init_mpu() and init_motors().
void mag_noise_test();

// Learned straight-drive balance.
// report_drift_correction is called by drive functions whenever a
// heading-tolerance pivot fires; it nudges a persistent global trim
// in the direction that will reduce drift on future symmetric drives.
// get_balance_trim returns the current learned trim value (PWM points).
void   report_drift_correction(int32_t dc1, int32_t dc2, float drift_deg);
int16_t get_balance_trim();

// ===== UPDATE =====
void mpu_update();
void position_update();
void refresh_gyro_bias();

// ===== LOW-LEVEL MOVEMENT =====
void drive(int16_t m1, int16_t m2);
void stop();
void intake_set_percent(int pct);   // + = pull in, - = eject, 0 = off
void rotate_to(float angle);
void rotate_to_slow(float angle);   // half-speed rotation; use at coverage waypoints
void drive_distance(float inches);
void drive_forward_until_lost();

// ===== WAYPOINT NAVIGATION =====
void go_to_xy(float x_in, float y_in);
void go_to_xy_approx(float x_in, float y_in, float tol_in);  // slow turn + stops short by tol_in
void go_to_xy_scanning(float x_in, float y_in, bool* ball_seen);

float get_pos_x_in();
float get_pos_y_in();
void  set_pose(float x_in, float y_in, float heading_deg);

// ===== SHARED STATE =====
extern float angle_z;           // raw gyro -- diagnostic only
extern float fused_heading;     // complementary-filtered; motion uses this
extern bool  g_sonar_skip_next; // set when a drive is blocked; ST_COVERAGE advances an extra waypoint