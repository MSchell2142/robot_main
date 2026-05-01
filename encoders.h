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
void encoder_id_test();

// ===== UPDATE =====
void mpu_update();
void position_update();
void refresh_gyro_bias();

// ===== LOW-LEVEL MOVEMENT =====
void drive(int16_t m1, int16_t m2);
void stop();
void intake_set_percent(int pct);   // + = pull in, - = eject, 0 = off
void rotate_to(float angle);
void drive_distance(float inches);
void drive_forward_until_lost();

// ===== WAYPOINT NAVIGATION =====
void go_to_xy(float x_in, float y_in);
void go_to_xy_scanning(float x_in, float y_in, bool* ball_seen);

float get_pos_x_in();
float get_pos_y_in();
void  set_pose(float x_in, float y_in, float heading_deg);

// ===== SHARED STATE =====
extern float angle_z;        // raw gyro -- diagnostic only
extern float fused_heading;  // complementary-filtered; motion uses this