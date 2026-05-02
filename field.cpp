#include "field.h"

#include <cstdio>
#include <cmath>

// ============================================================================
// State
// ============================================================================
HomeSide g_home_side = HOME_SIDE_DEFAULT;

#define MAX_WAYPOINTS 24

struct Waypoint {
    float x;
    float y;
    bool  is_phase_boundary;    // rotate-to-final-heading after this WP?
    bool  is_scan_checkpoint;   // stop and take a fresh image after arriving?
    float tolerance_in;         // stop within this many inches of the target
};

static Waypoint s_wps[MAX_WAYPOINTS];
static int      s_wp_count = 0;
static int      s_wp_index = 0;
static int      s_loop_count = 0;   // diagnostic: how many times we've looped

// ============================================================================
// Start pose
// ============================================================================
float field_home_x_in()         { return 14.0f; }
float field_home_y_in()         { return 12.0f; }
float field_start_heading_deg() { return 90.0f; }

// Park position: where the robot ends up at the end of each pattern
// loop and where RETURN_HOME drives to if the match clock runs low.
// 42 in along X = above goal center; 24 in along Y = 12 in clear of
// goal mouth (which extends to y=12). Final rotation faces +Y so the
// robot's backside is toward the goal -- ready to back into it.
float field_park_x_in()         { return 42.0f; }
float field_park_y_in()         { return 24.0f; }

// ============================================================================
// My goal location (bottom wall, centered on X)
// ============================================================================
float field_goal_center_x_in() { return GOAL_X_CENTER_IN; }
float field_goal_center_y_in() { return MY_GOAL_Y_MAX_IN * 0.5f; }
float field_goal_approach_x_in() { return GOAL_X_CENTER_IN; }
float field_goal_approach_y_in() { return MY_GOAL_Y_MAX_IN + 6.0f; }

// ============================================================================
// Final heading
// ============================================================================
// 90 deg = facing +Y, robot's backside toward the goal/bottom wall.
float field_final_heading_deg() { return 90.0f; }

// ============================================================================
// SCANNING PATH: Lawnmower with intermediate scan checkpoints
// ============================================================================
// From start (12, 12) facing +Y, the robot drives 3 long lanes parallel
// to Y. Long lanes are broken into ~36-48 in segments by scan checkpoints:
// the robot stops at each checkpoint and takes a fresh camera image instead
// of scanning during driving.
//
// Lane 1  (x=12, up):
//   WP 1: (12,  48) [checkpoint] ~36 in
//   WP 2: (12,  84) [checkpoint] ~36 in
//   WP 3: (12, 114)              ~30 in  -- top of lane
//
// Transition to lane 2:
//   WP 4: (42, 114)              ~30 in
//
// Lane 2 (x=42, down to safe row y=18):
//   WP 5: (42,  66) [checkpoint] ~48 in
//   WP 6: (42,  18)              ~48 in
//
// Transition to lane 3:
//   WP 7: (70,  18)              ~28 in
//
// Lane 3 (x=70, up):
//   WP 8: (70,  66) [checkpoint] ~48 in
//   WP 9: (70, 114)              ~48 in
//
// Return down (x=70):
//   WP10: (70,  66) [checkpoint] ~48 in
//   WP11: (70,  18)              ~48 in
//
// Park:
//   WP12: (42,  24) [PHASE BOUNDARY]
//
// After WP12 the iterator wraps back to WP1 and the pattern repeats.
// Goal-clearance rule: lane 2 stops at y=18 (not y=12) to avoid the
// goal-mouth speed bumps. Final park at (42, 24) is 12 in above the goal.
static void build_waypoints() {
    s_wp_count = 0;
    s_wp_index = 0;

    // Lane 1 (x=12, driving up from start at y=12)
    s_wps[s_wp_count++] = { 14.0f,  48.0f, false, true,  4.0f };  // WP 1  - lane 1 checkpoint (+Y)
    s_wps[s_wp_count++] = { 14.0f,  84.0f, false, true,  4.0f };  // WP 2  - lane 1 checkpoint (+Y)
    s_wps[s_wp_count++] = { 14.0f, 114.0f, false, true,  2.0f };  // WP 3  - lane 1 top (+Y)
    // Transition
    s_wps[s_wp_count++] = { 42.0f, 114.0f, false, false, 2.0f };  // WP 4  - top transition (+X, no scan)
    // Lane 2 (x=42, driving down to safe row)
    s_wps[s_wp_count++] = { 42.0f,  66.0f, false, true,  4.0f };  // WP 5  - lane 2 checkpoint (-Y)
    s_wps[s_wp_count++] = { 42.0f,  18.0f, false, true,  2.0f };  // WP 6  - lane 2 bottom (-Y)
    // Transition
    s_wps[s_wp_count++] = { 70.0f,  18.0f, false, false, 2.0f };  // WP 7  - bottom transition (+X, no scan)
    // Lane 3 (x=70, driving up)
    s_wps[s_wp_count++] = { 70.0f,  66.0f, false, true,  4.0f };  // WP 8  - lane 3 checkpoint (+Y)
    s_wps[s_wp_count++] = { 70.0f, 114.0f, false, true,  2.0f };  // WP 9  - lane 3 top (+Y)
    // Return down (x=70)
    s_wps[s_wp_count++] = { 70.0f,  66.0f, false, true,  4.0f };  // WP 10 - return checkpoint (-Y)
    s_wps[s_wp_count++] = { 70.0f,  18.0f, false, true,  2.0f };  // WP 11 - return bottom (-Y)
    // Park
    s_wps[s_wp_count++] = { 42.0f,  24.0f, true,  false, 2.0f };  // WP 12 - park [BOUNDARY]

    printf("[field] %d waypoints (lawnmower + scan checkpoints):\n", s_wp_count);
    for (int i = 0; i < s_wp_count; i++) {
        printf("  WP %2d: (%.1f, %.1f)%s%s\n",
               i + 1,
               (double)s_wps[i].x, (double)s_wps[i].y,
               s_wps[i].is_phase_boundary    ? "  <- phase boundary"  : "",
               s_wps[i].is_scan_checkpoint   ? "  <- scan checkpoint" : "");
    }
    printf("  Final heading at boundaries: %.1f deg\n",
           (double)field_final_heading_deg());
    printf("  Path repeats from WP 1 after WP %d completes.\n", s_wp_count);
}

void field_init() {
    s_loop_count = 0;
    build_waypoints();
}

bool field_next_waypoint(float* x, float* y) {
    if (s_wp_count == 0) return false;
    if (s_wp_index >= s_wp_count) return false;
    if (x) *x = s_wps[s_wp_index].x;
    if (y) *y = s_wps[s_wp_index].y;
    return true;
}

bool field_current_is_phase_boundary() {
    if (s_wp_index >= s_wp_count) return false;
    return s_wps[s_wp_index].is_phase_boundary;
}

bool field_current_is_scan_checkpoint() {
    if (s_wp_index >= s_wp_count) return false;
    return s_wps[s_wp_index].is_scan_checkpoint;
}

float field_current_tolerance_in() {
    if (s_wp_index >= s_wp_count) return 0.0f;
    return s_wps[s_wp_index].tolerance_in;
}

void field_advance_waypoint() {
    if (s_wp_index < s_wp_count) {
        s_wp_index++;
        // Auto-wrap: if we've finished the list, loop back to start
        // for another full pass. Match timer in main.cpp stops the loop
        // when time runs out.
        if (s_wp_index >= s_wp_count) {
            s_loop_count++;
            s_wp_index = 0;
            printf("[field] *** path complete - starting loop %d ***\n",
                   s_loop_count + 1);
        }
    }
}

void field_reset_coverage() {
    s_wp_index = 0;
}

int field_current_waypoint_index() { return s_wp_index + 1; }
int field_waypoint_count()         { return s_wp_count; }

bool field_point_is_in_bounds(float x, float y) {
    const float M = 2.0f;
    return (x > M) && (x < FIELD_X_IN - M) &&
           (y > M) && (y < FIELD_Y_IN - M);
}