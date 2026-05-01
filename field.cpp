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
    bool  is_phase_boundary;   // rotate-to-final-heading after this WP?
};

static Waypoint s_wps[MAX_WAYPOINTS];
static int      s_wp_count = 0;
static int      s_wp_index = 0;
static int      s_loop_count = 0;   // diagnostic: how many times we've looped

// ============================================================================
// Start pose
// ============================================================================
float field_home_x_in()         { return 12.0f; }
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
// SCANNING PATH: Lawnmower (skips centerline sweep, starts from (12,12))
// ============================================================================
// From start (12, 12) facing +Y, the robot drives 3 long lanes parallel
// to Y, snaking the field for full coverage:
//
//   WP 1: (12, 132) - lane 1: drive UP all the way along left edge
//   WP 2: (42, 132) - top transition to lane 2
//   WP 3: (42,  18) - lane 2: drive DOWN to safe row (NOT into goal)
//   WP 4: (70,  18) - bottom transition to lane 3 along safe row
//   WP 5: (70, 132) - lane 3: drive UP all the way to top
//   WP 6: (70,  18) - return down to safe row
//   WP 7: (42,  24) - park above goal center (3.5 ft x, 2 ft y)
//                     [PHASE BOUNDARY] rotate to face +Y so backside is
//                     toward the goal mouth, ready to back into goal.
//
// After WP 7, the iterator wraps back to WP 1 and the whole pattern
// repeats. The state machine in main.cpp watches the match clock and
// stops when time runs out.
//
// Goal-clearance rule: any segment that would cross x in [24, 60] at
// y < 18 is rerouted via y = 18 to avoid the goal-mouth speed bumps.
// Lane 2 stops at y=18 instead of continuing to y=12 for this reason.
// Final park at (42, 24) is 12 in above the goal mouth, safely clear.
static void build_waypoints() {
    s_wp_count = 0;
    s_wp_index = 0;

    s_wps[s_wp_count++] = { 12.0f, 132.0f, false };  // WP 1 - lane 1 up
    s_wps[s_wp_count++] = { 42.0f, 132.0f, false };  // WP 2 - top transition
    s_wps[s_wp_count++] = { 42.0f,  18.0f, false };  // WP 3 - lane 2 down (safe)
    s_wps[s_wp_count++] = { 70.0f,  18.0f, false };  // WP 4 - bottom transition
    s_wps[s_wp_count++] = { 70.0f, 132.0f, false };  // WP 5 - lane 3 up
    s_wps[s_wp_count++] = { 70.0f,  18.0f, false };  // WP 6 - return down
    s_wps[s_wp_count++] = { 42.0f,  24.0f, true  };  // WP 7 - park above goal

    printf("[field] %d waypoints (lawnmower from start):\n", s_wp_count);
    for (int i = 0; i < s_wp_count; i++) {
        printf("  WP %2d: (%.1f, %.1f)%s\n",
               i + 1,
               (double)s_wps[i].x, (double)s_wps[i].y,
               s_wps[i].is_phase_boundary ? "  <- phase boundary" : "");
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