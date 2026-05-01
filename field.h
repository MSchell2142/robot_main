// field.h - field geometry and scripted scanning path
#pragma once
#include <cstdint>

// ============================================================================
// FIELD GEOMETRY
// ============================================================================
// Coordinate system: origin (0, 0) at the BOTTOM-LEFT corner of the field.
// +X points right, along the 7 ft (84") short axis.
// +Y points up,    along the 12 ft (144") long axis.
// Heading 0   = facing +X
// Heading 90  = facing +Y (this is the robot's start heading)
// Heading -90 = facing -Y
// Heading 180 = facing -X
//
// Field: 84 in (7 ft) wide x 144 in (12 ft) long.
// Goals: 36 in (3 ft) wide along X, 12 in (1 ft) deep into the field along Y.
//   MY goal:        x in [24, 60], y in [0, 12]   (centered on bottom wall)
//   OPPONENT goal:  x in [24, 60], y in [132, 144] (centered on top wall)
// ============================================================================
#define FIELD_X_IN           84.0f
#define FIELD_Y_IN          144.0f

#define GOAL_WIDTH_IN        36.0f
#define GOAL_DEPTH_IN        12.0f
#define GOAL_X_CENTER_IN    (FIELD_X_IN * 0.5f)
#define GOAL_X_MIN_IN       (GOAL_X_CENTER_IN - GOAL_WIDTH_IN * 0.5f)
#define GOAL_X_MAX_IN       (GOAL_X_CENTER_IN + GOAL_WIDTH_IN * 0.5f)

#define MY_GOAL_Y_MIN_IN     0.0f
#define MY_GOAL_Y_MAX_IN     GOAL_DEPTH_IN
#define OPP_GOAL_Y_MIN_IN   (FIELD_Y_IN - GOAL_DEPTH_IN)
#define OPP_GOAL_Y_MAX_IN    FIELD_Y_IN

// ============================================================================
// HOME SIDE
// ============================================================================
enum HomeSide {
    HOME_LEFT  = 0,
    HOME_RIGHT = 1,
};
#define HOME_SIDE_DEFAULT HOME_LEFT
extern HomeSide g_home_side;

// ============================================================================
// FUNCTIONS
// ============================================================================

// Initial robot pose. (12, 12) facing +Y.
float field_home_x_in();
float field_home_y_in();
float field_start_heading_deg();

// Park position - where the robot ends up at the end of each pattern
// loop, OR where it returns to if match time expires. (42, 24) puts
// it directly above goal center, 12 in clear of the goal mouth speed
// bumps, ready to back into the goal after a final +Y rotation.
float field_park_x_in();
float field_park_y_in();

// My goal center (for delivery navigation).
float field_goal_center_x_in();
float field_goal_center_y_in();
float field_goal_approach_x_in();
float field_goal_approach_y_in();

// FINAL HEADING for the end of each phase. Robot rotates to this heading
// before the phase transitions. 90 deg = facing +Y (back to goal).
float field_final_heading_deg();

// Build the scripted scanning path (Step 1 centerline + Step 2 lawnmower).
void  field_init();

// ============================================================================
// WAYPOINT ITERATOR (extended for phases + repeat)
// ============================================================================
// Waypoints carry a "phase boundary" flag. When the iterator returns a
// boundary waypoint, the state machine should rotate to final heading
// AFTER reaching it, before requesting the next waypoint. This is how
// Step 1 and Step 2 are joined into one list cleanly.
//
// After the last waypoint is reached, field_advance_waypoint() resets
// to 0 automatically — the path repeats indefinitely. The state machine
// is responsible for noticing match time has expired and parking.

bool  field_next_waypoint(float* x, float* y);

// Returns true if the *current* waypoint is a phase boundary (i.e. the
// robot should rotate to final heading after arriving at it). Used by
// the state machine to do the rotation before advancing.
bool  field_current_is_phase_boundary();

void  field_advance_waypoint();
void  field_reset_coverage();

// Returns the current waypoint index (1-based) and total count, for logs.
int   field_current_waypoint_index();
int   field_waypoint_count();

bool  field_point_is_in_bounds(float x, float y);