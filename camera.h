#pragma once
#include <stdint.h>
#include "vision.h"

// ============================================================================
// Return codes from get_camera_direction()
// ============================================================================
//   CAM_LEFT          ball is left of the loose center band (|err| > LOOSE)
//   CAM_RIGHT         ball is right of the loose center band
//   CAM_CENTER_LOOSE  ball inside ±LOOSE but outside ±TIGHT — coarsely aimed
//   CAM_CENTER_TIGHT  ball inside ±TIGHT — perfectly aimed
//   CAM_NONE          no ball detected in frame
//
// The numeric values of LEFT/CENTER_TIGHT/RIGHT/NONE match the old scheme
// (0/1/2/3) so any older code that still checks == 0/1/2/3 continues to
// work; CENTER_LOOSE is new and has value 4.
enum {
    CAM_LEFT         = 0,
    CAM_CENTER_TIGHT = 1,
    CAM_RIGHT        = 2,
    CAM_NONE         = 3,
    CAM_CENTER_LOOSE = 4,
};

// ============================================================================
// Pixel tolerances around the frame's horizontal center line
// ============================================================================
// Frame is 128 px wide, so midpoint is at x=64.
//   TIGHT = ±5 px  → ~8% of frame width; the "dead-on" band
//   LOOSE = ±10 px → ~16% of frame width; the "good enough to advance" band
// LOOSE_TOL must be >= TIGHT_TOL.
#define CAM_TIGHT_TOL  5
#define CAM_LOOSE_TOL  10

// ============================================================================
// Color bits OR'd into the return value of get_camera_direction()
// ============================================================================
// The low nibble holds the direction (CAM_LEFT/RIGHT/etc).
// The high nibble holds which color was detected. Use CAM_COLOR_MASK to
// extract: (raw & CAM_COLOR_MASK) gives one of CAM_COLOR_RED/BLUE/GREEN.
#define CAM_COLOR_MASK   0xF0
#define CAM_COLOR_RED    0x10
#define CAM_COLOR_BLUE   0x20
#define CAM_COLOR_GREEN  0x40

typedef enum {
    CAM_MODE_RED        = 0,   // detect red only, ignore blue/green
    CAM_MODE_BLUE_GREEN = 1,   // detect blue OR green, ignore red
} CameraMode;

void set_camera_mode(CameraMode mode);

void init_camera();
uint8_t get_camera_direction();

// Captures one fresh frame and returns which side of the frame center
// the black boundary line appears on. Call after init_camera() during
// robot initialization. Returns LINE_SIDE_NONE if no line is detected.
LineSide camera_detect_start_side(void);