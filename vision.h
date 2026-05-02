#ifndef VISION_H
#define VISION_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define VISION_FRAME_W 128
#define VISION_FRAME_H 128

typedef enum {
    TARGET_RED   = 0,
    TARGET_BLUE  = 1,
    TARGET_GREEN = 2,
} TargetColor;

int detect_red_blob(const uint8_t* frame, uint32_t min_count);
int detect_color_blob(const uint8_t* frame, uint32_t min_count, TargetColor color);
uint32_t get_red_pixel_count(void);

int get_blob_midpoint(int16_t* cx, int16_t* cy);

// Black boundary-line detection for start-side determination.
// Scans the bottom half of the frame for the largest dark cluster.
// Returns which side of the frame center the cluster is on.
typedef enum {
    LINE_SIDE_NONE  = 0,
    LINE_SIDE_LEFT  = 1,
    LINE_SIDE_RIGHT = 2,
} LineSide;

LineSide detect_black_line_side(const uint8_t* frame);

int verify_is_ball(int16_t* radius_px, float* circularity);

// Overlays
void draw_bbox_overlay(uint8_t* frame, uint16_t color_rgb565);
void draw_midpoint_overlay(uint8_t* frame, uint16_t color_rgb565);
void draw_centroid_overlay(uint8_t* frame, uint16_t color_rgb565);
void draw_radii_overlay(uint8_t* frame, uint16_t color_rgb565);

#ifdef __cplusplus
}
#endif

#endif // VISION_H