// vision.c

#include "vision.h"
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#define PI_F 3.14159265f

// ===== PER-PIXEL RED CLASSIFIER =====
// These thresholds work on 8-bit RGB expanded from RGB565.
//
// RED_MIN_R            — raw red channel floor. 80 filters most dark/warm
//                        surfaces. Lower it if the ball fails in dim light.
// RED_MAX_G_ABS        — absolute green ceiling. Keeps orange/yellow out.
// RED_MAX_B_ABS        — absolute blue ceiling. Keeps pink/magenta out.
// RED_R_OVER_G         — red must beat green by this margin (out of 255).
//                        60 is strict enough to reject warm wood/carpet.
// RED_R_OVER_B         — red must beat blue by this margin.
// RED_MIN_SATURATION   — r must exceed (g+b)/2 by this much. Rejects grey.
// RED_MIN_NEIGHBORS    — how many of the 8 neighbors must also be red before
//                        a pixel is kept (morphological noise filter).
#define RED_MIN_R            80
#define RED_MAX_G_ABS        110
#define RED_MAX_B_ABS        110
#define RED_R_OVER_G         60
#define RED_R_OVER_B         55
#define RED_MIN_SATURATION   60
#define RED_MIN_NEIGHBORS    3

// ===== PER-PIXEL BLUE CLASSIFIER (tuned values) =====
#define BLUE_MIN_B            60
#define BLUE_B_OVER_R         40
#define BLUE_B_OVER_G         28

// ===== PER-PIXEL GREEN CLASSIFIER (tuned values) =====
#define GREEN_MIN_G           70
#define GREEN_G_OVER_R        20
#define GREEN_G_OVER_B        20

// ===== BALL SHAPE GATES =====
//
// BALL_MIN_RADIUS      — minimum card_avg (average cardinal radius from blob
//                        center in pixels). Keeps tiny noise clusters out.
//                        4 px = blob is at least ~8px wide = real ball from
//                        several feet. Do NOT raise this above 6 or you will
//                        miss the ball across the field.
// BALL_MIN_ASPECT      — bbox short/long ratio. 0.60 allows slightly oblong.
// BALL_MAX_SPREAD_FRAC — Gate4 fraction: max allowed (spread / total_avg).
//                        0.45 means the longest radius can be no more than
//                        ~1.45× the shortest. Tight enough to reject
//                        elongated ghosts, loose enough for real balls
//                        that are slightly squished at image corners.
// BALL_MIN_RATIO/MAX   — diag_avg / card_avg. Diagonal radii of a circle
//                        are ~0.707× cardinal, so the ratio should be
//                        near 0.70. We allow 0.55–1.20 for pixel-grid noise.
// BALL_MIN_CIRC/MAX    — count / (pi * card_avg^2). A filled disc = 1.0.
//                        Noise and edge effects push this up toward 1.3.
#define BALL_MIN_RADIUS       3
#define BALL_MIN_ASPECT       0.60f
#define BALL_MAX_SPREAD_FRAC  0.45f    // was broken: "/ 1" = always passes
#define BALL_MIN_RATIO        0.55f
#define BALL_MAX_RATIO        1.20f    // was 1.45 — tighter catches more non-circles
#define BALL_MIN_CIRC         0.50f
#define BALL_MAX_CIRC         1.35f    // was 1.40 — tighter
#define BALL_CENTROID_OFFSET  49       // max squared pixel offset bbox-ctr vs centroid

// ===== EDGE REJECTION =====
// Clusters touching the frame edge are either partially clipped (unreliable
// shape) or fragments of a large background feature that looks round where
// the frame cut it. Require at least this many pixels of inset on all sides.
#define BALL_EDGE_MARGIN      2

// ===== MINIMUM CLUSTER SIZE =====
// Passed from camera.cpp into detect_red_blob(). Documented here for
// reference: 50 pixels ≈ a ball ~10px wide = roughly 6–8 ft detection range
// depending on lens. Raising to 70 adds false-negative risk at long range;
// lowering to 30 adds false-positive risk from small red specks.
// The actual value is the argument to detect_red_blob() in camera.cpp.

// ============================================================================

#define BMP_BYTES ((VISION_FRAME_W * VISION_FRAME_H + 7) / 8)
#define MAX_CLUSTERS 16

typedef struct {
    uint32_t count;
    int16_t  min_x, max_x, min_y, max_y;
    int16_t  centroid_x, centroid_y;
} Cluster;

static Cluster s_clusters[MAX_CLUSTERS];
static int     s_num_clusters = 0;

static bool     s_found;
static uint32_t s_count;
static int16_t  s_min_x, s_max_x, s_min_y, s_max_y;
static int16_t  s_centroid_x, s_centroid_y;

static int32_t  s_sum_x, s_sum_y;

static uint8_t s_red[BMP_BYTES];
static uint8_t s_eroded[BMP_BYTES];
static uint8_t s_temp[BMP_BYTES];
static uint8_t s_vis[BMP_BYTES];

static inline void bmp_set(uint8_t* b, int x, int y) {
    int i = y * VISION_FRAME_W + x;
    b[i >> 3] |= (uint8_t)(1u << (i & 7));
}
static inline bool bmp_get(uint8_t* b, int x, int y) {
    if (x < 0 || x >= VISION_FRAME_W) return false;
    if (y < 0 || y >= VISION_FRAME_H) return false;
    int i = y * VISION_FRAME_W + x;
    return (b[i >> 3] >> (i & 7)) & 1u;
}

static inline bool is_red(uint8_t r, uint8_t g, uint8_t b) {
    if (r < RED_MIN_R)                      return false;
    if (g > RED_MAX_G_ABS)                  return false;
    if (b > RED_MAX_B_ABS)                  return false;
    if ((int)r < (int)g + RED_R_OVER_G)     return false;
    if ((int)r < (int)b + RED_R_OVER_B)     return false;
    int avg = ((int)g + (int)b) / 2;
    if ((int)r - avg < RED_MIN_SATURATION)  return false;
    return true;
}

static inline bool is_blue(uint8_t r, uint8_t g, uint8_t b) {
    if (b < BLUE_MIN_B)                   return false;
    if ((int)b < (int)r + BLUE_B_OVER_R)  return false;
    if ((int)b < (int)g + BLUE_B_OVER_G)  return false;
    return true;
}

static inline bool is_green(uint8_t r, uint8_t g, uint8_t b) {
    if (g < GREEN_MIN_G)                   return false;
    if ((int)g < (int)r + GREEN_G_OVER_R)  return false;
    if ((int)g < (int)b + GREEN_G_OVER_B)  return false;
    return true;
}

#define FLOOD_STACK 4096
static uint16_t s_flood_stack[FLOOD_STACK];

static uint32_t flood(int sx, int sy,
                      int16_t* mnx, int16_t* mxx,
                      int16_t* mny, int16_t* mxy)
{
    int sp = 0;
    s_flood_stack[sp++] = (uint16_t)((sy << 8) | sx);
    bmp_set(s_vis, sx, sy);

    uint32_t count = 0;
    *mnx = *mxx = (int16_t)sx;
    *mny = *mxy = (int16_t)sy;

    while (sp > 0) {
        uint16_t packed = s_flood_stack[--sp];
        int x = packed & 0xFF;
        int y = (packed >> 8) & 0xFF;

        count++;
        s_sum_x += x;
        s_sum_y += y;

        if (x < *mnx) *mnx = (int16_t)x;
        if (x > *mxx) *mxx = (int16_t)x;
        if (y < *mny) *mny = (int16_t)y;
        if (y > *mxy) *mxy = (int16_t)y;

        static const int dx[4] = { -1, 1,  0, 0 };
        static const int dy[4] = {  0, 0, -1, 1 };
        for (int k = 0; k < 4; k++) {
            int nx = x + dx[k];
            int ny = y + dy[k];
            if (!bmp_get(s_eroded, nx, ny)) continue;
            if ( bmp_get(s_vis,    nx, ny)) continue;

            bmp_set(s_vis, nx, ny);
            if (sp < FLOOD_STACK) {
                s_flood_stack[sp++] = (uint16_t)((ny << 8) | nx);
            }
        }
    }
    return count;
}

static int measure(int cx, int cy, int dx, int dy) {
    int r = 0;
    while (1) {
        int x = cx + dx * r;
        int y = cy + dy * r;
        if (!bmp_get(s_eroded, x, y)) break;
        r++;
    }
    return r;
}

int detect_color_blob(const uint8_t* frame, uint32_t min_count, TargetColor color)
{
    s_found        = false;
    s_count        = 0;
    s_min_x = s_max_x = s_min_y = s_max_y = 0;
    s_centroid_x = s_centroid_y = 0;
    s_num_clusters = 0;

    for (int i = 0; i < BMP_BYTES; i++) {
        s_red[i] = 0; s_eroded[i] = 0; s_temp[i] = 0; s_vis[i] = 0;
    }

    // Pass 1: classify pixels
    const uint8_t* p = frame;
    for (int y = 0; y < VISION_FRAME_H; y++) {
        for (int x = 0; x < VISION_FRAME_W; x++) {
            uint16_t px = ((uint16_t)p[0] << 8) | (uint16_t)p[1];
            p += 2;
            uint8_t r = (uint8_t)(((px >> 11) & 31) << 3);
            uint8_t g = (uint8_t)(((px >>  5) & 63) << 2);
            uint8_t b = (uint8_t)(( px        & 31) << 3);
            bool match;
            switch (color) {
                case TARGET_BLUE:  match = is_blue(r, g, b);  break;
                case TARGET_GREEN: match = is_green(r, g, b); break;
                case TARGET_RED:
                default:           match = is_red(r, g, b);   break;
            }
            if (match) bmp_set(s_red, x, y);
        }
    }

    // Pass 2: noise filter — keep only pixels with enough red neighbors
    for (int y = 0; y < VISION_FRAME_H; y++) {
        for (int x = 0; x < VISION_FRAME_W; x++) {
            if (!bmp_get(s_red, x, y)) continue;
            int n = 0;
            for (int dy = -1; dy <= 1; dy++) {
                for (int dx = -1; dx <= 1; dx++) {
                    if ((dx || dy) && bmp_get(s_red, x + dx, y + dy)) n++;
                }
            }
            if (n >= RED_MIN_NEIGHBORS) bmp_set(s_temp, x, y);
        }
    }

    // Pass 3: dilate to reconnect blobs thinned by the neighbor filter
    for (int y = 0; y < VISION_FRAME_H; y++) {
        for (int x = 0; x < VISION_FRAME_W; x++) {
            bool any = false;
            for (int dy = -1; dy <= 1 && !any; dy++) {
                for (int dx = -1; dx <= 1 && !any; dx++) {
                    if (bmp_get(s_temp, x + dx, y + dy)) any = true;
                }
            }
            if (any) bmp_set(s_eroded, x, y);
        }
    }

    // Pass 3b: morphological CLOSE (dilate then erode) fills small holes
    // inside detected blobs -- particularly specular highlights where
    // the bright spot on a glossy ball goes near-white and fails the
    // red-dominance checks. We already dilated above, so first dilate
    // one more time into s_temp (giving a 5x5 effective kernel), then
    // erode that back into s_eroded to fill gaps without growing the
    // outer boundary too aggressively.
    for (int i = 0; i < BMP_BYTES; i++) s_temp[i] = 0;
    for (int y = 0; y < VISION_FRAME_H; y++) {
        for (int x = 0; x < VISION_FRAME_W; x++) {
            bool any = false;
            for (int dy = -1; dy <= 1 && !any; dy++) {
                for (int dx = -1; dx <= 1 && !any; dx++) {
                    if (bmp_get(s_eroded, x + dx, y + dy)) any = true;
                }
            }
            if (any) bmp_set(s_temp, x, y);
        }
    }
    // Now erode s_temp back into s_eroded -- only set pixels where ALL
    // 8 neighbors are also set (true 3x3 erosion). Net effect: holes
    // up to ~3 px wide get filled, outer boundary stays roughly stable.
    for (int i = 0; i < BMP_BYTES; i++) s_eroded[i] = 0;
    for (int y = 0; y < VISION_FRAME_H; y++) {
        for (int x = 0; x < VISION_FRAME_W; x++) {
            bool all = true;
            for (int dy = -1; dy <= 1 && all; dy++) {
                for (int dx = -1; dx <= 1 && all; dx++) {
                    if (!bmp_get(s_temp, x + dx, y + dy)) all = false;
                }
            }
            if (all) bmp_set(s_eroded, x, y);
        }
    }

    // Pass 4: flood-fill connected components
    for (int y = 0; y < VISION_FRAME_H; y++) {
        for (int x = 0; x < VISION_FRAME_W; x++) {
            if (!bmp_get(s_eroded, x, y))  continue;
            if ( bmp_get(s_vis,    x, y))  continue;

            s_sum_x = 0; s_sum_y = 0;
            int16_t a, b, c, d;
            uint32_t ccount = flood(x, y, &a, &b, &c, &d);
            if (ccount < min_count) continue;

            if (s_num_clusters < MAX_CLUSTERS) {
                Cluster* cl = &s_clusters[s_num_clusters++];
                cl->count      = ccount;
                cl->min_x      = a; cl->max_x = b;
                cl->min_y      = c; cl->max_y = d;
                cl->centroid_x = (int16_t)(s_sum_x / (int32_t)ccount);
                cl->centroid_y = (int16_t)(s_sum_y / (int32_t)ccount);
            }
        }
    }

    return s_num_clusters > 0 ? 1 : 0;
}

uint32_t get_red_pixel_count(void) {
    return s_count;
}

int detect_red_blob(const uint8_t* frame, uint32_t min_count) {
    return detect_color_blob(frame, min_count, TARGET_RED);
}

static bool cluster_is_ball(const Cluster* cl, int16_t* r_out, float* c_out)
{
    int w = cl->max_x - cl->min_x + 1;
    int h = cl->max_y - cl->min_y + 1;

    printf("=== cluster check: count=%lu bbox=%d,%d to %d,%d (w=%d h=%d)\r\n",
           (unsigned long)cl->count,
           cl->min_x, cl->min_y, cl->max_x, cl->max_y, w, h);

    if (w <= 0 || h <= 0) {
        printf("  FAIL: degenerate bbox\r\n");
        return false;
    }

    // Gate 0: EDGE REJECTION
    if (cl->min_x <  BALL_EDGE_MARGIN ||
        cl->min_y <  BALL_EDGE_MARGIN ||
        cl->max_x >= VISION_FRAME_W - BALL_EDGE_MARGIN ||
        cl->max_y >= VISION_FRAME_H - BALL_EDGE_MARGIN)
    {
        printf("  FAIL: touches frame edge (bbox x=[%d..%d] y=[%d..%d])\r\n",
               cl->min_x, cl->max_x, cl->min_y, cl->max_y);
        return false;
    }

    // Gate 1: ASPECT RATIO — bbox must be roughly square
    int longer  = (w > h) ? w : h;
    int shorter = (w < h) ? w : h;
    float aspect = (float)shorter / (float)longer;
    printf("  Gate1 aspect: %.2f (need >= %.2f) %s\r\n",
           (double)aspect, (double)BALL_MIN_ASPECT,
           aspect < BALL_MIN_ASPECT ? "FAIL" : "pass");
    if (aspect < BALL_MIN_ASPECT) return false;

    // Gate 2: CENTROID vs BBOX CENTER
    int cx = (cl->min_x + cl->max_x) / 2;
    int cy = (cl->min_y + cl->max_y) / 2;
    int ddx = cx - cl->centroid_x;
    int ddy = cy - cl->centroid_y;
    int dist2 = ddx*ddx + ddy*ddy;
    printf("  Gate2 centroid: bbox_ctr=(%d,%d) centroid=(%d,%d) dist2=%d (max %d) %s\r\n",
           cx, cy, cl->centroid_x, cl->centroid_y, dist2, BALL_CENTROID_OFFSET,
           dist2 > BALL_CENTROID_OFFSET ? "FAIL" : "pass");
    if (dist2 > BALL_CENTROID_OFFSET) return false;

    // Gate 3: RADII — measure in 8 directions from bbox center
    int r_N  = measure(cx, cy,  0, -1);
    int r_S  = measure(cx, cy,  0,  1);
    int r_W  = measure(cx, cy, -1,  0);
    int r_E  = measure(cx, cy,  1,  0);
    int r_NE = measure(cx, cy,  1, -1);
    int r_NW = measure(cx, cy, -1, -1);
    int r_SE = measure(cx, cy,  1,  1);
    int r_SW = measure(cx, cy, -1,  1);

    int card_sum = r_N + r_S + r_W + r_E;
    int diag_sum = r_NE + r_NW + r_SE + r_SW;
    int card_avg = card_sum / 4;
    int diag_avg = diag_sum / 4;

    printf("  Gate3 radii: N=%d S=%d W=%d E=%d NE=%d NW=%d SE=%d SW=%d card_avg=%d diag_avg=%d\r\n",
           r_N, r_S, r_W, r_E, r_NE, r_NW, r_SE, r_SW, card_avg, diag_avg);

    if (card_avg < BALL_MIN_RADIUS) {
        printf("  FAIL: card_avg %d < %d\r\n", card_avg, BALL_MIN_RADIUS);
        return false;
    }
    if (diag_avg < BALL_MIN_RADIUS) {
        printf("  FAIL: diag_avg %d < %d\r\n", diag_avg, BALL_MIN_RADIUS);
        return false;
    }

    // Gate 4: SPREAD — (max_radius - min_radius) / total_avg <= BALL_MAX_SPREAD_FRAC
    // FIX: Previously used "/ BALL_RADII_SPREAD" where BALL_RADII_SPREAD = 1,
    // so spread_limit = total_avg, meaning ANY shape passed. Now uses a fraction.
    int all[8] = { r_N, r_S, r_W, r_E, r_NE, r_NW, r_SE, r_SW };
    int total_sum = card_sum + diag_sum;
    int total_avg = total_sum / 8;
    if (total_avg == 0) {
        printf("  FAIL: total_avg=0\r\n");
        return false;
    }

    // ------------------------------------------------------------------
    // Gate 4: SPREAD — How "uneven" the radii are.
    //
    // The camera produces horizontally-stretched images, so a real round
    // ball will have E/W radii larger than N/S radii by ~30%. We do NOT
    // simply look at max-min over all 8 radii — that would always fail
    // a stretched ball. Instead:
    //   - compute spread within the cardinal pair (N,S) and (E,W) separately
    //   - compute spread within the diagonal group (NE,NW,SE,SW)
    //   - check that opposite radii (N vs S, E vs W) are roughly equal
    //     (this is what "round" actually means — symmetric, not isotropic)
    //
    // This rejects crescents, bars, L-shapes, and asymmetric noise blobs
    // while accepting real balls under known horizontal stretch.
    // ------------------------------------------------------------------
    int ns_spread = (r_N > r_S) ? (r_N - r_S) : (r_S - r_N);
    int ew_spread = (r_E > r_W) ? (r_E - r_W) : (r_W - r_E);

    int diag_mn = all[4], diag_mx = all[4];   // NE,NW,SE,SW are indices 4-7
    for (int i = 5; i < 8; i++) {
        if (all[i] < diag_mn) diag_mn = all[i];
        if (all[i] > diag_mx) diag_mx = all[i];
    }
    int diag_spread = diag_mx - diag_mn;

    // Per-pair tolerance based on average radius. Small balls (~4 px)
    // get an absolute floor of 2 because pixel quantization alone can
    // produce that much variation. Larger balls scale linearly.
    float total_avg_f = (float)total_sum / 8.0f;
    int   pair_limit  = (int)(total_avg_f * 0.45f + 0.5f);
    if (pair_limit < 2) pair_limit = 2;

    bool symmetric = (ns_spread   <= pair_limit) &&
                     (ew_spread   <= pair_limit) &&
                     (diag_spread <= pair_limit + 1);

    printf("  Gate4 sym: NS=%d EW=%d DIAG=%d (limit %d, avg=%.1f) %s\r\n",
           ns_spread, ew_spread, diag_spread, pair_limit,
           (double)total_avg_f, symmetric ? "pass" : "FAIL");
    if (!symmetric) return false;

    // Gate 5: DIAG/CARD RATIO — diagonal radii should be ~0.707× cardinal
    float ratio = (float)diag_avg / (float)card_avg;
    printf("  Gate5 ratio: %.2f (need %.2f .. %.2f) %s\r\n",
           (double)ratio, (double)BALL_MIN_RATIO, (double)BALL_MAX_RATIO,
           (ratio < BALL_MIN_RATIO || ratio > BALL_MAX_RATIO) ? "FAIL" : "pass");
    if (ratio < BALL_MIN_RATIO) return false;
    if (ratio > BALL_MAX_RATIO) return false;

    // Gate 6: CIRCULARITY — pixel count vs expected disc area
    float disc_area = PI_F * (float)card_avg * (float)card_avg;
    float circ_val = (float)cl->count / disc_area;
    printf("  Gate6 circ: count=%lu disc_area=%.1f circ=%.2f (need %.2f .. %.2f) %s\r\n",
           (unsigned long)cl->count, (double)disc_area, (double)circ_val,
           (double)BALL_MIN_CIRC, (double)BALL_MAX_CIRC,
           (circ_val < BALL_MIN_CIRC || circ_val > BALL_MAX_CIRC) ? "FAIL" : "pass");
    if (circ_val < BALL_MIN_CIRC) return false;
    if (circ_val > BALL_MAX_CIRC) return false;

    printf("  >>> BALL PASS <<<\r\n");
    if (r_out) *r_out = (int16_t)card_avg;
    if (c_out) *c_out = circ_val;
    return true;
}

int verify_is_ball(int16_t* radius, float* circ)
{
    if (radius) *radius = 0;
    if (circ)   *circ   = 0.0f;

    for (int i = 0; i < s_num_clusters; i++) {
        int16_t r = 0;
        float   c = 0.0f;

        if (cluster_is_ball(&s_clusters[i], &r, &c)) {
            const Cluster* cl = &s_clusters[i];

            s_found      = true;
            s_count      = cl->count;
            s_min_x      = cl->min_x;
            s_max_x      = cl->max_x;
            s_min_y      = cl->min_y;
            s_max_y      = cl->max_y;
            s_centroid_x = cl->centroid_x;
            s_centroid_y = cl->centroid_y;

            if (radius) *radius = r;
            if (circ)   *circ   = c;
            return 1;
        }
    }
    return 0;
}

int get_blob_midpoint(int16_t* cx, int16_t* cy)
{
    if (!s_found) {
        if (cx) *cx = VISION_FRAME_W / 2;
        if (cy) *cy = VISION_FRAME_H / 2;
        return 0;
    }
    if (cx) *cx = (int16_t)((s_min_x + s_max_x) / 2);
    if (cy) *cy = (int16_t)((s_min_y + s_max_y) / 2);
    return 1;
}

// ===== BLACK LINE SIDE DETECTION =====
// Matches the Python validator: all three channels below BLACK_MAX_CHANNEL,
// bottom half of frame only, largest cluster >= BLACK_MIN_PIXELS,
// side determined by whether the centroid is left or right of frame center.
#define BLACK_MAX_CHANNEL  87
#define BLACK_MIN_PIXELS   30

LineSide detect_black_line_side(const uint8_t* frame)
{
    for (int i = 0; i < BMP_BYTES; i++) {
        s_red[i] = 0; s_eroded[i] = 0; s_temp[i] = 0; s_vis[i] = 0;
    }

    // Pass 1: classify black pixels in bottom half only
    for (int y = VISION_FRAME_H / 2; y < VISION_FRAME_H; y++) {
        const uint8_t* p = frame + y * VISION_FRAME_W * 2;
        for (int x = 0; x < VISION_FRAME_W; x++) {
            uint16_t pxv = ((uint16_t)p[0] << 8) | (uint16_t)p[1];
            p += 2;
            uint8_t r = (uint8_t)(((pxv >> 11) & 31) << 3);
            uint8_t g = (uint8_t)(((pxv >>  5) & 63) << 2);
            uint8_t b = (uint8_t)(( pxv        & 31) << 3);
            if (r < BLACK_MAX_CHANNEL && g < BLACK_MAX_CHANNEL && b < BLACK_MAX_CHANNEL)
                bmp_set(s_red, x, y);
        }
    }

    // Pass 2: neighbor filter — keep pixels with >= RED_MIN_NEIGHBORS dark neighbors
    for (int y = VISION_FRAME_H / 2; y < VISION_FRAME_H; y++) {
        for (int x = 0; x < VISION_FRAME_W; x++) {
            if (!bmp_get(s_red, x, y)) continue;
            int n = 0;
            for (int ny = -1; ny <= 1; ny++) {
                for (int nx = -1; nx <= 1; nx++) {
                    if ((nx || ny) && bmp_get(s_red, x + nx, y + ny)) n++;
                }
            }
            if (n >= RED_MIN_NEIGHBORS) bmp_set(s_temp, x, y);
        }
    }

    // Pass 3: dilate to reconnect thinned clusters
    for (int y = VISION_FRAME_H / 2 - 1; y < VISION_FRAME_H; y++) {
        for (int x = 0; x < VISION_FRAME_W; x++) {
            bool any = false;
            for (int ny = -1; ny <= 1 && !any; ny++) {
                for (int nx = -1; nx <= 1 && !any; nx++) {
                    if (bmp_get(s_temp, x + nx, y + ny)) any = true;
                }
            }
            if (any) bmp_set(s_eroded, x, y);
        }
    }

    // Pass 4: find largest connected component starting from bottom half
    int best_count = 0;
    int best_cx    = 0;

    for (int y = VISION_FRAME_H / 2; y < VISION_FRAME_H; y++) {
        for (int x = 0; x < VISION_FRAME_W; x++) {
            if (!bmp_get(s_eroded, x, y)) continue;
            if ( bmp_get(s_vis,    x, y)) continue;

            s_sum_x = 0; s_sum_y = 0;
            int16_t mnx, mxx, mny, mxy;
            uint32_t count = flood(x, y, &mnx, &mxx, &mny, &mxy);
            if ((int)count > best_count) {
                best_count = (int)count;
                best_cx    = (int)(s_sum_x / (int32_t)count);
            }
        }
    }

    if (best_count < BLACK_MIN_PIXELS) {
        printf("[line_side] no line (best_px=%d < %d)\n", best_count, BLACK_MIN_PIXELS);
        return LINE_SIDE_NONE;
    }

    LineSide side = (best_cx < VISION_FRAME_W / 2) ? LINE_SIDE_LEFT : LINE_SIDE_RIGHT;
    printf("[line_side] cx=%d count=%d -> %s\n",
           best_cx, best_count, side == LINE_SIDE_LEFT ? "LEFT" : "RIGHT");
    return side;
}

static void px(uint8_t* f, int x, int y, uint16_t color) {
    if (x < 0 || y < 0 || x >= VISION_FRAME_W || y >= VISION_FRAME_H) return;
    int i = (y * VISION_FRAME_W + x) * 2;
    f[i]     = (uint8_t)(color & 0xFF);
    f[i + 1] = (uint8_t)((color >> 8) & 0xFF);
}

void draw_bbox_overlay(uint8_t* f, uint16_t color) {
    if (!s_found) return;
    for (int x = s_min_x; x <= s_max_x; x++) {
        px(f, x, s_min_y, color);
        px(f, x, s_max_y, color);
    }
    for (int y = s_min_y; y <= s_max_y; y++) {
        px(f, s_min_x, y, color);
        px(f, s_max_x, y, color);
    }
}

void draw_midpoint_overlay(uint8_t* f, uint16_t color) {
    if (!s_found) return;
    int cx = (s_min_x + s_max_x) / 2;
    int cy = (s_min_y + s_max_y) / 2;
    for (int d = -3; d <= 3; d++) {
        px(f, cx + d, cy, color);
        px(f, cx, cy + d, color);
    }
}

void draw_centroid_overlay(uint8_t* f, uint16_t color) {
    if (!s_found) return;
    for (int d = -2; d <= 2; d++) {
        px(f, s_centroid_x + d, s_centroid_y, color);
        px(f, s_centroid_x, s_centroid_y + d, color);
    }
}

void draw_radii_overlay(uint8_t* f, uint16_t color) {
    if (s_num_clusters == 0) return;

    int best = 0;
    for (int i = 1; i < s_num_clusters; i++) {
        if (s_clusters[i].count > s_clusters[best].count) best = i;
    }
    const Cluster* cl = &s_clusters[best];

    int cx = (cl->min_x + cl->max_x) / 2;
    int cy = (cl->min_y + cl->max_y) / 2;

    int r_N  = measure(cx, cy,  0, -1);
    int r_S  = measure(cx, cy,  0,  1);
    int r_W  = measure(cx, cy, -1,  0);
    int r_E  = measure(cx, cy,  1,  0);
    int r_NE = measure(cx, cy,  1, -1);
    int r_NW = measure(cx, cy, -1, -1);
    int r_SE = measure(cx, cy,  1,  1);
    int r_SW = measure(cx, cy, -1,  1);

    for (int r = 1; r < r_N;  r++) px(f, cx,     cy - r, color);
    for (int r = 1; r < r_S;  r++) px(f, cx,     cy + r, color);
    for (int r = 1; r < r_W;  r++) px(f, cx - r, cy,     color);
    for (int r = 1; r < r_E;  r++) px(f, cx + r, cy,     color);
    for (int r = 1; r < r_NE; r++) px(f, cx + r, cy - r, color);
    for (int r = 1; r < r_NW; r++) px(f, cx - r, cy - r, color);
    for (int r = 1; r < r_SE; r++) px(f, cx + r, cy + r, color);
    for (int r = 1; r < r_SW; r++) px(f, cx - r, cy + r, color);

    px(f, cx, cy, 0xFFE0);
}