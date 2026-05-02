#include <cstdio>
#include <cstring>
#include <cstdint>

#include "pico/stdlib.h"
#include "hardware/gpio.h"

#include "Arducam_Mega.h"
#include "Arducam/ArducamCamera.h"
#include "vision.h"
#include "camera.h"

// ===== CONFIG =====
static const int CS_PIN  = 17;

// How long to wait for myCAM.begin() before giving up (ms).
// begin() can hang indefinitely if the SPI bus or camera hardware
// is not responding. This lets the robot continue (without camera)
// rather than freezing at startup.
static const uint32_t CAM_INIT_TIMEOUT_MS = 3000;

#define FRAME_W      VISION_FRAME_W
#define FRAME_H      VISION_FRAME_H
#define FRAME_BYTES  (FRAME_W * FRAME_H * 2)

Arducam_Mega myCAM(CS_PIN);
static bool s_cam_ok = false;
static CameraMode s_cam_mode = CAM_MODE_RED;

void set_camera_mode(CameraMode mode) {
    s_cam_mode = mode;
    printf("[camera] mode = %s\n",
           mode == CAM_MODE_RED ? "RED" : "BLUE+GREEN");
}

// ===== INIT =====
void init_camera()
{
    printf("[camera] init start (CS=GPIO%d, timeout=%lums)...\n",
           CS_PIN, (unsigned long)CAM_INIT_TIMEOUT_MS);

    // Arducam_Mega::begin() has no timeout — it can spin forever if the
    // SPI bus is broken or the camera module is missing/miswired.
    // We time-box it by checking elapsed time.  Because begin() is
    // blocking inside the library, we can't interrupt it mid-call, but
    // we can at least detect that it returned and how long it took.
    uint32_t t0 = to_ms_since_boot(get_absolute_time());
    myCAM.begin();
    uint32_t elapsed = to_ms_since_boot(get_absolute_time()) - t0;

    if (elapsed >= CAM_INIT_TIMEOUT_MS) {
        // begin() took suspiciously long — SPI likely not responding.
        printf("[camera] WARN: begin() took %lums (>= timeout %lums) "
               "— camera may be unavailable\n",
               (unsigned long)elapsed, (unsigned long)CAM_INIT_TIMEOUT_MS);
        s_cam_ok = false;
    } else {
        printf("[camera] begin() returned in %lums — OK\n",
               (unsigned long)elapsed);
        s_cam_ok = true;
    }
}

// ===== FRAME READ =====
static bool read_frame(ArducamCamera* cam,
                       uint8_t* frame_buf,
                       uint32_t total,
                       uint8_t* chunk)
{
    // Hard wall-clock cap on a single frame read. Camera SPI has been
    // observed to occasionally hang indefinitely on readBuff() if the
    // bus desyncs. Anything longer than 200 ms means we're stuck and
    // it's safer to abort than to freeze the drive loop.
    const uint32_t FRAME_READ_TIMEOUT_MS = 200;
    uint32_t t_start = to_ms_since_boot(get_absolute_time());

    uint32_t bytes_read = 0;
    uint32_t fails = 0;

    while (bytes_read < total && bytes_read < FRAME_BYTES) {
        if (to_ms_since_boot(get_absolute_time()) - t_start > FRAME_READ_TIMEOUT_MS) {
            printf("[camera] WARN: read_frame timeout (got %lu/%lu bytes)\n",
                   (unsigned long)bytes_read, (unsigned long)total);
            return false;
        }

        uint32_t remaining = total - bytes_read;
        uint8_t size = (remaining > 200) ? 200 : (uint8_t)remaining;

        uint32_t got = readBuff(cam, chunk, size);
        if (got == 0) {
            if (++fails > 10) return false;
            continue;
        }
        fails = 0;

        uint32_t copyLen = got;
        if (bytes_read + copyLen > FRAME_BYTES) copyLen = FRAME_BYTES - bytes_read;
        memcpy(&frame_buf[bytes_read], chunk, copyLen);
        bytes_read += got;
    }
    return true;
}

LineSide camera_detect_start_side()
{
    if (!s_cam_ok) {
        printf("[camera] detect_start_side: camera not OK\n");
        return LINE_SIDE_NONE;
    }

    static ArducamCamera* cam = myCAM.getCameraInstance();
    static uint8_t frame_buf[FRAME_BYTES];
    static uint8_t chunk[200];

    myCAM.takePicture(CAM_IMAGE_MODE_128X128, CAM_IMAGE_PIX_FMT_RGB565);
    uint32_t total = myCAM.getTotalLength();
    if (total == 0 || total > (uint32_t)(FRAME_BYTES * 2)) {
        printf("[camera] detect_start_side: bad frame size %lu\n", (unsigned long)total);
        return LINE_SIDE_NONE;
    }
    if (!read_frame(cam, frame_buf, total, chunk)) {
        printf("[camera] detect_start_side: read_frame failed\n");
        return LINE_SIDE_NONE;
    }
    return detect_black_line_side(frame_buf);
}

// ===== MAIN CAMERA FUNCTION =====
uint8_t get_camera_direction()
{
    if (!s_cam_ok) return CAM_NONE;

    static ArducamCamera* cam = myCAM.getCameraInstance();
    static uint8_t frame_buf[FRAME_BYTES];
    static uint8_t chunk[200];

    static uint32_t dbg_calls = 0;
    static uint32_t dbg_bad_total = 0;
    static uint32_t dbg_bad_read = 0;
    static uint32_t dbg_no_ball = 0;
    static uint32_t dbg_ball = 0;
    dbg_calls++;

    myCAM.takePicture(CAM_IMAGE_MODE_128X128, CAM_IMAGE_PIX_FMT_RGB565);

    uint32_t total = myCAM.getTotalLength();
    if (total == 0 || total > (uint32_t)(FRAME_BYTES * 2)) {
        dbg_bad_total++;
        if ((dbg_calls % 20) == 0) {
            printf("[cam-stat] calls=%lu bad_total=%lu bad_read=%lu no_ball=%lu ball=%lu (last total=%lu)\n",
                   (unsigned long)dbg_calls, (unsigned long)dbg_bad_total,
                   (unsigned long)dbg_bad_read,
                   (unsigned long)dbg_no_ball, (unsigned long)dbg_ball,
                   (unsigned long)total);
        }
        return CAM_NONE;
    }

    if (!read_frame(cam, frame_buf, total, chunk)) {
        dbg_bad_read++;
        return CAM_NONE;
    }

    // Build candidate list based on current mode.
    //   RED mode:        check red only      (1 detection pass)
    //   BLUE+GREEN mode: check blue + green  (2 detection passes)
    // For multi-candidate modes we pick the candidate closest to frame
    // center -- that's the ball we should chase first.
    struct { TargetColor color; uint8_t flag; bool seen; int16_t cx; int16_t err; } cand[2];
    int n_cand;
    if (s_cam_mode == CAM_MODE_RED) {
        cand[0] = { TARGET_RED, CAM_COLOR_RED, false, 0, 0 };
        n_cand = 1;
    } else {
        cand[0] = { TARGET_BLUE,  CAM_COLOR_BLUE,  false, 0, 0 };
        cand[1] = { TARGET_GREEN, CAM_COLOR_GREEN, false, 0, 0 };
        n_cand = 2;
    }

    int mid = FRAME_W / 2;
    for (int i = 0; i < n_cand; i++) {
        if (!detect_color_blob(frame_buf, 100, cand[i].color)) continue;

        int16_t r_px = 0;
        float c_val = 0.0f;
        if (!verify_is_ball(&r_px, &c_val)) continue;

        int16_t cx = (int16_t)mid, cy = FRAME_H / 2;
        get_blob_midpoint(&cx, &cy);
        cand[i].seen = true;
        cand[i].cx   = cx;
        cand[i].err  = (int16_t)((int)cx - mid);
    }

    int pick = -1;
    int best_err = 32767;
    for (int i = 0; i < n_cand; i++) {
        if (!cand[i].seen) continue;
        int e = cand[i].err < 0 ? -cand[i].err : cand[i].err;
        if (e < best_err) { best_err = e; pick = i; }
    }

    if (pick < 0) {
        dbg_no_ball++;
        if ((dbg_calls % 20) == 0) {
            printf("[cam-stat] calls=%lu bad_total=%lu bad_read=%lu no_ball=%lu ball=%lu\n",
                   (unsigned long)dbg_calls, (unsigned long)dbg_bad_total,
                   (unsigned long)dbg_bad_read,
                   (unsigned long)dbg_no_ball, (unsigned long)dbg_ball);
        }
        return CAM_NONE;
    }
    dbg_ball++;

    // For BLUE+GREEN mode, the second detect_color_blob() overwrote module
    // state. Re-run for the picked color so s_found / bbox / centroid match
    // the chosen ball (callers like main.cpp's chase loop read that state
    // via get_blob_midpoint()).
    if (n_cand > 1) {
        detect_color_blob(frame_buf, 100, cand[pick].color);
        verify_is_ball(NULL, NULL);
    }

    if ((dbg_calls % 20) == 0) {
        printf("[cam-stat] calls=%lu bad_total=%lu bad_read=%lu no_ball=%lu ball=%lu\n",
               (unsigned long)dbg_calls, (unsigned long)dbg_bad_total,
               (unsigned long)dbg_bad_read,
               (unsigned long)dbg_no_ball, (unsigned long)dbg_ball);
    }

    int err = cand[pick].err;
    uint8_t direction;
    if      (err < -CAM_LOOSE_TOL)                          direction = CAM_LEFT;
    else if (err >  CAM_LOOSE_TOL)                          direction = CAM_RIGHT;
    else if (err >= -CAM_TIGHT_TOL && err <= CAM_TIGHT_TOL) direction = CAM_CENTER_TIGHT;
    else                                                     direction = CAM_CENTER_LOOSE;

    return direction | cand[pick].flag;
}