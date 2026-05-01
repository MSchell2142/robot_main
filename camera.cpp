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
    static uint32_t dbg_no_clusters = 0;
    static uint32_t dbg_no_ball = 0;
    static uint32_t dbg_ball = 0;
    dbg_calls++;

    myCAM.takePicture(CAM_IMAGE_MODE_128X128, CAM_IMAGE_PIX_FMT_RGB565);

    uint32_t total = myCAM.getTotalLength();
    if (total == 0 || total > (uint32_t)(FRAME_BYTES * 2)) {
        dbg_bad_total++;
        if ((dbg_calls % 20) == 0) {
            printf("[cam-stat] calls=%lu bad_total=%lu bad_read=%lu no_cluster=%lu no_ball=%lu ball=%lu (last total=%lu)\n",
                   (unsigned long)dbg_calls, (unsigned long)dbg_bad_total,
                   (unsigned long)dbg_bad_read, (unsigned long)dbg_no_clusters,
                   (unsigned long)dbg_no_ball, (unsigned long)dbg_ball,
                   (unsigned long)total);
        }
        return CAM_NONE;
    }

    if (!read_frame(cam, frame_buf, total, chunk)) {
        dbg_bad_read++;
        return CAM_NONE;
    }

    int has_clusters = detect_red_blob(frame_buf, 50);
    if (!has_clusters) dbg_no_clusters++;

    int16_t radius = 0;
    float circ = 0.0f;
    int is_ball = verify_is_ball(&radius, &circ);

    if (has_clusters && !is_ball) dbg_no_ball++;
    if (is_ball) dbg_ball++;

    if ((dbg_calls % 20) == 0) {
        printf("[cam-stat] calls=%lu bad_total=%lu bad_read=%lu no_cluster=%lu no_ball=%lu ball=%lu\n",
               (unsigned long)dbg_calls, (unsigned long)dbg_bad_total,
               (unsigned long)dbg_bad_read, (unsigned long)dbg_no_clusters,
               (unsigned long)dbg_no_ball, (unsigned long)dbg_ball);
    }

    int16_t cx = FRAME_W / 2;
    int16_t cy = FRAME_H / 2;
    get_blob_midpoint(&cx, &cy);

    uint8_t direction = CAM_NONE;

    if (is_ball) {
        int mid = FRAME_W / 2;
        int err = (int)cx - mid;   // negative = left of center, positive = right

        if      (err < -CAM_LOOSE_TOL)                          direction = CAM_LEFT;
        else if (err >  CAM_LOOSE_TOL)                          direction = CAM_RIGHT;
        else if (err >= -CAM_TIGHT_TOL && err <= CAM_TIGHT_TOL) direction = CAM_CENTER_TIGHT;
        else                                                     direction = CAM_CENTER_LOOSE;
    }

    return direction;
}