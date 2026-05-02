#include <cstdio>
#include <cstdint>
#include <cmath>

#include "pico/stdlib.h"

#include "camera.h"
#include "encoders.h"
#include "vision.h"
#include "field.h"
#include "sonar.h"

// ============================================================================
// MATCH TIMER
// ============================================================================
static const uint32_t MATCH_DURATION_MS     = 5u * 60u * 1000u;   // 5 min
static const uint32_t HOME_RETURN_BUDGET_MS = 20u * 1000u;        // last 20 s

static uint32_t s_match_start_ms = 0;

static uint32_t match_elapsed_ms() {
    return to_ms_since_boot(get_absolute_time()) - s_match_start_ms;
}
static bool match_time_up() {
    return MATCH_DURATION_MS > 0 &&
           match_elapsed_ms() >= MATCH_DURATION_MS;
}
static bool match_return_now() {
    return MATCH_DURATION_MS > 0 &&
           match_elapsed_ms() >= (MATCH_DURATION_MS - HOME_RETURN_BUDGET_MS);
}

// ============================================================================
// CHASE TUNING
// ============================================================================
static const float DEG_PER_PIXEL = 0.55f;
static const float TURN_GAIN     = 0.6f;
static const float MAX_TURN_DEG  = 15.0f;
static const int   TIGHT_CONFIRM = 2;

// ============================================================================
// STATE MACHINE
// ============================================================================
enum State {
    ST_COVERAGE,
    ST_CHASING,
    ST_DELIVERING,
    ST_RETURN_HOME,
    ST_DONE,
};

static const char* state_name(State s) {
    switch (s) {
        case ST_COVERAGE:    return "COVERAGE";
        case ST_CHASING:     return "CHASING";
        case ST_DELIVERING:  return "DELIVERING";
        case ST_RETURN_HOME: return "RETURN_HOME";
        case ST_DONE:        return "DONE";
    }
    return "?";
}

// ============================================================================
// CHASE SUB-BEHAVIOR
// ============================================================================
static bool chase_ball_once()
{
    const int LOST_LIMIT = 3;

    int tight_streak = 0;
    int lost_count   = 0;

    while (true) {
        mpu_update();
        position_update();

        if (match_return_now()) return false;

        uint8_t raw = get_camera_direction();
        uint8_t dir = raw & 0x0F;

        int16_t cx = VISION_FRAME_W / 2, cy = 0;
        get_blob_midpoint(&cx, &cy);
        int pixel_err = (int)cx - (VISION_FRAME_W / 2);

        switch (dir) {
            case CAM_LEFT:
            case CAM_RIGHT: {
                lost_count   = 0;
                tight_streak = 0;

                float turn_deg = (float)pixel_err * DEG_PER_PIXEL * TURN_GAIN;
                if (turn_deg >  MAX_TURN_DEG) turn_deg =  MAX_TURN_DEG;
                if (turn_deg < -MAX_TURN_DEG) turn_deg = -MAX_TURN_DEG;

                printf("  chase %s err=%+4d -> turn %+.1f\n",
                       (dir == CAM_LEFT ? "L" : "R"), pixel_err, (double)turn_deg);

                rotate_to(fused_heading - turn_deg);
                break;
            }

            case CAM_CENTER_LOOSE: {
                lost_count   = 0;
                tight_streak = 0;
                printf("  chase LOOSE err=%+4d -> drive until lost\n", pixel_err);
                drive_forward_until_lost();
                return true;
            }

            case CAM_CENTER_TIGHT: {
                lost_count = 0;
                tight_streak++;
                printf("  chase TIGHT err=%+4d streak=%d\n", pixel_err, tight_streak);
                if (tight_streak >= TIGHT_CONFIRM) {
                    drive_forward_until_lost();
                    return true;
                }
                break;
            }

            case CAM_NONE:
            default: {
                tight_streak = 0;
                lost_count++;
                printf("  chase LOST (%d/%d)\n", lost_count, LOST_LIMIT);
                if (lost_count >= LOST_LIMIT) return false;
                drive_distance(2.0f);
                break;
            }
        }

        sleep_ms(40);
    }
}

// ============================================================================
// DELIVERY SUB-BEHAVIOR
// ============================================================================
static void deliver_ball()
{
    printf("[deliver] (stub) collected ball, resuming coverage from (%.1f, %.1f) head=%.1f\n",
           (double)get_pos_x_in(), (double)get_pos_y_in(),
           (double)fused_heading);
    stop();
    sleep_ms(200);
}

// ============================================================================
// DISPENSE SHAKE
// ============================================================================
// After parking above the goal and rotating backside-toward-goal, do a
// settle-then-wiggle sequence to dislodge any balls stuck against each
// other inside the robot's hopper. The 3-second settle lets balls roll
// to a stable position; the 5-second wiggle alternately spins one wheel
// forward and the other backward at moderate PWM, rocking the robot
// in place so internal friction breaks loose.
//
// Wiggle does NOT use rotate_to() because we don't care about ending
// at a precise heading — we just want mechanical agitation. Direct
// motor commands give cleaner timing.
static void dispense_shake() {
    printf("[dispense] settling 3s...\n");
    stop();
    sleep_ms(3000);

    printf("[dispense] shaking 5s to dislodge balls...\n");
    const int16_t  WIGGLE_PWM    = 350;
    const uint32_t WIGGLE_HALF_MS = 250;   // 250 ms each direction = 4 Hz
    const uint32_t WIGGLE_TOTAL_MS = 5000;
    uint32_t t0 = to_ms_since_boot(get_absolute_time());
    bool left = true;
    while (to_ms_since_boot(get_absolute_time()) - t0 < WIGGLE_TOTAL_MS) {
        if (left) drive(-WIGGLE_PWM, +WIGGLE_PWM);  // pivot left
        else      drive(+WIGGLE_PWM, -WIGGLE_PWM);  // pivot right
        sleep_ms(WIGGLE_HALF_MS);
        left = !left;
    }
    stop();
    printf("[dispense] done\n");
}

// ============================================================================
// MAIN
// ============================================================================
int main()
{
    stdio_init_all();
    while (!stdio_usb_connected()) sleep_ms(100);
    sleep_ms(200);

    printf("[init] encoders...\n");
    init_encoders();

    // encoder_id_test();       // hand-spin diagnostic; uncomment if encoders need re-verifying
    // encoder1_test();         // focused 3-phase test for encoder 1 only; uncomment to run
    // encoder_rotation_test(); // spin one full turn, read count = true PPR; uncomment to run

    printf("[init] motors...\n");
    init_motors();
    // motor_balance_cal();     // measure raw M2/M1 ratio; re-comment when done
    // motor_power_test();      // re-enable once PPR is calibrated

    printf("[init] MPU...\n");
    init_mpu();

    // drive_forward_test();    // drive 24 in, check enc counts + tape measure; uncomment to run

    // printf("[init] camera...\n");      // camera disabled during navigation testing
    // init_camera();
    // set_camera_mode(CAM_MODE_RED);

    // ----- MAG NOISE TEST (Stage 2 validation) -----
    // Hold the robot OFF the ground when this runs. Comment out
    // when no longer needed -- this only runs at startup.
    // mag_noise_test();   // PASS validated -- enable to re-test if mag chip is moved

    printf("[init] sonar...\n");
    init_sonar();
    // while (true) { sonar_validation_test(); }  // sonar validated -- re-enable if sensors are moved

    printf("[init] field geometry...\n");
    g_home_side = HOME_SIDE_DEFAULT;
    // Auto-detect home side from black boundary line.
    // Verify LINE_SIDE_LEFT -> HOME_RIGHT (or swap) on the actual field before enabling.
    // LineSide start_side = camera_detect_start_side();
    // if (start_side != LINE_SIDE_NONE)
    //     g_home_side = (start_side == LINE_SIDE_LEFT) ? HOME_RIGHT : HOME_LEFT;
    // printf("[init] start_side=%s -> home=%s\n",
    //        start_side == LINE_SIDE_LEFT ? "LEFT" : (start_side == LINE_SIDE_RIGHT ? "RIGHT" : "NONE"),
    //        g_home_side == HOME_LEFT ? "LEFT" : "RIGHT");
    set_pose(field_home_x_in(), field_home_y_in(), field_start_heading_deg());
    field_init();

    // mag_auto_cal();           // offsets now hardcoded in MAG_OFFSET_X/Y -- re-enable only if mag chip is moved

    printf("[init] done -- all systems go\n");

    // drive_raw_test();              // hard+soft correction kick test -- re-enable when needed
    // while (true) { distance_calibration_test(); }  // calibrated: WHEEL_RADIUS=0.034313 m

    s_match_start_ms = to_ms_since_boot(get_absolute_time());

    State state = ST_COVERAGE;
    int   collected_this_match = 0;
    printf("\n=== MATCH START (HOME_%s) ===\n",
           (g_home_side == HOME_LEFT) ? "LEFT" : "RIGHT");

    while (true) {
        mpu_update();
        position_update();

        // Global: match-end handling
        if (match_time_up()) {
            if (state != ST_DONE) {
                printf("[match] TIME UP -- stopping. Collected=%d\n",
                       collected_this_match);
                stop();
                state = ST_DONE;
            }
            sleep_ms(100);
            continue;
        }
        if (match_return_now() && state != ST_RETURN_HOME && state != ST_DONE) {
            printf("[match] return window -- switching to RETURN_HOME\n");
            state = ST_RETURN_HOME;
        }

        switch (state) {

            case ST_COVERAGE: {
                float wx, wy;
                if (!field_next_waypoint(&wx, &wy)) {
                    // Should be unreachable: field_advance_waypoint auto-wraps,
                    // so the iterator is never exhausted. Defensive only.
                    printf("[coverage] no waypoint -> reset\n");
                    field_reset_coverage();
                    continue;
                }

                bool is_boundary   = field_current_is_phase_boundary();
                bool is_checkpoint = field_current_is_scan_checkpoint();
                int  wp_idx        = field_current_waypoint_index();
                int  wp_total      = field_waypoint_count();

                printf("\n[COVERAGE] WP %d/%d -> (%.1f, %.1f)  pos=(%.1f, %.1f)  head=%.1f%s%s\n",
                       wp_idx, wp_total,
                       (double)wx, (double)wy,
                       (double)get_pos_x_in(), (double)get_pos_y_in(),
                       (double)fused_heading,
                       is_boundary   ? "  [phase boundary]"  : "",
                       is_checkpoint ? "  [scan checkpoint]" : "");

                // Drive to waypoint without mid-run scanning. The overhead of
                // camera SPI during driving was causing stalls, so camera reads
                // are deferred to scan checkpoints where the robot is stopped.
                go_to_xy_approx(wx, wy, field_current_tolerance_in());

                // Camera disabled during navigation testing -- re-enable when lawnmower is validated.
                // bool ball_seen = false;
                // if (is_checkpoint) {
                //     uint8_t dir = get_camera_direction();
                //     if (dir != CAM_NONE) {
                //         ball_seen = true;
                //         printf("  [coverage] ball seen at checkpoint -> chasing\n");
                //     }
                // }
                // if (ball_seen) {
                //     state = ST_CHASING;
                // } else {
                if (is_boundary) {
                    float final_h = field_final_heading_deg();
                    printf("[coverage] phase boundary - rotating to %.1f\n",
                           (double)final_h);
                    rotate_to(final_h);
                    dispense_shake();
                }
                field_advance_waypoint();
                if (g_sonar_skip_next) {
                    printf("[coverage] sonar blocked -- skipping next waypoint\n");
                    field_advance_waypoint();
                    g_sonar_skip_next = false;
                }
                break;
            }

            case ST_CHASING: {
                printf("\n[CHASING] start, pos=(%.1f, %.1f)\n",
                       (double)get_pos_x_in(), (double)get_pos_y_in());

                bool collected = chase_ball_once();

                if (collected) {
                    collected_this_match++;
                    printf("[CHASING] collected! total=%d\n", collected_this_match);
                    state = ST_DELIVERING;
                } else {
                    printf("[CHASING] lost the ball, back to coverage\n");
                    state = ST_COVERAGE;
                }
                break;
            }

            case ST_DELIVERING: {
                printf("\n[DELIVERING] start\n");
                deliver_ball();
                state = ST_COVERAGE;
                break;
            }

            case ST_RETURN_HOME: {
                printf("\n[RETURN_HOME] -> park at (%.1f, %.1f)\n",
                       (double)field_park_x_in(),
                       (double)field_park_y_in());
                go_to_xy(field_park_x_in(), field_park_y_in());
                rotate_to(field_final_heading_deg());
                dispense_shake();
                stop();
                state = ST_DONE;
                break;
            }

            case ST_DONE:
            default:
                stop();
                sleep_ms(100);
                break;
        }
    }
}