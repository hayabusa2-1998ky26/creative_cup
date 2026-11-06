#include "ev3api.h"
#include "pybricks_ev3rt.h"
#include "programs.h"
#include <stdio.h>
#include <math.h>

extern pybricks::DriveBase* robot;
extern pybricks::Motor* left_motor;
extern pybricks::Motor* right_motor;
extern pybricks::ColorSensor* left_sensor;
extern pybricks::ColorSensor* right_sensor;

namespace {
    const float test_speed = 150.0f;
    volatile int selected_test = 0;
    volatile bool finished = false;
    const char* const names[] = {"1: FORWARD ALIGN", "2: FORWARD 3 LINES", "3: REVERSE 3 LINES"};

    void release_buttons() {
        while (ev3_button_is_pressed(ENTER_BUTTON) || ev3_button_is_pressed(UP_BUTTON) ||
               ev3_button_is_pressed(DOWN_BUTTON) || ev3_button_is_pressed(LEFT_BUTTON)) {
            tslp_tsk(10000);
        }
    }

    void show_values(const char* title, int32_t start_l, int32_t start_r) {
        int raw_l = left_sensor->reflection();
        int raw_r = right_sensor->reflection();
        float norm_l = left_sensor->normalized_reflection();
        float norm_r = right_sensor->normalized_reflection();
        int32_t moved_l = left_motor->angle() - start_l;
        int32_t moved_r = right_motor->angle() - start_r;
        char raw[32], norm[32], enc[32], threshold[32];
        snprintf(raw, sizeof(raw), "Raw L:%d R:%d", raw_l, raw_r);
        snprintf(norm, sizeof(norm), "Norm %.2f %.2f", norm_l, norm_r);
        snprintf(enc, sizeof(enc), "Deg %ld %ld", (long)moved_l, (long)moved_r);
        snprintf(threshold, sizeof(threshold), "Target %.2f", pybricks::DriveConfig::target_val);
        ev3_lcd_fill_rect(0, 0, EV3_LCD_WIDTH, EV3_LCD_HEIGHT, EV3_LCD_WHITE);
        ev3_lcd_draw_string(title, 0, 0);
        ev3_lcd_draw_string(raw, 0, 20);
        ev3_lcd_draw_string(norm, 0, 40);
        ev3_lcd_draw_string(enc, 0, 60);
        ev3_lcd_draw_string(threshold, 0, 80);
        ev3_lcd_draw_string("LEFT:STOP ENTER:GO", 0, 100);
    }
}

extern "C" void line_test_task(intptr_t unused) {
    if (selected_test == 0) {
        // Same strict alignment option as the first straight in run_main_course().
        robot->straight_line(330.0f, test_speed, -1.0f, false, false, false, false, true);
    } else {
        int count = selected_test == 1 ? 3 : -3;
        robot->straight_lines(count, 720.0f, pybricks::Opt().speed(test_speed));
    }
    finished = true;
    ext_tsk();
}

void run_line_test() {
    robot->set_sensor_coordinates(72.0f, 48.0f);
    ev3_lcd_set_font(EV3_FONT_MEDIUM);
    release_buttons();
    while (true) {
        robot->brake();
        int32_t start_l = left_motor->angle(), start_r = right_motor->angle();
        // Stationary sensor monitor; UP/DOWN chooses the production routine to test.
        while (!ev3_button_is_pressed(ENTER_BUTTON)) {
            if (ev3_button_is_pressed(LEFT_BUTTON)) return;
            if (ev3_button_is_pressed(UP_BUTTON)) {
                selected_test = (selected_test + 2) % 3;
                release_buttons();
            }
            if (ev3_button_is_pressed(DOWN_BUTTON)) {
                selected_test = (selected_test + 1) % 3;
                release_buttons();
            }
            show_values(names[selected_test], start_l, start_r);
            tslp_tsk(100000);
        }
        release_buttons();
        start_l = left_motor->angle();
        start_r = right_motor->angle();
        int32_t prev_l = start_l, prev_r = start_r;
        float travel_mm = 0.0f;
        const float mm_per_deg = pybricks::DriveConfig::wheel_diameter * (float)M_PI / 360.0f;
        SYSTIM start_time;
        get_tim(&start_time);
        finished = false;
        const char* result = "DONE";
        if (act_tsk(LINE_TEST_TASK) != E_OK) {
            result = "TASK START ERROR";
            finished = true;
        }
        int frames = 0;
        while (!finished) {
            int32_t cur_l = left_motor->angle(), cur_r = right_motor->angle();
            travel_mm += (fabsf((float)(cur_l - prev_l)) + fabsf((float)(cur_r - prev_r))) * 0.5f * mm_per_deg;
            prev_l = cur_l;
            prev_r = cur_r;
            SYSTIM now;
            get_tim(&now);
            if (ev3_button_is_pressed(LEFT_BUTTON)) { result = "STOPPED"; break; }
            if (now - start_time >= 20000000U) { result = "TIMEOUT"; break; }
            if (travel_mm >= 1200.0f) { result = "DISTANCE LIMIT"; break; }
            if (frames++ % 5 == 0) show_values("RUN / LEFT:STOP", start_l, start_r);
            tslp_tsk(20000);
        }
        if (!finished) ter_tsk(LINE_TEST_TASK);
        robot->brake();
        tslp_tsk(100000);
        show_values(result, start_l, start_r);
        ev3_speaker_play_tone(1000, 100);
        release_buttons();
        // Keep the result on screen until explicitly returning to the menu.
        while (!ev3_button_is_pressed(ENTER_BUTTON)) {
            if (ev3_button_is_pressed(LEFT_BUTTON)) return;
            tslp_tsk(10000);
        }
        release_buttons();
    }
}
