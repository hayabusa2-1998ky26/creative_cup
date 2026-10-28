#include "ev3api.h"
#include "pybricks_ev3rt.h"
#include "programs.h"

extern pybricks::Motor* left_motor;
extern pybricks::Motor* right_motor;

void run_ff_calibration() {
    ulong_t sleep_us = (ulong_t)(pybricks::DriveConfig::period_ms * 1000.0f); 

    ev3_lcd_set_font(EV3_FONT_MEDIUM);
    ev3_lcd_fill_rect(0, 0, EV3_LCD_WIDTH, EV3_LCD_HEIGHT, EV3_LCD_WHITE);
    tslp_tsk(sleep_us); 

    ev3_lcd_draw_string("FF Calibration", 0, 0);
    tslp_tsk(sleep_us);

    ev3_lcd_draw_string("Running for 3 sec...", 0, 20);
    tslp_tsk(sleep_us);

    left_motor->reset_angle(0);
    right_motor->reset_angle(0);
    tslp_tsk(sleep_us); 

    int test_speed = 300; 
    left_motor->run(test_speed);
    right_motor->run(test_speed);

    int elapsed_ms = 0;
    while (elapsed_ms < 3000) {
        tslp_tsk(sleep_us);
        elapsed_ms += (int)pybricks::DriveConfig::period_ms;
    }

    ev3_speaker_play_tone(440, 100);
    tslp_tsk(100 * 1000); // 音が鳴り終わるまで待機

    left_motor->brake();
    right_motor->brake();
    
    ev3_speaker_play_tone(880, 100);
    tslp_tsk(100 * 1000); // 音が鳴り終わるまで待機

    int32_t l_angle = left_motor->angle();
    int32_t r_angle = right_motor->angle();
    
    float ff_ratio = 1.0f;
    if (r_angle != 0) {
        ff_ratio = (float)l_angle / (float)r_angle;
    }

    // ★フリーズ対策: ％f を使わず、整数部と小数部に分けて表示する
    int ff_int = (int)ff_ratio;
    int ff_dec = (int)(ff_ratio * 1000.0f) % 1000;
    if (ff_dec < 0) ff_dec = -ff_dec; // マイナス対策

    char buf1[32], buf2[32], buf3[32];
    snprintf(buf1, sizeof(buf1), "L_deg: %ld", (long)l_angle);
    snprintf(buf2, sizeof(buf2), "R_deg: %ld", (long)r_angle);
    snprintf(buf3, sizeof(buf3), "FF Ratio: %d.%03d", ff_int, ff_dec);

    ev3_lcd_draw_string(buf1, 0, 45);
    tslp_tsk(sleep_us);

    ev3_lcd_draw_string(buf2, 0, 65);
    tslp_tsk(sleep_us);

    ev3_lcd_draw_string(buf3, 0, 85);
    tslp_tsk(sleep_us);
    
    // 全処理完了
    ev3_speaker_play_tone(1000, 200);

    while (true) {
        if (ev3_button_is_pressed(ENTER_BUTTON)) break;
        tslp_tsk(sleep_us);
    }
}
