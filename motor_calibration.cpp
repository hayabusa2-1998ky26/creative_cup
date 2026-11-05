#include "ev3api.h"
#include "pybricks_ev3rt.h"
#include "programs.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

extern pybricks::Motor* left_motor;
extern pybricks::Motor* right_motor;
extern pybricks::DriveBase* robot;

void run_motor_calibration() {
    auto& db = *::robot;
    auto& l_mot = *::left_motor;
    auto& r_mot = *::right_motor;

    ev3_lcd_set_font(EV3_FONT_MEDIUM);
    ev3_lcd_fill_rect(0, 0, EV3_LCD_WIDTH, EV3_LCD_HEIGHT, EV3_LCD_WHITE);
    ev3_lcd_draw_string("MOTOR CALIB", 0, 10);
    ev3_lcd_draw_string("Press Enter...", 0, 40);

    // Enterボタンで開始
    while (!ev3_button_is_pressed(ENTER_BUTTON)) { 
        tslp_tsk(10 * 1000); 
    }
    ev3_speaker_play_tone(1000, 50);
    tslp_tsk(500 * 1000); 

    ev3_lcd_fill_rect(0, 0, EV3_LCD_WIDTH, EV3_LCD_HEIGHT, EV3_LCD_WHITE);
    ev3_lcd_draw_string("Testing Motors...", 0, 10);

    // エンコーダー初期化
    l_mot.reset_angle(0);
    r_mot.reset_angle(0);
    tslp_tsk(100 * 1000);

    // テスト走行のパラメータ（オープンループで50%出力）
    const float test_power = 50.0f;
    const int32_t target_deg = 2000; // 約1m分走行
    ulong_t sleep_us = (ulong_t)(pybricks::DriveConfig::period_ms * 1000.0f);

    // 同期制御なしの純粋なオープンループ駆動
    l_mot.dc(test_power);
    r_mot.dc(test_power);

    while (true) {
        int32_t cur_l = fabsf((float)l_mot.angle());
        int32_t cur_r = fabsf((float)r_mot.angle());
        
        float avg_deg = (cur_l + cur_r) / 2.0f;
        if (avg_deg >= target_deg) {
            break;
        }

        tslp_tsk(sleep_us);
    }

    db.brake();

    int32_t final_l = labs(l_mot.angle());
    int32_t final_r = labs(r_mot.angle());

    // 両輪パワー補正比率の計算（上限 1.000f）
    float left_ratio = 1.000f;
    float right_ratio = 1.000f;

    if (final_l > 0 && final_r > 0) {
        if (final_l < final_r) {
            // 左の方が回りにくい場合：左を 1.000f に固定し、右を割り引く
            left_ratio = 1.000f;
            right_ratio = (float)final_l / (float)final_r;
        } else {
            // 右の方が回りにくい場合：右を 1.000f に固定し、左を割り引く
            right_ratio = (float)final_r / (float)final_l;
            left_ratio = 1.000f;
        }
    }

    // LCD結果表示
    char buf_l_deg[32], buf_r_deg[32], buf_l_ratio[32], buf_r_ratio[32];
    snprintf(buf_l_deg,   sizeof(buf_l_deg),   "L_Deg: %ld", (long)final_l);
    snprintf(buf_r_deg,   sizeof(buf_r_deg),   "R_Deg: %ld", (long)final_r);
    snprintf(buf_l_ratio, sizeof(buf_l_ratio), "L_Power: %.3ff", left_ratio);
    snprintf(buf_r_ratio, sizeof(buf_r_ratio), "R_Power: %.3ff", right_ratio);

    ev3_lcd_fill_rect(0, 0, EV3_LCD_WIDTH, EV3_LCD_HEIGHT, EV3_LCD_WHITE);
    ev3_lcd_draw_string("--- RESULT ---", 0, 0);
    ev3_lcd_draw_string(buf_l_deg, 0, 20);
    ev3_lcd_draw_string(buf_r_deg, 0, 40);
    ev3_lcd_draw_string(buf_l_ratio, 0, 65);
    ev3_lcd_draw_string(buf_r_ratio, 0, 85);

    // 完了音
    ev3_speaker_play_tone(1000, 100); tslp_tsk(150 * 1000);
    ev3_speaker_play_tone(1200, 100);

    tslp_tsk(500 * 1000); 
    while (!ev3_button_is_pressed(ENTER_BUTTON)) { 
        tslp_tsk(10 * 1000); 
    }
}
