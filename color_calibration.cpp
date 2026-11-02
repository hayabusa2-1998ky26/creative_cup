#include "ev3api.h"
#include "pybricks_ev3rt.h"
#include "programs.h"
#include <stdio.h>
#include <math.h>

// app.cpp で宣言されているグローバルポインタを参照
extern pybricks::DriveBase* robot;
extern pybricks::Motor* left_motor;
extern pybricks::Motor* right_motor;
extern pybricks::ColorSensor* left_sensor;
extern pybricks::ColorSensor* right_sensor;

void run_color_calibration() {
    auto& db = *::robot;
    auto& l_mot = *::left_motor;
    auto& r_mot = *::right_motor;
    auto& l_sens = *::left_sensor;
    auto& r_sens = *::right_sensor;

    ev3_lcd_set_font(EV3_FONT_MEDIUM);
    ev3_lcd_fill_rect(0, 0, EV3_LCD_WIDTH, EV3_LCD_HEIGHT, EV3_LCD_WHITE);
    ev3_lcd_draw_string("COLOR CALIBRATION", 0, 10);
    ev3_lcd_draw_string("Press Enter...", 0, 40);

    // Enterボタンが押されるまで待機
    while (!ev3_button_is_pressed(ENTER_BUTTON)) { 
        tslp_tsk(10 * 1000); 
    }
    ev3_speaker_play_tone(1000, 50);
    tslp_tsk(500 * 1000); // 開始前のタメ

    ev3_lcd_fill_rect(0, 0, EV3_LCD_WIDTH, EV3_LCD_HEIGHT, EV3_LCD_WHITE);
    ev3_lcd_draw_string("Scanning...", 0, 10);

    // 最大値・最小値の初期化
    int l_min = 100, l_max = 0;
    int r_min = 100, r_max = 0;

    // エンコーダーの初期値を取得
    int32_t start_l = l_mot.angle();
    int32_t start_r = r_mot.angle();

    // 200mm進むための目標モーター角度を計算
    const float target_dist_mm = 200.0f;
    float wheel_circ = pybricks::DriveConfig::wheel_diameter * (float)M_PI;
    float target_deg = (target_dist_mm / wheel_circ) * 360.0f;

    // 直進速度 (mm/s) - 低速で定速
    const float scan_speed_mm_s = 50.0f; 
    ulong_t sleep_us = (ulong_t)(pybricks::DriveConfig::period_ms * 1000.0f);

    while (true) {
        // 定速で直進 (同期制御を適用するため drive を呼び続ける)
        db.drive(scan_speed_mm_s, 0.0f);

        int32_t cur_l = l_mot.angle();
        int32_t cur_r = r_mot.angle();

        // 左右の平均移動角度を計算
        float moved_deg = (fabsf((float)(cur_l - start_l)) + fabsf((float)(cur_r - start_r))) / 2.0f;

        // 200mm分進んだら終了
        if (moved_deg >= target_deg) {
            break;
        }

        // カラーセンサーの値を取得して最小・最大を更新
        int l_val = l_sens.reflection();
        int r_val = r_sens.reflection();

        if (l_val < l_min) l_min = l_val;
        if (l_val > l_max) l_max = l_val;
        
        if (r_val < r_min) r_min = r_val;
        if (r_val > r_max) r_max = r_val;

        tslp_tsk(sleep_us);
    }

    // 停止
    db.brake();

    // 結果の表示用に文字列をフォーマット
    char buf_l[32], buf_r[32];
    snprintf(buf_l, sizeof(buf_l), "L: Min=%2d Max=%2d", l_min, l_max);
    snprintf(buf_r, sizeof(buf_r), "R: Min=%2d Max=%2d", r_min, r_max);

    ev3_lcd_fill_rect(0, 0, EV3_LCD_WIDTH, EV3_LCD_HEIGHT, EV3_LCD_WHITE);
    ev3_lcd_draw_string("--- RESULT ---", 0, 10);
    ev3_lcd_draw_string(buf_l, 0, 45);
    ev3_lcd_draw_string(buf_r, 0, 70);

    // 終了を音でお知らせ
    ev3_speaker_play_tone(1000, 100);
    tslp_tsk(150 * 1000); 
    ev3_speaker_play_tone(1200, 100);

    // 結果を確認できるよう、再度Enterが押されるまで待機
    tslp_tsk(500 * 1000); 
    while (!ev3_button_is_pressed(ENTER_BUTTON)) { 
        tslp_tsk(10 * 1000); 
    }
}
