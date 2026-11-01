#include "ev3api.h"
#include "pybricks_ev3rt.h"
#include "programs.h"
#include <stdio.h>
#include <math.h>

using namespace pybricks;

// app.cpp で宣言されているグローバルポインタ
extern pybricks::DriveBase* robot;
extern pybricks::Motor* left_motor;
extern pybricks::Motor* right_motor;
extern pybricks::ColorSensor* left_sensor;
extern pybricks::ColorSensor* right_sensor;

// ==========================================
// キャリブレーション設定値
// ==========================================
const float LINE_INTERVAL_MM = 240.0f; 
const float CALIBRATION_DIST_MM = LINE_INTERVAL_MM * 3.0f - 10.0f; // 1本目から4本目まで
const float CALIB_SPEED_MM_S = 150.0f;
const float REQUESTED_TURN_DEG = 540.0f; // トレッド幅測定用の旋回角度

void run_calibration() {
    auto& db = *::robot;
    auto& l_mot = *::left_motor;
    auto& r_mot = *::right_motor;
    auto& l_sens = *::left_sensor;
    auto& r_sens = *::right_sensor;

    ev3_lcd_set_font(EV3_FONT_MEDIUM);
    ev3_lcd_fill_rect(0, 0, EV3_LCD_WIDTH, EV3_LCD_HEIGHT, EV3_LCD_WHITE);
    ev3_lcd_draw_string("FULL CALIBRATION", 0, 10);
    ev3_lcd_draw_string("Press Enter...", 0, 40);

    while (!ev3_button_is_pressed(ENTER_BUTTON)) { tslp_tsk(10 * 1000); }
    tslp_tsk(500 * 1000); 

    // ==========================================
    // 前半: タイヤ径 (wheel_diameter) の測定
    // ==========================================
    ev3_lcd_fill_rect(0, 0, EV3_LCD_WIDTH, EV3_LCD_HEIGHT, EV3_LCD_WHITE);
    ev3_lcd_draw_string("1/4: Find Line 1", 0, 10);
    db.straight_lines(1, 0.0f, nullptr, 0.0f, CALIB_SPEED_MM_S, -1);
    db.brake();
    tslp_tsk(500 * 1000);
    db.straight(10);
    tslp_tsk(100 * 1000);

    int32_t start_l = l_mot.angle();
    int32_t start_r = r_mot.angle();

    ev3_lcd_draw_string("2/4: Measure Dia", 0, 35);
    db.straight_lines(3, 0.0f, nullptr, 0.0f, CALIB_SPEED_MM_S, -1);
    db.brake();
    tslp_tsk(500 * 1000);

    float avg_moved_deg = ((l_mot.angle() - start_l) + (r_mot.angle() - start_r)) / 2.0f;
    float measured_diameter = (CALIBRATION_DIST_MM / (avg_moved_deg / 360.0f)) / M_PI;

    // ==========================================
    // 後半: トレッド幅 (axle_track) の測定
    // ==========================================
    // 100mm前進して旋回スペースを確保
    ev3_lcd_draw_string("3/4: Turn 540", 0, 60);
    db.straight(100.0f);
    tslp_tsk(300 * 1000);

    // 540度旋回（現在の設定値に依存して回るため、ズレが生じる）
    db.turn(REQUESTED_TURN_DEG);
    tslp_tsk(500 * 1000);

    ev3_lcd_draw_string("4/4: Check Track", 0, 85);

    int crossed_lines = 0;
    float measured_angle_error_deg = 0.0f;
    float default_axle_track = pybricks::DriveConfig::axle_track; 
    int threshold = pybricks::DriveConfig::target_val;

    db.drive(CALIB_SPEED_MM_S, 0.0f);

    while (crossed_lines < 2) {
        bool l_on_black = false, r_on_black = false;
        int32_t l_enter = 0, r_enter = 0;
        int32_t l_exit = 0, r_exit = 0;

        while(l_sens.reflection() < threshold + 10 && r_sens.reflection() < threshold + 10) {
            tslp_tsk(10 * 1000);
        }

        while (true) {
            int l_val = l_sens.reflection();
            int r_val = r_sens.reflection();
            int32_t cur_avg_deg = (l_mot.angle() + r_mot.angle()) / 2;

            if (!l_on_black && l_val <= threshold) { l_on_black = true; l_enter = cur_avg_deg; }
            if (!r_on_black && r_val <= threshold) { r_on_black = true; r_enter = cur_avg_deg; }

            if (l_on_black && l_exit == 0 && l_val > threshold + 10) { l_exit = cur_avg_deg; }
            if (r_on_black && r_exit == 0 && r_val > threshold + 10) { r_exit = cur_avg_deg; }

            if (l_exit != 0 && r_exit != 0) break;
            tslp_tsk(5 * 1000);
        }

        int32_t l_center = (l_enter + l_exit) / 2;
        int32_t r_center = (r_enter + r_exit) / 2;
        
        // 修正箇所: 左が先に検知（l_centerが小さい）＝右向きに回りすぎ＝プラスのエラーとするため
        // r_center - l_center で計算します。
        int32_t center_diff = r_center - l_center; 
        
        float diff_mm = ((float)center_diff / 360.0f) * (measured_diameter * (float)M_PI);
        
        float ratio = diff_mm / default_axle_track;
        if (ratio > 1.0f) ratio = 1.0f;
        if (ratio < -1.0f) ratio = -1.0f;

        float angle_rad = asinf(ratio);
        measured_angle_error_deg = angle_rad * 180.0f / (float)M_PI;

        crossed_lines++;
        ev3_speaker_play_tone(1000, 50);
        tslp_tsk(200 * 1000); 
    }
    db.brake();

    // ==========================================
    // トレッド幅の逆算
    // ==========================================
    // 実際の旋回角度 = 540度 + ズレた角度
    // （プラスエラーなら回りすぎた＝実際の旋回角度が540度以上だったということ）
    float actual_turn_deg = REQUESTED_TURN_DEG + measured_angle_error_deg;
    
    // 正確なトレッド幅 = (指定角度 / 実際の角度) * 現在のトレッド幅設定値
    // 回りすぎた場合、分母が大きくなるためトレッド幅の設定値は現在より小さく補正されます。
    float measured_axle_track = default_axle_track * (REQUESTED_TURN_DEG / actual_turn_deg);

    // ==========================================
    // 結果の表示
    // ==========================================
    char buf1[32], buf2[32], buf3[32], buf4[32];
    snprintf(buf1, sizeof(buf1), "W_DIA : %.2f mm", measured_diameter);
    snprintf(buf2, sizeof(buf2), "T_Err : %+.1f deg", measured_angle_error_deg);
    snprintf(buf3, sizeof(buf3), "T_Act : %.1f deg", actual_turn_deg);
    snprintf(buf4, sizeof(buf4), "TRACK : %.2f mm", measured_axle_track);

    ev3_lcd_fill_rect(0, 0, EV3_LCD_WIDTH, EV3_LCD_HEIGHT, EV3_LCD_WHITE);
    ev3_lcd_draw_string("--- RESULTS ---", 0, 0);
    ev3_lcd_draw_string(buf1, 0, 20);
    ev3_lcd_draw_string(buf2, 0, 45);
    ev3_lcd_draw_string(buf3, 0, 70);
    ev3_lcd_draw_string(buf4, 0, 95);
    
    ev3_speaker_play_tone(NOTE_C5, 200); tslp_tsk(200 * 1000);
    ev3_speaker_play_tone(NOTE_E5, 200); tslp_tsk(200 * 1000);
    ev3_speaker_play_tone(NOTE_G5, 400);

    tslp_tsk(500 * 1000); 
    while (!ev3_button_is_pressed(ENTER_BUTTON)) { tslp_tsk(10 * 1000); }
}
