#include "pybricks_ev3rt.h"
#include <stdio.h>
#include <math.h>

using namespace pybricks;

// ==========================================
// キャリブレーション設定値
// ==========================================
// 線と線の間隔（ミリメートル）。実際のコースに合わせて変更してください。
const float LINE_INTERVAL_MM = 100.0f; 

// 4本の線をまたぐため、1本目から4本目までは「3区間」になります
const float CALIBRATION_DIST_MM = LINE_INTERVAL_MM * 3.0f;

// キャリブレーション時の走行スピード
const float CALIB_SPEED_MM_S = 150.0f;

void main_task(intptr_t unused) {
    // 1. ポート設定とインスタンス化
    Motor left_motor(EV3_PORT_B, Direction::CLOCKWISE);
    Motor right_motor(EV3_PORT_C, Direction::CLOCKWISE);
    ColorSensor left_sensor(EV3_PORT_2);
    ColorSensor right_sensor(EV3_PORT_3);

    // ※ここは仮のタイヤ径（ここでは56.0mm）で初期化します。
    // キャリブレーション走行自体には仮の径が使われますが、
    // 最終的にエンコーダーの生値から正しい径を逆算するため問題ありません。
    DriveBase drivebase(left_motor, right_motor, left_sensor, right_sensor, 56.0f, 120.0f);

    ev3_lcd_set_font(EV3_FONT_MEDIUM);
    ev3_lcd_fill_rect(0, 0, EV3_LCD_WIDTH, EV3_LCD_HEIGHT, EV3_LCD_WHITE);
    ev3_lcd_draw_string("CALIBRATION MODE", 0, 10);
    ev3_lcd_draw_string("Press Enter...", 0, 40);

    // Enterボタンが押されるまで待機
    while (!ev3_button_is_pressed(ENTER_BUTTON)) {
        tslp_tsk(10 * 1000);
    }
    tslp_tsk(500 * 1000); // チャタリング防止と走行開始前のタメ

    ev3_lcd_fill_rect(0, 0, EV3_LCD_WIDTH, EV3_LCD_HEIGHT, EV3_LCD_WHITE);
    ev3_lcd_draw_string("Finding Line 1...", 0, 10);

    // ==========================================
    // ステップ1: 1本目の線まで移動して起点を確定
    // ==========================================
    // 距離を 0.0f に指定し、線を見つけるまで走らせる
    drivebase.straight_lines(1, 0.0f, CALIB_SPEED_MM_S, -1);
    
    // 完全に停止し、モーターのブレが収まるのを待つ
    drivebase.brake();
    tslp_tsk(500 * 1000); 

    // 起点となるエンコーダーの生値（角度）を取得
    int32_t start_l = left_motor.angle();
    int32_t start_r = right_motor.angle();

    // ==========================================
    // ステップ2: 残り3本の線をまたぐ
    // ==========================================
    ev3_lcd_draw_string("Measuring 3 lines...", 0, 30);
    drivebase.straight_lines(3, 0.0f, CALIB_SPEED_MM_S, -1);
    
    drivebase.brake();
    tslp_tsk(500 * 1000);

    // 終点のエンコーダーの生値（角度）を取得
    int32_t end_l = left_motor.angle();
    int32_t end_r = right_motor.angle();

    // ==========================================
    // ステップ3: タイヤ径の計算
    // ==========================================
    // 左右モーターの回転角の平均を算出
    float moved_deg_l = (float)(end_l - start_l);
    float moved_deg_r = (float)(end_r - start_r);
    float avg_moved_deg = (moved_deg_l + moved_deg_r) / 2.0f;

    if (avg_moved_deg <= 0.0f) {
        ev3_lcd_draw_string("ERROR: No rotation", 0, 70);
        return;
    }

    // 計算式:
    // タイヤの円周(周長) = 走行距離 / (回転角度 / 360度)
    // 直径 = 円周 / π
    float rotations = avg_moved_deg / 360.0f;
    float wheel_circumference = CALIBRATION_DIST_MM / rotations;
    float measured_diameter = wheel_circumference / M_PI;

    // ==========================================
    // 結果の表示
    // ==========================================
    char buf1[32], buf2[32], buf3[32];
    snprintf(buf1, sizeof(buf1), "Deg: %.1f", avg_moved_deg);
    snprintf(buf2, sizeof(buf2), "Dist: %.1f mm", CALIBRATION_DIST_MM);
    snprintf(buf3, sizeof(buf3), "DIA: %.2f mm", measured_diameter);

    ev3_lcd_fill_rect(0, 0, EV3_LCD_WIDTH, EV3_LCD_HEIGHT, EV3_LCD_WHITE);
    ev3_lcd_draw_string("--- RESULT ---", 0, 10);
    ev3_lcd_draw_string(buf1, 0, 35);
    ev3_lcd_draw_string(buf2, 0, 60);
    
    // 直径は目立つように少し下に表示
    ev3_lcd_draw_string(buf3, 0, 95);
    
    // 完了のビープ音
    ev3_speaker_play_tone(NOTE_C5, 200);
    tslp_tsk(200 * 1000);
    ev3_speaker_play_tone(NOTE_E5, 200);
    tslp_tsk(200 * 1000);
    ev3_speaker_play_tone(NOTE_G5, 400);

    // プログラムが終了しないように無限ループで待機
    while (true) {
        tslp_tsk(100 * 1000);
    }
}
