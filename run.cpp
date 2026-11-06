// Read C++ standard headers before EV3RT defines its alignof macro.
#include <algorithm>
#include "ev3api.h"
#include "pybricks_ev3rt.h"
#include "programs.h"
#include "motion_sequence.h"
#include "motor_control.h"

// すべてポインタ（*）に変更します
extern pybricks::DriveBase* robot;
extern pybricks::ColorSensor* left_sensor;
extern pybricks::ColorSensor* right_sensor;
extern pybricks::Motor* left_motor;
extern pybricks::Motor* right_motor;

sensor_port_t color_sensor_port = EV3_PORT_4;
int rgbs[4][3];

bool is_in_progress = false;

using namespace pybricks;

void color_reading() {
    ev3_speaker_play_tone(500, 50);

    rgb_raw_t rgb_val;
        
    // カラーセンサーからRGBのRaw値を取得
    ev3_color_sensor_get_rgb_raw(color_sensor_port, &rgb_val);
    
    for (int i = 0; i < 4; i++) {
        if (rgbs[i][0] != 0) {
            continue;
        }

        rgbs[i][0] = rgb_val.r;
        rgbs[i][1] = rgb_val.g;
        rgbs[i][2] = rgb_val.b;
        break;
    }
}


int find(const char* target_list, const char target_element) {
    for (int i = 0; i < 4; i++) {
        if (target_list[i] == target_element) {
            return i; 
        }
    }

    return -1;
}


void run_main_course() {
    auto& robot = *::robot;
    ev3_lcd_draw_string("Press Enter...", 0, 40);

    // Enterボタンが押されるまで待機
    ev3_speaker_play_tone(1000, 50);
    while (!ev3_button_is_pressed(ENTER_BUTTON)) {
        tslp_tsk(10 * 1000);
    }
    tslp_tsk(500 * 1000); // チャタリング防止と走行開始前のタメ
    
    ev3_lcd_fill_rect(0, 0, EV3_LCD_WIDTH, EV3_LCD_HEIGHT, EV3_LCD_WHITE);
    ev3_lcd_draw_string("Main Course Start", 0, 0);

    // ★ 左右対称の設計に基づき、X座標（絶対値）とY座標の2つだけを渡す
    robot.set_sensor_coordinates(72.0f, 48.0f);

    const float LINE_INTERVAL = 240.0;
    const float LINE_INTERVAL_START = 363.0;
    
    // tslp_tsk(1000*100);
    // act_tsk(INIT_ARM);
    // robot.straight_lines(4, LINE_INTERVAL * 3 - 80 + 128, Opt().with_red_mind());
    // tslp_tsk(1000000000);


    // 開始
    robot.straight_line(330, 0.0f, -1.0f, false, false, false, false, true);
    tslp_tsk(1000 * 50);

    robot.straight(15);
    tslp_tsk(1000 * 50);
    
    robot.turn(90);
    tslp_tsk(1000*50);

    // 色読み
    robot.straight_lines(4, LINE_INTERVAL_START + LINE_INTERVAL * 3 - 80 + 40, Opt().on_line(&color_reading, 128).with_red_mind()); // 40は微調整値
    robot.straight(128);
    color_reading();
    // robot.straight_lines(4, LINE_INTERVAL_START + LINE_INTERVAL * 3 - 80 + 140 + 40, &color_reading, 128, 0.0f, -1.0f, true, 140); // 40は微調整値
    tslp_tsk(1000*100);
    
    
    // 下側に移動
    robot.straight(50);
    robot.wide_forward(180, 206 / 2);
    ev3_sensor_config(EV3_PORT_4, NONE_SENSOR);
    // ev3_sensor_config(EV3_PORT_1, COLOR_SENSOR);
    act_tsk(INIT_ARM);
    robot.straight_lines(4, LINE_INTERVAL * 3 - 80 + 128, Opt().with_red_mind());

    // RGB値から色を推定
    char colors[4] = {' ', ' ', ' ', ' '};
    float y_score[4];
    float b_score[4];
    float n_score[4];

    for (int i = 0; i < 4; i++) {
        int r = rgbs[i][0];
        int g = rgbs[i][1];
        int b = rgbs[i][2];
        
        b_score[i] = b - (r + g) / 2.0f;
        n_score[i] = abs(r - g) + abs(g - b) + abs(b - r);
        y_score[i] = g - (r + b) / 2.0f;
    }

    colors[(int)(std::max_element(b_score, b_score + 4) - b_score)] = 'b';
    n_score[(int)(std::max_element(b_score, b_score + 4) - b_score)] = 999;
    colors[(int)(std::min_element(n_score, n_score + 4) - n_score)] = 'n';
    y_score[(int)(std::max_element(b_score, b_score + 4) - b_score)] = 0;
    y_score[(int)(std::min_element(n_score, n_score + 4) - n_score)] = 0;
    colors[(int)(std::max_element(y_score, y_score + 4) - y_score)] = 'y';

    for (int i = 0; i < 4; i++) {
        if (colors[i] == ' ') {
            colors[i] = 'r';
        }
    }

    // // 推定した色を表示
    // char buf_colors[4][2];
    // char buf_rs[4][4];
    // char buf_gs[4][4];
    // char buf_bs[4][4];
    // for (int i = 0; i < 4; i++) {
    //     snprintf(buf_colors[i], sizeof(buf_colors[i]), "%1c", colors[i]);
    //     snprintf(buf_rs[i], sizeof(buf_rs[i]), "%2d", rgbs[i][0]);
    //     snprintf(buf_gs[i], sizeof(buf_gs[i]), "%2d", rgbs[i][1]);
    //     snprintf(buf_bs[i], sizeof(buf_bs[i]), "%2d", rgbs[i][2]);
    // }
    //
    // // 画面を白でクリア
    // ev3_lcd_fill_rect(0, 0, EV3_LCD_WIDTH, EV3_LCD_HEIGHT, EV3_LCD_WHITE);
    //
    // // 値をLCDに描画
    // ev3_lcd_draw_string("RGB Raw Sensor", 0, 0);
    // for (int i = 0; i < 4; i++) {
    // is_in_progress = true;
    //     ev3_lcd_draw_string(buf_colors[i], 0, 20 * (i + 1));
    //     ev3_lcd_draw_string(buf_rs[i], 30, 20 * (i + 1));
    //     ev3_lcd_draw_string(buf_gs[i], 60, 20 * (i + 1));
    //     ev3_lcd_draw_string(buf_bs[i], 90, 20 * (i + 1));
    // }
    //
    // ev3_speaker_play_tone(1000, 50);
    // while (!ev3_button_is_pressed(ENTER_BUTTON)) {
    //     tslp_tsk(10 * 1000);
    // }
    // tslp_tsk(500 * 1000); // チャタリング防止と走行開始前のタメ
    tslp_tsk(100 * 1000);
    

    // オブジェクト処理
    char COLOR_ORDER[4] = {'g', 'b', 'y', 'r'};
    int ago_pos = 0;
    int goal_pos;

    for (int i = 0; i < 3; i++) {
        // オブジェクト入れる
        while (is_in_progress) {
            tslp_tsk(1000*50);
        }

        catch_block();
        act_tsk(RETURN_ARM);

        goal_pos = find(COLOR_ORDER, colors[ago_pos]);
        robot.straight_lines(ago_pos - goal_pos, abs(ago_pos - goal_pos) * LINE_INTERVAL, Opt().with_red_mind().with_start_red_mind());
        
        ago_pos = goal_pos;
    }

    // 最後のオブジェクトだけ別処理
    catch_block();
    act_tsk(RETURN_ARM);

    goal_pos = 3;
    robot.straight_lines(ago_pos - goal_pos, abs(ago_pos - goal_pos) * LINE_INTERVAL, Opt().with_red_mind());
    
    // ゴール
    robot.straight(48 - LINE_INTERVAL_START);
    tslp_tsk(1000*50);
    robot.turn(90);
    tslp_tsk(1000*50);
    robot.straight_line(-80, 0.0f, -1.0f, false, false, false, false, true);
    tslp_tsk(1000*50);
    
    catch_motor(-5, 100);
    rack_motor_until_stalled(1, 20);

    robot.straight(-250 / 2 + 48 - 4); // 最後の値は微調整値
    tslp_tsk(1000*50);
    robot.turn(90);

}

