#include "ev3api.h"
#include "pybricks_ev3rt.h"
#include "programs.h"
#include <algorithm>

// すべてポインタ（*）に変更します
extern pybricks::DriveBase* robot;
extern pybricks::ColorSensor* left_sensor;
extern pybricks::ColorSensor* right_sensor;
extern pybricks::Motor* left_motor;
extern pybricks::Motor* right_motor;

sensor_port_t color_sensor_port = EV3_PORT_4;
int rgbs[4][3];


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
    ev3_sensor_config(color_sensor_port, COLOR_SENSOR);

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

    // robot.straight_lines(-3, LINE_INTERVAL * 2 + LINE_INTERVAL_START - 20);
    // robot.straight_line(-LINE_INTERVAL_START);

    // ライントレースなどの処理を追加していく
    // robot.straight_line(300);
    robot.straight_lines(-4, LINE_INTERVAL * 3 + LINE_INTERVAL_START);
    tslp_tsk(1000000000);
    //
    robot.straight_line(315);
    tslp_tsk(1000 * 50);
    robot.straight(18);
    tslp_tsk(1000 * 50);
    robot.turn(90);
    robot.straight_lines(4, LINE_INTERVAL_START + LINE_INTERVAL * 3 - 80, &color_reading, 128);
    robot.straight(128);
    tslp_tsk(1000*100);
    color_reading();
    robot.wide_forward(180, 213 / 2);
    robot.straight_lines(4, LINE_INTERVAL * 3 - 80 + 128);

    // // 画面表示用に文字列をフォーマット
    // char buf_r[32], buf_g[32], buf_b[32];
    // snprintf(buf_r, sizeof(buf_r), "R: %5d", rgbs[0][0]);
    // snprintf(buf_g, sizeof(buf_g), "G: %5d", rgbs[0][1]);
    // snprintf(buf_b, sizeof(buf_b), "B: %5d", rgbs[0][2]);
    //
    // // 画面を白でクリア
    // ev3_lcd_fill_rect(0, 0, EV3_LCD_WIDTH, EV3_LCD_HEIGHT, EV3_LCD_WHITE);
    //
    // // 値をLCDに描画
    // ev3_lcd_draw_string("RGB Raw Sensor", 0, 0);
    // ev3_lcd_draw_string(buf_r, 0, 20);
    // ev3_lcd_draw_string(buf_g, 0, 40);
    // ev3_lcd_draw_string(buf_b, 0, 60);
    //
    // // ループが早すぎるのを防ぐため、100ms（100,000μs）待機
    // tslp_tsk(100 * 1000);

    char colors[4] = {' ', ' ', ' ', ' '};
    float y_score[4];
    float b_score[4];
    float n_score[4];

    for (int i = 0; i < 4; i++) {
        int r = rgbs[i][0];
        int g = rgbs[i][1];
        int b = rgbs[i][2];
        
        b_score[i] = b - (r + g) / 2.0;
        n_score[i] = abs(r - g) + abs(g - b) + abs(b - r);
        y_score[i] = g;
    }

    colors[(int)(std::max_element(b_score, b_score + 4) - b_score)] = 'b';
    colors[(int)(std::min_element(n_score, n_score + 4) - n_score)] = 'n';
    y_score[(int)(std::max_element(b_score, b_score + 4) - b_score)] = 0;
    y_score[(int)(std::min_element(n_score, n_score + 4) - n_score)] = 0;
    colors[(int)(std::max_element(y_score, y_score + 4) - y_score)] = 'y';

    for (int i = 0; i < 4; i++) {
        if (colors[i] == ' ') {
            colors[i] = 'r';
        }
    }

    // 画面表示用に文字列をフォーマット
    char buf_colors[4][32];
    char buf_gs[4][32];
    for (int i = 0; i < 4; i++) {
        snprintf(buf_colors[i], sizeof(buf_colors[i]), "%5c", colors[i]);
        snprintf(buf_gs[i], sizeof(buf_gs[i]), "%5d", rgbs[i][1]);
    }

    // 画面を白でクリア
    ev3_lcd_fill_rect(0, 0, EV3_LCD_WIDTH, EV3_LCD_HEIGHT, EV3_LCD_WHITE);

    // 値をLCDに描画
    ev3_lcd_draw_string("RGB Raw Sensor", 0, 0);
    for (int i = 0; i < 4; i++) {
        ev3_lcd_draw_string(buf_colors[i], 0, 20 * (i + 1));
        ev3_lcd_draw_string(buf_gs[i], 50, 20 * (i + 1));
    }

    tslp_tsk(100 * 1000);

    tslp_tsk(1000*1000*2);

    char COLOR_ORDER[4] = {'g', 'b', 'y', 'r'};
    int ago_pos = 0;
    int goal_pos;

    for (int i = 0; i < 3; i++) {
        // オブジェクト入れる
        

        goal_pos = find(COLOR_ORDER, colors[ago_pos]);
        robot.straight_lines(ago_pos - goal_pos, abs(ago_pos - goal_pos) * LINE_INTERVAL);
        ago_pos = goal_pos;

        tslp_tsk(1000*1000);
    }

    goal_pos = 3;
    robot.straight_lines(ago_pos - goal_pos, abs(ago_pos - goal_pos) * LINE_INTERVAL);
    robot.straight(-48 - LINE_INTERVAL_START);
    tslp_tsk(1000*50);
    robot.turn(-90);
    tslp_tsk(1000*50);
    robot.straight(100);
}

