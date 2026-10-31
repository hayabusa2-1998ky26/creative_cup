#include "ev3api.h"
#include "pybricks_ev3rt.h"
#include "programs.h"

// すべてポインタ（*）に変更します
extern pybricks::DriveBase* robot;
extern pybricks::ColorSensor* left_sensor;
extern pybricks::ColorSensor* right_sensor;
extern pybricks::Motor* left_motor;
extern pybricks::Motor* right_motor;

void run_main_course() {
    auto& robot = *::robot;

    ev3_lcd_fill_rect(0, 0, EV3_LCD_WIDTH, EV3_LCD_HEIGHT, EV3_LCD_WHITE);
    ev3_lcd_draw_string("Main Course Start", 0, 0);

    // ★ 左右対称の設計に基づき、X座標（絶対値）とY座標の2つだけを渡す
    robot.set_sensor_coordinates(80.0f, 48.0f);

    const float LINE_INTERVAL = 240.0;
    const float LINE_INTERVAL_START = 363.0;

    robot.straight_lines(-3, LINE_INTERVAL * 2 + LINE_INTERVAL_START - 20);
    // robot.straight(-1000);
    // robot.straight_line(-LINE_INTERVAL_START);

    // ライントレースなどの処理を追加していく
}
