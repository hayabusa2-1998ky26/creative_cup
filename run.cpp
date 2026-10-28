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
    ev3_lcd_fill_rect(0, 0, EV3_LCD_WIDTH, EV3_LCD_HEIGHT, EV3_LCD_WHITE);
    ev3_lcd_draw_string("Main Course Start", 0, 0);

    // 【重要】以下の行は app.cpp 側で生成済みなため削除します
    // pybricks::DriveBase robot(left_motor, right_motor, 62.5f, 184.0f);

    robot->set_sensor_coordinates(-80.0f, 48.0f, 80.0f, 48.0f);

    const float LINE_INTERVAL = 240.0;
    const float LINE_INTERVAL_START = 363.0;

    // straight_lines は参照（&）を求めているため、ポインタ変数の前に * をつけて実体として渡します
    // robot->straight_lines(*left_sensor, *right_sensor, 3, LINE_INTERVAL * 2 + LINE_INTERVAL_START - 20);
    robot->straight(1000);

    // ライントレースなどの処理を追加していく
}
