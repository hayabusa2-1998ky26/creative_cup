#include "ev3api.h"
#include "app.h"
#include "pybricks_ev3rt.h"
#include "programs.h"

// =========================================================
// グローバルポインタの宣言
// =========================================================
pybricks::Motor* left_motor;
pybricks::Motor* right_motor;
pybricks::ColorSensor* left_sensor;
pybricks::ColorSensor* right_sensor;
pybricks::DriveBase* robot;

// =========================================================
// メインタスク
// =========================================================
void main_task(intptr_t unused) {
    left_motor   = new pybricks::Motor(EV3_PORT_B, pybricks::Direction::COUNTERCLOCKWISE);
    right_motor  = new pybricks::Motor(EV3_PORT_C, pybricks::Direction::CLOCKWISE);

    // ★変更: キャリブレーション設定値を渡してインスタンス生成
    left_sensor  = new pybricks::ColorSensor(EV3_PORT_2, pybricks::DriveConfig::left_sensor_min, pybricks::DriveConfig::left_sensor_max);
    right_sensor = new pybricks::ColorSensor(EV3_PORT_3, pybricks::DriveConfig::right_sensor_min, pybricks::DriveConfig::right_sensor_max);

    robot = new pybricks::DriveBase(*left_motor, *right_motor, *left_sensor, *right_sensor);

    run_main_course();
    // run_calibration();
    run_color_calibration();

    ext_tsk();
}
