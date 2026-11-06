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
    ev3_sensor_config(EV3_PORT_4, COLOR_SENSOR);
    rgb_raw_t rgb_val;
    ev3_color_sensor_get_rgb_raw(EV3_PORT_4, &rgb_val); // 空打ち

    ev3_sensor_config(EV3_PORT_1, COLOR_SENSOR);
    ev3_color_sensor_get_rgb_raw(EV3_PORT_1, &rgb_val); // 空打ち
    tslp_tsk(1000*500);

    // ev3_sensor_config(EV3_PORT_1, NONE_SENSOR);

    robot = new pybricks::DriveBase(*left_motor, *right_motor, *left_sensor, *right_sensor);

    run_main_course();
    // run_calibration();
    // run_color_calibration();
    // run_motor_calibration();

    ext_tsk();
}

// EV3-RT環境用のメモリバリアダミー関数
extern "C" void __sync_synchronize() {
    // 空実装で問題ありません
}

