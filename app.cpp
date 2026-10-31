#include "ev3api.h"
#include "app.h"
#include "pybricks_ev3rt.h"
#include "programs.h"

// =========================================================
// グローバルポインタの宣言（ここではまだハードウェア通信しない）
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
    // OSが起動したここで初めて実体を生成する（ハードウェア通信が正常に行われる）
    left_motor   = new pybricks::Motor(EV3_PORT_B, pybricks::Direction::COUNTERCLOCKWISE);
    right_motor  = new pybricks::Motor(EV3_PORT_C, pybricks::Direction::CLOCKWISE);
    left_sensor  = new pybricks::ColorSensor(EV3_PORT_2);
    right_sensor = new pybricks::ColorSensor(EV3_PORT_3);

    // タイヤ径と車輪間隔を設定
    robot = new pybricks::DriveBase(*left_motor, *right_motor, *left_sensor, *right_sensor, 62.5f, 184.0f);

    // 実行したいプログラム
    run_main_course();
    // run_ff_calibration();

    ext_tsk();
}
