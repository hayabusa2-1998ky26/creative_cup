#include "app.h"
#include "pybricks_ev3rt.h"

extern "C" {
    void main_task(intptr_t exinf) {
        pybricks::Motor left_motor(EV3_PORT_B, pybricks::Direction::COUNTERCLOCKWISE);
        pybricks::Motor right_motor(EV3_PORT_C, pybricks::Direction::CLOCKWISE);
        
        pybricks::ColorSensor left_sensor(EV3_PORT_2);
        pybricks::ColorSensor right_sensor(EV3_PORT_3);

        tslp_tsk(1000 * 1000);

        pybricks::DriveBase robot(left_motor, right_motor, 62.5f, 184.0f);

        // カラーセンサーの座標を設定 (左X, 左Y, 右X, 右Y) [単位: mm]
        // 例: 左センサー (-92mm, 55mm), 右センサー (92mm, 55mm)
        robot.set_sensor_coordinates(-80.0f, 48.0f, 80.0f, 48.0f);

        // 設定座標から自動計算された距離で、3本目の黒線まで走行＆位置補正
        robot.settings(1000.0f, 800.0f, 150.0f, 300.0f);
        robot.straight(1000);
        robot.turn(90);
        
          // robot.straight_lines(left_sensor, right_sensor, 3, 200.0f, 45);

        ext_tsk();
    }
}
