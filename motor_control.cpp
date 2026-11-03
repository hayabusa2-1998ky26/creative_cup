#include "motor_control.h"
#include "pybricks_ev3rt.h"

namespace {
    int limit_speed(int speed_percent) {
        if (speed_percent <= 0) return 0;
        return speed_percent > 100 ? 100 : speed_percent;
    }

    void rotate_motor(pybricks::Motor& motor, int angle_deg, int speed_percent) {
        const int speed = limit_speed(speed_percent);
        if (speed == 0) {
            motor.brake();
            return;
        }
        if (angle_deg == 0) return;
        const int degrees = motor.get_reversed() ? -angle_deg : angle_deg;
        // EV3RTの角度指定APIは速度を最大速度に対する%で受け取る。
        ev3_motor_rotate(motor.get_port(), degrees, speed, true);
        motor.brake();
    }

    // 初回の呼び出し時に初期化し、以後は同じモーターを使用する。
    pybricks::Motor& rack_device() {
        static pybricks::Motor motor(EV3_PORT_A);
        return motor;
    }

    pybricks::Motor& catch_device() {
        static pybricks::Motor motor(EV3_PORT_D);
        return motor;
    }
}

void init_aux_motors() {
    rack_device();
    catch_device();
}

void rack_motor(int angle_deg, int speed_percent) {
    // rotate_motor(rack_device(), angle_deg, speed_percent);
    rack_device().run_angle(speed_percent * 0.01f * 1500, angle_deg);
}

void catch_motor(int angle_deg, int speed_percent) {
    // rotate_motor(catch_device(), angle_deg, speed_percent);
    catch_device().run_angle(speed_percent * 0.01f * 1500, angle_deg);
}

bool rack_motor_until_stalled(int direction, int speed_percent) {
    const int speed = limit_speed(speed_percent);
    return rack_device().run_until_stalled(direction > 0 ? speed : (direction < 0 ? -speed : 0));
}

bool catch_motor_until_stalled(int direction, int speed_percent) {
    const int speed = limit_speed(speed_percent);
    return catch_device().run_until_stalled(direction > 0 ? speed : (direction < 0 ? -speed : 0));
}
