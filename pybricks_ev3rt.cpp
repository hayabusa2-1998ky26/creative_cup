#include "pybricks_ev3rt.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

namespace pybricks {

    // ==========================================
    // Motor Class Implementation
    // ==========================================

    Motor::Motor(motor_port_t p, Direction positive_direction) : port(p), last_power(-999) {
        is_reversed = (positive_direction == Direction::COUNTERCLOCKWISE);
        ev3_motor_config(port, LARGE_MOTOR);
        tslp_tsk(50 * 1000); // 50ms
        reset_angle(0);
    }

    void Motor::run(int speed_deg_per_sec) {
        if (speed_deg_per_sec == 0) {
            if (last_power != 0) {
                ev3_motor_set_power(port, 0);
                last_power = 0;
            }
            return;
        }

        int power = speed_deg_per_sec / 10;
        if (power > 100) power = 100;
        if (power < -100) power = -100;
        
        // 不感帯対策：最低パワー ±12 確保
        if (speed_deg_per_sec > 0 && power < 12) power = 12;
        if (speed_deg_per_sec < 0 && power > -12) power = -12;

        int target_power = is_reversed ? -power : power;
        if (target_power != last_power) {
            ev3_motor_set_power(port, target_power);
            last_power = target_power;
        }
    }

    void Motor::stop() {
        last_power = -999;
        ev3_motor_stop(port, false);
    }

    void Motor::brake() {
        last_power = -999;
        ev3_motor_stop(port, true);
    }

    void Motor::hold() {
        last_power = -999;
        ev3_motor_stop(port, true);
    }

    void Motor::run_angle(int speed_deg_per_sec, int angle_deg, Stop then, bool wait) {
        if (angle_deg == 0) return;

        int32_t start_ang = angle();
        int32_t target_ang = start_ang + angle_deg;

        int speed = speed_deg_per_sec;
        if (angle_deg < 0) speed = -speed;

        run(speed);

        if (wait) {
            while (true) {
                int32_t current = angle();
                if (angle_deg > 0 && current >= target_ang) break;
                if (angle_deg < 0 && current <= target_ang) break;
                tslp_tsk(5 * 1000); // 5ms
            }
            if (then == Stop::COAST) stop();
            else brake();
        }
    }

    int32_t Motor::angle() {
        int32_t counts = ev3_motor_get_counts(port);
        return is_reversed ? -counts : counts;
    }

    void Motor::reset_angle(int32_t angle_val) {
        ev3_motor_reset_counts(port);
    }


    // ==========================================
    // ColorSensor Class Implementation
    // ==========================================

    ColorSensor::ColorSensor(sensor_port_t p) : port(p) {
        ev3_sensor_config(port, COLOR_SENSOR);
        tslp_tsk(500 * 1000); // 500ms
    }

    int ColorSensor::reflection() { 
        return ev3_color_sensor_get_reflect(port); 
    }
    
    int ColorSensor::ambient() { 
        return ev3_color_sensor_get_ambient(port); 
    }
    
    colorid_t ColorSensor::color() { 
        return ev3_color_sensor_get_color(port); 
    }


    // ==========================================
    // DriveBase Class Implementation
    // ==========================================

    DriveBase::DriveBase(Motor& left, Motor& right, float wheel_diam, float axle_track_dist)
        : left_motor(&left), right_motor(&right), wheel_diameter(wheel_diam), axle_track(axle_track_dist) {
        
        // drive_config.h から定数を読み込み
        min_speed = DriveConfig::min_speed;
        default_straight_speed = DriveConfig::straight_speed;
        default_straight_accel = DriveConfig::straight_accel;
        default_turn_rate = DriveConfig::turn_rate;
        default_turn_accel = DriveConfig::turn_accel;

        straight_kp = DriveConfig::straight_kp;
        straight_ki = DriveConfig::straight_ki;
        straight_kd = DriveConfig::straight_kd;
        config_tolerance = DriveConfig::tolerance;
        config_target_val = DriveConfig::target_val;

        drive_sync_initialized = false;
        drive_start_left = 0;
        drive_start_right = 0;

        set_sensor_coordinates(-axle_track_dist / 2.0f, 50.0f, axle_track_dist / 2.0f, 50.0f);
    }

    void DriveBase::set_sensor_coordinates(float left_x, float left_y, float right_x, float right_y) {
        left_sensor_x = left_x;
        left_sensor_y = left_y;
        right_sensor_x = right_x;
        right_sensor_y = right_y;

        sensor_width = sqrtf(powf(right_x - left_x, 2.0f) + powf(right_y - left_y, 2.0f));
    }

    void DriveBase::settings(float straight_speed, float straight_acceleration, float turn_rate, float turn_acceleration) {
        default_straight_speed = straight_speed;
        default_straight_accel = straight_acceleration;
        default_turn_rate = turn_rate;
        default_turn_accel = turn_acceleration;
    }

    void DriveBase::straight(float distance_mm, Stop then, bool wait) {
        if (fabsf(distance_mm) < 1.0f) return;

        float wheel_circ = wheel_diameter * M_PI;
        float target_deg = fabsf((distance_mm / wheel_circ) * 360.0f);

        int32_t start_left = left_motor->angle();
        int32_t start_right = right_motor->angle();

        float dir = (distance_mm >= 0) ? 1.0f : -1.0f;

        float max_speed = (default_straight_speed / wheel_circ) * 360.0f; 
        float accel = (default_straight_accel / wheel_circ) * 360.0f; 

        float current_speed = min_speed;

        while (true) {
            int32_t cur_left = left_motor->angle();
            int32_t cur_right = right_motor->angle();

            int32_t left_moved = cur_left - start_left;
            int32_t right_moved = cur_right - start_right;
            float avg_moved = (fabsf((float)left_moved) + fabsf((float)right_moved)) / 2.0f;

            float remaining_deg = target_deg - avg_moved;
            if (remaining_deg <= 0) break;

            float decel_dist = (current_speed * current_speed) / (2.0f * accel);

            if (remaining_deg <= decel_dist) {
                current_speed -= accel * 0.005f; // 5ms
                if (current_speed < min_speed) current_speed = min_speed;
            } else if (current_speed < max_speed) {
                current_speed += accel * 0.005f; // 5ms
                if (current_speed > max_speed) current_speed = max_speed;
            }

            float diff = (float)(left_moved - right_moved);
            float sync_p = diff * 0.5f; 

            left_motor->run((int)(dir * current_speed - sync_p));
            right_motor->run((int)(dir * current_speed + sync_p));

            tslp_tsk(5 * 1000); // 5ms
        }

        if (then == Stop::COAST) stop();
        else brake();
    }

    void DriveBase::turn(float angle_deg, Stop then, bool wait) {
        if (fabsf(angle_deg) < 1.0f) return;

        float robot_circ = axle_track * M_PI;
        float wheel_circ = wheel_diameter * M_PI;

        float dist_mm = (angle_deg / 360.0f) * robot_circ;
        float target_deg = fabsf((dist_mm / wheel_circ) * 360.0f);

        int32_t start_left = left_motor->angle();
        int32_t start_right = right_motor->angle();

        float dir_left = (angle_deg >= 0) ? 1.0f : -1.0f;
        float dir_right = (angle_deg >= 0) ? -1.0f : 1.0f;

        float max_speed = (default_turn_rate * robot_circ / wheel_circ);
        float accel = (default_turn_accel * robot_circ / wheel_circ);

        float current_speed = min_speed;

        while (true) {
            int32_t cur_left = left_motor->angle();
            int32_t cur_right = right_motor->angle();

            int32_t left_moved = cur_left - start_left;
            int32_t right_moved = cur_right - start_right;
            float avg_moved = (fabsf((float)left_moved) + fabsf((float)right_moved)) / 2.0f;

            float remaining_deg = target_deg - avg_moved;
            if (remaining_deg <= 0) break;

            float decel_dist = (current_speed * current_speed) / (2.0f * accel);

            if (remaining_deg <= decel_dist) {
                current_speed -= accel * 0.005f; // 5ms
                if (current_speed < min_speed) current_speed = min_speed;
            } else if (current_speed < max_speed) {
                current_speed += accel * 0.005f; // 5ms
                if (current_speed > max_speed) current_speed = max_speed;
            }

            float diff = (float)(left_moved + right_moved);
            float sync_p = diff * 0.5f;

            left_motor->run((int)(dir_left * current_speed - sync_p));
            right_motor->run((int)(dir_right * current_speed - sync_p));

            tslp_tsk(5 * 1000); // 5ms
        }

        if (then == Stop::COAST) stop();
        else brake();
    }

    void DriveBase::straight_line(ColorSensor& left_sensor, ColorSensor& right_sensor, float distance_mm, float speed_mm_per_sec, int target_val) {
        int target = (target_val < 0) ? config_target_val : target_val;

        float wheel_circ = wheel_diameter * M_PI;
        float target_deg = fabsf((distance_mm / wheel_circ) * 360.0f);

        int32_t start_left = left_motor->angle();
        int32_t start_right = right_motor->angle();

        float dir = (distance_mm >= 0) ? 1.0f : -1.0f;

        float max_speed = (speed_mm_per_sec / wheel_circ) * 360.0f; 
        float accel = 350.0f;

        float current_speed = min_speed;
        bool pid_mode = false;
        int stable_count = 0;

        float l_integral = 0.0f;
        float r_integral = 0.0f;
        float l_prev_error = 0.0f;
        float r_prev_error = 0.0f;

        while (true) {
            int l_val = left_sensor.reflection();
            int r_val = right_sensor.reflection();

            if (l_val <= target + 10 || r_val <= target + 10) {
                pid_mode = true;
            }

            if (!pid_mode) {
                int32_t cur_left = left_motor->angle();
                int32_t cur_right = right_motor->angle();

                int32_t left_moved = cur_left - start_left;
                int32_t right_moved = cur_right - start_right;
                float avg_moved = (fabsf((float)left_moved) + fabsf((float)right_moved)) / 2.0f;

                float remaining_deg = target_deg - avg_moved;

                if (remaining_deg <= 0) {
                    current_speed = min_speed;
                } else {
                    float decel_dist = (current_speed * current_speed - min_speed * min_speed) / (2.0f * accel);

                    if (remaining_deg <= decel_dist) {
                        current_speed -= accel * 0.005f;
                        if (current_speed < min_speed) current_speed = min_speed;
                    } else if (current_speed < max_speed) {
                        current_speed += accel * 0.005f;
                        if (current_speed > max_speed) current_speed = max_speed;
                    }
                }

                float diff = (float)(left_moved - right_moved);
                float sync_p = diff * 0.5f;

                left_motor->run((int)(dir * current_speed - sync_p));
                right_motor->run((int)(dir * current_speed + sync_p));

            } else {
                float l_error = (float)(l_val - target);
                float r_error = (float)(r_val - target);

                if (abs(l_val - target) <= config_tolerance && abs(r_val - target) <= config_tolerance) {
                    stable_count++;
                    if (stable_count >= 10) { // 50ms 安定で完了
                        break;
                    }
                } else {
                    stable_count = 0;
                }

                l_integral += l_error * 0.005f;
                r_integral += r_error * 0.005f;
                if (l_integral > 30.0f) l_integral = 30.0f;
                if (l_integral < -30.0f) l_integral = -30.0f;
                if (r_integral > 30.0f) r_integral = 30.0f;
                if (r_integral < -30.0f) r_integral = -30.0f;

                float l_derivative = (l_error - l_prev_error) / 0.005f;
                float r_derivative = (r_error - r_prev_error) / 0.005f;

                l_prev_error = l_error;
                r_prev_error = r_error;

                float l_output = (straight_kp * l_error) + (straight_ki * l_integral) + (straight_kd * l_derivative);
                float r_output = (straight_kp * r_error) + (straight_ki * r_integral) + (straight_kd * r_derivative);

                if (l_output > 120.0f) l_output = 120.0f;
                if (l_output < -120.0f) l_output = -120.0f;
                if (r_output > 120.0f) r_output = 120.0f;
                if (r_output < -120.0f) r_output = -120.0f;

                left_motor->run((int)(dir * l_output));
                right_motor->run((int)(dir * r_output));
            }

            tslp_tsk(5 * 1000); // 5ms
        }

        brake();
    }

    void DriveBase::straight_lines(ColorSensor& left_sensor, ColorSensor& right_sensor, int line_count, float speed_mm_per_sec, int target_val) {
        if (line_count <= 0) return;

        int target = (target_val < 0) ? config_target_val : target_val;

        float wheel_circ = wheel_diameter * M_PI;
        float run_speed = (speed_mm_per_sec / wheel_circ) * 360.0f; // deg/s

        int crossed_lines = 0;
        bool in_line_zone = false;

        bool first_detected = false;
        bool first_is_left = false;
        int32_t first_l_angle = 0;
        int32_t first_r_angle = 0;

        int32_t white_start_left = left_motor->angle();
        int32_t white_start_right = right_motor->angle();

        ev3_lcd_set_font(EV3_FONT_MEDIUM);
        ev3_lcd_fill_rect(0, 0, EV3_LCD_WIDTH, EV3_LCD_HEIGHT, EV3_LCD_WHITE);
        ev3_lcd_draw_string("RUNNING LINES...", 0, 20);

        while (true) {
            int l_val = left_sensor.reflection();
            int r_val = right_sensor.reflection();

            bool l_on_black = (l_val <= target);
            bool r_on_black = (r_val <= target);

            // 1. 白領域復帰判定
            if (l_val > target + 8 && r_val > target + 8) {
                if (in_line_zone) {
                    white_start_left = left_motor->angle();
                    white_start_right = right_motor->angle();
                }
                in_line_zone = false;
                first_detected = false;
            }

            // 2. 新しい黒線進入の検出
            if (!in_line_zone && (l_on_black || r_on_black)) {
                in_line_zone = true;
                crossed_lines++;

                if (crossed_lines >= line_count) {
                    straight_line(left_sensor, right_sensor, 0.0f, speed_mm_per_sec, target);
                    return;
                }
            }

            // 3. モータ駆動制御
            if (in_line_zone) {
                if (!first_detected) {
                    first_detected = true;
                    first_l_angle = left_motor->angle();
                    first_r_angle = right_motor->angle();

                    if (l_on_black && !r_on_black) {
                        first_is_left = true;
                        left_motor->run(0);
                        right_motor->run((int)run_speed);
                    } else if (!l_on_black && r_on_black) {
                        first_is_left = false;
                        left_motor->run((int)run_speed);
                        right_motor->run(0);
                    }
                } else {
                    if (first_is_left && !r_on_black) {
                        left_motor->run(0);
                        right_motor->run((int)run_speed);
                    } else if (!first_is_left && !l_on_black) {
                        left_motor->run((int)run_speed);
                        right_motor->run(0);
                    } else {
                        int32_t current_l = left_motor->angle();
                        int32_t current_r = right_motor->angle();

                        int32_t delta_l = fabsf((float)(current_l - first_l_angle));
                        int32_t delta_r = fabsf((float)(current_r - first_r_angle));
                        float delta_deg = (delta_l + delta_r) / 2.0f;

                        float delta_s_mm = (delta_deg / 360.0f) * wheel_circ;
                        float theta_rad = atan2f(delta_s_mm, sensor_width);
                        float corr_wheel_deg = (axle_track * theta_rad / wheel_circ) * 360.0f;

                        if (first_is_left) {
                            left_motor->run_angle((int)run_speed, -(int)corr_wheel_deg, Stop::BRAKE, false);
                        } else {
                            right_motor->run_angle((int)run_speed, -(int)corr_wheel_deg, Stop::BRAKE, false);
                        }

                        white_start_left = left_motor->angle();
                        white_start_right = right_motor->angle();

                        left_motor->run((int)run_speed);
                        right_motor->run((int)run_speed);
                    }
                }
            } else {
                int32_t cur_left = left_motor->angle();
                int32_t cur_right = right_motor->angle();

                int32_t moved_left = cur_left - white_start_left;
                int32_t moved_right = cur_right - white_start_right;

                float diff = (float)(moved_left - moved_right);
                float sync_p = diff * 0.5f;

                left_motor->run((int)(run_speed - sync_p));
                right_motor->run((int)(run_speed + sync_p));
            }

            tslp_tsk(5 * 1000); // 5ms
        }

        brake();
    }

    void DriveBase::drive(float speed_mm_per_sec, float turn_rate_deg_per_sec) {
        float wheel_circ = wheel_diameter * M_PI;
        float speed_deg_s = (speed_mm_per_sec / wheel_circ) * 360.0f;

        if (turn_rate_deg_per_sec == 0.0f) {
            if (!drive_sync_initialized) {
                drive_start_left = left_motor->angle();
                drive_start_right = right_motor->angle();
                drive_sync_initialized = true;
            }

            int32_t cur_left = left_motor->angle();
            int32_t cur_right = right_motor->angle();

            float diff = (float)((cur_left - drive_start_left) - (cur_right - drive_start_right));
            float sync_p = diff * 0.5f;

            left_motor->run((int)(speed_deg_s - sync_p));
            right_motor->run((int)(speed_deg_s + sync_p));
        } else {
            drive_sync_initialized = false;

            float robot_circ = axle_track * M_PI;
            float turn_speed_deg_s = (turn_rate_deg_per_sec * robot_circ / wheel_circ);

            float left_speed = speed_deg_s + (turn_speed_deg_s / 2.0f);
            float right_speed = speed_deg_s - (turn_speed_deg_s / 2.0f);

            left_motor->run((int)left_speed);
            right_motor->run((int)right_speed);
        }
    }

    void DriveBase::stop() {
        drive_sync_initialized = false;
        left_motor->stop();
        right_motor->stop();
    }

    void DriveBase::brake() {
        drive_sync_initialized = false;
        left_motor->brake();
        right_motor->brake();
    }

} // namespace pybricks
