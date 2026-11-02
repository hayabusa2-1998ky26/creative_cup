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
        tslp_tsk(50 * 1000); 
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
            ulong_t sleep_us = (ulong_t)(DriveConfig::period_ms * 1000.0f);
            while (true) {
                int32_t current = angle();
                if (angle_deg > 0 && current >= target_ang) break;
                if (angle_deg < 0 && current <= target_ang) break;
                tslp_tsk(sleep_us);
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

    ColorSensor::ColorSensor(sensor_port_t p, int min_v, int max_v) 
        : port(p), min_val(min_v), max_val(max_v) {
        ev3_sensor_config(port, COLOR_SENSOR);
        tslp_tsk(500 * 1000); 
    }

    void ColorSensor::set_calibration(int min_v, int max_v) {
        min_val = min_v;
        max_val = max_v;
    }

    int ColorSensor::reflection() { 
        return ev3_color_sensor_get_reflect(port); 
    }

    float ColorSensor::normalized_reflection() {
        int raw = reflection();
        if (max_val == min_val) return 0.0f;
        
        float norm = (float)(raw - min_val) / (float)(max_val - min_val);
        if (norm < 0.0f) norm = 0.0f;
        if (norm > 1.0f) norm = 1.0f;
        return norm;
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

    DriveBase::DriveBase(Motor& left, Motor& right, ColorSensor& left_sens, ColorSensor& right_sens)
        : left_motor(&left), right_motor(&right), left_sensor(&left_sens), right_sensor(&right_sens) {
        
        wheel_diameter = DriveConfig::wheel_diameter;
        axle_track = DriveConfig::axle_track;

        control_period_ms = DriveConfig::period_ms;
        min_speed = DriveConfig::min_speed;
        decel_margin = DriveConfig::decel_margin;
        line_blind_dist = DriveConfig::line_blind_dist;
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

        pose_x = 0.0f;
        pose_y = 0.0f;
        pose_theta = 0.0f;
        prev_left_enc = 0;
        prev_right_enc = 0;

        set_sensor_coordinates(axle_track / 2.0f, 50.0f);
    }

    void DriveBase::run_motors(float left_target, float right_target) {
        left_motor->run((int)left_target);
        right_motor->run((int)right_target);
    }

    float DriveBase::calculate_sync_p(int32_t left_moved, int32_t right_moved, bool is_turn) {
        float diff = is_turn ? (float)(left_moved + right_moved) : (float)(left_moved - right_moved);

        static float integral = 0.0f;
        static float prev_diff = 0.0f;

        if (left_moved == 0 && right_moved == 0) {
            integral = 0.0f;
            prev_diff = 0.0f;
        }

        float dt = control_period_ms / 1000.0f;
        if (dt <= 0.0f) dt = 0.010f;

        const float Kp = 1.20f;  
        const float Ki = 0.05f;  
        const float Kd = 0.04f;  

        float p_term = Kp * diff;

        integral += diff * dt;
        if (integral > 30.0f)  integral = 30.0f;
        if (integral < -30.0f) integral = -30.0f;
        float i_term = Ki * integral;

        float derivative = (diff - prev_diff) / dt;
        float d_term = Kd * derivative;
        prev_diff = diff;

        float sync_p = p_term + i_term + d_term;

        const float MAX_SYNC_OUTPUT = 150.0f;
        if (sync_p > MAX_SYNC_OUTPUT)  sync_p = MAX_SYNC_OUTPUT;
        if (sync_p < -MAX_SYNC_OUTPUT) sync_p = -MAX_SYNC_OUTPUT;

        return sync_p;
    }

    void DriveBase::reset_odometry() {
        pose_x = 0.0f;
        pose_y = 0.0f;
        pose_theta = 0.0f;
        prev_left_enc = left_motor->angle();
        prev_right_enc = right_motor->angle();
    }

    void DriveBase::update_odometry(int32_t cur_left, int32_t cur_right) {
        float wheel_circ = wheel_diameter * M_PI;
        
        float delta_l_mm = ((float)(cur_left - prev_left_enc) / 360.0f) * wheel_circ;
        float delta_r_mm = ((float)(cur_right - prev_right_enc) / 360.0f) * wheel_circ;

        prev_left_enc = cur_left;
        prev_right_enc = cur_right;

        float delta_s = (delta_l_mm + delta_r_mm) / 2.0f;
        float delta_theta = (delta_r_mm - delta_l_mm) / axle_track;

        float mid_theta = pose_theta + (delta_theta / 2.0f);
        pose_x += delta_s * cosf(mid_theta);
        pose_y += delta_s * sinf(mid_theta);
        pose_theta += delta_theta;

        while (pose_theta > M_PI)  pose_theta -= 2.0f * (float)M_PI;
        while (pose_theta < -M_PI) pose_theta += 2.0f * (float)M_PI;
    }

    float DriveBase::calculate_lookahead_heading_pd(float lookahead_dist_mm, float dir) {
        float lookahead_sign = (dir >= 0.0f) ? 1.0f : -1.0f;
        float target_x = pose_x + (lookahead_dist_mm * lookahead_sign);
        float target_y = 0.0f; 

        float target_theta = atan2f(target_y - pose_y, target_x - pose_x);
        
        if (lookahead_sign < 0.0f) {
            target_theta += (float)M_PI;
        }

        float error_theta = target_theta - pose_theta;

        while (error_theta > M_PI)  error_theta -= 2.0f * (float)M_PI;
        while (error_theta < -M_PI) error_theta += 2.0f * (float)M_PI;

        static float prev_error = 0.0f;
        float dt = control_period_ms / 1000.0f;
        if (dt <= 0.0f) dt = 0.010f;

        const float Kp = 2.0f;   
        const float Kd = 0.1f;   

        float derivative = (error_theta - prev_error) / dt;
        prev_error = error_theta;

        float u = (Kp * error_theta) + (Kd * derivative);

        const float MAX_U = 0.30f;
        if (u > MAX_U)  u = MAX_U;
        if (u < -MAX_U) u = -MAX_U;

        return u;
    }

    void DriveBase::run_straight_sync(float base_speed, int32_t cur_left, int32_t cur_right, float lookahead_dist) {
        update_odometry(cur_left, cur_right);

        float dir = (base_speed >= 0.0f) ? 1.0f : -1.0f;
        float u = calculate_lookahead_heading_pd(lookahead_dist, dir);
        float turn_power = fabsf(base_speed) * u;

        float left_target  = base_speed - turn_power;
        float right_target = base_speed + turn_power;

        run_motors(left_target, right_target);
    }

    void DriveBase::set_sensor_coordinates(float sensor_x_mm, float sensor_y_mm) {
        sensor_x = fabsf(sensor_x_mm);
        sensor_y = sensor_y_mm;
    }

    void DriveBase::settings(float straight_speed, float straight_acceleration, float turn_rate, float turn_acceleration) {
        default_straight_speed = straight_speed;
        default_straight_accel = straight_acceleration;
        default_turn_rate = turn_rate;
        default_turn_accel = turn_acceleration;
    }

    void DriveBase::straight(float distance_mm, Stop then, bool wait) {
        if (fabsf(distance_mm) < 1.0f) return;

        float dt = control_period_ms / 1000.0f;
        ulong_t sleep_us = (ulong_t)(control_period_ms * 1000.0f);

        float wheel_circ = wheel_diameter * M_PI;
        float target_deg = fabsf((distance_mm / wheel_circ) * 360.0f);

        int32_t start_left = left_motor->angle();
        int32_t start_right = right_motor->angle();

        reset_odometry();

        ev3_lcd_set_font(EV3_FONT_MEDIUM);
        ev3_lcd_fill_rect(0, 0, EV3_LCD_WIDTH, EV3_LCD_HEIGHT, EV3_LCD_WHITE);
        ev3_lcd_draw_string("STRAIGHT MONITOR", 0, 0);

        float dir = (distance_mm >= 0) ? 1.0f : -1.0f;

        float max_speed = (default_straight_speed / wheel_circ) * 360.0f; 
        float accel = (default_straight_accel / wheel_circ) * 360.0f; 

        float current_speed = min_speed;
        int32_t max_abs_diff = 0;

        while (true) {
            int32_t cur_left = left_motor->angle();
            int32_t cur_right = right_motor->angle();

            int32_t left_moved = cur_left - start_left;
            int32_t right_moved = cur_right - start_right;
            float avg_moved = (fabsf((float)left_moved) + fabsf((float)right_moved)) / 2.0f;

            int32_t current_diff = left_moved - right_moved;
            int32_t abs_diff = labs(current_diff);
            if (abs_diff > max_abs_diff) {
                max_abs_diff = abs_diff;
            }

            char buf1[32], buf2[32], buf3[32];
            snprintf(buf1, sizeof(buf1), "L_deg: %ld deg", (long)left_moved);
            snprintf(buf2, sizeof(buf2), "R_deg: %ld deg", (long)right_moved);
            snprintf(buf3, sizeof(buf3), "MaxDiff: %ld deg", (long)max_abs_diff);

            ev3_lcd_draw_string(buf1, 0, 25);
            ev3_lcd_draw_string(buf2, 0, 45);
            ev3_lcd_draw_string(buf3, 0, 65);

            float remaining_deg = target_deg - avg_moved;
            if (remaining_deg <= 0) break;

            float decel_dist = 0.0f;
            if (current_speed > min_speed) {
                decel_dist = ((current_speed * current_speed - min_speed * min_speed) / (2.0f * accel)) * decel_margin;
            }

            if (remaining_deg <= decel_dist) {
                current_speed -= accel * dt;
                if (current_speed < min_speed) current_speed = min_speed;
            } else if (current_speed < max_speed) {
                current_speed += accel * dt;
                if (current_speed > max_speed) current_speed = max_speed;
            }

            float base_speed = dir * current_speed;

            run_straight_sync(base_speed, cur_left, cur_right);

            tslp_tsk(sleep_us);
        }

        if (then == Stop::COAST) stop();
        else brake();
    }

    void DriveBase::turn(float angle_deg, Stop then, bool wait) {
        if (fabsf(angle_deg) < 1.0f) return;

        float dt = control_period_ms / 1000.0f;
        ulong_t sleep_us = (ulong_t)(control_period_ms * 1000.0f);

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

            float decel_dist = 0.0f;
            if (current_speed > min_speed) {
                decel_dist = ((current_speed * current_speed - min_speed * min_speed) / (2.0f * accel)) * decel_margin;
            }

            if (remaining_deg <= decel_dist) {
                current_speed -= accel * dt;
                if (current_speed < min_speed) current_speed = min_speed;
            } else if (current_speed < max_speed) {
                current_speed += accel * dt;
                if (current_speed > max_speed) current_speed = max_speed;
            }

            float sync_p = calculate_sync_p(left_moved, right_moved, true);

            run_motors(dir_left * current_speed - sync_p, dir_right * current_speed - sync_p);

            tslp_tsk(sleep_us);
        }

        if (then == Stop::COAST) stop();
        else brake();
    }

    void DriveBase::wide_forward(float angle_deg, float radius_mm, Stop then, bool wait) {
        float conf_axle_track = DriveConfig::axle_track;
        float conf_wheel_diameter = DriveConfig::wheel_diameter;

        if (radius_mm < conf_axle_track / 2.0f) {
            radius_mm = conf_axle_track / 2.0f;
        }

        if (fabsf(angle_deg) < 1.0f) return;

        float dt = DriveConfig::period_ms / 1000.0f;
        ulong_t sleep_us = (ulong_t)(DriveConfig::period_ms * 1000.0f);

        bool turn_right = (angle_deg >= 0.0f);
        
        float radius_l = turn_right ? (radius_mm + conf_axle_track / 2.0f) : (radius_mm - conf_axle_track / 2.0f);
        float radius_r = turn_right ? (radius_mm - conf_axle_track / 2.0f) : (radius_mm + conf_axle_track / 2.0f);

        float max_radius = fmaxf(fabsf(radius_l), fabsf(radius_r));

        float ratio_l = radius_l / max_radius;
        float ratio_r = radius_r / max_radius;

        float wheel_circ = conf_wheel_diameter * (float)M_PI;
        float outer_circ = max_radius * 2.0f * (float)M_PI;
        float outer_dist_mm = (fabsf(angle_deg) / 360.0f) * outer_circ;
        float target_deg = (outer_dist_mm / wheel_circ) * 360.0f; 

        float max_speed = DriveConfig::turn_rate * (conf_axle_track / conf_wheel_diameter);
        float accel = DriveConfig::turn_accel * (conf_axle_track / conf_wheel_diameter);

        float current_speed = DriveConfig::min_speed;
        float expected_outer_deg = 0.0f;

        int32_t start_left = left_motor->angle();
        int32_t start_right = right_motor->angle();

        while (true) {
            int32_t cur_left = left_motor->angle();
            int32_t cur_right = right_motor->angle();

            float left_moved = (float)(cur_left - start_left);
            float right_moved = (float)(cur_right - start_right);

            float actual_outer_moved;
            if (ratio_l > ratio_r) {
                actual_outer_moved = left_moved / ratio_l; 
            } else {
                actual_outer_moved = right_moved / ratio_r; 
            }

            float remaining_deg = target_deg - actual_outer_moved;
            if (remaining_deg <= 0.0f) break;

            float decel_dist = 0.0f;
            if (current_speed > DriveConfig::min_speed) {
                decel_dist = ((current_speed * current_speed - DriveConfig::min_speed * DriveConfig::min_speed) / (2.0f * accel)) * DriveConfig::decel_margin;
            }

            if (remaining_deg <= decel_dist) {
                current_speed -= accel * dt;
                if (current_speed < DriveConfig::min_speed) current_speed = DriveConfig::min_speed;
            } else if (current_speed < max_speed) {
                current_speed += accel * dt;
                if (current_speed > max_speed) current_speed = max_speed;
            }

            expected_outer_deg += current_speed * dt;
            if (expected_outer_deg > target_deg) {
                expected_outer_deg = target_deg;
            }

            float expected_l = expected_outer_deg * ratio_l;
            float expected_r = expected_outer_deg * ratio_r;

            float err_l = expected_l - left_moved;
            float err_r = expected_r - right_moved;

            float Kp = 2.0f; 
            
            float left_target = (current_speed * ratio_l) + Kp * err_l;
            float right_target = (current_speed * ratio_r) + Kp * err_r;

            run_motors(left_target, right_target);

            tslp_tsk(sleep_us);
        }

        if (then == Stop::COAST) stop();
        else brake();
    }

    // ★変更: 引数 skip_initial_ignore を追加
    void DriveBase::straight_line(float distance_mm, float speed_mm_per_sec, float target_val, bool skip_white_exit_check, bool reverse, bool skip_initial_ignore) {
        float target = (target_val < 0.0f) ? config_target_val : target_val;
        
        float run_speed_mm = (speed_mm_per_sec == 0.0f) ? DriveConfig::straight_speed : fabsf(speed_mm_per_sec);

        float dt = control_period_ms / 1000.0f;
        ulong_t sleep_us = (ulong_t)(control_period_ms * 1000.0f);

        float wheel_circ = wheel_diameter * M_PI;
        float target_deg = fabsf((distance_mm / wheel_circ) * 360.0f);

        int32_t start_left = left_motor->angle();
        int32_t start_right = right_motor->angle();

        reset_odometry();

        float dir = 1.0f;
        if (distance_mm != 0.0f) {
            dir = (distance_mm > 0.0f) ? 1.0f : -1.0f;
        } else {
            dir = reverse ? -1.0f : 1.0f;
        }

        float max_speed = (run_speed_mm / wheel_circ) * 360.0f; 
        float accel = (DriveConfig::straight_accel / wheel_circ) * 360.0f;

        float current_speed = min_speed;
        bool pid_mode = false;
        int stable_count = 0;
        int target_stable_count = (int)(0.050f / dt);

        float l_integral = 0.0f;
        float r_integral = 0.0f;
        float l_prev_error = 0.0f;
        float r_prev_error = 0.0f;

        float filtered_l_val = left_sensor->normalized_reflection();
        float filtered_r_val = right_sensor->normalized_reflection();

        bool waiting_for_white_exit = false;
        bool white_exit_initialized = false;

        // ★ skip_initial_ignore が true の場合は無視距離を 0mm に設定
        float ignore_dist = skip_initial_ignore ? 0.0f : DriveConfig::initial_ignore_dist;

        while (true) {
            int32_t cur_left = left_motor->angle();
            int32_t cur_right = right_motor->angle();

            int32_t left_moved = cur_left - start_left;
            int32_t right_moved = cur_right - start_right;
            float avg_moved_deg = (fabsf((float)left_moved) + fabsf((float)right_moved)) / 2.0f;
            float moved_mm = (avg_moved_deg / 360.0f) * wheel_circ;

            float l_val = left_sensor->normalized_reflection();
            float r_val = right_sensor->normalized_reflection();

            filtered_l_val += DriveConfig::color_lpf_gain * (l_val - filtered_l_val);
            filtered_r_val += DriveConfig::color_lpf_gain * (r_val - filtered_r_val);

            // ★ ignore_dist (0mm または 20mm) 以上進んだら有効化
            if (moved_mm >= ignore_dist) {
                if (!white_exit_initialized) {
                    if (!skip_white_exit_check) {
                        waiting_for_white_exit = (l_val <= target + 0.2f || r_val <= target + 0.2f);
                    }
                    white_exit_initialized = true;
                }

                if (waiting_for_white_exit) {
                    if (l_val > target + 0.16f && r_val > target + 0.16f) {
                        waiting_for_white_exit = false;
                    }
                } else {
                    if (!pid_mode && (l_val <= target + 0.2f || r_val <= target + 0.2f)) {
                        ev3_speaker_play_tone(1000, 50);
                        pid_mode = true;
                    }
                }
            }

            if (!pid_mode) {
                float remaining_deg = target_deg - avg_moved_deg;

                float decel_dist = 0.0f;
                if (current_speed > min_speed) {
                    decel_dist = ((current_speed * current_speed - min_speed * min_speed) / (2.0f * accel)) * decel_margin;
                }

                if (distance_mm != 0.0f && remaining_deg <= decel_dist) {
                    current_speed -= accel * dt;
                    if (current_speed < min_speed) current_speed = min_speed;
                } else if (current_speed < max_speed) {
                    current_speed += accel * dt;
                    if (current_speed > max_speed) current_speed = max_speed;
                }

                run_straight_sync(dir * current_speed, cur_left, cur_right);

            } else {
                float l_error = filtered_l_val - target;
                float r_error = filtered_r_val - target;

                float total_error = fabsf(l_error) + fabsf(r_error);
                
                if (total_error <= config_tolerance) {
                    stable_count++;
                    if (stable_count >= target_stable_count) {
                        break;
                    }
                } else {
                    stable_count = 0;
                }

                l_integral += l_error * dt;
                r_integral += r_error * dt;
                if (l_integral > 30.0f) l_integral = 30.0f;
                if (l_integral < -30.0f) l_integral = -30.0f;
                if (r_integral > 30.0f) r_integral = 30.0f;
                if (r_integral < -30.0f) r_integral = -30.0f;

                float l_derivative = (l_error - l_prev_error) / dt;
                float r_derivative = (r_error - r_prev_error) / dt;

                l_prev_error = l_error;
                r_prev_error = r_error;

                float l_output = (straight_kp * l_error) + (straight_ki * l_integral) + (straight_kd * l_derivative);
                float r_output = (straight_kp * r_error) + (straight_ki * r_integral) + (straight_kd * r_derivative);

                float max_out = DriveConfig::align_max_power;
                if (l_output > max_out) l_output = max_out;
                if (l_output < -max_out) l_output = -max_out;
                if (r_output > max_out) r_output = max_out;
                if (r_output < -max_out) r_output = -max_out;

                float align_dir = (dir >= 0.0f) ? 1.0f : -1.0f;
                run_motors(align_dir * l_output, align_dir * r_output);
            }

            tslp_tsk(sleep_us);
        }

        brake();
    }

    void DriveBase::straight_lines(int line_count, float distance_mm, void (*func)(), float func_distance, float speed_mm_per_sec, float target_val) {
        if (line_count == 0) return;

        float dir = (line_count > 0) ? 1.0f : -1.0f;
        int target_lines = abs(line_count);

        float target = (target_val < 0.0f) ? config_target_val : target_val;
        
        float run_speed_mm = (speed_mm_per_sec <= 0.0f) ? DriveConfig::straight_speed : speed_mm_per_sec;

        float dt = control_period_ms / 1000.0f;
        ulong_t sleep_us = (ulong_t)(control_period_ms * 1000.0f);

        float wheel_circ = wheel_diameter * M_PI;
        float target_run_speed = (run_speed_mm / wheel_circ) * 360.0f;
        
        float accel = (DriveConfig::straight_accel / wheel_circ) * 360.0f;
        float target_deg = fabsf((distance_mm / wheel_circ) * 360.0f);

        int32_t start_left = left_motor->angle();
        int32_t start_right = right_motor->angle();

        reset_odometry();

        float current_speed = min_speed;
        int crossed_lines = 0;

        bool l_in_line = false, r_in_line = false;
        int32_t l_enter_deg = 0, l_exit_deg = 0;
        int32_t r_enter_deg = 0, r_exit_deg = 0;

        bool in_line_zone = false;
        bool align_processing = false;

        bool has_detected_any_line = false;
        int32_t last_line_detected_l = start_left;
        int32_t last_line_detected_r = start_right;

        // ★ 白抜け判定の初期化を20mm走順まで保留
        bool waiting_for_white_exit = false;
        bool white_exit_initialized = false;

        bool callback_pending = false;
        int32_t callback_start_deg = 0;

        ev3_lcd_set_font(EV3_FONT_MEDIUM);
        ev3_lcd_fill_rect(0, 0, EV3_LCD_WIDTH, EV3_LCD_HEIGHT, EV3_LCD_WHITE);
        ev3_lcd_draw_string("RUNNING LINES...", 0, 20);

        while (true) {
            int32_t cur_left_total = left_motor->angle();
            int32_t cur_right_total = right_motor->angle();
            int32_t cur_forward = (cur_left_total + cur_right_total) / 2;

            int32_t total_left_moved = cur_left_total - start_left;
            int32_t total_right_moved = cur_right_total - start_right;
            float total_avg_moved = (fabsf((float)total_left_moved) + fabsf((float)total_right_moved)) / 2.0f;
            float total_moved_mm = (total_avg_moved / 360.0f) * wheel_circ;
            float remaining_deg = target_deg - total_avg_moved;

            float decel_dist = 0.0f;
            if (current_speed > min_speed) {
                decel_dist = ((current_speed * current_speed - min_speed * min_speed) / (2.0f * accel)) * decel_margin;
            }

            if (fabsf(distance_mm) > 0.0f && remaining_deg <= decel_dist) {
                current_speed -= accel * dt;
                if (current_speed < min_speed) current_speed = min_speed;
            } else if (current_speed < target_run_speed) {
                current_speed += accel * dt;
                if (current_speed > target_run_speed) current_speed = target_run_speed;
            }

            float l_val = left_sensor->normalized_reflection();
            float r_val = right_sensor->normalized_reflection();
            bool l_on_black = (l_val <= target);
            bool r_on_black = (r_val <= target);

            // ★ メソッド開始から 20mm 以上進んでから各種検知処理を有効化
            if (total_moved_mm >= DriveConfig::initial_ignore_dist) {
                if (!white_exit_initialized) {
                    waiting_for_white_exit = (l_on_black || r_on_black);
                    white_exit_initialized = true;
                }

                if (waiting_for_white_exit) {
                    if (l_val > target + 0.16f && r_val > target + 0.16f) {
                        waiting_for_white_exit = false;
                        has_detected_any_line = true;
                        last_line_detected_l = cur_left_total;
                        last_line_detected_r = cur_right_total;
                    }
                } else {
                    float moved_from_last_line_deg = (fabsf((float)(cur_left_total - last_line_detected_l)) + 
                                                      fabsf((float)(cur_right_total - last_line_detected_r))) / 2.0f;
                    float moved_from_last_line_mm = (moved_from_last_line_deg / 360.0f) * wheel_circ;
                    bool blind_ok = !has_detected_any_line || (moved_from_last_line_mm >= line_blind_dist);

                    if (!in_line_zone && blind_ok && (l_on_black || r_on_black)) {
                        in_line_zone = true;
                        crossed_lines++;
                        has_detected_any_line = true;
                        last_line_detected_l = cur_left_total;
                        last_line_detected_r = cur_right_total;

                        ev3_speaker_play_tone(1000, 50);

                        // DriveBase::straight_lines の最終線到達時の処理
                        if (crossed_lines >= target_lines) {
                            bool is_reverse = (dir < 0.0f);
                            // ★ 第6引数 (skip_initial_ignore) に true を指定して呼び出す
                            straight_line(0.0f, run_speed_mm, target, true, is_reverse, true);
                            return;
                        } else {
                            if (func != nullptr) {
                                callback_pending = true;
                                callback_start_deg = cur_forward;
                            }
                        }

                        align_processing = true;
                        l_in_line = l_on_black;
                        r_in_line = r_on_black;
                        l_enter_deg = l_on_black ? cur_forward : 0;
                        r_enter_deg = r_on_black ? cur_forward : 0;
                        l_exit_deg = 0;
                        r_exit_deg = 0;
                    }
                    else if (in_line_zone && align_processing) {
                        if (l_on_black && !l_in_line) {
                            l_in_line = true;
                            l_enter_deg = cur_forward;
                        } else if (!l_on_black && l_in_line) {
                            l_in_line = false;
                            l_exit_deg = cur_forward;
                        }

                        if (r_on_black && !r_in_line) {
                            r_in_line = true;
                            r_enter_deg = cur_forward;
                        } else if (!r_on_black && r_in_line) {
                            r_in_line = false;
                            r_exit_deg = cur_forward;
                        }

                        if (!l_on_black && !r_on_black && l_exit_deg != 0 && r_exit_deg != 0) {
                            int32_t l_center = (l_enter_deg + l_exit_deg) / 2;
                            int32_t r_center = (r_enter_deg + r_exit_deg) / 2;

                            int32_t center_diff = l_center - r_center;
                            float center_diff_mm = ((float)center_diff / 360.0f) * (wheel_diameter * (float)M_PI);

                            float sensor_width_total = 2.0f * sensor_x;

                            float angle_error_rad = asinf(center_diff_mm / sensor_width_total);
                            pose_theta = angle_error_rad; 
                            
                            pose_y = -sensor_y * sinf(pose_theta);

                            align_processing = false;
                            in_line_zone = false;
                            l_exit_deg = 0;
                            r_exit_deg = 0;
                        }
                    }
                }
            }

            if (callback_pending) {
                float moved_deg = fabsf((float)(cur_forward - callback_start_deg));
                float target_callback_deg = (fabsf(func_distance) / wheel_circ) * 360.0f;
                if (moved_deg >= target_callback_deg) {
                    func();
                    callback_pending = false;
                }
            }

            run_straight_sync(dir * current_speed, cur_left_total, cur_right_total);

            tslp_tsk(sleep_us);
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

            int32_t moved_left = cur_left - drive_start_left;
            int32_t moved_right = cur_right - drive_start_right;

            float sync_p = calculate_sync_p(moved_left, moved_right);

            run_motors(speed_deg_s - sync_p, speed_deg_s + sync_p);
        } else {
            drive_sync_initialized = false;

            float robot_circ = axle_track * M_PI;
            float turn_speed_deg_s = (turn_rate_deg_per_sec * robot_circ / wheel_circ);

            float left_speed = speed_deg_s + (turn_speed_deg_s / 2.0f);
            float right_speed = speed_deg_s - (turn_speed_deg_s / 2.0f);

            run_motors(left_speed, right_speed);
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
