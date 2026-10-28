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

    ColorSensor::ColorSensor(sensor_port_t p) : port(p) {
        ev3_sensor_config(port, COLOR_SENSOR);
        tslp_tsk(500 * 1000); 
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

        set_sensor_coordinates(-axle_track_dist / 2.0f, 50.0f, axle_track_dist / 2.0f, 50.0f);
    }

    // すべてのモーター出力を一元管理し、右モーターにFF比率を適用する関数
    void DriveBase::run_motors(float left_target, float right_target) {
        left_motor->run((int)left_target);
        right_motor->run((int)(right_target * DriveConfig::motor_ff_ratio));
    }

    // 左右同期計算関数（turnやstraight_linesなど各種操作用）
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

    // オドメトリのリセット
    void DriveBase::reset_odometry() {
        pose_x = 0.0f;
        pose_y = 0.0f;
        pose_theta = 0.0f;
        prev_left_enc = left_motor->angle();
        prev_right_enc = right_motor->angle();
    }

    // 毎周期呼び出す自己位置推定（オドメトリ更新）
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

    // ★ 乗算型補正用の比率 u を出力する前方注視点PD制御
    float DriveBase::calculate_lookahead_heading_pd(float lookahead_dist_mm) {
        float target_x = pose_x + lookahead_dist_mm;
        float target_y = 0.0f; // 直進ライン(y=0)

        float target_theta = atan2f(target_y - pose_y, target_x - pose_x);
        float error_theta = target_theta - pose_theta;

        while (error_theta > M_PI)  error_theta -= 2.0f * (float)M_PI;
        while (error_theta < -M_PI) error_theta += 2.0f * (float)M_PI;

        static float prev_error = 0.0f;
        float dt = control_period_ms / 1000.0f;
        if (dt <= 0.0f) dt = 0.010f;

        // --- 乗算型(比率) 用ゲイン ---
        // 角度偏差(rad) に対する 速度補正比率 u (例: 0.1 で 10% 補正)
        const float Kp = 2.0f;   
        const float Kd = 0.1f;   

        float derivative = (error_theta - prev_error) / dt;
        prev_error = error_theta;

        float u = (Kp * error_theta) + (Kd * derivative);

        // 補正比率の最大値を ±30% (0.3) に制限
        const float MAX_U = 0.30f;
        if (u > MAX_U)  u = MAX_U;
        if (u < -MAX_U) u = -MAX_U;

        return u;
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

    // ★ 車速依存スケーリング（乗算型補正）適用版 straight
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

            update_odometry(cur_left, cur_right);

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

            // ★ 補正比率 u を取得 (最大 ±0.30)
            float u = calculate_lookahead_heading_pd(80.0f);

            // ★ 車速依存スケーリング（乗算型）によるモーター出力の合成
            // base_speed が大きければ大きな差分、小さくなれば自動的に小さな差分になる
            float left_target  = base_speed * (1.0f - u);
            float right_target = base_speed * (1.0f + u);

            run_motors(left_target, right_target);

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

    void DriveBase::straight_line(ColorSensor& left_sensor, ColorSensor& right_sensor, float distance_mm, float speed_mm_per_sec, int target_val, bool skip_white_exit_check) {
        int target = (target_val < 0) ? config_target_val : target_val;
        float run_speed_mm = (speed_mm_per_sec <= 0.0f) ? default_straight_speed : speed_mm_per_sec;

        float dt = control_period_ms / 1000.0f;
        ulong_t sleep_us = (ulong_t)(control_period_ms * 1000.0f);

        float wheel_circ = wheel_diameter * M_PI;
        float target_deg = fabsf((distance_mm / wheel_circ) * 360.0f);

        int32_t start_left = left_motor->angle();
        int32_t start_right = right_motor->angle();

        float dir = (distance_mm >= 0) ? 1.0f : -1.0f;

        float max_speed = (run_speed_mm / wheel_circ) * 360.0f; 
        float accel = (default_straight_accel / wheel_circ) * 360.0f;

        float current_speed = min_speed;
        bool pid_mode = false;
        int stable_count = 0;
        int target_stable_count = (int)(0.050f / dt);

        float l_integral = 0.0f;
        float r_integral = 0.0f;
        float l_prev_error = 0.0f;
        float r_prev_error = 0.0f;

        bool waiting_for_white_exit = false;
        if (!skip_white_exit_check) {
            int init_l = left_sensor.reflection();
            int init_r = right_sensor.reflection();
            waiting_for_white_exit = (init_l <= target + 10 || init_r <= target + 10);
        }

        while (true) {
            int l_val = left_sensor.reflection();
            int r_val = right_sensor.reflection();

            if (waiting_for_white_exit) {
                if (l_val > target + 8 && r_val > target + 8) {
                    waiting_for_white_exit = false;
                }
            } else {
                if (!pid_mode && (l_val <= target + 10 || r_val <= target + 10)) {
                    ev3_speaker_play_tone(1000, 50);
                    pid_mode = true;
                }
            }

            if (!pid_mode) {
                int32_t cur_left = left_motor->angle();
                int32_t cur_right = right_motor->angle();

                int32_t left_moved = cur_left - start_left;
                int32_t right_moved = cur_right - start_right;
                float avg_moved = (fabsf((float)left_moved) + fabsf((float)right_moved)) / 2.0f;

                float remaining_deg = target_deg - avg_moved;

                float decel_dist = 0.0f;
                if (current_speed > min_speed) {
                    decel_dist = ((current_speed * current_speed - min_speed * min_speed) / (2.0f * accel)) * decel_margin;
                }

                if (distance_mm > 0.0f && remaining_deg <= decel_dist) {
                    current_speed -= accel * dt;
                    if (current_speed < min_speed) current_speed = min_speed;
                } else if (current_speed < max_speed) {
                    current_speed += accel * dt;
                    if (current_speed > max_speed) current_speed = max_speed;
                }

                float sync_p = calculate_sync_p(left_moved, right_moved);

                run_motors(dir * current_speed - sync_p, dir * current_speed + sync_p);

            } else {
                float l_error = (float)(l_val - target);
                float r_error = (float)(r_val - target);

                if (abs(l_val - target) <= config_tolerance && abs(r_val - target) <= config_tolerance) {
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

                if (l_output > 120.0f) l_output = 120.0f;
                if (l_output < -120.0f) l_output = -120.0f;
                if (r_output > 120.0f) r_output = 120.0f;
                if (r_output < -120.0f) r_output = -120.0f;

                run_motors(dir * l_output, dir * r_output);
            }

            tslp_tsk(sleep_us);
        }

        brake();
    }

    void DriveBase::straight_lines(ColorSensor& left_sensor, ColorSensor& right_sensor, int line_count, float distance_mm, float speed_mm_per_sec, int target_val) {
        if (line_count <= 0) return;

        int target = (target_val < 0) ? config_target_val : target_val;
        float run_speed_mm = (speed_mm_per_sec <= 0.0f) ? default_straight_speed : speed_mm_per_sec;

        float dt = control_period_ms / 1000.0f;
        ulong_t sleep_us = (ulong_t)(control_period_ms * 1000.0f);

        float wheel_circ = wheel_diameter * M_PI;
        float target_run_speed = (run_speed_mm / wheel_circ) * 360.0f;
        float accel = (default_straight_accel / wheel_circ) * 360.0f;

        float target_deg = fabsf((distance_mm / wheel_circ) * 360.0f);

        int32_t start_left = left_motor->angle();
        int32_t start_right = right_motor->angle();

        float current_speed = min_speed;

        int crossed_lines = 0;
        bool in_line_zone = false;

        bool first_detected = false;
        bool first_is_left = false;
        bool alignment_done = false; 

        int32_t white_start_left = start_left;
        int32_t white_start_right = start_right;

        bool has_detected_any_line = false;
        int32_t last_line_detected_l = start_left;
        int32_t last_line_detected_r = start_right;

        int init_l = left_sensor.reflection();
        int init_r = right_sensor.reflection();
        bool waiting_for_white_exit = (init_l <= target || init_r <= target);

        ev3_lcd_set_font(EV3_FONT_MEDIUM);
        ev3_lcd_fill_rect(0, 0, EV3_LCD_WIDTH, EV3_LCD_HEIGHT, EV3_LCD_WHITE);
        ev3_lcd_draw_string("RUNNING LINES...", 0, 20);

        while (true) {
            int32_t cur_left_total = left_motor->angle();
            int32_t cur_right_total = right_motor->angle();

            int32_t total_left_moved = cur_left_total - start_left;
            int32_t total_right_moved = cur_right_total - start_right;
            float total_avg_moved = (fabsf((float)total_left_moved) + fabsf((float)total_right_moved)) / 2.0f;

            float remaining_deg = target_deg - total_avg_moved;

            float decel_dist = 0.0f;
            if (current_speed > min_speed) {
                decel_dist = ((current_speed * current_speed - min_speed * min_speed) / (2.0f * accel)) * decel_margin;
            }

            if (distance_mm > 0.0f && remaining_deg <= decel_dist) {
                current_speed -= accel * dt;
                if (current_speed < min_speed) current_speed = min_speed;
            } else if (current_speed < target_run_speed) {
                current_speed += accel * dt;
                if (current_speed > target_run_speed) current_speed = target_run_speed;
            }

            int l_val = left_sensor.reflection();
            int r_val = right_sensor.reflection();

            bool l_on_black = (l_val <= target);
            bool r_on_black = (r_val <= target);

            if (waiting_for_white_exit) {
                if (l_val > target + 8 && r_val > target + 8) {
                    waiting_for_white_exit = false;
                    white_start_left = cur_left_total;
                    white_start_right = cur_right_total;

                    has_detected_any_line = true;
                    last_line_detected_l = cur_left_total;
                    last_line_detected_r = cur_right_total;
                }
            } else {
                if (l_val > target + 8 && r_val > target + 8) {
                    if (in_line_zone) {
                        white_start_left = left_motor->angle();
                        white_start_right = right_motor->angle();
                    }
                    in_line_zone = false;
                    first_detected = false;
                    alignment_done = false; 
                }

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

                    if (crossed_lines >= line_count) {
                        straight_line(left_sensor, right_sensor, 0.0f, run_speed_mm, target, true);
                        return;
                    }
                }
            }

            if (in_line_zone && !waiting_for_white_exit) {
                if (!alignment_done) {
                    if (!first_detected) {
                        first_detected = true;

                        if (l_on_black && !r_on_black) {
                            first_is_left = true;
                            run_motors(0, current_speed);
                        } else if (!l_on_black && r_on_black) {
                            first_is_left = false;
                            run_motors(current_speed, 0);
                        } else {
                            alignment_done = true;
                            white_start_left = left_motor->angle();
                            white_start_right = right_motor->angle();
                            run_motors(current_speed, current_speed);
                        }
                    } else {
                        if (first_is_left && !r_on_black) {
                            run_motors(0, current_speed);
                        } else if (!first_is_left && !l_on_black) {
                            run_motors(current_speed, 0);
                        } else {
                            alignment_done = true;
                            
                            white_start_left = left_motor->angle();
                            white_start_right = right_motor->angle();

                            run_motors(current_speed, current_speed);
                        }
                    }
                } else {
                    int32_t cur_left = left_motor->angle();
                    int32_t cur_right = right_motor->angle();

                    int32_t moved_left = cur_left - white_start_left;
                    int32_t moved_right = cur_right - white_start_right;

                    float sync_p = calculate_sync_p(moved_left, moved_right);

                    run_motors(current_speed - sync_p, current_speed + sync_p);
                }
            } else {
                int32_t cur_left = left_motor->angle();
                int32_t cur_right = right_motor->angle();

                int32_t moved_left = cur_left - white_start_left;
                int32_t moved_right = cur_right - white_start_right;

                float sync_p = calculate_sync_p(moved_left, moved_right);

                run_motors(current_speed - sync_p, current_speed + sync_p);
            }

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
