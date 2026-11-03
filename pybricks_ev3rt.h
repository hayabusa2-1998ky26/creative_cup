#ifndef PYBRICKS_EV3RT_H
#define PYBRICKS_EV3RT_H

#include "ev3api.h"
#include "drive_config.h"
#include <stdint.h>
#include <math.h>

namespace pybricks {

    enum class Direction {
        CLOCKWISE,
        COUNTERCLOCKWISE
    };

    enum class Stop {
        COAST,
        BRAKE,
        HOLD
    };

    // --- Motor Class ---
    class Motor {
    private:
        motor_port_t port;
        bool is_reversed;
        int last_power;

    public:
        Motor(motor_port_t port, Direction positive_direction = Direction::CLOCKWISE);
        
        void run(int speed_deg_per_sec);
        void dc(float power_percent);
        void stop();
        void brake();
        void hold();
        
        void run_angle(int speed_deg_per_sec, int angle_deg, Stop then = Stop::HOLD, bool wait = true);

        // Run at signed power (%) until stalled or a five-second timeout.
        bool run_until_stalled(int power);

        int32_t angle();
        void reset_angle(int32_t angle_val = 0);

        motor_port_t get_port() const { return port; }
        bool get_reversed() const { return is_reversed; }
    };


    // --- ColorSensor Class ---
    class ColorSensor {
    private:
        sensor_port_t port;
        int min_val;
        int max_val;

    public:
        ColorSensor(sensor_port_t port, int min_val = 0, int max_val = 100);
        void set_calibration(int min_v, int max_v);

        int reflection();
        // ★追加: 0.0f 〜 1.0f の範囲に正規化された反射光度を取得する関数
        float normalized_reflection();

        int ambient();
        colorid_t color();
    };


    // --- DriveBase Class ---
    class DriveBase {
    private:
        Motor* left_motor;
        Motor* right_motor;
        ColorSensor* left_sensor;
        ColorSensor* right_sensor;

        float wheel_diameter; 
        float axle_track;     

        float sensor_x;
        float sensor_y;

        float control_period_ms; 
        float min_speed;
        float decel_margin;      
        float line_blind_dist;   
        float default_straight_speed;
        float default_straight_accel;
        float default_turn_rate;
        float default_turn_accel;

        float straight_kp;
        float straight_ki;
        float straight_kd;
        // ★変更: 浮動小数点(float)に変更
        float config_tolerance;
        float config_target_val;

        bool drive_sync_initialized;
        int32_t drive_start_left;
        int32_t drive_start_right;

        float pose_x;     
        float pose_y;     
        float pose_theta; 
        float heading_prev_error;
        bool heading_pd_initialized;
        int32_t prev_left_enc;
        int32_t prev_right_enc;

        void run_motors(float left_target, float right_target);
        float calculate_sync_p(int32_t left_moved, int32_t right_moved, bool is_turn = false);
        void reset_odometry();
        void update_odometry(int32_t cur_left, int32_t cur_right);
        float calculate_lookahead_heading_pd(float lookahead_dist_mm, float dir = 1.0f, bool heading_only = false);
        void run_straight_sync(float base_speed, int32_t cur_left, int32_t cur_right, float lookahead_dist = 80.0f, bool heading_only = false);

    public:
        DriveBase(Motor& left, Motor& right, ColorSensor& left_sens, ColorSensor& right_sens);

        void set_sensor_coordinates(float sensor_x_mm, float sensor_y_mm);
        void settings(float straight_speed, float straight_acceleration = DriveConfig::straight_accel, float turn_rate = DriveConfig::turn_rate, float turn_acceleration = DriveConfig::turn_accel);
        
        void straight(float distance_mm, Stop then = Stop::HOLD, bool wait = true);
        void turn(float angle_deg, Stop then = Stop::HOLD, bool wait = true);
        void wide_forward(float angle_deg, float radius_mm = -1.0f, Stop then = Stop::HOLD, bool wait = true);
        void drive(float speed_mm_per_sec, float turn_rate_deg_per_sec);
        
        // ★変更: 第6引数 skip_initial_ignore を追加 (デフォルトは false)
        void straight_line(float distance_mm = 0.0f, float speed_mm_per_sec = 0.0f, float target_val = -1.0f, bool skip_white_exit_check = false, bool reverse = false, bool skip_initial_ignore = false);
        void straight_lines(int line_count, float distance_mm = 0.0f, void (*func)() = nullptr, float func_distance = 0.0f, float speed_mm_per_sec = 0.0f, float target_val = -1.0f);

        void stop();
        void brake();
    };

} // namespace pybricks

#endif // PYBRICKS_EV3RT_H
