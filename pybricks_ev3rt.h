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
        void stop();
        void brake();
        void hold();
        
        void run_angle(int speed_deg_per_sec, int angle_deg, Stop then = Stop::HOLD, bool wait = true);

        int32_t angle();
        void reset_angle(int32_t angle_val = 0);

        motor_port_t get_port() const { return port; }
        bool get_reversed() const { return is_reversed; }
    };


    // --- ColorSensor Class ---
    class ColorSensor {
    private:
        sensor_port_t port;

    public:
        ColorSensor(sensor_port_t port);
        int reflection();
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

        // ★ 左右対称のため片側分だけで管理
        float sensor_x; // 車体中心からセンサーまでの横幅の絶対値
        float sensor_y; // 車軸からセンサーまでの前方距離

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
        int config_tolerance;
        int config_target_val;

        bool drive_sync_initialized;
        int32_t drive_start_left;
        int32_t drive_start_right;

        // オドメトリ（自己位置推定）用変数
        float pose_x;      // mm
        float pose_y;      // mm
        float pose_theta;  // rad
        int32_t prev_left_enc;
        int32_t prev_right_enc;

        void run_motors(float left_target, float right_target);
        float calculate_sync_p(int32_t left_moved, int32_t right_moved, bool is_turn = false);
        void reset_odometry();
        void update_odometry(int32_t cur_left, int32_t cur_right);
        float calculate_lookahead_heading_pd(float lookahead_dist_mm, float dir = 1.0f);
        void run_straight_sync(float base_speed, int32_t cur_left, int32_t cur_right, float lookahead_dist = 80.0f);

    public:
        DriveBase(Motor& left, Motor& right, ColorSensor& left_sens, ColorSensor& right_sens, float wheel_diameter_mm, float axle_track_mm);

        // ★ 引数を変更
        void set_sensor_coordinates(float sensor_x_mm, float sensor_y_mm);
        void settings(float straight_speed, float straight_acceleration = DriveConfig::straight_accel, float turn_rate = DriveConfig::turn_rate, float turn_acceleration = DriveConfig::turn_accel);
        
        void straight(float distance_mm, Stop then = Stop::HOLD, bool wait = true);
        void turn(float angle_deg, Stop then = Stop::HOLD, bool wait = true);
        void drive(float speed_mm_per_sec, float turn_rate_deg_per_sec);
        
        void straight_line(float distance_mm = 0.0f, float speed_mm_per_sec = -1.0f, int target_val = -1, bool skip_white_exit_check = false);
        void straight_lines(int line_count, float distance_mm = 0.0f, float speed_mm_per_sec = -1.0f, int target_val = -1);

        void stop();
        void brake();
    };

} // namespace pybricks

#endif // PYBRICKS_EV3RT_H
