#ifndef DRIVE_CONFIG_H
#define DRIVE_CONFIG_H

namespace pybricks {

    struct DriveConfig {
        // 制御最小速度 (deg/s)
        static constexpr float min_speed = 40.0f;

        // デフォルト直進速度・加速度 (mm/s, mm/s^2)
        static constexpr float straight_speed = 200.0f;
        static constexpr float straight_accel = 600.0f;

        // デフォルト旋回速度・加速度 (deg/s, deg/s^2)
        static constexpr float turn_rate = 150.0f;
        static constexpr float turn_accel = 500.0f;

        // PIDアライメントゲイン (straight_line)
        static constexpr float straight_kp = 2.0f;
        static constexpr float straight_ki = 0.002f;
        static constexpr float straight_kd = 0.02f;

        // アライメント許容閾値 & 目標反射光値
        static constexpr int tolerance = 30;
        static constexpr int target_val = 45;
    };

} // namespace pybricks

#endif // DRIVE_CONFIG_H
