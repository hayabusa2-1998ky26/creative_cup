#ifndef DRIVE_CONFIG_H
#define DRIVE_CONFIG_H

namespace pybricks {

    struct DriveConfig {
        // 車輪の直径 (mm)
        static constexpr float wheel_diameter = 63.35f;

        // 車輪間の距離 / トレッド幅 (mm)
        static constexpr float axle_track = 179.81f;

        // 制御周期 (ms) [例: 5.0f = 5ms (5,000us)]
        static constexpr float period_ms = 3;

        // 制御最小速度 (deg/s)
        static constexpr float min_speed = 50.0f;

        // 減速マージン倍率 (1.0f = 理論値, 1.3f = 30%早めに減速開始してスムーズに停止)
        static constexpr float decel_margin = 1.3f;

        // 黒線検知後の不感帯距離 (mm) [例: 30.0f = 検知後30mm進むまで次の黒線を無視]
        static constexpr float line_blind_dist = 100.0f;

        // デフォルト直進速度・加速度 (mm/s, mm/s^2)
        static constexpr float straight_speed = 600.0f;
        static constexpr float straight_accel = 2000.0f;

        // デフォルト旋回速度・加速度 (deg/s, deg/s^2)
        static constexpr float turn_rate = 300.0f;
        static constexpr float turn_accel = 800.0f;

        // PIDアライメントゲイン (straight_line)
        static constexpr float straight_kp = 2.0f;
        static constexpr float straight_ki = 0.002f;
        static constexpr float straight_kd = 0.02f;

        // ★追加: アライメント時（pid_mode = true）のモーター出力上限 (100以下)
        // ここを小さくすると、線を検知してからの姿勢合わせがゆっくりになります。
        static constexpr float align_max_power = 1.0f;

        // アライメント許容閾値 & 目標反射光値
        static constexpr int tolerance = 30;
        static constexpr int target_val = 25;

        // ローパスフィルタのゲイン (1.0でフィルタなし、0.1などで強く平滑化)
        static constexpr float color_lpf_gain = 0.8f;

    };

} // namespace pybricks

#endif // DRIVE_CONFIG_H
