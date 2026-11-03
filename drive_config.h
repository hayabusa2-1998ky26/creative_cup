#ifndef DRIVE_CONFIG_H
#define DRIVE_CONFIG_H

namespace pybricks {

    struct DriveConfig {
        // --- カラーセンサー キャリブレーション設定 ---
        // 先ほどのキャリブレーションコードで取得した値をここに入力します
        static constexpr int left_sensor_min = 4;
        static constexpr int left_sensor_max = 55;
        static constexpr int right_sensor_min = 4;
        static constexpr int right_sensor_max = 61;

        // 車輪の直径 (mm)
        static constexpr float wheel_diameter = 63.61f;

        // 車輪間の距離 / トレッド幅 (mm)
        static constexpr float axle_track = 180.61f;

        // 制御周期 (ms) [例: 5.0f = 5ms (5,000us)]
        static constexpr float period_ms = 7.0f;

        // 制御最小速度 (deg/s)
        static constexpr float min_speed = 30.0f;

        // 減速マージン倍率 (1.0f = 理論値, 1.3f = 30%早めに減速開始してスムーズに停止)
        static constexpr float decel_margin = 1.3f;

        // 黒線検知後の不感帯距離 (mm) [例: 30.0f = 検知後30mm進むまで次の黒線を無視]
        static constexpr float line_blind_dist = 100.0f;

        // ★追加: メソッド開始直後に線・白検知を無視する距離 (mm)
        static constexpr float initial_ignore_dist = 20.0f;

        // 後退時も途中のラインから初期姿勢の傾きを補正する。
        static constexpr bool reverse_line_heading_correction = true;

        // デフォルト直進速度・加速度 (mm/s, mm/s^2)
        static constexpr float straight_speed = 600.0f;
        static constexpr float straight_accel = 1200.0f;

        // デフォルト旋回速度・加速度 (deg/s, deg/s^2)
        static constexpr float turn_rate = 300.0f;
        static constexpr float turn_accel = 800.0f;

        // PIDアライメントゲイン (straight_line)
        static constexpr float straight_kp = 100.0f;
        static constexpr float straight_ki = 0.1f;
        static constexpr float straight_kd = 1.0f;

        // ★追加: アライメント時（pid_mode = true）のモーター出力上限 (100以下)
        // ここを小さくすると、線を検知してからの姿勢合わせがゆっくりになります。
        static constexpr float align_max_power = 12.0f;

        // アライメント許容閾値 & 目標反射光値
        // 左右それぞれの正規化反射光の許容誤差
        static constexpr float tolerance = 0.3f;
        static constexpr float target_val = 0.5f;

        // ローパスフィルタのゲイン (1.0でフィルタなし、0.1などで強く平滑化)
        static constexpr float color_lpf_gain = 0.8f;

    };

} // namespace pybricks

#endif // DRIVE_CONFIG_H
