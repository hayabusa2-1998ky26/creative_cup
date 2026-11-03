#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

// AポートのラックとDポートのキャッチを初期化する。
void init_aux_motors();

// speed_percent: 1～100 (%)、省略時70。0以下なら停止、100超は100に制限。
// 現在位置から指定角度だけ回転し、完了まで待つ。負の角度で逆回転。
void rack_motor(int angle_deg, int speed_percent = 70);
void catch_motor(int angle_deg, int speed_percent = 70);

// direction: 1で正回転、-1で逆回転。指定速度で停止検出まで回転。
// true: 停止検出、false: 約5秒の時間切れ、directionが0、または速度0以下。
bool rack_motor_until_stalled(int direction, int speed_percent = 70);
bool catch_motor_until_stalled(int direction, int speed_percent = 70);

#endif
