#include "motion_sequence.h"
#include "motor_control.h"
#include "ev3api.h"

void catch_block() {
    rack_motor(-120, 100);
    rack_motor(-50, 20);
    
    catch_motor(-60, 50);
    catch_motor(-80, 35);
    catch_motor(-20, 20);

    rack_motor(-390, 100);

    catch_motor(90, 50);
    catch_motor(20, 20);

    rack_motor(100, 100);
    // rack_motor_until_stalled(1,30);

}

void return_arm(intptr_t unused) {
    rack_motor(460, 100);
    catch_motor(50, 100);


    // 処理完了後にタスクを自発的に終了する
    ext_tsk();
}

// 今後、一連の動作の関数をここに追加する。
