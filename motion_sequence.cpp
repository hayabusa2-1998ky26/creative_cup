#include "motion_sequence.h"
#include "motor_control.h"
#include "ev3api.h"

void catch_block() {
    rack_motor(-100, 100);
    rack_motor(-50, 20);
    tslp_tsk(1000*10);
    
    catch_motor(-40, 100);
    catch_motor(-60, 50);
    catch_motor(-80, 35);
    catch_motor(-20, 10);
    tslp_tsk(1000*10);

    rack_motor(-410, 100);
    tslp_tsk(1000*10);

    catch_motor(90, 50);
    catch_motor(15, 20);
    tslp_tsk(1000*10);

    rack_motor(100, 100);
}

extern bool is_in_progress;

void return_arm(intptr_t unused) {
    is_in_progress = true;
    tslp_tsk(1000*10);
    rack_motor(450, 100);
    tslp_tsk(1000*10);
    // catch_motor(55, 100);
    catch_motor(55 + 40, 100);

    act_tsk(INIT_ARM);


    // 処理完了後にタスクを自発的に終了する
    ext_tsk();
}

void init_arm(intptr_t unused) {

    rack_motor_until_stalled(1, 10);
    tslp_tsk(1000*10);
    
    catch_motor_until_stalled(1, 30);
    tslp_tsk(1000*10);

    is_in_progress = false;


    // 処理完了後にタスクを自発的に終了する
    ext_tsk();
}

// 今後、一連の動作の関数をここに追加する。
