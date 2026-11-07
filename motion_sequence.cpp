#include "motion_sequence.h"
#include "motor_control.h"
#include "ev3api.h"

void delay() {
    tslp_tsk(1000*10);
}

void delay_little() {
    tslp_tsk(1000*5);
}

extern bool is_last_red;

void catch_block() {
    if (is_last_red) {
        rack_motor(-70, 100);
    } else {
        rack_motor(-90, 100);
    }
    delay_little();
    rack_motor(-50, 20);
    delay();
    
    catch_motor(-65, 50);
    delay_little();
    catch_motor(-80, 35);
    delay_little();
    catch_motor(-20, 10);
    delay();

    rack_motor(-430, 100);
    delay();

    catch_motor(85, 50);
    delay_little();
    catch_motor(20, 30);
    delay();
    
    if (is_last_red) {
        rack_motor(180, 100);
    } else {
        rack_motor(200, 100);
    }
}

extern bool is_in_progress;

void return_arm(intptr_t unused) {
    is_in_progress = true;
    delay();
    rack_motor(370, 100);
    delay();
    // catch_motor(55, 100);
    catch_motor(60, 100);

    // act_tsk(INIT_ARM);


    // 処理完了後にタスクを自発的に終了する
    is_in_progress = false;
    ext_tsk();
}

void init_arm(intptr_t unused) {
    rack_motor_until_stalled(1, 10);
    delay();
    
    catch_motor_until_stalled(1, 15);
    delay();

    catch_motor(-40, 100);





    // 処理完了後にタスクを自発的に終了する
    ext_tsk();
}

// 今後、一連の動作の関数をここに追加する。
