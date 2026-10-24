#ifndef _APP_H_
#define _APP_H_

#include <stdint.h>  // ← この行を追加！
#include "ev3api.h"  // もし入っていなければこれも確認

#ifdef __cplusplus
extern "C" {
#endif

/*
 *  ターゲットに依存する可能性のあるテストプログラムの定義
 */
#ifndef MAIN_PRIORITY
#define MAIN_PRIORITY    MAIN_PRI_MID
#endif

#ifndef STACK_SIZE
#define STACK_SIZE       512
#endif

#ifndef TOPPERS_MACRO_ONLY

extern void main_task(intptr_t exinf);

#endif /* TOPPERS_MACRO_ONLY */

#ifdef __cplusplus
}
#endif

#endif /* _APP_H_ */
