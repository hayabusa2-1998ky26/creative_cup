#ifndef MOTION_SEQUENCE_H
#define MOTION_SEQUENCE_H

#include "ev3api.h"

#ifdef __cplusplus
extern "C" {
#endif

void catch_block();
void return_arm(intptr_t unused);
void init_arm(intptr_t unused);

#ifdef __cplusplus
}
#endif

#endif // MOTION_SEQUENCE_H
