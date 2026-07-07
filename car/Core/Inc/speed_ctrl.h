#ifndef __SPEED_CTRL_H__
#define __SPEED_CTRL_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

void SpeedCtrl_Init(void);
void SpeedCtrl_SetTarget(int32_t left_mm_s, int32_t right_mm_s);
void SpeedCtrl_Task(void);
void SpeedCtrl_Stop(void);

#ifdef __cplusplus
}
#endif

#endif

/* by codex */
