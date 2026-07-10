#ifndef __SPEED_CTRL_H__
#define __SPEED_CTRL_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

void SpeedCtrl_Init(void);
void SpeedCtrl_SetTarget(int32_t left_mm_s, int32_t right_mm_s);
void SpeedCtrl_SetSpeedGains(float kp, float ki);
void SpeedCtrl_SetSpeedKp(float kp);
void SpeedCtrl_SetSpeedKi(float ki);
void SpeedCtrl_SetDiffGains(float kp, float ki);
void SpeedCtrl_SetDiffKp(float kp);
void SpeedCtrl_SetDiffKi(float ki);
void SpeedCtrl_SetLimits(float max_velocity_pwm, float max_turn_pwm);
void SpeedCtrl_SetEnabled(uint8_t enabled);
void SpeedCtrl_Task(void);
void SpeedCtrl_Stop(void);
float SpeedCtrl_GetVelocityPwm(void);
float SpeedCtrl_GetTurnPwm(void);
void SpeedCtrl_PrintStatus(const char *prefix);

#ifdef __cplusplus
}
#endif

#endif

/* by codex */
