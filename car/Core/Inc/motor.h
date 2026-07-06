#ifndef __MOTOR_H__
#define __MOTOR_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#define MOTOR_MAX_DUTY_PERCENT 100
#define MOTOR_DEFAULT_DEAD_ZONE_PERCENT 56 //by codex

HAL_StatusTypeDef Motor_Init(void);
void Motor_Stop(void);
void Motor_Brake(void);
void Motor_SetDuty(int16_t left_percent, int16_t right_percent);
void Motor_SetDutyWithDeadZone(int16_t left_percent, int16_t right_percent, uint8_t dead_zone_percent);

#ifdef __cplusplus
}
#endif

#endif

/* by codex */
