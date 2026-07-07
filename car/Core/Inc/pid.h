#ifndef __PID_H__
#define __PID_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef struct {
  float Kp;
  float Ki;
  float Ts; /* 采样周期，单位秒 */
  int32_t target;       /* 目标值 */
  int32_t output;       /* 当前输出 */
  int32_t output_max;   /* 输出上限 */
  int32_t output_min;   /* 输出下限 */
  int32_t last_error;   /* e(k-1) */
  int32_t last_actual;  /* 上一次实际值，用于滤波 */
  uint8_t initialized;  /* 是否已经跑过一次 */
} PI_Controller_t;

void PI_Init(PI_Controller_t *pi, float Kp, float Ki, float Ts,
             int32_t out_min, int32_t out_max);
int32_t PI_Compute(PI_Controller_t *pi, int32_t actual);
void PI_SetTarget(PI_Controller_t *pi, int32_t target);
void PI_Reset(PI_Controller_t *pi);

#ifdef __cplusplus
}
#endif

#endif

/* by codex */
