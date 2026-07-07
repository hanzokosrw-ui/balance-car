#include "pid.h"

/* 初始化 PI 控制器参数并清零内部状态。 */
void PI_Init(PI_Controller_t *pi, float Kp, float Ki, float Ts,
             int32_t out_min, int32_t out_max) {
  pi->Kp = Kp;
  pi->Ki = Ki;
  pi->Ts = Ts;
  pi->output = 0;
  pi->output_max = out_max;
  pi->output_min = out_min;
  pi->target = 0;
  pi->last_error = 0;
  pi->last_actual = 0;
  pi->initialized = 0;
}

/* 设置目标值。 */
void PI_SetTarget(PI_Controller_t *pi, int32_t target) { pi->target = target; }

/* 复位内部状态（切换目标或重新启动时调用）。 */
void PI_Reset(PI_Controller_t *pi) {
  pi->output = 0;
  pi->last_error = 0;
  pi->last_actual = 0;
  pi->initialized = 0;
}

/*
 * 增量式 PI 计算。
 *    e(k)   = target - actual
 *    Δu     = Kp * (e(k) - e(k-1)) + Ki * Ts * e(k)
 *    u(k)   = u(k-1) + Δu
 * 输出经上下限钳位后返回。
 */
int32_t PI_Compute(PI_Controller_t *pi, int32_t actual) {
  int32_t error;
  float delta_u;
  float output_f;

  error = pi->target - actual;

  /* 首次调用不做增量，直接用比例项初始化输出 */
  if (pi->initialized == 0U) {
    pi->initialized = 1;
    pi->last_error = error;
    pi->last_actual = actual;
    output_f = pi->Kp * (float)error;
    if (output_f > (float)pi->output_max) {
      output_f = (float)pi->output_max;
    } else if (output_f < (float)pi->output_min) {
      output_f = (float)pi->output_min;
    }
    pi->output = (int32_t)output_f;
    return pi->output;
  }

  /* 增量式: Δu = Kp*(e(k)-e(k-1)) + Ki*Ts*e(k) */
  delta_u = pi->Kp * (float)(error - pi->last_error) +
            pi->Ki * pi->Ts * (float)error;

  output_f = (float)pi->output + delta_u;

  /* 钳位输出 */
  if (output_f > (float)pi->output_max) {
    output_f = (float)pi->output_max;
  } else if (output_f < (float)pi->output_min) {
    output_f = (float)pi->output_min;
  }

  pi->output = (int32_t)output_f;
  pi->last_error = error;
  pi->last_actual = actual;

  return pi->output;
}

/* by codex */
