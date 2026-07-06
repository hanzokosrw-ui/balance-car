#ifndef __ENCODER_H__
#define __ENCODER_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#define ENCODER_LINES_PER_MOTOR_REV 500L
#define ENCODER_GEAR_RATIO 30L
#define ENCODER_QUADRATURE_MULTIPLIER 4L
#define ENCODER_COUNTS_PER_WHEEL_REV \
  (ENCODER_LINES_PER_MOTOR_REV * ENCODER_GEAR_RATIO * ENCODER_QUADRATURE_MULTIPLIER)
#define ENCODER_SAMPLE_PERIOD_MS 10U
#define ENCODER_REPORT_PERIOD_MS 500U
#define ENCODER_WHEEL_DIAMETER_MM 66L

typedef struct
{
  int32_t left_count;
  int32_t right_count;
  int32_t left_speed_mm_s;
  int32_t right_speed_mm_s;
  int32_t left_speed_mm_s_x100;
  int32_t right_speed_mm_s_x100;
  int32_t left_rpm_x100;
  int32_t right_rpm_x100;
} Encoder_State_t;

void Encoder_Init(void);
void Encoder_Task(void);
void Encoder_Reset(void);
void Encoder_GetState(Encoder_State_t *state);
int32_t Encoder_GetLeftSpeedMmS(void);
int32_t Encoder_GetRightSpeedMmS(void);
int32_t Encoder_GetLeftSpeedMmSX100(void);
int32_t Encoder_GetRightSpeedMmSX100(void);
int32_t Encoder_GetLeftRpmX100(void);
int32_t Encoder_GetRightRpmX100(void);
int32_t Encoder_GetLeftCount(void);
int32_t Encoder_GetRightCount(void);

#ifdef __cplusplus
}
#endif

#endif

/* by codex */
