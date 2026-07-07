#include "encoder.h"

#include <stdint.h>
#include <stdio.h>

#include "tim.h"

#ifndef ENCODER_LINES_PER_MOTOR_REV
#define ENCODER_LINES_PER_MOTOR_REV 500L
#endif

#ifndef ENCODER_GEAR_RATIO
#define ENCODER_GEAR_RATIO 30L
#endif

#ifndef ENCODER_QUADRATURE_MULTIPLIER
#define ENCODER_QUADRATURE_MULTIPLIER 4L
#endif

#ifndef ENCODER_COUNTS_PER_WHEEL_REV
#define ENCODER_COUNTS_PER_WHEEL_REV                  \
  (ENCODER_LINES_PER_MOTOR_REV * ENCODER_GEAR_RATIO * \
   ENCODER_QUADRATURE_MULTIPLIER)
#endif

#ifndef ENCODER_SAMPLE_PERIOD_MS
#define ENCODER_SAMPLE_PERIOD_MS 10U
#endif

#ifndef ENCODER_REPORT_PERIOD_MS
#define ENCODER_REPORT_PERIOD_MS 500U
#endif

#ifndef ENCODER_WHEEL_DIAMETER_MM
#define ENCODER_WHEEL_DIAMETER_MM 66L
#endif

#ifndef ENCODER_STATE_T_DEFINED
#define ENCODER_STATE_T_DEFINED
typedef struct Encoder_State_t {
  int32_t left_count;
  int32_t right_count;
  int32_t left_speed_mm_s;
  int32_t right_speed_mm_s;
  int32_t left_speed_mm_s_x100;
  int32_t right_speed_mm_s_x100;
  int32_t left_rpm_x100;
  int32_t right_rpm_x100;
} Encoder_State_t;
#endif

#define ENCODER_PI_X1000 3142L
#define ENCODER_LEFT_INVERT 1
#define ENCODER_RIGHT_INVERT 0

typedef struct {
  TIM_HandleTypeDef *htim;
  uint16_t last_counter;
  int32_t total_count;
  int32_t speed_mm_s_x100;
  int32_t rpm_x100;
  uint8_t inverted;
} Encoder_Channel_t;

static Encoder_Channel_t encoder_left = {&htim4, 0U, 0L,
                                         0L,     0L, ENCODER_LEFT_INVERT};
static Encoder_Channel_t encoder_right = {&htim8, 0U, 0L,
                                          0L,     0L, ENCODER_RIGHT_INVERT};
static uint32_t encoder_last_sample_ms = 0U;
static uint32_t encoder_last_report_ms = 0U;

static void Encoder_StartTimer(TIM_HandleTypeDef *htim);
static void Encoder_ResetChannel(Encoder_Channel_t *channel);
static void Encoder_UpdateChannel(Encoder_Channel_t *channel,
                                  uint32_t elapsed_ms);
static int32_t Encoder_DeltaToSpeedMmSX100(int16_t delta_count,
                                           uint32_t elapsed_ms);
static int32_t Encoder_DeltaToRpmX100(int16_t delta_count, uint32_t elapsed_ms);
static int32_t Encoder_RoundX100ToInteger(int32_t value_x100);
static int32_t Encoder_DivideRounded(int64_t numerator, int64_t denominator);
static void Encoder_Report(void);
static void Encoder_PrintSignedFixed2(int32_t value);

/* 启动左右编码器定时器，并输出初始化后的理论计数信息。 */
void Encoder_Init(void) {
  Encoder_StartTimer(&htim4);
  Encoder_StartTimer(&htim8);
  Encoder_Reset();
  printf("encoder init ok, cpr=%ld\r\n", (long)ENCODER_COUNTS_PER_WHEEL_REV);
}

/* 主循环周期调用，按采样周期更新编码器状态并定期打印。 */
void Encoder_Task(void) {
  uint32_t now_ms;
  uint32_t elapsed_ms;

  now_ms = HAL_GetTick();
  elapsed_ms = now_ms - encoder_last_sample_ms;

  if (elapsed_ms > 1000U) {
    Encoder_Reset();
    return;
  }

  if (elapsed_ms < ENCODER_SAMPLE_PERIOD_MS) {
    return;
  }

  encoder_last_sample_ms = now_ms;
  Encoder_UpdateChannel(&encoder_left, elapsed_ms);
  Encoder_UpdateChannel(&encoder_right, elapsed_ms);

  if ((now_ms - encoder_last_report_ms) >= ENCODER_REPORT_PERIOD_MS) {
    encoder_last_report_ms = now_ms;
    Encoder_Report();
  }
}

/* 清零左右编码器的累计状态，并同步当前计数作为基准。 */
void Encoder_Reset(void) {
  Encoder_ResetChannel(&encoder_left);
  Encoder_ResetChannel(&encoder_right);
  encoder_last_sample_ms = HAL_GetTick();
  encoder_last_report_ms = encoder_last_sample_ms;
}

/* 导出左右编码器当前累计脉冲、速度和转速到外部状态结构。 */
void Encoder_GetState(Encoder_State_t *state) {
  if (state == NULL) {
    return;
  }

  state->left_count = encoder_left.total_count;
  state->right_count = encoder_right.total_count;
  state->left_speed_mm_s =
      Encoder_RoundX100ToInteger(encoder_left.speed_mm_s_x100);
  state->right_speed_mm_s =
      Encoder_RoundX100ToInteger(encoder_right.speed_mm_s_x100);
  state->left_speed_mm_s_x100 = encoder_left.speed_mm_s_x100;
  state->right_speed_mm_s_x100 = encoder_right.speed_mm_s_x100;
  state->left_rpm_x100 = encoder_left.rpm_x100;
  state->right_rpm_x100 = encoder_right.rpm_x100;
}

/* 返回左轮当前速度，单位为 mm/s。 */
int32_t Encoder_GetLeftSpeedMmS(void) {
  return Encoder_RoundX100ToInteger(encoder_left.speed_mm_s_x100);
}

/* 返回右轮当前速度，单位为 mm/s。 */
int32_t Encoder_GetRightSpeedMmS(void) {
  return Encoder_RoundX100ToInteger(encoder_right.speed_mm_s_x100);
}

/* 返回左轮当前速度，单位为 mm/s，保留 2 位小数。 */
int32_t Encoder_GetLeftSpeedMmSX100(void) {
  return encoder_left.speed_mm_s_x100;
}

/* 返回右轮当前速度，单位为 mm/s，保留 2 位小数。 */
int32_t Encoder_GetRightSpeedMmSX100(void) {
  return encoder_right.speed_mm_s_x100;
}

/* 返回左轮当前转速，单位为 rpm，保留 2 位小数。 */
int32_t Encoder_GetLeftRpmX100(void) { return encoder_left.rpm_x100; }

/* 返回右轮当前转速，单位为 rpm，保留 2 位小数。 */
int32_t Encoder_GetRightRpmX100(void) { return encoder_right.rpm_x100; }

/* 返回左轮累计脉冲数。 */
int32_t Encoder_GetLeftCount(void) { return encoder_left.total_count; }

/* 返回右轮累计脉冲数。 */
int32_t Encoder_GetRightCount(void) { return encoder_right.total_count; }

/* 启动指定编码器定时器的编码器模式。 */
static void Encoder_StartTimer(TIM_HandleTypeDef *htim) {
  if (HAL_TIM_Encoder_Start(htim, TIM_CHANNEL_ALL) != HAL_OK) {
    Error_Handler();
  }
}

/* 读取当前计数并把通道状态重置到一个新的采样基准。 */
static void Encoder_ResetChannel(Encoder_Channel_t *channel) {
  channel->last_counter = (uint16_t)__HAL_TIM_GET_COUNTER(channel->htim);
  channel->total_count = 0L;
  channel->speed_mm_s_x100 = 0L;
  channel->rpm_x100 = 0L;
}

/* 根据采样间隔更新单个通道的累计脉冲、速度和转速。 */
static void Encoder_UpdateChannel(Encoder_Channel_t *channel,
                                  uint32_t elapsed_ms) {
  uint16_t current_counter;
  int16_t delta_count;

  current_counter = (uint16_t)__HAL_TIM_GET_COUNTER(channel->htim);
  delta_count = (int16_t)(current_counter - channel->last_counter);
  channel->last_counter = current_counter;

  if (channel->inverted != 0U) {
    delta_count = (int16_t)(-delta_count);
  }

  channel->total_count += delta_count;
  channel->speed_mm_s_x100 =
      Encoder_DeltaToSpeedMmSX100(delta_count, elapsed_ms);
  channel->rpm_x100 = Encoder_DeltaToRpmX100(delta_count, elapsed_ms);
}

/* 把脉冲增量换算成速度，结果保留 2 位小数。 */
static int32_t Encoder_DeltaToSpeedMmSX100(int16_t delta_count,
                                           uint32_t elapsed_ms) {
  int64_t numerator;
  int64_t denominator;

  numerator = (int64_t)delta_count * ENCODER_PI_X1000 *
              ENCODER_WHEEL_DIAMETER_MM * 100L;
  denominator = (int64_t)ENCODER_COUNTS_PER_WHEEL_REV * elapsed_ms;

  return Encoder_DivideRounded(numerator, denominator);
}

/* 把脉冲增量换算成转速 rpm，结果保留 2 位小数。 */
static int32_t Encoder_DeltaToRpmX100(int16_t delta_count,
                                      uint32_t elapsed_ms) {
  int64_t numerator;
  int64_t denominator;

  numerator = (int64_t)delta_count * 60000L * 100L;
  denominator = (int64_t)ENCODER_COUNTS_PER_WHEEL_REV * elapsed_ms;

  return Encoder_DivideRounded(numerator, denominator);
}

/* 把 x100 形式的数值转换成整数显示值。 */
static int32_t Encoder_RoundX100ToInteger(int32_t value_x100) {
  if (value_x100 >= 0L) {
    return (value_x100 + 50L) / 100L;
  }

  return (value_x100 - 50L) / 100L;
}

/* 对分子/分母做四舍五入的整数除法，避免速度计算出现明显截断误差。 */
static int32_t Encoder_DivideRounded(int64_t numerator, int64_t denominator) {
  if (denominator <= 0L) {
    return 0L;
  }

  if (numerator >= 0L) {
    return (int32_t)((numerator + (denominator / 2L)) / denominator);
  }

  return (int32_t)((numerator - (denominator / 2L)) / denominator);
}

/* 按固定格式打印左右轮的累计计数、转速和线速度。 */
static void Encoder_Report(void) {
  printf("enc left cnt=%ld rpm=", (long)encoder_left.total_count);
  Encoder_PrintSignedFixed2(encoder_left.rpm_x100);
  printf(" line=");
  Encoder_PrintSignedFixed2(encoder_left.speed_mm_s_x100);
  printf("mm/s, right cnt=%ld rpm=", (long)encoder_right.total_count);
  Encoder_PrintSignedFixed2(encoder_right.rpm_x100);
  printf(" line=");
  Encoder_PrintSignedFixed2(encoder_right.speed_mm_s_x100);
  printf("mm/s\r\n");
}

/* 以固定的两位小数格式输出数值。 */
static void Encoder_PrintSignedFixed2(int32_t value) {
  long abs_value;

  if (value < 0L) {
    printf("-");
    value = -value;
  }

  abs_value = (long)value;
  printf("%ld.%02ld", abs_value / 100L, abs_value % 100L);
}

/* by codex */
