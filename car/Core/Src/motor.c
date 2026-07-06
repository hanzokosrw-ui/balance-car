#include "motor.h"
#include "tim.h"

static int16_t Motor_LimitDuty(int16_t duty_percent);
static int16_t Motor_ApplyDeadZone(int16_t duty_percent, uint8_t dead_zone_percent);
static uint32_t Motor_DutyToCompare(uint16_t duty_percent);
static HAL_StatusTypeDef Motor_StartChannel(uint32_t channel);
static void Motor_SetLeftDuty(int16_t duty_percent);
static void Motor_SetRightDuty(int16_t duty_percent);

HAL_StatusTypeDef Motor_Init(void)
{
  HAL_StatusTypeDef status;

  Motor_Stop();

  status = Motor_StartChannel(TIM_CHANNEL_1);
  if (status != HAL_OK)
  {
    return status;
  }

  status = Motor_StartChannel(TIM_CHANNEL_2);
  if (status != HAL_OK)
  {
    return status;
  }

  status = Motor_StartChannel(TIM_CHANNEL_3);
  if (status != HAL_OK)
  {
    return status;
  }

  status = Motor_StartChannel(TIM_CHANNEL_4);
  if (status != HAL_OK)
  {
    return status;
  }

  Motor_Stop();
  return HAL_OK;
}

void Motor_Stop(void)
{
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0U);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0U);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0U);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0U);
}

void Motor_Brake(void)
{
  uint32_t compare;

  compare = Motor_DutyToCompare(MOTOR_MAX_DUTY_PERCENT);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, compare);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, compare);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, compare);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, compare);
}

void Motor_SetDuty(int16_t left_percent, int16_t right_percent)
{
  Motor_SetLeftDuty(Motor_LimitDuty(left_percent));
  Motor_SetRightDuty(Motor_LimitDuty(right_percent));
}

void Motor_SetDutyWithDeadZone(int16_t left_percent, int16_t right_percent, uint8_t dead_zone_percent)
{
  Motor_SetDuty(Motor_ApplyDeadZone(left_percent, dead_zone_percent),
                Motor_ApplyDeadZone(right_percent, dead_zone_percent));
}

static int16_t Motor_LimitDuty(int16_t duty_percent)
{
  if (duty_percent > MOTOR_MAX_DUTY_PERCENT)
  {
    return MOTOR_MAX_DUTY_PERCENT;
  }

  if (duty_percent < -MOTOR_MAX_DUTY_PERCENT)
  {
    return -MOTOR_MAX_DUTY_PERCENT;
  }

  return duty_percent;
}

static int16_t Motor_ApplyDeadZone(int16_t duty_percent, uint8_t dead_zone_percent)
{
  int16_t limited_duty;
  int16_t limited_dead_zone;

  limited_duty = Motor_LimitDuty(duty_percent);
  limited_dead_zone = (int16_t)dead_zone_percent;

  if (limited_dead_zone > MOTOR_MAX_DUTY_PERCENT)
  {
    limited_dead_zone = MOTOR_MAX_DUTY_PERCENT;
  }

  if ((limited_duty > 0) && (limited_duty < limited_dead_zone))
  {
    return limited_dead_zone;
  }

  if ((limited_duty < 0) && (limited_duty > -limited_dead_zone))
  {
    return -limited_dead_zone;
  }

  return limited_duty;
}

static uint32_t Motor_DutyToCompare(uint16_t duty_percent)
{
  uint32_t compare;
  uint32_t period;

  if (duty_percent > MOTOR_MAX_DUTY_PERCENT)
  {
    duty_percent = MOTOR_MAX_DUTY_PERCENT;
  }

  period = htim3.Init.Period;
  compare = ((period + 1U) * duty_percent) / MOTOR_MAX_DUTY_PERCENT;

  if (compare > period)
  {
    compare = period;
  }

  return compare;
}

static HAL_StatusTypeDef Motor_StartChannel(uint32_t channel)
{
  return HAL_TIM_PWM_Start(&htim3, channel);
}

static void Motor_SetLeftDuty(int16_t duty_percent)
{
  uint32_t compare;

  if (duty_percent > 0)
  {
    compare = Motor_DutyToCompare((uint16_t)duty_percent);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, compare);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0U);
  }
  else if (duty_percent < 0)
  {
    compare = Motor_DutyToCompare((uint16_t)(-duty_percent));
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0U);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, compare);
  }
  else
  {
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0U);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0U);
  }
}

static void Motor_SetRightDuty(int16_t duty_percent)
{
  uint32_t compare;

  if (duty_percent > 0)
  {
    compare = Motor_DutyToCompare((uint16_t)duty_percent);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0U);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, compare);
  }
  else if (duty_percent < 0)
  {
    compare = Motor_DutyToCompare((uint16_t)(-duty_percent));
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, compare);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0U);
  }
  else
  {
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0U);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0U);
  }
}

/* by codex */
