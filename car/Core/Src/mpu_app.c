#include "mpu_app.h"

#include <math.h>
#include <stdio.h>
#include "MPU6050.h"

#define MPU_APP_PRINT_PERIOD_MS 20U
#define MPU_APP_MECHANICAL_MID_FILTER_ALPHA 0.04f
#define MPU_APP_MECHANICAL_MID_UPRIGHT_WINDOW_DEG 12.0f
#define MPU_APP_MECHANICAL_MID_MAX_GYRO_DPS 2.0f
#define MPU_APP_MECHANICAL_MID_MAX_FILTER_ERROR_DEG 0.60f
#define MPU_APP_MECHANICAL_MID_MAX_RANGE_DEG 0.80f
#define MPU_APP_MECHANICAL_MID_STABLE_SAMPLES 300U
#define MPU_APP_MECHANICAL_MID_MAX_INVALID_SAMPLES 5U
#define MPU_APP_GYRO_LSB_PER_DPS 16.4f
#define MPU_APP_ACCEL_1G_LSB 16384.0f
#define MPU_APP_MECHANICAL_MID_ACCEL_MIN_G 0.85f
#define MPU_APP_MECHANICAL_MID_ACCEL_MAX_G 1.15f

extern __IO float fAX, fAY, fAZ;
extern __IO short gx, gy, gz, ax, ay, az;

static uint32_t last_print_ms = 0U;
static volatile uint32_t latest_sample_id = 0U;
static volatile uint8_t latest_valid = 0U;
static MpuApp_Attitude_t latest_attitude;
static volatile MpuApp_MechanicalMidpoint_t mechanical_midpoint;
static volatile uint8_t mechanical_midpoint_filter_initialized = 0U;
static volatile uint8_t mechanical_midpoint_invalid_samples = 0U;
static volatile uint8_t mechanical_midpoint_ready_report_pending = 0U;
static volatile float mechanical_midpoint_candidate_min_angle = 0.0f;
static volatile float mechanical_midpoint_candidate_max_angle = 0.0f;

static void MpuApp_UpdateLatest(MpuApp_Attitude_t *attitude);
static void MpuApp_UpdateMechanicalMidpoint(const MpuApp_Attitude_t *attitude);
static void MpuApp_ResetMechanicalMidpointCandidate(void);
static uint8_t MpuApp_TakeMechanicalMidpointReadyReport(MpuApp_MechanicalMidpoint_t *midpoint);
static float MpuApp_Abs(float value);
static uint8_t MpuApp_IsMechanicalMidpointSampleValid(const MpuApp_Attitude_t *attitude);

uint8_t MpuApp_Init(void) {
  last_print_ms = 0U;
  latest_sample_id = 0U;
  latest_valid = 0U;
  MpuApp_ResetMechanicalMidpoint();

  i2cInit();
  HAL_Delay(100U);

  if (MPU_init() == 0) {
    printf("mpu dmp init failed\r\n");
    return 0U;
  }
  printf("pitch,roll,yaw\r\n");
  printf("keep car upright and still for mechanical midpoint calibration\r\n");
  return 1U;
}

void MpuApp_Task(void) {
  uint32_t now_ms;
  MpuApp_Attitude_t attitude;
  MpuApp_MechanicalMidpoint_t midpoint;

  if (MpuApp_TakeMechanicalMidpointReadyReport(&midpoint) != 0U)
  {
    printf("MID_OK=1 MID=%.2f\r\n", (double)midpoint.angle_deg);
  }

  now_ms = HAL_GetTick();
  if ((now_ms - last_print_ms) < MPU_APP_PRINT_PERIOD_MS) return;
  if (MpuApp_GetLatest(&attitude) == 0U) return;

 /* printf("%.2f,%.2f,%.2f\r\n",
         (double)attitude.pitch,
         (double)attitude.roll,
         (double)attitude.yaw); */
	
	printf("%.2f\r\n", (double)attitude.pitch); 
  last_print_ms = now_ms;
}

uint8_t MpuApp_Read(MpuApp_Attitude_t *attitude) {
  MpuApp_Attitude_t data;

  if (MPU_getdata() == 0U) return 0U;

  data.tick_ms = HAL_GetTick();
  data.sample_id = 0U;
  data.pitch   = fAY;
  data.roll    = fAX;
  data.yaw     = fAZ;
  data.gyro_x  = gx;
  data.gyro_y  = gy;
  data.gyro_z  = gz;
  data.accel_x = ax;
  data.accel_y = ay;
  data.accel_z = az;

  MpuApp_UpdateMechanicalMidpoint(&data);
  MpuApp_UpdateLatest(&data);

  if (attitude != NULL) {
    *attitude = data;
  }
  return 1U;
}

uint8_t MpuApp_GetLatest(MpuApp_Attitude_t *attitude) {
  uint32_t primask;
  uint8_t valid;

  if (attitude == NULL) {
    return 0U;
  }

  primask = __get_PRIMASK();
  __disable_irq();
  valid = latest_valid;
  if (valid != 0U) {
    *attitude = latest_attitude;
  }
  if (primask == 0U) {
    __enable_irq();
  }

  return valid;
}

uint8_t MpuApp_GetMechanicalMidpoint(MpuApp_MechanicalMidpoint_t *midpoint)
{
  uint32_t primask;

  if (midpoint == NULL)
  {
    return 0U;
  }

  primask = __get_PRIMASK();
  __disable_irq();
  midpoint->angle_deg = mechanical_midpoint.angle_deg;
  midpoint->filtered_vertical_angle_deg = mechanical_midpoint.filtered_vertical_angle_deg;
  midpoint->stable_samples = mechanical_midpoint.stable_samples;
  midpoint->ready = mechanical_midpoint.ready;
  if (primask == 0U)
  {
    __enable_irq();
  }

  return midpoint->ready;
}

void MpuApp_ResetMechanicalMidpoint(void)
{
  uint32_t primask;

  primask = __get_PRIMASK();
  __disable_irq();
  mechanical_midpoint.angle_deg = 0.0f;
  mechanical_midpoint.filtered_vertical_angle_deg = 0.0f;
  mechanical_midpoint.stable_samples = 0U;
  mechanical_midpoint.ready = 0U;
  mechanical_midpoint_ready_report_pending = 0U;
  MpuApp_ResetMechanicalMidpointCandidate();
  if (primask == 0U)
  {
    __enable_irq();
  }
}

static void MpuApp_UpdateLatest(MpuApp_Attitude_t *attitude) {
  if (attitude == NULL) {
    return;
  }

  latest_sample_id++;
  attitude->sample_id = latest_sample_id;
  latest_attitude = *attitude;
  latest_valid = 1U;
}

static void MpuApp_UpdateMechanicalMidpoint(const MpuApp_Attitude_t *attitude)
{
  float filtered_angle;

  if ((attitude == NULL) || (mechanical_midpoint.ready != 0U))
  {
    return;
  }

  if (MpuApp_IsMechanicalMidpointSampleValid(attitude) == 0U)
  {
    if (mechanical_midpoint_invalid_samples < MPU_APP_MECHANICAL_MID_MAX_INVALID_SAMPLES)
    {
      mechanical_midpoint_invalid_samples++;
    }
    if (mechanical_midpoint_invalid_samples >= MPU_APP_MECHANICAL_MID_MAX_INVALID_SAMPLES)
    {
      MpuApp_ResetMechanicalMidpointCandidate();
    }
    return;
  }

  mechanical_midpoint_invalid_samples = 0U;

  if (mechanical_midpoint_filter_initialized == 0U)
  {
    mechanical_midpoint.filtered_vertical_angle_deg = attitude->pitch;
    mechanical_midpoint.stable_samples = 1U;
    mechanical_midpoint_filter_initialized = 1U;
    mechanical_midpoint_candidate_min_angle = attitude->pitch;
    mechanical_midpoint_candidate_max_angle = attitude->pitch;
    return;
  }

  if (attitude->pitch < mechanical_midpoint_candidate_min_angle)
  {
    mechanical_midpoint_candidate_min_angle = attitude->pitch;
  }
  if (attitude->pitch > mechanical_midpoint_candidate_max_angle)
  {
    mechanical_midpoint_candidate_max_angle = attitude->pitch;
  }

  if ((mechanical_midpoint_candidate_max_angle -
       mechanical_midpoint_candidate_min_angle) >
      MPU_APP_MECHANICAL_MID_MAX_RANGE_DEG)
  {
    MpuApp_ResetMechanicalMidpointCandidate();
    return;
  }

  filtered_angle = mechanical_midpoint.filtered_vertical_angle_deg;
  filtered_angle += MPU_APP_MECHANICAL_MID_FILTER_ALPHA *
                    (attitude->pitch - filtered_angle);

  if (MpuApp_Abs(attitude->pitch - filtered_angle) >
      MPU_APP_MECHANICAL_MID_MAX_FILTER_ERROR_DEG)
  {
    MpuApp_ResetMechanicalMidpointCandidate();
    return;
  }

  mechanical_midpoint.filtered_vertical_angle_deg = filtered_angle;
  if (mechanical_midpoint.stable_samples < MPU_APP_MECHANICAL_MID_STABLE_SAMPLES)
  {
    mechanical_midpoint.stable_samples++;
  }

  if (mechanical_midpoint.stable_samples >= MPU_APP_MECHANICAL_MID_STABLE_SAMPLES)
  {
    mechanical_midpoint.angle_deg = filtered_angle;
    mechanical_midpoint.ready = 1U;
    mechanical_midpoint_ready_report_pending = 1U;
  }
}

static void MpuApp_ResetMechanicalMidpointCandidate(void)
{
  mechanical_midpoint.filtered_vertical_angle_deg = 0.0f;
  mechanical_midpoint.stable_samples = 0U;
  mechanical_midpoint_filter_initialized = 0U;
  mechanical_midpoint_invalid_samples = 0U;
  mechanical_midpoint_candidate_min_angle = 0.0f;
  mechanical_midpoint_candidate_max_angle = 0.0f;
}

static uint8_t MpuApp_TakeMechanicalMidpointReadyReport(MpuApp_MechanicalMidpoint_t *midpoint)
{
  uint32_t primask;
  uint8_t report_ready;

  if (midpoint == NULL)
  {
    return 0U;
  }

  primask = __get_PRIMASK();
  __disable_irq();
  report_ready = 0U;
  if ((mechanical_midpoint_ready_report_pending != 0U) &&
      (mechanical_midpoint.ready != 0U))
  {
    midpoint->angle_deg = mechanical_midpoint.angle_deg;
    midpoint->filtered_vertical_angle_deg = mechanical_midpoint.filtered_vertical_angle_deg;
    midpoint->stable_samples = mechanical_midpoint.stable_samples;
    midpoint->ready = mechanical_midpoint.ready;
    mechanical_midpoint_ready_report_pending = 0U;
    report_ready = 1U;
  }
  if (primask == 0U)
  {
    __enable_irq();
  }

  return report_ready;
}

static float MpuApp_Abs(float value)
{
  if (value < 0.0f)
  {
    return -value;
  }

  return value;
}

static uint8_t MpuApp_IsMechanicalMidpointSampleValid(const MpuApp_Attitude_t *attitude)
{
  float gyro_limit;
  float accel_x;
  float accel_y;
  float accel_z;
  float accel_magnitude_squared;
  float accel_min;
  float accel_max;

  if (!((attitude->pitch >= -MPU_APP_MECHANICAL_MID_UPRIGHT_WINDOW_DEG) &&
        (attitude->pitch <= MPU_APP_MECHANICAL_MID_UPRIGHT_WINDOW_DEG)))
  {
    return 0U;
  }

  gyro_limit = MPU_APP_MECHANICAL_MID_MAX_GYRO_DPS * MPU_APP_GYRO_LSB_PER_DPS;
  if (MpuApp_Abs((float)attitude->gyro_y) > gyro_limit)
  {
    return 0U;
  }

  accel_x = (float)attitude->accel_x;
  accel_y = (float)attitude->accel_y;
  accel_z = (float)attitude->accel_z;
  accel_magnitude_squared = accel_x * accel_x +
                              accel_y * accel_y +
                              accel_z * accel_z;
  accel_min = MPU_APP_ACCEL_1G_LSB * MPU_APP_MECHANICAL_MID_ACCEL_MIN_G;
  accel_max = MPU_APP_ACCEL_1G_LSB * MPU_APP_MECHANICAL_MID_ACCEL_MAX_G;

  if ((accel_magnitude_squared < (accel_min * accel_min)) ||
      (accel_magnitude_squared > (accel_max * accel_max)))
  {
    return 0U;
  }

  return 1U;
}

/* by codex */
