#ifndef __MPU_APP_H__
#define __MPU_APP_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

typedef struct
{
  uint32_t tick_ms;
  uint32_t sample_id;
  float pitch;
  float roll;
  float yaw;
  int16_t accel_x;
  int16_t accel_y;
  int16_t accel_z;
  int16_t gyro_x;
  int16_t gyro_y;
  int16_t gyro_z;
} MpuApp_Attitude_t;

typedef struct
{
  float angle_deg;
  float filtered_vertical_angle_deg;
  uint16_t stable_samples;
  uint8_t ready;
} MpuApp_MechanicalMidpoint_t;

uint8_t MpuApp_Init(void);
uint8_t MpuApp_Read(MpuApp_Attitude_t *attitude);
uint8_t MpuApp_GetLatest(MpuApp_Attitude_t *attitude);
uint8_t MpuApp_GetMechanicalMidpoint(MpuApp_MechanicalMidpoint_t *midpoint);
void MpuApp_ResetMechanicalMidpoint(void);
void MpuApp_Task(void);

#ifdef __cplusplus
}
#endif

#endif

/* by codex */
