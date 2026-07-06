#include "mpu_app.h"

#include <stdio.h>
#include "MPU6050.h"

#define MPU_APP_READ_PERIOD_MS 20U
#define MPU_APP_PRINT_PERIOD_MS 100U

static uint8_t mpu_app_ready = 0U;
static uint32_t mpu_app_last_read_ms = 0U;
static uint32_t mpu_app_last_print_ms = 0U;

static void MpuApp_PrintFixed2(int32_t value);
static int32_t MpuApp_AngleToFixed2(float value);

uint8_t MpuApp_Init(void)
{
  i2cInit();
  HAL_Delay(100U);

  if (MPU_init() == 0)
  {
    mpu_app_ready = 0U;
    printf("mpu init failed\r\n");
    return 0U;
  }

  mpu_app_ready = 1U;
  printf("mpu init ok\r\n");
  return 1U;
}

uint8_t MpuApp_Read(MpuApp_Attitude_t *attitude)
{
  if (mpu_app_ready == 0U)
  {
    return 0U;
  }

  if (MPU_getdata() == 0U)
  {
    return 0U;
  }

  if (attitude != NULL)
  {
    attitude->pitch = fAX;
    attitude->roll = fAY;
    attitude->yaw = fAZ;
    attitude->accel_x = ax;
    attitude->accel_y = ay;
    attitude->accel_z = az;
    attitude->gyro_x = gx;
    attitude->gyro_y = gy;
    attitude->gyro_z = gz;
  }

  return 1U;
}

void MpuApp_Task(void)
{
  uint32_t now_ms;
  MpuApp_Attitude_t attitude;

  now_ms = HAL_GetTick();
  if ((now_ms - mpu_app_last_read_ms) < MPU_APP_READ_PERIOD_MS)
  {
    return;
  }
  mpu_app_last_read_ms = now_ms;

  if (MpuApp_Read(&attitude) == 0U)
  {
    return;
  }

  if ((now_ms - mpu_app_last_print_ms) < MPU_APP_PRINT_PERIOD_MS)
  {
    return;
  }
  mpu_app_last_print_ms = now_ms;

  printf("att pitch=");
  MpuApp_PrintFixed2(MpuApp_AngleToFixed2(attitude.pitch));
  printf(", roll=");
  MpuApp_PrintFixed2(MpuApp_AngleToFixed2(attitude.roll));
  printf(", yaw=");
  MpuApp_PrintFixed2(MpuApp_AngleToFixed2(attitude.yaw));
  printf(", acc=%d,%d,%d, gyro=%d,%d,%d\r\n",
         attitude.accel_x,
         attitude.accel_y,
         attitude.accel_z,
         attitude.gyro_x,
         attitude.gyro_y,
         attitude.gyro_z);
}

static void MpuApp_PrintFixed2(int32_t value)
{
  long abs_value;

  if (value < 0)
  {
    printf("-");
    value = -value;
  }

  abs_value = (long)value;
  printf("%ld.%02ld", abs_value / 100L, abs_value % 100L);
}

static int32_t MpuApp_AngleToFixed2(float value)
{
  if (value >= 0.0f)
  {
    return (int32_t)(value * 100.0f + 0.5f);
  }

  return (int32_t)(value * 100.0f - 0.5f);
}

/* by codex */
