/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

#include "gpio.h"
#include "tim.h"
#include "usart.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "encoder.h"
#include "motor.h"
#include "mpu_app.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SPEED_KP       1.0f
#define SPEED_KI       1.0f
#define SPEED_LOOP_MS  10U
#define SPEED_OUT_MAX  100
#define SPEED_OUT_MIN  -100
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static int32_t speed_target_mm_s = 200; /* 目标速度 mm/s */

/* 增量式 PI 状态 */
static float    pi_last_error_l = 0.0f;
static float    pi_last_error_r = 0.0f;
static float    pi_output_l     = 0.0f;
static float    pi_output_r     = 0.0f;
static uint8_t  pi_inited       = 0;
static uint32_t pi_last_ms      = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/*
 * 增量式 PI 速度闭环。
 *   e(k)   = target - actual
 *   Δu     = Kp*(e(k)-e(k-1)) + Ki*Ts*e(k)
 *   output = clamp(output + Δu, min, max)
 * 首次调用直接用比例项初始化输出。
 */
static int16_t PI_SpeedStep(float *last_error, float *output,
                            int32_t actual, int32_t target) {
  float error, delta;

  error = (float)(target - actual);

  if (pi_inited == 0U) {
    *output = SPEED_KP * error;
    if      (*output > SPEED_OUT_MAX)  *output = SPEED_OUT_MAX;
    else if (*output < SPEED_OUT_MIN)  *output = SPEED_OUT_MIN;
    *last_error = error;
    return (int16_t)*output;
  }

  delta = SPEED_KP * (error - *last_error) +
          SPEED_KI * 0.01f * error;          /* Ts = 10ms = 0.01s */

  *output += delta;
  if      (*output > SPEED_OUT_MAX)  *output = SPEED_OUT_MAX;
  else if (*output < SPEED_OUT_MIN)  *output = SPEED_OUT_MIN;

  *last_error = error;
  return (int16_t)*output;
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
/* 程序入口，完成外设初始化并进入主循环。 */
int main(void) {
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick.
   */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM8_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  Motor_Init();
  Encoder_Init();
  pi_last_ms = HAL_GetTick();

  printf("speed pi init ok, target=%ld mm/s\r\n", (long)speed_target_mm_s);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    uint32_t now_ms;
    int32_t  actual_l, actual_r;
    int16_t  out_l, out_r;

    Encoder_Task();

    /* 每 10ms 跑一次增量式 PI 速度闭环 */
    now_ms = HAL_GetTick();
    if ((now_ms - pi_last_ms) >= SPEED_LOOP_MS) {
      pi_last_ms = now_ms;

      actual_l = Encoder_GetLeftSpeedMmS();
      actual_r = Encoder_GetRightSpeedMmS();

      out_l = PI_SpeedStep(&pi_last_error_l, &pi_output_l,
                           -actual_l, speed_target_mm_s);
      out_r = PI_SpeedStep(&pi_last_error_r, &pi_output_r,
                           -actual_r, speed_target_mm_s);

      if (pi_inited == 0U) {
        pi_inited = 1;
        printf("first PI: actual=(%ld,%ld) out=(%d,%d)\r\n",
               (long)actual_l, (long)actual_r, out_l, out_r);
      }

      Motor_SetDuty(out_l, out_r);
    }

    HAL_Delay(1);
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
/* 配置系统时钟、总线分频和 PLL 倍频。 */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
/* 出现错误时关闭中断并停在死循环，便于调试。 */
void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1) {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
/* 断言失败时上报文件名和行号。 */
void assert_failed(uint8_t *file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/* by codex */
