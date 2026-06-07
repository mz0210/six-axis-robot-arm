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
#include "can.h"
#include "usart.h"
#include "gpio.h"
#include "Emm_V5.h"
#include "MotorControl.h"
#include "robot_kinematics.h"
#include "app_robot.h"
#include "tim.h"
#include "trajectory_planner.h"
#include "freertos.h"
#include "cmsis_os2.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
/* 关节配置示例（6轴） */
static joint_motor_t g_joint_cfg[6] = {
    {1, 0, 50.0f, 0.0f, 360.0f, 0.0f},   // J1
    {2, 1, 51.0f, -90.0f, 90.0f, 0.0f},   // J2
    {3, 0, 51.0f, 0.0f, 180.0f, 0.0f},    // J3
    {4, 0, 51.0f, 0.0f, 360.0f, 0.0f},    // J4
    {5, 1, 27.0f, -90.0f, 0.0f, 0.0f},    // J5
    {6, 0, 51.0f, 0.0f, 360.0f, 0.0f},    // J6
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void print_can_error(const char *tag)
{
    uint32_t e = HAL_CAN_GetError(&hcan1);
    LOG("[%s] CAN_ERR=0x%08lX\r\n", tag, e);
}
void MX_FREERTOS_Init(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int __io_putchar(int ch)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* MCU Configuration--------------------------------------------------------*/
  HAL_Init();
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN1_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  uart1_rx_start();
  /* 启动 TIM3 20ms 周期中断 */
  if (HAL_TIM_Base_Start_IT(&htim3) != HAL_OK)
  {
      LOG("[ERR] TIM3 start IT failed\r\n");
      Error_Handler();
  }

  /* 轨迹模块初始化 */
  trajectory_init(0.02f); // 20ms

  /* CAN 初始化 */
  USER_CAN1_Filter_Init();

  if (HAL_CAN_Start(&hcan1) != HAL_OK)
  {
      LOG("[ERR] HAL_CAN_Start failed\r\n");
      Error_Handler();
  }

  if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
  {
      LOG("[ERR] CAN RX IRQ enable failed\r\n");
      Error_Handler();
  }

  print_can_error("AfterCANStart");
  HAL_Delay(300);

  /* 初始化关节封装层 */
  joint_motor_init_all(g_joint_cfg, 6);

  /* 使能六轴 */
  for (uint8_t i = 0; i < 6; i++)
  {
      joint_motor_enable(i, true);
      HAL_Delay(50);
  }

  /* 清除六轴堵转保护 */
  for (uint8_t addr = 1; addr <= 6; addr++)
  {
      Emm_V5_Reset_Clog_Pro(addr);
      HAL_Delay(50);
  }

  /* 启动 FreeRTOS */
  osKernelInitialize();
  MX_FREERTOS_Init();
  osKernelStart();

  /* Should never get here */
  while (1)
  {
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                              | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* 这里后面可以放一些 main 专属但暂时未迁移的辅助函数 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number */
}
#endif /* USE_FULL_ASSERT */
