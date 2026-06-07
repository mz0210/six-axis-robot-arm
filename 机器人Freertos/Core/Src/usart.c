/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
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
#include "usart.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

static char uart_tx_buf[256];
/* USER CODE BEGIN 0 */
volatile char uart1_line[UART1_RX_BUFF_SIZE] = {0};
volatile bool uart1_line_ready = false;



static uint8_t g_uart1_rx_ch = 0;
static uint16_t g_uart1_rx_idx = 0;

/* USER CODE END 0 */

UART_HandleTypeDef huart1;

/* USART1 init function */

void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */

  /* USER CODE END USART1_MspInit 0 */
    /* USART1 clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART1 interrupt Init */
    HAL_NVIC_SetPriority(USART1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspInit 1 */

  /* USER CODE END USART1_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();

    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9|GPIO_PIN_10);

    /* USART1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspDeInit 1 */

  /* USER CODE END USART1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
/* USER CODE BEGIN 1 */

void uart1_rx_start(void)
{
    g_uart1_rx_idx = 0;
    uart1_line_ready = false;

    // 启动1字节中断接收
    HAL_UART_Receive_IT(&huart1, &g_uart1_rx_ch, 1);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        char c = (char)g_uart1_rx_ch;

        // 遇到换行：提交一行
        if (c == '\r' || c == '\n')
        {
            if (g_uart1_rx_idx > 0)
            {
                // 结束符
                if (g_uart1_rx_idx >= UART1_RX_BUFF_SIZE)
                    g_uart1_rx_idx = UART1_RX_BUFF_SIZE - 1;

                uart1_line[g_uart1_rx_idx] = '\0';
                uart1_line_ready = true;

                // 下一行重新开始（CmdTask会很快把 ready 清掉）
                g_uart1_rx_idx = 0;
            }
            // 空行直接忽略
        }
        else
        {
            // 正常字符入缓冲
            if (g_uart1_rx_idx < (UART1_RX_BUFF_SIZE - 1))
            {
                uart1_line[g_uart1_rx_idx++] = c;
            }
            else
            {
                // 溢出：丢弃并重置（防止卡死）
                g_uart1_rx_idx = 0;
            }
        }

        // 继续接收下一个字节
        HAL_UART_Receive_IT(&huart1, &g_uart1_rx_ch, 1);
    }
}

/* USER CODE END 1 */
void safe_printf(const char *format, ...)
{
    va_list args;
    va_start(args, format);
    vsnprintf(uart_tx_buf, sizeof(uart_tx_buf), format, args);
    va_end(args);

    HAL_UART_Transmit(&huart1, (uint8_t *)uart_tx_buf, strlen(uart_tx_buf), HAL_MAX_DELAY);
}

void safe_printf_from_isr(const char *format, ...)
{
    va_list args;
    va_start(args, format);
    vsnprintf(uart_tx_buf, sizeof(uart_tx_buf), format, args);
    va_end(args);

    HAL_UART_Transmit(&huart1, (uint8_t *)uart_tx_buf, strlen(uart_tx_buf), HAL_MAX_DELAY);
}
/* USER CODE END 1 */
