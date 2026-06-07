/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.c
  * @brief   This file provides code for the configuration
  *          of the CAN instances.
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "can.h"

/* USER CODE BEGIN 0 */
#include "usart.h"
#include "MotorControl.h"
#include "cmsis_os.h"
#include <string.h>

extern osMessageQueueId_t CanQueueHandle;


__IO CAN_t can = {0};
volatile uint32_t g_can_rx_cnt = 0;
volatile uint32_t g_can_queue_drop_cnt = 0;
/* USER CODE END 0 */

CAN_HandleTypeDef hcan1;

/* CAN1 init function */
void MX_CAN1_Init(void)
{
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 6;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_11TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = ENABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = ENABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
}

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN1)
  {
    __HAL_RCC_CAN1_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{
  if(canHandle->Instance==CAN1)
  {
    __HAL_RCC_CAN1_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_8|GPIO_PIN_9);
    HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
  }
}

/* USER CODE BEGIN 1 */
void USER_CAN1_Filter_Init(void)
{
  CAN_FilterTypeDef canFilter;
  __IO uint8_t id_o, im_o;
  __IO uint16_t id_l, id_h, im_l, im_h;

  id_o = 0x00;
  id_h = (uint16_t)(id_o >> 5);
  id_l = (uint16_t)(id_o << 11) | CAN_ID_EXT;
  im_o = 0x00;
  im_h = (uint16_t)(im_o >> 5);
  im_l = (uint16_t)(im_o << 11) | CAN_ID_EXT;

  canFilter.FilterBank = 0;
  canFilter.FilterMode = CAN_FILTERMODE_IDMASK;
  canFilter.FilterScale = CAN_FILTERSCALE_32BIT;
  canFilter.FilterIdHigh = id_h;
  canFilter.FilterIdLow = id_l;
  canFilter.FilterMaskIdHigh = im_h;
  canFilter.FilterMaskIdLow = im_l;
  canFilter.FilterFIFOAssignment = CAN_RX_FIFO0;
  canFilter.FilterActivation = ENABLE;
  canFilter.SlaveStartFilterBank = 14;

  while(HAL_CAN_ConfigFilter(&hcan1, &canFilter) != HAL_OK);
}

void can_SendCmd(__IO uint8_t *cmd, uint8_t len)
{
  static uint32_t TxMailbox;
  __IO uint8_t i = 0, j = 0, k = 0, l = 0, packNum = 0;

  j = len - 2;
  while(i < j)
  {
    k = j - i;
    can.CAN_TxMsg.StdId = 0x00;
    can.CAN_TxMsg.ExtId = ((uint32_t)cmd[0] << 8) | (uint32_t)packNum;
    can.txData[0] = cmd[1];
    can.CAN_TxMsg.IDE = CAN_ID_EXT;
    can.CAN_TxMsg.RTR = CAN_RTR_DATA;

    if(k < 8) {
      for(l=0; l<k; l++,i++) { can.txData[l + 1] = cmd[i + 2]; }
      can.CAN_TxMsg.DLC = k + 1;
    } else {
      for(l=0; l<7; l++,i++) { can.txData[l + 1] = cmd[i + 2]; }
      can.CAN_TxMsg.DLC = 8;
    }

    while(HAL_CAN_AddTxMessage(&hcan1,
                               (CAN_TxHeaderTypeDef *)(&can.CAN_TxMsg),
                               (uint8_t *)(&can.txData),
                               &TxMailbox) != HAL_OK);
    ++packNum;
  }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0,
      (CAN_RxHeaderTypeDef *)(&can.CAN_RxMsg),
      (uint8_t *)(can.rxData)) == HAL_OK)
  {
    can.rxFrameFlag = true;
    g_can_rx_cnt++;

    can_rx_msg_t msg;
    msg.extid = can.CAN_RxMsg.ExtId;
    msg.dlc = can.CAN_RxMsg.DLC;
    for (uint8_t i = 0; i < 8; i++)
    {
        msg.data[i] = can.rxData[i];
    }

    if (osMessageQueuePut(CanQueueHandle, &msg, 0, 0) != osOK)
    {
      g_can_queue_drop_cnt++;
    }
  }
}
/* USER CODE END 1 */ 
