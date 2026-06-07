/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms of LICENSE file in the root directory
  * of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "app_robot.h"
#include "usart.h"
#include "robot_kinematics.h"
#include "trajectory_planner.h"
#include "MotorControl.h"
#include "can.h"
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef enum {
  MOTION_JOINT = 0,
  MOTION_POSE,
  MOTION_LINE
} motion_type_t;

typedef struct {
  motion_type_t type;
  union {
    struct {
      float q[JOINT_NUM];
    } joint;

    struct {
      pose_t target;
    } pose;

    struct {
      pose_t p0;
      pose_t p1;
      float T;
    } line;
  } u;
} motion_req_t;


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MOTION_QUEUE_LEN  5
#define CAN_QUEUE_LEN     20
#define ENABLE_J1_REL_TEST  1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

extern volatile char uart1_line[UART1_RX_BUFF_SIZE];
extern volatile bool uart1_line_ready;
extern volatile uint8_t g_traj_tick;


osThreadId_t defaultTaskHandle;
osThreadId_t CmdTaskHandle;
osThreadId_t TrajTaskHandle;
osThreadId_t CanRxTaskHandle;

osMessageQueueId_t MotionQueueHandle;
osMessageQueueId_t CanQueueHandle;

osMutexId_t UartMutexHandle;

const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t)osPriorityNormal,
};

const osThreadAttr_t CmdTask_attributes = {
  .name = "CmdTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t)osPriorityAboveNormal,
};

const osThreadAttr_t TrajTask_attributes = {
  .name = "TrajTask",
  .stack_size = 1536 * 4,
  .priority = (osPriority_t)osPriorityHigh,
};

const osThreadAttr_t CanRxTask_attributes = {
  .name = "CanRxTask",
  .stack_size = 768 * 4,
  .priority = (osPriority_t)osPriorityAboveNormal,
};

const osMessageQueueAttr_t MotionQueue_attributes = {
  .name = "MotionQueue"
};

const osMessageQueueAttr_t CanQueue_attributes = {
  .name = "CanQueue"
};

const osMutexAttr_t UartMutex_attributes = {
  .name = "UartMutex"
};

/* USER CODE END Variables */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void StartDefaultTask(void *argument);
void StartCmdTask(void *argument);
void StartTrajTask(void *argument);
void StartCanRxTask(void *argument);
void UartPrint(const char *str);


/* USER CODE END FunctionPrototypes */



void MX_FREERTOS_Init(void)
{
  UartMutexHandle = osMutexNew(&UartMutex_attributes);

  MotionQueueHandle = osMessageQueueNew(MOTION_QUEUE_LEN, sizeof(motion_req_t), &MotionQueue_attributes);
  CanQueueHandle    = osMessageQueueNew(CAN_QUEUE_LEN, sizeof(can_rx_msg_t), &CanQueue_attributes);

  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);
  CmdTaskHandle     = osThreadNew(StartCmdTask, NULL, &CmdTask_attributes);
  TrajTaskHandle    = osThreadNew(StartTrajTask, NULL, &TrajTask_attributes);
  CanRxTaskHandle   = osThreadNew(StartCanRxTask, NULL, &CanRxTask_attributes);
}

void StartDefaultTask(void *argument)
{
  for(;;)
  {
    osDelay(1000);
  }
}

void StartCmdTask(void *argument)
{
  motion_req_t req;
  float q[6];
  pose_t target;

  for(;;)
  {
    if (uart1_line_ready)
    {
      uart1_line_ready = false;

      if (app_parse_cmd_angles((const char*)uart1_line, q))
      {
        req.type = MOTION_JOINT;
        memcpy(req.u.joint.q, q, sizeof(q));

        if (osMessageQueuePut(MotionQueueHandle, &req, 0, 0) == osOK)
        {
          UartPrint("[CmdTask] JOINT motion queued");
        }
        else
        {
          UartPrint("[CmdTask] MotionQueue full");
        }
      }
      else if (strncmp((const char*)uart1_line, "LINEMOVE", 8) == 0)
      {
        float x1, y1, z1, rx1, ry1, rz1;
        float x2, y2, z2, rx2, ry2, rz2;
        float T;

        int n = sscanf((const char*)uart1_line,
                       "LINEMOVE %f %f %f %f %f %f %f %f %f %f %f %f %f",
                       &x1, &y1, &z1, &rx1, &ry1, &rz1,
                       &x2, &y2, &z2, &rx2, &ry2, &rz2,
                       &T);

        if (n == 13)
        {
          req.type = MOTION_LINE;

          req.u.line.p0.x = x1;
          req.u.line.p0.y = y1;
          req.u.line.p0.z = z1;
          req.u.line.p0.roll = rx1;
          req.u.line.p0.pitch = ry1;
          req.u.line.p0.yaw = rz1;

          req.u.line.p1.x = x2;
          req.u.line.p1.y = y2;
          req.u.line.p1.z = z2;
          req.u.line.p1.roll = rx2;
          req.u.line.p1.pitch = ry2;
          req.u.line.p1.yaw = rz2;

          req.u.line.T = T;

          pose_set_rpy_deg(&req.u.line.p0);
          pose_set_rpy_deg(&req.u.line.p1);

          if (osMessageQueuePut(MotionQueueHandle, &req, 0, 0) == osOK)
          {
            UartPrint("[CmdTask] LINE motion queued");
          }
          else
          {
            UartPrint("[CmdTask] MotionQueue full");
          }
        }
        else
        {
          UartPrint("[CmdTask] LINEMOVE format error");
        }
      }
      else if (app_parse_cmd_pose((const char*)uart1_line, &target))
      {
        req.type = MOTION_POSE;
        req.u.pose.target = target;
        pose_set_rpy_deg(&req.u.pose.target);

        if (osMessageQueuePut(MotionQueueHandle, &req, 0, 0) == osOK)
        {
          UartPrint("[CmdTask] POSE motion queued");
        }
        else
        {
          UartPrint("[CmdTask] MotionQueue full");
        }
      }
      else
      {
        UartPrint("[CmdTask] unknown command");
      }
    }

    osDelay(1);
  }
}

void StartTrajTask(void *argument)
{
  motion_req_t req;

  for(;;)
  {
    if (osMessageQueueGet(MotionQueueHandle, &req, NULL, osWaitForever) == osOK)
    {
            UartPrint("[TrajTask] state=BUSY");

      if (req.type == MOTION_JOINT)
      {
        float q_cur[6];
        joint_motor_sync_all(200);
        for (int i = 0; i < 6; i++)
        {
          q_cur[i] = joint_motor_get_angle((uint8_t)i);
        }

        app_motion_cfg_t mc = {
          .steps = 80,
          .total_time_s = 4.0f,
          .vel_deg_s = 30.0f,
          .acc = 12,
          .sync_every = 1,   // ✅ 每段回读
        };
#if ENABLE_J1_REL_TEST
  UartPrint("[TEST] J1 +10deg rel");

  joint_motor_sync_all(200);
  float j1_before = joint_motor_get_angle(0);
  LOG("[TEST] j1_before=%.2f\r\n", j1_before);

  joint_motor_move_rel_deg(0, 10.0f, 30.0f, 25);  // J1 相对 +10deg
  osDelay(1500);                                   // 等它走完（先给足时间）

  joint_motor_sync_all(200);
  float j1_after = joint_motor_get_angle(0);
  LOG("[TEST] j1_after=%.2f delta=%.2f\r\n", j1_after, (j1_after - j1_before));
#endif
        app_move_joints_scurve_stable(q_cur, req.u.joint.q, &mc);

        UartPrint("[TrajTask] JOINT motion done");
      }
      else if (req.type == MOTION_POSE)
      {
        float q_cur[6], q_out[6];
        joint_motor_sync_all(200);
        for (int i = 0; i < 6; i++)
        {
          q_cur[i] = joint_motor_get_angle((uint8_t)i);
        }

        int iter = 0;
        float pos_err = 0.0f, ori_err = 0.0f;
        bool ok = ikine_deg_ex(&req.u.pose.target, q_cur, q_out, &iter, &pos_err, &ori_err);

        if (ok)
        {
          app_motion_cfg_t mc = {
            .steps = 80,
            .total_time_s = 2.0f,
            .vel_deg_s = 60.0f,
            .acc = 25,
            .sync_every = 8,
          };

          app_move_joints_scurve_stable(q_cur, q_out, &mc);
          
          UartPrint("[TrajTask] POSE motion done");
        }
        else
        {
         
          UartPrint("[TrajTask] POSE IK failed");
        }
      }
      else if (req.type == MOTION_LINE)
      {
        bool ok = start_linear_move(&req.u.line.p0, &req.u.line.p1, req.u.line.T);
        if (ok)
        {
          while (trajectory_get_state() == TRAJ_RUNNING)
          {
            if (g_traj_tick)
            {
              g_traj_tick = 0;
              trajectory_timer_callback();
            }
            osDelay(1);
          }

         
          UartPrint("[TrajTask] LINE motion done");
        }
        else
        {
          
          UartPrint("[TrajTask] LINE motion start failed");
        }
      }
      else
      {
       
        UartPrint("[TrajTask] unknown motion type");
      }
    }
  }
}

void StartCanRxTask(void *argument)
{
  can_rx_msg_t msg;

  for(;;)
  {
    if (osMessageQueueGet(CanQueueHandle, &msg, NULL, osWaitForever) == osOK)
    {
      joint_motor_on_can_frame(msg.extid, msg.data, msg.dlc);
    }
  }
}

void UartPrint(const char *str)
{
  osMutexAcquire(UartMutexHandle, osWaitForever);
  LOG("%s\r\n", str);
  osMutexRelease(UartMutexHandle);
}
