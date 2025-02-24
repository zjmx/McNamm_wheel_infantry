/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ins_task.h"
#include "gimbal.h"
#include "USART_task.h"
#include "imu_temp_task.h"
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
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId INS_TASKHandle;
osThreadId GIMBAL_TASKHandle;
osThreadId USART_TASKHandle;
osThreadId Imu_Temp_TaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void INS_Task(void const * argument);
void GIMBAL_Task(void const * argument);
void USART_Task(void const * argument);
void Imu_temp_task(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of INS_TASK */
  osThreadDef(INS_TASK, INS_Task, osPriorityRealtime, 0, 512);
  INS_TASKHandle = osThreadCreate(osThread(INS_TASK), NULL);

  /* definition and creation of GIMBAL_TASK */
  osThreadDef(GIMBAL_TASK, GIMBAL_Task, osPriorityAboveNormal, 0, 512);
  GIMBAL_TASKHandle = osThreadCreate(osThread(GIMBAL_TASK), NULL);

  /* definition and creation of USART_TASK */
  osThreadDef(USART_TASK, USART_Task, osPriorityAboveNormal, 0, 128);
  USART_TASKHandle = osThreadCreate(osThread(USART_TASK), NULL);

  /* definition and creation of Imu_Temp_Task */
  osThreadDef(Imu_Temp_Task, Imu_temp_task, osPriorityAboveNormal, 0, 512);
  Imu_Temp_TaskHandle = osThreadCreate(osThread(Imu_Temp_Task), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_INS_Task */
/**
* @brief Function implementing the INS_TASK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_INS_Task */
void INS_Task(void const * argument)
{
  /* USER CODE BEGIN INS_Task */
  /* Infinite loop */
  for(;;)
  {
		INS_task();
  }
  /* USER CODE END INS_Task */
}

/* USER CODE BEGIN Header_GIMBAL_Task */
/**
* @brief Function implementing the GIMBAL_TASK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_GIMBAL_Task */
void GIMBAL_Task(void const * argument)
{
  /* USER CODE BEGIN GIMBAL_Task */
  /* Infinite loop */
  for(;;)
  {
		gimbal_task();
  }
  /* USER CODE END GIMBAL_Task */
}

/* USER CODE BEGIN Header_USART_Task */
/**
* @brief Function implementing the USART_TASK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_USART_Task */
void USART_Task(void const * argument)
{
  /* USER CODE BEGIN USART_Task */
  /* Infinite loop */
  for(;;)
  {
		USART_task();
  }
  /* USER CODE END USART_Task */
}

/* USER CODE BEGIN Header_Imu_temp_task */
/**
* @brief Function implementing the Imu_Temp_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Imu_temp_task */
void Imu_temp_task(void const * argument)
{
  /* USER CODE BEGIN Imu_temp_task */
  /* Infinite loop */
  for(;;)
  {
//    imu_temp_task();
  }
  /* USER CODE END Imu_temp_task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
