/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "fdcan.h"
#include "Globals.h"
#include "string.h"
#include "ST7735.h"
#include "GFX_FUNCTIONS.h"
#include "tcpServerRAW.h"
#include "lwip.h"
#include "tim.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define  TEST_MODE_1 1
#define  TEST_MODE_2 2
#define  TEST_MODE_3 3
#define  TEST_MODE_4 4
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for Controller */
osThreadId_t ControllerHandle;
const osThreadAttr_t Controller_attributes = {
  .name = "Controller",
  .stack_size = 300 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for TestMode1 */
osThreadId_t TestMode1Handle;
const osThreadAttr_t TestMode1_attributes = {
  .name = "TestMode1",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for TestMode2 */
osThreadId_t TestMode2Handle;
const osThreadAttr_t TestMode2_attributes = {
  .name = "TestMode2",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for TestMode3 */
osThreadId_t TestMode3Handle;
const osThreadAttr_t TestMode3_attributes = {
  .name = "TestMode3",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for TCPSvr */
osThreadId_t TCPSvrHandle;
const osThreadAttr_t TCPSvr_attributes = {
  .name = "TCPSvr",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for CAN */
osThreadId_t CANHandle;
const osThreadAttr_t CAN_attributes = {
  .name = "CAN",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for TestMode4 */
osThreadId_t TestMode4Handle;
const osThreadAttr_t TestMode4_attributes = {
  .name = "TestMode4",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void Controller_handler(void *argument);
void TestMode1_handler(void *argument);
void TestMode2_handler(void *argument);
void TestMode3_handler(void *argument);
void TCPSvr_handler(void *argument);
void CAN_handler(void *argument);
void TestMode4_handler(void *argument);

extern void MX_LWIP_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void configureTimerForRunTimeStats(void);
unsigned long getRunTimeCounterValue(void);

/* USER CODE BEGIN 1 */
/* Functions needed when configGENERATE_RUN_TIME_STATS is on */
__weak void configureTimerForRunTimeStats(void)
{

}

__weak unsigned long getRunTimeCounterValue(void)
{
return 0;
}
/* USER CODE END 1 */

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
  /* creation of Controller */
  ControllerHandle = osThreadNew(Controller_handler, NULL, &Controller_attributes);

  /* creation of TestMode1 */
  TestMode1Handle = osThreadNew(TestMode1_handler, NULL, &TestMode1_attributes);

  /* creation of TestMode2 */
  TestMode2Handle = osThreadNew(TestMode2_handler, NULL, &TestMode2_attributes);

  /* creation of TestMode3 */
  TestMode3Handle = osThreadNew(TestMode3_handler, NULL, &TestMode3_attributes);

  /* creation of TCPSvr */
  TCPSvrHandle = osThreadNew(TCPSvr_handler, NULL, &TCPSvr_attributes);

  /* creation of CAN */
  CANHandle = osThreadNew(CAN_handler, NULL, &CAN_attributes);

  /* creation of TestMode4 */
  TestMode4Handle = osThreadNew(TestMode4_handler, NULL, &TestMode4_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_Controller_handler */
/**
  * @brief  Function implementing the Controller thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Controller_handler */
void Controller_handler(void *argument)
{
  /* init code for LWIP */
  MX_LWIP_Init();
  /* USER CODE BEGIN Controller_handler */
  SM_STATES state = INIT;
  BaseType_t status;
  uint32_t test_mode;

  /*init code for TCP Server*/
  tcp_server_init();

  /* Infinite loop */
  for(;;)
  {
	  switch(state)
	  {
		  case INIT:
			        print_to_serial("Hello FreeRTOS!");
			  	    ST7735_SetRotation(3);
			  		ST7735_WriteString(0, 0, "  Mini HIL Tester v0.1", Font_7x10, WHITE,BLACK);
			  		HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, myTxData);
			  		Toggle_CAN_Data();

			  		state = IDLE;
					break;

		  case IDLE:
			        status = xTaskNotifyWait(0, 0, &test_mode, pdMS_TO_TICKS(20));

			        if(status == pdPASS)
			        {

			        	switch(test_mode)
			        	{
			        		case TEST_MODE_1:
			        			              ST7735_WriteString(0, 12, "TEST MODE 1!", Font_7x10, WHITE,BLACK);
				        	        		  HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, myTxData);
				        	                  Toggle_CAN_Data();

				        	                  break;

			        		case TEST_MODE_2:
			        					      ST7735_WriteString(0, 12, "TEST MODE 2!", Font_7x10, WHITE,BLACK);
			        					      HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, myTxData);
			        					      Toggle_CAN_Data();
			        			              break;

			        		case TEST_MODE_3:
			        			              ST7735_WriteString(0, 12, "TEST MODE 3!", Font_7x10, WHITE,BLACK);
			        						  HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, myTxData);
			        						  Toggle_CAN_Data();
			        			              break;

			        		case TEST_MODE_4:
			        			              ST7735_WriteString(0, 12, "TEST MODE 4!", Font_7x10, WHITE,BLACK);
			        						  HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, myTxData);
			        						  Toggle_CAN_Data();
			        			              break;

			        		default:          break;
			        	}
			        }

			        state = IDLE;
			        break;

		  default:  break;
	  }

	  osDelay(1);
  }
  /* USER CODE END Controller_handler */
}

/* USER CODE BEGIN Header_TestMode1_handler */
/**
* @brief Function implementing the TestMode1 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_TestMode1_handler */
void TestMode1_handler(void *argument)
{
  /* USER CODE BEGIN TestMode1_handler */
  uint32_t flag;
  BaseType_t status;
  TM_STATES TM1_state = TM_INIT;

  /* Infinite loop */
  for(;;)
  {

	  switch(TM1_state)
	  {
		  case TM_INIT:
					       break;

		  case TM_MAIN:
			               break;

		  case TM_CLEANUP:
		  			       break;
	  }

      osDelay(1);
  }
  /* USER CODE END TestMode1_handler */
}

/* USER CODE BEGIN Header_TestMode2_handler */
/**
* @brief Function implementing the TestMode2 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_TestMode2_handler */
void TestMode2_handler(void *argument)
{
  /* USER CODE BEGIN TestMode2_handler */

  uint32_t flag;
  BaseType_t status;
  TM_STATES TM2_state = TM_INIT;

  /* Infinite loop */
  for(;;)
  {
	  switch(TM2_state)
	  {
		  case TM_INIT:
						   break;

		  case TM_MAIN:
						   break;

		  case TM_CLEANUP:
						   break;
	  }

	  osDelay(1);
  }
  /* USER CODE END TestMode2_handler */
}

/* USER CODE BEGIN Header_TestMode3_handler */
/**
* @brief Function implementing the TestMode3 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_TestMode3_handler */
void TestMode3_handler(void *argument)
{
  /* USER CODE BEGIN TestMode3_handler */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END TestMode3_handler */
}

/* USER CODE BEGIN Header_TCPSvr_handler */
/**
* @brief Function implementing the TCPSvr thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_TCPSvr_handler */
void TCPSvr_handler(void *argument)
{
  /* USER CODE BEGIN TCPSvr_handler */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END TCPSvr_handler */
}

/* USER CODE BEGIN Header_CAN_handler */
/**
* @brief Function implementing the CAN thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_CAN_handler */
void CAN_handler(void *argument)
{
  /* USER CODE BEGIN CAN_handler */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END CAN_handler */
}

/* USER CODE BEGIN Header_TestMode4_handler */
/**
* @brief Function implementing the TestMode4 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_TestMode4_handler */
void TestMode4_handler(void *argument)
{
  /* USER CODE BEGIN TestMode4_handler */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END TestMode4_handler */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

