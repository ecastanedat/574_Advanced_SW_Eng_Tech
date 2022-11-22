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

#define  START 1
#define  SEND  1

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

uint8_t current_test_mode = 0;

/* USER CODE END Variables */
/* Definitions for Controller */
//osThreadId_t ControllerHandle;
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
//osThreadId_t CANHandle;
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
void resetMyCANTxData_TM1(void);
void resetMyCANTxData_TM2(void);
void resetMyCANTxData_TM3(void);

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
  //MX_LWIP_Init();
  /* USER CODE BEGIN Controller_handler */
  SM_STATES state = INIT;
  BaseType_t status;
  uint32_t test_mode;

  /*init code for TCP Server*/
  //tcp_server_init();

  /* Infinite loop */
  for(;;)
  {
	  switch(state)
	  {
		  case INIT:
			        MX_LWIP_Init();
			        tcp_server_init();
			        ST7735_Init(3);
			        fillScreen(BLACK);
			  		ST7735_WriteString(0, 0, "  Mini HIL Tester v0.1", Font_7x10, WHITE,BLACK);

			  		state = IDLE;
					break;

		  case IDLE:
			        status = xTaskNotifyWait(0, 0, &test_mode, pdMS_TO_TICKS(20));

			        if(status == pdPASS)
			        {

			        	switch(test_mode)
			        	{
			        		case TEST_MODE_1: current_test_mode = TEST_MODE_1;
			        			              ST7735_WriteString(0, 20, "Test Mode 1: Testing..", Font_7x10, WHITE,BLACK);
			        			              xTaskNotify((TaskHandle_t)TestMode1Handle, START, eSetValueWithOverwrite);
				        	                  break;

			        		case TEST_MODE_2: current_test_mode = TEST_MODE_2;
			        						  ST7735_WriteString(0, 30, "Test Mode 2: Testing..", Font_7x10, WHITE,BLACK);
			        					      xTaskNotify((TaskHandle_t)TestMode2Handle, START, eSetValueWithOverwrite);
			        			              break;

			        		case TEST_MODE_3: current_test_mode = TEST_MODE_3;
			        						  ST7735_WriteString(0, 40, "Test Mode 3: Testing..", Font_7x10, WHITE,BLACK);
			        					      xTaskNotify((TaskHandle_t)TestMode3Handle, START, eSetValueWithOverwrite);
			        			              break;

			        		case TEST_MODE_4: current_test_mode = TEST_MODE_4;
			        			              ST7735_WriteString(0, 12, "TEST MODE 4!", Font_7x10, WHITE,BLACK);

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
  BaseType_t status;
  uint32_t parameter1, failure_flag;
  uint8_t expected_speed;
  TM_STATES TM1_state = TM_INIT;

  /* Infinite loop */
  for(;;)
  {

	  switch(TM1_state)
	  {
		  case TM_INIT:
			               status = xTaskNotifyWait(0, 0, &parameter1, pdMS_TO_TICKS(20));
		  	  	  	  	   if(status == pdPASS)
		  			       {
								resetMyCANTxData_TM1();                                                             // Resets the Tx CAN frame for a new test.
								TM1_state = TM_MAIN;
								HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);
								HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
								HAL_GPIO_WritePin(LED_YELLOW_GPIO_Port, LED_YELLOW_Pin, GPIO_PIN_RESET);
								expected_speed = myTxData[6];
								HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, myTxData);                       // Send initial CAN msg to DUT to indicate START of test.

		  			        	status = pdFALSE;															        //Reset status flag to be ready for a new test.
		  			       }

		  	  	  	  	   break;

		  case TM_MAIN:    /*Test Case #1: SPEED ENGINE TEST*/
			               status = xTaskNotifyWait(0, 0, &failure_flag, pdMS_TO_TICKS(10));                        // Wait for DUT to respond with CAN ID 0x762.
			               if(status == pdPASS)
			               {
			            	   if(failure_flag != DUT_FAILURE)
			            	   {
			            		   if(myTxData[6] < 0xFF)
			                       {
			            		       myTxData[6]++;                                                               // Increment LSB and send the msg back to the DUT.
			            		       HAL_GPIO_TogglePin(LED_YELLOW_GPIO_Port, LED_YELLOW_Pin);
			            		   	   expected_speed = myTxData[6];
			            		   	   HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, myTxData);
			            		   }
			            		   else                                                                             // Finish test and print PASS results to TFT screen.
			            		   {
			            			   HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin);
			            		   	   ST7735_WriteString(0, 20, "                       ", Font_7x10, WHITE,BLACK);
			            		       ST7735_WriteString(0, 20, "Test Mode 1: PASS", Font_7x10, WHITE,BLACK);
			            		   	   TM1_state = TM_CLEANUP;                                                      // Send Test Mode to CLEANUP.
			            		   	   //tcp_server_send(tpcb, esTx);
			            		   }
			            	   }
			            	   else																					// Finish test and print FAIL results to TFT screen.
			            	   {
			            		   HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin);
			            		   ST7735_WriteString(0, 20, "                       ", Font_7x10, WHITE,BLACK);
			            		   ST7735_WriteString(0, 20, "Test Mode 1: FAIL", Font_7x10, WHITE,BLACK);
			            		   TM1_state = TM_CLEANUP;
			            	   }
			               }

			               /* USER CODE BEGIN Test Case #2 */

			               /* USER CODE END Test Case #2 */

			               /* USER CODE BEGIN Test Case #3 */

			               /* USER CODE END Test Case #3 */
			               break;

		  case TM_CLEANUP:
			               myTxData[4] = 0;                                                                         // Send Speed Engine back to ZERO.
			               HAL_GPIO_WritePin(LED_YELLOW_GPIO_Port, LED_YELLOW_Pin, GPIO_PIN_RESET);
			               HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, myTxData);
			               TM1_state = TM_INIT;
		  			       break;
	  }

      osDelay(10);
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

  uint32_t parameter1, failure_flag;
  uint8_t expected_result;
  BaseType_t status;
  TM_STATES TM2_state = TM_INIT;

  /* Infinite loop */
  for(;;)
  {
	  switch(TM2_state)
	  {
		  case TM_INIT:
						   status = xTaskNotifyWait(0, 0, &parameter1, pdMS_TO_TICKS(20));
						   if(status == pdPASS)
						   {
								resetMyCANTxData_TM2();                                                             // Resets the Tx CAN frame for a new test.
								TM2_state = TM_MAIN;
								HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);
								HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
								HAL_GPIO_WritePin(LED_YELLOW_GPIO_Port, LED_YELLOW_Pin, GPIO_PIN_RESET);
								expected_result = myTxData[6];
								HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, myTxData);                       // Send initial CAN msg to DUT to indicate START of test.

								status = pdFALSE;															        //Reset status flag to be ready for a new test.
						   }

						   break;

		  case TM_MAIN:    /*Test Case #1: HEADLIGHT TEST*/
			               status = xTaskNotifyWait(0, 0, &failure_flag, pdMS_TO_TICKS(10));                        // Wait for DUT to respond with CAN ID 0x762.
			               if(status == pdPASS)
						   {
							   if(failure_flag != DUT_FAILURE)
							   {
								   HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin);
								   ST7735_WriteString(0, 30, "                        ", Font_7x10, WHITE,BLACK);
								   ST7735_WriteString(0, 30, "Test Mode 2: PASS", Font_7x10, WHITE,BLACK);
								   TM2_state = TM_CLEANUP;
							   }
							   else																					// Finish test and print FAIL results to TFT screen.
							   {
								   HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin);
								   ST7735_WriteString(0, 30, "                       ", Font_7x10, WHITE,BLACK);
								   ST7735_WriteString(0, 30, "Test Mode 2: FAIL", Font_7x10, WHITE,BLACK);
								   TM2_state = TM_CLEANUP;
							   }
						   }

						   break;

		  case TM_CLEANUP:
			  	  	  	   //myTxData[4] = 0;
			  			   //HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, myTxData);
			               HAL_GPIO_WritePin(LED_YELLOW_GPIO_Port, LED_YELLOW_Pin, GPIO_PIN_RESET);
			  			   TM2_state = TM_INIT;
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

  uint32_t parameter1, failure_flag;
  uint8_t expected_result;
  BaseType_t status;
  TM_STATES TM3_state = TM_INIT;

  /* Infinite loop */
  for(;;)
  {
	  switch(TM3_state)
	  {
		  case TM_INIT:
			            status = xTaskNotifyWait(0, 0, &parameter1, pdMS_TO_TICKS(20));
					    if(status == pdPASS)
					    {
							resetMyCANTxData_TM3();                                                             // Resets the Tx CAN frame for a new test.
							TM3_state = TM_MAIN;
							HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);
							HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
							HAL_GPIO_WritePin(LED_YELLOW_GPIO_Port, LED_YELLOW_Pin, GPIO_PIN_RESET);
							expected_result = myTxData[6];
							HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, myTxData);                       // Send initial CAN msg to DUT to indicate START of test.

							status = pdFALSE;															        //Reset status flag to be ready for a new test.
					    }

			            break;

		  case TM_MAIN:    /*Test Case #3: HAZARD LIGHTS TEST*/
						   status = xTaskNotifyWait(0, 0, &failure_flag, pdMS_TO_TICKS(10));                        // Wait for DUT to respond with CAN ID 0x762.
						   if(status == pdPASS)
						   {
							   if(failure_flag != DUT_FAILURE)
							   {
								   HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin);
								   ST7735_WriteString(0, 40, "                        ", Font_7x10, WHITE,BLACK);
								   ST7735_WriteString(0, 40, "Test Mode 3: PASS", Font_7x10, WHITE,BLACK);
								   TM3_state = TM_CLEANUP;
							   }
							   else																					// Finish test and print FAIL results to TFT screen.
							   {
								   HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin);
								   ST7735_WriteString(0, 40, "                       ", Font_7x10, WHITE,BLACK);
								   ST7735_WriteString(0, 40, "Test Mode 3: FAIL", Font_7x10, WHITE,BLACK);
								   TM3_state = TM_CLEANUP;
							   }
						   }

			               break;

		  case TM_CLEANUP:
			               HAL_GPIO_WritePin(LED_YELLOW_GPIO_Port, LED_YELLOW_Pin, GPIO_PIN_RESET);
			               TM3_state = TM_INIT;
			               break;
	  }

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
  BaseType_t status;
  uint32_t failure_flag;

  /* Infinite loop */
  for(;;)
  {
	  status = xTaskNotifyWait(0, 0, &failure_flag, pdMS_TO_TICKS(10));

	  if(status == pdPASS)
	  {
		  switch(current_test_mode)
		  {
		  	  case TEST_MODE_1:   xTaskNotify((TaskHandle_t)TestMode1Handle, failure_flag, eSetValueWithOverwrite);  break;
		  	  case TEST_MODE_2:   xTaskNotify((TaskHandle_t)TestMode2Handle, failure_flag, eSetValueWithOverwrite);  break;
		  	  case TEST_MODE_3:   xTaskNotify((TaskHandle_t)TestMode3Handle, failure_flag, eSetValueWithOverwrite);  break;
		  	  case TEST_MODE_4:   xTaskNotify((TaskHandle_t)TestMode2Handle, failure_flag, eSetValueWithOverwrite);  break;
		  }
	  }

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
void resetMyCANTxData_TM1(void)
{
	myTxData[0] = 0x05;
	myTxData[1] = 0x22;
	myTxData[2] = 0xFE;
	myTxData[3] = 0x01;

	myTxData[4] = 0x01;
	myTxData[5] = 0x00;
	myTxData[6] = 0x00;
	myTxData[7] = 0x00;
}

void resetMyCANTxData_TM2(void)
{
	myTxData[0] = 0x05;
	myTxData[1] = 0x22;
	myTxData[2] = 0xFE;
	myTxData[3] = 0x02;

	myTxData[4] = 0x01;
	myTxData[5] = 0x00;
	myTxData[6] = 0x00;
	myTxData[7] = 0x00;
}

void resetMyCANTxData_TM3(void)
{
	myTxData[0] = 0x05;
	myTxData[1] = 0x22;
	myTxData[2] = 0xFE;
	myTxData[3] = 0x03;

	myTxData[4] = 0x01;
	myTxData[5] = 0x00;
	myTxData[6] = 0x00;
	myTxData[7] = 0x00;
}
/* USER CODE END Application */

