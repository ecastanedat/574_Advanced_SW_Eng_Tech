/*
  * ****************************************************************************
  * University of Michigan - Dearborn
  *
  * Course: ECE 574 Adv SW Engineering Methods
  * Project: HIL Test Tool
  ******************************************************************************
  * File Name          : Globals.h
  * Description        : Code for freertos applications
  ******************************************************************************
 */

#ifndef INC_GLOBALS_H_
#define INC_GLOBALS_H_

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32h7xx_hal.h"
#include "usart.h"
#include "string.h"
#include "cmsis_os.h"
#include "lwip/tcp.h"

/* Data Overhead--------------------------------------------------------------*/
/* USER CODE BEGIN Data_Overhead */

typedef enum{
	INIT,
	IDLE,
	STATE_2,
	EXIT
}SM_STATES;

typedef enum{
	TM_INIT,
	TM_MAIN,
	TM_CLEANUP,
	TM_EXIT
}TM_STATES;

/* USER CODE END Data_Overhead */

uint16_t timer_val;

extern struct netif gnetif;

#define CAN_MSG_FROM_DUT 0
#define CAN_MSG_TO_DUT   1

#define DUT_FAILURE      2

osThreadId_t ControllerHandle;
osThreadId_t CANHandle;


/* USER CODE BEGIN Prototypes */
void print_to_serial(char *myString);

/* USER CODE END Prototypes */

#endif /* INC_GLOBALS_H_ */
