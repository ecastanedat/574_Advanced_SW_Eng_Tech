/*
 * Globals.h
 *
 *  Created on: Nov 29, 2021
 *      Author: uib01493
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

/* structure for maintaining connection infos to be passed as argument
   to LwIP callbacks*/
struct tcp_server_struct
{
  u8_t state;             /* current connection state */
  u8_t retries;
  struct tcp_pcb *pcb;    /* pointer on the current tcp_pcb */
  struct pbuf *p;         /* pointer on the received/to be transmitted pbuf */
};

uint16_t timer_val;

extern struct netif gnetif;

osThreadId_t ControllerHandle;

/* USER CODE BEGIN Prototypes */
void print_to_serial(char *myString);

/* USER CODE END Prototypes */

#endif /* INC_GLOBALS_H_ */
