/*
  ***************************************************************************************************************
  ***************************************************************************************************************
  ***************************************************************************************************************

  File:		  tcpServerRAW.h
  Author:     ControllersTech.com
  Updated:    26-Jul-2021

  ***************************************************************************************************************
  Copyright (C) 2017 ControllersTech.com

  This is a free software under the GNU license, you can redistribute it and/or modify it under the terms
  of the GNU General Public License version 3 as published by the Free Software Foundation.
  This software library is shared with public for educational purposes, without WARRANTY and Author is not liable for any damages caused directly
  or indirectly by this software, read more about this on the GNU General Public License.

  ***************************************************************************************************************
*/


#ifndef INC_TCPSERVERRAW_H_
#define INC_TCPSERVERRAW_H_

#include "Globals.h"

/* structure for maintaining connection infos to be passed as argument
   to LwIP callbacks*/
struct tcp_server_struct
{
  u8_t state;             /* current connection state */
  u8_t retries;
  struct tcp_pcb *pcb;    /* pointer on the current tcp_pcb */
  struct pbuf *p;         /* pointer on the received/to be transmitted pbuf */
};

struct tcp_server_struct *esTx;
struct tcp_pcb *myTcpb;

uint16_t inPort;
ip4_addr_t inIP;

void tcp_server_init(void);
void tcp_server_send(struct tcp_pcb *tpcb, struct tcp_server_struct *es);

#endif /* INC_TCPSERVERRAW_H_ */
