#ifndef __CAN_H
#define	__CAN_H

#include "gd32f10x.h"
#include <stdbool.h>
#include <stdio.h>

#include "main.h"

#ifdef USER_DEBUG
#define CAN_DEBUG_EN	0		/* 0:Non DEBUG status, 1:DEBUG status */
#else
#define CAN_DEBUG_EN	0
#endif
#define CAN_DEBUG(fmt,arg...)	do{if(CAN_DEBUG_EN){printf(fmt,##arg);printf("[%s][%d]\r\n",__func__,__LINE__);}}while(0)
#define CAN_RETURN_FALSE	do{CAN_DEBUG("Return failed");return false;}while(0)
//#define CAN_RETURN_TRUE do{CAN_DEBUG("Return success");return true;}while(0)
#define CAN_RETURN_TRUE do{return true;}while(0)

#define CAN_RX_PIN                 GPIO_PIN_11
#define CAN_TX_PIN                 GPIO_PIN_12
#define CAN_TX_GPIO_PORT           GPIOA
#define CAN_RX_GPIO_PORT           GPIOA

#define CAN0_USED

typedef struct {
	uint32_t uiId;
	uint8_t ucDLC;
	uint8_t aucData[8];
} CAN_RBUF_S;

#define FDCAN_MAX_DATA_CACHE	64

typedef struct {
	CAN_RBUF_S astRBuf[FDCAN_MAX_DATA_CACHE];
	uint16_t usRIdx;	//next input index
	uint16_t usCIdx;	//next output index
} CAN_RCA_S;

extern CAN_RCA_S g_stCanRCA;

extern can_receive_message_struct receive_message;  //Receive data structure
extern can_trasnmit_message_struct transmit_message;//Sending data structures

static void CAN0_GPIO_Config(void);
static void CAN0_NVIC_Config(void);
static void CAN0_Mode_Config(uint32_t uiBaud);
extern void CAN0_Init(uint32_t uiBaud);
extern bool CAN0_SendMsg(uint32_t uiId, uint8_t *pucData, uint8_t ucLength);
#endif
