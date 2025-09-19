#ifndef __BSP_USART_H
#define	__BSP_USART_H

#include "gd32f10x.h"

#ifdef USER_DEBUG
#define BSP_DEBUG_EN	1		/* 0:Non DEBUG status, 1:DEBUG status */
#else
#define BSP_DEBUG_EN	1
#endif
#define BSP_DEBUG(fmt,arg...)	do{if(BSP_DEBUG_EN){printf(fmt,##arg);printf("[%s][%d]\r\n",__func__,__LINE__);}}while(0)
#define BSP_RETURN_FALSE	do{BSP_DEBUG("Return failed");return false;}while(0)
#define BSP_RETURN_TRUE do{return true;}while(0)

typedef enum {
	eBLEPort = 0,
	eLEDPort,
	e485Port,
	ePortNum
} ePortId;


#define USART_MLEN	256		/* USART buffer max byte size */

typedef struct {
	uint32_t hUart[ePortNum];
	uint16_t usRLen[ePortNum];
	uint8_t aaucRBuf[ePortNum][USART_MLEN];
	uint16_t usSLen[ePortNum];
	uint8_t aaucSBuf[ePortNum][USART_MLEN];
} USART_S;

extern USART_S g_stUsart;

void USART_Init(ePortId port, uint32_t uiBaud, uint32_t uiWordLen, uint32_t uiStopBit, uint32_t uiParity);
void UART_SendBuf(ePortId ePort,char ch);
void UART_SendString(ePortId ePort, char* fmt);

#endif /* __BSP_USART_H */
