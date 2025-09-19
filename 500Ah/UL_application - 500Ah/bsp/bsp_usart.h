#ifndef __BSP_USART_H
#define	__BSP_USART_H

#include "gd32f10x.h"

#define USART_MLEN	1200		/* USART buffer max byte size */

typedef struct {
	uint16_t usRLen;
	uint8_t aucRBuf[USART_MLEN];
} USART_S;

extern USART_S g_stUsart;

void UART_Init(uint32_t uiBaud, uint32_t uiWordLen, uint32_t uiStopBit, uint32_t uiParity);
void UART_Init_Dma_Receive(void);
void UART_SendBuf(char ch);
void UART_SendString(char* fmt);
extern void UART_Send(uint8_t* pucData, uint16_t usLen);

#endif /* __BSP_USART_H */
