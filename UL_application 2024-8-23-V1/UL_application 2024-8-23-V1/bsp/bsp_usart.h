#ifndef __BSP_USART_H
#define	__BSP_USART_H

#include "gd32f10x.h"

#define USART_MLEN	1200		/* USART buffer max byte size */

typedef enum {
	e485Port1 = 0,
	ebluetooth,
	ePortNum
} ePortId;


typedef struct {
	uint32_t hUart[ePortNum];
	uint16_t usRLen[ePortNum];
	uint8_t  aucRBuf[ePortNum][USART_MLEN];
    uint16_t usSLen[ePortNum];
	uint8_t  aaucSBuf[ePortNum][USART_MLEN];
} USART_S;

extern USART_S g_stUsart;

void UART_Init(ePortId port,uint32_t uiBaud, uint32_t uiWordLen, uint32_t uiStopBit, uint32_t uiParity);
void UART_Init_Dma_Receive(void);

void UART_SendByte(ePortId port,uint8_t ch);
void UART_SendString(ePortId port,uint8_t* fmt);
void UART_SendBuf(ePortId port,uint8_t* pucBuf, uint16_t usLen);
void UART485_SendBuf(uint8_t* pucBuf, uint16_t usLen);
extern void UART_Send(uint8_t* pucData, uint16_t usLen);

#endif /* __BSP_USART_H */
