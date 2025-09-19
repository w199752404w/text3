#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "bsp_can.h"
#include "main.h"

extern uint8_t USART2_RxFlag;
extern uint8_t USART2_UgFlag;
extern uint8_t TimeFlag;
extern uint8_t Timeout;
extern uint8_t CAN_READ;
#ifdef USER_DEBUG
#define PTC_DEBUG_EN	1		/* 0:NON-DEBUG STATUS, 1:DEBUG status */
#else
#define PTC_DEBUG_EN 0
#endif
#define PTC_DEBUG(fmt,arg...)	do{if(PTC_DEBUG_EN){printf(fmt,##arg);printf("[%s][%d]\r\n",__func__,__LINE__);}}while(0)
#define PTC_RETURN_FALSE	do{PTC_DEBUG("Return failed");return false;}while(0)
//#define PTC_RETURN_TRUE do{PTC_DEBUG("Return success");return true;}while(0)
#define PTC_RETURN_TRUE do{return true;}while(0)

#endif
