#ifndef LVLON_H
#define LVLON_H

#include <stdbool.h>
#include <stdint.h>

#include "bsp_can.h"

#define LVCAN_DEBUG_EN	1		/* 0:非DEBUG状态, 1:DEBUG状态 */
#define LVCAN_DEBUG(fmt,arg...)	do{if(LVCAN_DEBUG_EN){printf(fmt,##arg);printf("[%s][%d]\r\n",__func__,__LINE__);}}while(0)
#define LVCAN_RETURN_FALSE	do{LVCAN_DEBUG("Return failed");return false;}while(0)
//#define LVCAN_RETURN_TRUE do{LVCAN_DEBUG("Return success");return true;}while(0)
#define LVCAN_RETURN_TRUE do{return true;}while(0)

#define LVCAN_HB_TICK	0			/* 0: 不考虑逆变器通信中断的问题 */
#define LVCAN_BLK_SIZE 0x08
#define LVCAN_PRL_TIMEOUT	30

typedef enum {
	eLVCANHeart 	= 0x18FFE303,						
	eLVCANBMSINFO = 0x18FFE0F0,			
	eLVCANSOC 		= 0x18FFE0F1,
	eLVCANBMSSTAT = 0x18FFE0F2,
	eLVCANT 			= 0x18FFE0F3,
	eLVCANCellTM 	= 0x18FFE0F4,
	eLVCANCellT 	= 0x18FFE0F5,
	eLVCANVBMS 		= 0x18C8E000,
	eLVCANCellV1 	= 0x18C8E001,
	eLVCANCellV2 	= 0x18C8E002,
	eLVCANCellV3 	= 0x18C8E003,
	eLVCANCellV4 	= 0x18C8E004,
	eLVCANCellV5 	= 0x18C8E005,
	eLVCANCellV6 	= 0x18C8E006,
	eLVCANErrInfo = 0x18FFF004	
} LVCAN_FUNC_E;


extern bool LVCAN_recv(CAN_RBUF_S stCanRBuf);
extern void LVCAN_send(uint32_t uiId, uint8_t *pucData, uint8_t ucLength);

extern void LVCAN_prol(void);
#endif
