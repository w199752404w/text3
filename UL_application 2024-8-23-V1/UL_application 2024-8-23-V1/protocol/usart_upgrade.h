#ifndef USART_UPGRADE_H
#define USART_UPGRADE_H

#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
//#include "hybcan.h"

#define UPGRADE_DEBUG_EN	0		/* 0:��DEBUG״̬, 1:DEBUG״̬ */
#define UPGRADE_DEBUG(fmt,arg...)	do{if(UPGRADE_DEBUG_EN){printf(fmt,##arg);printf("[%s][%d]\r\n",__func__,__LINE__);}}while(0)
#define UPGRADE_RETURN_FALSE	do{UPGRADE_DEBUG("Return failed");return false;}while(0)
//#define PYCAN_RETURN_TRUE do{PYCAN_DEBUG("Return success");return true;}while(0)
#define UPGRADE_RETURN_TRUE do{return true;}while(0)


#define UPGRADE_HB_TICK 0			/* 0: �����������ͨ���жϵ����� */
#define UPGRADE_BLK_SIZE 0x08
#define UPGRADE_PRL_TIMEOUT 30
#define UPGRADE_PacketMaxLen 1024   //֧�ֵ����ݰ���󳤶�

extern bool g_bOTAflag;

typedef enum {
    OTARequest = 0x50,  //OTA����+ OTA��Ϣ     �̼����ͣ�1��+�̼��汾��3��
    OTAInfo,            //OTA��Ϣ     �̼���С��4��+�ܰ�����3��
    DataEnd,            //���ݽ���
}OTACode_E;



//void pycan_on_FWSize1(uint32_t uiId, uint8_t* pucData, bool bPrl);

//void pycan_on_blkSN1(uint32_t uiId, uint8_t* pucData, bool bPrl);
//void pycan_on_blkData1(uint32_t uiId, uint8_t* pucData, bool bPrl);
//void pycan_on_blkCRC1(uint32_t uiId, uint8_t* pucData, bool bPrl);
//void pycan_on_FWCRC1(uint32_t uiId, uint8_t* pucData, bool bPrl);
//void pycan_on_FWUpgrade1(uint32_t uiId, bool bPrl);
//void pycan_on_upgradeStat1(uint32_t uiId, bool bPrl);
//void pycan_on_replyFWSize1(uint32_t uiId, uint8_t* pucData);
//void pycan_on_replyBlk1(uint32_t uiId, uint8_t* pucData);
//void pycan_on_replyFWCRC1(uint32_t uiId, uint8_t* pucData);
//void pycan_on_replyUpgrade1(uint32_t uiId, uint8_t* pucData);

//void OTA_Send(uint32_t uiId, uint8_t *pucData);
//extern void OTA_Ctrl_recv_proc(CAN_HEADER_U uiId,uint8_t* pucData);
//extern void OTA_Data_recv_proc(CAN_HEADER_U uiId,uint8_t* pucData);
extern bool USARTOTA_recv_proc(const uint8_t* pucRBuf, const uint16_t usRLen, uint8_t* pucSBuf, uint16_t* pusSLen);
#endif
