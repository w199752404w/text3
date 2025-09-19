#ifndef CAN_UPGRADE_H
#define CAN_UPGRADE_H

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
#define UPGRADE_PacketMaxLen 2048   //֧�ֵ����ݰ���󳤶�

extern bool g_bOTAflag;

//RTN ��Ӧ״̬��
typedef enum {
	eNormal = 0x00,         //����
	eCmdInvalid,            //CMDCode��Ч
	eCrcError,              //CRCУ�����
	eUGforwarding,          //����ת����
	eSizeAbnormal,          //�̼��ܴ�С�쳣
	eUGSucess,              //�����ɹ�
	eUGFail,                //����ʧ��
	eCascadeFail,           //����ʧ��
	eOperateError           //������д�����
} RTN_E;

typedef struct{
    uint32_t bFrameNum:8;       //֡���
    uint32_t bFrameFlag:1;      //֡��־
    uint32_t bFunCode:4;        //������
    uint32_t bTagAddr:5;        //Ŀ���ַ
    uint32_t bSourAddr:5;       //Դ��ַ
    uint32_t bDirection:1;      //����
} CAN_HEADER_S;

typedef union{
    uint32_t uiCanId;
    CAN_HEADER_S stCanHeader;
} CAN_HEADER_U;









typedef enum {
    OTARequest = 0x00,  //OTA����     �̼����ͣ�1��+�̼��汾��3��
    OTAInfo,            //OTA��Ϣ     �̼���С��4��+�ܰ�����3��
    DataSubInfo,        //���ݷְ���Ϣ
    DataEnd,            //���ݽ���
    FirmUpProc,         //�̼���������
    GetUpStatus         //��ȡ����״̬
}OTACode_E;



void pycan_on_FWSize1(uint32_t uiId, uint8_t* pucData, bool bPrl);

void pycan_on_blkSN1(uint32_t uiId, uint8_t* pucData, bool bPrl);
void pycan_on_blkData1(uint32_t uiId, uint8_t* pucData, bool bPrl);
void pycan_on_blkCRC1(uint32_t uiId, uint8_t* pucData, bool bPrl);
void pycan_on_FWCRC1(uint32_t uiId, uint8_t* pucData, bool bPrl);
void pycan_on_FWUpgrade1(uint32_t uiId, bool bPrl);
void pycan_on_upgradeStat1(uint32_t uiId, bool bPrl);
void pycan_on_replyFWSize1(uint32_t uiId, uint8_t* pucData);
void pycan_on_replyBlk1(uint32_t uiId, uint8_t* pucData);
void pycan_on_replyFWCRC1(uint32_t uiId, uint8_t* pucData);
void pycan_on_replyUpgrade1(uint32_t uiId, uint8_t* pucData);

void OTA_Send(uint32_t uiId, uint8_t *pucData);
extern void OTA_Ctrl_recv_proc(CAN_HEADER_U uiId,uint8_t* pucData);
extern void OTA_Data_recv_proc(CAN_HEADER_U uiId,uint8_t* pucData);

#endif
