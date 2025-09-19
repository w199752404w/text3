#ifndef CAN_UPGRADE_H
#define CAN_UPGRADE_H

#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
//#include "hybcan.h"

#define UPGRADE_DEBUG_EN	0		/* 0:非DEBUG状态, 1:DEBUG状态 */
#define UPGRADE_DEBUG(fmt,arg...)	do{if(UPGRADE_DEBUG_EN){printf(fmt,##arg);printf("[%s][%d]\r\n",__func__,__LINE__);}}while(0)
#define UPGRADE_RETURN_FALSE	do{UPGRADE_DEBUG("Return failed");return false;}while(0)
//#define PYCAN_RETURN_TRUE do{PYCAN_DEBUG("Return success");return true;}while(0)
#define UPGRADE_RETURN_TRUE do{return true;}while(0)


#define UPGRADE_HB_TICK 0			/* 0: 不考虑逆变器通信中断的问题 */
#define UPGRADE_BLK_SIZE 0x08
#define UPGRADE_PRL_TIMEOUT 30
#define UPGRADE_PacketMaxLen 2048   //支持的数据包最大长度

extern bool g_bOTAflag;

//RTN 响应状态字
typedef enum {
	eNormal = 0x00,         //正常
	eCmdInvalid,            //CMDCode无效
	eCrcError,              //CRC校验错误
	eUGforwarding,          //升级转发中
	eSizeAbnormal,          //固件总大小异常
	eUGSucess,              //升级成功
	eUGFail,                //升级失败
	eCascadeFail,           //级联失败
	eOperateError           //操作或写入错误
} RTN_E;

typedef struct{
    uint32_t bFrameNum:8;       //帧序号
    uint32_t bFrameFlag:1;      //帧标志
    uint32_t bFunCode:4;        //功能码
    uint32_t bTagAddr:5;        //目标地址
    uint32_t bSourAddr:5;       //源地址
    uint32_t bDirection:1;      //方向
} CAN_HEADER_S;

typedef union{
    uint32_t uiCanId;
    CAN_HEADER_S stCanHeader;
} CAN_HEADER_U;









typedef enum {
    OTARequest = 0x00,  //OTA请求     固件类型（1）+固件版本（3）
    OTAInfo,            //OTA信息     固件大小（4）+总包数（3）
    DataSubInfo,        //数据分包信息
    DataEnd,            //数据结束
    FirmUpProc,         //固件升级处理
    GetUpStatus         //获取升级状态
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
