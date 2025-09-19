#ifndef MODBUS_RTU_H
#define MODBUS_RTU_H

#include <stdbool.h>
#include <stdint.h>

#define MBS_DEBUG_EN	0		/* 0:非DEBUG状态, 1:DEBUG状态 */
#define MBS_DEBUG(fmt,arg...)	do{if(MBS_DEBUG_EN){printf(fmt,##arg);printf("[%s][%d]\r\n",__func__,__LINE__);}}while(0)
#define MBS_RETURN_FALSE	do{MBS_DEBUG("Return failed");return false;}while(0)
//#define MBS_RETURN_TRUE do{MBS_DEBUG("Return success");return true;}while(0)
#define MBS_RETURN_TRUE do{return true;}while(0)



#define CCS_ID 0x01  //CCS地址
#define R_StartAddr 0x1000     //读地址
#define W_StartAddr 0x2000     //写地址

typedef enum {
	embsFuncRir = 0x03,      //读输出寄存器
	embsFuncAo  = 0x06,      //写单个输出寄存器
    //embsFuncUpgrade = 0x50   //升级
    embsFuncOTARequest = 0x50,  //OTA请求+ OTA信息     固件类型（1）+固件版本（3）
    embsFuncOTAInfo = 0x51,            //OTA信息     固件大小（4）+总包数（3）
    embsFuncDataEnd  = 0x52            //数据结束
    
} MBS_FUNC_E;

typedef enum  {
	embsErrNull = 0,
	embsErrFunc,
	embsErrRegAddr,
	embsErrVal,
	embsErrSlvAddr,
	embsErrConfirm,
	embsErrBusy,
	embsErrOE,
	embsErrRoute,
	embsErrRespond
} MBS_ERR_CODE_E;

typedef struct{
    uint8_t  ucFunC;             //功能码
    uint16_t usStartAddr;        //起始地址
    uint16_t usRegNum;           //寄存器数量
    uint16_t usRegData[100];      //寄存器数据
}Mbs_S;


extern  Mbs_S g_stMbs;

extern uint16_t mbs_crc16(const uint8_t* pucBuf, const uint16_t usLen, const uint16_t usDefCRC);
//static bool mbs_check(const uint8_t* pucBuf, const uint16_t usLen);

extern bool mbs_init(void);
extern bool mbs_recv(const uint8_t* pucRBuf, const uint16_t usRLen, uint8_t* pucSBuf, uint16_t* pusSLen);
//extern bool mbs_rir_proc(const uint8_t* pucRBuf, const uint16_t usRLen, uint8_t* pucSBuf, uint16_t* pusSLen);
//extern bool mbs_ao_proc(const uint8_t* pucRBuf, const uint16_t usRLen, uint8_t* pucSBuf, uint16_t* pusSLen);

extern bool mbs_SendMsg(uint8_t ucFunC,uint16_t usAddr,uint8_t* pucData, uint16_t usLen);
extern bool mbs_rir_SendMsg(Mbs_S mbsData);
extern bool mbs_ao_SendMsg(Mbs_S mbsData);

extern void mbs_Rir(void);

void mbs_send(uint8_t* pucData, uint16_t usLen);
extern void dcc_proc(void);
#endif
