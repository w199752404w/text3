#include "modbus_rtu.h"
#include "config.h"
#include "bsp_sh367303.h"
#include "bsp_usart.h"
#include "local.h"
#include "usart_upgrade.h"
#include <stdlib.h>
#include <stdio.h>
#include "string.h"



Mbs_S g_stMbs = {0};



/********************************************************************************************************
**function        : mb_crc16
**description : make modbus protocol crc code
**input para1    : const uint8_t* pucBuf, modbus buffer before crc
**input para2    : const uint16_t usLen, length of input para1
**input para2 : const uint16_t usDefCRC, default CRC code at the beginning,
                                在首次计算是该值为0xFFFF, 当进行分段计算时, 该值为上一段的CRC计算结果
**output para :
**return            : crc code, or 0 if an error occurred
********************************************************************************************************/
uint16_t mbs_crc16(const uint8_t* pucBuf, const uint16_t usLen, const uint16_t usDefCRC) {
    if(0 == pucBuf) {
        MBS_RETURN_FALSE;
    }
    if(0 == usLen) {
        MBS_RETURN_FALSE;
    }
    uint16_t crc = usDefCRC;
    for(uint16_t i=0;i<usLen;i++) {
        crc ^= pucBuf[i];
        for(uint16_t j=8;j!=0;j--) {
            if((crc & 0x01) != 0) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    
    return crc;
}


/********************************************************************************************************
**function        : mbs_check
**description : check if input para1 is modbus buffer or not, for usart protocol self-adaption
**input para1    : const uint8_t* pucBuf, the buffer to be checked
**input para2    : const uint16_t usLen, length of input para1
**output para :
**return            : return 0 when failed, otherwise return 1
********************************************************************************************************/
//bool mbs_check(const uint8_t* pucBuf, const uint16_t usLen) {
//    if(0 == pucBuf) {
//        MBS_RETURN_FALSE;
//    }
////    if(usLen < 8) {
////        MBS_RETURN_FALSE;
////    }
//    uint16_t crc = mbs_crc16(pucBuf, usLen - 2, 0xFFFF);
//    if(pucBuf[usLen - 1] == (crc >> 8) && pucBuf[usLen - 2] == (crc & 0xff)) {
//        MBS_RETURN_TRUE;
//    }
//    
//    MBS_RETURN_FALSE;
//}

/*******************************************************************************
* Function Name  : mbs_init
* Description    : modbus初始化
* Input          : None
* Output         : None
* Return         : result, 1写成功 0写失败
*******************************************************************************/
bool mbs_init(void) {

    MBS_RETURN_TRUE;
}



/********************************************************************************************************
**function        : mbs_rir_proc
**input para1    : const uint8_t* pucRBuf, buffer to be process   读取输入寄存器
**input para2    : const uint16_t usRLen, length of input para1
**output para1: uint8_t* pucSBuf, buffer to be send back to usart, it should be malloced before this function
**output para2: uint16_t* pusSLen, length of output para1 malloced input and used output
**return            : return 0 when failed, otherwise return 1
********************************************************************************************************/
bool mbs_rir_proc(const uint8_t* pucRBuf, const uint16_t usRLen, uint8_t* pucSBuf, uint16_t* pusSLen) {    
    uint16_t Value;
    uint16_t usCrc = 0xFFFF;
    uint16_t usCrcR = 0;
	
    if(0 == pucRBuf) {
        MBS_RETURN_FALSE;
    }
    if(usRLen < 7) {
        MBS_RETURN_FALSE;
    }
    
//    usCrcR = pucRBuf[6] | pucRBuf[7] << 8;
//    usCrc = mbs_crc16(pucRBuf,usRLen,usCrc);
//		
//    if(usCrcR != usCrc) {
//        MBS_RETURN_FALSE;
//    }
    
    uint16_t usStartAddr = pucRBuf[2]* 0x100 + pucRBuf[3];
    uint16_t usRegNum = pucRBuf[4]* 0x100 + pucRBuf[5];
//    if(usRLen <= usRegNum * 2 + 5) {
//		MBS_RETURN_FALSE;
//	}
    pucSBuf[0] = pucRBuf[0];        //设备地址
	pucSBuf[1] = pucRBuf[1];        //功能码
	pucSBuf[2] = usRegNum * 2;      //数据内容长度，两字节一个寄存器数据

    //寄存器数据（local实时数据）
    for(uint8_t i = 0;i < usRegNum;i++){
        local_read_reg(0,0,usStartAddr+i,&Value);
        pucSBuf[3+i*2] =   (Value >> 8) & 0xFF;
        pucSBuf[3+i*2+1] = Value & 0xFF;
    }
    *pusSLen = usRegNum * 2 + 3;
    uint16_t crc = mbs_crc16(pucSBuf, *pusSLen, 0xFFFF);
    pucSBuf[*pusSLen] = crc & 0xff;
	pucSBuf[*pusSLen + 1] = crc >> 8;
    *pusSLen += 2;

    MBS_RETURN_TRUE;
}



/********************************************************************************************************
**function        : mbs_ao_proc
**description : process ao buffer received from usart       写保存寄存器命令
**input para1    : const uint8_t* pucRBuf, buffer to be process
**input para2    : const uint16_t usRLen, length of input para1
**output para1: uint8_t* pucSBuf, buffer to be send back to usart, it should be malloced before this function
**output para2: uint16_t* pusSLen, length of output para1 malloced input and used output
**return            : return 0 when failed, otherwise return 1
********************************************************************************************************/
bool mbs_ao_proc(const uint8_t* pucRBuf, const uint16_t usRLen, uint8_t* pucSBuf, uint16_t* pusSLen) {
    if(0 == pucRBuf) {
        MBS_RETURN_FALSE;
    }
    if(usRLen < 8) {
        MBS_RETURN_FALSE;
    }

    unsigned short usStartAddr = pucRBuf[2] * 0x100 + pucRBuf[3];
    

    pucSBuf[0] = pucRBuf[0];
	pucSBuf[1] = pucRBuf[1];
    uint16_t pfVal;
    pfVal = pucRBuf[4] << 8 | pucRBuf[5];
        
    if(!local_Write_reg(0,1,usStartAddr,&pfVal)){
        MBS_RETURN_FALSE;
    }

    pucSBuf[2] = pucRBuf[2];
    pucSBuf[3] = pucRBuf[3];
    pucSBuf[4] = pucRBuf[4];
    pucSBuf[5] = pucRBuf[5];
    uint16_t crc = mbs_crc16(pucSBuf, 6, 0xFFFF);
    pucSBuf[6] = crc & 0xff;
    pucSBuf[7] = crc >> 8;
    *pusSLen = 8;

        
    MBS_RETURN_TRUE;
}


/********************************************************************************************************
**function        : mbs_recv
**description : process buffer received from usart
**input para1    : const uint8_t* pucRBuf, buffer to be process
**input para2    : const uint16_t usRLen, length of input para1
**output para1: uint8_t* pucSBuf, buffer to be send back to usart, it should be malloced before this function
**output para2: uint16_t* pusSLen, length of output para1 malloced input and used output
**return            : return 0 when failed, otherwise return 1
********************************************************************************************************/
bool mbs_recv(const uint8_t* pucRBuf, const uint16_t usRLen, uint8_t* pucSBuf, uint16_t* pusSLen) {
    uint8_t Addr = pucRBuf[0];
    
    if(Addr != CCS_ID){
        MBS_RETURN_TRUE; 
    }
    
    switch(pucRBuf[1]){
    case embsFuncRir:
        return mbs_rir_proc(pucRBuf, usRLen, pucSBuf, pusSLen);
    case embsFuncAo:
        return mbs_ao_proc(pucRBuf, usRLen, pucSBuf, pusSLen);
    case embsFuncOTARequest:
        return USARTOTA_recv_proc(pucRBuf, usRLen, pucSBuf, pusSLen);
    case embsFuncOTAInfo:
        return USARTOTA_recv_proc(pucRBuf, usRLen, pucSBuf, pusSLen);
    case embsFuncDataEnd:
        return USARTOTA_recv_proc(pucRBuf, usRLen, pucSBuf, pusSLen);
    default:
        MBS_RETURN_FALSE;
    }
}


bool mbs_rir_SendMsg(Mbs_S mbsData)
{
    uint16_t usCrc = 0xFFFF;
    uint8_t pucSBuf[8];
    
    pucSBuf[0] = CCS_ID;
    pucSBuf[1] = embsFuncRir;
    pucSBuf[2] = (mbsData.usStartAddr >> 8) & 0xFF;
    pucSBuf[3] = mbsData.usStartAddr & 0xFF;
    pucSBuf[4] = (mbsData.usRegNum >> 8) & 0xFF;
    pucSBuf[5] = mbsData.usRegNum & 0xFF;
    
    usCrc = mbs_crc16(pucSBuf,6,usCrc);
    pucSBuf[6] = usCrc & 0xFF;
    pucSBuf[7] = (usCrc >> 8) & 0xFF;
    
    mbs_send(pucSBuf,8);
    MBS_RETURN_TRUE;
}


bool mbs_ao_SendMsg(Mbs_S mbsData)
{
    uint16_t usCrc = 0xFFFF;
    uint8_t pucSBuf[mbsData.usRegNum*2 + 9];
    
    pucSBuf[0] = CCS_ID;
    pucSBuf[1] = embsFuncAo;
    pucSBuf[2] = (mbsData.usStartAddr >> 8) & 0xFF;
    pucSBuf[3] = mbsData.usStartAddr & 0xFF;
    pucSBuf[4] = (mbsData.usRegNum >> 8) & 0xFF;
    pucSBuf[5] = mbsData.usRegNum & 0xFF;
    pucSBuf[6] = mbsData.usRegNum *2;
    
    for(uint8_t i = 0; i < mbsData.usRegNum; i++)
    {
        pucSBuf[2*i+7] = (mbsData.usRegData[i] >> 8) & 0xFF;
        pucSBuf[2*i+8] = mbsData.usRegData[i] & 0xFF;
    }
    usCrc = mbs_crc16(pucSBuf,mbsData.usRegNum*2 + 7,usCrc);
    pucSBuf[mbsData.usRegNum*2 + 7] = usCrc & 0xFF;
    pucSBuf[mbsData.usRegNum*2 + 8] = (usCrc >> 8) & 0xFF;
    
    mbs_send(pucSBuf,mbsData.usRegNum*2 + 9);
		//UARTISP_SendBuf(pucSBuf,mbsData.usRegNum*2 + 9);
    MBS_RETURN_TRUE;
}

/*
 * mbs_Srevice_SendMsg
 * 描述  ：modbus通信报文内容设置,设置一个数据包
 * 输入  ：
                     Funcode, 功能码
                     startRegAddr, 起始地址
                     RegNum, 寄存器数量
 * 输出  : 无
 * 调用  ：外部调用
 */
bool mbs_SendMsg(uint8_t ucFunC,uint16_t usAddr,uint8_t* pucData, uint16_t usLen)
{
    
    g_stMbs.ucFunC = ucFunC;
    g_stMbs.usStartAddr = usAddr;
    g_stMbs.usRegNum = usLen;
    memcpy(&g_stMbs.usRegData, pucData, usLen);
    
    switch(g_stMbs.ucFunC)
    {
    case embsFuncRir:
        return mbs_rir_SendMsg(g_stMbs);
    case embsFuncAo:
        return mbs_ao_SendMsg(g_stMbs);
    }
    MBS_RETURN_TRUE;
}


void mbs_Rir(void)
{
    uint16_t usCrc = 0xFFFF;
    uint16_t usStartAddr = 0x1001;
    uint16_t usRegNum = 1;
    uint8_t pucSBuf[8];
    
    pucSBuf[0] = CCS_ID;
    pucSBuf[1] = embsFuncRir;
    pucSBuf[2] = (usStartAddr >> 8) & 0xFF;
    pucSBuf[3] = usStartAddr & 0xFF;
    pucSBuf[4] = (usRegNum >> 8) & 0xFF;
    pucSBuf[5] = usRegNum & 0xFF;
    
    usCrc = mbs_crc16(pucSBuf,6,usCrc);
    pucSBuf[6] = usCrc & 0xFF;
    pucSBuf[7] = (usCrc >> 8) & 0xFF;
    
    mbs_send(pucSBuf,8);

}


//void Ymodem_send(uint8 sendtype,uint8_t* pucBuf, uint16_t usLen){
//	switch(sendtype){
//		case 0:
//			UART_SendByte(eISPPort,pucBuf[0]);
//			break;
//		case 1:
//			UART_SendBuf(eISPPort,pucBuf,usLen);
//			break;
//		default:
//			break;
//	}
//}





void dcc_proc(void) {
		/* 每500毫秒调用一次本函数以下部分 */
    static uint32_t uiTick = 0;
    uiTick++;
    if(uiTick < 10) {
        return;
    }
    uiTick = 0;

//    uint8_t pucSBuf[8];
//    
//    pucSBuf[0] = 1;
//    pucSBuf[1] = 2;
//    pucSBuf[2] = 0;
//    pucSBuf[3] = 0;
//    pucSBuf[4] = 0;
//    pucSBuf[5] = 0;
//    

//    pucSBuf[6] = 0;
//    pucSBuf[7] = 7;

    //mbs_send(pucSBuf,8);
    
}

void mbs_send(uint8_t* pucData, uint16_t usLen)
{
    UART_SendBuf(e485Port1,pucData,usLen);
}

