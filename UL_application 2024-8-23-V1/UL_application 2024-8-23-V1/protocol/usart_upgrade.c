#include "usart_upgrade.h"
//#include "hybcan.h"
//#include "bsp_fdcan.h"

#include "history.h"
#include "bsp_gd25q32.h"
//#include "modbus_s.h"
//#include "dcc_mbs.h"
//#include "bsp_clkconfig.h"

//#include "YModem.h"




uint32_t g_uiUartUpDataLen = 0;         //�����̼��ܳ���
uint32_t g_uiUartUGDataCount = 0;       //�ܰ���

uint8_t  g_aucUartOTAPackData[UPGRADE_PacketMaxLen];              //��ÿһ��2048
uint16_t g_usUartUGDataCrc = 0xFFFF;    //����������CRCУ��
uint16_t g_usUartUGPackSN = 0;       //1024��ID
uint8_t  g_ucUartUGPack128SN = 0;      //128������֡���
uint32_t g_uiUartUGWriteAddr;    //��ַ





/********************************************************************************************************
**function        : mb_crc16
**description : make modbus protocol crc code
**input para1    : const uint8_t* pucBuf, modbus buffer before crc
**input para2    : const uint16_t usLen, length of input para1
**input para2 : const uint16_t usDefCRC, default CRC code at the beginning,
                                ���״μ����Ǹ�ֵΪ0xFFFF, �����зֶμ���ʱ, ��ֵΪ��һ�ε�CRC������
**output para :
**return            : crc code, or 0 if an error occurred
********************************************************************************************************/
uint16_t up_crc16(const uint8_t* pucBuf, const uint16_t usLen, const uint16_t usDefCRC) {
    if(0 == pucBuf) {
        UPGRADE_RETURN_FALSE;
    }
    if(0 == usLen) {
        UPGRADE_RETURN_FALSE;
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

void USARTOTA_Send(uint32_t uiId, uint8_t *pucData){
		//FDCAN2_SendMsg(uiId, pucData, FDCAN_DLC_BYTES_8); 
}








/*******************************************************************************
* Function Name  : OTARequest_recv
* Description    : 0x00   ���յ�OTA����      
* Input          : uiId         CanID
                   pucData      ��������    �̼����ͣ�1��+�̼��汾��3��
* Output         : None
* Return         : None
*******************************************************************************/
bool USARTOTARequest_recv(const uint8_t* pucRBuf, const uint16_t usRLen, uint8_t* pucSBuf, uint16_t* pusSLen){
//    uint8_t ucSVerVMajor;			// ������汾�� 
//	uint8_t ucSVerVMinor;			// ����ΰ汾�� 
//	uint8_t ucSVerVAmend;			// ����޶��汾�� 
    uint16_t usRecCRC = 0xFFFF;
    uint16_t usSendCRC = 0xFFFF;
    
    if(0 == pucRBuf) {
        return 0;
        //UPGRADE_RETURN_FALSE;
    }
//    ucSVerVMajor =  pucRBuf[2];
//    ucSVerVMinor =  pucRBuf[3];			
//    ucSVerVAmend =  pucRBuf[4];
    g_uiUartUpDataLen = pucRBuf[8] | (pucRBuf[7] << 8) | (pucRBuf[6] << 16) | (pucRBuf[5] << 24);       //�̼���С
    g_uiUartUGDataCount = pucRBuf[11] + (pucRBuf[10] << 8) + (pucRBuf[9] << 16) ;                         //�ܰ���
    g_uiUartUGWriteAddr = (eHisOTAPStart + HIS_OTA_PLEN / 2) * PAGE_SIZE;
    usRecCRC = pucRBuf[13] + (pucRBuf[12] << 8);
    usSendCRC = up_crc16(pucRBuf, 12, usSendCRC);
    
    pucSBuf[0] = pucRBuf[0];
    pucSBuf[1] = OTARequest;
    
    
    if(usRecCRC != usSendCRC){
        pucSBuf[2] = 0;
        //UPGRADE_RETURN_FALSE;
    }else{
        pucSBuf[2] = 1;
    }

    usSendCRC = up_crc16(pucSBuf, 3, usSendCRC);
    pucSBuf[3] = (usSendCRC >> 8) & 0xFF;
    pucSBuf[4] = usSendCRC  & 0xFF;
    UPGRADE_RETURN_TRUE;
}



bool USARTDataSubInfo_recv(const uint8_t* pucRBuf, const uint16_t usRLen, uint8_t* pucSBuf, uint16_t* pusSLen){
//    uint16_t usUGPackId = 0;          //�����������
//    uint16_t g_usUGPackLen = 0;         //������������
//    uint16_t usRecCRC = 0xFFFF;
//    uint16_t usSendCRC = 0xFFFF;
//    usUGPackId = (pucRBuf[2]<<8) + pucRBuf[3];
//    g_usUGPackLen = (pucRBuf[4] << 8) + pucRBuf[5];
//    
//    if(usUGPackId != (g_usUartUGPackSN + g_ucUartUGPack128SN)){
//        return;
//    } 
//    usRecCRC = pucRBuf[13] + (pucRBuf[12] << 8);
//    
//    if(g_usUGPackLen == 1024){
//        memcpy(g_aucUartOTAPackData, &pucRBuf[7], 1024);
//        usSendCRC = up_crc16(pucRBuf, 1024 + 6, usSendCRC);
//        g_usUartUGDataCrc =  up_crc16(pucRBuf, 1024 + 6, g_usUartUGDataCrc);
//    }else if(g_usUGPackLen == 128){
//        memcpy(g_aucUartOTAPackData, &pucRBuf[7], 128);
//        usSendCRC = up_crc16(pucRBuf, 128 + 6, usSendCRC);
//        g_usUartUGDataCrc =  up_crc16(pucRBuf, 128 + 6, g_usUartUGDataCrc);
//    }
//    

//    if(usRecCRC != usSendCRC){
//        pucSBuf[2] = 0;
//        //UPGRADE_RETURN_FALSE;
//    }else{
//        pucSBuf[2] = 1;
//        if(g_usUGPackLen == 1024){
//            MEM_FlashWrite(g_uiUartUGWriteAddr + g_usUartUGPackSN * UPGRADE_PacketMaxLen,g_aucUartOTAPackData,g_usUGPackLen);
//            g_usUartUGPackSN++;
//        }else if(g_usUGPackLen == 128){
//            MEM_FlashWrite(g_uiUartUGWriteAddr + g_usUartUGPackSN * UPGRADE_PacketMaxLen + g_ucUartUGPack128SN*128,g_aucUartOTAPackData,g_usUGPackLen);
//            g_ucUartUGPack128SN++;
//        }
//    }
//    usSendCRC = up_crc16(pucSBuf, 3, usSendCRC);
//    pucSBuf[3] = (usSendCRC >> 8) & 0xFF;
//    pucSBuf[4] = usSendCRC  & 0xFF;

    uint16_t usUGPackId = 0;          //�����������
    uint16_t g_usUGPackLen = 0;         //������������
    uint16_t usRecCRC = 0xFFFF;
    uint16_t usSendCRC = 0xFFFF;
    usUGPackId = (pucRBuf[2]<<8) + pucRBuf[3];
    g_usUGPackLen = (pucRBuf[4] << 8) + pucRBuf[5];
    
    if(usUGPackId != (g_usUartUGPackSN + g_ucUartUGPack128SN)){
        return 0;
    } 
    usRecCRC = pucRBuf[13] + (pucRBuf[12] << 8);
    

    memcpy(g_aucUartOTAPackData, &pucRBuf[7], 1024);
    usSendCRC = up_crc16(pucRBuf, 1024 + 6, usSendCRC);
    g_usUartUGDataCrc =  up_crc16(pucRBuf, 1024 + 6, g_usUartUGDataCrc);

    if(usRecCRC != usSendCRC){
        pucSBuf[2] = 0;
        //UPGRADE_RETURN_FALSE;
    }else{
        pucSBuf[2] = 1;
        MEM_FlashWrite(g_uiUartUGWriteAddr + g_usUartUGPackSN * UPGRADE_PacketMaxLen,g_aucUartOTAPackData,g_usUGPackLen);
        g_usUartUGPackSN++;

    }
    usSendCRC = up_crc16(pucSBuf, 3, usSendCRC);
    pucSBuf[3] = (usSendCRC >> 8) & 0xFF;
    pucSBuf[4] = usSendCRC  & 0xFF;

UPGRADE_RETURN_TRUE;

}



///*******************************************************************************
//* Function Name  : DataEnd_recv
//* Description    : 0x03   ���յ����ݽ�����Ϣ
//* Input          : uiId         CanID
//                   pucData      ��������        ������CRC16
//* Output         : None
//* Return         : None
//*******************************************************************************/
bool USARTDataEnd_recv(const uint8_t* pucRBuf, const uint16_t usRLen, uint8_t* pucSBuf, uint16_t* pusSLen){
    uint16_t usRecCRC = 0xFFFF;
    uint16_t usSendCRC = 0xFFFF;
    usRecCRC = (pucRBuf[2]<<8) + pucRBuf[3];
    
    if(usRecCRC != g_usUartUGDataCrc){
        pucSBuf[2] = 0;
        //UPGRADE_RETURN_FALSE;
    }else{
        pucSBuf[2] = 1;
    }
    usSendCRC = up_crc16(pucSBuf, 3, usSendCRC);
    pucSBuf[3] = (usSendCRC >> 8) & 0xFF;
    pucSBuf[4] = usSendCRC  & 0xFF;
    UPGRADE_RETURN_TRUE;
}

bool USARTOTA_recv_proc(const uint8_t* pucRBuf, const uint16_t usRLen, uint8_t* pucSBuf, uint16_t* pusSLen){
    uint8_t OTACode;
    OTACode = pucRBuf[1];
    switch(OTACode){
        case OTARequest:
             return USARTOTARequest_recv(pucRBuf, usRLen, pucSBuf, pusSLen);
        case OTAInfo:
             return USARTDataSubInfo_recv(pucRBuf, usRLen, pucSBuf, pusSLen);
        case DataEnd:
             return USARTDataEnd_recv(pucRBuf, usRLen, pucSBuf, pusSLen);
    }
    UPGRADE_RETURN_TRUE;
}

