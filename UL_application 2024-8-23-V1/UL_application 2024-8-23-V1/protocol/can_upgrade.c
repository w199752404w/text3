#include "can_upgrade.h"
//#include "hybcan.h"
#include "bsp_exti.h"
#include "parallel.h"
#include "history.h"
#include "bsp_gd25q32.h"
#include "config.h"
//#include "modbus_s.h"
//#include "dcc_mbs.h"
//#include "bsp_clkconfig.h"

//#include "YModem.h"


bool g_bOTAflag = false;

//uint8_t  g_ForceUGFlag = 0;
uint8_t  g_ucPackUGType = 0;	    //0-���ع̼���1-�ӿع̼���2-����
uint32_t g_uiUpDataLen = 0;         //�����̼��ܳ���
uint32_t g_uiUGDataCount = 0;       //�ܰ���
uint16_t g_usUGPackLen = 0;         //������������
uint16_t g_usUGPackId = 0;          //�����������
uint16_t g_usUGPackCrc = 0;         //��������CRCУ��
uint16_t g_usUGDataCrc = 0xFFFF;    //����������CRCУ��
uint8_t  g_stat = 0;                //״̬

uint8_t  g_aucOTAPackData[UPGRADE_PacketMaxLen];              //��ÿһ��2048

uint16_t g_usUGPackSN;       //��ID
uint32_t g_uiUGWriteAddr;    //��ַ
uint8_t  g_ucUGBlkIdx = 0;      //������֡���
uint8_t  g_aucOTAPackDataREAD[UPGRADE_PacketMaxLen];           //��ÿһ��2048


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
uint16_t canup_crc16(const uint8_t* pucBuf, const uint16_t usLen, const uint16_t usDefCRC) {
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


/*******************************************************************************
* Function Name  : OTARequest_recv
* Description    : 0x00   ���յ�OTA����      
* Input          : uiId         CanID
                   pucData      ��������    �̼����ͣ�1��+�̼��汾��3��
* Output         : None
* Return         : None
*******************************************************************************/
void OTARequest_recv(CAN_HEADER_U uiId,uint8_t* pucData){
//�ж��Ƿ񱾻���Ϣ
//    if(uiId.stCanHeader.bTagAddr != ){
//        return;
//    }
//    
    g_bOTAflag = true;
    uiId.stCanHeader.bDirection = 1;
    uint8_t aucData[8] = {0};
    uint8_t ucSVerVMajor;			/* ������汾�� */
	uint8_t ucSVerVMinor;			/* ����ΰ汾�� */
	uint8_t ucSVerVAmend;			/* ����޶��汾�� */
    uint8_t ucUpType;
    g_usUGPackId  = 0;
    g_usUGPackLen = 0;
    g_usUGPackSN = 0;
    g_ucUGBlkIdx = 0;
    g_usUGDataCrc = 0xFFFF;
    g_uiUGWriteAddr = (eHisOTAPStart + HIS_OTA_PLEN / 2) * PAGE_SIZE;
    
	uint16_t PacketMaxLen = UPGRADE_PacketMaxLen;	

    g_ucPackUGType = pucData[1];
    ucSVerVMajor =  pucData[2];
    ucSVerVMinor =  pucData[3];			
    ucSVerVAmend =  pucData[4];
    ucUpType = pucData[5];
    
    if(ucUpType == 0){
			switch(g_ucPackUGType){
				case 0:
					if(ucSVerVMajor > g_stCfg.stLocal.ucSVerVMajor  || ucSVerVMinor > g_stCfg.stLocal.ucSVerVMinor || ucSVerVAmend > g_stCfg.stLocal.ucSVerRMajor ){
							aucData[1] = eNormal;
					}else{
							aucData[1] = eOperateError;
					}
				case 1:
					aucData[1] = eNormal;
				default:
					aucData[1] = eOperateError;
			}
		}else{
				if(g_ucPackUGType < 2){
						aucData[1] = eNormal;
				}else{
					aucData[1] = eOperateError;
				}
		}
			
//        if((ucSVerVMajor > g_stCfg.stLocal.ucSVerVMajor || ucSVerVMinor > g_stCfg.stLocal.ucSVerVMinor || ucSVerVAmend > g_stCfg.stLocal.ucSVerRMinor) && g_ucPackUGType <2){
////    if(g_ucPackUGType < 2){
//            aucData[1] = eNormal;
//        }else{
//        aucData[1] = eOperateError;
//        }
//    }else{
//        if(g_ucPackUGType < 2){
//            aucData[1] = eNormal;
//        }else{
//            aucData[1] = eOperateError;
//        }
//    }
//    if((ucSVerVMajor > g_stCfg.stLocal.ucSVerVMajor || ucSVerVMinor > g_stCfg.stLocal.ucSVerVMinor || ucSVerVAmend > g_stCfg.stLocal.ucSVerRMajor) && g_ucPackUGType <2){
////    if(g_ucPackUGType < 2){
//        aucData[1] = eNormal;
//    }else{
//        aucData[1] = eOperateError;
//    }
    aucData[0] = OTARequest;
    aucData[2] = (PacketMaxLen >>8)& 0xFF;
    aucData[3] = PacketMaxLen & 0xFF;
    OTA_Send(uiId.uiCanId, aucData);
}

/*******************************************************************************
* Function Name  : OTAInfo_recv
* Description    : 0x01   ���յ�OTA��Ϣ
* Input          : uiId         CanID
                   pucData      ��������        �̼���С��4��+�ܰ�����3��
* Output         : None
* Return         : None
*******************************************************************************/
void OTAInfo_recv(CAN_HEADER_U uiId,uint8_t* pucData){
    uiId.stCanHeader.bDirection = 1;
    uint8_t aucData[8] = {0};
    g_uiUpDataLen = pucData[1] | (pucData[2] << 8) | (pucData[3] << 16) | (pucData[4] << 24);       //�̼���С
    g_uiUGDataCount = pucData[5] + (pucData[6] << 8) + (pucData[7] << 16) ;                         //�ܰ���
    aucData[0] = OTAInfo;
		//g_stOTA.ucFWLen = g_uiUpDataLen;
    if(g_uiUpDataLen <= HIS_OTA_PLEN * 256 / 2){
        aucData[1] = eNormal;
        g_stat = eUGforwarding;
    }else{
        aucData[1] = eSizeAbnormal;
        g_stat = eUGFail;
    }
    OTA_Send(uiId.uiCanId, aucData);
}

/*******************************************************************************
* Function Name  : DataSubInfo_recv
* Description    : 0x02   ���յ����ݷְ���Ϣ
* Input          : uiId         CanID
                   pucData      ��������        PacketNum+PacketLen+CRC16
* Output         : None
* Return         : None
*******************************************************************************/
void DataSubInfo_recv(CAN_HEADER_U uiId,uint8_t* pucData){
    uiId.stCanHeader.bDirection = 1;
    uint8_t aucData[8] = {0};
    
    g_usUGPackId =  pucData[1] + (pucData[2] << 8) + (pucData[3] << 16) ;       //����� 
    g_usUGPackLen = pucData[4] + (pucData[5] << 8);                             //�����ݳ���
    g_usUGPackCrc = pucData[6] + (pucData[7] << 8);                             //CRC
    
    aucData[0] = DataSubInfo;
    if(g_usUGPackSN == g_usUGPackId){
        aucData[1] = eNormal;
        g_stat = eUGforwarding;
    }else{
        aucData[1] = eOperateError;
        UPGRADE_DEBUG("�����SN: %02x, ������SN: %02x", g_usUGPackId, g_usUGPackSN);
        g_stat = eUGFail;
    }
    
    OTA_Send(uiId.uiCanId, aucData);
}

/*******************************************************************************
* Function Name  : DataEnd_recv
* Description    : 0x03   ���յ����ݽ�����Ϣ
* Input          : uiId         CanID
                   pucData      ��������        ������CRC16
* Output         : None
* Return         : None
*******************************************************************************/
void DataEnd_recv(CAN_HEADER_U uiId,uint8_t* pucData){
    
    uint8_t aucData[8] = {0};
    uint16_t usCrc = 0xFFFF;
    uiId.stCanHeader.bDirection = 1;
    uiId.stCanHeader.bFrameNum = 0;
    aucData[0] = pucData[0];
    usCrc = pucData[1] + (pucData[2] << 8);
    //��CRCУ��Ա�
    if(g_usUGDataCrc != usCrc){
        UPGRADE_DEBUG("�����CRC");
        //RTN
        aucData[1] = eCrcError;
        g_stat = eUGFail;
    }else{
        //RTN
        aucData[1] = eNormal;
        g_stat = eUGforwarding;
    }
    
    OTA_Send(uiId.uiCanId, aucData);
}

/*******************************************************************************
* Function Name  : FirmUpProc_recv
* Description    : 0x04   �̼���������
* Input          : uiId         CanID
* Output         : None
* Return         : None
*******************************************************************************/
void FirmUpProc_recv(CAN_HEADER_U uiId){
    
    uint8_t aucData[8] = {0};
    uiId.stCanHeader.bDirection = 1;


    g_stCfg.stOta.uiAddr = (eHisOTAPStart + HIS_OTA_PLEN / 2) * PAGE_SIZE;
    g_stCfg.stOta.uiLen = g_uiUpDataLen;
    g_stCfg.stOta.ucUpdate = 0x55;
    //g_stCfg.stOta.usUpDataCrc = g_usUGDataCrc;
    cfg_ota_save();
    aucData[0] = 0x04;
    aucData[1] = eUGSucess;
    OTA_Send(uiId.uiCanId, aucData);
    g_stat = eUGSucess;
    //HAL_Delay(100);
    System_Reset();




    
}

/*******************************************************************************
* Function Name  : DataEnd_recv
* Description    : 0x05   ��ȡ����״̬
* Input          : uiId         CanID
* Output         : None
* Return         : None
*******************************************************************************/
void GetUpStatus_recv(CAN_HEADER_U uiId){
    
//    uint8_t aucData[8] = {0};
//    uiId.stCanHeader.bDirection = 1;
////    aucData[0] = 0x05;
////    aucData[1] = g_stat;
//    
//    aucData[0] = GetUpStatus;
//    aucData[1] = g_stCfg.stLocal.ucSVerVMajor;
//    aucData[2] = g_stCfg.stLocal.ucSVerVMinor;
//    aucData[3] = g_stCfg.stLocal.ucSVerRMajor;
//    aucData[4] = g_stCfg.stLocal.ucSVerRMinor;
//    OTA_Send(uiId.uiCanId, aucData, FDCAN_DLC_BYTES_8);
//    Mbs_S stmbs;
//    stmbs.ucFunC = 0x04;
//    stmbs.usRegNum = 0x15;
//    stmbs.usStartAddr = 0x1001;

//    dccMbs_rir_SendMsg(stmbs);
//    HAL_Delay(100);
//    g_stMbs.ucFunC = 0x01;
//    g_stMbs.usRegData[0] = 0x5A5A;
//    //stMbs.usRegData[1] = 0x5A;
//    g_stMbs.usRegNum = 1;
//    g_stMbs.usStartAddr = 0x5010;
//    dccMbs_ao_SendMsg(g_stMbs);
}

/*******************************************************************************
* Function Name  : OTA_Ctrl_recv_proc
* Description    : 0x04         OTA Ctrl
* Input          : uiId         CanID
                   pucData      ��������
* Output         : None
* Return         : None
*******************************************************************************/
void OTA_Ctrl_recv_proc(CAN_HEADER_U uiId,uint8_t* pucData){
    uint8_t OTACode;
    OTACode  = pucData[0];
    switch(OTACode){
        case 0x00:
            OTARequest_recv(uiId,pucData);
            break;
        case 0x01:
            OTAInfo_recv(uiId,pucData);
            break;
        case 0x02:
            DataSubInfo_recv(uiId,pucData);
            break;
        case 0x03:
            DataEnd_recv(uiId,pucData);
            break;
        case 0x04:
            FirmUpProc_recv(uiId);
            break;
        case 0x05:
            GetUpStatus_recv(uiId);
            break;
        default:
        break;
    }
}


/*******************************************************************************
* Function Name  : OTA_Data_recv_proc
* Description    : 0x05         OTA Data
* Input          : uiId         CanID
                   pucData      ��������
* Output         : None
* Return         : None
*******************************************************************************/
void OTA_Data_recv_proc(CAN_HEADER_U uiId,uint8_t* pucData){

    uiId.stCanHeader.bDirection = 1;
    uint8_t aucData[8] = {0};
    
    if(uiId.stCanHeader.bFrameFlag == 0 && g_ucUGBlkIdx == uiId.stCanHeader.bFrameNum){                 
        memcpy(g_aucOTAPackData + g_ucUGBlkIdx * UPGRADE_BLK_SIZE, pucData, 8);
        g_ucUGBlkIdx++;
    }else if(uiId.stCanHeader.bFrameFlag == 1 && g_ucUGBlkIdx == uiId.stCanHeader.bFrameNum){       //���һ֡
        uint16_t usCrc = 0xFFFF;
        uint8_t  g_ucLeftLen = 0;       //ʣ�೤��
        g_ucLeftLen = (g_usUGPackLen - 1) % UPGRADE_BLK_SIZE + 1;
        memcpy(g_aucOTAPackData + g_ucUGBlkIdx * UPGRADE_BLK_SIZE, pucData, g_ucLeftLen);
        g_ucUGBlkIdx++;

        usCrc = canup_crc16(g_aucOTAPackData, g_usUGPackLen, usCrc);
        //������CRCУ��
        g_usUGDataCrc = canup_crc16(g_aucOTAPackData,g_usUGPackLen,g_usUGDataCrc);
        if(g_usUGPackCrc != usCrc){
            //HYBCAN_DEBUG("�����CRC");
            //RTN
            aucData[0] = eCrcError;
        }else{
            MEM_FlashWrite(g_uiUGWriteAddr + g_usUGPackId * UPGRADE_PacketMaxLen,g_aucOTAPackData,g_usUGPackLen);
            MEM_FlashRead(g_uiUGWriteAddr + g_usUGPackId * UPGRADE_PacketMaxLen,g_aucOTAPackDataREAD,g_usUGPackLen);
            if(memcmp(g_aucOTAPackData,g_aucOTAPackDataREAD,2048) != 0){
                aucData[0] = eOperateError;
            }
            //RTN
            aucData[0] = eNormal;
            g_usUGPackSN++;
        }
        memset(g_aucOTAPackData, 0, 2048);
        
        
        uiId.stCanHeader.bFunCode = 0x05;
        uiId.stCanHeader.bFrameNum = 0;
        uiId.stCanHeader.bFrameFlag = 1;
        //�����
        aucData[1] = g_usUGPackId & 0xFF;
        aucData[2] = (g_usUGPackId >> 8) & 0xFF;
        aucData[3] = (g_usUGPackId >> 16) & 0xFF;
        //������
        aucData[4] = g_usUGPackLen & 0xFF;
        aucData[5] = (g_usUGPackLen >> 8) & 0xFF;

        OTA_Send(uiId.uiCanId, aucData);
    }else{
        aucData[0] = eOperateError;
        OTA_Send(uiId.uiCanId, aucData); 
    }
}


void OTA_Send(uint32_t uiId, uint8_t *pucData){

		//FDCAN2_SendMsg(uiId, pucData, FDCAN_DLC_BYTES_8); 
}

