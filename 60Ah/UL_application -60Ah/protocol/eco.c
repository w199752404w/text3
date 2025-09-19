#include "eco.h"
#include "config.h"
#include "bsp_rtc.h"
#include "parallel.h"
#include "local.h"
#include "bsp.h"
#include "bsp_sh367309.h"
#include "bsp_exti.h"
#include "bsp_gpio.h"
#include "bsp_wdg.h"
#include "history.h"
#include "bsp_gd25q32.h"
#include "bsp_usart.h"
#include "systick.h"
#include "parallel.h"
#include "protocol.h"

#include <string.h>
#include <time.h>
#include <stdio.h>
#include <math.h>

ECO_RTV_S g_stEcoRtv = {0};
ECO_RTV_S g_stEcoRtv_Parallel[PRL_MAX_NODE_NUM + 1] = {0};	//0 ~ PRL_MAX_NODE_NUM-1: every node, PRL_MAX_NODE_NUM: parallel totol info
ECO_LOG_S g_stEcoLog = {0};
ECO_CD_S g_stEcoCD = {0};

uint8_t g_aucCanSBuf[512] = {0};
uint8_t g_aucUartSBuf[256] = {0};

uint32_t g_uiUGBufIdx = 0;		//position to be write next
uint8_t g_aucUGBuf[2048] = {0};
uint32_t g_uiUGDataLen = 0;
uint32_t g_uiUGCrc = 0;
int16_t g_sUGSegIdx = -1;

bool g_bWriteWave = false;
uint16_t g_usMChgerComTick = 0;	//timeout of no can id 0x18ff50e5 received from external can
uint16_t g_usSChgerComTick = 0;	//timeout of no can id 0x773 received from external can
uint16_t g_us18F880F3ComTick = 0;	//timeout of no can id 0x18F880F3 received from external can
uint16_t g_usGbms3ComTick = 0;  //Identify if GBMS3 is connected
bool g_bMChgerComAct = false;		//0-off, 1-on; judge main charger on/off by external can id 0x18FF50E5 received timeout
bool g_bMChging = false;				//0-off, 1-on; judge main charger is charging now by external can id 0x18FF50E5
bool g_bMChgerAbNor = false;		//0-normal, 1- abnormal; judge main charger connecting state by external can id 0x18ff50e5 data byte[1]
bool g_bSChging = false;				//0-off, 1-on; judge second charger is charging now by external can id 0x773
bool g_bSChgerComAct = false;		//0-off, 1-on; judge second charger on/off by external can id 0x773 received timeout
bool g_bSetMChgerAct = false;		//0-off, 1-on; set main charger on or off
bool g_bSetSChgerAct = false;		//0-off, 1-on; set second charger on or off
bool g_bs18F880F3Act = false;   //0-off, 1-on; set 0x18F880F3 on or off

bool g_bGbms3 = true;					//0-off, 1-on; set 0x203000 on or off

#ifdef USER_DEBUG
	uint16_t g_usSrlCntTick = 1;
#else
	uint16_t g_usSrlCntTick = 0;
#endif
uint16_t g_usPassword = 0;

/*******************************************************************************
* Function Name  : eco_crc16
* Description    : Calculation of CRC16
* Input          : pucBuf,	The start address of the array being calculated
									 uiDataLen, The length of the array being calculated
									 usCRCin, The initial value of CRC, which is used for piecewise validation of long arrays
									 usPoly, Calculate the factor, 0x1021-XMODEM, 0x8005- MODBUS...
* Output         : None
* Return         : result, CRC calculation results
*******************************************************************************/
uint16_t eco_crc16(const uint8_t* pucBuf, const uint16_t usLen, const uint16_t usDefCRC, const uint16_t usPoly) {
	if(0 == pucBuf) {
		return 0;
	}
	if(0 == usLen) {
		return 0;
	}
	uint16_t crc = usDefCRC;
	for(uint16_t i=0;i<usLen;i++) {
		crc ^= pucBuf[i];
		for(uint16_t j=8;j!=0;j--) {
			if((crc & 0x01) != 0) {
				crc >>= 1;
				crc ^= usPoly;
			} else {
				crc >>= 1;
			}
		}
	}
	
	return crc;
}

unsigned short eco_crc16_XMODEM(unsigned char *ptr, int len)
{
    unsigned int i;
    unsigned short crc = 0x0000;
    
    while(len--)
    {
        crc ^= (unsigned short)(*ptr++) << 8;
        for (i = 0; i < 8; ++i)
        {
            if (crc & 0x8000)
                crc = (crc << 1) ^ 0x1021;
            else
                crc <<= 1;
        }
    }
    
    return crc;
}

uint32_t eco_crc32(uint8_t* pucBuf, uint32_t uiDataLen, uint32_t uiCRCin, uint32_t uiPoly) {
    uint32_t uiRet = uiCRCin;
    for(uint32_t i=0;i<uiDataLen;i++) {
        uiRet ^= pucBuf[i];
        for(uint8_t j=0;j<8;j++) {
            if(uiRet & 1) {
                uiRet = (uiRet >> 1) ^ uiPoly;
            } else {
                uiRet >>= 1;
            }
        }
    }

    return ~uiRet;
}

void eco_can_ug_proc(CAN_RBUF_S stCanRBuf) {
	uint8_t aucSBuf[8] = {0};
	uint32_t uiCrc;
	uint8_t CanugDelay = 0;
	switch (g_stPrl.ucSelfId) {
		case 0: CanugDelay = 1; break;
		case 1: CanugDelay = 5; break;
		case 2: CanugDelay = 6 ; break;
		case 3: CanugDelay = 7 ; break;
	}
	switch(stCanRBuf.uiId) {
	case eCanIdUGMode:
		if(g_stLocalArrayRVal.eLocalStat == eLocalStatUartUpgrade || g_stLocalArrayRVal.eLocalStat == eLocalStatUartBootMode) {
			aucSBuf[0] = 0x02;
			aucSBuf[1] = g_stPrl.ucSelfId;
			CAN0_SendMsg(eCanIdUGRly, aucSBuf, 2);
			return;
		}
		g_uiUGBufIdx = 0;
		g_uiUGDataLen = 0;
		g_sUGSegIdx = -1;
		g_stLocalArrayRVal.eLocalStat = eLocalStatCanUpgrade;
		BSP_Stop();
		g_eBaseStat = eBStatDownload;
		his_data_write();
		aucSBuf[0] = 0x01;
		aucSBuf[1] = g_stPrl.ucSelfId;
		CAN0_SendMsg(eCanIdUGRly, aucSBuf, 2);
		break;
	case eCanIdUGRun:
		if(stCanRBuf.aucData[0] == 0x02 && (g_uiUGBufIdx >= g_uiUGDataLen || g_uiUGDataLen == 0)) {
			if(g_sUGSegIdx + 1 == stCanRBuf.aucData[1]) {
				g_sUGSegIdx = stCanRBuf.aucData[1];
				g_uiUGBufIdx = 0;
				aucSBuf[0] = 0x01;
				aucSBuf[1] = g_stPrl.ucSelfId;
				delay_1ms(CanugDelay);
				CAN0_SendMsg(eCanIdUGRly, aucSBuf, 2);
			} else {
				aucSBuf[0] = 0x02;
				aucSBuf[1] = g_stPrl.ucSelfId;
				CAN0_SendMsg(eCanIdUGRly, aucSBuf, 2);
				return;
			}
			g_uiUGDataLen = stCanRBuf.aucData[2] * 0x100 + stCanRBuf.aucData[3];
			g_uiUGCrc = (stCanRBuf.aucData[4] << 24) | (stCanRBuf.aucData[5] << 16) | (stCanRBuf.aucData[6] << 8) |stCanRBuf.aucData[7];
		} else {
			if(stCanRBuf.ucDLC == 5 && stCanRBuf.aucData[0] == 0x03 && stCanRBuf.aucData[1] == 0x42 && stCanRBuf.aucData[2] == 0x6F
				&& stCanRBuf.aucData[3] == 0x6F && stCanRBuf.aucData[4] == 0x74) {
					return;
				}
			if(g_uiUGBufIdx >= 2048) {
				aucSBuf[0] = 0x02;
				aucSBuf[1] = g_stPrl.ucSelfId;
				CAN0_SendMsg(eCanIdUGRly, aucSBuf, 2);
				return;
			}
			if(g_uiUGBufIdx + 8 > 2048) {
				memcpy(g_aucUGBuf + g_uiUGBufIdx, stCanRBuf.aucData, 2048 - g_uiUGBufIdx);
				g_uiUGBufIdx = 2048;
			} else {
				memcpy(g_aucUGBuf + g_uiUGBufIdx, stCanRBuf.aucData, 8);
				g_uiUGBufIdx += 8;
			}
			if(g_uiUGBufIdx >= g_uiUGDataLen) {
				uiCrc = eco_crc32(g_aucUGBuf, g_uiUGDataLen, 0xFFFFFFFF, 0xEDB88320);
				if(uiCrc != g_uiUGCrc) {
					aucSBuf[0] = 0x02;
					aucSBuf[1] = g_stPrl.ucSelfId;
					CAN0_SendMsg(eCanIdUGRly, aucSBuf, 2);
					return;
				}
				MEM_FlashWrite(eHisOTAPStart * PAGE_SIZE + g_sUGSegIdx * 2048, g_aucUGBuf, g_uiUGDataLen);
				//printf("%08x, %04x\n", eHisOTAPStart * PAGE_SIZE + g_sUGSegIdx * 2048, g_uiUGDataLen);
				aucSBuf[0] = 0x01;
				aucSBuf[1] = g_stPrl.ucSelfId;
				delay_1ms(CanugDelay);
				CAN0_SendMsg(eCanIdUGRly, aucSBuf, 2);
				if(g_uiUGDataLen != 2048) {
					cfg_set_default();
					g_stCfg.stOta.uiAddr = eHisOTAPStart * PAGE_SIZE;
					g_stCfg.stOta.uiLen = g_sUGSegIdx * 2048 + g_uiUGDataLen;
					g_stCfg.stOta.ucUpdate = 0x01;
					g_stCfg.usGoRun = 1;
				 	aucSBuf[0] = 0x10;
					aucSBuf[1] = g_stPrl.ucSelfId;
					delay_1ms(1000);
				  CAN0_SendMsg(eCanIdUGRly, aucSBuf, 2);
					aucSBuf[0] = 0x20;
					aucSBuf[1] = g_stPrl.ucSelfId;
					delay_1ms(1000);
				 	CAN0_SendMsg(eCanIdUGRly, aucSBuf, 2);
					cfg_save();
					delay_1ms(100);
					g_eBaseStat = eBStatReset_52;
					his_data_write();
					delay_1ms(500);
					System_Reset();
				}
			}
		}
		break;
	case eCanIdUGStop:
		aucSBuf[0] = 0x01;
		aucSBuf[1] = g_stPrl.ucSelfId;
		CAN0_SendMsg(eCanIdUGRly, aucSBuf, 2);
		g_stLocalArrayRVal.eLocalStat = eLocalStatRun;
//		delay_1ms(100);
//		cfg_set_default();
//	  g_stCfg.usGoRun = 1;
//		cfg_save();
//	  g_eBaseStat = eBStatReset_52;
//	  delay_1ms(100);
//		his_data_write();
//	  delay_1ms(500);
//		System_Reset();
		break;
	default:
		break;
	}
}

void eco_can_recv_proc(CAN_RBUF_S stCanRBuf) {
	if(stCanRBuf.uiId == 0x111 || stCanRBuf.uiId == 0x108 || stCanRBuf.uiId == 0x110 || stCanRBuf.uiId == 0x11F) {
		return;
	}
	/* Identify whether it is connected to GBMS3 through 0x203000 */
	if(stCanRBuf.uiId == 0x203000) {
		g_bGbms3 = true;
		g_usGbms3ComTick = 5000 / g_stCfg.stLocal.usCyclePeriod;
		uint8_t aucSBuf[8] = {0};
		aucSBuf[0] = 1;
		CAN0_SendMsg(stCanRBuf.uiId , aucSBuf, 1);
		return ;
	}
	
	/* The broadcast goes into hibernation */
	if(stCanRBuf.uiId == 0x00203100 || stCanRBuf.uiId == 0x02203100 || stCanRBuf.uiId == 0x04203100 || stCanRBuf.uiId == 0x06203100) {
		g_stCfg.usGoRun = 0;
		cfg_save();
		g_bNeedSleep = true;
		return ;
	}
	
	/* 0x18F880F3 When there is a 0x7B1 or in the CAN message 0x7FF */
	if(/*stCanRBuf.uiId == 0x7B1 || */stCanRBuf.uiId == 0x7FF) {
		g_us18F880F3ComTick = 5000 / g_stCfg.stLocal.usCyclePeriod;
		return;
	}
	
	if(stCanRBuf.uiId >= eCanIdUGMode && stCanRBuf.uiId <= eCanIdUGRly) {
		SW_EN_LED_L;
		eco_can_ug_proc(stCanRBuf);		//firmware upgrade process
		SW_EN_LED_H;
		return;
	} 
//	else if(g_stLocalArrayRVal.eLocalStat == eLocalStatCanUpgrade) { //Prevent upgrade disruptions 2024.8.5
//		g_stCfg.usGoRun = 1;
//		cfg_save();
//		g_eBaseStat = eBStatReset_52;
//		delay_1ms(100);
//		his_data_write();
//		delay_1ms(500);
//		System_Reset();
//		return;
//	}
	
	/* The main charger is detected */ 
	if(stCanRBuf.uiId == eCanIdChgReq17) {
		if(stCanRBuf.ucDLC != 8) {
			return;
		}
		if((stCanRBuf.aucData[0] == 0x04 && stCanRBuf.aucData[1] == 0) || (stCanRBuf.aucData[0] == 0x06 && stCanRBuf.aucData[1] == 0)) {
			g_bMChgerAbNor = true;
		} else {
			g_bMChgerAbNor = false;
		}
		g_usMChgerComTick = 5000 / g_stCfg.stLocal.usCyclePeriod;
		g_stEcoRtv.usMChgerActCur = stCanRBuf.aucData[4] * 255 + stCanRBuf.aucData[5];
		if(stCanRBuf.aucData[2] != 0) {
			if(g_stEcoRtv.usMChgerActCur != 0) {
				g_bMChging = true;
			} else {
				g_bMChging = false;
			}
		}
		return;
	}
	
	/* The second charger is detected */
	if(stCanRBuf.uiId == eCanIdChgReq19) {
		if(stCanRBuf.ucDLC != 8) {
			return;
		}
		g_stEcoRtv.usSChgerActCur = stCanRBuf.aucData[0] + stCanRBuf.aucData[1] * 0x100;
		g_usSChgerComTick = 5000 / g_stCfg.stLocal.usCyclePeriod;
		return;
	}
	
	if(stCanRBuf.uiId == 0x02206500 || stCanRBuf.uiId == 0x04206500 || stCanRBuf.uiId == 0x06206500) {
		if(stCanRBuf.uiId >> 25 == g_stPrl.ucSelfId) {
			g_ausPrlComTick[g_stPrl.ucSelfId] = PRL_COM_MSEC * 1000 / g_stCfg.stLocal.usCyclePeriod;
		}
	}
	
	/* parallel communication， */	
	/* The master receives real-time data from the slave and stores it at the corresponding location */
	/* Begin :  Modified by dgx, 2024.4.2 */
	if((stCanRBuf.uiId == 0x2206500 || stCanRBuf.uiId == 0x4206500 || stCanRBuf.uiId == 0x6206500) && (g_stLocalArrayRVal.eLocalStat != eLocalStatCanUpgrade)) {
//	if(stCanRBuf.uiId == 0x023F6500 || stCanRBuf.uiId == 0x043F6500 || stCanRBuf.uiId == 0x063F6500) {
	/* End   :  Modified by dgx, 2024.4.2 */
		if(stCanRBuf.aucData[0] == 1) {
			g_ausPrlComTick[g_stPrl.ucSelfId] = PRL_COM_MSEC * 1000 / g_stCfg.stLocal.usCyclePeriod;
		}
		if(stCanRBuf.ucDLC <= 1) {
			return;
		}
		uint8_t ucSegFlag = (stCanRBuf.aucData[0] >> 6) & 0x03;
		uint8_t ucSegCnt = stCanRBuf.aucData[0] & 0x3F;
		uint8_t ucSlaveId = (stCanRBuf.uiId >> 25);
		
		if(ucSlaveId >= PRL_MAX_NODE_NUM) {
			return;
		}
		
		uint8_t aucSBuf[8] = {0};
		aucSBuf[0] = 1;
		CAN0_SendMsg(stCanRBuf.uiId , aucSBuf, 1);
		
		static uint32_t s_auiPrlRBufIdx[PRL_MAX_NODE_NUM] = {0};
		if(ucSegFlag == 1) {
			memcpy((uint8_t*)(g_stEcoRtv_Parallel + ucSlaveId), stCanRBuf.aucData + 1, stCanRBuf.ucDLC - 1);
			s_auiPrlRBufIdx[ucSlaveId] = stCanRBuf.ucDLC - 1;
		} else if(ucSegFlag == 2) {
			if(ucSegCnt * 7 != s_auiPrlRBufIdx[ucSlaveId]) {
				return;
			}
			memcpy((uint8_t*)(g_stEcoRtv_Parallel + ucSlaveId) + s_auiPrlRBufIdx[ucSlaveId], stCanRBuf.aucData + 1, stCanRBuf.ucDLC - 1);
			s_auiPrlRBufIdx[ucSlaveId] += stCanRBuf.ucDLC - 1;
		} else if(ucSegFlag == 3) {
			memcpy((uint8_t*)(g_stEcoRtv_Parallel + ucSlaveId) + s_auiPrlRBufIdx[ucSlaveId], stCanRBuf.aucData + 1, stCanRBuf.ucDLC - 1);
			s_auiPrlRBufIdx[ucSlaveId] = 0;
		} else {
			return;
		}
		return;
	}
	if(stCanRBuf.uiId == 0x02206400 || stCanRBuf.uiId == 0x04206400 || stCanRBuf.uiId == 0x06206400) {
		if(stCanRBuf.ucDLC <= 1) {
			return;
		}
		uint8_t ucSegFlag = (stCanRBuf.aucData[0] >> 6) & 0x03;
		uint8_t ucSegCnt = stCanRBuf.aucData[0] & 0x3F;
		uint8_t ucSlaveId = (stCanRBuf.uiId >> 25);
		if(ucSlaveId >= PRL_MAX_NODE_NUM) {
			return;
		}
		static uint32_t s_auiPrlRBufIdx[PRL_MAX_NODE_NUM] = {0};
		if(ucSegFlag == 1) {
			memcpy((uint8_t*)(g_eBaseAlm_Parallel + ucSlaveId), stCanRBuf.aucData + 1, stCanRBuf.ucDLC - 1);
			s_auiPrlRBufIdx[ucSlaveId] = stCanRBuf.ucDLC - 1;
		} else if(ucSegFlag == 2) {
			if(ucSegCnt * 7 != s_auiPrlRBufIdx[ucSlaveId]) {
				return;
			}
			memcpy((uint8_t*)(g_eBaseAlm_Parallel + ucSlaveId) + s_auiPrlRBufIdx[ucSlaveId], stCanRBuf.aucData + 1, stCanRBuf.ucDLC - 1);
			s_auiPrlRBufIdx[ucSlaveId] += stCanRBuf.ucDLC - 1;
		} else if(ucSegFlag == 3) {
			memcpy((uint8_t*)(g_eBaseAlm_Parallel + ucSlaveId) + s_auiPrlRBufIdx[ucSlaveId], stCanRBuf.aucData + 1, stCanRBuf.ucDLC - 1);
			s_auiPrlRBufIdx[ucSlaveId] = 0;
		} else {
			return;
		}
		return;
	}
	union ECO_CAN_HEADER_U {
		uint32_t uiCanId;
		struct ECO_CAN_HEADER_S {
//			uint32_t ucRegAddr:8;
//			uint32_t ucFunCode:4;
//			uint32_t usReserve:9;
//			uint32_t ucDevAddr:6;
//			uint32_t ucSelfId:2;
			uint32_t ucRegAddr:8;
			uint32_t ucFunCode:8;
			uint32_t usReserve:5;
			uint32_t ucDevAddr:4;
			uint32_t ucSelfId:4;
		} stCanId;
	} uHeader;
	uHeader.uiCanId = stCanRBuf.uiId;
	if(/*uHeader.stCanId.usReserve != 0x1F && */uHeader.stCanId.usReserve != 0) { //设备地址固定为1或者0，故修改 2024/8/14
		return;
	}
	if(uHeader.stCanId.ucDevAddr >= PRL_MAX_NODE_NUM || uHeader.stCanId.ucDevAddr <= 0) { //dgx 2025/1/14
		return;
	}
	g_ausPrlComTick[uHeader.stCanId.ucDevAddr - 1] = PRL_COM_MSEC * 1000 / g_stCfg.stLocal.usCyclePeriod;
	static uint8_t s_aucRData[512];
	static uint16_t s_usRDataLen = 0;
	static uint8_t s_ucDevAddr = 0;
	static uint8_t s_ucLastFunCode = 0;
	static uint8_t s_ucLastSegCnt = 0;
	if(s_ucLastFunCode != uHeader.stCanId.ucFunCode || s_ucDevAddr != uHeader.stCanId.ucDevAddr) {
		memset(s_aucRData, 0, 512);
		s_usRDataLen = 0;
		s_ucLastFunCode = uHeader.stCanId.ucFunCode;
		s_ucDevAddr = uHeader.stCanId.ucDevAddr;
	}
	uint8_t ucSegFlag = (stCanRBuf.aucData[0] >> 6) & 0x03;
	uint8_t ucSegCnt = stCanRBuf.aucData[0] & 0x3F;
	if(ucSegFlag == 0) {	//no frame split
		if(s_ucLastFunCode == 0x01 || s_ucLastFunCode == 0x09) {
			eco_data_proc(uHeader.stCanId.ucSelfId, uHeader.stCanId.ucDevAddr, uHeader.stCanId.ucFunCode, uHeader.stCanId.ucRegAddr,
			stCanRBuf.aucData + 1, 2);
		} else if(s_ucLastFunCode == 0x02 || s_ucLastFunCode == 0x0D || s_ucLastFunCode == 0x0F) {
			eco_data_proc(uHeader.stCanId.ucSelfId, uHeader.stCanId.ucDevAddr, uHeader.stCanId.ucFunCode, uHeader.stCanId.ucRegAddr,
			stCanRBuf.aucData + 1, stCanRBuf.aucData[1]);
		} else if(s_ucLastFunCode == 0x03) {		//read u16 register address info
			eco_data_proc(uHeader.stCanId.ucSelfId, uHeader.stCanId.ucDevAddr, uHeader.stCanId.ucFunCode, stCanRBuf.aucData[3] * 0x100 + stCanRBuf.aucData[4],
			stCanRBuf.aucData + 1, stCanRBuf.aucData[1] * 0x100 + stCanRBuf.aucData[2]);
		} else if(s_ucLastFunCode == 0x04) {		//write u16 register address info
			eco_data_proc(uHeader.stCanId.ucSelfId, uHeader.stCanId.ucDevAddr, uHeader.stCanId.ucFunCode, stCanRBuf.aucData[3] * 0x100 + stCanRBuf.aucData[4],
			stCanRBuf.aucData + 5, stCanRBuf.ucDLC - 5);
		} else {
			eco_data_proc(uHeader.stCanId.ucSelfId, uHeader.stCanId.ucDevAddr, uHeader.stCanId.ucFunCode, uHeader.stCanId.ucRegAddr,
			stCanRBuf.aucData + 1, stCanRBuf.ucDLC - 1);
		}
		s_usRDataLen = 0;
		s_ucLastSegCnt = 0;
	} else if(ucSegFlag == 1) {	//begining of frame split
		memcpy(s_aucRData, stCanRBuf.aucData + 1, stCanRBuf.ucDLC - 1);
		s_usRDataLen = stCanRBuf.ucDLC - 1;
		s_ucLastSegCnt = 0;
	} else if(ucSegFlag == 2 && s_ucLastSegCnt + 1 == ucSegCnt) {	//middle of frame split
		memcpy(s_aucRData + s_usRDataLen, stCanRBuf.aucData + 1, stCanRBuf.ucDLC - 1);
		s_usRDataLen += (stCanRBuf.ucDLC - 1);
		s_ucLastSegCnt ++;
	} else if(ucSegFlag == 3) {	//end of frame split
		memcpy(s_aucRData + s_usRDataLen, stCanRBuf.aucData + 1, stCanRBuf.ucDLC - 1);
		s_usRDataLen += (stCanRBuf.ucDLC - 1);
		s_ucLastSegCnt = 0;
		if(uHeader.stCanId.ucFunCode == 0x04) {
			eco_data_proc(uHeader.stCanId.ucSelfId, uHeader.stCanId.ucDevAddr, uHeader.stCanId.ucFunCode, s_aucRData[2] * 0x100 + s_aucRData[3],
			s_aucRData + 4, s_usRDataLen - 4);
		} else {
			eco_data_proc(uHeader.stCanId.ucSelfId, uHeader.stCanId.ucDevAddr, uHeader.stCanId.ucFunCode, uHeader.stCanId.ucRegAddr, s_aucRData, s_usRDataLen);
		}
	}
}

uint16_t eco_read_reg(uint8_t ucSelfId, uint8_t ucDevAddr, uint16_t usRegAddr) {
	uint16_t usRet = 0;
	if(usRegAddr < 0xFF) {
		if(usRegAddr * 2 >= sizeof(ECO_RTV_S)) {	//not enough registers
			return 0;
		}
		if(g_stPrl.ucDevNum == 1) {
			memcpy(&usRet, ((uint8_t*)&g_stEcoRtv) + usRegAddr * sizeof(uint16_t), sizeof(uint16_t));
		} else if(ucSelfId == 0 && ucDevAddr == 0) {
			memcpy(&usRet, ((uint8_t*)&g_stEcoRtv_Parallel[PRL_MAX_NODE_NUM]) + usRegAddr * sizeof(uint16_t), sizeof(uint16_t));
		} else if(ucSelfId < PRL_MAX_NODE_NUM && ucDevAddr == 1) {
			memcpy(&usRet, ((uint8_t*)&g_stEcoRtv_Parallel[ucSelfId]) + usRegAddr * sizeof(uint16_t), sizeof(uint16_t));
		} else {
			return 0;
		}
	} else if(usRegAddr >= 0x044C && usRegAddr < 0x0456) {
		usRet = g_stCfg.stLocal.aucBleName[(usRegAddr - 0x044C) * 2] * 0x100;
		usRet |= g_stCfg.stLocal.aucBleName[(usRegAddr - 0x044C) * 2 + 1];
	} else if(usRegAddr >= 0x0480 && usRegAddr < 0x0499) {
		usRet = g_stCfg.stLocal.aucBMS_ID[(usRegAddr - 0x0480) * 2] * 0x100;
		usRet |= g_stCfg.stLocal.aucBMS_ID[(usRegAddr - 0x0480) * 2 + 1];
	} else if(usRegAddr >= 0x04B0 && usRegAddr < 0x04BA) {
		uint8_t aucCanSBuf[20];
//		aucCanSBuf[0] = 'E';
//		aucCanSBuf[1] = 'B';
//		aucCanSBuf[2] = '3';
//		aucCanSBuf[3] = '0';
//		aucCanSBuf[4] = '-';
//		aucCanSBuf[5] = 'v';
//		aucCanSBuf[6] = g_stCfg.stLocal.ucSVerVMajor + 0x30;
//		aucCanSBuf[7] = '.';
//		aucCanSBuf[8] = g_stCfg.stLocal.ucSVerVMinor + 0x30;
//		aucCanSBuf[9] = '.';
////		aucCanSBuf[11] = g_stCfg.stLocal.ucSVerRMajor + 0x30;
//	  aucCanSBuf[10] = g_stCfg.stLocal.ucSVerRMinor + 0x30;
//		aucCanSBuf[11] = '5';
//		if(g_stCfg.stLocal.ucSVerAgree == 'A') {//2024.12.14 dgx
//		} else if(g_stCfg.stLocal.ucSVerAgree == 'B') {
//		} else if(g_stCfg.stLocal.ucSVerAgree == 'C') {
//		} else if(g_stCfg.stLocal.ucSVerAgree == 'D') {
//		} else if(g_stCfg.stLocal.ucSVerAgree == 'E') {
//		} else {
//			g_stCfg.stCom.ucCanPtcType = 1;
//			g_stCfg.stLocal.ucSVerAgree = 'A';
//			cfg_save();
//		}
//		aucCanSBuf[12] = g_stCfg.stLocal.ucSVerAgree;//'A'
//		aucCanSBuf[13] = '-';
//		aucCanSBuf[14] = 'G';
//		aucCanSBuf[15] = 'e';
//		aucCanSBuf[16] = 'n';
//		aucCanSBuf[17] = '3';
//		aucCanSBuf[18] = 0;
//		aucCanSBuf[19] = 0;
		aucCanSBuf[0] = 'L';
		aucCanSBuf[1] = 'Z';
		aucCanSBuf[2] = 'H';
		aucCanSBuf[3] = '3';
		aucCanSBuf[4] = '0';
		aucCanSBuf[5] = '-';
		aucCanSBuf[6] = 'v';
		aucCanSBuf[7] = g_stCfg.stLocal.ucSVerVMajor + 0x30;
		aucCanSBuf[8] = '.';
		aucCanSBuf[9] = g_stCfg.stLocal.ucSVerVMinor + 0x30;
		aucCanSBuf[10] = '.';
		aucCanSBuf[11] = g_stCfg.stLocal.ucSVerRMajor + 0x30;
	  aucCanSBuf[12] = g_stCfg.stLocal.ucSVerRMinor + 0x30;
//		aucCanSBuf[12] = '5';
		if(g_stCfg.stLocal.ucSVerAgree == 'A') {//2024.12.14 dgx
		} else if(g_stCfg.stLocal.ucSVerAgree == 'B') {
		} else if(g_stCfg.stLocal.ucSVerAgree == 'C') {
		} else if(g_stCfg.stLocal.ucSVerAgree == 'D') {
		} else if(g_stCfg.stLocal.ucSVerAgree == 'E') {
		} else if(g_stCfg.stLocal.ucSVerAgree == 'F') {
		} else if(g_stCfg.stLocal.ucSVerAgree == 'G') {
		} else {
			g_stCfg.stCom.ucCanPtcType = 1;
			g_stCfg.stLocal.ucSVerAgree = 'A';
			cfg_save();
		}
		aucCanSBuf[13] = g_stCfg.stLocal.ucSVerAgree;//'A'
		aucCanSBuf[14] = '-';
		aucCanSBuf[15] = 'G';
		aucCanSBuf[16] = 'e';
		aucCanSBuf[17] = 'n';
		aucCanSBuf[18] = '3';
		aucCanSBuf[19] = 0;
		usRet = aucCanSBuf[(usRegAddr - 0x04B0) * 2] * 0x100;
		usRet |= aucCanSBuf[(usRegAddr - 0x04B0) * 2 + 1];
	}
	return usRet;
}

void eco_write_reg(uint16_t usRegAddr, uint16_t usData) {
	uint8_t ucNeedSave = 0;
	uint8_t ucBackUpNeedSave = 0;
	uint8_t ucNeedHisSave = 0;
	LOCAL_PACK_RVAL_S* pstPackRVal = g_stLocalArrayRVal.astPackRVal + g_stPrl.ucSelfId;
	switch(usRegAddr) {
	case 0x01:	//usPackNominalAh
		g_stCfg.stLocal.usDesignAH = usData / 10;
		ucNeedSave = 1;
		break;
	case 0x05:	//usPackRTSoc
		if(usData > 1000) {
			break;
		}
		for(uint8_t i=0;i<CFG_CELL_NUM;i++) {
		  pstPackRVal->afCellLeftAH[i] = pstPackRVal->fPackRealAH * usData /*/ 10*/ / 100;
			g_stLocalArrayRVal.astPackRVal[g_stPrl.ucSelfId].afCellLeftAH[i] = pstPackRVal->afCellLeftAH[i];
		}
		pstPackRVal->fPackSoc = usData /** 0.1*/;
		g_stEcoRtv.usPackRTSoc = usData /** 0.1*/;
		ucNeedHisSave = 1;
		break;
	case 0x07:  //usCycle  2024.7.4增加清除循环次数
		pstPackRVal->usCycle = g_stCfg.stLocal.usCycle;
		ucNeedSave = 1;
		break;
	case 0x0A:	//usCellUVTVThr2
		g_stCfg.stLocal.usCellUVTVThr1 = usData;
		ucNeedSave = 1;
		break;
	case 0x0B:	//usCellOVTVThr1
		g_stCfg.stLocal.usCellOVTVThr1 = usData;
		ucNeedSave = 1;
		break;
	case 0x0C:	//sCellOCTTVThr1
		g_stCfg.stLocal.sCellOCTTVThr1 = (int16_t)usData;
		ucNeedSave = 1;
		break;
	case 0x0D:	//sMOSOTTVThr2
		g_stCfg.stLocal.sMOSOTTVThr2 = (int16_t)usData;
		ucNeedSave = 1;
		break;
	case 0x0E:	//sCellUCTTVThr1
		g_stCfg.stLocal.sCellUCTTVThr1 = (int16_t)usData;
		ucNeedSave = 1;
		break;
	case 0x0F:	//sCellUDTTVThr1
		g_stCfg.stLocal.sCellUDTTVThr1 = (int16_t)usData;
		ucNeedSave = 1;
		break;
	case 0x10:	//usODCTVThr1
		g_stCfg.stLocal.usODCTVThr1 = usData;
	  	g_stCfg.stLocal.usODCRVThr1 = usData;
		ucNeedSave = 1;
		break;
	case 0x11:	//usOCCTVThr1
		g_stCfg.stLocal.usOCCTVThr1 = usData;
	  	g_stCfg.stLocal.usOCCRVThr1 = usData;
		ucNeedSave = 1;
		break;
	case 0x12:	//usCellUVRVThr2
		g_stCfg.stLocal.usCellUVRVThr1 = usData;
		ucNeedSave = 1;
		break;
	case 0x13:	//usCellOVRVThr1
		g_stCfg.stLocal.usCellOVRVThr1 = usData;
		ucNeedSave = 1;
		break;
	case 0x14:	//sCellOCTRVThr1
		g_stCfg.stLocal.sCellOCTRVThr1 = (int16_t)usData;
		ucNeedSave = 1;
		break;
	case 0x15:	//sMOSOTRVThr2
		g_stCfg.stLocal.sMOSOTRVThr2 = (int16_t)usData;
		ucNeedSave = 1;
		break;
	case 0x1C:	//sCellUCTRVThr1
		g_stCfg.stLocal.sCellUCTRVThr1 = (int16_t)usData;
		ucNeedSave = 1;
		break;
	case 0x1D:	//sCellUCTRVThr1
		g_stCfg.stLocal.sCellUDTRVThr1 = (int16_t)usData;
		ucNeedSave = 1;
		break;
	case 0x1E:	//sCellODTTVThr1
		g_stCfg.stLocal.sCellODTTVThr1 = (int16_t)usData;
		ucNeedSave = 1;
		break;
	case 0x1F:	//sCellODTRVThr1
		g_stCfg.stLocal.sCellODTRVThr1 = (int16_t)usData;
		ucNeedSave = 1;
		break;
	case 0xC4:  //fCDATACaliA_CHG
		g_stCfg.stAfe.fCDATACaliA_CHG = (float)usData / 10000;
		g_stCfg_BackUp.fCDATACaliA_CHG_BackUp = g_stCfg.stAfe.fCDATACaliA_CHG;
		MEM_FlashWrite(eCfgBackUpStart, (uint8_t*)&g_stCfg_BackUp, sizeof(CFG_BackUp));
		pstPackRVal->fCDATA = g_stAfe.stRamApp.fCDATA;
		ucNeedSave = 1;
		break;
	case 0xC5:  //fCDATACaliA_DSG
		g_stCfg.stAfe.fCDATACaliA_DSG = (float)usData / 10000;
		g_stCfg_BackUp.fCDATACaliA_DSG_BackUp = g_stCfg.stAfe.fCDATACaliA_DSG;
		MEM_FlashWrite(eCfgBackUpStart, (uint8_t*)&g_stCfg_BackUp, sizeof(CFG_BackUp));
		pstPackRVal->fCDATA = g_stAfe.stRamApp.fCDATA;
		ucNeedSave = 1;
		break;
	case 0xC6: 	//fCDATACaliA_CHG
		if(usData && g_stAfe.stRamApp.fCDATA > 0){
	    g_stCfg.stAfe.fCDATACaliA_CHG = (float)usData / 10 *1000 / g_stAfe.stRamApp.fCDATA * g_stCfg.stAfe.fCDATACaliA_CHG;
			g_stCfg_BackUp.fCDATACaliA_CHG_BackUp = g_stCfg.stAfe.fCDATACaliA_CHG;
			MEM_FlashWrite(eCfgBackUpStart, (uint8_t*)&g_stCfg_BackUp, sizeof(CFG_BackUp));
			pstPackRVal->fCDATA = g_stAfe.stRamApp.fCDATA;
		}else{
			g_stCfg.stAfe.fCDATACaliB -= g_stAfe.stRamApp.fCDATA;
			pstPackRVal->fCDATA = g_stAfe.stRamApp.fCDATA;
		}
		ucNeedSave = 1;
		break;
	case 0xC7: 	//fCDATACaliA_DSG 
		if(usData && g_stAfe.stRamApp.fCDATA < 0){
	    g_stCfg.stAfe.fCDATACaliA_DSG = (float)usData / 10 * 1000 / fabs(g_stAfe.stRamApp.fCDATA) * g_stCfg.stAfe.fCDATACaliA_DSG;
			g_stCfg_BackUp.fCDATACaliA_DSG_BackUp = g_stCfg.stAfe.fCDATACaliA_DSG;
			MEM_FlashWrite(eCfgBackUpStart, (uint8_t*)&g_stCfg_BackUp, sizeof(CFG_BackUp));
			pstPackRVal->fCDATA = g_stAfe.stRamApp.fCDATA;
		}else{
			g_stCfg.stAfe.fCDATACaliB -= g_stAfe.stRamApp.fCDATA;
			pstPackRVal->fCDATA = g_stAfe.stRamApp.fCDATA;
		}
		ucNeedSave = 1;
		break;
	case 0xC8 + 0: 	//CellVolCali[0]
		if(usData){
			g_stCfg.stAfe.afCellVolCali[0] = (float)usData / g_stAfe.stRamApp.fCELLVol[0] * g_stCfg.stAfe.afCellVolCali[0];
			ucNeedSave = 1;
		}
		break;
	case 0xC8 + 1: 	//CellVolCali[1]
		if(usData){
	    g_stCfg.stAfe.afCellVolCali[1] = (float)usData / g_stAfe.stRamApp.fCELLVol[1] * g_stCfg.stAfe.afCellVolCali[1];
		  ucNeedSave = 1;
		}
		break;
	case 0xC8 + 2: 	//CellVolCali[2]
		if(usData){
	    g_stCfg.stAfe.afCellVolCali[2] = (float)usData / g_stAfe.stRamApp.fCELLVol[2] * g_stCfg.stAfe.afCellVolCali[2];
		  ucNeedSave = 1;
		}
		break;
	case 0xC8 + 3: 	//CellVolCali[3]
		if(usData){
	    g_stCfg.stAfe.afCellVolCali[3] = (float)usData / g_stAfe.stRamApp.fCELLVol[3] * g_stCfg.stAfe.afCellVolCali[3];
	  	ucNeedSave = 1;
		}
		break;
	case 0xC8 + 4: 	//CellVolCali[4]
		if(usData){
	    g_stCfg.stAfe.afCellVolCali[4] = (float)usData / g_stAfe.stRamApp.fCELLVol[4] * g_stCfg.stAfe.afCellVolCali[4];
		  ucNeedSave = 1;
		}
		break;
	case 0xC8 + 5: 	//CellVolCali[5]
		if(usData){
	    g_stCfg.stAfe.afCellVolCali[5] = (float)usData / g_stAfe.stRamApp.fCELLVol[5] * g_stCfg.stAfe.afCellVolCali[5];
		  ucNeedSave = 1;
		}
		break;
	case 0xC8 + 6: 	//CellVolCali[6]
		if(usData){
	    g_stCfg.stAfe.afCellVolCali[6] = (float)usData / g_stAfe.stRamApp.fCELLVol[6] * g_stCfg.stAfe.afCellVolCali[6];
		  ucNeedSave = 1;
		}
		break;
	case 0xC8 + 7: 	//CellVolCali[7]
		if(usData){
	    g_stCfg.stAfe.afCellVolCali[7] = (float)usData / g_stAfe.stRamApp.fCELLVol[7] * g_stCfg.stAfe.afCellVolCali[7];
		  ucNeedSave = 1;
		}
		break;
	case 0xC8 + 8: 	//CellVolCali[8]
		if(usData){
	    g_stCfg.stAfe.afCellVolCali[8] = (float)usData / g_stAfe.stRamApp.fCELLVol[8] * g_stCfg.stAfe.afCellVolCali[8];
		  ucNeedSave = 1;
		}
		break;
	case 0xC8 + 9: 	//CellVolCali[9]
		if(usData){
	    g_stCfg.stAfe.afCellVolCali[9] = (float)usData / g_stAfe.stRamApp.fCELLVol[9] * g_stCfg.stAfe.afCellVolCali[9];
		  ucNeedSave = 1;
		}
		break;
	case 0xC8 + 10: 	//CellVolCali[10]
		if(usData){
	    g_stCfg.stAfe.afCellVolCali[10] = (float)usData / g_stAfe.stRamApp.fCELLVol[10] * g_stCfg.stAfe.afCellVolCali[10];
		  ucNeedSave = 1;
		}
		break;
	case 0xC8 + 11: 	//CellVolCali[11]
		if(usData){
	    g_stCfg.stAfe.afCellVolCali[11] = (float)usData / g_stAfe.stRamApp.fCELLVol[11] * g_stCfg.stAfe.afCellVolCali[11];
		  ucNeedSave = 1;
		}
		break;
	case 0xC8 + 12: 	//CellVolCali[12]
		if(usData){
	    g_stCfg.stAfe.afCellVolCali[12] = (float)usData / g_stAfe.stRamApp.fCELLVol[12] * g_stCfg.stAfe.afCellVolCali[12];
		  ucNeedSave = 1;
		}
		break;
	case 0xC8 + 13: 	//CellVolCali[13]
		if(usData){
	    g_stCfg.stAfe.afCellVolCali[13] = (float)usData / g_stAfe.stRamApp.fCELLVol[13] * g_stCfg.stAfe.afCellVolCali[13];
		  ucNeedSave = 1;
		}
		break;
	case 0xC8 + 14: 	//CellVolCali[14]
		if(usData){
	    g_stCfg.stAfe.afCellVolCali[14] = (float)usData / g_stAfe.stRamApp.fCELLVol[14] * g_stCfg.stAfe.afCellVolCali[14];
		  ucNeedSave = 1;
		}
		break;
	case 0xC8 + 15: 	//CellVolCali[15]
		if(usData){
	    g_stCfg.stAfe.afCellVolCali[15] = (float)usData / g_stAfe.stRamApp.fCELLVol[15] * g_stCfg.stAfe.afCellVolCali[15];
		  ucNeedSave = 1;
		}
		break;
	case 0x20 + CFG_CELL_NUM + 1:		//usBALActVolt
		g_stCfg.stLocal.usBALActVolt = usData;
		g_stCfg.stLocal.usBalanceValue = usData;
		ucNeedSave = 1;
		break;
	case 0x20 + CFG_CELL_NUM + 2:		//usBALActDVolt
		g_stCfg.stLocal.usBALActDVolt = usData;
		g_stCfg.stLocal.usBalanceDiff = usData;
		ucNeedSave = 1;
		break;
//case 0x20 + CFG_CELL_NUM + 3:		//usVALReactDVolt
//	g_stCfg.stLocal.usVALReactDVolt = usData;
//	ucNeedSave = 1;
//	break;
	case 0x20 + CFG_CELL_NUM + 3 :		//usRTSlpCnt
	case 0x20 + CFG_CELL_NUM + 4 :		//usRTSlpCnt
		g_stCfg.stLocal.iSreenSlpSec = usData * 60;
		ucNeedSave = 1;
		break;
	case 0x20 + CFG_CELL_NUM + 5:		//sStandbySlpCnt
		g_stCfg.stLocal.iStandbySlpSec = usData * 60;
		ucNeedSave = 1;
		break;
	case 0x20 + CFG_CELL_NUM + 6:		//usDbgPara
		g_stCfg.stLocal.usDbgPara = usData;
		ucNeedSave = 1;
		break;
	case 0x20 + CFG_CELL_NUM + 7:		//usHWSCTVThr
		if(g_stAfe.uRom.stCode.SCT != usData) {
			g_stAfe.uRom.stCode.SCV = usData;
			afe_set_rom();
		}
		break;
	case 0x20 + CFG_CELL_NUM + 8:		//usMOSForceOn
		//do nothing, useless
		break;
	case 0x20 + CFG_CELL_NUM + 9:		//usPackUVTVThr1
		g_stCfg.stLocal.usPackUVTVThr1 = usData;
		ucNeedSave = 1;
		break;
	case 0x20 + CFG_CELL_NUM + 10:		//usPackOVTVThr1
		g_stCfg.stLocal.usPackOVTVThr1 = usData;
		ucNeedSave = 1;
		break;
	case 0x20 + CFG_CELL_NUM + 11:		//usPackUVRVThr1
		g_stCfg.stLocal.usPackUVRVThr1 = usData;
		ucNeedSave = 1;
		break;
	case 0x20 + CFG_CELL_NUM + 12:		//usPackOVRVThr1
		g_stCfg.stLocal.usPackOVRVThr1 = usData;
		ucNeedSave = 1;
		break;
	case 0x20 + CFG_CELL_NUM + 13:		//sChgCurSnrCoeB
		g_stCfg.stLocal.sChgCurSnrCoeB = (int16_t)usData;
		ucNeedSave = 1;
		break;
	case 0x20 + CFG_CELL_NUM + 14:		//sDsgCurSnrCoeB
		g_stCfg.stLocal.sDsgCurSnrCoeB = (int16_t)usData;
		ucNeedSave = 1;
		break;
	case 0x20 + CFG_CELL_NUM + 15:		//sVoltSnrCoeB
		g_stCfg.stLocal.sVoltSnrCoeB = (int16_t)usData;
		ucNeedSave = 1;
		break;
	case 0x20 + CFG_CELL_NUM + 16:		//usReset
		g_stCfg.usGoRun = 1;
		cfg_save();
	  g_eBaseStat = eBStatReset_52;
		his_data_write();
		uint8_t aucData[2];
		for(uint8_t i=0;i<3;i++) {
			for(uint8_t j=1;j<g_stPrl.ucDevNum;j++) {
				eco_can_data_send(j, 1, 0x01, 0x20 + CFG_CELL_NUM + 16, aucData, 2);
				delay_1ms(10);
			}
		}
	  delay_1ms(500);
		System_Reset();
		break;
	case 0x20 + CFG_CELL_NUM + 17:		//usSetDefault
		if(usData == 52445) {//00 CC DD
			his_clear();
			g_eBaseStat = eBStatClearCache;
			his_data_write();
			break;
		} else if(usData == 52446) {//00 CC DE
			pstPackRVal->usCycle = g_stCfg.stLocal.usCycle;
			ucNeedSave = 1;
			break;
		} else if(usData == 56780) {//00 DD CC
			cfg_set_default();
			g_eBaseStat = eBStatDefault;
			his_data_write();
			break;
		} else {
			g_stCfg.stLocal.sBalanceOn = usData;
			ucNeedSave = 1;
			break;
		}
	case 0x20 + CFG_CELL_NUM + 18:		//ausRTC[0]
		calender.w_year = (usData >> 8) + 2000;
		calender.w_month = (usData & 0xFF);
		RTC_Set(calender.w_year, calender.w_month, calender.w_day, calender.hour, calender.min, calender.sec);
		break;
	case 0x20 + CFG_CELL_NUM + 19:		//ausRTC[1]
		calender.w_day = (usData >> 8);
		calender.hour = (usData & 0xFF);
		RTC_Set(calender.w_year, calender.w_month, calender.w_day, calender.hour, calender.min, calender.sec);
		break;
	case 0x20 + CFG_CELL_NUM + 20:		//ausRTC[2]
		calender.min = (usData >> 8);
		calender.sec = (usData & 0xFF);
		RTC_Set(calender.w_year, calender.w_month, calender.w_day, calender.hour, calender.min, calender.sec);
		break;
	case 0x20 + CFG_CELL_NUM + 21:		//sHeaterUTTVThr
		g_stCfg.stLocal.sHeaterUTTVThr = (int16_t)usData;
		ucNeedSave = 1;
		break;
	case 0x20 + CFG_CELL_NUM + 22:		//usCellOCTTTThr1
		g_stCfg.stLocal.usCellOCTTTThr1 = usData * 200;
		ucNeedSave = 1;
		break;
	case 0x20 + CFG_CELL_NUM + 23:		//usCellODTTTThr1
		g_stCfg.stLocal.usCellODTTTThr1 = usData * 200;
		ucNeedSave = 1;
		break;
	case 0x20 + CFG_CELL_NUM + 24:		//usMOSOTTTThr2
		g_stCfg.stLocal.usMOSOTTTThr2 = usData * 200;
		ucNeedSave = 1;
		break;
	case 0x20 + CFG_CELL_NUM + 25:		//usCellUCTTTThr1
		g_stCfg.stLocal.usCellUCTTTThr1 = usData * 200;
		ucNeedSave = 1;
		break;
	case 0x20 + CFG_CELL_NUM + 26:		//usCellUDTTTThr1
		g_stCfg.stLocal.usCellUDTTTThr1 = usData * 200;
		ucNeedSave = 1;
		break;
	case 0x20 + CFG_CELL_NUM + 27:		//usCellUVTTThr2
		g_stCfg.stLocal.usCellUVTTThr2 = usData * 200;
		ucNeedSave = 1;
		break;
	case 0x20 + CFG_CELL_NUM + 28:		//usCellOVTTThr1
		g_stCfg.stLocal.usCellOVTTThr1 = usData * 200;
		ucNeedSave = 1;
		break;
	case 0x20 + CFG_CELL_NUM + 29:		//usPackOVTTThr1
		g_stCfg.stLocal.usPackOVTTThr1 = usData * 200;
		ucNeedSave = 1;
		break;
	case 0x20 + CFG_CELL_NUM + 30:		//usPackUVTTThr2
		g_stCfg.stLocal.usPackUVTTThr2 = usData * 200;
		ucNeedSave = 1;
		break;
	case 0x20 + CFG_CELL_NUM + 31:		//usODCTTThr1
		g_stCfg.stLocal.usODCTTThr1 = usData * 200;
		ucNeedSave = 1;
		break;
	case 0x20 + CFG_CELL_NUM + 32:		//usOCCTTThr1
		g_stCfg.stLocal.usOCCTTThr1 = usData * 200;
		ucNeedSave = 1;
		break;
	case 0x20 + CFG_CELL_NUM + 33:		//usOCCRTThr1
		g_stCfg.stLocal.usOCCRTThr1 = usData * 200;
		ucNeedSave = 1;
		break;
	case 0x20 + CFG_CELL_NUM + 34:		//usODCRTThr1
		g_stCfg.stLocal.usODCRTThr1 = usData * 200;
		ucNeedSave = 1;
		break;
	case 0x20 + CFG_CELL_NUM + 35:		//usSCRTThr
		g_stCfg.stLocal.usSCRTThr = usData * 200;
		ucNeedSave = 1;
		break;
	case 0x20 + CFG_CELL_NUM + 36:		//usCurLmtMode
		g_stCfg.stLocal.usCurLmtMode = usData;
		ucNeedSave = 1;
		break;
	case 0x20 + CFG_CELL_NUM + 39:		//sLowSocSlpTVThr
		g_stCfg.stLocal.iLowPwrSlpSec = usData * 60;
		ucNeedSave = 1;
		break;
	case 0x20 + CFG_CELL_NUM + 40:		//ausSN[0]
		g_stCfg.stLocal.ausHWSN[0] = usData;
		ucNeedSave = 1;
		break;
	case 0x20 + CFG_CELL_NUM + 41:		//ausSN[1]
		g_stCfg.stLocal.ausHWSN[1] = usData;
		ucNeedSave = 1;
		break;
	case 0x20 + CFG_CELL_NUM + 42:		//ausSN[2]
		g_stCfg.stLocal.usSocCaliDelay = usData;
		g_stCfg.stLocal.ausHWSN[2] = usData;
		ucNeedSave = 1;
		break;
	case 0x20 + CFG_CELL_NUM + 43:		//ausSN[3]
		g_stCfg.stLocal.usSocCaliCurVal = usData;
		g_stCfg.stLocal.ausHWSN[3] = usData;
		ucNeedSave = 1;
		break;
	case 0x20 + CFG_CELL_NUM + 44:		//sHeaterOTRVThr
		g_stCfg.stLocal.sHeaterUTRVThr = usData;
		ucNeedSave = 1;
		break;
	case 0x20 + CFG_CELL_NUM + 45:		//usSocFCaliU0
		g_stCfg.stLocal.ausSocFCaliU[0] = usData;
		ucNeedSave = 1;
		break;
	case 0x20 + CFG_CELL_NUM + 46:		//usSocECaliU0
		g_stCfg.stLocal.ausSocECaliU[0] = usData;
		ucNeedSave = 1;
		break;
	case 0x20 + CFG_CELL_NUM + 47:		//usSocECaliU1
		g_stCfg.stLocal.ausSocECaliU[1] = usData;
		ucNeedSave = 1;
		break;
	case 0x20 + CFG_CELL_NUM + 48:		// usSocFCaliU1
		g_stCfg.stLocal.ausSocFCaliU[1] = usData;
		ucNeedSave = 1;
		break;
	case 0x20 + CFG_CELL_NUM + 49:		//usSocECaliU2
		g_stCfg.stLocal.ausSocECaliU[2] = usData;
		ucNeedSave = 1;
		break;
	case 0x20 + CFG_CELL_NUM + 50:		//usSleep
//	g_stLocalArrayRVal.eLocalStat = eLocalStatSlp;
		g_stCfg.usGoRun = 0;
		cfg_save();
//	  g_eBaseStat = eBStatSleep_01;
//		his_data_write();
//		System_Reset();uint8_t aucData[2] = {0};
		if(prl_host()) {
			uint8_t aucData[2];
			for(uint8_t i=0;i<3;i++) {
				for(uint8_t j=1;j<g_stPrl.ucDevNum;j++) {
					eco_can_data_send(j, 1, 0x01, 0x20 + CFG_CELL_NUM + 50, aucData, 2);
					delay_1ms(10);
				}
			}
		}
	  g_bNeedSleep = true;
		break;
	case 0x20 + CFG_CELL_NUM + 51:		//usPassword
		if(usData == g_usPassword) {
			g_usSrlCntTick = 300 * (1000 / g_stCfg.stLocal.usCyclePeriod);
		} else {
			g_usSrlCntTick = 0;
		}
		break;
	case 0x20 + CFG_CELL_NUM + 54:		//usClearHistory
    his_clear();
	  g_eBaseStat = eBStatClearCache;
		his_data_write();
	  break;
	case 0x20 + CFG_CELL_NUM + 55:		//usSetVolCali
    for(uint8_t i=0;i<CFG_CELL_NUM;i++) {
			g_stCfg.stAfe.afCellVolCali[i] = 1;
		}
		ucNeedSave = 1;
	  break;
	case 0x20 + CFG_CELL_NUM + 56:		//usSetCurCali
    g_stCfg.stAfe.fCDATACaliA_CHG = 1;
    g_stCfg.stAfe.fCDATACaliA_DSG = 1;
		g_stCfg_BackUp.fCDATACaliA_CHG_BackUp = 1;
		g_stCfg_BackUp.fCDATACaliA_DSG_BackUp = 1;
	  afe_get_ram();
	  g_stCfg.stAfe.fCDATACaliB -= g_stAfe.stRamApp.fCDATA;
	  ucNeedSave = 1;
		ucBackUpNeedSave = 1;
	  break;
	case 0x20 + CFG_CELL_NUM + 57:		//usBalanceOn 
    g_stCfg.stLocal.sBalanceOn = usData;
	  ucNeedSave = 1;
	  break;
	case 0x20 + CFG_CELL_NUM + 58:		//usBalanceValue
    g_stCfg.stLocal.usBalanceValue = usData;
		g_stCfg.stLocal.usBALActVolt = usData;
	  ucNeedSave = 1;
	  break;
	case 0x20 + CFG_CELL_NUM + 59:		//usBalanceDiff
    g_stCfg.stLocal.usBalanceDiff = usData;
		g_stCfg.stLocal.usBALActDVolt = usData;
	  ucNeedSave = 1;
	  break;
	case 0x20 + CFG_CELL_NUM + 60:		//usSocCaliDelay
    g_stCfg.stLocal.usSocCaliDelay = usData;
	  ucNeedSave = 1;
	  break;
	case 0x20 + CFG_CELL_NUM + 61:		//usSocCaliCurVal
    g_stCfg.stLocal.usSocCaliCurVal = usData;
	  ucNeedSave = 1;
	  break;
	case 0x20 + CFG_CELL_NUM + 70:		//usSocCaliCurVal 
//    g_stCfg.stCom.ucCanPtcType = usData;
		if(usData == 0x11){
			g_stCfg.stCom.ucCanPtcType = 1;
			g_stCfg.stCom.uiCanBaud = 250;
			g_stCfg.stLocal.ucSVerAgree = 0x41;
			CAN0_Init(g_stCfg.stCom.uiCanBaud);
		} else if(usData == 0x22) {
			g_stCfg.stCom.ucCanPtcType = 2;
			g_stCfg.stCom.uiCanBaud = 250;
			g_stCfg.stLocal.ucSVerAgree = 0x42;
			CAN0_Init(g_stCfg.stCom.uiCanBaud);
		} else if(usData == 0x33) {
			g_stCfg.stCom.ucCanPtcType = 3;
			g_stCfg.stCom.uiCanBaud = 500;
			g_stCfg.stLocal.ucSVerAgree = 0x43;
			CAN0_Init(g_stCfg.stCom.uiCanBaud);
		} else if(usData == 0x44) {
			g_stCfg.stCom.ucCanPtcType = 4;
			g_stCfg.stCom.uiCanBaud = 500;
			g_stCfg.stLocal.ucSVerAgree = 0x44;
			CAN0_Init(g_stCfg.stCom.uiCanBaud);
		} else if(usData == 0x55) {
			g_stCfg.stCom.ucCanPtcType = 5;
			g_stCfg.stCom.uiCanBaud = 250;
			g_stCfg.stLocal.ucSVerAgree = 0x45;
			CAN0_Init(g_stCfg.stCom.uiCanBaud);
		} else if(usData == 0x66) {
			g_stCfg.stCom.ucCanPtcType = 6;
			g_stCfg.stCom.uiCanBaud = 250;
			g_stCfg.stLocal.ucSVerAgree = 0x46;
			CAN0_Init(g_stCfg.stCom.uiCanBaud);
		} else if(usData == 0x77) {
			g_stCfg.stCom.ucCanPtcType = 7;
			g_stCfg.stCom.uiCanBaud = 250;
			g_stCfg.stLocal.ucSVerAgree = 0x47;
			CAN0_Init(g_stCfg.stCom.uiCanBaud);
		}
	  ucNeedSave = 1;
	  break;
	default:
		return;
	}
	if(ucNeedSave != 0) {
		cfg_save();
	}
	if(ucBackUpNeedSave != 0) {
		MEM_FlashWrite(eCfgBackUpStart, (uint8_t*)&g_stCfg_BackUp, sizeof(CFG_BackUp));
	}
	if(ucNeedHisSave != 0) {
		his_data_write();
	}
}

uint16_t eco_get_log(uint8_t ucLogIdx, uint8_t* pucData) {
	if(pucData == 0) {
		return 0;
	}
	HIS_LOG_S stHisLog = {0};
	if(!his_log_read(ucLogIdx, &stHisLog)) {
		return 0;
	}
	pucData[0] = 0xAA;
	pucData[1] = 0x55;
	pucData[2] = 15 + CFG_CELL_NUM * 2 + 8;
	stHisLog.dt += 8 * 3600;
	struct tm* t = localtime(&stHisLog.dt);
	t = localtime(&stHisLog.dt);
	pucData[3] = t->tm_year - 100;
	pucData[4] = t->tm_mon + 1;
	pucData[5] = t->tm_mday;
	pucData[6] = t->tm_hour;
	pucData[7] = t->tm_min;
	pucData[8] = t->tm_sec;
	memcpy(pucData + 9, (uint8_t*)&stHisLog.stEcoLog, 9);
	for(uint8_t i=0;i<CFG_CELL_NUM;i++) {
		pucData[18 + i * 2] = (stHisLog.stEcoLog.ausCell[i] >> 8);
		pucData[18 + i * 2 + 1] = (stHisLog.stEcoLog.ausCell[i] & 0xFF);
	}
	for(uint8_t i=0;i<4;i++) {
		pucData[18 + CFG_CELL_NUM * 2 + i * 2] = ((uint16_t)(stHisLog.stEcoLog.asTemp[i]) >> 8);
		pucData[18 + CFG_CELL_NUM * 2 + i * 2 + 1] = ((uint16_t)(stHisLog.stEcoLog.asTemp[i]) & 0xFF);
	}
	if(g_bGbms3 == true) {
		pucData[26 + CFG_CELL_NUM * 2] = stHisLog.stEcoLog.ausSN[0];
		pucData[27 + CFG_CELL_NUM * 2] = stHisLog.stEcoLog.ausSN[1];
		pucData[28 + CFG_CELL_NUM * 2] = stHisLog.stEcoLog.ausSN[2];
		pucData[29 + CFG_CELL_NUM * 2] = stHisLog.stEcoLog.ausSN[3];
		pucData[30 + CFG_CELL_NUM * 2] = stHisLog.stEcoLog.ausSN[4];
		pucData[31 + CFG_CELL_NUM * 2] = stHisLog.stEcoLog.ausSN[5];
		pucData[32 + CFG_CELL_NUM * 2] = stHisLog.stEcoLog.ausSN[6];
		pucData[33 + CFG_CELL_NUM * 2] = stHisLog.stEcoLog.ausSN[7];
		uint16_t usCRC = eco_crc16(g_aucCanSBuf, 34 + CFG_CELL_NUM * 2, 0xFFFF, 0xA001);
		pucData[34 + CFG_CELL_NUM * 2] = (usCRC & 0xFF);
		pucData[35+ CFG_CELL_NUM * 2] = (usCRC >> 8);
		return 36 + CFG_CELL_NUM * 2;
	} else {
		uint16_t usCRC = eco_crc16(g_aucCanSBuf, 26 + CFG_CELL_NUM * 2, 0xFFFF, 0xA001);//26 34
		pucData[26 + CFG_CELL_NUM * 2] = (usCRC & 0xFF);
		pucData[27+ CFG_CELL_NUM * 2] = (usCRC >> 8);
		return 28 + CFG_CELL_NUM * 2;
	}
}

uint8_t eco_get_cd(uint8_t* pucData) {
	pucData[0] = 0xAA;		//header
	pucData[1] = 0x55;		//header
	pucData[2] = 0x1C;		//data length
	pucData[3] = calender.w_year - 2000;
	pucData[4] = calender.w_month;
	pucData[5] = calender.w_day;
	pucData[6] = calender.hour;
	pucData[7] = calender.min;
	pucData[8] = calender.sec;
	memcpy(pucData + 9, &g_stEcoCD, sizeof(ECO_CD_S));
	uint16_t usCRC = eco_crc16(pucData, 31, 0xFFFF, 0xA001);
	pucData[31] = usCRC & 0xFF;
	pucData[32] = (usCRC >> 8) & 0xFF;
	return 33;
}

void eco_data_proc(uint8_t ucSelfId, uint8_t ucDevAddr, uint8_t ucFunCode, uint16_t usRegAddr, uint8_t* pucData, uint16_t usLen) {
	if(pucData == NULL) {
		return;
	}
	if(g_usSrlCntTick == 0 && ((ucFunCode != 0x02 && ucFunCode != 0x03) || (usRegAddr != 0x04B0 && usRegAddr != 0x044C))) {
		return;
	}
//	uint16_t usVal;
		int16_t usVal;
//	int32_t iVal;
	switch(ucFunCode) {
	case 0x01:		//Write register
		if(ucSelfId != g_stPrl.ucSelfId) {
			return;
		}
		if(usLen < 2) {
			return;
		}
		if(usRegAddr > (0x20 + CFG_CELL_NUM + 70) && usRegAddr < 0xC6 && usRegAddr > 0xD7) {
			return;
		} 
	//for(uint16_t i=0;i<usLen;i++) {
		eco_write_reg(usRegAddr, pucData[0] << 8 | pucData[1]);
	//}
		eco_can_data_send(ucSelfId, ucDevAddr, ucFunCode, usRegAddr, g_aucCanSBuf, 0);
		break;
	case 0x02:		//Read u8 register address info
	case 0x03:		//Read u16 register address info
		if(ucSelfId != g_stPrl.ucSelfId) {
			return;
		}
		if(usLen > 256) { /* Byte length determination */
			return;
		}
		if(usRegAddr == 0x73) {
			return;
		}
		if(usRegAddr == 0xC6 || usRegAddr == 0xC7 || usRegAddr == 0xC8) {
			memset(g_aucCanSBuf, 0, usLen * sizeof(uint16_t));
			if(usRegAddr == 0xC6){
//				usVal = g_stAfe.stRamApp.fCDATA * (g_stCfg.stAfe.fCDATACaliA_CHG - 1) / 1000;
				usVal = g_stCfg.stAfe.fCDATACaliA_CHG * 10000;
				g_aucCanSBuf[0] = usVal >> 8;
				g_aucCanSBuf[1] = usVal & 0xFF;
			}	
      if(usRegAddr == 0xC7){
//				usVal = g_stAfe.stRamApp.fCDATA * (g_stCfg.stAfe.fCDATACaliA_DSG - 1) / 1000;
				usVal = g_stCfg.stAfe.fCDATACaliA_DSG * 10000;
				g_aucCanSBuf[0] = usVal >> 8;
				g_aucCanSBuf[1] = usVal & 0xFF;
			}					
			if(usRegAddr == 0xC8){
				for(uint16_t i=0;i<CFG_CELL_NUM;i++) {
				usVal = g_stAfe.stRamApp.fCELLVol[i] * (g_stCfg.stAfe.afCellVolCali[i] - 1);
				g_aucCanSBuf[i * 2] = usVal >> 8;
				g_aucCanSBuf[i * 2 + 1] = usVal & 0xFF;
				}
		  }
			eco_can_data_send(ucSelfId, ucDevAddr, ucFunCode, usRegAddr, g_aucCanSBuf, usLen * sizeof(uint16_t));
		} else {
			for(uint16_t i=0;i<usLen;i++) {
				usVal = eco_read_reg(ucSelfId, ucDevAddr, usRegAddr + i);
				g_aucCanSBuf[i * 2] = usVal >> 8;
				g_aucCanSBuf[i * 2 + 1] = usVal & 0xFF;
			}
			eco_can_data_send(ucSelfId, ucDevAddr, ucFunCode, usRegAddr, g_aucCanSBuf, usLen * sizeof(uint16_t));
	 }
		break;
	case 0x04:		//write u16 register address info
		if(ucSelfId != g_stPrl.ucSelfId) {
			return;
		}
//		if(usRegAddr != 0x044C || usLen > 19) {
//			return;
//		}
		if(usRegAddr == 0x044C && usLen < 20){
		  memcpy(g_stCfg.stLocal.aucBleName, pucData, usLen);
		  memset(g_stCfg.stLocal.aucBleName + usLen, 0, 20 - usLen);
		  cfg_save();
			memcpy(g_stCfg_BackUp.aucBleName_BackUp, pucData, usLen);
		  memset(g_stCfg_BackUp.aucBleName_BackUp + usLen, 0, 20 - usLen);
			MEM_FlashWrite(eCfgBackUpStart, (uint8_t*)&g_stCfg_BackUp, sizeof(CFG_BackUp));
			char acBle[64];
			sprintf(acBle, "AT+NAME=%s\r\n", g_stCfg.stLocal.aucBleName);
			UART_Send((uint8_t*)acBle, strlen(acBle));
			sprintf(acBle, "AT+REBOOT=1\r\n");
			UART_Send((uint8_t*)acBle, strlen(acBle));
		}
		if(usRegAddr == 0x0480 && usLen < 26){
		  memcpy(g_stCfg.stLocal.aucBMS_ID, pucData, usLen);
		  memset(g_stCfg.stLocal.aucBMS_ID + usLen, 0, 25 - usLen);
		  cfg_save();
			memcpy(g_stCfg_BackUp.aucBMS_ID_BackUp, pucData, usLen);
		  memset(g_stCfg_BackUp.aucBMS_ID_BackUp + usLen, 0, 25 - usLen);
		  MEM_FlashWrite(eCfgBackUpStart, (uint8_t*)&g_stCfg_BackUp, sizeof(CFG_BackUp));
		}
		break;
	case 0x09:
		if(ucSelfId != g_stPrl.ucSelfId) {
			return;
		}
		if(usLen < 2) {
			return;
		}
		if(usRegAddr == 3) {		//charge MOS force on enable
			if(pucData[1] == 1){
				g_stLocalArrayRVal.astPackRVal[g_stPrl.ucSelfId].ucChgForceEn = 0x55;
				MCU_CHG_ON;
			}else{
				g_stLocalArrayRVal.astPackRVal[g_stPrl.ucSelfId].ucChgForceEn = 0;
				MCU_CHG_OFF;
			}
		} else if(usRegAddr == 4) {	//charge MOS force on disable
			       if(pucData[1] == 1){
			         g_stLocalArrayRVal.astPackRVal[g_stPrl.ucSelfId].ucChgForceEn = 0xAA;
			         MCU_CHG_OFF;
						 }else{
							 g_stLocalArrayRVal.astPackRVal[g_stPrl.ucSelfId].ucChgForceEn = 0;
			         MCU_CHG_OFF;
						 }
		} else if(usRegAddr == 5) {	//discharge MOS force on enable
						 if(pucData[1] == 1){
							 g_stLocalArrayRVal.astPackRVal[g_stPrl.ucSelfId].ucDsgForceEn = 0x55;
							 MCU_DSG_ON;
							}else{
							 g_stLocalArrayRVal.astPackRVal[g_stPrl.ucSelfId].ucDsgForceEn = 0;
							 MCU_DSG_OFF;
							}
		} else if(usRegAddr == 6) {	//discharge MOS force on disable
						if(pucData[1] == 1){
							g_stLocalArrayRVal.astPackRVal[g_stPrl.ucSelfId].ucDsgForceEn = 0xAA;
							MCU_DSG_OFF;
							}else{
							g_stLocalArrayRVal.astPackRVal[g_stPrl.ucSelfId].ucDsgForceEn = 0;
							MCU_DSG_OFF;
							}
		} else if(usRegAddr == 7) {	//heat MOS force on enable
			g_stLocalArrayRVal.astPackRVal[g_stPrl.ucSelfId].ucHeatForceEn = 1;
		} else if(usRegAddr == 8) {	//heat MOS force on disable
			g_stLocalArrayRVal.astPackRVal[g_stPrl.ucSelfId].ucHeatForceEn = 0;
		}
		break;
	case 0x0A:		//Read history log
		if(ucSelfId != g_stPrl.ucSelfId) {
			return;
		}
		g_aucCanSBuf[0] = 0x00;
		g_aucCanSBuf[1] = 0x01;
		g_aucCanSBuf[2] = 0x02;
		g_aucCanSBuf[3] = 0x03;
		g_aucCanSBuf[4] = 0x04;
		g_aucCanSBuf[5] = 0x05;
	  g_aucCanSBuf[6] = 0x00;
		eco_can_data_send(ucSelfId, ucDevAddr, ucFunCode, usRegAddr, g_aucCanSBuf, 6);
		for(uint16_t i=0;i<500;i++) {//1000
			uint16_t usLen = eco_get_log(i, g_aucCanSBuf);
			if(usLen == 0) {
				break;
			}
			eco_can_data_send(ucSelfId, ucDevAddr, ucFunCode, usRegAddr, g_aucCanSBuf, usLen);
			WDG_DONE_H;
			WDG_DONE_L;
			gd32_wdgt_feed_dog(EWdgType_fwdg);	//feed dog
		}
		g_aucCanSBuf[0] = 0x01;
		g_aucCanSBuf[1] = 0x02;
		g_aucCanSBuf[2] = 0x03;
		g_aucCanSBuf[3] = 0x04;
		g_aucCanSBuf[4] = 0x05;
		g_aucCanSBuf[5] = 0x06;
		g_aucCanSBuf[6] = 0x00;
		eco_can_data_send(ucSelfId, ucDevAddr, ucFunCode, usRegAddr, g_aucCanSBuf, 6);
		break;
	case 0x0B:		//Read history data
		if(ucSelfId != g_stPrl.ucSelfId) {
			return;
		}
		g_aucCanSBuf[0] = 0x00;
		g_aucCanSBuf[1] = 0x00;
		g_aucCanSBuf[2] = 0x01;
		g_aucCanSBuf[3] = 0x02;
		g_aucCanSBuf[4] = 0x03;
		g_aucCanSBuf[5] = 0x04;
	  g_aucCanSBuf[6] = 0x05;
		eco_can_data_send(ucSelfId, ucDevAddr, ucFunCode, usRegAddr, g_aucCanSBuf, 7);
		for(uint16_t i=0;i<1000;i++) {//64
			if(!his_data_read(i, (HIS_DATA_S*)g_aucCanSBuf)) {
				break;
			}
			eco_can_data_send(ucSelfId, ucDevAddr, ucFunCode, usRegAddr, g_aucCanSBuf, sizeof(HIS_DATA_S));
			WDG_DONE_H;
			WDG_DONE_L;
			gd32_wdgt_feed_dog(EWdgType_fwdg);	//feed dog
		}
		g_aucCanSBuf[0] = 0x00;
		g_aucCanSBuf[1] = 0x01;
		g_aucCanSBuf[2] = 0x02;
		g_aucCanSBuf[3] = 0x03;
		g_aucCanSBuf[4] = 0x04;
		g_aucCanSBuf[5] = 0x05;
		g_aucCanSBuf[6] = 0x06;
		eco_can_data_send(ucSelfId, ucDevAddr, ucFunCode, usRegAddr, g_aucCanSBuf, 7);
		break;
	case 0x0C:		//Read charge & discharge info
		if(ucSelfId != g_stPrl.ucSelfId) {
			return;
		}
		if(pucData[5] == 0x02) {
			g_aucCanSBuf[0] = 0x00;
			g_aucCanSBuf[1] = 0x01;
			g_aucCanSBuf[2] = 0x02;
			g_aucCanSBuf[3] = 0x03;
			g_aucCanSBuf[4] = 0x04;
			g_aucCanSBuf[5] = 0x05;
			g_aucCanSBuf[6] = 0x00;
			eco_can_data_send(ucSelfId, ucDevAddr, ucFunCode, usRegAddr, g_aucCanSBuf, 6);
			for(uint16_t i=0;i<5000;i++) {
				delay_1ms(10);
				HIS_DATA_S stData = {0};
				if(!his_data_read(i, &stData)) {
					break;
				}
				g_aucCanSBuf[0] = 0xAA;
				g_aucCanSBuf[1] = 0x55;
				g_aucCanSBuf[2] = 0x27;
				stData.dt += 8 * 3600;
				struct tm* t = localtime(&stData.dt);
				g_aucCanSBuf[3] = t->tm_year - 100;
				g_aucCanSBuf[4] = t->tm_mon + 1;
				g_aucCanSBuf[5] = t->tm_mday;
				g_aucCanSBuf[6] = t->tm_hour;
				g_aucCanSBuf[7] = t->tm_min;
				g_aucCanSBuf[8] = t->tm_sec;
				g_aucCanSBuf[9] = stData.state;
				g_aucCanSBuf[10] = *((uint8_t*)&(stData.stRtv) + 16);
				g_aucCanSBuf[11] = *((uint8_t*)&(stData.stRtv) + 17);
				g_aucCanSBuf[12] = *((uint8_t*)&(stData.stRtv) + 18);
				g_aucCanSBuf[13] = *((uint8_t*)&(stData.stRtv) + 19);
				g_aucCanSBuf[14] = stData.stRtv.usPackRTSoc;
				g_aucCanSBuf[15] = stData.stRtv.usPackRTVolt / 256;
				g_aucCanSBuf[16] = (stData.stRtv.usPackRTVolt & 0xFF);
				g_aucCanSBuf[17] = (int16_t)stData.stRtv.sPackRTCur >> 8;
				g_aucCanSBuf[18] = (int16_t)stData.stRtv.sPackRTCur & 0xFF;
				g_aucCanSBuf[19] = (int16_t)stData.stRtv.sPackRTCur >> 8;
				g_aucCanSBuf[20] = (int16_t)stData.stRtv.sPackRTCur & 0xFF;
				uint16_t usPackVolt = 0;
				uint16_t usMaxVal = stData.stRtv.ausCellRTVolt[0];
				uint16_t usMinVal = stData.stRtv.ausCellRTVolt[0];
				for(uint8_t j=0;j<CFG_CELL_NUM;j++) {
					usPackVolt += stData.stRtv.ausCellRTVolt[j];
					if(usMaxVal < stData.stRtv.ausCellRTVolt[j]) {
						usMaxVal = stData.stRtv.ausCellRTVolt[j];
					}
					if(usMinVal > stData.stRtv.ausCellRTVolt[j]) {
						usMinVal = stData.stRtv.ausCellRTVolt[j];
					}
				}
				g_aucCanSBuf[21] = usMaxVal / 256;
				g_aucCanSBuf[22] = (usMaxVal & 0xFF);
				g_aucCanSBuf[23] = usMinVal / 256;
				g_aucCanSBuf[24] = (usMinVal & 0xFF);
				int16_t asTemp[3] = {stData.stRtv.sCellRTT1, stData.stRtv.sCellRTT2, stData.stRtv.sCellRTT3};
				int16_t sMaxVal = asTemp[0];
				int16_t sMinVal = asTemp[0];
				for(uint8_t j=1;j<3;j++) {
					if(sMaxVal < asTemp[j]) {
						sMaxVal = asTemp[j];
					}
					if(sMinVal > asTemp[j]) {
						sMinVal = asTemp[j];
					}
				}
				g_aucCanSBuf[25] = sMaxVal / 256;
				g_aucCanSBuf[26] = (sMaxVal & 0xFF);
				g_aucCanSBuf[27] = sMinVal / 256;
				g_aucCanSBuf[28] = (sMinVal & 0xFF);
				g_aucCanSBuf[29] = stData.stRtv.sMOSRTT1 / 256;
				g_aucCanSBuf[30] = (stData.stRtv.sMOSRTT1 & 0xFF);
				g_aucCanSBuf[31] = stData.stRtv.ausSN[0];
				g_aucCanSBuf[32] = stData.stRtv.ausSN[1];
				g_aucCanSBuf[33] = stData.stRtv.ausSN[2];
				g_aucCanSBuf[34] = stData.stRtv.ausSN[3];
				g_aucCanSBuf[35] = g_stCfg.stLocal.ucSVerAgree - 0x40;//stData.stRtv.ausSN[4]
				g_aucCanSBuf[36] = 3;//stData.stRtv.ausSN[5];
				g_aucCanSBuf[37] = stData.stRtv.ausSN[6];
				g_aucCanSBuf[38] = stData.stRtv.ausSN[7];
				g_aucCanSBuf[39] = stData.stRtv.sCellRTT1 / 256;
				g_aucCanSBuf[40] = (stData.stRtv.sCellRTT1 & 0xFF);
				g_aucCanSBuf[41] = stData.stRtv.sCellRTT2 / 256;
				g_aucCanSBuf[42] = (stData.stRtv.sCellRTT2 & 0xFF);
				g_aucCanSBuf[43] = stData.stRtv.sCellRTT3 / 256;
				g_aucCanSBuf[44] = (stData.stRtv.sCellRTT3 & 0xFF);
				g_aucCanSBuf[45] = stData.stRtv.usTempIsNotRising;
				g_aucCanSBuf[46] = stData.stRtv.usHeatingTimeout;
				g_aucCanSBuf[47] = stData.stRtv.usHeatingOverNumber;
				uint16_t usCRC = eco_crc16(g_aucCanSBuf, 48, 0xFFFF, 0xA001);
				g_aucCanSBuf[49] = (usCRC & 0xFF);
				g_aucCanSBuf[50] = (usCRC >> 8);
				eco_can_data_send(ucSelfId, ucDevAddr, ucFunCode, usRegAddr, g_aucCanSBuf, 51);
				WDG_DONE_H;
				WDG_DONE_L;
			}
			g_aucCanSBuf[0] = 0x01;
			g_aucCanSBuf[1] = 0x02;
			g_aucCanSBuf[2] = 0x03;
			g_aucCanSBuf[3] = 0x04;
			g_aucCanSBuf[4] = 0x05;
			g_aucCanSBuf[5] = 0x06;
			g_aucCanSBuf[6] = 0x00;
			eco_can_data_send(ucSelfId, ucDevAddr, ucFunCode, usRegAddr, g_aucCanSBuf, 6);
		} else {
			g_aucCanSBuf[0] = 0x00;
			g_aucCanSBuf[1] = 0x01;
			g_aucCanSBuf[2] = 0x02;
			g_aucCanSBuf[3] = 0x03;
			g_aucCanSBuf[4] = 0x04;
			g_aucCanSBuf[5] = 0x05;
			g_aucCanSBuf[6] = 0x00;
			eco_can_data_send(ucSelfId, ucDevAddr, ucFunCode, usRegAddr, g_aucCanSBuf, 6);
			for(uint16_t i=0;i<500;i++) {//1000
				delay_1ms(10);
				HIS_DATA_S stData = {0};
				if(!his_data_read(i, &stData)) {
					break;
				}
				g_aucCanSBuf[0] = 0xAA;
				g_aucCanSBuf[1] = 0x55;
				g_aucCanSBuf[2] = 0x1C;//0x24
				stData.dt += 8 * 3600;
				struct tm* t = localtime(&stData.dt);
				g_aucCanSBuf[3] = t->tm_year - 100;
				g_aucCanSBuf[4] = t->tm_mon + 1;
				g_aucCanSBuf[5] = t->tm_mday;
				g_aucCanSBuf[6] = t->tm_hour;
				g_aucCanSBuf[7] = t->tm_min;
				g_aucCanSBuf[8] = t->tm_sec;
				g_aucCanSBuf[9] = stData.state;
				g_aucCanSBuf[10] = *((uint8_t*)&(stData.stRtv) + 16);
				g_aucCanSBuf[11] = *((uint8_t*)&(stData.stRtv) + 17);
				g_aucCanSBuf[12] = *((uint8_t*)&(stData.stRtv) + 18);
				g_aucCanSBuf[13] = *((uint8_t*)&(stData.stRtv) + 19);
				g_aucCanSBuf[14] = stData.stRtv.usPackRTSoc;
				g_aucCanSBuf[15] = stData.stRtv.usPackRTVolt / 256;
				g_aucCanSBuf[16] = (stData.stRtv.usPackRTVolt & 0xFF);
				g_aucCanSBuf[17] = (int16_t)stData.stRtv.sPackRTCur >> 8;
				g_aucCanSBuf[18] = (int16_t)stData.stRtv.sPackRTCur & 0xFF;
				g_aucCanSBuf[19] = (int16_t)stData.stRtv.sPackRTCur >> 8;
				g_aucCanSBuf[20] = (int16_t)stData.stRtv.sPackRTCur & 0xFF;
				uint16_t usPackVolt = 0;
				uint16_t usMaxVal = stData.stRtv.ausCellRTVolt[0];
				uint16_t usMinVal = stData.stRtv.ausCellRTVolt[0];
				for(uint8_t j=0;j<CFG_CELL_NUM;j++) {
					usPackVolt += stData.stRtv.ausCellRTVolt[j];
					if(usMaxVal < stData.stRtv.ausCellRTVolt[j]) {
						usMaxVal = stData.stRtv.ausCellRTVolt[j];
					}
					if(usMinVal > stData.stRtv.ausCellRTVolt[j]) {
						usMinVal = stData.stRtv.ausCellRTVolt[j];
					}
				}
				g_aucCanSBuf[21] = usMaxVal / 256;
				g_aucCanSBuf[22] = (usMaxVal & 0xFF);
				g_aucCanSBuf[23] = usMinVal / 256;
				g_aucCanSBuf[24] = (usMinVal & 0xFF);
				int16_t asTemp[3] = {stData.stRtv.sCellRTT1, stData.stRtv.sCellRTT2, stData.stRtv.sCellRTT3};
				int16_t sMaxVal = asTemp[0];
				int16_t sMinVal = asTemp[0];
				for(uint8_t j=1;j<3;j++) {
					if(sMaxVal < asTemp[j]) {
						sMaxVal = asTemp[j];
					}
					if(sMinVal > asTemp[j]) {
						sMinVal = asTemp[j];
					}
				}
				g_aucCanSBuf[25] = sMaxVal / 256;
				g_aucCanSBuf[26] = (sMaxVal & 0xFF);
				g_aucCanSBuf[27] = sMinVal / 256;
				g_aucCanSBuf[28] = (sMinVal & 0xFF);
				g_aucCanSBuf[29] = stData.stRtv.sMOSRTT1 / 256;
				g_aucCanSBuf[30] = (stData.stRtv.sMOSRTT1 & 0xFF);
				if(g_bGbms3 == true) {
					g_aucCanSBuf[31] = stData.stRtv.ausSN[0];
					g_aucCanSBuf[32] = stData.stRtv.ausSN[1];
					g_aucCanSBuf[33] = stData.stRtv.ausSN[2];
					g_aucCanSBuf[34] = stData.stRtv.ausSN[3];
					g_aucCanSBuf[35] = g_stCfg.stLocal.ucSVerAgree - 0x40;//stData.stRtv.ausSN[4]
					g_aucCanSBuf[36] = 3;//stData.stRtv.ausSN[5];
					g_aucCanSBuf[37] = stData.stRtv.ausSN[6];
					g_aucCanSBuf[38] = stData.stRtv.ausSN[7];
					g_aucCanSBuf[39] = stData.stRtv.sCellRTT1 / 256;
					g_aucCanSBuf[40] = (stData.stRtv.sCellRTT1 & 0xFF);
					g_aucCanSBuf[41] = stData.stRtv.sCellRTT2 / 256;
					g_aucCanSBuf[42] = (stData.stRtv.sCellRTT2 & 0xFF);
					g_aucCanSBuf[43] = stData.stRtv.sCellRTT3 / 256;
					g_aucCanSBuf[44] = (stData.stRtv.sCellRTT3 & 0xFF);
					g_aucCanSBuf[45] = stData.stRtv.usTempIsNotRising;
					g_aucCanSBuf[46] = stData.stRtv.usHeatingTimeout;
					g_aucCanSBuf[47] = stData.stRtv.usHeatingOverNumber;
					uint16_t usCRC = eco_crc16(g_aucCanSBuf, 48, 0xFFFF, 0xA001);
					g_aucCanSBuf[49] = (usCRC & 0xFF);
					g_aucCanSBuf[50] = (usCRC >> 8);
					eco_can_data_send(ucSelfId, ucDevAddr, ucFunCode, usRegAddr, g_aucCanSBuf, 51);
				} else {
					uint16_t usCRC = eco_crc16(g_aucCanSBuf, 31, 0xFFFF, 0xA001);
					g_aucCanSBuf[31] = (usCRC & 0xFF);
					g_aucCanSBuf[32] = (usCRC >> 8);
					eco_can_data_send(ucSelfId, ucDevAddr, ucFunCode, usRegAddr, g_aucCanSBuf, 33);
				}
				WDG_DONE_H;
				WDG_DONE_L;
				gd32_wdgt_feed_dog(EWdgType_fwdg);	//feed dog
			}
			g_aucCanSBuf[0] = 0x01;
			g_aucCanSBuf[1] = 0x02;
			g_aucCanSBuf[2] = 0x03;
			g_aucCanSBuf[3] = 0x04;
			g_aucCanSBuf[4] = 0x05;
			g_aucCanSBuf[5] = 0x06;
			g_aucCanSBuf[6] = 0x00;
			eco_can_data_send(ucSelfId, ucDevAddr, ucFunCode, usRegAddr, g_aucCanSBuf, 6);
		}
	break;
	case 0x0D:		//Read wave
		if(ucSelfId != g_stPrl.ucSelfId) {
			return;
		}
		if(pucData[0] == 0 || pucData[0] > 8) {
			g_bWriteWave = true;
			return;
		}
		g_aucCanSBuf[0] = 0x00;
		g_aucCanSBuf[1] = 0x00;
		g_aucCanSBuf[2] = 0x01;
		g_aucCanSBuf[3] = 0x02;
		g_aucCanSBuf[4] = 0x03;
		g_aucCanSBuf[5] = 0x04;
		g_aucCanSBuf[6] = 0x05;
		eco_can_data_send(ucSelfId, ucDevAddr, ucFunCode, usRegAddr, g_aucCanSBuf, 7);
		usVal = 0;	//use as index of piece to be sent
//		while(1) {
			his_wave_read(pucData[0] - 1, usVal, g_aucCanSBuf);
			usVal++;
			if(sizeof(HIS_WAVE_S) <= 512 * usVal) {
				eco_can_data_send(ucSelfId, ucDevAddr, ucFunCode, usRegAddr, g_aucCanSBuf, sizeof(HIS_WAVE_S) - 512 * (usVal - 1));
				break;
			} else {
				eco_can_data_send(ucSelfId, ucDevAddr, ucFunCode, usRegAddr, g_aucCanSBuf, 512);
			}
//		}
		g_aucCanSBuf[0] = 0x00;
		g_aucCanSBuf[1] = 0x01;
		g_aucCanSBuf[2] = 0x02;
		g_aucCanSBuf[3] = 0x03;
		g_aucCanSBuf[4] = 0x04;
		g_aucCanSBuf[5] = 0x05;
		g_aucCanSBuf[6] = 0x06;
		eco_can_data_send(ucSelfId, ucDevAddr, ucFunCode, usRegAddr, g_aucCanSBuf, 7);
		break;
//	case 0x0E:		//parallel consult
//		if(g_stPrl.ucDevNum >= PRL_MAX_NODE_NUM) {
//			return;
//		}
//		for(uint8_t i=0;i<g_stPrl.ucDevNum;i++) {
//			if(!memcmp(g_aucCanSBuf, g_stPrl.auiDevList, 7)) {
//				return;
//			}
//		}
//		g_aucCanSBuf[7] = 0;
//		iVal = atoi((const char*)g_aucCanSBuf);
//		g_stPrl.auiDevList[g_stPrl.ucDevNum] = iVal;
//		g_stPrl.ucDevNum++;
//		usVal = 0;
//		for(uint8_t i=1;i<g_stPrl.ucDevNum;i++) {
//			if(g_stPrl.auiDevList[0] < g_stPrl.auiDevList[i]) {
//				usVal++;
//			}
//		}
//		g_stPrl.ucSelfId = usVal;
//		break;
	case 0x0F:		//parallel register read
		if(ucDevAddr - 1 == g_stPrl.ucSelfId || ucDevAddr - 1 >= g_stPrl.ucDevNum) {
			return;
		}
		if(prl_host()) {
			memcpy((uint8_t*)(g_stLocalArrayRVal.astPackRVal + g_stPrl.ucSelfId), pucData, usLen);
		} else if(prl_client()) {
			memcpy(g_aucCanSBuf, (uint8_t*)(g_stLocalArrayRVal.astPackRVal + g_stPrl.ucSelfId), sizeof(LOCAL_PACK_RVAL_S));
			eco_can_data_send(0, ucDevAddr, ucFunCode, usRegAddr, g_aucCanSBuf, sizeof(LOCAL_PACK_RVAL_S));
		}
		break;
	case 0x10:		//client MOS control
		if(!prl_client()) {
			return;
		}
		if(ucDevAddr != g_stPrl.ucSelfId) {
			return;
		}
		if(pucData[0] & 0x01) {
			g_stPrl.ucChgEn = 0x55;
		} else {
			g_stPrl.ucChgEn = 0xAA;
		}
		if(pucData[0] & 0x02) {
			g_stPrl.ucDsgEn = 0x55;
		} else {
			g_stPrl.ucDsgEn = 0xAA;
		}
		if(pucData[0] & 0x04) {
			g_stPrl.ucHeatEn = 0x55;
		} else {
			g_stPrl.ucHeatEn = 0xAA;
		}
		break;
//	case 0x65:  //并机
//		memcpy((uint8_t*)(g_stLocalArrayRVal.astPackRVal + ucDevAddr - 1), pucData, usLen);
//		break;
	default:
		break;
	}
}

void eco_can_data_send(uint8_t ucSelfId, uint8_t ucDevAddr, uint8_t ucFunCode, uint8_t ucRegAddr, uint8_t* pucData, uint16_t usLen) {
	if(pucData == NULL) {
		return;
	}
	uint8_t aucData[8];
	if(usLen <= 7) {
		aucData[0] = 0;
		memcpy(aucData + 1, pucData, usLen);
		CAN0_SendMsg(((ucSelfId << 25) | (ucDevAddr << 21) | (0x00 << 16) | (ucFunCode << 8) | ucRegAddr), aucData, usLen + 1);
	} else {
		uint8_t ucSegFlag = 1;
		uint8_t ucSegCnt = 0;
		aucData[0] = (ucSegFlag << 6) | ucSegCnt;
		memcpy(aucData + 1, pucData, 7);
		delay_1ms(1); //2024.12.22 dgx
		CAN0_SendMsg(((ucSelfId << 25) | (ucDevAddr << 21) | (0x00 << 16) | (ucFunCode << 8) | ucRegAddr), aucData, 8);
		uint16_t usSendCnt = 7;
		while(usLen - usSendCnt > 7) {
			ucSegFlag = 2;
			ucSegCnt++;
			if(ucSegCnt > 63){
				ucSegCnt = 0;
			}
			aucData[0] = (ucSegFlag << 6) | ucSegCnt;
			memcpy(aucData + 1, pucData + usSendCnt, 7);
			delay_1ms(1);
			CAN0_SendMsg(((ucSelfId << 25) | (ucDevAddr << 21) | (0x00 << 16) | (ucFunCode << 8) | ucRegAddr), aucData, 8);
			usSendCnt += 7;
		}
		ucSegFlag = 3;
		ucSegCnt = 0;
		aucData[0] = (ucSegFlag << 6) | ucSegCnt;
		memcpy(aucData + 1, pucData + usSendCnt, usLen - usSendCnt);
		delay_1ms(1);
		CAN0_SendMsg(((ucSelfId << 25) | (ucDevAddr << 21) | (0x00 << 16) | (ucFunCode << 8) | ucRegAddr), aucData, usLen - usSendCnt + 1);
	}
	delay_1ms(2);
}

void eco_uart_ug_proc(uint8_t* pucData, uint16_t usLen) {
	if(pucData == 0) {
		return;
	}
	if(usLen == 8 && g_uiUGDataLen !=0) {   //To prevent upgrade failure, switch the upgrade status to running status
	  g_stLocalArrayRVal.eLocalStat = eLocalStatRun;
	  USART2_UgFlag = 0;
		if(pucData[1] == 0x03) {		//read register
				uint16_t usRegAddr = pucData[2] * 0x100 + pucData[3];
				uint16_t usRegLen = pucData[4] * 0x100 + pucData[5];
				if(usRegLen > 127) {
					return;
				}
				g_aucUartSBuf[2] = usRegLen * 2;
				for(uint16_t i=0;i<usRegLen;i++) {
					uint16_t usVal = eco_read_reg(0, 0, usRegAddr + i);
					g_aucUartSBuf[3 + i * 2] = (usVal >> 8);
					g_aucUartSBuf[4 + i * 2] = (usVal & 0xFF);
				}
				uint16_t usCrc = eco_crc16(g_aucUartSBuf, usRegLen * 2 + 3, 0xFFFF, 0xA001);
				g_aucUartSBuf[usRegLen * 2 + 3] = usCrc & 0xFF;
				g_aucUartSBuf[usRegLen * 2 + 4] = usCrc >> 8;
				UART_Send(g_aucUartSBuf, usRegLen * 2 + 5);
	  }
		return;
	}
	if(*pucData == 0x31) {
		g_aucUartSBuf[0] = 0x43;
		UART_Send(g_aucUartSBuf, 1);
		BSP_Stop();
	} else if(pucData[0] == 0x01 && pucData[1] == 0) {
		if(strstr((char*)pucData + 3, "LZH") != 0) {																	//start
			char* pcPos = strchr((char*)pucData + 3, 0x00);
			while(*(pcPos + 1) == 0){
				pcPos++;
				if(*(pcPos + 1) == 0 && *(pcPos + 2) == 0 && *(pcPos + 3) == 0 && *(pcPos + 4) == 0){
					return;
				}
			}
			if(pcPos == 0) {
				return;
			}
			pcPos++;
			g_uiUGDataLen = atoi(pcPos);
			if(g_uiUGDataLen >= 256 * 1024 || g_uiUGDataLen == 0) {
				return;
			}
			g_sUGSegIdx = 0;
			g_uiUGBufIdx = 0;
			g_aucUartSBuf[0] = 0x06;
			UART_Send(g_aucUartSBuf, 1);
			delay_1ms(100);
			g_aucUartSBuf[0] = 0x43;
			UART_Send(g_aucUartSBuf, 1);
			USART2_UgFlag = 1; 
		} else if(strstr((char*)pucData + 2, "EB") != 0){
			char* pcPos = strchr((char*)pucData + 2, 0x00);
			while(*(pcPos + 1) == 0){
				pcPos++;
				if(*(pcPos + 1) == 0 && *(pcPos + 2) == 0 && *(pcPos + 3) == 0 && *(pcPos + 4) == 0){
					return;
				}
			}
			if(pcPos == 0) {
				return;
			}
			pcPos++;
			g_uiUGDataLen = atoi(pcPos);
			if(g_uiUGDataLen >= 256 * 1024 || g_uiUGDataLen == 0) {
				return;
			}
			g_sUGSegIdx = 0;
			g_uiUGBufIdx = 0;
			g_aucUartSBuf[0] = 0x06;
			UART_Send(g_aucUartSBuf, 1);
			delay_1ms(100);
			g_aucUartSBuf[0] = 0x43;
			UART_Send(g_aucUartSBuf, 1);
			USART2_UgFlag = 1; 
		}
	} else if(pucData[0] == 0x02 || (pucData[0] == 0x01 && pucData[1] != 0)) {	//(ug-seg = 1~n-1) || (ug-seg =  n)
		if(pucData[1] != g_sUGSegIdx + 1) {
			//return;
		}
		if(g_uiUGBufIdx + usLen - 5 > g_uiUGDataLen) {
			//return;
		}
		if(usLen > 5) {
			uint16_t usCrc = eco_crc16_XMODEM(pucData + 3, usLen - 5);
			if(usCrc != pucData[usLen - 2] * 0x100 + pucData[usLen - 1]) {
				g_aucUartSBuf[0] = 0x78;
			  g_aucUartSBuf[1] = 0x87;
		    UART_Send(g_aucUartSBuf, 2);
			  return;
			}
	  }
		MEM_FlashWrite(eHisOTAPStart * PAGE_SIZE + g_uiUGBufIdx, pucData + 3, usLen - 5);
		g_sUGSegIdx ++;
		g_uiUGBufIdx += (usLen - 5);
		g_aucUartSBuf[0] = 0x06;
		UART_Send(g_aucUartSBuf, 1);
	}else if(pucData[1] == 0x02 && pucData[2] + pucData[3] == 0xFF){      //Fault tolerance to prevent first byte misalignment
			if(usLen != 1029){
				g_aucUartSBuf[0] = 0x78;
				g_aucUartSBuf[1] = 0x87;
				UART_Send(g_aucUartSBuf, 2);
				return;
			}
	  	MEM_FlashWrite(eHisOTAPStart * PAGE_SIZE + g_uiUGBufIdx, pucData + 4, usLen - 5);
		  g_sUGSegIdx ++;
		  g_uiUGBufIdx += (usLen - 5);
		  g_aucUartSBuf[0] = 0x06;
		  UART_Send(g_aucUartSBuf, 1);
	}else if(pucData[0] == 0x04 && pucData[1] == 0x01 && pucData[2]==0x00 && pucData[3] == 0xFF){ //ug done
			g_aucUartSBuf[0] = 0x06;
			UART_Send(g_aucUartSBuf, 1);
			USART2_UgFlag = 0;
		  cfg_set_default();
		  delay_1ms(10);
		  g_stLocalArrayRVal.eLocalStat = eLocalStatRun;
			g_stCfg.stOta.uiAddr = eHisOTAPStart * PAGE_SIZE;
			g_stCfg.stOta.uiLen = g_uiUGDataLen;
			g_stCfg.stOta.ucUpdate = 0x01;
		  g_stCfg.usGoRun = 1;
			cfg_save();
			delay_1ms(10);
		  g_eBaseStat = eBStatReset_52;
			his_data_write();
			delay_1ms(500);
//	  usart_deinit(USART2);
//		dma_deinit(DMA0, DMA_CH2);
			System_Reset();
		}
}

void eco_uart_recv_proc(uint8_t* pucData, uint16_t usLen) {
	memset(g_aucUartSBuf, 0, sizeof(g_aucUartSBuf));
	if(pucData == 0) {
		return;
	}
	if(pucData[0] == 0x0D && pucData[1] == 0x0D && pucData[2] == 0x0A && pucData[3] == 0x49 && pucData[4] == 0x4D && pucData[5] == 0x5F){
		g_stLocalArrayRVal.eLocalStat = eLocalStatRun;
	  USART2_UgFlag = 0;
	  return;
	}
	if(g_usSrlCntTick != 0) {	//enter upgrade mode
		if(usLen >= 8) {
			if(!memcmp(pucData, "BootLoad", 8)) {
			  g_eBaseStat = eBStatDownload;
			  his_data_write();
				g_stLocalArrayRVal.eLocalStat = eLocalStatUartBootMode;
				UART_Send(pucData, usLen);
				return;
			}
		}
	}
	if(g_stLocalArrayRVal.eLocalStat == eLocalStatUartBootMode) {
		SW_EN_LED_L;
		eco_uart_ug_proc(pucData, usLen);
		SW_EN_LED_H;
		return;
	}
//	if(usLen < 8) {
//		return;
//	}
//	if(pucData[0] != g_stPrl.ucSelfId + 1) {
//		return;
//	}
//	uint16_t usCrc = eco_crc16(pucData, usLen - 2, 0xFFFF, 0xA001);
//	if((usCrc & 0xFF) != pucData[usLen - 2] || usCrc >> 8 != pucData[usLen - 1]) {
//		return;
//	}
	g_aucUartSBuf[0] = 1;
	g_aucUartSBuf[1] = pucData[1];
	uint16_t usRegAddr = pucData[2] * 0x100 + pucData[3];
	if(g_usSrlCntTick == 0 && ((pucData[1] != 0x06 && pucData[1] != 0x10) || (usRegAddr != 0x044C && usRegAddr != 0x04B0))) {
	//  return;
	}
	if(pucData[1] == 0x03) {		//read register
		uint16_t usRegLen = pucData[4] * 0x100 + pucData[5];
		if(usRegLen > 200) {
			return;
		}
		g_aucUartSBuf[2] = usRegLen * 2;
		for(uint16_t i=0;i<usRegLen;i++) {
			uint16_t usVal = eco_read_reg(0, 0, usRegAddr + i);
			g_aucUartSBuf[3 + i * 2] = (usVal >> 8);
			g_aucUartSBuf[4 + i * 2] = (usVal & 0xFF);
		}
		uint16_t usCrc = eco_crc16(g_aucUartSBuf, usRegLen * 2 + 3, 0xFFFF, 0xA001);
		g_aucUartSBuf[usRegLen * 2 + 3] = usCrc & 0xFF;
		g_aucUartSBuf[usRegLen * 2 + 4] = usCrc >> 8;
		UART_Send(g_aucUartSBuf, usRegLen * 2 + 5);
	} else if(pucData[1] == 0x06) {	//write single register
//		uint16_t usRegVal = pucData[4] * 0x100 + pucData[5];
		uint16_t usRegVal = pucData[4] * 0x100 + pucData[5];
		uint16_t usRegAddr = pucData[2] * 0x100 + pucData[3];
//		if(usRegAddr > 0x20 + CFG_CELL_NUM + 50) { //0x20 + CFG_CELL_NUM + 70
//			return;
//		}
		eco_write_reg(usRegAddr, usRegVal);
		memcpy(g_aucUartSBuf, pucData, usLen);
		UART_Send(pucData, usLen);
	} else if(pucData[1] == 0x09) {	//MOS control register
		  if(usRegAddr == 3) {		//charge MOS force on enable
						 if(pucData[5] == 1){
							 g_stLocalArrayRVal.astPackRVal[g_stPrl.ucSelfId].ucChgForceEn = 0x55;
							 MCU_CHG_ON;
						 }else{
						 	 g_stLocalArrayRVal.astPackRVal[g_stPrl.ucSelfId].ucChgForceEn = 0;
							 MCU_CHG_OFF;
						 }
		  } else if(usRegAddr == 4) {	//charge MOS force on disable
			       if(pucData[5] == 1){
			         g_stLocalArrayRVal.astPackRVal[g_stPrl.ucSelfId].ucChgForceEn = 0xAA;
			         MCU_CHG_OFF;
						 }else{
							 g_stLocalArrayRVal.astPackRVal[g_stPrl.ucSelfId].ucChgForceEn = 0;
			         MCU_CHG_OFF;
						 }
		  } else if(usRegAddr == 5) {	//discharge MOS force on enable
						 if(pucData[5] == 1){
							 g_stLocalArrayRVal.astPackRVal[g_stPrl.ucSelfId].ucDsgForceEn = 0x55;
							 MCU_DSG_ON;
							}else{
							 g_stLocalArrayRVal.astPackRVal[g_stPrl.ucSelfId].ucDsgForceEn = 0;
							 MCU_DSG_OFF;
							}
		  } else if(usRegAddr == 6) {	//discharge MOS force on disable
						if(pucData[5] == 1){
							g_stLocalArrayRVal.astPackRVal[g_stPrl.ucSelfId].ucDsgForceEn = 0xAA;
							MCU_DSG_OFF;
							}else{
							g_stLocalArrayRVal.astPackRVal[g_stPrl.ucSelfId].ucDsgForceEn = 0;
							MCU_DSG_OFF;
							}
			}
			UART_Send(pucData, usLen);
		} else if(pucData[1] == 0x10) {
		uint16_t usRegLen = pucData[4] * 0x100 + pucData[5];
		if(usRegLen * 2 != pucData[6]) {
			return;
		}
		for(uint16_t i=0;i<usRegLen;i++) {
			eco_write_reg(usRegAddr + i, pucData[7 * i * 2] * 0x100 + pucData[8 * i * 2]);
		}
		memcpy(g_aucUartSBuf + 2, pucData + 2, 4);
		uint16_t usCrc = eco_crc16(g_aucUartSBuf, 6, 0xFFFF, 0xA001);
		g_aucUartSBuf[6] = usCrc & 0xFF;
		g_aucUartSBuf[7] = usCrc >> 8;
		UART_Send(g_aucUartSBuf, 8);
	} else if(pucData[1] == 0xAA) {		//charge & discharge info
		g_aucUartSBuf[1] = 0xAA;
		g_aucUartSBuf[2] = 0x00;
		g_aucUartSBuf[3] = 0x01;
		g_aucUartSBuf[4] = 0x02;
		g_aucUartSBuf[5] = 0x03;
		g_aucUartSBuf[6] = 0x04;
		g_aucUartSBuf[7] = 0x05;
		UART_Send(g_aucUartSBuf, 8);
		uint16_t usSLen = eco_get_cd(g_aucUartSBuf);
		UART_Send(g_aucUartSBuf, usSLen);
		g_aucUartSBuf[1] = 0xAA;
		g_aucUartSBuf[2] = 0x01;
		g_aucUartSBuf[3] = 0x02;
		g_aucUartSBuf[4] = 0x03;
		g_aucUartSBuf[5] = 0x04;
		g_aucUartSBuf[6] = 0x05;
		g_aucUartSBuf[7] = 0x06;
		UART_Send(g_aucUartSBuf, 8);
	} else if(pucData[1] == 0xBB) {		//read history data
		g_aucUartSBuf[1] = 0xDD;
		g_aucUartSBuf[2] = 0x00;
		g_aucUartSBuf[3] = 0x01;
		g_aucUartSBuf[4] = 0x02;
		g_aucUartSBuf[5] = 0x03;
		g_aucUartSBuf[6] = 0x04;
		g_aucUartSBuf[7] = 0x05;
		UART_Send(g_aucUartSBuf, 8);
		for(uint16_t i=0;i<1000;i++) {//64
			if(!his_data_read(i, (HIS_DATA_S*)g_aucUartSBuf)) {
				break;
			}
			UART_Send(g_aucUartSBuf, usLen);
			WDG_DONE_H;
			WDG_DONE_L;
			gd32_wdgt_feed_dog(EWdgType_fwdg);	//feed dog
		}
		g_aucUartSBuf[1] = 0xDD;
		g_aucUartSBuf[2] = 0x01;
		g_aucUartSBuf[3] = 0x02;
		g_aucUartSBuf[4] = 0x03;
		g_aucUartSBuf[5] = 0x04;
		g_aucUartSBuf[6] = 0x05;
		g_aucUartSBuf[7] = 0x06;
		UART_Send(g_aucUartSBuf, 8);
	} else if(pucData[1] == 0xDD) {		//read history log
		g_aucUartSBuf[1] = 0xDD;
		g_aucUartSBuf[2] = 0x00;
		g_aucUartSBuf[3] = 0x01;
		g_aucUartSBuf[4] = 0x02;
		g_aucUartSBuf[5] = 0x03;
		g_aucUartSBuf[6] = 0x04;
		g_aucUartSBuf[7] = 0x05;
		UART_Send(g_aucUartSBuf, 8);
		for(uint16_t i=0;i<1000;i++) {//1024
			uint16_t usLen = eco_get_log(i, g_aucUartSBuf);
			if(usLen == 0) {
				break;
			}
			UART_Send(g_aucUartSBuf, usLen);
			WDG_DONE_H;
			WDG_DONE_L;
			gd32_wdgt_feed_dog(EWdgType_fwdg);	//feed dog
		}
		g_aucUartSBuf[1] = 0xDD;
		g_aucUartSBuf[2] = 0x01;
		g_aucUartSBuf[3] = 0x02;
		g_aucUartSBuf[4] = 0x03;
		g_aucUartSBuf[5] = 0x04;
		g_aucUartSBuf[6] = 0x05;
		g_aucUartSBuf[7] = 0x06;
		UART_Send(g_aucUartSBuf, 8);
	} else if(pucData[1] == 0xEE) {	//read history charge & discharge info
		g_aucUartSBuf[1] = 0xEE;
		g_aucUartSBuf[2] = 0x00;
		g_aucUartSBuf[3] = 0x01;
		g_aucUartSBuf[4] = 0x02;
		g_aucUartSBuf[5] = 0x03;
		g_aucUartSBuf[6] = 0x04;
		g_aucUartSBuf[7] = 0x05;
		UART_Send(g_aucUartSBuf, 8);
		if(pucData[2] == 0x01){
			for(uint16_t i=0;i<1000;i++) {//1000
				HIS_DATA_S stData = {0};
				if(!his_data_read(i, &stData)) {
					break;
				}
				g_aucUartSBuf[0] = 0xAA;
				g_aucUartSBuf[1] = 0x55;
				g_aucUartSBuf[2] = 0x27;
				stData.dt += 8 * 3600;
				struct tm* t = localtime(&stData.dt);
				g_aucUartSBuf[3] = t->tm_year - 100;
				g_aucUartSBuf[4] = t->tm_mon + 1;
				g_aucUartSBuf[5] = t->tm_mday;
				g_aucUartSBuf[6] = t->tm_hour;
				g_aucUartSBuf[7] = t->tm_min;
				g_aucUartSBuf[8] = t->tm_sec;
				g_aucUartSBuf[9] = stData.state;
				g_aucUartSBuf[10] = *((uint8_t*)&(stData.stRtv) + 16);
				g_aucUartSBuf[11] = *((uint8_t*)&(stData.stRtv) + 17);
				g_aucUartSBuf[12] = *((uint8_t*)&(stData.stRtv) + 18);
				g_aucUartSBuf[13] = *((uint8_t*)&(stData.stRtv) + 19);
				g_aucUartSBuf[14] = stData.stRtv.usPackRTSoc;
				g_aucUartSBuf[15] = stData.stRtv.usPackRTVolt / 256;
				g_aucUartSBuf[16] = (stData.stRtv.usPackRTVolt & 0xFF);
				g_aucUartSBuf[17] = (int16_t)stData.stRtv.sPackRTCur >> 8;
				g_aucUartSBuf[18] = (int16_t)stData.stRtv.sPackRTCur & 0xFF;
				g_aucUartSBuf[19] = (int16_t)stData.stRtv.sPackRTCur >> 8;
				g_aucUartSBuf[20] = (int16_t)stData.stRtv.sPackRTCur & 0xFF;
				uint16_t usPackVolt = 0;
				uint16_t usMaxVal = stData.stRtv.ausCellRTVolt[0];
				uint16_t usMinVal = stData.stRtv.ausCellRTVolt[0];
				for(uint8_t j=0;j<CFG_CELL_NUM;j++) {
					usPackVolt += stData.stRtv.ausCellRTVolt[j];
					if(usMaxVal < stData.stRtv.ausCellRTVolt[j]) {
						usMaxVal = stData.stRtv.ausCellRTVolt[j];
					}
					if(usMinVal > stData.stRtv.ausCellRTVolt[j]) {
						usMinVal = stData.stRtv.ausCellRTVolt[j];
					}
				}
				g_aucUartSBuf[21] = usMaxVal / 256;
				g_aucUartSBuf[22] = (usMaxVal & 0xFF);
				g_aucUartSBuf[23] = usMinVal / 256;
				g_aucUartSBuf[24] = (usMinVal & 0xFF);
				int16_t asTemp[3] = {stData.stRtv.sCellRTT1, stData.stRtv.sCellRTT2, stData.stRtv.sCellRTT3};
				int16_t sMaxVal = asTemp[0];
				int16_t sMinVal = asTemp[0];
				for(uint8_t j=1;j<3;j++) {
					if(sMaxVal < asTemp[j]) {
						sMaxVal = asTemp[j];
					}
					if(sMinVal > asTemp[j]) {
						sMinVal = asTemp[j];
					}
				}
				g_aucUartSBuf[25] = sMaxVal / 256;
				g_aucUartSBuf[26] = (sMaxVal & 0xFF);
				g_aucUartSBuf[27] = sMinVal / 256;
				g_aucUartSBuf[28] = (sMinVal & 0xFF);
				g_aucUartSBuf[29] = stData.stRtv.sMOSRTT1 / 256;
				g_aucUartSBuf[30] = (stData.stRtv.sMOSRTT1 & 0xFF);
				g_aucUartSBuf[31] = stData.stRtv.ausSN[0];
				g_aucUartSBuf[32] = stData.stRtv.ausSN[1];
				g_aucUartSBuf[33] = stData.stRtv.ausSN[2];
				g_aucUartSBuf[34] = stData.stRtv.ausSN[3];
				g_aucUartSBuf[35] = g_stCfg.stLocal.ucSVerAgree - 0x40;//stData.stRtv.ausSN[4]
				g_aucUartSBuf[36] = 3;//stData.stRtv.ausSN[5]
				g_aucUartSBuf[37] = stData.stRtv.ausSN[6];
				g_aucUartSBuf[38] = stData.stRtv.ausSN[7];
				g_aucUartSBuf[39] = stData.stRtv.usTempIsNotRising;
				g_aucUartSBuf[40] = stData.stRtv.usHeatingTimeout;
				g_aucUartSBuf[41] = stData.stRtv.usHeatingOverNumber;
				uint16_t usCRC = eco_crc16(g_aucUartSBuf, 42, 0xFFFF, 0xA001);//29  37
				g_aucUartSBuf[42] = (usCRC & 0xFF);
				g_aucUartSBuf[43] = (usCRC >> 8);
				UART_Send(g_aucUartSBuf, 44);
				WDG_DONE_H;
				WDG_DONE_L;
			}
		} else if(pucData[2] == 0x02) {
			for(uint16_t i=0;i<5000;i++) {//5000
				HIS_DATA_S stData = {0};
				if(!his_data_read(i, &stData)) {
					break;
				}
				g_aucUartSBuf[0] = 0xAA;
				g_aucUartSBuf[1] = 0x55;
				g_aucUartSBuf[2] = 0x24;
				stData.dt += 8 * 3600;
				struct tm* t = localtime(&stData.dt);
				g_aucUartSBuf[3] = t->tm_year - 100;
				g_aucUartSBuf[4] = t->tm_mon + 1;
				g_aucUartSBuf[5] = t->tm_mday;
				g_aucUartSBuf[6] = t->tm_hour;
				g_aucUartSBuf[7] = t->tm_min;
				g_aucUartSBuf[8] = t->tm_sec;
				g_aucUartSBuf[9] = stData.state;
				g_aucUartSBuf[10] = *((uint8_t*)&(stData.stRtv) + 16);
				g_aucUartSBuf[11] = *((uint8_t*)&(stData.stRtv) + 17);
				g_aucUartSBuf[12] = *((uint8_t*)&(stData.stRtv) + 18);
				g_aucUartSBuf[13] = *((uint8_t*)&(stData.stRtv) + 19);
				g_aucUartSBuf[14] = stData.stRtv.usPackRTSoc;
				g_aucUartSBuf[15] = stData.stRtv.usPackRTVolt / 256;
				g_aucUartSBuf[16] = (stData.stRtv.usPackRTVolt & 0xFF);
				g_aucCanSBuf[17] = (int16_t)stData.stRtv.sPackRTCur >> 8;
				g_aucCanSBuf[18] = (int16_t)stData.stRtv.sPackRTCur & 0xFF;
				g_aucCanSBuf[19] = (int16_t)stData.stRtv.sPackRTCur >> 8;
				g_aucCanSBuf[20] = (int16_t)stData.stRtv.sPackRTCur & 0xFF;
				uint16_t usPackVolt = 0;
				uint16_t usMaxVal = stData.stRtv.ausCellRTVolt[0];
				uint16_t usMinVal = stData.stRtv.ausCellRTVolt[0];
				for(uint8_t j=0;j<CFG_CELL_NUM;j++) {
					usPackVolt += stData.stRtv.ausCellRTVolt[j];
					if(usMaxVal < stData.stRtv.ausCellRTVolt[j]) {
						usMaxVal = stData.stRtv.ausCellRTVolt[j];
					}
					if(usMinVal > stData.stRtv.ausCellRTVolt[j]) {
						usMinVal = stData.stRtv.ausCellRTVolt[j];
					}
				}
				g_aucUartSBuf[21] = usMaxVal / 256;
				g_aucUartSBuf[22] = (usMaxVal & 0xFF);
				g_aucUartSBuf[23] = usMinVal / 256;
				g_aucUartSBuf[24] = (usMinVal & 0xFF);
				int16_t asTemp[3] = {stData.stRtv.sCellRTT1, stData.stRtv.sCellRTT2, stData.stRtv.sCellRTT3};
				int16_t sMaxVal = asTemp[0];
				int16_t sMinVal = asTemp[0];
				for(uint8_t j=1;j<3;j++) {
					if(sMaxVal < asTemp[j]) {
						sMaxVal = asTemp[j];
					}
					if(sMinVal > asTemp[j]) {
						sMinVal = asTemp[j];
					}
				}
				g_aucUartSBuf[25] = sMaxVal / 256;
				g_aucUartSBuf[26] = (sMaxVal & 0xFF);
				g_aucUartSBuf[27] = sMinVal / 256;
				g_aucUartSBuf[28] = (sMinVal & 0xFF);
				g_aucUartSBuf[29] = stData.stRtv.sMOSRTT1 / 256;
				g_aucUartSBuf[30] = (stData.stRtv.sMOSRTT1 & 0xFF);
				g_aucUartSBuf[31] = stData.stRtv.ausSN[0];
				g_aucUartSBuf[32] = stData.stRtv.ausSN[1];
				g_aucUartSBuf[33] = stData.stRtv.ausSN[2];
				g_aucUartSBuf[34] = stData.stRtv.ausSN[3];
				g_aucUartSBuf[35] = g_stCfg.stLocal.ucSVerAgree - 0x40;//stData.stRtv.ausSN[4]
				g_aucUartSBuf[36] = 3;//stData.stRtv.ausSN[5]
				g_aucUartSBuf[37] = stData.stRtv.ausSN[6];
				g_aucUartSBuf[38] = stData.stRtv.ausSN[7];
				g_aucUartSBuf[39] = stData.stRtv.usTempIsNotRising;
				g_aucUartSBuf[40] = stData.stRtv.usHeatingTimeout;
				g_aucUartSBuf[41] = stData.stRtv.usHeatingOverNumber;
				uint16_t usCRC = eco_crc16(g_aucUartSBuf, 42, 0xFFFF, 0xA001);
				g_aucUartSBuf[42] = (usCRC & 0xFF);
				g_aucUartSBuf[43] = (usCRC >> 8);
				UART_Send(g_aucUartSBuf, 44);
				WDG_DONE_H;
				WDG_DONE_L;
			}
		}
		g_aucUartSBuf[0] = 0x01;
		g_aucUartSBuf[1] = 0xEE;
		g_aucUartSBuf[2] = 0x01;
		g_aucUartSBuf[3] = 0x02;
		g_aucUartSBuf[4] = 0x03;
		g_aucUartSBuf[5] = 0x04;
		g_aucUartSBuf[6] = 0x05;
		g_aucUartSBuf[7] = 0x06;
		UART_Send(g_aucUartSBuf, 8);
	}
}

void eco_init(void) {
	char acBuf[64] = {0};
	sprintf(acBuf, "AT+NAME=%s\r\n", g_stCfg.stLocal.aucBleName);
	UART_Send((uint8_t*)acBuf, strlen(acBuf));
	sprintf(acBuf, "AT+REBOOT=1\r\n");
	UART_Send((uint8_t*)acBuf, strlen(acBuf));
	g_usPassword = eco_crc16((uint8_t*)acBuf, strlen(acBuf), 0xFFFF, 0xA001);
}

void eco_proc(void) {
#ifndef USER_DEBUG
	if(g_usSrlCntTick > 0) {
		g_usSrlCntTick--;
	}
#endif
	BSP_5V_1A_3A_Detection();		//5V/1A 3A ouput protection
	LOCAL_PACK_RVAL_S* pstPackRVal = g_stLocalArrayRVal.astPackRVal + g_stPrl.ucSelfId;
	/* Identify if GBMS3 is connected */
	if(g_usGbms3ComTick != 0) {
		g_usGbms3ComTick--;
	} else {
		g_bGbms3 = false;
	}
	
	/* Main charger communication countdown */
	if(g_usMChgerComTick != 0) {
		g_usMChgerComTick--;
		g_bMChgerComAct = true;
	} else {
		g_bMChgerComAct = false;
		g_bMChgerAbNor = false;
		g_stEcoRtv.usMChgerActCur = 0;
		if(round(pstPackRVal->fCellTMin) > g_stCfg.stLocal.sHeaterUTTVThr) {
			g_bNeedHeat = false;
		}
	}
	
	/* Second charger communication countdown */
	if(g_usSChgerComTick != 0) {
		g_usSChgerComTick--;
		g_bSChgerComAct = true;
	} else {
		g_bSChgerComAct = false;
		g_stEcoRtv.usSChgerActCur = 0;
	}
	if(g_bMChgerComAct || g_bSChgerComAct) {
		pstPackRVal->uBaseStat.stBaseStat.ucChgerON = 1;
	} else {
		pstPackRVal->uBaseStat.stBaseStat.ucChgerON = 0;
		g_bMChging = false;
	}
	
	/* 0x18F880F3 */
	if(g_us18F880F3ComTick != 0) {
		g_us18F880F3ComTick--;
		g_bs18F880F3Act = true;
	} else {
		g_bs18F880F3Act = false;
	}
	
//	if(g_ausPrlComTick[g_stPrl.ucSelfId] != 0) {
//		g_ausPrlComTick[g_stPrl.ucSelfId]--;
//	} else {
//		g_stPrl.ucDevNum = 1;
//		g_stPrl.ucSelfId = 0;
//	}
	
	eco_refresh_RTV();
	eco_refresh_CD();
	eco_refresh_log();
}
	
void eco_refresh_RTV(void) {
	LOCAL_PACK_RVAL_S* pstPackRVal = g_stLocalArrayRVal.astPackRVal + g_stPrl.ucSelfId;
	g_stEcoRtv.usPackNominalVolt = 0x8018;		//static code
	g_stEcoRtv.usPackNominalAh = g_stCfg.stLocal.usDesignAH * 10;
	g_stEcoRtv.usCellSeriesNum = g_stCfg.stLocal.usSerialCellNum;
	g_stEcoRtv.usPackRTVolt = pstPackRVal->fPackU * 10;
	g_stEcoRtv.sPackRTCur = pstPackRVal->fPackCur * 10;
	g_stEcoRtv.usPackRTSoc = pstPackRVal->fPackSoc;
	g_stEcoRtv.usPackRTSoh = pstPackRVal->fPackSoh * 10;
	g_stEcoRtv.usCycleCnt = pstPackRVal->usCycle;
	if(GET_ALM0_CODE(1)) g_stEcoRtv.MosAnomaly = 1; else g_stEcoRtv.MosAnomaly = 0;
	if(GET_ALM0_CODE(3)) g_stEcoRtv.bVDIFFW = 1; else g_stEcoRtv.bVDIFFW = 0;
	if(GET_ALM0_CODE(10)) g_stEcoRtv.bSWCellODT = 1; else g_stEcoRtv.bSWCellODT = 0;
	if(GET_ALM0_CODE(4)) {
		g_stEcoRtv.bSWCellOV = 1;
	} else if(GET_ALM1_CODE(58)) {
		g_stEcoRtv.bSWCellOV = 1;
	} else {
		g_stEcoRtv.bSWCellOV = 0;
	}
	if(GET_ALM0_CODE(5)) g_stEcoRtv.bSWCellUV = 1; else g_stEcoRtv.bSWCellUV = 0;
	if(GET_ALM0_CODE(7)) g_stEcoRtv.bSWPackUV = 1; else g_stEcoRtv.bSWPackUV = 0;
	if(GET_ALM0_CODE(6)) g_stEcoRtv.bSWPackOV = 1; else g_stEcoRtv.bSWPackOV = 0;
	if(GET_ALM0_CODE(2)) g_stEcoRtv.bHWSC = 1; else g_stEcoRtv.bHWSC = 0;
	if(GET_ALM0_CODE(9)) g_stEcoRtv.bSWOCC = 1; else g_stEcoRtv.bSWOCC = 0;
	if(GET_ALM0_CODE(8)) g_stEcoRtv.bSWODC = 1; else g_stEcoRtv.bSWODC = 0;
	if(GET_ALM0_CODE(13)) g_stEcoRtv.bSWCellUDT = 1; else g_stEcoRtv.bSWCellUDT = 0;
	if(GET_ALM0_CODE(12)) g_stEcoRtv.bSWCellUCT = 1; else g_stEcoRtv.bSWCellUCT = 0;
	if(GET_ALM0_CODE(14)) g_stEcoRtv.bSWMOSOT = 1; else g_stEcoRtv.bSWMOSOT = 0;
	if(GET_ALM0_CODE(11)) g_stEcoRtv.bSWOCT = 1; else g_stEcoRtv.bSWOCT = 0;
	if(GET_ALM0_CODE(15)) g_stEcoRtv.bsWSocLow = 1; else g_stEcoRtv.bsWSocLow = 0;
	if(pstPackRVal->uErrCode.stErrCode.bTempSensor) g_stEcoRtv.bTempSersorErr = 1; else g_stEcoRtv.bTempSersorErr = 0;
	if(pstPackRVal->uErrCode.stErrCode.bVoltSensor) g_stEcoRtv.bVoltSensor = 1;else g_stEcoRtv.bVoltSensor = 0;
	g_stEcoRtv.bDMOSOn = g_stAfe.uRam.stCode.DSG_FET;
	g_stEcoRtv.bCMOSOn = g_stAfe.uRam.stCode.CHG_FET;
//	if(!g_bMChging) {
//		pstPackRVal->uBaseStat.stBaseStat.ucHeating = 0;
//	}
	g_stEcoRtv.bHeaterOn = pstPackRVal->uBaseStat.stBaseStat.ucHeating;
	g_stEcoRtv.bBAL = g_BalanceFlag; 
	g_stEcoRtv.bPreDsgOn = PRE_DSG_READ;
	if(pstPackRVal->ucChgForceEn == 0) g_stEcoRtv.bDMOSForceOn = 0; else g_stEcoRtv.bDMOSForceOn = 1;
  if(pstPackRVal->ucDsgForceEn == 0) g_stEcoRtv.bCMOSForceOn = 0; else g_stEcoRtv.bCMOSForceOn = 1;
	if(pstPackRVal->ucHeatForceEn == 0) g_stEcoRtv.bHMOSForceOn = 0; else g_stEcoRtv.bHMOSForceOn = 1;
	g_stEcoRtv.usCellUVTVThr1 = g_stCfg.stLocal.usCellUVTVThr1;
	g_stEcoRtv.usCellOVTVThr1 = g_stCfg.stLocal.usCellOVTVThr1;
	g_stEcoRtv.sCellOCTTVThr1 = g_stCfg.stLocal.sCellOCTTVThr1;
	g_stEcoRtv.sMOSOTTVThr2 = g_stCfg.stLocal.sMOSOTTVThr2;
	g_stEcoRtv.sCellUCTTVThr1 = g_stCfg.stLocal.sCellUCTTVThr1;
	g_stEcoRtv.sCellUDTTVThr1 = g_stCfg.stLocal.sCellUDTTVThr1;
	g_stEcoRtv.usODCTVThr1 = g_stCfg.stLocal.usODCTVThr1;
	g_stEcoRtv.usOCCTVThr1 = g_stCfg.stLocal.usOCCTVThr1;
	g_stEcoRtv.usCellUVRVThr1 = g_stCfg.stLocal.usCellUVRVThr1;
	g_stEcoRtv.usCellOVRVThr1 = g_stCfg.stLocal.usCellOVRVThr1;
	g_stEcoRtv.sCellOCTRVThr1 = g_stCfg.stLocal.sCellOCTRVThr1;
	g_stEcoRtv.sMOSOTRVThr2 = g_stCfg.stLocal.sMOSOTRVThr2;
	g_stEcoRtv.sMOSRTT1 = pstPackRVal->fMosT;
	g_stEcoRtv.sCellRTT1 = pstPackRVal->afCellT[0];
	g_stEcoRtv.sCellRTT2 = pstPackRVal->afCellT[1];
	g_stEcoRtv.sCellRTT3 = pstPackRVal->afCellT[2];
	g_stEcoRtv.sCellRTT4 = 0x03E8;     //The reservation is useless, and the transmission of 1000 indicates N/A
	g_stEcoRtv.sMOSRTT2 = 0x3E8;       //The reservation is useless, and the transmission of 1000 indicates N/A
	g_stEcoRtv.sCellUCTRVThr1 = g_stCfg.stLocal.sCellUCTRVThr1;
	g_stEcoRtv.sCellUDTRVThr1 = g_stCfg.stLocal.sCellUDTRVThr1;
	g_stEcoRtv.sCellODTTVThr1 = g_stCfg.stLocal.sCellODTTVThr1;
	g_stEcoRtv.sCellODTRVThr1 = g_stCfg.stLocal.sCellODTRVThr1;
	for(uint8_t i=0;i<CFG_CELL_NUM;i++) {
		g_stEcoRtv.ausCellRTVolt[i] = pstPackRVal->afCellU[i] * 1000;
	}
	g_stEcoRtv.usBALActVolt = g_stCfg.stLocal.usBALActVolt;
	g_stEcoRtv.usBALActDVolt = g_stCfg.stLocal.usBALActDVolt;
//	g_stEcoRtv.usVALReactDVolt = g_stCfg.stLocal.usVALReactDVolt; //2024.8.22
	g_stEcoRtv.usVALReactDVolt = (g_stCfg.stLocal.iSreenSlpSec / 60) & 0xFFFF;  //In order to be compatible with upper computers
	g_stEcoRtv.usRTSlpCnt = (g_stCfg.stLocal.iSreenSlpSec / 60) & 0xFFFF;
	if(g_stCfg.stLocal.iStandbySlpSec < 0) {
		g_stEcoRtv.sStandbySlpCnt = 0xFFFF; 
	} else {
		g_stEcoRtv.sStandbySlpCnt = ((g_stCfg.stLocal.iStandbySlpSec / 60) & 0xFFFF);	//Moddified by h00205922, 2023.09.21
	}
	g_stEcoRtv.usDbgPara = g_stCfg.stLocal.usDbgPara;
	g_stEcoRtv.usHWSCTVThr = g_stAfe.uRom.stCode.SCV;
  g_stEcoRtv.usPackUVTVThr1 = g_stCfg.stLocal.usPackUVTVThr1;
	g_stEcoRtv.usPackOVTVThr1 = g_stCfg.stLocal.usPackOVTVThr1;
 	g_stEcoRtv.usPackUVRVThr1 = g_stCfg.stLocal.usPackUVRVThr1;
	g_stEcoRtv.usPackOVRVThr1 = g_stCfg.stLocal.usPackOVRVThr1;
	g_stEcoRtv.sChgCurSnrCoeB = g_stCfg.stLocal.sChgCurSnrCoeB;
	g_stEcoRtv.sDsgCurSnrCoeB = g_stCfg.stLocal.sDsgCurSnrCoeB;
	g_stEcoRtv.sVoltSnrCoeB = g_stCfg.stLocal.sVoltSnrCoeB;
	g_stEcoRtv.ausRTC[0] = (calender.w_year - 2000) * 0x100 + calender.w_month;
	g_stEcoRtv.ausRTC[1] = calender.w_day * 0x100 + calender.hour;
	g_stEcoRtv.ausRTC[2] = calender.min * 0x100 + calender.sec;
	g_stEcoRtv.usDevNum = g_stPrl.ucDevNum;
	g_stEcoRtv.ausSN[0] = 30;
	g_stEcoRtv.ausSN[1] = g_stCfg.stLocal.ucSVerVMajor;
	g_stEcoRtv.ausSN[2] = g_stCfg.stLocal.ucSVerVMinor;
	g_stEcoRtv.ausSN[3] = g_stCfg.stLocal.ucSVerRMajor * 10 + g_stCfg.stLocal.ucSVerRMinor;
	g_stEcoRtv.ausSN[4] = g_stCfg.stLocal.ucSVerAgree - 0x40;
	g_stEcoRtv.ausSN[5] = 3;
	g_stEcoRtv.sHeaterUTTVThr = g_stCfg.stLocal.sHeaterUTTVThr;
	g_stEcoRtv.usCellOCTTTThr1 = g_stCfg.stLocal.usCellOCTTTThr1 / 200;
	g_stEcoRtv.usCellODTTTThr1 = g_stCfg.stLocal.usCellODTTTThr1 / 200;//2024.8.22 /2
	g_stEcoRtv.usMOSOTTTThr2 = g_stCfg.stLocal.usMOSOTTTThr2 / 200;
	g_stEcoRtv.usCellUCTTTThr1 = g_stCfg.stLocal.usCellUCTTTThr1 / 200;
	g_stEcoRtv.usCellUDTTTThr1 = g_stCfg.stLocal.usCellUDTTTThr1 / 200;
	g_stEcoRtv.usCellUVTTThr2 = g_stCfg.stLocal.usCellUVTTThr2 / 200;
	g_stEcoRtv.usCellOVTTThr1 = g_stCfg.stLocal.usCellOVTTThr1 / 200;
	g_stEcoRtv.usPackOVTTThr1 = g_stCfg.stLocal.usPackOVTTThr1 / 200;
	g_stEcoRtv.usPackUVTTThr2 = g_stCfg.stLocal.usPackUVTTThr2 / 200;
	g_stEcoRtv.usODCTTThr1 = g_stCfg.stLocal.usODCTTThr1 / 200;
	g_stEcoRtv.usOCCTTThr1 = g_stCfg.stLocal.usOCCTTThr1 / 200;
	g_stEcoRtv.usOCCRTThr1 = g_stCfg.stLocal.usOCCRTThr1 / 200;
	g_stEcoRtv.usODCRTThr1 = g_stCfg.stLocal.usODCRTThr1 / 200;
	g_stEcoRtv.usSCRTThr = g_stCfg.stLocal.usSCRTThr / 200;
	g_stEcoRtv.usCurLmtMode = g_stCfg.stLocal.usCurLmtMode;
	if(g_stCfg.stLocal.iLowPwrSlpSec < 0) {
		g_stEcoRtv.sLowSocSlpTVThr = 0xFFFF; 
	} else {
		g_stEcoRtv.sLowSocSlpTVThr = ((g_stCfg.stLocal.iLowPwrSlpSec / 60) & 0xFFFF);
	}
//	g_stEcoRtv.usPrlSelfId = g_stPrl.ucSelfId;
//	g_stEcoRtv.usPrlDiffU = g_stLocalArrayRVal.fUDiff;
//	g_stEcoRtv.usStandbySocCaliSec = g_uiSlpTick;
//	g_stEcoRtv.usStandbySocCaliI = g_stCfg.stAfe.fCDATACaliB * 0.001;
//	memcpy(g_stEcoRtv.ausSN, g_stCfg.stLocal.ausHWSN, sizeof(uint16_t) * 4);
	g_stEcoRtv.sHeaterOTRVThr = g_stCfg.stLocal.sHeaterUTRVThr;
	g_stEcoRtv.usSocFCaliU0 = g_stCfg.stLocal.ausSocFCaliU[0];
	g_stEcoRtv.usSocECaliU0 = g_stCfg.stLocal.ausSocECaliU[0];
	g_stEcoRtv.usSocECaliU1 = g_stCfg.stLocal.ausSocECaliU[1];
	g_stEcoRtv.usSocFCaliU1 = g_stCfg.stLocal.ausSocFCaliU[1];
	g_stEcoRtv.usSocECaliU2 = g_stCfg.stLocal.ausSocECaliU[2];
	g_stEcoRtv.usPackRealAH = pstPackRVal->fPackRealAH * 10;
	g_stEcoRtv.usPackLeftAH = pstPackRVal->fPackLeftAH * 10;
	g_stEcoRtv.sBalanceOn = g_stCfg.stLocal.sBalanceOn;
	g_stEcoRtv.usBalanceValue = g_stCfg.stLocal.usBalanceValue;
	g_stEcoRtv.usBalanceDiff = g_stCfg.stLocal.usBalanceDiff;
	g_stEcoRtv.usSocCaliDelay = g_stCfg.stLocal.usSocCaliDelay;
	g_stEcoRtv.usSocCaliCurVal = g_stCfg.stLocal.usSocCaliCurVal;
	if(g_stAfe.uRam.stCode.CHG_FET || pstPackRVal->uBaseStat.stBaseStat.ucChgerON) {
		if(pstPackRVal->uBaseStat.stBaseStat.ucHeating != 0 && pstPackRVal->fCellTMin <= 2) {
			g_stEcoRtv.usReqChgCur = 50;
		} else {
			g_stEcoRtv.usReqChgCur = pstPackRVal->fReqChgI * 10;
		}
	} else {
		g_stEcoRtv.usReqChgCur = 0;
	}
	g_stEcoRtv.usPeakLmtDsgCur = pstPackRVal->fPeakLmtDsgI * 10;
	g_stEcoRtv.usLmtChgCur = pstPackRVal->fLmtChgI * 10;
	g_stEcoRtv.usLmtDsgCur = pstPackRVal->fLmtDsgI * 10;
	g_stEcoRtv.bNeedHeat = g_bNeedHeat;
	g_stEcoRtv.bSetMChgerAct = g_bSetMChgerAct;
	g_stEcoRtv.bSetSChgerAct = g_bSetSChgerAct;
	g_stEcoRtv.ucCanPtcType = g_stCfg.stCom.ucCanPtcType;
	g_stEcoRtv.usTempIsNotRising = g_bTempIsNotRisingFlag;
	g_stEcoRtv.usHeatingOverNumber = g_bHeatingOverNumberFlag;
	g_stEcoRtv.usHeatingTimeout = g_bHeatingTimeoutFlag;
	g_stEcoRtv.fCDATA = pstPackRVal->fCDATA;
	if(g_bGbms3 == false) {
		g_stEcoRtv.usReset = g_stPrl.ucDevNum;
		g_stEcoRtv.usSetDefault = g_stCfg.stLocal.sBalanceOn;
		g_stEcoRtv.usDevNum = 0;
		g_stEcoRtv.ausSN[5] = 0;
		g_stEcoRtv.ausSN[4] = g_stCfg.stLocal.usSocCaliDelay;
		g_stEcoRtv.ausSN[7] = 0;
		g_stEcoRtv.ausSN[6] = g_stCfg.stLocal.usSocCaliCurVal;
		g_stEcoRtv.usSocFCaliU0 = g_stCfg.stLocal.ausSocFCaliU[0];
		g_stEcoRtv.usSocECaliU0 = g_stCfg.stLocal.ausSocECaliU[1];
		g_stEcoRtv.usSocECaliU1 = g_stCfg.stLocal.ausSocECaliU[2];
	}
}

void eco_refresh_CD(void) {
	LOCAL_PACK_RVAL_S* pstPackRVal = g_stLocalArrayRVal.astPackRVal + g_stPrl.ucSelfId;
	g_stEcoCD.bSWOCC = GET_ALM0_CODE(9);
	g_stEcoCD.bSWODC = GET_ALM0_CODE(8);
	g_stEcoCD.bSWCellUDT = GET_ALM0_CODE(13);
	g_stEcoCD.bSWCellUCT = GET_ALM0_CODE(12);
	g_stEcoCD.bSWMOSOT = GET_ALM0_CODE(14);
	g_stEcoCD.bSWOCT = GET_ALM0_CODE(11);
	g_stEcoCD.bSWCellOV = GET_ALM0_CODE(4);
	g_stEcoCD.bSWCellUV = GET_ALM0_CODE(5);
	g_stEcoCD.bSWCellODT = GET_ALM0_CODE(10);
	g_stEcoCD.bSWPackUV = GET_ALM0_CODE(7);
	g_stEcoCD.bSWPackOV = GET_ALM0_CODE(6);
	g_stEcoCD.bHWSC = GET_ALM0_CODE(2);
	g_stEcoCD.bTempSersorErr = pstPackRVal->uErrCode.stErrCode.bTempSensor;
	g_stEcoCD.bVoltSensor = pstPackRVal->uErrCode.stErrCode.bVoltSensor;
	g_stEcoCD.bDMOSOn = g_stEcoLog.bDMOSOn;
	g_stEcoCD.bCMOSOn = g_stEcoLog.bCMOSOn;
	g_stEcoCD.MosAnomaly = GET_ALM0_CODE(1);
	g_stEcoCD.bBAL = g_stEcoLog.bBAL;
	g_stEcoCD.bHMOSOn = g_stEcoLog.bHMOSOn;
	g_stEcoCD.ucSoc = g_stEcoLog.ucSoc;
	g_stEcoCD.ucPackRTVoltHB = (uint16_t)(pstPackRVal->fPackU * 10) >> 8;
	g_stEcoCD.ucPackRTVoltLB = (uint16_t)(pstPackRVal->fPackU * 10) & 0xFF;
	g_stEcoCD.cPackRTCurHB = (int16_t)(pstPackRVal->fPackCur * 10) >> 8;
	g_stEcoCD.ucPackRTCurLB = (int16_t)(pstPackRVal->fPackCur * 10) & 0xFF;
	g_stEcoCD.cPackMaxCurHB = (int16_t)(pstPackRVal->fPackCurMax * 10) >> 8;
	g_stEcoCD.ucPackMaxCurLB = (int16_t)(pstPackRVal->fPackCurMax * 10) & 0xFF;
	g_stEcoCD.ucCellMaxVoltHB = (uint16_t)(pstPackRVal->fCellUMax) >> 8;
	g_stEcoCD.ucCellMaxVoltLB = (uint16_t)(pstPackRVal->fCellUMax) & 0xFF;
	g_stEcoCD.ucCellMinVoltHB = (uint16_t)(pstPackRVal->fCellUMin) >> 8;
	g_stEcoCD.ucCellMinVoltLB = (uint16_t)(pstPackRVal->fCellUMin) & 0xFF;
	g_stEcoCD.cCellMaxTempHB = pstPackRVal->fCellTMax / 256;
	g_stEcoCD.cCellMaxTempLB = (int16_t)(pstPackRVal->fCellTMax) & 0xFF;
	g_stEcoCD.cCellMinTempHB = pstPackRVal->fCellTMin / 256;
	g_stEcoCD.cCellMinTempLB = (int16_t)(pstPackRVal->fCellTMin) & 0xFF;
	g_stEcoCD.cMOSMaxTempHB = pstPackRVal->fMosT / 256;
	g_stEcoCD.cMOSMaxTempLB = (int16_t)(pstPackRVal->fMosT) & 0xFF;
}

bool eco_refresh_log(void){
	LOCAL_PACK_RVAL_S* pstPackRVal = g_stLocalArrayRVal.astPackRVal + g_stPrl.ucSelfId;
	if(pstPackRVal->fCellTMax - pstPackRVal->fCellTMin > 20) {
		g_stEcoLog.bTDIFFW = 1;
	} else {
		g_stEcoLog.bTDIFFW = 0;
	}
	g_stEcoLog.bVDIFFW = GET_ALM0_CODE(3);
	g_stEcoLog.bOCDW = GET_ALM0_CODE(8);
	g_stEcoLog.bOCCW = GET_ALM0_CODE(9);
	g_stEcoLog.bOTCW = GET_ALM0_CODE(11);
	g_stEcoLog.bOTDW = GET_ALM0_CODE(10);
	g_stEcoLog.bOTMW = GET_ALM1_CODE(64);
	g_stEcoLog.bSWOCC = GET_ALM0_CODE(9);
	g_stEcoLog.bSWODC = GET_ALM0_CODE(8);
	g_stEcoLog.bSWCellUDT = GET_ALM0_CODE(13);
	g_stEcoLog.bSWCellUCT = GET_ALM0_CODE(12);
	g_stEcoLog.bSWMOSOT = GET_ALM0_CODE(14);
	g_stEcoLog.bsWSocLow = GET_ALM0_CODE(15);
	g_stEcoLog.bSWOCT = GET_ALM0_CODE(11);
	if(GET_ALM0_CODE(4)) {
		g_stEcoLog.bSWCellOV = GET_ALM0_CODE(4);
	} else if(GET_ALM1_CODE(58)) {
		g_stEcoLog.bSWCellOV = GET_ALM1_CODE(58);
	} else {
		g_stEcoLog.bSWCellOV = 0;
	}
	g_stEcoLog.bSWCellUV = GET_ALM0_CODE(5);
	g_stEcoLog.bSWCellODT = GET_ALM0_CODE(10);
	g_stEcoLog.bSWPackUV = GET_ALM0_CODE(7);
	g_stEcoLog.bSWPackOV = GET_ALM0_CODE(6);
	g_stEcoLog.bHWSC = GET_ALM0_CODE(2);
	g_stEcoLog.MosAnomaly = GET_ALM0_CODE(1);
	g_stEcoLog.bTempSersorErr = pstPackRVal->uErrCode.stErrCode.bTempSensor;
	g_stEcoLog.bVoltSensor = pstPackRVal->uErrCode.stErrCode.bVoltSensor;
	g_stEcoLog.bDMOSOn = g_stAfe.uRam.stCode.DSG_FET;
	g_stEcoLog.bCMOSOn = g_stAfe.uRam.stCode.CHG_FET;
//	if(!g_bMChging) {
//		pstPackRVal->uBaseStat.stBaseStat.ucHeating = 0;
//	}
	g_stEcoLog.bHMOSOn = pstPackRVal->uBaseStat.stBaseStat.ucHeating;
	g_stEcoLog.ucSoc = (uint8_t)pstPackRVal->fPackSoc;
 	g_stEcoLog.cCurHB = (int16_t)(pstPackRVal->fPackCur * 10) >> 8;
	g_stEcoLog.ucCurLB = (int16_t)(pstPackRVal->fPackCur * 10) & 0xFF;
	for(uint8_t i=0;i<CFG_CELL_NUM;i++) {
		g_stEcoLog.ausCell[i] = pstPackRVal->afCellU[i] * 1000;
	}
	if(g_bTempIsNotRisingFlag) {
		g_stEcoLog.bTINR = 1;
	} else {
		g_stEcoLog.bTINR = 0;
	}
	if(g_bHeatingTimeoutFlag) {
		g_stEcoLog.bHT = 1;
	} else {
		g_stEcoLog.bHT = 0;
	}
	if(g_bHeatingOverNumberFlag) {
		g_stEcoLog.bHON = 1;
	} else {
		g_stEcoLog.bHON = 0;
	}
	g_stEcoLog.asTemp[0] = pstPackRVal->fMosT;
	g_stEcoLog.asTemp[1] = pstPackRVal->afCellT[0];
	g_stEcoLog.asTemp[2] = pstPackRVal->afCellT[1];
	g_stEcoLog.asTemp[3] = pstPackRVal->afCellT[2];
	g_stEcoLog.ausSN[0] = 30;
	g_stEcoLog.ausSN[1] = g_stCfg.stLocal.ucSVerVMajor;
	g_stEcoLog.ausSN[2] = g_stCfg.stLocal.ucSVerVMinor;
	g_stEcoLog.ausSN[3] = g_stCfg.stLocal.ucSVerRMajor * 10 + g_stCfg.stLocal.ucSVerRMinor;
	g_stEcoLog.ausSN[4] = g_stCfg.stLocal.ucSVerAgree - 0x40;
	g_stEcoLog.ausSN[5] = 3;
  for(uint8_t i=1; i<=15; i++){
		if(i == 3){
			continue;
		}
		if(GET_ALM0_CODE(i)) {
      return i;
		}		
	}
//	if(GET_ALM0_CODE(15) == 1){
//		return 15;
//	}
	if(GET_ALM1_CODE(53) == 1){
		return 53;
	}
	if(GET_ALM1_CODE(58) == 1){
		return 58;
	}
	if(GET_ALM1_CODE(64) ==1) {
		return 64;
	}
	return false;
}

