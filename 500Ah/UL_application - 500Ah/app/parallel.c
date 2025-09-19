#include "parallel.h"
#include "local.h"
#include "history.h"
#include "bsp_gd25q32.h"
#include "protocol.h"
#include "bsp_gpio.h"
#include "bsp_rtc.h"
#include "bsp_can.h"
#include "systick.h"
#include "bsp_exti.h"
#include "bsp_wdg.h"
#include "bsp_sh367309.h"
#include "eco.h"
#include "bsp.h"
#include "local.h"
#include "ptc_eco.h"

#include <stdio.h>
#include <math.h>
#include <string.h>
#include <systick.h>

PRL_S g_stPrl = {0};
uint32_t g_auiIEMI_IDs[PRL_MAX_NODE_NUM] = {0};
uint16_t g_ausPrlComTick[PRL_MAX_NODE_NUM] = {0};
extern ECO_RTV_S g_stEcoRtv_Parallel[PRL_MAX_NODE_NUM + 1];
uint32_t g_uiSlpTick = 0;
bool g_bNeedSleep = false;
uint32_t g_eCanIdParallel = 0;
bool g_eCanIdParallelFlag = 0;

/*******************************************************************************
* Function Name  : prl_host
* Description    : Determine whether it is a host
* Input          : None
* Output         : None
* Return         : result, 1 The machine is the host 0 The machine is not the host
*******************************************************************************/
bool prl_host() {
	if(0 == g_stPrl.ucSelfId && g_stPrl.ucDevNum > 1) {
		return true;
	} else {
		return false;
	}
}

/*******************************************************************************
* Function Name  : prl_client
* Description    : Determine whether it is a slave
* Input          : None
* Output         : None
* Return         : result, 1 This machine is a slave machine 0 This machine is not a slave machine
*******************************************************************************/
bool prl_client() {
	if(g_stPrl.ucSelfId > 0 && g_stPrl.ucSelfId < PRL_MAX_NODE_NUM) {
		return true;
	} else {
		return false;
	}
}

/*******************************************************************************
* Function Name  : prl_single
* Description    : Determine whether it is a stand-alone machine
* Input          : None
* Output         : None
* Return         : result, 1. This machine is a stand-alone machine, 0 This machine is not a stand-alone machine
*******************************************************************************/
bool prl_single() {
	if(0 == g_stPrl.ucSelfId && 1 == g_stPrl.ucDevNum) {
		return true;
	} else {
		return false;
	}
}

void prl_single_auto_send() {
	uint32_t uiId;
	uint8_t aucBuf[8];
	LOCAL_PACK_RVAL_S* pstPackRVal = g_stLocalArrayRVal.astPackRVal + g_stPrl.ucSelfId;
	uint16_t usVal;
	/* BMS Limits, 0xE005, motorola */
	uiId = eCanIdBMSLmt;
	memset(aucBuf, 0, 8);
	usVal = (uint16_t)(pstPackRVal->fPeakLmtDsgI * 10);
	aucBuf[0] = (usVal >> 8) & 0xFF;
	aucBuf[1] = usVal & 0xFF;
	aucBuf[2] = pstPackRVal->fMosT + 40;
	aucBuf[3] = 0;
	usVal = (uint16_t)(pstPackRVal->fLmtDsgI * 10);
	aucBuf[4] = (usVal >> 8) & 0xFF;
	aucBuf[5] = usVal & 0xFF;
	usVal = (uint16_t)(pstPackRVal->fLmtChgI * 10);
	aucBuf[6] = (usVal >> 8) & 0xFF;
	aucBuf[7] = usVal & 0xFF;
	CAN0_SendMsg(uiId, aucBuf, 8);
	delay_1ms(2);
	{	
		static uint8_t s_ucTick = 0;
	/* Charge Request 1, 0x1806E5F4, motorola */
		uiId = eCanIdChgReq0;
		memset(aucBuf, 0, 8);
		usVal = pstPackRVal->fPackSoc * 100;
		aucBuf[0] = (usVal >> 8) & 0xFF;
		aucBuf[1] = usVal & 0xFF;
	//	if(g_ucChgFull == 0) {
		if(pstPackRVal->fPackSoc < 100) {
			aucBuf[2] = 1;
			aucBuf[3] = 0;
			usVal = (uint16_t)(pstPackRVal->fReqChgI * 10);
			aucBuf[4] = (usVal >> 8) & 0xFF;
			aucBuf[5] = usVal & 0xFF;
		}
		if(round(pstPackRVal->fCellTMin) < g_stCfg.stLocal.sCellUCTTVThr1 || pstPackRVal->uErrCode.stErrCode.bTempSensor) {
			aucBuf[2] = 0;
			aucBuf[3] = 0;
			aucBuf[4] = 0;
			aucBuf[5] = 0;
		}
		if(g_bSetMChgerAct && pstPackRVal->uBaseStat.stBaseStat.ucHeating != 0) {
			aucBuf[2] = 1;
			aucBuf[3] = 0;
			aucBuf[4] = 0x00;
			aucBuf[5] = 0x32;
		}
		if(g_bSChgerComAct && !g_bMChgerComAct) {
			aucBuf[2] = 0;
			aucBuf[3] = 0;
			aucBuf[4] = 0;
			aucBuf[5] = 0;
		}
		if(g_bSChgerComAct && g_bMChgerComAct) {
			if(s_ucTick * g_stCfg.stLocal.usCyclePeriod < 5000) {
				s_ucTick++;
				aucBuf[2] = 0;
				aucBuf[3] = 0;
				aucBuf[4] = 0;
				aucBuf[5] = 0;
			}
		}
		usVal = 560;
		aucBuf[6] = (usVal >> 8) & 0xFF;
		aucBuf[7] = usVal & 0xFF;
		CAN0_SendMsg(uiId, aucBuf, 8);
		delay_1ms(2);
		/* Charge Request 2, 0x273, intel */
		uiId = eCanIdChgReq20;
		memset(aucBuf, 0, 8);
		if(pstPackRVal->uBaseStat.stBaseStat.ucHeating != 0) {
			usVal = 50;
		} else {
			usVal = (uint16_t)(pstPackRVal->fReqChgI * 10);
		}
		aucBuf[0] = usVal & 0xFF;
		aucBuf[1] = (usVal >> 8) & 0xFF;
		usVal = 5600;
		aucBuf[2] = usVal & 0xFF;
		aucBuf[3] = (usVal >> 8) & 0xFF;
		usVal = (uint16_t)(pstPackRVal->fPackU * 10);
		aucBuf[4] = usVal & 0xFF;
		aucBuf[5] = (usVal >> 8) & 0xFF;
		aucBuf[6] = pstPackRVal->fPackSoc;
		if(g_bMChgerComAct) {
			aucBuf[7] = 0x02;
		} 
		if(pstPackRVal->fReqChgI > 0 && g_bSChgerComAct) {
			aucBuf[7] += 1;
		}
		if(g_bMChgerComAct && !g_bSChgerComAct) {
			aucBuf[0] = 0;
			aucBuf[1] = 0;
		}
		if(g_bSChgerComAct && g_bMChgerComAct) {
			aucBuf[0] = 0;
			aucBuf[1] = 0;
			aucBuf[7] = 0x02;
		} else {
			s_ucTick = 0;
		}
		if(!g_bSChgerComAct && !g_bMChgerComAct) {
			aucBuf[7] += 1;
		}
		if(pstPackRVal->fPackSoc == 100 || pstPackRVal->uErrCode.stErrCode.bTempSensor) {
			aucBuf[0] = 0;
			aucBuf[1] = 0;
			aucBuf[7] = 0x00;
		}
		CAN0_SendMsg(uiId, aucBuf, 8);
		delay_1ms(2);
	}
	/* BMS Info, 0xE000, motorola */
	uiId = eCanIdBmsInf;
	memset(aucBuf, 0, 8);
	usVal = pstPackRVal->fPackSoc * 100;
	aucBuf[0] = (usVal >> 8) & 0xFF;
	aucBuf[1] = usVal & 0xFF;
	aucBuf[2] = (usVal >> 8) & 0xFF;
	aucBuf[3] = usVal & 0xFF;
	usVal = (uint16_t)(pstPackRVal->fPackU * 10);
	aucBuf[4] = (usVal >> 8) & 0xFF;
	aucBuf[5] = usVal & 0xFF;
	usVal = (int16_t)(pstPackRVal->fPackCur * 10);
	aucBuf[6] = (usVal >> 8) & 0xFF;
	aucBuf[7] = usVal & 0xFF;
	CAN0_SendMsg(uiId, aucBuf, 8);
	delay_1ms(2);
	//return;
	/* Soc, 0xE004, motorola */
	uiId = eCanIdSoc;
	memset(aucBuf, 0, 8);
	aucBuf[0] = (int8_t)pstPackRVal->fCellTMax;
	aucBuf[1] = (int8_t)pstPackRVal->fCellTMin;
	usVal = (uint16_t)(pstPackRVal->fPackRealAH * 10);
	aucBuf[2] = (usVal >> 8) & 0xFF;
	aucBuf[3] = usVal & 0xFF;
	usVal = (uint16_t)(g_stCfg.stLocal.usDesignAH * 10);
	aucBuf[4] = (usVal >> 8) & 0xFF;
	aucBuf[5] = usVal & 0xFF;
	usVal = (uint16_t)(pstPackRVal->fPackLeftAH * 10);
	aucBuf[6] = (usVal >> 8) & 0xFF;
	aucBuf[7] = usVal & 0xFF;
	CAN0_SendMsg(uiId, aucBuf, 8);
	delay_1ms(2);
	/* Cell Voltage 0, 0xE006, motorola */
	uiId = eCanIdCellVolt0;
	memset(aucBuf, 0, 8);
	usVal = (uint16_t)(pstPackRVal->afCellU[0] * 1000);
	aucBuf[0] = (usVal >> 8) & 0xFF;
	aucBuf[1] = usVal & 0xFF;
	usVal = (uint16_t)(pstPackRVal->afCellU[1] * 1000);
	aucBuf[2] = (usVal >> 8) & 0xFF;
	aucBuf[3] = usVal & 0xFF;
	usVal = (uint16_t)(pstPackRVal->afCellU[2] * 1000);
	aucBuf[4] = (usVal >> 8) & 0xFF;
	aucBuf[5] = usVal & 0xFF;
	usVal = (uint16_t)(pstPackRVal->afCellU[3] * 1000);
	aucBuf[6] = (usVal >> 8) & 0xFF;
	aucBuf[7] = usVal & 0xFF;
	CAN0_SendMsg(uiId, aucBuf, 8);
	delay_1ms(2);
	/* Cell Voltage 1, 0xE007, motorola */
	uiId = eCanIdCellVolt1;
	memset(aucBuf, 0, 8);
	usVal = (uint16_t)(pstPackRVal->afCellU[4] * 1000);
	aucBuf[0] = (usVal >> 8) & 0xFF;
	aucBuf[1] = usVal & 0xFF;
	usVal = (uint16_t)(pstPackRVal->afCellU[5] * 1000);
	aucBuf[2] = (usVal >> 8) & 0xFF;
	aucBuf[3] = usVal & 0xFF;
	usVal = (uint16_t)(pstPackRVal->afCellU[6] * 1000);
	aucBuf[4] = (usVal >> 8) & 0xFF;
	aucBuf[5] = usVal & 0xFF;
	usVal = (uint16_t)(pstPackRVal->afCellU[7] * 1000);
	aucBuf[6] = (usVal >> 8) & 0xFF;
	aucBuf[7] = usVal & 0xFF;
	CAN0_SendMsg(uiId, aucBuf, 8);
	delay_1ms(2);
	/* Cell Voltage 2, 0xE008, motorola */
	uiId = eCanIdCellVolt2;
	memset(aucBuf, 0, 8);
	usVal = (uint16_t)(pstPackRVal->afCellU[8] * 1000);
	aucBuf[0] = (usVal >> 8) & 0xFF;
	aucBuf[1] = usVal & 0xFF;
	usVal = (uint16_t)(pstPackRVal->afCellU[9] * 1000);
	aucBuf[2] = (usVal >> 8) & 0xFF;
	aucBuf[3] = usVal & 0xFF;
	usVal = (uint16_t)(pstPackRVal->afCellU[10] * 1000);
	aucBuf[4] = (usVal >> 8) & 0xFF;
	aucBuf[5] = usVal & 0xFF;
	usVal = (uint16_t)(pstPackRVal->afCellU[11] * 1000);
	aucBuf[6] = (usVal >> 8) & 0xFF;
	aucBuf[7] = usVal & 0xFF;
	CAN0_SendMsg(uiId, aucBuf, 8);
	delay_1ms(2);
	/* Cell Voltage 3, 0xE009, motorola */
	uiId = eCanIdCellVolt3;
	memset(aucBuf, 0, 8);
	usVal = (uint16_t)(pstPackRVal->afCellU[12] * 1000);
	aucBuf[0] = (usVal >> 8) & 0xFF;
	aucBuf[1] = usVal & 0xFF;
	usVal = (uint16_t)(pstPackRVal->afCellU[13] * 1000);
	aucBuf[2] = (usVal >> 8) & 0xFF;
	aucBuf[3] = usVal & 0xFF;
	usVal = (uint16_t)(pstPackRVal->afCellU[14] * 1000);
	aucBuf[4] = (usVal >> 8) & 0xFF;
	aucBuf[5] = usVal & 0xFF;
	usVal = (uint16_t)(pstPackRVal->afCellU[15] * 1000);
	aucBuf[6] = (usVal >> 8) & 0xFF;
	aucBuf[7] = usVal & 0xFF;
	CAN0_SendMsg(uiId, aucBuf, 8);
	delay_1ms(2);
	/* Cell Voltage 4, 0xE010, motorola */
	uiId = eCanIdCellVolt4;
	memset(aucBuf, 0, 8);
	CAN0_SendMsg(uiId, aucBuf, 8);
	delay_1ms(2);
	/* Cell Voltage 5, 0xE011, motorola */
	uiId = eCanIdCellVolt5;
	CAN0_SendMsg(uiId, aucBuf, 8);
	delay_1ms(2);
	/* Cells Average Voltage, 0xE012, motorola */
	uiId = eCanIdCellVoltAvg;
//	memset(aucBuf, 0, 8);
	usVal = 0;
	for(uint8_t i=0;i<CFG_CELL_NUM;i++) {
		usVal += (pstPackRVal->afCellU[i] * 1000);
	}
	usVal /= CFG_CELL_NUM;
	aucBuf[0] = (usVal >> 8) & 0xFF;
	aucBuf[1] = usVal & 0xFF;
	usVal = (uint16_t)((pstPackRVal->fCellUMax - pstPackRVal->fCellUMin) * 1000);
	aucBuf[2] = (usVal >> 8) & 0xFF;
	aucBuf[3] = usVal & 0xFF;
	CAN0_SendMsg(uiId, aucBuf, 8);
	delay_1ms(2);
	/* Fault Info., 0xF001, motorola */
	uiId = eCanIdFaultInf;
	memset(aucBuf, 0, 8);
	if(GET_ALM0_CODE(1)) {
		aucBuf[0] |= 0x01;
	}
	if(GET_ALM0_CODE(2)) {
		aucBuf[0] |= 0x04;
	}
	if(GET_ALM0_CODE(3)) {
		aucBuf[0] |= 0x10;
	}
	if(GET_ALM0_CODE(4)) {
		aucBuf[0] |= 0x80;
	}
	if(GET_ALM0_CODE(5)) {
		aucBuf[1] |= 0x02;
	}
//	if(GET_ALM0_CODE(6)) {
//		aucBuf[1] |= 0x08;
//	}
	if(GET_ALM0_CODE(7)) {
		aucBuf[1] |= 0x20;
	}
	if(GET_ALM0_CODE(8)) {
		aucBuf[1] |= 0xC0;
	}
	if(GET_ALM0_CODE(9)) {
		aucBuf[2] |= 0x03;
	}
	if(GET_ALM0_CODE(10)) {
		aucBuf[2] |= 0x0C;
	}
	if(GET_ALM0_CODE(11)) {
		aucBuf[2] |= 0x30;
	}
	if(GET_ALM0_CODE(12)) {
		aucBuf[2] |= 0xC0;
	}
	if(GET_ALM0_CODE(13)) {
		aucBuf[3] |= 0x03;
	}
	if(GET_ALM0_CODE(14) && GET_ALM0_CODE(14)) {
		aucBuf[3] |= 0x0C;
	}
	if(GET_ALM0_CODE(15)) {
		aucBuf[3] |= 0x30;
	}
	if(GET_ALM0_CODE(16)) {
		aucBuf[3] |= 0xC0;
	}
	if(GET_ALM0_CODE(17)) {
		aucBuf[4] |= 0x03;
	}
	CAN0_SendMsg(uiId, aucBuf, 8);
	delay_1ms(2);
	/* eCanIdChgReq14, 0x18FF28F4, motorola */
	uiId = eCanIdChgReq14;
	memset(aucBuf, 0, 8);
	if(pstPackRVal->uBaseStat.stBaseStat.ucChgerON) {
		aucBuf[0] |= 0x01;
	}
	if(g_stAfe.uRam.stCode.DSG_FET && pstPackRVal->ucDsgForceEn == 0) {
		aucBuf[0] |= 0x10;
	}
	aucBuf[1] = pstPackRVal->fPackSoc;
	aucBuf[4] = (uint16_t)(pstPackRVal->fPackU * 10) >> 8;
	aucBuf[5] = (uint16_t)(pstPackRVal->fPackU * 10) & 0xFF;
	aucBuf[6] = g_ucAlmLevel;
	{
		static uint8_t s_uctick = 1, uc_tick = 0;
		for(uint8_t i=s_uctick; i<65; i++){
			if(i == 6 || (i > 17 && i < 53) || (i > 57 && i < 64)) {
				continue;
			}
 			if(GET_ALM1_CODE(i)) {
				aucBuf[7] = i;
				CAN0_SendMsg(uiId, aucBuf, 8);
				delay_1ms(2);	
				s_uctick = i;
				uc_tick++;
				if(uc_tick > 10){
					uc_tick = 0;
					s_uctick = i + 1;
				}
				if(s_uctick == 65){
					s_uctick = 1;
				}
				break;
			}		
		}
		if(aucBuf[7] == 0) {
			if(s_uctick == 1){
				CAN0_SendMsg(uiId, aucBuf, 8);
				delay_1ms(2);
			}
			s_uctick = 1;
		}
  }
	/* eCanIdChgReq15, 0x18FE28F4, motorola */
	uiId = eCanIdChgReq15;
	memset(aucBuf, 0, 8);
	usVal = (uint16_t)(pstPackRVal->fCellUMax * 1000);
	aucBuf[0] = usVal >> 8;
	aucBuf[1] = usVal & 0xFF;
	usVal = (uint16_t)(pstPackRVal->fCellUMin * 1000);
	aucBuf[2] = usVal >> 8;
	aucBuf[3] = usVal & 0xFF;
	aucBuf[4] = (uint8_t)(pstPackRVal->fCellTMax + 40);
	aucBuf[5] = (uint8_t)(pstPackRVal->fCellTMin + 40);
	CAN0_SendMsg(uiId, aucBuf, 8);
	delay_1ms(2);
	/* eCanIdChgReq16, 0x18FD28F4, motorola */
	uiId = eCanIdChgReq14;
	memset(aucBuf, 0, 8);
//	usVal = (uint16_t)(pstPackRVal->fPeakLmtDsgI * 10);  //A30 version changes
//	aucBuf[0] = usVal >> 8;
//	aucBuf[1] = usVal & 0xFF;
//	usVal = (uint16_t)(-pstPackRVal->fLmtChgI * 10);     //A30 version changes
//	aucBuf[2] = usVal >> 8;
//	aucBuf[3] = usVal & 0xFF;
	usVal = pstPackRVal->fPeakLmtDsgI * 10;  //A33 version changes
	aucBuf[0] = usVal >> 8;
	aucBuf[1] = usVal & 0xFF;
	usVal = -pstPackRVal->fLmtChgI * 10;     //A33 version changes
	aucBuf[2] = usVal >> 8;
	aucBuf[3] = usVal & 0xFF;
	CAN0_SendMsg(uiId, aucBuf, 8);
	delay_1ms(2);
	/* Begin :  Modified by dgx, 2024.4.2 */
	/* eCanIdEx5, 0x1E1, motorola */
	uiId = eCanIdEx5;
	memset(aucBuf, 0, 8);
	usVal = pstPackRVal->fPackU * 10;
	aucBuf[0] = usVal >> 8;
	aucBuf[1] = usVal & 0xFF;
	usVal = (pstPackRVal->fPackCur + 1000) * 10;
	aucBuf[2] = usVal >> 8;
	aucBuf[3] = usVal & 0xFF;
	usVal = pstPackRVal->fPackSoc * 10;
	aucBuf[4] = usVal >> 8;
	aucBuf[5] = usVal & 0xFF;
	aucBuf[6] = usVal >> 8;
	aucBuf[7] = usVal & 0xFF;
	CAN0_SendMsg(uiId, aucBuf, 8);
	delay_1ms(2);
	/* eCanIdEx6, 0x1F5, motorola */
	uiId = eCanIdEx6;
	memset(aucBuf, 0, 8);
	aucBuf[0] = g_ucAlmLevel;
	CAN0_SendMsg(uiId, aucBuf, 8);
	delay_1ms(2);
	/* eCanIdEx7, 0x800A6A9, intel */
	{
		uiId = eCanIdEx7;
		memset(aucBuf, 0, 8);
		static uint8_t s_ucF_I_Num = 0;
		static uint8_t s_ucF_II_Num = 0;
		static uint8_t s_ucF_III_Num = 0;
		if(GET_ALM0_CODE(9)) {
			aucBuf[4] |= 0x01;
			s_ucF_III_Num += 1;
		}
		if(GET_ALM0_CODE(8)) {
			aucBuf[4] |= 0x02;
			s_ucF_III_Num += 1;
		}
		if(GET_ALM0_CODE(6) || GET_ALM1_CODE(6)) {
			aucBuf[4] |= 0x04;
			s_ucF_III_Num += 1;
		} else if(GET_ALM1_CODE(56)) {
			aucBuf[2] |= 0x04;
			s_ucF_II_Num += 1;
		}
		if(GET_ALM0_CODE(7) || GET_ALM1_CODE(7)) {
			aucBuf[4] |= 0x08;
			s_ucF_III_Num += 1;
		} else if(GET_ALM1_CODE(57)) {
			aucBuf[2] |= 0x08;
			s_ucF_II_Num += 1;
		}
		if(GET_ALM0_CODE(15) || GET_ALM1_CODE(15)) {
			aucBuf[4] |= 0x20;
			s_ucF_III_Num += 1;
		}
		if(GET_ALM0_CODE(5) || GET_ALM1_CODE(5)) {
			aucBuf[4] |= 0x40;
			s_ucF_III_Num += 1;
		} else if(GET_ALM1_CODE(55)) {
			aucBuf[2] |= 0x40;
			s_ucF_II_Num += 1;
		}
		if(GET_ALM0_CODE(4) || GET_ALM1_CODE(4)) {
			aucBuf[4] |= 0x80;
			s_ucF_III_Num += 1;
		} else if(GET_ALM1_CODE(54)) {
			aucBuf[2] |= 0x80;
			s_ucF_II_Num += 1;
		}
//		if(GET_ALM0_CODE(12) || GET_ALM1_CODE(12) || GET_ALM0_CODE(13) || GET_ALM1_CODE(13)) {
//			aucBuf[1] |= 0x01;
//			s_ucF_III_Num += 1;
//		}
		if(GET_ALM0_CODE(10) || GET_ALM1_CODE(10) || GET_ALM0_CODE(11) || GET_ALM1_CODE(11)) {
			aucBuf[5] |= 0x01;
			s_ucF_III_Num += 1;
		}
		if(GET_ALM0_CODE(17) || GET_ALM1_CODE(17)) {
			aucBuf[5] |= 0x20;
			s_ucF_III_Num += 1;
		}
		if(pstPackRVal->uErrCode.stErrCode.bTempSensor == 1) {
			aucBuf[1] |= 0x10;
			s_ucF_I_Num += 1;
		}
		if(pstPackRVal->uErrCode.stErrCode.bVoltSensor == 1) {
			aucBuf[1] |= 0x20;
			s_ucF_I_Num += 1;
		}
//		if(GET_ALM0_CODE(9)) {
//			aucBuf[4] |= 0x01; 
//			s_ucF_I_Num += 1;
//		}
//		if(GET_ALM0_CODE(8)) {
//			aucBuf[4] |= 0x02; 
//			s_ucF_I_Num += 1;
//		}
		aucBuf[6] |= s_ucF_I_Num;
		aucBuf[6] |= (s_ucF_II_Num << 4);
		aucBuf[7] = s_ucF_III_Num;
		CAN0_SendMsg(uiId, aucBuf, 8);
		delay_1ms(2);
		s_ucF_I_Num = 0;
		s_ucF_II_Num = 0;
		s_ucF_III_Num = 0;
	}
	/* eCanIdEx8, 0x1000A6A9, intel */
	uiId = eCanIdEx8;
	memset(aucBuf, 0, 8);
	usVal = pstPackRVal->fPackU * 10;
	aucBuf[0] = usVal & 0xFF;
	aucBuf[1] = usVal >> 8;
	usVal = (pstPackRVal->fPackCur + 500) * 10;
	aucBuf[2] = usVal & 0xFF;
	aucBuf[3] = usVal >> 8;
	aucBuf[4] = pstPackRVal->fPackSoc;
	aucBuf[5] = pstPackRVal->fCellTMax + 50;
	aucBuf[6] = pstPackRVal->fCellTMin + 50;
	usVal = 0;
	for(uint8_t i = 0; i < CFG_TMP_NUM; i++) {
		usVal += pstPackRVal->afCellT[i];
	}
	aucBuf[7] = usVal / CFG_TMP_NUM + 50;
	CAN0_SendMsg(uiId, aucBuf, 8);
	delay_1ms(2);
	/* eCanIdEx9, 0x1C00A6A9, intel */
	uiId = eCanIdEx9;
	memset(aucBuf, 0, 8);
	if(pstPackRVal->uBaseStat.stBaseStat.ucChgerON == 1) {
		aucBuf[0] |= 0x02;
	}
	if(pstPackRVal->ucDsgEn == 0x55 || pstPackRVal->ucDsgForceEn == 0x55) {
		aucBuf[0] |= 0x10;
	}
	if(pstPackRVal->ucChgEn == 0x55 || pstPackRVal->ucChgForceEn ==0x55) {
		aucBuf[0] |= 0x20;
	}
	if(pstPackRVal->uBaseStat.stBaseStat.ucChgerON == 1 && pstPackRVal->uBaseStat.stBaseStat.ucHeating != 0) {
		aucBuf[1] |= 0x01;
	}
	if(g_eBaseStat == eBStatWorking && pstPackRVal->uBaseStat.stBaseStat.ucChgerON == 1) {
		aucBuf[2] |= 0x02;
	} else if(g_eBaseStat == eBStatWorking && pstPackRVal->uBaseStat.stBaseStat.ucChgerON != 1) {
		aucBuf[2] |= 0x01;
	}
	if(pstPackRVal->uBaseStat.stBaseStat.ucChgerON == 1) {
		aucBuf[3] = (pstPackRVal->fPackRealAH - pstPackRVal->fPackLeftAH) / pstPackRVal->fPackCur * 60;
	}
	aucBuf[5] = 0x03;
	CAN0_SendMsg(uiId, aucBuf, 8);
	delay_1ms(2);
	/* End :  Modified by dgx, 2024.4.2 */
	/* eCanIdEx3, 0x4D4, intel */
	uiId = eCanIdEx3;
	memset(aucBuf, 0, 8);
	aucBuf[2] = pstPackRVal->fPackSoc;
	usVal = (uint16_t)(pstPackRVal->fPackCur * 10 + 32000);
	aucBuf[3] = usVal & 0xFF;
	aucBuf[4] = usVal >> 8;
	usVal = (uint16_t)(pstPackRVal->fPackU * 10);
	aucBuf[5] = usVal & 0xFF;
	aucBuf[6] = usVal >> 8;
	if(g_bMChgerComAct == true) {
		aucBuf[7] = 1;
	} else {
		aucBuf[7] = 3;
	}
	{
		static uint8_t s_uctick = 1, uc_tick = 0;
		for(uint8_t i=s_uctick; i<65; i++){
			if((i > 17 && i < 53) || (i > 57 && i < 64)) {
				continue;
			}
 			if(GET_ALM1_CODE(i)) {
				aucBuf[0] = i;
				CAN0_SendMsg(uiId, aucBuf, 8);
				delay_1ms(2);	
				s_uctick = i;
				uc_tick++;
				if(uc_tick > 10){
					uc_tick = 0;
					s_uctick = i + 1;
				}
				if(s_uctick == 65){
					s_uctick = 1;
				}
				break;
			}		
		}
		
		if(aucBuf[0] == 0) {
			if(s_uctick == 1){
				CAN0_SendMsg(uiId, aucBuf, 8);
				delay_1ms(2);
			}
			s_uctick = 1;
		}
  }
	/* eCanIdEx4, 0x4D5, intel */
	uiId = eCanIdEx4;
	memset(aucBuf, 0, 8);
	usVal = (uint16_t)(pstPackRVal->fCellUMin * 1000);
	aucBuf[0] = usVal & 0xFF;
	aucBuf[1] = usVal >> 8;
	usVal = (uint16_t)(pstPackRVal->fCellUMax * 1000);
	aucBuf[2] = usVal & 0xFF;
	aucBuf[3] = usVal >> 8;
	aucBuf[4] = (uint8_t)(pstPackRVal->fCellTMin + 40);
	aucBuf[5] = (uint8_t)(pstPackRVal->fCellTMax + 40);
	CAN0_SendMsg(uiId, aucBuf, 8);
	delay_1ms(2);
	/* eCanIdEx5, 0x273, intel */
//	uiId = eCanIdEx5;
//	memset(aucBuf, 0, 8);
//	usVal = (uint16_t)(pstPackRVal->fReqChgI * 10);
//	aucBuf[0] = usVal & 0xFF;
//	aucBuf[1] = usVal >> 8;
//	usVal = 5600;
//	aucBuf[2] = usVal & 0xFF;
//	aucBuf[3] = usVal >> 8;
//	CAN0_SendMsg(uiId, aucBuf, 8);
//	delay_1ms(2);
	if(g_bs18F880F3Act == true) {
	/* eCanIdEx0, 0x108, intel */
		uiId = eCanIdEx0;
		memset(aucBuf, 0, 8);
		aucBuf[0] = 1;
		usVal = (uint16_t)(pstPackRVal->fPackCur * 10 + 10000);
		aucBuf[1] = usVal & 0xFF;
		aucBuf[2] = usVal >> 8;
		aucBuf[3] = pstPackRVal->fPackSoc;
		aucBuf[4] = (uint8_t)(pstPackRVal->fCellTMax + 40);
		aucBuf[5] = (uint8_t)(pstPackRVal->fCellTMin + 40);
		if(pstPackRVal->ucChgEn == 0x55 && (pstPackRVal->ucChgForceEn == 0x55 || pstPackRVal->ucChgForceEn == 0)) {
			aucBuf[6] |= 0x01;
		}
		if(pstPackRVal->ucDsgEn == 0x55 && (pstPackRVal->ucDsgForceEn == 0x55 || pstPackRVal->ucDsgForceEn == 0)) {
			aucBuf[6] |= 0x04;
		}
		if(pstPackRVal->uBaseStat.stBaseStat.ucHeating) {
			aucBuf[6] |= 0x10;
		}
		if(pstPackRVal->uBaseStat.stBaseStat.ucDCOut1) {
			aucBuf[6] |= 0x40;
		}
		if(prl_host()) {
			aucBuf[6] |= 0x01;
			for(uint8_t i=0;i<g_stPrl.ucDevNum;i++) {
				if(g_ausPrlComTick[i] == 0) {
					aucBuf[6] |= 0x03;
					break;
				}
			}
		}
		if(pstPackRVal->uBaseStat.stBaseStat.ucChgerON) {
			aucBuf[7] |= 0xC0;
		}
		CAN0_SendMsg(uiId, aucBuf, 8);
		delay_1ms(2);
	/* eCanIdEx1, 0x110, intel */
		uiId = eCanIdEx1;
		memset(aucBuf, 0, 8);
		usVal = (uint16_t)(pstPackRVal->fCellUMax * 1000);
		aucBuf[0] = usVal & 0xFF;
		aucBuf[1] = usVal >> 8;
		aucBuf[2] = pstPackRVal->ucCellUMaxId + 1;
		usVal = (uint16_t)(pstPackRVal->fCellUMin * 1000);
		aucBuf[3] = usVal & 0xFF;
		aucBuf[4] = usVal >> 8;
		aucBuf[5] = pstPackRVal->ucCellUMinId + 1;
		usVal = (uint16_t)(pstPackRVal->fPackU * 100);
		aucBuf[6] = usVal & 0xFF;
		aucBuf[7] = usVal >> 8;
		CAN0_SendMsg(uiId, aucBuf, 8);
		delay_1ms(2);
	/* eCanIdEx2, 0x111, intel */
		uiId = eCanIdEx2;
		memset(aucBuf, 0, 8);
		{
			for(uint8_t i=1; i<17; i++){
				if(i==4 || i==6 || i==8 || i==9 || i==12){
					continue;
				}
				if(GET_ALM0_CODE(i)){
					aucBuf[4] = 0xFF;
				}
			}
		}
		CAN0_SendMsg(uiId, aucBuf, 8);
		delay_1ms(2);
	}
	/* eCanIdChgReq4, 0x18F810F3, intel */
	uiId = eCanIdChgReq4;
	memset(aucBuf, 0, 8);
	usVal = (uint16_t)(pstPackRVal->fPackU * 10);
	aucBuf[0] = usVal & 0xFF;
	aucBuf[1] = usVal >> 8;
//	usVal = (uint16_t)(pstPackRVal->fPackCur * 10 + 1000);
	usVal = (-pstPackRVal->fPackCur * 10 + 1000);
	aucBuf[2] = usVal & 0xFF;
	aucBuf[3] = usVal >> 8;
	aucBuf[4] = pstPackRVal->fPackSoc;
	aucBuf[5] = pstPackRVal->fPackSoh;
	if(pstPackRVal->uBaseStat.stBaseStat.ucChgerON == 0) {
		aucBuf[6] = 1;
	}
	CAN0_SendMsg(uiId, aucBuf, 8);
	delay_1ms(2);
	/* eCanIdChgReq2, 0x18914010, motorola */
	uiId = eCanIdChgReq2;
	memset(aucBuf, 0, 8);
	usVal = (uint16_t)(pstPackRVal->fCellUMax * 1000);
	aucBuf[0] = usVal >> 8;
	aucBuf[1] = usVal & 0xFF;
	aucBuf[2] = pstPackRVal->ucCellUMaxId + 1;
	usVal = (uint16_t)(pstPackRVal->fCellUMin * 1000);
	aucBuf[3] = usVal >> 8;
	aucBuf[4] = usVal & 0xFF;
	aucBuf[5] = pstPackRVal->ucCellUMinId + 1;
	CAN0_SendMsg(uiId, aucBuf, 8);
	delay_1ms(2);
	/* eCanIdChgReq1, 0x18904010, motorola */
	uiId = eCanIdChgReq1;
	memset(aucBuf, 0, 8);
	usVal = (uint16_t)(pstPackRVal->fPackU * 10);
	aucBuf[0] = usVal >> 8;
	aucBuf[1] = usVal & 0xFF;
	usVal = (uint16_t)(pstPackRVal->fPackCur * 10 + 30000);
	aucBuf[4] = usVal >> 8;
	aucBuf[5] = usVal & 0xFF;
	usVal = pstPackRVal->fPackSoc * 10;
	aucBuf[6] = usVal >> 8;
	aucBuf[7] = usVal & 0xFF;
	CAN0_SendMsg(uiId, aucBuf, 8);
	delay_1ms(2);
	/* eCanIdChgReq11, 0x18FA28F4, intel */
	uiId = eCanIdChgReq11;
	memset(aucBuf, 0, 8);
	usVal = (uint16_t)(pstPackRVal->fPeakLmtDsgI + 1000) * 10;
	aucBuf[0] = usVal & 0xFF;
	aucBuf[1] = usVal >> 8;
	usVal = (-pstPackRVal->fLmtChgI + 1000) * 10;
	aucBuf[2] = usVal & 0xFF;
	aucBuf[3] = usVal >> 8;
	usVal = (-pstPackRVal->fPackCur + 1000) * 10;
	aucBuf[4] = usVal & 0xFF;
	aucBuf[5] = usVal >> 8;
	CAN0_SendMsg(uiId, aucBuf, 8);
	delay_1ms(2);
	/* eCanIdChgReq12, 0x18FB28F4, intel */
	uiId = eCanIdChgReq12;
	memset(aucBuf, 0, 8);
	usVal = (uint16_t)(pstPackRVal->fCellUMax * 1000);
	aucBuf[0] = usVal & 0xFF;
	aucBuf[1] = usVal >> 8;
	usVal = (uint16_t)(pstPackRVal->fCellUMin * 1000);
	aucBuf[2] = usVal & 0xFF;
	aucBuf[3] = usVal >> 8;
	aucBuf[4] = (uint8_t)(pstPackRVal->fCellTMax + 40);
	aucBuf[5] = (uint8_t)(pstPackRVal->fCellTMin + 40);
	CAN0_SendMsg(uiId, aucBuf, 8);
	delay_1ms(2);
	/* eCanIdChgReq13, 0x18FC28F4, intel */
	uiId = eCanIdChgReq13;
	memset(aucBuf, 0, 8);
	if(pstPackRVal->uBaseStat.stBaseStat.ucChgerON) {
		aucBuf[0] |= 0x01;
	}
	if(pstPackRVal->uBaseStat.stBaseStat.ucChg) {
		aucBuf[0] |= 0x02;
	}
	if(pstPackRVal->fCellUMin < 2.6) {
		aucBuf[0] |= 0x04;
	}
	if(g_stLocalArrayRVal.eLocalStat == eLocalStatRun) {
		aucBuf[0] |= 0x08;
	}
	if(g_stAfe.uRam.stCode.DSG_FET && pstPackRVal->ucDsgForceEn == 0) {
		aucBuf[0] |= 0x10;
	}
	if(g_stAfe.uRam.stCode.CHG_FET && pstPackRVal->ucChgForceEn == 0) {
		aucBuf[0] |= 0x20;
	}
	aucBuf[1] = pstPackRVal->fPackSoc;
	usVal = (uint16_t)(pstPackRVal->fPackU * 10);
	aucBuf[4] = usVal & 0xFF;
	aucBuf[5] = usVal >> 8;
	aucBuf[6] = g_ucAlmLevel;
	{
		static uint8_t s_uctick = 1, uc_tick = 0;
		for(uint8_t i=s_uctick; i<65; i++){
			if(i == 6 || (i > 17 && i < 53) || (i > 57 && i < 64)) {
				continue;
			}
 			if(GET_ALM1_CODE(i)) {
				aucBuf[7] = i;
				CAN0_SendMsg(uiId, aucBuf, 8);
				delay_1ms(2);	
				s_uctick = i;
				uc_tick++;
				if(uc_tick > 10){
					uc_tick = 0;
					s_uctick = i + 1;
				}
				if(s_uctick == 65){
					s_uctick = 1;
				}
				break;
			}		
		}
		
		if(aucBuf[7] == 0) {
			if(s_uctick == 1){
				CAN0_SendMsg(uiId, aucBuf, 8);
				delay_1ms(2);
			}
			s_uctick = 1;
		}
  }
	{
		/* eCanIdChgReq5, 0x18F811F3, intel */
		static uint8_t s_ucTick = 0;
		if(s_ucTick * g_stCfg.stLocal.usCyclePeriod < 250) {
			s_ucTick++;
		}
		if(s_ucTick * g_stCfg.stLocal.usCyclePeriod >= 250) {
			s_ucTick = 0;
			uiId = eCanIdChgReq5;
			memset(aucBuf, 0, 8);
			usVal = (uint16_t)(g_stLocalArrayRVal.fCellUMax * 1000);
			aucBuf[0] = usVal & 0xFF;
			aucBuf[1] = usVal >> 8;
			aucBuf[2] = (g_stLocalArrayRVal.usCellUMaxId & 0xFF) + 1;
			aucBuf[3] = (g_stLocalArrayRVal.usCellUMaxId >> 8) + 1;
			usVal = (uint16_t)(g_stLocalArrayRVal.fCellUMin * 1000);
			aucBuf[4] = usVal & 0xFF;
			aucBuf[5] = usVal >> 8;
			aucBuf[6] = (g_stLocalArrayRVal.usCellUMinId & 0xFF) + 1;
			aucBuf[7] = (g_stLocalArrayRVal.usCellUMinId >> 8) + 1;
			CAN0_SendMsg(uiId, aucBuf, 8);
			delay_1ms(2);
			/* eCanIdChgReq6, 0x18F812F3, intel */
			uiId = eCanIdChgReq6;
			memset(aucBuf, 0, 8);
			aucBuf[0] = (uint8_t)(g_stLocalArrayRVal.fPackTMax + 40);
			aucBuf[1] = (g_stLocalArrayRVal.usPackTMaxId & 0xFF) + 1;
			aucBuf[2] = (g_stLocalArrayRVal.usPackTMaxId >> 8) + 1;
			aucBuf[3] = (uint8_t)(g_stLocalArrayRVal.fPackTMin + 40);
			aucBuf[4] = (g_stLocalArrayRVal.usPackTMinId & 0xFF) + 1;
			aucBuf[5] = (g_stLocalArrayRVal.usPackTMinId >> 8) + 1;
			CAN0_SendMsg(uiId, aucBuf, 8);
			delay_1ms(2);
			/* eCanIdChgReq7, 0x18F813F3, intel */
			uiId = eCanIdChgReq7;
			memset(aucBuf, 0, 8);
//			if(pstPackRVal->uBaseStat.stBaseStat.ucChgerON == 0) {
//				aucBuf[0] = 1;
//			}
			if(g_bMChgerComAct || g_bSChgerComAct) {
				aucBuf[0] = 0;
			} else {
				aucBuf[0] = 1;
			}
			aucBuf[1] = 1;
			if(pstPackRVal->uBaseStat.stBaseStat.ucChg) {
				aucBuf[2] = 1;
			}
//			if(pstPackRVal->uBaseStat.stBaseStat.ucStandby && pstPackRVal->uBaseStat.stBaseStat.ucChgerON) {
//				aucBuf[2] = 2;
//			}
			if(g_ucChgFull == 1) {
				aucBuf[2] = 2;
			}
			if(g_stAfe.uRam.stCode.DSG_FET && pstPackRVal->ucDsgForceEn == 0) {
				aucBuf[3] = 1;
				aucBuf[4] = 1;
			}
			if(g_stAfe.uRam.stCode.CHG_FET && pstPackRVal->ucChgForceEn == 0) {
				aucBuf[5] = 1;
			}
			CAN0_SendMsg(uiId, aucBuf, 8);
			delay_1ms(2);
			/* eCanIdChgReq8, 0x18F814F3, intel */
			uiId = eCanIdChgReq8;
			memset(aucBuf, 0, 8);
			usVal = (-pstPackRVal->fReqChgI * 10 + 1000);
			aucBuf[0] = usVal & 0xFF;
			aucBuf[1] = usVal >> 8;
			usVal = (uint16_t)(pstPackRVal->fPeakLmtDsgI * 10 + 1000);
			aucBuf[2] = usVal & 0xFF;
			aucBuf[3] = usVal >> 8;
			usVal = (-pstPackRVal->fLmtChgI * 10 + 1000);
			aucBuf[4] = usVal & 0xFF;
			aucBuf[5] = usVal >> 8;
			usVal = (uint16_t)(g_stCfg.stLocal.usDesignAH);
			aucBuf[6] = usVal & 0xFF;
			aucBuf[7] = usVal >> 8;
			CAN0_SendMsg(uiId, aucBuf, 8);
			delay_1ms(2);
			/* Begin :  Modified by dgx, 2024.4.2 */
			{
			/* eCanIdBMSID1, 0xE013, motorola */
				uiId = eCanIdPackID1;
				memset(aucBuf, 0, 8);
				static uint8_t aucData[14];
				memcpy(aucData, g_stCfg.stLocal.aucBleName, 14);
				aucBuf[0] = aucData[0];
				aucBuf[1] = aucData[1];
				aucBuf[2] = aucData[2];
				aucBuf[3] = aucData[3];
				aucBuf[4] = aucData[4];
				aucBuf[5] = aucData[5];
				aucBuf[6] = aucData[6];
				aucBuf[7] = aucData[7];
				CAN0_SendMsg(uiId, aucBuf, 8);
				delay_1ms(2);
			/* eCanIdBMSID2, 0xE014, motorola */
				uiId = eCanIdPackID2;
				memset(aucBuf, 0, 8);
				aucBuf[0] = aucData[8];
				aucBuf[1] = aucData[9];
				aucBuf[2] = aucData[10];
				aucBuf[3] = aucData[11];
				aucBuf[4] = aucData[12];
				aucBuf[5] = aucData[13];
				CAN0_SendMsg(uiId, aucBuf, 8);
				delay_1ms(2);
			/* eCanIdBMSID3, 0xE015, motorola */
				uiId = eCanIdPackID3;
				memset(aucBuf, 0, 8);
				aucBuf[7] = 0x03;		//means GEN3, added by h00205922, 2024.05.01
				CAN0_SendMsg(uiId, aucBuf, 8);
			}
			/* End :  Modified by dgx, 2024.4.2 */
			/* BMS equalization opening string number, 0xF000, motorola */
			uiId = eCanIdBMSEqOpenStrNum;
			memset(aucBuf, 0, 8);
			usVal = 0;
			if(g_BalanceFlag == 1) {
				switch(usVal = pstPackRVal->ucCellUMaxId + 1){
					case 1:aucBuf[0] = 0x01,aucBuf[1] = 0x00;break;
					case 2:aucBuf[0] = 0x02,aucBuf[1] = 0x00;break;
					case 3:aucBuf[0] = 0x04,aucBuf[1] = 0x00;break;
					case 4:aucBuf[0] = 0x08,aucBuf[1] = 0x00;break;
					case 5:aucBuf[0] = 0x10,aucBuf[1] = 0x00;break;
					case 6:aucBuf[0] = 0x20,aucBuf[1] = 0x00;break;
					case 7:aucBuf[0] = 0x40,aucBuf[1] = 0x00;break;
					case 8:aucBuf[0] = 0x80,aucBuf[1] = 0x00;break;
					case 9:aucBuf[0] = 0x00,aucBuf[1] = 0x01;break;
					case 10:aucBuf[0] = 0x00,aucBuf[1] = 0x02;break;
					case 11:aucBuf[0] = 0x00,aucBuf[1] = 0x04;break;
					case 12:aucBuf[0] = 0x00,aucBuf[1] = 0x08;break;
					case 13:aucBuf[0] = 0x00,aucBuf[1] = 0x10;break;
					case 14:aucBuf[0] = 0x00,aucBuf[1] = 0x20;break;
					case 15:aucBuf[0] = 0x00,aucBuf[1] = 0x40;break;
					case 16:aucBuf[0] = 0x00,aucBuf[1] = 0x80;break;
				}
			}
			CAN0_SendMsg(uiId, aucBuf, 8);
			delay_1ms(2);
		}
	}
	/* eCanIdChgReq9, 0x18F815F3, intel */
	uiId = eCanIdChgReq9;
	memset(aucBuf, 0, 8);
//	if(GET_ALM0_CODE(6)) {
//		aucBuf[0] |= 0x03;
//	} else if(GET_ALM1_CODE(56)) {
//		aucBuf[0] |= 0x02;
//	}
	if(GET_ALM1_CODE(56)) {
		aucBuf[0] |= 0x02;
	}
	if(GET_ALM1_CODE(7)) {
		aucBuf[0] |= 0x0C;
	} else if(GET_ALM1_CODE(57)) {
		aucBuf[0] |= 0x08;
	}
	if(GET_ALM1_CODE(4)) {
		aucBuf[0] |= 0x30;
	} else if(GET_ALM1_CODE(54)) {
		aucBuf[0] |= 0x20;
	}
	if(GET_ALM1_CODE(5)) {
		aucBuf[0] |= 0xC0;
	} else if(GET_ALM1_CODE(55)) {
		aucBuf[0] |= 0x80;
	}
	if(GET_ALM1_CODE(3)) {
		aucBuf[1] |= 0x03;
	} else if(GET_ALM1_CODE(53)) {
		aucBuf[1] |= 0x02;
	}
	if(GET_ALM1_CODE(10) || GET_ALM1_CODE(11)) {
		aucBuf[1] |= 0x0C;
	}
	if(GET_ALM1_CODE(12) || GET_ALM1_CODE(13)) {
		aucBuf[1] |= 0x30;
	}
	if(pstPackRVal->fCellTMax - pstPackRVal->fCellTMin > 20) {
		aucBuf[1] |= 0xC0;
	}
	if(GET_ALM0_CODE(9)) {
		aucBuf[2] |= 0x03;
	}
	if(GET_ALM0_CODE(8)) {
		aucBuf[2] |= 0x0C;
	}
	if(GET_ALM0_CODE(15) || GET_ALM1_CODE(15)) {
		aucBuf[2] |= 0xC0;
	}
	aucBuf[4] = CFG_CELL_NUM;
	aucBuf[5] = CFG_TMP_NUM;
	CAN0_SendMsg(uiId, aucBuf, 8);
	/* eCanIdChgReq10, 0x18F880F3, intel */
//	if(g_bs18F880F3Act == true) {
//		static uint8_t s_ucTick = 0;
//		if(s_ucTick * g_stCfg.stLocal.usCyclePeriod < 250) {
//			s_ucTick++;
//		}
//		if(s_ucTick * g_stCfg.stLocal.usCyclePeriod >= 250) {
//			s_ucTick = 0;
//			uiId = eCanIdChgReq10;
//			RTC_Get();
//			memset(aucBuf, 0, 8);
//			aucBuf[0] = (calender.w_year & 0xFF);
//			aucBuf[1] = (calender.w_year >> 8) & 0xFF;
//			aucBuf[2] = calender.w_month;
//			aucBuf[3] = calender.w_day;
//			aucBuf[4] = calender.hour;
//			aucBuf[5] = calender.min;
//			CAN0_SendMsg(uiId, aucBuf, 8);
//		}
//	}
	/* eCanIdRexBox, 0x10086 */
	{
		uiId = eCanIdRexBox;
		memset(aucBuf, 0, 8);
		if(pstPackRVal->uBaseStat.stBaseStat.ucChgerON) {
			aucBuf[0] = 1;
		}
		if(GET_ALM0_CODE(4) || GET_ALM0_CODE(6) || GET_ALM0_CODE(11) || GET_ALM0_CODE(12) || GET_ALM0_CODE(14) || GET_ALM0_CODE(9)) {
			aucBuf[1] = 1;
		}
		usVal = pstPackRVal->fPackU * 10;
		aucBuf[2] = usVal >> 8;
		aucBuf[3] = usVal & 0xFF;
//		CAN0_SendMsg(uiId, aucBuf, 8);
	}
}

void prl_host_auto_send(void) {
	uint32_t uiId;
	uint8_t aucBuf[8];
	LOCAL_PACK_RVAL_S* pstPackRVal = g_stLocalArrayRVal.astPackRVal + g_stPrl.ucSelfId;
	uint16_t usVal;
	ECO_RTV_S* pstPrlEcoRtv = g_stEcoRtv_Parallel + PRL_MAX_NODE_NUM;
	/* BMS Limits, 0xE005, motorola */
	{
		uiId = eCanIdBMSLmt;
		usVal = pstPrlEcoRtv->usPeakLmtDsgCur;
		aucBuf[0] = (usVal >> 8) & 0xFF;
		aucBuf[1] = usVal & 0xFF;
		for(uint8_t i = 1; i < g_stPrl.ucDevNum; i++) {
			if(i == 1) {
				if(pstPackRVal->fMosT > g_stEcoRtv_Parallel[i].sMOSRTT1) {
					usVal = pstPackRVal->fMosT;
				} else {
					usVal = g_stEcoRtv_Parallel[i].sMOSRTT1;
				}
			} else {
				if(usVal > g_stEcoRtv_Parallel[i].sMOSRTT1) {
					usVal = g_stEcoRtv_Parallel[i].sMOSRTT1;
				} else {
					usVal = g_stEcoRtv_Parallel[i].sMOSRTT1;
				}
			}
		}
		if(usVal > 10) {
			aucBuf[2] = usVal + 40;
			aucBuf[3] = 0;
		} else {
			for(uint8_t i = 1; i < g_stPrl.ucDevNum; i++) {
				if(i == 1) {
					if(pstPackRVal->fMosT < g_stEcoRtv_Parallel[i].sMOSRTT1) {
						usVal = pstPackRVal->fMosT;
					} else {
						usVal = g_stEcoRtv_Parallel[i].sMOSRTT1;
					}
				} else {
					if(usVal < g_stEcoRtv_Parallel[i].sMOSRTT1) {
						usVal = g_stEcoRtv_Parallel[i].sMOSRTT1;
					} else {
						usVal = g_stEcoRtv_Parallel[i].sMOSRTT1;
					}
				}
			}
			aucBuf[2] = usVal + 40;
			aucBuf[3] = 0;
		}
		usVal = pstPrlEcoRtv->usLmtDsgCur;
		aucBuf[4] = (usVal >> 8) & 0xFF;
		aucBuf[5] = usVal & 0xFF;
		usVal = pstPrlEcoRtv->usLmtChgCur;
		aucBuf[6] = (usVal >> 8) & 0xFF;
		aucBuf[7] = usVal & 0xFF;
		CAN0_SendMsg(uiId, aucBuf, 8);
		delay_1ms(2);
	}
	/* Charge Request 1, 0x1806E5F4, motorola */
	{
		static uint8_t s_ucTick = 0;
		uiId = eCanIdChgReq0;
		memset(aucBuf, 0, 8);
		usVal = pstPrlEcoRtv->usPackRTSoc * 100;
		aucBuf[0] = (usVal >> 8) & 0xFF;
		aucBuf[1] = usVal & 0xFF;
		if(pstPrlEcoRtv->usPackRTSoc < 100) {
			aucBuf[2] = 1;
			aucBuf[3] = 0;
			usVal = (uint16_t)(pstPrlEcoRtv->usReqChgCur * 10);
			aucBuf[4] = (usVal >> 8) & 0xFF;
			aucBuf[5] = usVal & 0xFF;
		}
		if(round(pstPackRVal->fCellTMin) < g_stCfg.stLocal.sCellUCTTVThr1 || pstPackRVal->uErrCode.stErrCode.bTempSensor) {
			aucBuf[2] = 0;
			aucBuf[3] = 0;
			aucBuf[4] = 0;
			aucBuf[5] = 0;
		}
		if(g_bSetMChgerAct && pstPackRVal->uBaseStat.stBaseStat.ucHeating != 0) {
			aucBuf[2] = 1;
			aucBuf[3] = 0;
			aucBuf[4] = 0x00;
			aucBuf[5] = 0x32;
		}
		if(g_bSChgerComAct && !g_bMChgerComAct) {
			aucBuf[2] = 0;
			aucBuf[3] = 0;
			aucBuf[4] = 0;
			aucBuf[5] = 0;
		}
		if(g_bSChgerComAct && g_bMChgerComAct) {
			if(s_ucTick * g_stCfg.stLocal.usCyclePeriod < 5000) {
				s_ucTick++;
				aucBuf[2] = 0;
				aucBuf[3] = 0;
				aucBuf[4] = 0;
				aucBuf[5] = 0;
			}
		}
		usVal = 560;
		aucBuf[6] = (usVal >> 8) & 0xFF;
		aucBuf[7] = usVal & 0xFF;
		CAN0_SendMsg(uiId, aucBuf, 8);
		delay_1ms(2);
	}
	/* Begin : Modified by h00205922, 2023.03.29 */
	/* Charge Request 2, 0x273, intel */
	uiId = eCanIdChgReq20;
	memset(aucBuf, 0, 8);
	if(g_bMChgerComAct) {
	  aucBuf[0] = 0x00;
		aucBuf[1] = 0x00;
		aucBuf[2] = 0x00;
		aucBuf[3] = 0x00;
	} else {
		usVal = pstPrlEcoRtv->usReqChgCur;
		aucBuf[0] = usVal & 0xFF;
		aucBuf[1] = (usVal >> 8) & 0xFF;
		usVal = 5600;
		aucBuf[2] = usVal & 0xFF;
		aucBuf[3] = (usVal >> 8) & 0xFF;
	}
	usVal = pstPrlEcoRtv->usPackRTVolt;
	aucBuf[4] = usVal & 0xFF;
	aucBuf[5] = (usVal >> 8) & 0xFF;
	aucBuf[6] = round(pstPrlEcoRtv->usPackRTSoc);
	if(g_bMChgerComAct) {
		aucBuf[7] = 0x01;
	}
	if(round(pstPrlEcoRtv->usPackRTSoc) == 100) {
		aucBuf[7] = 0x00;
	}
	if(!g_bMChgerComAct && pstPrlEcoRtv->usReqChgCur > 0) {
		aucBuf[7] += 2;
	} else {
		aucBuf[7] = 0;
	}
	CAN0_SendMsg(uiId, aucBuf, 8);
	delay_1ms(2);
	/* End   : Modified by h00205922, 2023.03.29 */
	/* BMS Info, 0xE000, motorola */
	uiId = eCanIdBmsInf;
	memset(aucBuf, 0, 8);
	usVal = round(pstPrlEcoRtv->usPackRTSoc) * 100;
	aucBuf[0] = (usVal >> 8) & 0xFF;
	aucBuf[1] = usVal & 0xFF;
	aucBuf[2] = (usVal >> 8) & 0xFF;
	aucBuf[3] = usVal & 0xFF;
	usVal = pstPrlEcoRtv->usPackRTVolt;
	aucBuf[4] = (usVal >> 8) & 0xFF;
	aucBuf[5] = usVal & 0xFF;
	usVal = (uint16_t)(pstPrlEcoRtv->sPackRTCur);
	aucBuf[6] = (usVal >> 8) & 0xFF;
	aucBuf[7] = usVal & 0xFF;
	CAN0_SendMsg(uiId, aucBuf, 8);
	delay_1ms(2);
	/* Soc, 0xE004, motorola */
	uiId = eCanIdSoc;
	memset(aucBuf, 0, 8);
	int16_t g_stEcoRtv_Parallel_sCellTMax[3] = {0};
	int16_t g_stEcoRtv_Parallel_sCellTMin[3] = {0};
	for(uint8_t i = 1; i < g_stPrl.ucDevNum; i++) {
		if(g_stEcoRtv_Parallel[i].sCellRTT1 > g_stEcoRtv_Parallel[i].sCellRTT2) {
			g_stEcoRtv_Parallel_sCellTMax[i] = g_stEcoRtv_Parallel[i].sCellRTT1;
		} else {
			g_stEcoRtv_Parallel_sCellTMax[i] = g_stEcoRtv_Parallel[i].sCellRTT2;
		}
		if(g_stEcoRtv_Parallel_sCellTMax[i] > g_stEcoRtv_Parallel[i].sCellRTT3) {
			break;
		} else {
			g_stEcoRtv_Parallel_sCellTMax[i] = g_stEcoRtv_Parallel[i].sCellRTT3;
		}
	}
	for(uint8_t i = 1; i < g_stPrl.ucDevNum; i++) {
		if(g_stEcoRtv_Parallel[i].sCellRTT1 < g_stEcoRtv_Parallel[i].sCellRTT2) {
			g_stEcoRtv_Parallel_sCellTMin[i] = g_stEcoRtv_Parallel[i].sCellRTT1;
		} else {
			g_stEcoRtv_Parallel_sCellTMin[i] = g_stEcoRtv_Parallel[i].sCellRTT2;
		}
		if(g_stEcoRtv_Parallel_sCellTMin[i] < g_stEcoRtv_Parallel[i].sCellRTT3) {
			break;
		} else {
			g_stEcoRtv_Parallel_sCellTMin[i] = g_stEcoRtv_Parallel[i].sCellRTT3;
		}
	}
	for(uint8_t i = 1; i < g_stPrl.ucDevNum; i++) {
		if(i == 1) {
			if(pstPackRVal->fCellTMax > g_stEcoRtv_Parallel_sCellTMax[i]) {
				aucBuf[0] = (int8_t)pstPackRVal->fCellTMax;
			} else {
				aucBuf[0] = (int8_t)g_stEcoRtv_Parallel_sCellTMax[i];
			}
		} else {
			if(aucBuf[0] > g_stEcoRtv_Parallel_sCellTMax[i]) {
				break;
			} else {
				aucBuf[0] = (int8_t)g_stEcoRtv_Parallel_sCellTMax[i];
			}
		}
	}
//	aucBuf[0] = (int8_t)pstPackRVal->fCellTMax;
	for(uint8_t i = 1; i < g_stPrl.ucDevNum; i++) {
		if(i == 1) {
			if(pstPackRVal->fCellTMin < g_stEcoRtv_Parallel_sCellTMin[i]) {
				aucBuf[1] = (int8_t)pstPackRVal->fCellTMin;
			} else {
				aucBuf[1] = (int8_t)g_stEcoRtv_Parallel_sCellTMax[i];
			}
		} else {
			if(aucBuf[1] < g_stEcoRtv_Parallel_sCellTMin[i]) {
				break;
			} else {
				aucBuf[1] = (int8_t)g_stEcoRtv_Parallel_sCellTMin[i];
			}
		}
	}
//	aucBuf[1] = (int8_t)pstPackRVal->fCellTMin;
	usVal = pstPrlEcoRtv->usPackNominalAh;
	aucBuf[2] = (usVal >> 8) & 0xFF;
	aucBuf[3] = usVal & 0xFF;
	usVal = pstPrlEcoRtv->usPackNominalAh;
	aucBuf[4] = (usVal >> 8) & 0xFF;
	aucBuf[5] = usVal & 0xFF;
	usVal = pstPrlEcoRtv->usPackLeftAH;
	aucBuf[6] = (usVal >> 8) & 0xFF;
	aucBuf[7] = usVal & 0xFF;
	CAN0_SendMsg(uiId, aucBuf, 8);
	delay_1ms(2);
	/* Cell Voltage 0, 0xE006, motorola */
	uiId = eCanIdCellVolt0;
	usVal = (uint16_t)(pstPackRVal->afCellU[0] * 1000);
	aucBuf[0] = (usVal >> 8) & 0xFF;
	aucBuf[1] = usVal & 0xFF;
	usVal = (uint16_t)(pstPackRVal->afCellU[1] * 1000);
	aucBuf[2] = (usVal >> 8) & 0xFF;
	aucBuf[3] = usVal & 0xFF;
	usVal = (uint16_t)(pstPackRVal->afCellU[2] * 1000);
	aucBuf[4] = (usVal >> 8) & 0xFF;
	aucBuf[5] = usVal & 0xFF;
	usVal = (uint16_t)(pstPackRVal->afCellU[3] * 1000);
	aucBuf[6] = (usVal >> 8) & 0xFF;
	aucBuf[7] = usVal & 0xFF;
	CAN0_SendMsg(uiId, aucBuf, 8);
	delay_1ms(2);
	/* Cell Voltage 1, 0xE007, motorola */
	uiId = eCanIdCellVolt1;
	usVal = (uint16_t)(pstPackRVal->afCellU[4] * 1000);
	aucBuf[0] = (usVal >> 8) & 0xFF;
	aucBuf[1] = usVal & 0xFF;
	usVal = (uint16_t)(pstPackRVal->afCellU[5] * 1000);
	aucBuf[2] = (usVal >> 8) & 0xFF;
	aucBuf[3] = usVal & 0xFF;
	usVal = (uint16_t)(pstPackRVal->afCellU[6] * 1000);
	aucBuf[4] = (usVal >> 8) & 0xFF;
	aucBuf[5] = usVal & 0xFF;
	usVal = (uint16_t)(pstPackRVal->afCellU[7] * 1000);
	aucBuf[6] = (usVal >> 8) & 0xFF;
	aucBuf[7] = usVal & 0xFF;
	CAN0_SendMsg(uiId, aucBuf, 8);
	delay_1ms(2);
	/* Cell Voltage 2, 0xE008, motorola */
	uiId = eCanIdCellVolt2;
	usVal = (uint16_t)(pstPackRVal->afCellU[8] * 1000);
	aucBuf[0] = (usVal >> 8) & 0xFF;
	aucBuf[1] = usVal & 0xFF;
	usVal = (uint16_t)(pstPackRVal->afCellU[9] * 1000);
	aucBuf[2] = (usVal >> 8) & 0xFF;
	aucBuf[3] = usVal & 0xFF;
	usVal = (uint16_t)(pstPackRVal->afCellU[10] * 1000);
	aucBuf[4] = (usVal >> 8) & 0xFF;
	aucBuf[5] = usVal & 0xFF;
	usVal = (uint16_t)(pstPackRVal->afCellU[11] * 1000);
	aucBuf[6] = (usVal >> 8) & 0xFF;
	aucBuf[7] = usVal & 0xFF;
	CAN0_SendMsg(uiId, aucBuf, 8);
	delay_1ms(2);
	/* Cell Voltage 3, 0xE009, motorola */
	uiId = eCanIdCellVolt3;
	usVal = (uint16_t)(pstPackRVal->afCellU[12] * 1000);
	aucBuf[0] = (usVal >> 8) & 0xFF;
	aucBuf[1] = usVal & 0xFF;
	usVal = (uint16_t)(pstPackRVal->afCellU[13] * 1000);
	aucBuf[2] = (usVal >> 8) & 0xFF;
	aucBuf[3] = usVal & 0xFF;
	usVal = (uint16_t)(pstPackRVal->afCellU[14] * 1000);
	aucBuf[4] = (usVal >> 8) & 0xFF;
	aucBuf[5] = usVal & 0xFF;
	usVal = (uint16_t)(pstPackRVal->afCellU[15] * 1000);
	aucBuf[6] = (usVal >> 8) & 0xFF;
	aucBuf[7] = usVal & 0xFF;
	CAN0_SendMsg(uiId, aucBuf, 8);
	delay_1ms(2);
	/* Cell Voltage 4, 0xE010, motorola */
	uiId = eCanIdCellVolt4;
	memset(aucBuf, 0, 8);
	CAN0_SendMsg(uiId, aucBuf, 8);
	delay_1ms(2);
	/* Cell Voltage 5, 0xE011, motorola */
	uiId = eCanIdCellVolt5;
	CAN0_SendMsg(uiId, aucBuf, 8);
	delay_1ms(2);
	/* Cells Average Voltage, 0xE012, motorola */
	uiId = eCanIdCellVoltAvg;
	memset(aucBuf, 0, 8);
	usVal = 0;
	for(uint8_t i=0;i<CFG_CELL_NUM;i++) {
		usVal += (pstPackRVal->afCellU[i] * 1000);
	}
	usVal /= CFG_CELL_NUM;
	aucBuf[0] = (usVal >> 8) & 0xFF;
	aucBuf[1] = usVal & 0xFF;
	usVal = (uint16_t)((pstPackRVal->fCellUMax - pstPackRVal->fCellUMin) * 1000);
	aucBuf[2] = (usVal >> 8) & 0xFF;
	aucBuf[3] = usVal & 0xFF;
	CAN0_SendMsg(uiId, aucBuf, 8);
	delay_1ms(2);
	/* Fault Info., 0xF001, motorola */
	uiId = eCanIdFaultInf;
	memset(aucBuf, 0, 8);
	if(GET_ALM0_CODE(1)) {
		aucBuf[0] |= 0x01;
	}
	if(GET_ALM0_CODE(2)) {
		aucBuf[0] |= 0x04;
	}
	if(GET_ALM0_CODE(3)) {
		aucBuf[0] |= 0x10;
	}
	if(GET_ALM0_CODE(4)) {
		aucBuf[0] |= 0x80;
	}
	if(GET_ALM0_CODE(5)) {
		aucBuf[1] |= 0x02;
	}
//	if(GET_ALM0_CODE(6)) {
//		aucBuf[1] |= 0x08;
//	}
	if(GET_ALM0_CODE(7)) {
		aucBuf[1] |= 0x20;
	}
	if(GET_ALM0_CODE(8)) {
		aucBuf[1] |= 0xC0;
	}
	if(GET_ALM0_CODE(9)) {
		aucBuf[2] |= 0x03;
	}
	if(GET_ALM0_CODE(10)) {
		aucBuf[2] |= 0x0C;
	}
	if(GET_ALM0_CODE(11)) {
		aucBuf[2] |= 0x30;
	}
	if(GET_ALM0_CODE(12)) {
		aucBuf[2] |= 0xC0;
	}
	if(GET_ALM0_CODE(13)) {
		aucBuf[3] |= 0x03;
	}
	if(GET_ALM0_CODE(14) && GET_ALM0_CODE(14)) {
		aucBuf[3] |= 0x0C;
	}
	if(GET_ALM0_CODE(15)) {
		aucBuf[3] |= 0x30;
	}
	if(GET_ALM0_CODE(16)) {
		aucBuf[3] |= 0xC0;
	}
	if(GET_ALM0_CODE(17)) {
		aucBuf[4] |= 0x03;
	}
	CAN0_SendMsg(uiId, aucBuf, 8);
	delay_1ms(2);
	/* eCanIdChgReq16, 0x18FF28F4, motorola */
	uiId = eCanIdChgReq16;
	memset(aucBuf, 0, 8);
	for(int i = 0; i < g_stPrl.ucDevNum; i++) {
		if(pstPrlEcoRtv->bCMOSOn != 0) {
			aucBuf[0] |= 0x01;
			break;
		}
	}
	for(int i = 0; i < g_stPrl.ucDevNum; i++) {
		if(pstPrlEcoRtv->bDMOSOn != 0) {
			aucBuf[0] |= 0x10;
			break;
		}
	}
	aucBuf[1] = round(pstPrlEcoRtv->usPackRTSoc);
	usVal = pstPrlEcoRtv->usPackRTVolt;
	aucBuf[4] = usVal >> 8;
	aucBuf[5] = usVal & 0xFF;
	if(g_stPrl.ucDevNum == 4) {
		if((g_eBaseAlm.ALM_CODE1 && g_eBaseAlm_Parallel[1].ALM_CODE1 && g_eBaseAlm_Parallel[2].ALM_CODE1 && g_eBaseAlm_Parallel[3].ALM_CODE1) 
		  || (g_eBaseAlm.ALM_CODE2 && g_eBaseAlm_Parallel[1].ALM_CODE2 && g_eBaseAlm_Parallel[2].ALM_CODE2 && g_eBaseAlm_Parallel[3].ALM_CODE2)
		 	|| (g_eBaseAlm.ALM_CODE3 && g_eBaseAlm_Parallel[1].ALM_CODE3 && g_eBaseAlm_Parallel[2].ALM_CODE3 && g_eBaseAlm_Parallel[3].ALM_CODE3)
			|| (g_eBaseAlm.ALM_CODE4 && g_eBaseAlm_Parallel[1].ALM_CODE4 && g_eBaseAlm_Parallel[2].ALM_CODE4 && g_eBaseAlm_Parallel[3].ALM_CODE4)
			|| (g_eBaseAlm.ALM_CODE5 && g_eBaseAlm_Parallel[1].ALM_CODE5 && g_eBaseAlm_Parallel[2].ALM_CODE5 && g_eBaseAlm_Parallel[3].ALM_CODE5)
			|| (g_eBaseAlm.ALM_CODE6 && g_eBaseAlm_Parallel[1].ALM_CODE6 && g_eBaseAlm_Parallel[2].ALM_CODE6 && g_eBaseAlm_Parallel[3].ALM_CODE6)
			|| (g_eBaseAlm.ALM_CODE7 && g_eBaseAlm_Parallel[1].ALM_CODE7 && g_eBaseAlm_Parallel[2].ALM_CODE7 && g_eBaseAlm_Parallel[3].ALM_CODE7)
			|| (g_eBaseAlm.ALM_CODE8 && g_eBaseAlm_Parallel[1].ALM_CODE8 && g_eBaseAlm_Parallel[2].ALM_CODE8 && g_eBaseAlm_Parallel[3].ALM_CODE8)
			|| (g_eBaseAlm.ALM_CODE9 && g_eBaseAlm_Parallel[1].ALM_CODE9 && g_eBaseAlm_Parallel[2].ALM_CODE9 && g_eBaseAlm_Parallel[3].ALM_CODE9)
			|| (g_eBaseAlm.ALM_CODE10 && g_eBaseAlm_Parallel[1].ALM_CODE10 && g_eBaseAlm_Parallel[2].ALM_CODE10 && g_eBaseAlm_Parallel[3].ALM_CODE10)
			|| (g_eBaseAlm.ALM_CODE11 && g_eBaseAlm_Parallel[1].ALM_CODE11 && g_eBaseAlm_Parallel[2].ALM_CODE11 && g_eBaseAlm_Parallel[3].ALM_CODE11)
			|| (g_eBaseAlm.ALM_CODE12 && g_eBaseAlm_Parallel[1].ALM_CODE12 && g_eBaseAlm_Parallel[2].ALM_CODE12 && g_eBaseAlm_Parallel[3].ALM_CODE12)
			|| (g_eBaseAlm.ALM_CODE13 && g_eBaseAlm_Parallel[1].ALM_CODE13 && g_eBaseAlm_Parallel[2].ALM_CODE13 && g_eBaseAlm_Parallel[3].ALM_CODE13)
			|| (g_eBaseAlm.ALM_CODE14 && g_eBaseAlm_Parallel[1].ALM_CODE14 && g_eBaseAlm_Parallel[2].ALM_CODE14 && g_eBaseAlm_Parallel[3].ALM_CODE14)
			|| (g_eBaseAlm.ALM_CODE15 && g_eBaseAlm_Parallel[1].ALM_CODE15 && g_eBaseAlm_Parallel[2].ALM_CODE15 && g_eBaseAlm_Parallel[3].ALM_CODE15)
			|| (g_eBaseAlm.ALM_CODE16 && g_eBaseAlm_Parallel[1].ALM_CODE16 && g_eBaseAlm_Parallel[2].ALM_CODE16 && g_eBaseAlm_Parallel[3].ALM_CODE16)
			|| (g_eBaseAlm.ALM_CODE17 && g_eBaseAlm_Parallel[1].ALM_CODE17 && g_eBaseAlm_Parallel[2].ALM_CODE17 && g_eBaseAlm_Parallel[3].ALM_CODE17)) {
				g_ucAlmLevel = 1;
		} else if(!g_eBaseAlm.ALM_CODE1 && !g_eBaseAlm_Parallel[1].ALM_CODE1 && !g_eBaseAlm_Parallel[2].ALM_CODE1 && !g_eBaseAlm_Parallel[3].ALM_CODE1 
		  && !g_eBaseAlm.ALM_CODE2 && !g_eBaseAlm_Parallel[1].ALM_CODE2 && !g_eBaseAlm_Parallel[2].ALM_CODE2 && !g_eBaseAlm_Parallel[3].ALM_CODE2
		 	&& !g_eBaseAlm.ALM_CODE3 && !g_eBaseAlm_Parallel[1].ALM_CODE3 && !g_eBaseAlm_Parallel[2].ALM_CODE3 && !g_eBaseAlm_Parallel[3].ALM_CODE3
			&& !g_eBaseAlm.ALM_CODE4 && !g_eBaseAlm_Parallel[1].ALM_CODE4 && !g_eBaseAlm_Parallel[2].ALM_CODE4 && !g_eBaseAlm_Parallel[3].ALM_CODE4
			&& !g_eBaseAlm.ALM_CODE5 && !g_eBaseAlm_Parallel[1].ALM_CODE5 && !g_eBaseAlm_Parallel[2].ALM_CODE5 && !g_eBaseAlm_Parallel[3].ALM_CODE5
			&& !g_eBaseAlm.ALM_CODE6 && !g_eBaseAlm_Parallel[1].ALM_CODE6 && !g_eBaseAlm_Parallel[2].ALM_CODE6 && !g_eBaseAlm_Parallel[3].ALM_CODE6
			&& !g_eBaseAlm.ALM_CODE7 && !g_eBaseAlm_Parallel[1].ALM_CODE7 && !g_eBaseAlm_Parallel[2].ALM_CODE7 && !g_eBaseAlm_Parallel[3].ALM_CODE7
			&& !g_eBaseAlm.ALM_CODE8 && !g_eBaseAlm_Parallel[1].ALM_CODE8 && !g_eBaseAlm_Parallel[2].ALM_CODE8 && !g_eBaseAlm_Parallel[3].ALM_CODE8
			&& !g_eBaseAlm.ALM_CODE9 && !g_eBaseAlm_Parallel[1].ALM_CODE9 && !g_eBaseAlm_Parallel[2].ALM_CODE9 && !g_eBaseAlm_Parallel[3].ALM_CODE9
			&& !g_eBaseAlm.ALM_CODE10 && !g_eBaseAlm_Parallel[1].ALM_CODE10 && !g_eBaseAlm_Parallel[2].ALM_CODE10 && !g_eBaseAlm_Parallel[3].ALM_CODE10
			&& !g_eBaseAlm.ALM_CODE11 && !g_eBaseAlm_Parallel[1].ALM_CODE11 && !g_eBaseAlm_Parallel[2].ALM_CODE11 && !g_eBaseAlm_Parallel[3].ALM_CODE11
			&& !g_eBaseAlm.ALM_CODE12 && !g_eBaseAlm_Parallel[1].ALM_CODE12 && !g_eBaseAlm_Parallel[2].ALM_CODE12 && !g_eBaseAlm_Parallel[3].ALM_CODE12
			&& !g_eBaseAlm.ALM_CODE13 && !g_eBaseAlm_Parallel[1].ALM_CODE13 && !g_eBaseAlm_Parallel[2].ALM_CODE13 && !g_eBaseAlm_Parallel[3].ALM_CODE13
			&& !g_eBaseAlm.ALM_CODE14 && !g_eBaseAlm_Parallel[1].ALM_CODE14 && !g_eBaseAlm_Parallel[2].ALM_CODE14 && !g_eBaseAlm_Parallel[3].ALM_CODE14
			&& !g_eBaseAlm.ALM_CODE15 && !g_eBaseAlm_Parallel[1].ALM_CODE15 && !g_eBaseAlm_Parallel[2].ALM_CODE15 && !g_eBaseAlm_Parallel[3].ALM_CODE15
			&& !g_eBaseAlm.ALM_CODE16 && !g_eBaseAlm_Parallel[1].ALM_CODE16 && !g_eBaseAlm_Parallel[2].ALM_CODE16 && !g_eBaseAlm_Parallel[3].ALM_CODE16
			&& !g_eBaseAlm.ALM_CODE17 && !g_eBaseAlm_Parallel[1].ALM_CODE17 && !g_eBaseAlm_Parallel[2].ALM_CODE17 && !g_eBaseAlm_Parallel[3].ALM_CODE17
			&& !g_eBaseAlm.ALM_CODE53 && !g_eBaseAlm_Parallel[1].ALM_CODE53 && !g_eBaseAlm_Parallel[2].ALM_CODE53 && !g_eBaseAlm_Parallel[3].ALM_CODE53
			&& !g_eBaseAlm.ALM_CODE54 && !g_eBaseAlm_Parallel[1].ALM_CODE54 && !g_eBaseAlm_Parallel[2].ALM_CODE54 && !g_eBaseAlm_Parallel[3].ALM_CODE54
			&& !g_eBaseAlm.ALM_CODE55 && !g_eBaseAlm_Parallel[1].ALM_CODE55 && !g_eBaseAlm_Parallel[2].ALM_CODE55 && !g_eBaseAlm_Parallel[3].ALM_CODE55
			&& !g_eBaseAlm.ALM_CODE56 && !g_eBaseAlm_Parallel[1].ALM_CODE56 && !g_eBaseAlm_Parallel[2].ALM_CODE56 && !g_eBaseAlm_Parallel[3].ALM_CODE56
			&& !g_eBaseAlm.ALM_CODE57 && !g_eBaseAlm_Parallel[1].ALM_CODE57 && !g_eBaseAlm_Parallel[2].ALM_CODE57 && !g_eBaseAlm_Parallel[3].ALM_CODE57
			&& !g_eBaseAlm.ALM_CODE64 && !g_eBaseAlm_Parallel[1].ALM_CODE64 && !g_eBaseAlm_Parallel[2].ALM_CODE64 && !g_eBaseAlm_Parallel[3].ALM_CODE64) {
			g_ucAlmLevel = 0;
		} else {
			g_ucAlmLevel = 2;
		}
	} else if(g_stPrl.ucDevNum == 3) {
		if((g_eBaseAlm.ALM_CODE1 && g_eBaseAlm_Parallel[1].ALM_CODE1 && g_eBaseAlm_Parallel[2].ALM_CODE1) 
		  || (g_eBaseAlm.ALM_CODE2 && g_eBaseAlm_Parallel[1].ALM_CODE2 && g_eBaseAlm_Parallel[2].ALM_CODE2)
		 	|| (g_eBaseAlm.ALM_CODE3 && g_eBaseAlm_Parallel[1].ALM_CODE3 && g_eBaseAlm_Parallel[2].ALM_CODE3)
			|| (g_eBaseAlm.ALM_CODE4 && g_eBaseAlm_Parallel[1].ALM_CODE4 && g_eBaseAlm_Parallel[2].ALM_CODE4)
			|| (g_eBaseAlm.ALM_CODE5 && g_eBaseAlm_Parallel[1].ALM_CODE5 && g_eBaseAlm_Parallel[2].ALM_CODE5)
			|| (g_eBaseAlm.ALM_CODE6 && g_eBaseAlm_Parallel[1].ALM_CODE6 && g_eBaseAlm_Parallel[2].ALM_CODE6)
			|| (g_eBaseAlm.ALM_CODE7 && g_eBaseAlm_Parallel[1].ALM_CODE7 && g_eBaseAlm_Parallel[2].ALM_CODE7)
			|| (g_eBaseAlm.ALM_CODE8 && g_eBaseAlm_Parallel[1].ALM_CODE8 && g_eBaseAlm_Parallel[2].ALM_CODE8)
			|| (g_eBaseAlm.ALM_CODE9 && g_eBaseAlm_Parallel[1].ALM_CODE9 && g_eBaseAlm_Parallel[2].ALM_CODE9)
			|| (g_eBaseAlm.ALM_CODE10 && g_eBaseAlm_Parallel[1].ALM_CODE10 && g_eBaseAlm_Parallel[2].ALM_CODE10)
			|| (g_eBaseAlm.ALM_CODE11 && g_eBaseAlm_Parallel[1].ALM_CODE11 && g_eBaseAlm_Parallel[2].ALM_CODE11)
			|| (g_eBaseAlm.ALM_CODE12 && g_eBaseAlm_Parallel[1].ALM_CODE12 && g_eBaseAlm_Parallel[2].ALM_CODE12)
			|| (g_eBaseAlm.ALM_CODE13 && g_eBaseAlm_Parallel[1].ALM_CODE13 && g_eBaseAlm_Parallel[2].ALM_CODE13)
			|| (g_eBaseAlm.ALM_CODE14 && g_eBaseAlm_Parallel[1].ALM_CODE14 && g_eBaseAlm_Parallel[2].ALM_CODE14)
			|| (g_eBaseAlm.ALM_CODE15 && g_eBaseAlm_Parallel[1].ALM_CODE15 && g_eBaseAlm_Parallel[2].ALM_CODE15)
			|| (g_eBaseAlm.ALM_CODE16 && g_eBaseAlm_Parallel[1].ALM_CODE16 && g_eBaseAlm_Parallel[2].ALM_CODE16)
			|| (g_eBaseAlm.ALM_CODE17 && g_eBaseAlm_Parallel[1].ALM_CODE17 && g_eBaseAlm_Parallel[2].ALM_CODE17)) {
				g_ucAlmLevel = 1;
		} else if(!g_eBaseAlm.ALM_CODE1 && !g_eBaseAlm_Parallel[1].ALM_CODE1 && !g_eBaseAlm_Parallel[2].ALM_CODE1 
		  && !g_eBaseAlm.ALM_CODE2 && !g_eBaseAlm_Parallel[1].ALM_CODE2 && !g_eBaseAlm_Parallel[2].ALM_CODE2
		 	&& !g_eBaseAlm.ALM_CODE3 && !g_eBaseAlm_Parallel[1].ALM_CODE3 && !g_eBaseAlm_Parallel[2].ALM_CODE3
			&& !g_eBaseAlm.ALM_CODE4 && !g_eBaseAlm_Parallel[1].ALM_CODE4 && !g_eBaseAlm_Parallel[2].ALM_CODE4
			&& !g_eBaseAlm.ALM_CODE5 && !g_eBaseAlm_Parallel[1].ALM_CODE5 && !g_eBaseAlm_Parallel[2].ALM_CODE5
			&& !g_eBaseAlm.ALM_CODE6 && !g_eBaseAlm_Parallel[1].ALM_CODE6 && !g_eBaseAlm_Parallel[2].ALM_CODE6
			&& !g_eBaseAlm.ALM_CODE7 && !g_eBaseAlm_Parallel[1].ALM_CODE7 && !g_eBaseAlm_Parallel[2].ALM_CODE7
			&& !g_eBaseAlm.ALM_CODE8 && !g_eBaseAlm_Parallel[1].ALM_CODE8 && !g_eBaseAlm_Parallel[2].ALM_CODE8
			&& !g_eBaseAlm.ALM_CODE9 && !g_eBaseAlm_Parallel[1].ALM_CODE9 && !g_eBaseAlm_Parallel[2].ALM_CODE9
			&& !g_eBaseAlm.ALM_CODE10 && !g_eBaseAlm_Parallel[1].ALM_CODE10 && !g_eBaseAlm_Parallel[2].ALM_CODE10
			&& !g_eBaseAlm.ALM_CODE11 && !g_eBaseAlm_Parallel[1].ALM_CODE11 && !g_eBaseAlm_Parallel[2].ALM_CODE11
			&& !g_eBaseAlm.ALM_CODE12 && !g_eBaseAlm_Parallel[1].ALM_CODE12 && !g_eBaseAlm_Parallel[2].ALM_CODE12
			&& !g_eBaseAlm.ALM_CODE13 && !g_eBaseAlm_Parallel[1].ALM_CODE13 && !g_eBaseAlm_Parallel[2].ALM_CODE13
			&& !g_eBaseAlm.ALM_CODE14 && !g_eBaseAlm_Parallel[1].ALM_CODE14 && !g_eBaseAlm_Parallel[2].ALM_CODE14
			&& !g_eBaseAlm.ALM_CODE15 && !g_eBaseAlm_Parallel[1].ALM_CODE15 && !g_eBaseAlm_Parallel[2].ALM_CODE15
			&& !g_eBaseAlm.ALM_CODE16 && !g_eBaseAlm_Parallel[1].ALM_CODE16 && !g_eBaseAlm_Parallel[2].ALM_CODE16
			&& !g_eBaseAlm.ALM_CODE17 && !g_eBaseAlm_Parallel[1].ALM_CODE17 && !g_eBaseAlm_Parallel[2].ALM_CODE17
			&& !g_eBaseAlm.ALM_CODE53 && !g_eBaseAlm_Parallel[1].ALM_CODE53 && !g_eBaseAlm_Parallel[2].ALM_CODE53
			&& !g_eBaseAlm.ALM_CODE54 && !g_eBaseAlm_Parallel[1].ALM_CODE54 && !g_eBaseAlm_Parallel[2].ALM_CODE54
			&& !g_eBaseAlm.ALM_CODE55 && !g_eBaseAlm_Parallel[1].ALM_CODE55 && !g_eBaseAlm_Parallel[2].ALM_CODE55
			&& !g_eBaseAlm.ALM_CODE56 && !g_eBaseAlm_Parallel[1].ALM_CODE56 && !g_eBaseAlm_Parallel[2].ALM_CODE56
			&& !g_eBaseAlm.ALM_CODE57 && !g_eBaseAlm_Parallel[1].ALM_CODE57 && !g_eBaseAlm_Parallel[2].ALM_CODE57
			&& !g_eBaseAlm.ALM_CODE64 && !g_eBaseAlm_Parallel[1].ALM_CODE64 && !g_eBaseAlm_Parallel[2].ALM_CODE64) {
			g_ucAlmLevel = 0;
		} else {
			g_ucAlmLevel = 2;
		}
	} else if(g_stPrl.ucDevNum == 2) {
		if((g_eBaseAlm.ALM_CODE1 && g_eBaseAlm_Parallel[1].ALM_CODE1) 
		  || (g_eBaseAlm.ALM_CODE2 && g_eBaseAlm_Parallel[1].ALM_CODE2)
		 	|| (g_eBaseAlm.ALM_CODE3 && g_eBaseAlm_Parallel[1].ALM_CODE3)
			|| (g_eBaseAlm.ALM_CODE4 && g_eBaseAlm_Parallel[1].ALM_CODE4)
			|| (g_eBaseAlm.ALM_CODE5 && g_eBaseAlm_Parallel[1].ALM_CODE5)
			|| (g_eBaseAlm.ALM_CODE6 && g_eBaseAlm_Parallel[1].ALM_CODE6)
			|| (g_eBaseAlm.ALM_CODE7 && g_eBaseAlm_Parallel[1].ALM_CODE7)
			|| (g_eBaseAlm.ALM_CODE8 && g_eBaseAlm_Parallel[1].ALM_CODE8)
			|| (g_eBaseAlm.ALM_CODE9 && g_eBaseAlm_Parallel[1].ALM_CODE9)
			|| (g_eBaseAlm.ALM_CODE10 && g_eBaseAlm_Parallel[1].ALM_CODE10)
			|| (g_eBaseAlm.ALM_CODE11 && g_eBaseAlm_Parallel[1].ALM_CODE11)
			|| (g_eBaseAlm.ALM_CODE12 && g_eBaseAlm_Parallel[1].ALM_CODE12)
			|| (g_eBaseAlm.ALM_CODE13 && g_eBaseAlm_Parallel[1].ALM_CODE13)
			|| (g_eBaseAlm.ALM_CODE14 && g_eBaseAlm_Parallel[1].ALM_CODE14)
			|| (g_eBaseAlm.ALM_CODE15 && g_eBaseAlm_Parallel[1].ALM_CODE15)
			|| (g_eBaseAlm.ALM_CODE16 && g_eBaseAlm_Parallel[1].ALM_CODE16)
			|| (g_eBaseAlm.ALM_CODE17 && g_eBaseAlm_Parallel[1].ALM_CODE17)) {
				g_ucAlmLevel = 1;
		} else if(!g_eBaseAlm.ALM_CODE1 && !g_eBaseAlm_Parallel[1].ALM_CODE1
		  && !g_eBaseAlm.ALM_CODE2 && !g_eBaseAlm_Parallel[1].ALM_CODE2
		 	&& !g_eBaseAlm.ALM_CODE3 && !g_eBaseAlm_Parallel[1].ALM_CODE3
			&& !g_eBaseAlm.ALM_CODE4 && !g_eBaseAlm_Parallel[1].ALM_CODE4
			&& !g_eBaseAlm.ALM_CODE5 && !g_eBaseAlm_Parallel[1].ALM_CODE5
			&& !g_eBaseAlm.ALM_CODE6 && !g_eBaseAlm_Parallel[1].ALM_CODE6
			&& !g_eBaseAlm.ALM_CODE7 && !g_eBaseAlm_Parallel[1].ALM_CODE7
			&& !g_eBaseAlm.ALM_CODE8 && !g_eBaseAlm_Parallel[1].ALM_CODE8
			&& !g_eBaseAlm.ALM_CODE9 && !g_eBaseAlm_Parallel[1].ALM_CODE9 
			&& !g_eBaseAlm.ALM_CODE10 && !g_eBaseAlm_Parallel[1].ALM_CODE10
			&& !g_eBaseAlm.ALM_CODE11 && !g_eBaseAlm_Parallel[1].ALM_CODE11
			&& !g_eBaseAlm.ALM_CODE12 && !g_eBaseAlm_Parallel[1].ALM_CODE12
			&& !g_eBaseAlm.ALM_CODE13 && !g_eBaseAlm_Parallel[1].ALM_CODE13
			&& !g_eBaseAlm.ALM_CODE14 && !g_eBaseAlm_Parallel[1].ALM_CODE14
			&& !g_eBaseAlm.ALM_CODE15 && !g_eBaseAlm_Parallel[1].ALM_CODE15
			&& !g_eBaseAlm.ALM_CODE16 && !g_eBaseAlm_Parallel[1].ALM_CODE16
			&& !g_eBaseAlm.ALM_CODE17 && !g_eBaseAlm_Parallel[1].ALM_CODE17
			&& !g_eBaseAlm.ALM_CODE53 && !g_eBaseAlm_Parallel[1].ALM_CODE53
			&& !g_eBaseAlm.ALM_CODE54 && !g_eBaseAlm_Parallel[1].ALM_CODE54
			&& !g_eBaseAlm.ALM_CODE55 && !g_eBaseAlm_Parallel[1].ALM_CODE55
			&& !g_eBaseAlm.ALM_CODE56 && !g_eBaseAlm_Parallel[1].ALM_CODE56
			&& !g_eBaseAlm.ALM_CODE57 && !g_eBaseAlm_Parallel[1].ALM_CODE57
			&& !g_eBaseAlm.ALM_CODE64 && !g_eBaseAlm_Parallel[1].ALM_CODE64) {
			g_ucAlmLevel = 0;
		} else {
			g_ucAlmLevel = 2;
		}
	}
	
	aucBuf[6] = g_ucAlmLevel;
	{
		static uint8_t s_uctick = 1, uc_tick = 0;
		for(uint8_t i=s_uctick; i<65; i++){
			if(i == 6 || (i > 17 && i < 53) || (i > 57 && i < 64)) {
				continue;
			}
 			if(GET_ALM1_CODE(i)) {
				aucBuf[7] = i;
				CAN0_SendMsg(uiId, aucBuf, 8);
				delay_1ms(2);	
				s_uctick = i;
				uc_tick++;
				if(uc_tick > 10){
					uc_tick = 0;
					s_uctick = i + 1;
				}
				if(s_uctick == 65){
					s_uctick = 1;
				}
				break;
			}		
		}
		if(aucBuf[7] == 0) {
			if(s_uctick == 1){
				CAN0_SendMsg(uiId, aucBuf, 8);
				delay_1ms(2);
			}
			s_uctick = 1;
		}
  }
	/* eCanIdChgReq15, 0x18FE28F4, motorola */
	uiId = eCanIdChgReq15;
	memset(aucBuf, 0, 8);
	float fCellUMax = pstPackRVal->fCellUMax * 1000;
	float fCellUMin = pstPackRVal->fCellUMin * 1000;
	for(int i = 0; i < g_stPrl.ucDevNum; i++) {
		for(int j = 0; j < CFG_CELL_NUM; j++) {
			if(fCellUMax < g_stEcoRtv_Parallel[i].ausCellRTVolt[j]) {
				fCellUMax = g_stEcoRtv_Parallel[i].ausCellRTVolt[j];
			}
		}
		for(int j = 0; j < CFG_CELL_NUM; j++) {
			if(fCellUMin > g_stEcoRtv_Parallel[i].ausCellRTVolt[j]) {
				fCellUMin = g_stEcoRtv_Parallel[i].ausCellRTVolt[j];
			}
		}
	}
	usVal = (uint16_t)fCellUMax;
	aucBuf[0] = usVal >> 8;
	aucBuf[1] = usVal & 0xFF;
	usVal = (uint16_t)fCellUMin;
	aucBuf[2] = usVal >> 8;
	aucBuf[3] = usVal & 0xFF;
	aucBuf[4] = (uint8_t)(pstPackRVal->fCellTMax + 40);
	aucBuf[5] = (uint8_t)(pstPackRVal->fCellTMin + 40);
	CAN0_SendMsg(uiId, aucBuf, 8);
	delay_1ms(2);
	/* eCanIdChgReq14, 0x18FD28F4, motorola */
	uiId = eCanIdChgReq14;
	memset(aucBuf, 0, 8);
//	usVal = (uint16_t)(pstPackRVal->fPeakLmtDsgI * 10);  //A30 version changes
//	aucBuf[0] = usVal >> 8;
//	aucBuf[1] = usVal & 0xFF;
//	usVal = (uint16_t)(-pstPackRVal->fLmtChgI * 10);     //A30 version changes
//	aucBuf[2] = usVal >> 8;
//	aucBuf[3] = usVal & 0xFF;
	usVal = pstPrlEcoRtv->usPeakLmtDsgCur;
	aucBuf[0] = usVal >> 8;
	aucBuf[1] = usVal & 0xFF;
	usVal = -pstPrlEcoRtv->usLmtChgCur;
	aucBuf[2] = usVal >> 8;
	aucBuf[3] = usVal & 0xFF;
	CAN0_SendMsg(uiId, aucBuf, 8);
	delay_1ms(2);
	/* Begin :  Modified by dgx, 2024.4.2 */
	/* eCanIdEx5, 0x1E1, motorola */
	uiId = eCanIdEx5;
	memset(aucBuf, 0, 8);
	usVal = pstPrlEcoRtv->usPackRTVolt;
	aucBuf[0] = usVal >> 8;
	aucBuf[1] = usVal & 0xFF;
	usVal = (pstPrlEcoRtv->sPackRTCur * 0.1 + 1000) * 10;
	aucBuf[2] = usVal >> 8;
	aucBuf[3] = usVal & 0xFF;
	usVal = pstPrlEcoRtv->usPackRTSoc * 10;
	aucBuf[4] = usVal >> 8;
	aucBuf[5] = usVal & 0xFF;
	aucBuf[6] = usVal >> 8;
	aucBuf[7] = usVal & 0xFF;
	CAN0_SendMsg(uiId, aucBuf, 8);
	delay_1ms(2);
	/* eCanIdEx6, 0x1F5, motorola */
	uiId = eCanIdEx6;
	memset(aucBuf, 0, 8);
	aucBuf[0] = g_ucAlmLevel;
	CAN0_SendMsg(uiId, aucBuf, 8);
	delay_1ms(2);
	/* eCanIdEx7, 0x800A6A9, intel */
	{
		uiId = eCanIdEx7;
		memset(aucBuf, 0, 8);
		static uint8_t s_ucF_I_Num = 0;
		static uint8_t s_ucF_II_Num = 0;
		static uint8_t s_ucF_III_Num = 0;
		if(GET_ALM0_CODE(6) || GET_ALM1_CODE(6)) {
			aucBuf[0] |= 0x04;
			s_ucF_I_Num += 1;
		}
		if(GET_ALM0_CODE(7) || GET_ALM1_CODE(7)) {
			aucBuf[0] |= 0x08;
			s_ucF_I_Num += 1;
		}
		if(GET_ALM0_CODE(15) || GET_ALM1_CODE(15)) {
			aucBuf[0] |= 0x20;
			s_ucF_I_Num += 1;
		}
		if(GET_ALM0_CODE(5) || GET_ALM1_CODE(5)) {
			aucBuf[0] |= 0x40;
			s_ucF_I_Num += 1;
		}
		if(GET_ALM0_CODE(4) || GET_ALM1_CODE(4)) {
			aucBuf[0] |= 0x80;
			s_ucF_I_Num += 1;
		}
		if(GET_ALM0_CODE(12) || GET_ALM1_CODE(12) || GET_ALM0_CODE(13) || GET_ALM1_CODE(13)) {
			aucBuf[1] |= 0x01;
			s_ucF_I_Num += 1;
		}
		if(GET_ALM0_CODE(10) || GET_ALM1_CODE(10) || GET_ALM0_CODE(11) || GET_ALM1_CODE(11)) {
			aucBuf[1] |= 0x02;
			s_ucF_I_Num += 1;
		}
		if(pstPackRVal->uErrCode.stErrCode.bTempSensor == 1) {
			aucBuf[1] |= 0x10;
			s_ucF_I_Num += 1;
		}
		if(pstPackRVal->uErrCode.stErrCode.bVoltSensor == 1) {
			aucBuf[1] |= 0x40;
			s_ucF_I_Num += 1;
		}
		if(GET_ALM1_CODE(56)) {
			aucBuf[2] |= 0x04;
			s_ucF_II_Num += 1;
		}
		if(GET_ALM1_CODE(57)) {
			aucBuf[2] |= 0x08;
			s_ucF_II_Num += 1;
		}
		if(GET_ALM1_CODE(55)) {
			aucBuf[2] |= 0x40;
			s_ucF_II_Num += 1;
		}
		if(GET_ALM1_CODE(54)) {
			aucBuf[2] |= 0x80;
			s_ucF_II_Num += 1;
		}
		if(GET_ALM0_CODE(9)) {
			aucBuf[4] |= 0x01; 
			s_ucF_III_Num += 1;
		}
		if(GET_ALM0_CODE(8)) {
			aucBuf[4] |= 0x02; 
			s_ucF_III_Num += 1;
		}
		aucBuf[6] |= s_ucF_I_Num;
		aucBuf[6] |= (s_ucF_I_Num << 4);
		aucBuf[7] = s_ucF_III_Num;
		CAN0_SendMsg(uiId, aucBuf, 8);
		delay_1ms(2);
		s_ucF_I_Num = 0;
		s_ucF_II_Num = 0;
		s_ucF_III_Num = 0;
	}
	/* eCanIdEx8, 0x1000A6A9, intel */
	uiId = eCanIdEx8;
	memset(aucBuf, 0, 8);
	usVal = pstPrlEcoRtv->usPackRTVolt;
	aucBuf[0] = usVal & 0xFF;
	aucBuf[1] = usVal >> 8;
	usVal = (pstPrlEcoRtv->sPackRTCur * 0.1 + 500) * 10;
	aucBuf[2] = usVal & 0xFF;
	aucBuf[3] = usVal >> 8;
	aucBuf[4] = round(pstPrlEcoRtv->usPackRTSoc);
	aucBuf[5] = pstPackRVal->fCellTMax + 50;
	aucBuf[6] = pstPackRVal->fCellTMin + 50;
	usVal = 0;
	for(uint8_t i = 0; i < CFG_TMP_NUM; i++) {
		usVal += pstPackRVal->afCellT[i];
	}
	aucBuf[7] = usVal / CFG_TMP_NUM + 50;
	CAN0_SendMsg(uiId, aucBuf, 8);
	delay_1ms(2);
	/* eCanIdEx9, 0x1C00A6A9, intel */
	uiId = eCanIdEx9;
	memset(aucBuf, 0, 8);
	if(pstPackRVal->uBaseStat.stBaseStat.ucChgerON == 1) {
		aucBuf[0] |= 0x02;
	}
	if(pstPackRVal->ucDsgEn == 0x55 || pstPackRVal->ucDsgForceEn == 0x55) {
		aucBuf[0] |= 0x10;
	}
	if(pstPackRVal->ucChgEn == 0x55 || pstPackRVal->ucChgForceEn ==0x55) {
		aucBuf[0] |= 0x20;
	}
	if(pstPackRVal->uBaseStat.stBaseStat.ucChgerON == 1 && pstPackRVal->uBaseStat.stBaseStat.ucHeating != 0) {
		aucBuf[1] |= 0x01;
	}
	if(g_eBaseStat == eBStatWorking && pstPackRVal->uBaseStat.stBaseStat.ucChgerON == 1) {
		aucBuf[2] |= 0x02;
	} else if(g_eBaseStat == eBStatWorking && pstPackRVal->uBaseStat.stBaseStat.ucChgerON != 1) {
		aucBuf[2] |= 0x01;
	}
	if(pstPackRVal->uBaseStat.stBaseStat.ucChgerON == 1) {
		aucBuf[3] = (pstPackRVal->fPackRealAH - pstPackRVal->fPackLeftAH) / pstPackRVal->fPackCur * 60;
	}
	aucBuf[5] = 0x03;
	CAN0_SendMsg(uiId, aucBuf, 8);
	delay_1ms(2);
	/* End :  Modified by dgx, 2024.4.2 */
	/* eCanIdEx3, 0x4D4, intel */
	uiId = eCanIdEx3;
	memset(aucBuf, 0, 8);
	aucBuf[2] = round(pstPrlEcoRtv->usPackRTSoc);
	usVal = (uint16_t)(pstPrlEcoRtv->sPackRTCur + 32000);
	aucBuf[3] = usVal & 0xFF;
	aucBuf[4] = usVal >> 8;
	usVal = pstPrlEcoRtv->usPackRTVolt;
	aucBuf[5] = usVal & 0xFF;
	aucBuf[6] = usVal >> 8;
	if(g_bMChgerComAct == true) {
		aucBuf[7] = 1;
	} else {
		aucBuf[7] = 3;
	}
	{
		static uint8_t s_uctick = 1, uc_tick = 0;
		for(uint8_t i=s_uctick; i<65; i++){
			if((i > 17 && i < 53) || (i > 57 && i < 64)) {
				continue;
			}
 			if(GET_ALM1_CODE(i)) {
				aucBuf[0] = i;
				CAN0_SendMsg(uiId, aucBuf, 8);
				delay_1ms(2);	
				s_uctick = i;
				uc_tick++;
				if(uc_tick > 10){
					uc_tick = 0;
					s_uctick = i + 1;
				}
				if(s_uctick == 65){
					s_uctick = 1;
				}
				break;
			}		
		}
		
		if(aucBuf[0] == 0) {
			if(s_uctick == 1){
				CAN0_SendMsg(uiId, aucBuf, 8);
				delay_1ms(2);
			}
			s_uctick = 1;
		}
  }
	/* eCanIdEx4, 0x4D5, intel */
	uiId = eCanIdEx4;
	memset(aucBuf, 0, 8);
	usVal = (uint16_t)fCellUMin;
	aucBuf[0] = usVal & 0xFF;
	aucBuf[1] = usVal >> 8;
	usVal = (uint16_t)fCellUMax;
	aucBuf[2] = usVal & 0xFF;
	aucBuf[3] = usVal >> 8;
	aucBuf[4] = (uint8_t)(pstPackRVal->fCellTMin + 40);
	aucBuf[5] = (uint8_t)(pstPackRVal->fCellTMax + 40);
	CAN0_SendMsg(uiId, aucBuf, 8);
	delay_1ms(2);
	/* eCanIdEx5, 0x273, intel */
//	uiId = eCanIdEx5;
//	memset(aucBuf, 0, 8);
//	usVal = (uint16_t)(pstPackRVal->fReqChgI * 10);
//	aucBuf[0] = usVal & 0xFF;
//	aucBuf[1] = usVal >> 8;
//	usVal = 5600;
//	aucBuf[2] = usVal & 0xFF;
//	aucBuf[3] = usVal >> 8;
//	CAN0_SendMsg(uiId, aucBuf, 8);
//	delay_1ms(2);
	/* eCanIdEx0, 0x108, intel */
//	uiId = eCanIdEx0;
//	memset(aucBuf, 0, 8);
//	aucBuf[0] = 1;
//	usVal = (uint16_t)(pstPrlEcoRtv->sPackRTCur + 10000);
//	aucBuf[1] = usVal & 0xFF;
//	aucBuf[2] = usVal >> 8;
//	aucBuf[3] = round(pstPrlEcoRtv->usPackRTSoc);
//	aucBuf[4] = (uint8_t)(pstPackRVal->fCellTMax + 40);
//	aucBuf[5] = (uint8_t)(pstPackRVal->fCellTMin + 40);
//	if(pstPackRVal->ucChgEn == 0x55 && (pstPackRVal->ucChgForceEn == 0x55 || pstPackRVal->ucChgForceEn == 0)) {
//		aucBuf[6] |= 0x01;
//	}
//	if(pstPackRVal->ucDsgEn == 0x55 && (pstPackRVal->ucDsgForceEn == 0x55 || pstPackRVal->ucDsgForceEn == 0)) {
//		aucBuf[6] |= 0x04;
//	}
//	if(pstPackRVal->uBaseStat.stBaseStat.ucHeating) {
//		aucBuf[6] |= 0x10;
//	}
//	if(pstPackRVal->uBaseStat.stBaseStat.ucDCOut1) {
//		aucBuf[6] |= 0x40;
//	}
//	if(prl_host()) {
//		aucBuf[6] |= 0x01;
//		for(uint8_t i=0;i<g_stPrl.ucDevNum;i++) {
//			if(g_ausPrlComTick[i] == 0) {
//				aucBuf[6] |= 0x03;
//				break;
//			}
//		}
//	}
//	if(pstPackRVal->uBaseStat.stBaseStat.ucChgerON) {
//		aucBuf[7] |= 0xC0;
//	}
//	CAN0_SendMsg(uiId, aucBuf, 8);
//	delay_1ms(2);
	/* eCanIdEx1, 0x110, intel */
//	uiId = eCanIdEx1;
//	memset(aucBuf, 0, 8);
//	usVal = (uint16_t)fCellUMax;
//	aucBuf[0] = usVal & 0xFF;
//	aucBuf[1] = usVal >> 8;
//	
//	uint8_t ucCellUMaxId_Parallel[PRL_MAX_NODE_NUM] = {0};
//	uint8_t ucCellUMinId_Parallel[PRL_MAX_NODE_NUM] = {0};
//	uint16_t fCellUMax_Parallel[PRL_MAX_NODE_NUM] = {0};
//	uint16_t fCellUMin_Parallel[PRL_MAX_NODE_NUM] = {0};
//	for(uint8_t j = 1; j < g_stPrl.ucDevNum; j++) {
//		for(uint8_t i=0;i<CFG_CELL_NUM;i++) {
//			if(g_stEcoRtv_Parallel[j].ausCellRTVolt[ucCellUMaxId_Parallel[j]] < g_stEcoRtv_Parallel[j].ausCellRTVolt[i]) {
//				ucCellUMaxId_Parallel[j] = i;
//			}
//			if(g_stEcoRtv_Parallel[j].ausCellRTVolt[ucCellUMinId_Parallel[j]] > g_stEcoRtv_Parallel[j].ausCellRTVolt[i]) {
//				ucCellUMinId_Parallel[j] = i;
//			}
//		}
//		fCellUMax_Parallel[j] = g_stEcoRtv_Parallel[j].ausCellRTVolt[ucCellUMaxId_Parallel[j]];
//		fCellUMin_Parallel[j] = g_stEcoRtv_Parallel[j].ausCellRTVolt[ucCellUMinId_Parallel[j]];
//	}
//	for(uint8_t i = 0; i < g_stPrl.ucDevNum - 1; i++) {
//		if(i == 0) {
//			if(pstPackRVal->fCellUMax < fCellUMax_Parallel[i + 1]) {
//				aucBuf[2] = pstPackRVal->ucCellUMaxId + 1;
//			} else {
//				aucBuf[2] = CFG_CELL_NUM * (i + 1) + ucCellUMaxId_Parallel[i + 1] + 1;
//			}
//		} else {
//			if(usVal < fCellUMax_Parallel[i + 1]) {
//				aucBuf[2] = CFG_CELL_NUM * (i + 1) + fCellUMax_Parallel[i + 1] + 1;
//			}
//		}
//	}
//	for(uint8_t i = 0; i < g_stPrl.ucDevNum - 1; i++) {
//		if(i == 0) {
//			if(pstPackRVal->fCellUMin > fCellUMin_Parallel[i + 1]) {
//				aucBuf[5] = pstPackRVal->ucCellUMinId + 1;
//				usVal = (uint16_t)pstPackRVal->fCellUMin;
//			} else {
//				aucBuf[5] = CFG_CELL_NUM * (i + 1) + ucCellUMinId_Parallel[i + 1] + 1;
//				usVal = (uint16_t)fCellUMin_Parallel[i + 1];
//			}
//		} else {
//			if(usVal > fCellUMin_Parallel[i + 1]) {
//				usVal = fCellUMin_Parallel[i + 1];
//				aucBuf[5] = CFG_CELL_NUM * (i + 1) + ucCellUMinId_Parallel[i + 1] + 1;
//			}
//		}
//	}
//	aucBuf[3] = usVal & 0xFF;
//	aucBuf[4] = usVal >> 8;
//	usVal = (uint16_t)(pstPrlEcoRtv->usPackRTVolt * 10);
//	aucBuf[6] = usVal & 0xFF;
//	aucBuf[7] = usVal >> 8;
//	CAN0_SendMsg(uiId, aucBuf, 8);
//	delay_1ms(2);
	/* eCanIdEx2, 0x111, intel */
//	uiId = eCanIdEx2;
//	memset(aucBuf, 0, 8);
//	{
//		for(uint8_t i=1; i<17; i++){
//			if(i==4 || i==6 || i==8 || i==9 || i==12){
//				continue;
//			}
//			if(GET_ALM0_CODE(i)){
//				aucBuf[4] = 0xFF;
//			}
//		}
//  }
//	CAN0_SendMsg(uiId, aucBuf, 8);
//	delay_1ms(2);
	/* eCanIdChgReq4, 0x18F810F3, intel */
	uiId = eCanIdChgReq4;
	memset(aucBuf, 0, 8);
	usVal = (uint16_t)(pstPrlEcoRtv->usPackRTVolt);
	aucBuf[0] = usVal & 0xFF;
	aucBuf[1] = usVal >> 8;
//	usVal = (uint16_t)(pstPackRVal->fPackCur * 10 + 1000);
	usVal = (-pstPrlEcoRtv->sPackRTCur + 1000);
	aucBuf[2] = usVal & 0xFF;
	aucBuf[3] = usVal >> 8;
	aucBuf[4] = round(pstPrlEcoRtv->usPackRTSoc);
	aucBuf[5] = pstPrlEcoRtv->usPackRTSoh * 0.1;
	if(pstPackRVal->uBaseStat.stBaseStat.ucChgerON == 0) {
		aucBuf[6] = 1;
	}
	CAN0_SendMsg(uiId, aucBuf, 8);
	delay_1ms(2);
	/* eCanIdChgReq2, 0x18914010, motorola */
	uiId = eCanIdChgReq2;
	memset(aucBuf, 0, 8);
	usVal = (uint16_t)fCellUMax;
	aucBuf[0] = usVal >> 8;
	aucBuf[1] = usVal & 0xFF;
	aucBuf[2] = pstPackRVal->ucCellUMaxId + 1;
	usVal = (uint16_t)fCellUMin;
	aucBuf[3] = usVal >> 8;
	aucBuf[4] = usVal & 0xFF;
	aucBuf[5] = pstPackRVal->ucCellUMinId + 1;
	CAN0_SendMsg(uiId, aucBuf, 8);
	delay_1ms(2);
	/* eCanIdChgReq1, 0x18904010, motorola */
	uiId = eCanIdChgReq1;
	memset(aucBuf, 0, 8);
	usVal = (uint16_t)(pstPrlEcoRtv->usPackRTVolt);
	aucBuf[0] = usVal >> 8;
	aucBuf[1] = usVal & 0xFF;
	usVal = (uint16_t)(pstPrlEcoRtv->sPackRTCur + 30000);
	aucBuf[4] = usVal >> 8;
	aucBuf[5] = usVal & 0xFF;
	usVal = (uint16_t)(round(pstPrlEcoRtv->usPackRTSoc) * 10);
	aucBuf[6] = usVal >> 8;
	aucBuf[7] = usVal & 0xFF;
	CAN0_SendMsg(uiId, aucBuf, 8);
	delay_1ms(2);
	/* eCanIdChgReq11, 0x18FA28F4, intel */
	uiId = eCanIdChgReq11;
	memset(aucBuf, 0, 8);
	usVal = pstPrlEcoRtv->usPeakLmtDsgCur + 10000;
	aucBuf[0] = usVal & 0xFF;
	aucBuf[1] = usVal >> 8;
	usVal = -pstPrlEcoRtv->usLmtChgCur + 10000;
	aucBuf[2] = usVal & 0xFF;
	aucBuf[3] = usVal >> 8;
	usVal = -pstPrlEcoRtv->sPackRTCur + 10000;
	aucBuf[4] = usVal & 0xFF;
	aucBuf[5] = usVal >> 8;
	CAN0_SendMsg(uiId, aucBuf, 8);
	delay_1ms(2);
	/* eCanIdChgReq12, 0x18FB28F4, intel */
	uiId = eCanIdChgReq12;
	memset(aucBuf, 0, 8);
	usVal = (uint16_t)fCellUMax;
	aucBuf[0] = usVal & 0xFF;
	aucBuf[1] = usVal >> 8;
	usVal = (uint16_t)fCellUMin;
	aucBuf[2] = usVal & 0xFF;
	aucBuf[3] = usVal >> 8;
	aucBuf[4] = (uint8_t)(pstPackRVal->fCellTMax + 40);
	aucBuf[5] = (uint8_t)(pstPackRVal->fCellTMin + 40);
	CAN0_SendMsg(uiId, aucBuf, 8);
	delay_1ms(2);
	/* eCanIdChgReq13, 0x18FC28F4, intel */
	uiId = eCanIdChgReq13;
	memset(aucBuf, 0, 8);
	if(pstPackRVal->uBaseStat.stBaseStat.ucChgerON) {
		aucBuf[0] |= 0x01;
	}
	if(pstPackRVal->uBaseStat.stBaseStat.ucChg) {
		aucBuf[0] |= 0x02;
	}
	if(fCellUMin < 2.6) {
		aucBuf[0] |= 0x04;
	}
	if(g_stLocalArrayRVal.eLocalStat == eLocalStatRun) {
		aucBuf[0] |= 0x08;
	}
	for(int i = 0; i < g_stPrl.ucDevNum; i++) {
		if(g_stEcoRtv_Parallel[i].bDMOSOn != 0) {
			aucBuf[0] |= 0x10;
			break;
		}
	}
	for(int i = 0; i < g_stPrl.ucDevNum; i++) {
		if(g_stEcoRtv_Parallel[i].bCMOSOn != 0) {
			aucBuf[0] |= 0x20;
			break;
		}
	}
	aucBuf[1] = round(pstPrlEcoRtv->usPackRTSoc);
	usVal = (uint16_t)(pstPrlEcoRtv->usPackRTVolt);
	aucBuf[4] = usVal & 0xFF;
	aucBuf[5] = usVal >> 8;
	aucBuf[6] = g_ucAlmLevel;
	{
		static uint8_t s_uctick = 1, uc_tick = 0;
		for(uint8_t i=s_uctick; i<65; i++){
			if(i == 6 || (i > 17 && i < 53) || (i > 57 && i < 64)) {
				continue;
			}
 			if(GET_ALM1_CODE(i)) {
				aucBuf[7] = i;
				CAN0_SendMsg(uiId, aucBuf, 8);
				delay_1ms(2);	
				s_uctick = i;
				uc_tick++;
				if(uc_tick > 10){
					uc_tick = 0;
					s_uctick = i + 1;
				}
				if(s_uctick == 65){
					s_uctick = 1;
				}
				break;
			}		
		}
		
		if(aucBuf[7] == 0) {
			if(s_uctick == 1){
				CAN0_SendMsg(uiId, aucBuf, 8);
				delay_1ms(2);
			}
			s_uctick = 1;
		}
  }
	{
		/* eCanIdChgReq5, 0x18F811F3, intel */
		static uint8_t s_ucTick = 0;
		if(s_ucTick * g_stCfg.stLocal.usCyclePeriod < 250) {
			s_ucTick++;
		}
		if(s_ucTick * g_stCfg.stLocal.usCyclePeriod >= 250) {
			s_ucTick = 0;
			uiId = eCanIdChgReq5;
			memset(aucBuf, 0, 8);
			usVal = (uint16_t)fCellUMax;
			aucBuf[0] = usVal & 0xFF;
			aucBuf[1] = usVal >> 8;
			aucBuf[2] = 1;
			aucBuf[3] = 1;
			usVal = (uint16_t)fCellUMin;
			aucBuf[4] = usVal & 0xFF;
			aucBuf[5] = usVal >> 8;
			aucBuf[6] = 1;
			aucBuf[7] = 1;
			CAN0_SendMsg(uiId, aucBuf, 8);
			delay_1ms(2);
			/* eCanIdChgReq6, 0x18F812F3, intel */
			uiId = eCanIdChgReq6;
			memset(aucBuf, 0, 8);
			aucBuf[0] = (uint8_t)(g_stLocalArrayRVal.fPackTMax + 40);
			aucBuf[1] = (g_stLocalArrayRVal.usPackTMaxId & 0xFF) + 1;
			aucBuf[2] = (g_stLocalArrayRVal.usPackTMaxId >> 8) + 1;
			aucBuf[3] = (uint8_t)(g_stLocalArrayRVal.fPackTMin + 40);
			aucBuf[4] = (g_stLocalArrayRVal.usPackTMinId & 0xFF) + 1;
			aucBuf[5] = (g_stLocalArrayRVal.usPackTMinId >> 8) + 1;
			CAN0_SendMsg(uiId, aucBuf, 8);
			delay_1ms(2);
			/* eCanIdChgReq7, 0x18F813F3, intel */
			uiId = eCanIdChgReq7;
			memset(aucBuf, 0, 8);
			if(g_bMChgerComAct || g_bSChgerComAct) {
				aucBuf[0] = 0;
			} else {
				aucBuf[0] = 1;
			}
			aucBuf[1] = 1;
			if(pstPackRVal->uBaseStat.stBaseStat.ucChg) {
				aucBuf[2] = 1;
			}
			if(g_ucChgFull == 1) {
				aucBuf[2] = 2;
			}
			for(int i = 0; i < g_stPrl.ucDevNum; i++) {
				if(g_stEcoRtv_Parallel[i].bDMOSOn) {
					aucBuf[3] = 1;
					aucBuf[4] = 1;
					break;
				}
			}
			for(int i = 0; i < g_stPrl.ucDevNum; i++) {
				if(g_stEcoRtv_Parallel[i].bCMOSOn) {
					aucBuf[5] = 1;
					break;
				}
			}
			CAN0_SendMsg(uiId, aucBuf, 8);
			delay_1ms(2);
			/* eCanIdChgReq8, 0x18F814F3, intel */
			uiId = eCanIdChgReq8;
			memset(aucBuf, 0, 8);
			usVal = (-pstPrlEcoRtv->usReqChgCur + 1000);
			aucBuf[0] = usVal & 0xFF;
			aucBuf[1] = usVal >> 8;
			usVal = (uint16_t)(pstPrlEcoRtv->usPeakLmtDsgCur + 1000);
			aucBuf[2] = usVal & 0xFF;
			aucBuf[3] = usVal >> 8;
			usVal = (-pstPrlEcoRtv->usLmtChgCur + 1000);
			aucBuf[4] = usVal & 0xFF;
			aucBuf[5] = usVal >> 8;
			usVal = (uint16_t)(pstPrlEcoRtv->usPackNominalAh / 10);
			aucBuf[6] = usVal & 0xFF;
			aucBuf[7] = usVal >> 8;
			CAN0_SendMsg(uiId, aucBuf, 8);
			delay_1ms(2);
				/* Begin :  Modified by dgx, 2024.4.2 */
			{
			/* eCanIdBMSID1, 0xE013, motorola */
				uiId = eCanIdPackID1;
				memset(aucBuf, 0, 8);
				static uint8_t aucData[14];
				memcpy(aucData, g_stCfg.stLocal.aucBleName, 14);
				aucBuf[0] = aucData[0];
				aucBuf[1] = aucData[1];
				aucBuf[2] = aucData[2];
				aucBuf[3] = aucData[3];
				aucBuf[4] = aucData[4];
				aucBuf[5] = aucData[5];
				aucBuf[6] = aucData[6];
				aucBuf[7] = aucData[7];
				CAN0_SendMsg(uiId, aucBuf, 8);
				delay_1ms(2);
			/* eCanIdBMSID2, 0xE014, motorola */
				uiId = eCanIdPackID2;
				memset(aucBuf, 0, 8);
				aucBuf[0] = aucData[8];
				aucBuf[1] = aucData[9];
				aucBuf[2] = aucData[10];
				aucBuf[3] = aucData[11];
				aucBuf[4] = aucData[12];
				aucBuf[5] = aucData[13];
				CAN0_SendMsg(uiId, aucBuf, 8);
				delay_1ms(2);
			/* eCanIdBMSID3, 0xE015, motorola */
				uiId = eCanIdPackID3;
				memset(aucBuf, 0, 8);
				aucBuf[7] = 0x03;		//means GEN3, added by h00205922, 2024.05.01
				CAN0_SendMsg(uiId, aucBuf, 8);
			}
			/* End :  Modified by dgx, 2024.4.2 */
			/* BMS equalization opening string number, 0xF000, motorola */
			uiId = eCanIdBMSEqOpenStrNum;
			memset(aucBuf, 0, 8);
			usVal = 0;
			if(g_BalanceFlag == 1) {
				switch(usVal = pstPackRVal->ucCellUMaxId + 1){
					case 1:aucBuf[0] = 0x01,aucBuf[1] = 0x00;break;
					case 2:aucBuf[0] = 0x02,aucBuf[1] = 0x00;break;
					case 3:aucBuf[0] = 0x04,aucBuf[1] = 0x00;break;
					case 4:aucBuf[0] = 0x08,aucBuf[1] = 0x00;break;
					case 5:aucBuf[0] = 0x10,aucBuf[1] = 0x00;break;
					case 6:aucBuf[0] = 0x20,aucBuf[1] = 0x00;break;
					case 7:aucBuf[0] = 0x40,aucBuf[1] = 0x00;break;
					case 8:aucBuf[0] = 0x80,aucBuf[1] = 0x00;break;
					case 9:aucBuf[0] = 0x00,aucBuf[1] = 0x01;break;
					case 10:aucBuf[0] = 0x00,aucBuf[1] = 0x02;break;
					case 11:aucBuf[0] = 0x00,aucBuf[1] = 0x04;break;
					case 12:aucBuf[0] = 0x00,aucBuf[1] = 0x08;break;
					case 13:aucBuf[0] = 0x00,aucBuf[1] = 0x10;break;
					case 14:aucBuf[0] = 0x00,aucBuf[1] = 0x20;break;
					case 15:aucBuf[0] = 0x00,aucBuf[1] = 0x40;break;
					case 16:aucBuf[0] = 0x00,aucBuf[1] = 0x80;break;
				}
			}
			CAN0_SendMsg(uiId, aucBuf, 8);
			delay_1ms(2);
		}
	}
	/* eCanIdChgReq9, 0x18F815F3, intel */
	uiId = eCanIdChgReq9;
	memset(aucBuf, 0, 8);
//	if(GET_ALM0_CODE(6)) {
//		aucBuf[0] |= 0x03;
//	} else if(GET_ALM1_CODE(56)) {
//		aucBuf[0] |= 0x02;
//	}
	if(g_stPrl.ucDevNum == 4) {
		if(g_eBaseAlm.ALM_CODE56 || g_eBaseAlm_Parallel[1].ALM_CODE56 || g_eBaseAlm_Parallel[2].ALM_CODE56 || g_eBaseAlm_Parallel[3].ALM_CODE56) {
			aucBuf[0] |= 0x02;
		}
		if(g_eBaseAlm.ALM_CODE7 && g_eBaseAlm_Parallel[1].ALM_CODE7 && g_eBaseAlm_Parallel[2].ALM_CODE7 && g_eBaseAlm_Parallel[3].ALM_CODE7) {
			aucBuf[0] |= 0x0C;
		} else if(g_eBaseAlm.ALM_CODE57 || g_eBaseAlm_Parallel[1].ALM_CODE57 || g_eBaseAlm_Parallel[2].ALM_CODE57 || g_eBaseAlm_Parallel[3].ALM_CODE57) {
			aucBuf[0] |= 0x08;
		}
		if(g_eBaseAlm.ALM_CODE4 && g_eBaseAlm_Parallel[1].ALM_CODE4 && g_eBaseAlm_Parallel[2].ALM_CODE4 && g_eBaseAlm_Parallel[3].ALM_CODE4) {
			aucBuf[0] |= 0x30;
		} else if(g_eBaseAlm.ALM_CODE54 || g_eBaseAlm_Parallel[1].ALM_CODE54 || g_eBaseAlm_Parallel[2].ALM_CODE54 || g_eBaseAlm_Parallel[3].ALM_CODE54) {
			aucBuf[0] |= 0x20;
		}
		if(g_eBaseAlm.ALM_CODE5 && g_eBaseAlm_Parallel[1].ALM_CODE5 && g_eBaseAlm_Parallel[2].ALM_CODE5 && g_eBaseAlm_Parallel[3].ALM_CODE5) {
			aucBuf[0] |= 0xC0;
		} else if(g_eBaseAlm.ALM_CODE55 || g_eBaseAlm_Parallel[1].ALM_CODE55 || g_eBaseAlm_Parallel[2].ALM_CODE55 || g_eBaseAlm_Parallel[3].ALM_CODE55) {
			aucBuf[0] |= 0x80;
		}
		if(g_eBaseAlm.ALM_CODE3 && g_eBaseAlm_Parallel[1].ALM_CODE3 && g_eBaseAlm_Parallel[2].ALM_CODE3 && g_eBaseAlm_Parallel[3].ALM_CODE3) {
			aucBuf[1] |= 0x03;
		} else if(g_eBaseAlm.ALM_CODE53 || g_eBaseAlm_Parallel[1].ALM_CODE53 || g_eBaseAlm_Parallel[2].ALM_CODE53 || g_eBaseAlm_Parallel[3].ALM_CODE53) {
			aucBuf[1] |= 0x02;
		}
		if((g_eBaseAlm.ALM_CODE10 && g_eBaseAlm_Parallel[1].ALM_CODE10 && g_eBaseAlm_Parallel[2].ALM_CODE10 && g_eBaseAlm_Parallel[3].ALM_CODE10) 
				|| (g_eBaseAlm.ALM_CODE11 && g_eBaseAlm_Parallel[1].ALM_CODE11 && g_eBaseAlm_Parallel[2].ALM_CODE11 && g_eBaseAlm_Parallel[3].ALM_CODE11)) {
			aucBuf[1] |= 0x0C;
		}
		if((g_eBaseAlm.ALM_CODE12 && g_eBaseAlm_Parallel[1].ALM_CODE12 && g_eBaseAlm_Parallel[2].ALM_CODE12 && g_eBaseAlm_Parallel[3].ALM_CODE12) 
				|| (g_eBaseAlm.ALM_CODE13 && g_eBaseAlm_Parallel[1].ALM_CODE13 && g_eBaseAlm_Parallel[2].ALM_CODE13 && g_eBaseAlm_Parallel[3].ALM_CODE13)) {
			aucBuf[1] |= 0x30;
		}
		if(pstPackRVal->fCellTMax - pstPackRVal->fCellTMin > 20) {
			aucBuf[1] |= 0xC0;
		}
		if(g_eBaseAlm.ALM_CODE9 && g_eBaseAlm_Parallel[1].ALM_CODE9 && g_eBaseAlm_Parallel[2].ALM_CODE9 && g_eBaseAlm_Parallel[3].ALM_CODE9) {
			aucBuf[2] |= 0x03;
		}
		if(g_eBaseAlm.ALM_CODE8 && g_eBaseAlm_Parallel[1].ALM_CODE8 && g_eBaseAlm_Parallel[2].ALM_CODE8 && g_eBaseAlm_Parallel[3].ALM_CODE8) {
			aucBuf[2] |= 0x0C;
		}
		if(g_eBaseAlm.ALM_CODE15 && g_eBaseAlm_Parallel[1].ALM_CODE15 && g_eBaseAlm_Parallel[2].ALM_CODE15 && g_eBaseAlm_Parallel[3].ALM_CODE15) {
			aucBuf[2] |= 0xC0;
		}
	}
	if(g_stPrl.ucDevNum == 3) {
		if(g_eBaseAlm.ALM_CODE56 || g_eBaseAlm_Parallel[1].ALM_CODE56 || g_eBaseAlm_Parallel[2].ALM_CODE56) {
			aucBuf[0] |= 0x02;
		}
		if(g_eBaseAlm.ALM_CODE7 && g_eBaseAlm_Parallel[1].ALM_CODE7 && g_eBaseAlm_Parallel[2].ALM_CODE7) {
			aucBuf[0] |= 0x0C;
		} else if(g_eBaseAlm.ALM_CODE57 || g_eBaseAlm_Parallel[1].ALM_CODE57 || g_eBaseAlm_Parallel[2].ALM_CODE57) {
			aucBuf[0] |= 0x08;
		}
		if(g_eBaseAlm.ALM_CODE4 && g_eBaseAlm_Parallel[1].ALM_CODE4 && g_eBaseAlm_Parallel[2].ALM_CODE4) {
			aucBuf[0] |= 0x30;
		} else if(g_eBaseAlm.ALM_CODE54 || g_eBaseAlm_Parallel[1].ALM_CODE54 || g_eBaseAlm_Parallel[2].ALM_CODE54) {
			aucBuf[0] |= 0x20;
		}
		if(g_eBaseAlm.ALM_CODE5 && g_eBaseAlm_Parallel[1].ALM_CODE5 && g_eBaseAlm_Parallel[2].ALM_CODE5) {
			aucBuf[0] |= 0xC0;
		} else if(g_eBaseAlm.ALM_CODE55 || g_eBaseAlm_Parallel[1].ALM_CODE55 || g_eBaseAlm_Parallel[2].ALM_CODE55) {
			aucBuf[0] |= 0x80;
		}
		if(g_eBaseAlm.ALM_CODE3 && g_eBaseAlm_Parallel[1].ALM_CODE3 && g_eBaseAlm_Parallel[2].ALM_CODE3) {
			aucBuf[1] |= 0x03;
		} else if(g_eBaseAlm.ALM_CODE53 || g_eBaseAlm_Parallel[1].ALM_CODE53 || g_eBaseAlm_Parallel[2].ALM_CODE53) {
			aucBuf[1] |= 0x02;
		}
		if((g_eBaseAlm.ALM_CODE10 && g_eBaseAlm_Parallel[1].ALM_CODE10 && g_eBaseAlm_Parallel[2].ALM_CODE10) 
				|| (g_eBaseAlm.ALM_CODE11 && g_eBaseAlm_Parallel[1].ALM_CODE11 && g_eBaseAlm_Parallel[2].ALM_CODE11)) {
			aucBuf[1] |= 0x0C;
		}
		if((g_eBaseAlm.ALM_CODE12 && g_eBaseAlm_Parallel[1].ALM_CODE12 && g_eBaseAlm_Parallel[2].ALM_CODE12) 
				|| (g_eBaseAlm.ALM_CODE13 && g_eBaseAlm_Parallel[1].ALM_CODE13 && g_eBaseAlm_Parallel[2].ALM_CODE13)) {
			aucBuf[1] |= 0x30;
		}
		if(pstPackRVal->fCellTMax - pstPackRVal->fCellTMin > 20) {
			aucBuf[1] |= 0xC0;
		}
		if(g_eBaseAlm.ALM_CODE9 && g_eBaseAlm_Parallel[1].ALM_CODE9 && g_eBaseAlm_Parallel[2].ALM_CODE9) {
			aucBuf[2] |= 0x03;
		}
		if(g_eBaseAlm.ALM_CODE8 && g_eBaseAlm_Parallel[1].ALM_CODE8 && g_eBaseAlm_Parallel[2].ALM_CODE8) {
			aucBuf[2] |= 0x0C;
		}
		if(g_eBaseAlm.ALM_CODE15 && g_eBaseAlm_Parallel[1].ALM_CODE15 && g_eBaseAlm_Parallel[2].ALM_CODE15) {
			aucBuf[2] |= 0xC0;
		}
	}
	if(g_stPrl.ucDevNum == 2) {
		if(g_eBaseAlm.ALM_CODE56 || g_eBaseAlm_Parallel[1].ALM_CODE56) {
			aucBuf[0] |= 0x02;
		}
		if(g_eBaseAlm.ALM_CODE7 && g_eBaseAlm_Parallel[1].ALM_CODE7) {
			aucBuf[0] |= 0x0C;
		} else if(g_eBaseAlm.ALM_CODE57 || g_eBaseAlm_Parallel[1].ALM_CODE57) {
			aucBuf[0] |= 0x08;
		}
		if(g_eBaseAlm.ALM_CODE4 && g_eBaseAlm_Parallel[1].ALM_CODE4) {
			aucBuf[0] |= 0x30;
		} else if(g_eBaseAlm.ALM_CODE54 || g_eBaseAlm_Parallel[1].ALM_CODE54) {
			aucBuf[0] |= 0x20;
		}
		if(g_eBaseAlm.ALM_CODE5 && g_eBaseAlm_Parallel[1].ALM_CODE5) {
			aucBuf[0] |= 0xC0;
		} else if(g_eBaseAlm.ALM_CODE55 || g_eBaseAlm_Parallel[1].ALM_CODE55) {
			aucBuf[0] |= 0x80;
		}
		if(g_eBaseAlm.ALM_CODE3 && g_eBaseAlm_Parallel[1].ALM_CODE3) {
			aucBuf[1] |= 0x03;
		} else if(g_eBaseAlm.ALM_CODE53 || g_eBaseAlm_Parallel[1].ALM_CODE53) {
			aucBuf[1] |= 0x02;
		}
		if((g_eBaseAlm.ALM_CODE10 && g_eBaseAlm_Parallel[1].ALM_CODE10) 
				|| (g_eBaseAlm.ALM_CODE11 && g_eBaseAlm_Parallel[1].ALM_CODE11)) {
			aucBuf[1] |= 0x0C;
		}
		if((g_eBaseAlm.ALM_CODE12 && g_eBaseAlm_Parallel[1].ALM_CODE12) 
				|| (g_eBaseAlm.ALM_CODE13 && g_eBaseAlm_Parallel[1].ALM_CODE13)) {
			aucBuf[1] |= 0x30;
		}
		if(pstPackRVal->fCellTMax - pstPackRVal->fCellTMin > 20) {
			aucBuf[1] |= 0xC0;
		}
		if(g_eBaseAlm.ALM_CODE9 && g_eBaseAlm_Parallel[1].ALM_CODE9) {
			aucBuf[2] |= 0x03;
		}
		if(g_eBaseAlm.ALM_CODE8 && g_eBaseAlm_Parallel[1].ALM_CODE8) {
			aucBuf[2] |= 0x0C;
		}
		if(g_eBaseAlm.ALM_CODE15 && g_eBaseAlm_Parallel[1].ALM_CODE15) {
			aucBuf[2] |= 0xC0;
		}
	}
	aucBuf[4] = CFG_CELL_NUM;
	aucBuf[5] = CFG_TMP_NUM;
	CAN0_SendMsg(uiId, aucBuf, 8);
	/* eCanIdChgReq10, 0x18F880F3, intel */
//	{
//		static uint8_t s_ucTick = 0;
//		if(s_ucTick * g_stCfg.stLocal.usCyclePeriod < 250) {
//			s_ucTick++;
//		}
//		if(s_ucTick * g_stCfg.stLocal.usCyclePeriod >= 250) {
//			s_ucTick = 0;
//			uiId = eCanIdChgReq10;
//			RTC_Get();
//			memset(aucBuf, 0, 8);
//			aucBuf[0] = (calender.w_year & 0xFF);
//			aucBuf[1] = (calender.w_year >> 8) & 0xFF;
//			aucBuf[2] = calender.w_month;
//			aucBuf[3] = calender.w_day;
//			aucBuf[4] = calender.hour;
//			aucBuf[5] = calender.min;
//			CAN0_SendMsg(uiId, aucBuf, 8);
//		}
//	}
	/* eCanIdRexBox, 0x10086 */
	{
		uiId = eCanIdRexBox;
		memset(aucBuf, 0, 8);
		if(pstPackRVal->uBaseStat.stBaseStat.ucChgerON) {
			aucBuf[0] = 1;
		}
		if(GET_ALM0_CODE(4) || GET_ALM0_CODE(6) || GET_ALM0_CODE(11) || GET_ALM0_CODE(12) || GET_ALM0_CODE(14) || GET_ALM0_CODE(9)) {
			aucBuf[1] = 1;
		}
		usVal = pstPackRVal->fPackU * 10;
		aucBuf[2] = usVal >> 8;
		aucBuf[3] = usVal & 0xFF;
		CAN0_SendMsg(uiId, aucBuf, 8);
	}
}

/*******************************************************************************
* Function Name  : prl_hproc
* Description    : The host parallel service is called by the prl_proc and the call cycle is 1000 ms
* Input          : None
* Output         : None
* Return         : result, 1 write succeeded, 0 write failed
*******************************************************************************/
bool prl_hproc(void) {
	/* Communication Interruption Handling */
	for(uint8_t i=1;i<g_stPrl.ucDevNum;i++) {
		if(g_ausPrlComTick[i] > 0) {
			g_ausPrlComTick[i]--;
			if(0 == g_ausPrlComTick[i]) {		/* Master-slave communication is interrupted */
				PRL_DEBUG("node %d disconnect", i);
				g_stPrl.ucDevNum = 1;
				g_stPrl.ucSelfId = 0;
			}
		}
	}
	/* read client node from can */
	{
		static uint8_t s_ucTick = 0;
		if(s_ucTick * g_stCfg.stLocal.usCyclePeriod < 1000) {
			s_ucTick++;
		} else {
			s_ucTick = 0;
			for(uint8_t i=1;i<g_stPrl.ucDevNum;i++) {
				uint8_t ucData = sizeof(ECO_RTV_S) / 2;
				eco_can_data_send(0, i, 0x0F, 0x00, &ucData, 0);
				delay_1ms(1);
			}
		}
	}
	/* client mos control */
	{
		uint8_t ucVal = 0;
		if(g_stLocalArrayRVal.astPackRVal[g_stPrl.ucSelfId].ucChgEn == 0x55) {
			ucVal |= 0x01;
		}
		if(g_stLocalArrayRVal.astPackRVal[g_stPrl.ucSelfId].ucDsgEn == 0x55) {
			ucVal |= 0x02;
		}
		if(g_stLocalArrayRVal.astPackRVal[g_stPrl.ucSelfId].ucHeatEn == 0x55) {
			ucVal |= 0x04;
		}
		for(uint8_t i=1;i<g_stPrl.ucDevNum;i++) {
			eco_can_data_send(0, i, 0x10, 0x00, &ucVal, 1);
		}
	}
	/* screen sleep */
	if(g_stCfg.stLocal.iSreenSlpSec >= 0) {
		static uint32_t s_uiTick = 0;
		if(g_stLocalArrayRVal.uBaseStat.stBaseStat.ucStandby) {
			if(s_uiTick * g_stCfg.stLocal.usCyclePeriod < g_stCfg.stLocal.iSreenSlpSec * 1000) {
				s_uiTick++;
			}
		} else {
			MCU_1A_ON;
			MCU_3A_ON;
			MCU_3A_ON_old;
			g_stLocalArrayRVal.astPackRVal[g_stPrl.ucSelfId].uBaseStat.stBaseStat.ucDCOut1 = 1;
			s_uiTick = 0;
		}
		if(s_uiTick * g_stCfg.stLocal.usCyclePeriod >= g_stCfg.stLocal.iSreenSlpSec * 1000) {
			MCU_1A_OFF;
			MCU_3A_OFF;
			MCU_3A_OFF_old;
			g_stLocalArrayRVal.astPackRVal[g_stPrl.ucSelfId].uBaseStat.stBaseStat.ucDCOut1 = 0;
		}
	}
	/* low power sleep */
	if(g_stCfg.stLocal.iLowPwrSlpSec >= 0) {
		static uint32_t s_uiTick = 0;
		if(g_stLocalArrayRVal.uBaseStat.stBaseStat.ucStandby) {
			if(s_uiTick * g_stCfg.stLocal.usCyclePeriod < g_stCfg.stLocal.iLowPwrSlpSec * 1000 && g_stLocalArrayRVal.astPackRVal[g_stPrl.ucSelfId].fPackSoc < 20) {
				s_uiTick++;
			}
		} else {
			s_uiTick = 0;
		}
		if(s_uiTick * g_stCfg.stLocal.usCyclePeriod >= g_stCfg.stLocal.iLowPwrSlpSec * 1000) {
			/* Send 3 hibernation broadcasts to the outside world */
			uint8_t aucData[2] = {0};
			for(uint8_t i=0;i<3;i++) {
				for(uint8_t j=1;j<g_stPrl.ucDevNum;j++) {
					eco_can_data_send(j, j, 0x01, 0x20 + CFG_CELL_NUM + 48, aucData, 2);
					delay_1ms(10);
				}
			}
			/* Disable the services of the local node */
			PRL_DEBUG("timeout, going to sleep...\r\n");
//		g_stLocalArrayRVal.eLocalStat = eLocalStatSlp;
			g_stCfg.usGoRun = 0;
			cfg_save();
//			g_eBaseStat = eBStatSleep_01;
//			his_data_write();
//			System_Reset();
			g_bNeedSleep = true;
		}
	}
	/* standby sleep */
	if(g_stCfg.stLocal.iStandbySlpSec >= 0) {
		if(g_stLocalArrayRVal.uBaseStat.stBaseStat.ucStandby) {
			if(g_uiSlpTick * g_stCfg.stLocal.usCyclePeriod < g_stCfg.stLocal.iStandbySlpSec * 1000
				&& g_stLocalArrayRVal.astPackRVal[g_stPrl.ucSelfId].fPackSoc >= 20) {
				g_uiSlpTick++;
			}
		} else {
			g_uiSlpTick = 0;
		}
		if(g_uiSlpTick * g_stCfg.stLocal.usCyclePeriod >= g_stCfg.stLocal.iStandbySlpSec * 1000) {
			/* Send 3 hibernation broadcasts to the outside world */
			uint8_t aucData[2] = {0};
			for(uint8_t i=0;i<3;i++) {
				for(uint8_t j=1;j<g_stPrl.ucDevNum;j++) {
					eco_can_data_send(j, 1, 0x01, 0x20 + CFG_CELL_NUM + 48, aucData, 2);
					delay_1ms(10);
				}
			}
			/* Disable the services of the local node */
			PRL_DEBUG("timeout, going to sleep...\r\n");
//		g_stLocalArrayRVal.eLocalStat = eLocalStatSlp;
			g_stCfg.usGoRun = 0;
			cfg_save();
//			g_eBaseStat = eBStatSleep_01;
//			his_data_write();
//			System_Reset();
			g_bNeedSleep = true;
		}
	}
	/* parallel info calculation */
	ECO_RTV_S stEcoRtv = g_stEcoRtv;
	g_stEcoRtv_Parallel[0] = g_stEcoRtv;
	for(uint8_t  i = 1; i < g_stPrl.ucDevNum; i++) {
		stEcoRtv.usPackNominalAh += g_stEcoRtv_Parallel[i].usPackNominalAh;
		stEcoRtv.usCellSeriesNum += g_stEcoRtv_Parallel[i].usCellSeriesNum;
		stEcoRtv.usPackRTVolt += g_stEcoRtv_Parallel[i].usPackRTVolt;
		stEcoRtv.sPackRTCur += g_stEcoRtv_Parallel[i].sPackRTCur;
		stEcoRtv.usPackRealAH += g_stEcoRtv_Parallel[i].usPackRealAH;
		stEcoRtv.usPackLeftAH += g_stEcoRtv_Parallel[i].usPackLeftAH;
		stEcoRtv.usPackRTSoc = stEcoRtv.usPackLeftAH * 100 / stEcoRtv.usPackRealAH;
		if(stEcoRtv.usReqChgCur < g_stEcoRtv_Parallel[i].usReqChgCur) {
			stEcoRtv.usReqChgCur = g_stEcoRtv_Parallel[i].usReqChgCur;
		}
		if(stEcoRtv.usPeakLmtDsgCur < g_stEcoRtv_Parallel[i].usPeakLmtDsgCur) {
			stEcoRtv.usPeakLmtDsgCur = g_stEcoRtv_Parallel[i].usPeakLmtDsgCur;
		}
		if(stEcoRtv.usLmtChgCur < g_stEcoRtv_Parallel[i].usLmtChgCur) {
			stEcoRtv.usLmtChgCur = g_stEcoRtv_Parallel[i].usLmtChgCur;
		}
		if(stEcoRtv.usLmtDsgCur < g_stEcoRtv_Parallel[i].usLmtDsgCur) {
			stEcoRtv.usLmtDsgCur = g_stEcoRtv_Parallel[i].usLmtDsgCur;
		}
	}
	stEcoRtv.usPackRTVolt /= g_stPrl.ucDevNum;
	stEcoRtv.usPackRTSoc = round(100 * stEcoRtv.usPackLeftAH / stEcoRtv.usPackRealAH);
	g_stEcoRtv_Parallel[PRL_MAX_NODE_NUM] = stEcoRtv;

	/* Unsolicited sending */
	{
		static uint8_t s_ucTick = 0;
		if(s_ucTick == 0)	{
			if(g_stLocalArrayRVal.eLocalStat != eLocalStatCanUpgrade) {
				prl_host_auto_send();
			}
			s_ucTick++;
		} else {
			s_ucTick = 0;
		}
	}
	
	PRL_RETURN_TRUE;
}

/*******************************************************************************
* Function Name  : prl_cproc
* Description    : Slave parallel service
* Input          : None
* Output         : None
* Return         : result, 1 write succeeded, 0 write failed
*******************************************************************************/
bool prl_cproc(void) {
	/* Communication Interruption Handling  */
	if(g_ausPrlComTick[g_stPrl.ucSelfId] > 0) {
		g_ausPrlComTick[g_stPrl.ucSelfId]--;
		if(0 == g_ausPrlComTick[g_stPrl.ucSelfId]) {		/*  Master-slave communication is interrupted */
			PRL_DEBUG("disconnect from host");
			g_stPrl.ucDevNum = 1;
			g_stPrl.ucSelfId = 0;
//			g_stLocalArrayRVal.eLocalStat = eLocalStatStop;
		} else {
			g_stLocalArrayRVal.eLocalStat = eLocalStatRun;
		}
	}
	MCU_1A_OFF;
	g_stLocalArrayRVal.astPackRVal[g_stPrl.ucSelfId].uBaseStat.stBaseStat.ucDCOut1 = 0;
	if(g_stPrl.ucSelfId < 4) {
		memcpy(g_stEcoRtv_Parallel + g_stPrl.ucSelfId, &g_stEcoRtv, sizeof(ECO_RTV_S));
		memcpy(g_eBaseAlm_Parallel + g_stPrl.ucSelfId, &g_eBaseAlm, sizeof(BASE_ALM_CODE_E));
	}
	static uint8_t s_ucTick = 0;
	if(s_ucTick * g_stCfg.stLocal.usCyclePeriod < 500) {
		s_ucTick++;
	} else {
		s_ucTick = 0;
		if(g_stLocalArrayRVal.eLocalStat != eLocalStatCanUpgrade) {
			prl_cproc_send();
		}
	}
	/* low power sleep */
	if(g_stCfg.stLocal.iLowPwrSlpSec >= 0) {
		static uint32_t s_uiTick = 0;
		if(g_stLocalArrayRVal.uBaseStat.stBaseStat.ucStandby) {
			if(s_uiTick * g_stCfg.stLocal.usCyclePeriod < g_stCfg.stLocal.iLowPwrSlpSec * 1000 && g_stLocalArrayRVal.astPackRVal[g_stPrl.ucSelfId].fPackSoc < 20) {
				s_uiTick++;
			}
		} else {
			s_uiTick = 0;
		}
		if(s_uiTick * g_stCfg.stLocal.usCyclePeriod >= g_stCfg.stLocal.iLowPwrSlpSec * 1000) {
			/* Send 3 hibernation broadcasts to the outside world */
			uint8_t aucData[2] = {0};
			for(uint8_t i=0;i<3;i++) {
				for(uint8_t j=1;j<g_stPrl.ucDevNum;j++) {
					eco_can_data_send(j, j, 0x01, 0x20 + CFG_CELL_NUM + 48, aucData, 2);
					delay_1ms(10);
				}
			}
			/* Disable the services of the local node */
			PRL_DEBUG("timeout, going to sleep...\r\n");
//		g_stLocalArrayRVal.eLocalStat = eLocalStatSlp;
			g_stCfg.usGoRun = 0;
			cfg_save();
//			g_eBaseStat = eBStatSleep_01;
//			his_data_write();
//			System_Reset();
			g_bNeedSleep = true;
		}
	}
	/* standby sleep */
	if(g_stCfg.stLocal.iStandbySlpSec >= 0) {
		if(g_stLocalArrayRVal.uBaseStat.stBaseStat.ucStandby) {
			if(g_uiSlpTick * g_stCfg.stLocal.usCyclePeriod < g_stCfg.stLocal.iStandbySlpSec * 1000
				&& g_stLocalArrayRVal.astPackRVal[g_stPrl.ucSelfId].fPackSoc >= 20) {
				g_uiSlpTick++;
			}
		} else {
			g_uiSlpTick = 0;
		}
		if(g_uiSlpTick * g_stCfg.stLocal.usCyclePeriod >= g_stCfg.stLocal.iStandbySlpSec * 1000) {
			/* Send 3 hibernation broadcasts to the outside world */
			uint8_t aucData[2] = {0};
			for(uint8_t i=0;i<3;i++) {
				for(uint8_t j=1;j<g_stPrl.ucDevNum;j++) {
					eco_can_data_send(j, 1, 0x01, 0x20 + CFG_CELL_NUM + 48, aucData, 2);
					delay_1ms(10);
				}
			}
			/* Disable the services of the local node */
			PRL_DEBUG("timeout, going to sleep...\r\n");
//		g_stLocalArrayRVal.eLocalStat = eLocalStatSlp;
			g_stCfg.usGoRun = 0;
			cfg_save();
//			g_eBaseStat = eBStatSleep_01;
//			his_data_write();
//			System_Reset();
			g_bNeedSleep = true;
		}
	}
	PRL_RETURN_TRUE;
}

/*******************************************************************************
* Function Name  : prl_sproc
* Description    : Stand-alone service
* Input          : None
* Output         : None
* Return         : result, 1 write succeeded, 0 write failed
*******************************************************************************/
bool prl_sproc(void) {
	/* Stand-alone data updates */
	LOCAL_PACK_RVAL_S* pstPackRVal = g_stLocalArrayRVal.astPackRVal;
	g_stLocalArrayRVal.fRealAH = pstPackRVal->fPackRealAH;
	g_stLocalArrayRVal.fLeftAH = pstPackRVal->fPackLeftAH;
	g_stLocalArrayRVal.fU = pstPackRVal->fPackU;
	g_stLocalArrayRVal.fCur = pstPackRVal->fPackCur;
	g_stLocalArrayRVal.fSoc = pstPackRVal->fPackSoc;
	g_stLocalArrayRVal.fSoh = pstPackRVal->fPackSoh;
	g_stLocalArrayRVal.fLmtChgI = pstPackRVal->fLmtChgI;
	g_stLocalArrayRVal.fLmtDsgI = pstPackRVal->fLmtDsgI;
	g_stLocalArrayRVal.fReqChgI = pstPackRVal->fReqChgI;
	g_stLocalArrayRVal.fPeakLmtDsgI = pstPackRVal->fPeakLmtDsgI;
	g_stLocalArrayRVal.fPackUMin = pstPackRVal->fPackU;
	g_stLocalArrayRVal.fPackUMax = pstPackRVal->fPackU;
	g_stLocalArrayRVal.usPackUMaxId = 0;
	g_stLocalArrayRVal.usPackUMinId = 0;
	g_stLocalArrayRVal.fPackTMax = pstPackRVal->fCellTMax;
	g_stLocalArrayRVal.fPackTMin = pstPackRVal->fCellTMin;
	g_stLocalArrayRVal.usPackTMaxId = pstPackRVal->ucCellTMaxId;
	g_stLocalArrayRVal.usPackTMinId = pstPackRVal->ucCellTMinId;
	g_stLocalArrayRVal.fCellUMax = pstPackRVal->fCellUMax;
	g_stLocalArrayRVal.fCellUMin = pstPackRVal->fCellUMin;
	g_stLocalArrayRVal.usCellUMaxId = pstPackRVal->ucCellUMaxId;
	g_stLocalArrayRVal.usCellUMinId = pstPackRVal->ucCellUMinId;
	g_stLocalArrayRVal.uBaseStat = pstPackRVal->uBaseStat;
	g_stLocalArrayRVal.uErrCode = pstPackRVal->uErrCode;
	g_stLocalArrayRVal.ucChgEn = pstPackRVal->ucChgEn;
	g_stLocalArrayRVal.ucDsgEn = pstPackRVal->ucDsgEn;
	
	/* Sleep timeout judgment */
	/* screen sleep */
	if(g_stCfg.stLocal.iSreenSlpSec >= 0) {
		static uint32_t s_uiTick = 0;
		if(g_stLocalArrayRVal.uBaseStat.stBaseStat.ucStandby) {
			if(s_uiTick * g_stCfg.stLocal.usCyclePeriod < g_stCfg.stLocal.iSreenSlpSec * 1000) {
				s_uiTick++;
			}
		} else {
			MCU_1A_ON;
			MCU_3A_ON;
			MCU_3A_ON_old;
			g_stLocalArrayRVal.astPackRVal[g_stPrl.ucSelfId].uBaseStat.stBaseStat.ucDCOut1 = 1;
			s_uiTick = 0;
		}
		if(s_uiTick * g_stCfg.stLocal.usCyclePeriod >= g_stCfg.stLocal.iSreenSlpSec * 1000) {
			MCU_1A_OFF;
			MCU_3A_OFF;
			MCU_3A_OFF_old;
			g_stLocalArrayRVal.astPackRVal[g_stPrl.ucSelfId].uBaseStat.stBaseStat.ucDCOut1 = 0;
		}
	}
	/* low power sleep */
	if(g_stCfg.stLocal.iLowPwrSlpSec >= 0) {
		static uint32_t s_uiTick = 0;
		if(g_stLocalArrayRVal.uBaseStat.stBaseStat.ucStandby && g_stLocalArrayRVal.astPackRVal[g_stPrl.ucSelfId].fPackSoc < 20) {
			if(s_uiTick * g_stCfg.stLocal.usCyclePeriod < g_stCfg.stLocal.iLowPwrSlpSec * 1000) {
				s_uiTick++;
			}
		} else {
			s_uiTick = 0;
		}
		if(s_uiTick * g_stCfg.stLocal.usCyclePeriod >= g_stCfg.stLocal.iLowPwrSlpSec * 1000) {
			/* Disable the services of the local node */
			PRL_DEBUG("timeout, going to sleep...\r\n");
//		g_stLocalArrayRVal.eLocalStat = eLocalStatSlp;
			g_stCfg.usGoRun = 0;
			cfg_save();
//			g_eBaseStat = eBStatSleep_01;
//			his_data_write();
//			System_Reset();
			g_bNeedSleep = true;
		}
	}
	/* standby sleep */
	if(g_stCfg.stLocal.iStandbySlpSec >= 0) {
		static uint32_t s_uiTick = 0;
		if(g_stLocalArrayRVal.uBaseStat.stBaseStat.ucStandby) {
			if(s_uiTick * g_stCfg.stLocal.usCyclePeriod < g_stCfg.stLocal.iStandbySlpSec * 1000
				&& g_stLocalArrayRVal.astPackRVal[g_stPrl.ucSelfId].fPackSoc >= 20) {
				s_uiTick++;
			}
		} else {
			s_uiTick = 0;
		}
		if(s_uiTick * g_stCfg.stLocal.usCyclePeriod >= g_stCfg.stLocal.iStandbySlpSec * 1000) {
			/* Disable the services of the local node */
			PRL_DEBUG("timeout, going to sleep...\r\n");
//		g_stLocalArrayRVal.eLocalStat = eLocalStatSlp;
			g_stCfg.usGoRun = 0;
			cfg_save();
//			g_eBaseStat = eBStatSleep_01;
//			his_data_write();
//			System_Reset();
			g_bNeedSleep = true;
		}
	}
	
	/* Unsolicited sending */
//	{
//		static uint8_t s_ucTick = 0;
//		if(s_ucTick < 1)	{
//			s_ucTick++;
//		} else {
//			ptc_eco_proc();
//			s_ucTick = 0;
//		}
//	}
	ptc_eco_proc(); //2024.12.22 dgx  Fix the exception of CAN external packet cycle
	
	PRL_RETURN_TRUE;
}

/*******************************************************************************
* Function Name  : prl_init
* Description    : Parallel initialization
* Input          : None
* Output         : None
* Return         : result, 1 write succeeded, 0 write failed
*******************************************************************************/
bool prl_init(void) {uint8_t srcLen = strlen(g_stCfg.stLocal.aucBleName);
	uint8_t aucData[7];
	memcpy(aucData, g_stCfg.stLocal.aucBleName + srcLen - 7, 7);
	g_stPrl.uiSelfCode = 0;
	for(uint8_t i = 0; i < 7;i++) {
		if(aucData[i] >= '0' && aucData[i] <= '9') {
			g_stPrl.uiSelfCode += ((aucData[i] - '0') * pow(0x10, 6 - i));
		} else if(aucData[i] >= 'a' && aucData[i] <= 'f') {
			g_stPrl.uiSelfCode += ((aucData[i] - 'a' + 10) * pow(0x10, 6 - i));
		} else if(aucData[i] >= 'A' && aucData[i] <= 'F') {
			g_stPrl.uiSelfCode += ((aucData[i] - 'A' + 10) * pow(0x10, 6 - i));
		} else {
			//illegal bluetooth name
		}
	}
	g_stPrl.ucDevNum = 1;
	g_stPrl.ucChgEn = 0xAA;
	g_stPrl.ucDsgEn = 0xAA;
	g_stPrl.ucHeatEn = 0xAA;
	for(uint8_t i=0;i<PRL_MAX_NODE_NUM;i++) {
		g_ausPrlComTick[i] = PRL_COM_MSEC * 1000 / g_stCfg.stLocal.usCyclePeriod;
	}
	
	PRL_RETURN_TRUE;
}

/*******************************************************************************
* Function Name  : prl_proc
* Description    : Parallel service
* Input          : None
* Output         : None
* Return         : result, 1 write succeeded, 0 write failed
*******************************************************************************/
bool prl_proc(void) {
	// 10s sleep
	{
		WDG_DONE_H;
	  WDG_DONE_L;
		static uint8_t s_ucTick = 0;
		static uint8_t s_ucTick2 = 0;
		if(g_bNeedSleep) {
			uint8_t aucDataSleep[7] = {0};
			aucDataSleep[0] = 1;
		  s_ucTick++;
			if(g_stPrl.ucDevNum == 1) {
				ptc_eco_proc();
			} else {
				if(s_ucTick2 == 0) {
					eco_can_data_send(g_stPrl.ucSelfId, 1, 0x31, 0, aucDataSleep, 7);
					s_ucTick2++;
				}
			}
			float fFalse = 0;
			afe_set_ao(eAfeRamCodeCHGMOS, 1, &fFalse);
			PRE_DSG_L;
			afe_set_ao(eAfeRamCodeDSGMOS, 1, &fFalse);
			DECOUPLER_H;
//			SW_EN_LED_L;
			if(g_stPrl.ucSelfId == 0 && g_stPrl.ucDevNum == 1) {
				if(s_ucTick * g_stCfg.stLocal.usCyclePeriod >= 2000) {
					g_stLocalArrayRVal.eLocalStat = eLocalStatSlp;
					g_eBaseStat = eBStatSleep_01;
					his_data_write();
					delay_1ms(500);
					SW_EN_LED_L;
					System_Reset();
					s_ucTick = 0;
				}
			} else if(g_stPrl.ucSelfId == 0 && g_stPrl.ucDevNum > 1) {
				if(s_ucTick * g_stCfg.stLocal.usCyclePeriod >= 2000) {//5000
					g_stLocalArrayRVal.eLocalStat = eLocalStatSlp;
					g_eBaseStat = eBStatSleep_01;
					his_data_write();
					delay_1ms(500);
					SW_EN_LED_L;
					System_Reset();
					s_ucTick = 0;
				}
			} else {
				if(s_ucTick * g_stCfg.stLocal.usCyclePeriod >= 1000) {
					g_stLocalArrayRVal.eLocalStat = eLocalStatSlp;
					g_eBaseStat = eBStatSleep_01;
					his_data_write();
					delay_1ms(500);
					SW_EN_LED_L;
					System_Reset();
					s_ucTick = 0;
				}
			}
			PRL_RETURN_FALSE;
		} else {
			s_ucTick = 0;
		}
  }
	
	//pack button trigger sleep
	static uint8_t ucSWTick = 0;
	if(SW_WK_READ == 1) {
		if(ucSWTick * g_stCfg.stLocal.usCyclePeriod < 500) { //500
			ucSWTick++;
		}
	}
	if(ucSWTick * g_stCfg.stLocal.usCyclePeriod >= 500) {
			if(prl_host()) {
				/* Send 3 hibernation broadcasts to the outside world */
				uint8_t aucData[2] = {0};
				for(uint8_t i = 0; i < 3; i++) {
					for(uint8_t j = 1; j < g_stPrl.ucDevNum; j++) {
						eco_can_data_send(j, 1, 0x01, 0x20 + CFG_CELL_NUM + 48, aucData, 2);
						delay_1ms(10);
					}
				}
			}
			PRL_DEBUG("pack button1 pressed timeout, going to sleep...\r\n");
			g_stCfg.usGoRun = 2;
			cfg_save();
			g_bNeedSleep = true;
//		}
	}
	if(SW_WK_READ != 1) {
		ucSWTick = 0;
	}
#ifdef PARALLEL_EN
	//auto send parallel consult frame
	{
		static uint8_t s_ucTick = 0;
		if(s_ucTick * g_stCfg.stLocal.usCyclePeriod < 5000) {
			s_ucTick++;
		} else {
			s_ucTick = 0;
			uint8_t srcLen = strlen(g_stCfg.stLocal.aucBleName);
			uint8_t aucData[7];
			memcpy(aucData, g_stCfg.stLocal.aucBleName + srcLen - 7, 7);
			eco_can_data_send(0, 0, 0x0E, 0, aucData, 7);
		}
	}
#endif
	/* Stand-alone processing */
	if(prl_single()) {
		return prl_sproc();
	}
	/* Host processing */
	if(prl_host()) {
		return prl_hproc();
	}
	/* Client processing */
	if(prl_client()) {
		return prl_cproc();
	}
	
	PRL_RETURN_FALSE;
}

void prl_cproc_send(void) {
	uint16_t usVal;
	memset(g_aucCanSBuf, 1, sizeof(ECO_RTV_S));
	for(uint16_t i=0;i<sizeof(ECO_RTV_S) / 2;i++) {
		usVal = eco_read_reg(g_stPrl.ucSelfId, 1, 0 + i);
//		g_aucCanSBuf[i * 2] = usVal >> 8;
//		g_aucCanSBuf[i * 2 + 1] = usVal & 0xFF;
		g_aucCanSBuf[i * 2] = usVal & 0xFF;
		g_aucCanSBuf[i * 2 + 1] = usVal >> 8;
	}
	eco_can_data_send(g_stPrl.ucSelfId, 1, 0x65, 0, g_aucCanSBuf, sizeof(ECO_RTV_S));
}
