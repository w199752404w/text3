#include "local.h" 
#include "history.h"
#include "bsp_rtc.h"
#include "parallel.h"
#include "protocol.h"
#include "bsp_adc.h"
#include "bsp_gpio.h"
#include "bsp_exti.h"
#include "bsp_sh367309.h"
#include "history.h"
#include "parallel.h"
#include "eco.h"


#include <stdio.h>
#include <string.h>
#include <math.h>

LOCAL_ARRAY_RVAL_S g_stLocalArrayRVal = {0};
BASE_STATU_E g_eBaseStat;
BASE_ALM_CODE_E g_eBaseAlm;
BASE_ALM_CODE_E g_eBaseAlm_Parallel[PRL_MAX_NODE_NUM] = {0};

LOCAL_CR_PARA_S g_stLocalCRPara = {0};
LOCAL_CL_PARA_S g_stLocalCLPara = {0};
LOCAL_DL_PARA_S g_stLocalDLPara = {0};
LOCAL_PDCL_PARA_S g_stLocalPDCLPara = {0};
//LOCAL_3SPDCL_PARA_S g_stLocal3SPDCLPara = {0};
//LOCAL_30SPDCL_PARA_S g_stLocal30SPDCLPara = {0};
LOCAL_PDCLMOS_PARA_S g_stLocalPDCLMOSPara = {0};
LOCAL_SOC_CALI_S g_stLocalSocCali = {0};

uint8_t g_ucChgFull = 0;
uint8_t g_ucFullChgRec = 0;
uint8_t g_ucDsgEmpty = 0;
uint8_t g_ucDsgZero = 0;
uint8_t g_ucStandbyCali = 0;
uint8_t  balanc[16] = {0};
uint8_t g_ucHeatingStat = 0;
uint8_t g_ucExtrinsicFlag = 0;
uint32_t s_iTick = 0;
float fReqChgI100[80] = {0};
float g_fPackSoc = 0;
uint32_t g_iReqChgITick = 0;

uint32_t g_auiAlmCode0[4] = {0};		//every bit is an alarm code
uint32_t g_auiAlmCode1[4] = {0};		//every bit is an alarm code
uint8_t g_ucAlmLevel = 0;		//0- no alarm; 1- high level alarm; 2- middle level alarm; 3- low level alarm
uint8_t g_ucAlmLeve3 = 0;		//0- no alarm; 1- high level alarm; 2- middle level alarm; 3- low level alarm
uint8_t g_ucAlmLevel2 = 0;	//0- no alarm; 3- high level alarm; 2- middle level alarm; 1- low level alarm
bool g_ChgDisable[16]={0};        //0- Chg open; 1- Chg close
bool g_DsgDisable[16]={0};        //0- Dsg open; 1- Dsg close
bool g_BalanceFlag = 0;
bool g_BalanceFlag_2 = 0;
bool g_bNeedHeat = false;
bool g_bChgLowTempFlag = false;
bool g_bTempIsNotRisingFlag = false;
bool g_bHeatingTimeoutFlag = false;
bool g_bHeatingOverNumberFlag = false;
bool g_bSocCaliFlag = false;

/*******************************************************************************
* Function Name  : local_init
* Description    : Native service initialization, it is worth noting that all data refreshes here must be automic in nature, because external interrupts such as CAN will interrupt this function and get intermediate processing state at any time
* Input          : None
* Output         : None
* Return         : result, 1 write succeeded, 0 write failed
*******************************************************************************/
bool local_init(void) {
	g_stCfg.stAfe.aucRomByte[24] = g_stAfe.uRom.stCode.TR;
	LOCAL_PACK_RVAL_S* pstPackRVal = g_stLocalArrayRVal.astPackRVal + g_stPrl.ucSelfId;
	/* Cell Remaining Capacity, Actual Capacity, SOC, SOH Initialization */
	HIS_DATA_S stHData;
	uint8_t ucLoadFail = 0;
	
	uint8_t ucRetries = 0;
	while(ucRetries < MAX_RETRIES) {
		if(his_data_read(ucRetries, &stHData)) {
			pstPackRVal->fPackSoc = stHData.aucCellSoc[0];
			pstPackRVal->fPackSoc = stHData.stRtv.usPackRTSoc;
			for(uint8_t i=0;i<CFG_CELL_NUM;i++) {
				if(stHData.ausCellRealAh[i] < g_stCfg.stLocal.usDesignAH * 0.7 || stHData.ausCellRealAh[i] > g_stCfg.stLocal.usDesignAH * 1.3) {
					ucLoadFail = 1;
					break;
				}
				if(stHData.aucCellSoc[i] > 100) {
					ucLoadFail = 1;
					break;
				}
				pstPackRVal->afCellRealAH[i] = stHData.ausCellRealAh[i];
				pstPackRVal->afCellLeftAH[i] = stHData.ausCellRealAh[i] * stHData.aucCellSoc[i] * 0.01;
				pstPackRVal->afCellSoc[i] = stHData.aucCellSoc[i];
				if(pstPackRVal->fPackSoc > pstPackRVal->afCellSoc[i]) {
					pstPackRVal->fPackSoc = pstPackRVal->afCellSoc[i];
				}
			}
			pstPackRVal->fCDATA = stHData.stRtv.fCDATA;
			pstPackRVal->usCycle = stHData.stRtv.usCycleCnt;
			uint32_t tt=DS1302_gtime();
			if(DS1302_gtime() - stHData.dt > 24 * 3600) {	//standby more then 24h, soc calibration
				g_ucStandbyCali = 1;
			}
			break;
		}
		if(ucRetries == 0) {	//2025.2.28 dgx
			ucLoadFail = 0;
		}
		ucRetries++;
	}
	if(ucRetries >= MAX_RETRIES) {
		for(uint8_t i=0;i<CFG_CELL_NUM;i++) {
			pstPackRVal->afCellLeftAH[i] = g_stCfg.stLocal.usDesignAH * 0.5;
			pstPackRVal->afCellRealAH[i] = g_stCfg.stLocal.usDesignAH;
			pstPackRVal->afCellSoc[i] = 50;
		}
		pstPackRVal->usCycle = 0;
		g_eBaseStat = eBStatStewing;
		g_ucStandbyCali = 1;
	}
	
//	if(his_data_read(0, &stHData)) {
//		pstPackRVal->fPackSoc = stHData.aucCellSoc[0];
//		pstPackRVal->fPackSoc = stHData.stRtv.usPackRTSoc;
//		for(uint8_t i=0;i<CFG_CELL_NUM;i++) {
//			if(stHData.ausCellRealAh[i] < g_stCfg.stLocal.usDesignAH * 0.7 || stHData.ausCellRealAh[i] > g_stCfg.stLocal.usDesignAH * 1.3) {
//				ucLoadFail = 1;
//				break;
//			}
//			if(stHData.aucCellSoc[i] > 100) {
//				ucLoadFail = 1;
//				break;
//			}
//			pstPackRVal->afCellRealAH[i] = stHData.ausCellRealAh[i];
//			pstPackRVal->afCellLeftAH[i] = stHData.ausCellRealAh[i] * stHData.aucCellSoc[i] * 0.01;
//			pstPackRVal->afCellSoc[i] = stHData.aucCellSoc[i];
//			if(pstPackRVal->fPackSoc > pstPackRVal->afCellSoc[i]) {
//			  pstPackRVal->fPackSoc = pstPackRVal->afCellSoc[i];
//		  }
//		}
//		pstPackRVal->fCDATA = stHData.stRtv.fCDATA;
//		pstPackRVal->usCycle = stHData.stRtv.usCycleCnt;
//		uint32_t tt=DS1302_gtime();
//		if(DS1302_gtime() - stHData.dt > 24 * 3600) {	//standby more then 24h, soc calibration
//			g_ucStandbyCali = 1;
//		}
//	} else {	//first time start
//		for(uint8_t i=0;i<CFG_CELL_NUM;i++) {
//			pstPackRVal->afCellLeftAH[i] = g_stCfg.stLocal.usDesignAH * 0.5;
//			pstPackRVal->afCellRealAH[i] = g_stCfg.stLocal.usDesignAH;
//			pstPackRVal->afCellSoc[i] = 50;
//		}
//		pstPackRVal->usCycle = 0;
//		g_eBaseStat = eBStatStewing;
//		g_ucStandbyCali = 1;
//	}
	if(ucLoadFail != 0) {
		for(uint8_t i=0;i<CFG_CELL_NUM;i++) {
			pstPackRVal->afCellLeftAH[i] = g_stCfg.stLocal.usDesignAH * 0.5;
			pstPackRVal->afCellRealAH[i] = g_stCfg.stLocal.usDesignAH;
			pstPackRVal->afCellSoc[i] = 50;
		}
		pstPackRVal->usCycle = 0;
		g_eBaseStat = eBStatStewing;
		g_ucStandbyCali = 1;
	}
	/* Continuous charge limit init */
	g_stLocalCRPara.afTemp[0] = 0;
	g_stLocalCRPara.afTemp[1] = 2;
	g_stLocalCRPara.afTemp[2] = 5;
	g_stLocalCRPara.afTemp[3] = 7;
	g_stLocalCRPara.afTemp[4] = 10;
	g_stLocalCRPara.afTemp[5] = 25;
	g_stLocalCRPara.afTemp[6] = 45;
	g_stLocalCRPara.afTemp[7] = 55;
	g_stLocalCRPara.afTemp[8] = 63;
	g_stLocalCRPara.afSoc[0] = 0;
	g_stLocalCRPara.afSoc[1] = 5;
	g_stLocalCRPara.afSoc[2] = 10;
	g_stLocalCRPara.afSoc[3] = 20;
	g_stLocalCRPara.afSoc[4] = 30;
	g_stLocalCRPara.afSoc[5] = 40;
	g_stLocalCRPara.afSoc[6] = 50;
	g_stLocalCRPara.afSoc[7] = 60;
	g_stLocalCRPara.afSoc[8] = 70;
	g_stLocalCRPara.afSoc[9] = 80;
	g_stLocalCRPara.afSoc[10] = 90;
	g_stLocalCRPara.afSoc[11] = 95;
	g_stLocalCRPara.afSoc[12] = 100;
	g_stLocalCRPara.aafChgRate[0][0] = 0.2;
	g_stLocalCRPara.aafChgRate[0][1] = 0.2;
	g_stLocalCRPara.aafChgRate[0][2] = 0.1;
	g_stLocalCRPara.aafChgRate[0][3] = 0.1;
	g_stLocalCRPara.aafChgRate[0][4] = 0.1;
	g_stLocalCRPara.aafChgRate[0][5] = 0.1;
	g_stLocalCRPara.aafChgRate[0][6] = 0.1;
	g_stLocalCRPara.aafChgRate[0][7] = 0.1;
	g_stLocalCRPara.aafChgRate[0][8] = 0.1;
	g_stLocalCRPara.aafChgRate[0][9] = 0.1;
	g_stLocalCRPara.aafChgRate[0][10] = 0.06;
	g_stLocalCRPara.aafChgRate[0][11] = 0.06;
	g_stLocalCRPara.aafChgRate[0][12] = 0;
	g_stLocalCRPara.aafChgRate[1][0] = 0.4;
	g_stLocalCRPara.aafChgRate[1][1] = 0.4;
	g_stLocalCRPara.aafChgRate[1][2] = 0.2;
	g_stLocalCRPara.aafChgRate[1][3] = 0.2;
	g_stLocalCRPara.aafChgRate[1][4] = 0.2;
	g_stLocalCRPara.aafChgRate[1][5] = 0.2;
	g_stLocalCRPara.aafChgRate[1][6] = 0.2;
	g_stLocalCRPara.aafChgRate[1][7] = 0.2;
	g_stLocalCRPara.aafChgRate[1][8] = 0.2;
	g_stLocalCRPara.aafChgRate[1][9] = 0.2;
	g_stLocalCRPara.aafChgRate[1][10] = 0.12;
	g_stLocalCRPara.aafChgRate[1][11] = 0.12;
	g_stLocalCRPara.aafChgRate[1][12] = 0;
	g_stLocalCRPara.aafChgRate[2][0] = 0.4;
	g_stLocalCRPara.aafChgRate[2][1] = 0.4;
	g_stLocalCRPara.aafChgRate[2][2] = 0.2;
	g_stLocalCRPara.aafChgRate[2][3] = 0.2;
	g_stLocalCRPara.aafChgRate[2][4] = 0.2;
	g_stLocalCRPara.aafChgRate[2][5] = 0.2;
	g_stLocalCRPara.aafChgRate[2][6] = 0.2;
	g_stLocalCRPara.aafChgRate[2][7] = 0.2;
	g_stLocalCRPara.aafChgRate[2][8] = 0.2;
	g_stLocalCRPara.aafChgRate[2][9] = 0.2;
	g_stLocalCRPara.aafChgRate[2][10] = 0.12;
	g_stLocalCRPara.aafChgRate[2][11] = 0.12;
	g_stLocalCRPara.aafChgRate[2][12] = 0;
	g_stLocalCRPara.aafChgRate[3][0] = 0.8;
	g_stLocalCRPara.aafChgRate[3][1] = 0.8;
	g_stLocalCRPara.aafChgRate[3][2] = 0.6;
	g_stLocalCRPara.aafChgRate[3][3] = 0.6;
	g_stLocalCRPara.aafChgRate[3][4] = 0.6;
	g_stLocalCRPara.aafChgRate[3][5] = 0.6;
	g_stLocalCRPara.aafChgRate[3][6] = 0.4;
	g_stLocalCRPara.aafChgRate[3][7] = 0.4;
	g_stLocalCRPara.aafChgRate[3][8] = 0.4;
	g_stLocalCRPara.aafChgRate[3][9] = 0.2;
	g_stLocalCRPara.aafChgRate[3][10] = 0.2;
	g_stLocalCRPara.aafChgRate[3][11] = 0.12;
	g_stLocalCRPara.aafChgRate[3][12] = 0;
	for(uint8_t i=4;i<=7;i++) {
		g_stLocalCRPara.aafChgRate[i][0] = 1;
		g_stLocalCRPara.aafChgRate[i][1] = 1;
		g_stLocalCRPara.aafChgRate[i][2] = 1;
		g_stLocalCRPara.aafChgRate[i][3] = 1;
		g_stLocalCRPara.aafChgRate[i][4] = 1;
		g_stLocalCRPara.aafChgRate[i][5] = 1;
		g_stLocalCRPara.aafChgRate[i][6] = 1;
		g_stLocalCRPara.aafChgRate[i][7] = 1;
		g_stLocalCRPara.aafChgRate[i][8] = 1;
		g_stLocalCRPara.aafChgRate[i][9] = 1;
		g_stLocalCRPara.aafChgRate[i][10] = 0.8;
		g_stLocalCRPara.aafChgRate[i][11] = 0.4;
		g_stLocalCRPara.aafChgRate[i][12] = 0;
	}
	/* Feedback current limit init */
	g_stLocalCLPara.afTempMin[0] = -35;
	g_stLocalCLPara.afTempMin[1] = -30;
	g_stLocalCLPara.afTempMin[2] = -20;
	g_stLocalCLPara.afTempMin[3] = -10;
	g_stLocalCLPara.afTempMin[4] = 0;
	g_stLocalCLPara.afTempMin[5] = 5;
	g_stLocalCLPara.afTempMin[6] = 10;
	g_stLocalCLPara.afTempMax[0] = 20;
	g_stLocalCLPara.afTempMax[1] = 40;
	g_stLocalCLPara.afTempMax[2] = 55;
	g_stLocalCLPara.afTempMax[3] = 60;
	g_stLocalCLPara.afTempMax[4] = 63;
	g_stLocalCLPara.afCellVMax[0] = 3500;
	g_stLocalCLPara.afCellVMax[1] = 3450;
	g_stLocalCLPara.afCellVMax[2] = 3400;
	g_stLocalCLPara.afCellVMax[3] = 3350;
	g_stLocalCLPara.afCellVMax[4] = 3300;
	g_stLocalCLPara.afCellVMax[5] = 3100;
	g_stLocalCLPara.afCellVMin[0] = 2900;
	g_stLocalCLPara.afCellVMin[1] = 2850;
	g_stLocalCLPara.afCellVMin[2] = 2800;
	g_stLocalCLPara.afCellVMin[3] = 2750;
	g_stLocalCLPara.afCellVMin[4] = 2700;
	g_stLocalCLPara.afCellVMin[5] = 2650;
	g_stLocalCLPara.afCellVMin[6] = 2600;
	g_stLocalCLPara.afCellVMin[7] = 2550;
	for(uint8_t i = 0; i < 6; i++) {
		g_stLocalCLPara.aafChgRate1[0][i] = 0;
		g_stLocalCLPara.aafChgRate1[1][i] = 0;
		g_stLocalCLPara.aafChgRate1[2][i] = 0;
		g_stLocalCLPara.aafChgRate1[3][i] = 0;
	}
	g_stLocalCLPara.aafChgRate1[4][0] = 2;
	g_stLocalCLPara.aafChgRate1[4][1] = 0.15;
	g_stLocalCLPara.aafChgRate1[4][2] = 0.2;
	g_stLocalCLPara.aafChgRate1[4][3] = 0.25;
	g_stLocalCLPara.aafChgRate1[4][4] = 0.25;
	g_stLocalCLPara.aafChgRate1[4][5] = 0.25;
	g_stLocalCLPara.aafChgRate1[5][0] = 0;
	g_stLocalCLPara.aafChgRate1[5][1] = 0.25;
	g_stLocalCLPara.aafChgRate1[5][2] = 0.5;
	g_stLocalCLPara.aafChgRate1[5][3] = 1;
	g_stLocalCLPara.aafChgRate1[5][4] = 1.5;
	g_stLocalCLPara.aafChgRate1[5][5] = 1.5;
	g_stLocalCLPara.aafChgRate1[6][0] = 0;
	g_stLocalCLPara.aafChgRate1[6][1] = 0.25;
	g_stLocalCLPara.aafChgRate1[6][2] = 0.5;
	g_stLocalCLPara.aafChgRate1[6][3] = 1.5;
	g_stLocalCLPara.aafChgRate1[6][4] = 1.5;
	g_stLocalCLPara.aafChgRate1[6][5] = 1.5;
	for(uint8_t i = 0; i < 9; i++) {
		g_stLocalCLPara.aafChgRate2[0][i] = 0;
		g_stLocalCLPara.aafChgRate2[1][i] = 0;
		g_stLocalCLPara.aafChgRate2[2][i] = 0;
		g_stLocalCLPara.aafChgRate2[3][i] = 0;
	}
	g_stLocalCLPara.aafChgRate2[4][0] = 0.25;
	g_stLocalCLPara.aafChgRate2[4][1] = 0.25;
	g_stLocalCLPara.aafChgRate2[4][2] = 0.25;
	g_stLocalCLPara.aafChgRate2[4][3] = 0.25;
	g_stLocalCLPara.aafChgRate2[4][4] = 0.25;
	g_stLocalCLPara.aafChgRate2[4][5] = 0.25;
	g_stLocalCLPara.aafChgRate2[4][6] = 0.25;
	g_stLocalCLPara.aafChgRate2[4][7] = 0.25;
	g_stLocalCLPara.aafChgRate2[4][8] = 0.1;
	g_stLocalCLPara.aafChgRate2[5][0] = 1.5;
	g_stLocalCLPara.aafChgRate2[5][1] = 1.5;
	g_stLocalCLPara.aafChgRate2[5][2] = 1.5;
	g_stLocalCLPara.aafChgRate2[5][3] = 1.5;
	g_stLocalCLPara.aafChgRate2[5][4] = 1.5;
	g_stLocalCLPara.aafChgRate2[5][5] = 1.5;
	g_stLocalCLPara.aafChgRate2[5][6] = 1.5;
	g_stLocalCLPara.aafChgRate2[5][7] = 1.5;
	g_stLocalCLPara.aafChgRate2[5][8] = 0.75;
	g_stLocalCLPara.aafChgRate2[6][0] = 1.5;
	g_stLocalCLPara.aafChgRate2[6][1] = 1.5;
	g_stLocalCLPara.aafChgRate2[6][2] = 1.5;
	g_stLocalCLPara.aafChgRate2[6][3] = 1.5;
	g_stLocalCLPara.aafChgRate2[6][4] = 1.5;
	g_stLocalCLPara.aafChgRate2[6][5] = 1.5;
	g_stLocalCLPara.aafChgRate2[6][6] = 1.5;
	g_stLocalCLPara.aafChgRate2[6][7] = 1.5;
	g_stLocalCLPara.aafChgRate2[6][8] = 0.75;
	for(uint8_t i = 0; i < 3; i++) {
		g_stLocalCLPara.aafChgRate3[i][0] = 0;
		g_stLocalCLPara.aafChgRate3[i][1] = 0.25;
		g_stLocalCLPara.aafChgRate3[i][2] = 0.5;
		g_stLocalCLPara.aafChgRate3[i][3] = 1.5;
		g_stLocalCLPara.aafChgRate3[i][4] = 1.5;
		g_stLocalCLPara.aafChgRate3[i][5] = 1.5;
	}
	g_stLocalCLPara.aafChgRate3[3][0] = 0;
	g_stLocalCLPara.aafChgRate3[3][1] = 0.25;
	g_stLocalCLPara.aafChgRate3[3][2] = 0.5;
	g_stLocalCLPara.aafChgRate3[3][3] = 1.25;
	g_stLocalCLPara.aafChgRate3[3][4] = 1.25;
	g_stLocalCLPara.aafChgRate3[3][5] = 1.25;
	g_stLocalCLPara.aafChgRate3[4][0] = 0;
	g_stLocalCLPara.aafChgRate3[4][1] = 0.25;
	g_stLocalCLPara.aafChgRate3[4][2] = 0.5;
	g_stLocalCLPara.aafChgRate3[4][3] = 1;
	g_stLocalCLPara.aafChgRate3[4][4] = 1;
	g_stLocalCLPara.aafChgRate3[4][5] = 1;
	g_stLocalCLPara.aafChgRate3[5][0] = 0;
	g_stLocalCLPara.aafChgRate3[5][1] = 0.25;
	g_stLocalCLPara.aafChgRate3[5][2] = 0.25;
	g_stLocalCLPara.aafChgRate3[5][3] = 0.25;
	g_stLocalCLPara.aafChgRate3[5][4] = 0.25;
	g_stLocalCLPara.aafChgRate3[5][5] = 0.25;
	for(uint8_t i = 0; i < 3; i++) {
		g_stLocalCLPara.aafChgRate4[i][0] = 1.5;
		g_stLocalCLPara.aafChgRate4[i][1] = 1.5;
		g_stLocalCLPara.aafChgRate4[i][2] = 1.5;
		g_stLocalCLPara.aafChgRate4[i][3] = 1.5;
		g_stLocalCLPara.aafChgRate4[i][4] = 1.5;
		g_stLocalCLPara.aafChgRate4[i][5] = 1.5;
		g_stLocalCLPara.aafChgRate4[i][6] = 1.5;
		g_stLocalCLPara.aafChgRate4[i][7] = 1.5;
		g_stLocalCLPara.aafChgRate4[i][8] = 0.75;
	}
	g_stLocalCLPara.aafChgRate4[3][0] = 1.25;
	g_stLocalCLPara.aafChgRate4[3][1] = 1.25;
	g_stLocalCLPara.aafChgRate4[3][2] = 1.25;
	g_stLocalCLPara.aafChgRate4[3][3] = 1.25;
	g_stLocalCLPara.aafChgRate4[3][4] = 1.25;
	g_stLocalCLPara.aafChgRate4[3][5] = 1.25;
	g_stLocalCLPara.aafChgRate4[3][6] = 1.25;
	g_stLocalCLPara.aafChgRate4[3][7] = 1.25;
	g_stLocalCLPara.aafChgRate4[3][8] = 0.75;
	g_stLocalCLPara.aafChgRate4[4][0] = 1;
	g_stLocalCLPara.aafChgRate4[4][1] = 1;
	g_stLocalCLPara.aafChgRate4[4][2] = 1;
	g_stLocalCLPara.aafChgRate4[4][3] = 1;
	g_stLocalCLPara.aafChgRate4[4][4] = 1;
	g_stLocalCLPara.aafChgRate4[4][5] = 1;
	g_stLocalCLPara.aafChgRate4[4][6] = 1;
	g_stLocalCLPara.aafChgRate4[4][7] = 1;
	g_stLocalCLPara.aafChgRate4[4][8] = 0.75;
	g_stLocalCLPara.aafChgRate4[5][0] = 0.25;
	g_stLocalCLPara.aafChgRate4[5][1] = 0.25;
	g_stLocalCLPara.aafChgRate4[5][2] = 0.25;
	g_stLocalCLPara.aafChgRate4[5][3] = 0.25;
	g_stLocalCLPara.aafChgRate4[5][4] = 0.25;
	g_stLocalCLPara.aafChgRate4[5][5] = 0.25;
	g_stLocalCLPara.aafChgRate4[5][6] = 0.25;
	g_stLocalCLPara.aafChgRate4[5][7] = 0.25;
	g_stLocalCLPara.aafChgRate4[5][8] = 0.25;

	/* Discharge current limit init */
	g_stLocalDLPara.afTempMin[0] = -35;
	g_stLocalDLPara.afTempMin[1] = -30;
	g_stLocalDLPara.afTempMin[2] = -20;
	g_stLocalDLPara.afTempMin[3] = -10;
	g_stLocalDLPara.afTempMin[4] = 0;
	g_stLocalDLPara.afTempMin[5] = 5;
	g_stLocalDLPara.afTempMin[6] = 10;
	g_stLocalDLPara.afTempMax[0] = 20;
	g_stLocalDLPara.afTempMax[1] = 40;
	g_stLocalDLPara.afTempMax[2] = 55;
	g_stLocalDLPara.afTempMax[3] = 60;
	g_stLocalDLPara.afTempMax[4] = 63;
	g_stLocalDLPara.afCellVMax[0] = 3500;
	g_stLocalDLPara.afCellVMax[1] = 3450;
	g_stLocalDLPara.afCellVMax[2] = 3400;
	g_stLocalDLPara.afCellVMax[3] = 3350;
	g_stLocalDLPara.afCellVMax[4] = 3300;
	g_stLocalDLPara.afCellVMax[5] = 3100;
	g_stLocalDLPara.afCellVMin[0] = 2900;
	g_stLocalDLPara.afCellVMin[1] = 2850;
	g_stLocalDLPara.afCellVMin[2] = 2800;
	g_stLocalDLPara.afCellVMin[3] = 2750;
	g_stLocalDLPara.afCellVMin[4] = 2700;
	g_stLocalDLPara.afCellVMin[5] = 2650;
	g_stLocalDLPara.afCellVMin[6] = 2600;
	g_stLocalDLPara.afCellVMin[7] = 2550;
	g_stLocalDLPara.aafDsgRate1[1][0] = 0.25;
	g_stLocalDLPara.aafDsgRate1[1][1] = 0.25;
	g_stLocalDLPara.aafDsgRate1[1][2] = 0.25;
	g_stLocalDLPara.aafDsgRate1[1][3] = 0.25;
	g_stLocalDLPara.aafDsgRate1[1][4] = 0.25;
	g_stLocalDLPara.aafDsgRate1[1][5] = 0.25;
	g_stLocalDLPara.aafDsgRate1[2][0] = 0.5;
	g_stLocalDLPara.aafDsgRate1[2][1] = 0.5;
	g_stLocalDLPara.aafDsgRate1[2][2] = 0.5;
	g_stLocalDLPara.aafDsgRate1[2][3] = 0.5;
	g_stLocalDLPara.aafDsgRate1[2][4] = 0.5;
	g_stLocalDLPara.aafDsgRate1[2][5] = 0.5;
	g_stLocalDLPara.aafDsgRate1[3][0] = 0.5;
	g_stLocalDLPara.aafDsgRate1[3][1] = 0.5;
	g_stLocalDLPara.aafDsgRate1[3][2] = 0.5;
	g_stLocalDLPara.aafDsgRate1[3][3] = 0.5;
	g_stLocalDLPara.aafDsgRate1[3][4] = 0.5;
	g_stLocalDLPara.aafDsgRate1[3][5] = 0.5;
	g_stLocalDLPara.aafDsgRate1[4][0] = 2;
	g_stLocalDLPara.aafDsgRate1[4][1] = 2;
	g_stLocalDLPara.aafDsgRate1[4][2] = 2;
	g_stLocalDLPara.aafDsgRate1[4][3] = 2;
	g_stLocalDLPara.aafDsgRate1[4][4] = 2;
	g_stLocalDLPara.aafDsgRate1[4][5] = 2;
	g_stLocalDLPara.aafDsgRate1[5][0] = 2;
	g_stLocalDLPara.aafDsgRate1[5][1] = 2;
	g_stLocalDLPara.aafDsgRate1[5][2] = 2;
	g_stLocalDLPara.aafDsgRate1[5][3] = 2;
	g_stLocalDLPara.aafDsgRate1[5][4] = 2;
	g_stLocalDLPara.aafDsgRate1[5][5] = 2;
	g_stLocalDLPara.aafDsgRate1[6][0] = 2;
	g_stLocalDLPara.aafDsgRate1[6][1] = 2;
	g_stLocalDLPara.aafDsgRate1[6][2] = 2;
	g_stLocalDLPara.aafDsgRate1[6][3] = 2;
	g_stLocalDLPara.aafDsgRate1[6][4] = 2;
	g_stLocalDLPara.aafDsgRate1[6][5] = 2;
	g_stLocalDLPara.aafDsgRate2[1][0] = 0.25;
	g_stLocalDLPara.aafDsgRate2[1][1] = 0.25;
	g_stLocalDLPara.aafDsgRate2[1][2] = 0.25;
	g_stLocalDLPara.aafDsgRate2[1][3] = 0.25;
	g_stLocalDLPara.aafDsgRate2[1][4] = 0.12;
	g_stLocalDLPara.aafDsgRate2[1][5] = 0.06;
	g_stLocalDLPara.aafDsgRate2[1][6] = 0;
	g_stLocalDLPara.aafDsgRate2[1][7] = 0;
	g_stLocalDLPara.aafDsgRate2[1][8] = 0;
	g_stLocalDLPara.aafDsgRate2[2][0] = 0.5;
	g_stLocalDLPara.aafDsgRate2[2][1] = 0.5;
	g_stLocalDLPara.aafDsgRate2[2][2] = 0.5;
	g_stLocalDLPara.aafDsgRate2[2][3] = 0.5;
	g_stLocalDLPara.aafDsgRate2[2][4] = 0.25;
	g_stLocalDLPara.aafDsgRate2[2][5] = 0.12;
	g_stLocalDLPara.aafDsgRate2[2][6] = 0.5;
	g_stLocalDLPara.aafDsgRate2[2][7] = 0.5;
	g_stLocalDLPara.aafDsgRate2[2][8] = 0.25;
	g_stLocalDLPara.aafDsgRate2[3][0] = 0.5;
	g_stLocalDLPara.aafDsgRate2[3][1] = 0.5;
	g_stLocalDLPara.aafDsgRate2[3][2] = 0.5;
	g_stLocalDLPara.aafDsgRate2[3][3] = 0.5;
	g_stLocalDLPara.aafDsgRate2[3][4] = 0.5;
	g_stLocalDLPara.aafDsgRate2[3][5] = 0.3;
	g_stLocalDLPara.aafDsgRate2[3][6] = 0.5;
	g_stLocalDLPara.aafDsgRate2[3][7] = 0.5;
	g_stLocalDLPara.aafDsgRate2[3][8] = 0.25;
	g_stLocalDLPara.aafDsgRate2[4][0] = 2;
	g_stLocalDLPara.aafDsgRate2[4][1] = 2;
	g_stLocalDLPara.aafDsgRate2[4][2] = 2;
	g_stLocalDLPara.aafDsgRate2[4][3] = 1;
	g_stLocalDLPara.aafDsgRate2[4][4] = 1;
	g_stLocalDLPara.aafDsgRate2[4][5] = 0.75;
	g_stLocalDLPara.aafDsgRate2[4][6] = 0.5;
	g_stLocalDLPara.aafDsgRate2[4][7] = 0.5;
	g_stLocalDLPara.aafDsgRate2[4][8] = 0.25;
	g_stLocalDLPara.aafDsgRate2[5][0] = 2;
	g_stLocalDLPara.aafDsgRate2[5][1] = 2;
	g_stLocalDLPara.aafDsgRate2[5][2] = 2;
	g_stLocalDLPara.aafDsgRate2[5][3] = 1;
	g_stLocalDLPara.aafDsgRate2[5][4] = 1;
	g_stLocalDLPara.aafDsgRate2[5][5] = 0.75;
	g_stLocalDLPara.aafDsgRate2[5][6] = 0.5;
	g_stLocalDLPara.aafDsgRate2[5][7] = 0.5;
	g_stLocalDLPara.aafDsgRate2[5][8] = 0.25;
	g_stLocalDLPara.aafDsgRate2[6][0] = 2;
	g_stLocalDLPara.aafDsgRate2[6][1] = 2;
	g_stLocalDLPara.aafDsgRate2[6][2] = 2;
	g_stLocalDLPara.aafDsgRate2[6][3] = 1;
	g_stLocalDLPara.aafDsgRate2[6][4] = 1;
	g_stLocalDLPara.aafDsgRate2[6][5] = 0.75;
	g_stLocalDLPara.aafDsgRate2[6][6] = 0.5;
	g_stLocalDLPara.aafDsgRate2[6][7] = 0.5;
	g_stLocalDLPara.aafDsgRate2[6][8] = 0.25;
	for(uint8_t i = 0; i < 5; i++) {
		g_stLocalDLPara.aafDsgRate3[i][0] = 2;
		g_stLocalDLPara.aafDsgRate3[i][1] = 2;
		g_stLocalDLPara.aafDsgRate3[i][2] = 2;
		g_stLocalDLPara.aafDsgRate3[i][3] = 2;
		g_stLocalDLPara.aafDsgRate3[i][4] = 2;
		g_stLocalDLPara.aafDsgRate3[i][5] = 2;
	}
	g_stLocalDLPara.aafDsgRate3[5][0] = 1;
	g_stLocalDLPara.aafDsgRate3[5][1] = 1;
	g_stLocalDLPara.aafDsgRate3[5][2] = 1;
	g_stLocalDLPara.aafDsgRate3[5][3] = 1;
	g_stLocalDLPara.aafDsgRate3[5][4] = 1;
	g_stLocalDLPara.aafDsgRate3[5][5] = 1;
	for(uint8_t i = 0; i < 5; i++) {
		g_stLocalDLPara.aafDsgRate4[i][0] = 2;
		g_stLocalDLPara.aafDsgRate4[i][1] = 2;
		g_stLocalDLPara.aafDsgRate4[i][2] = 2;
		g_stLocalDLPara.aafDsgRate4[i][3] = 1;
		g_stLocalDLPara.aafDsgRate4[i][4] = 1;
		g_stLocalDLPara.aafDsgRate4[i][5] = 0.75;
		g_stLocalDLPara.aafDsgRate4[i][6] = 0.5;
		g_stLocalDLPara.aafDsgRate4[i][7] = 0.5;
		g_stLocalDLPara.aafDsgRate4[i][8] = 0.25;
	}
	g_stLocalDLPara.aafDsgRate4[5][0] = 1;
	g_stLocalDLPara.aafDsgRate4[5][1] = 1;
	g_stLocalDLPara.aafDsgRate4[5][2] = 1;
	g_stLocalDLPara.aafDsgRate4[5][3] = 0.75;
	g_stLocalDLPara.aafDsgRate4[5][4] = 0.75;
	g_stLocalDLPara.aafDsgRate4[5][5] = 0.5;
	g_stLocalDLPara.aafDsgRate4[5][6] = 0.25;
	g_stLocalDLPara.aafDsgRate4[5][7] = 0.25;
	g_stLocalDLPara.aafDsgRate4[5][8] = 0.25;

	/* PEAK discharge current limit init */ 
	g_stLocalPDCLPara.afTempMin[0] = -35;
	g_stLocalPDCLPara.afTempMin[1] = -30;
	g_stLocalPDCLPara.afTempMin[2] = -20;
	g_stLocalPDCLPara.afTempMin[3] = -10;
	g_stLocalPDCLPara.afTempMin[4] = 0;
	g_stLocalPDCLPara.afTempMin[5] = 5;
	g_stLocalPDCLPara.afTempMin[6] = 10;
	g_stLocalPDCLPara.afTempMax[0] = 20;
	g_stLocalPDCLPara.afTempMax[1] = 40;
	g_stLocalPDCLPara.afTempMax[2] = 55;
	g_stLocalPDCLPara.afTempMax[3] = 60;
	g_stLocalPDCLPara.afTempMax[4] = 63;
	g_stLocalPDCLPara.afCellVMax[0] = 3500;
	g_stLocalPDCLPara.afCellVMax[1] = 3450;
	g_stLocalPDCLPara.afCellVMax[2] = 3400;
	g_stLocalPDCLPara.afCellVMax[3] = 3350;
	g_stLocalPDCLPara.afCellVMax[4] = 3300;
	g_stLocalPDCLPara.afCellVMax[5] = 3100;
	g_stLocalPDCLPara.afCellVMin[0] = 2900;
	g_stLocalPDCLPara.afCellVMin[1] = 2850;
	g_stLocalPDCLPara.afCellVMin[2] = 2800;
	g_stLocalPDCLPara.afCellVMin[3] = 2750;
	g_stLocalPDCLPara.afCellVMin[4] = 2700;
	g_stLocalPDCLPara.afCellVMin[5] = 2650;
	g_stLocalPDCLPara.afCellVMin[6] = 2600;
	g_stLocalPDCLPara.afCellVMin[7] = 2550;
	for(uint8_t i = 0; i < 6; i++) {
		g_stLocalPDCLPara.aafDsgRate1[0][i] = 0;
		g_stLocalPDCLPara.aafDsgRate1[1][i] = 0.75;
		g_stLocalPDCLPara.aafDsgRate1[2][i] = 1;
		g_stLocalPDCLPara.aafDsgRate1[3][i] = 1.75;
		g_stLocalPDCLPara.aafDsgRate1[4][i] = 2.5;
		g_stLocalPDCLPara.aafDsgRate1[5][i] = 4;
		g_stLocalPDCLPara.aafDsgRate1[6][i] = 4;
	}
	g_stLocalPDCLPara.aafDsgRate2[0][0] = 0;
	g_stLocalPDCLPara.aafDsgRate2[0][1] = 0;
	g_stLocalPDCLPara.aafDsgRate2[0][2] = 0;
	g_stLocalPDCLPara.aafDsgRate2[0][3] = 0;
	g_stLocalPDCLPara.aafDsgRate2[0][4] = 0;
	g_stLocalPDCLPara.aafDsgRate2[0][5] = 0;
	g_stLocalPDCLPara.aafDsgRate2[0][6] = 0;
	g_stLocalPDCLPara.aafDsgRate2[0][7] = 0;
	g_stLocalPDCLPara.aafDsgRate2[0][8] = 0;
	g_stLocalPDCLPara.aafDsgRate2[1][0] = 0.75;
	g_stLocalPDCLPara.aafDsgRate2[1][1] = 0.75;
	g_stLocalPDCLPara.aafDsgRate2[1][2] = 0.75;
	g_stLocalPDCLPara.aafDsgRate2[1][3] = 0.75;
	g_stLocalPDCLPara.aafDsgRate2[1][4] = 0.75;
	g_stLocalPDCLPara.aafDsgRate2[1][5] = 0.5;
	g_stLocalPDCLPara.aafDsgRate2[1][6] = 0.5;
	g_stLocalPDCLPara.aafDsgRate2[1][7] = 0.5;
	g_stLocalPDCLPara.aafDsgRate2[1][8] = 0;
	g_stLocalPDCLPara.aafDsgRate2[2][0] = 1;
	g_stLocalPDCLPara.aafDsgRate2[2][1] = 1;
	g_stLocalPDCLPara.aafDsgRate2[2][2] = 1;
	g_stLocalPDCLPara.aafDsgRate2[2][3] = 1;
	g_stLocalPDCLPara.aafDsgRate2[2][4] = 0.75;
	g_stLocalPDCLPara.aafDsgRate2[2][5] = 0.5;
	g_stLocalPDCLPara.aafDsgRate2[2][6] = 0.5;
	g_stLocalPDCLPara.aafDsgRate2[2][7] = 0.5;
	g_stLocalPDCLPara.aafDsgRate2[2][8] = 0;
	g_stLocalPDCLPara.aafDsgRate2[3][0] = 1.75;
	g_stLocalPDCLPara.aafDsgRate2[3][1] = 1.75;
	g_stLocalPDCLPara.aafDsgRate2[3][2] = 1.75;
	g_stLocalPDCLPara.aafDsgRate2[3][3] = 1.75;
	g_stLocalPDCLPara.aafDsgRate2[3][4] = 1.25;
	g_stLocalPDCLPara.aafDsgRate2[3][5] = 1.25;
	g_stLocalPDCLPara.aafDsgRate2[3][6] = 1.25;
	g_stLocalPDCLPara.aafDsgRate2[3][7] = 0.75;
	g_stLocalPDCLPara.aafDsgRate2[3][8] = 0.75;
	g_stLocalPDCLPara.aafDsgRate2[4][0] = 2.5;
	g_stLocalPDCLPara.aafDsgRate2[4][1] = 2.5;
	g_stLocalPDCLPara.aafDsgRate2[4][2] = 2.5;
	g_stLocalPDCLPara.aafDsgRate2[4][3] = 2.75;
	g_stLocalPDCLPara.aafDsgRate2[4][4] = 2.5;
	g_stLocalPDCLPara.aafDsgRate2[4][5] = 2;
	g_stLocalPDCLPara.aafDsgRate2[4][6] = 1.75;
	g_stLocalPDCLPara.aafDsgRate2[4][7] = 1.5;
	g_stLocalPDCLPara.aafDsgRate2[4][8] = 1;
	g_stLocalPDCLPara.aafDsgRate2[5][0] = 4;
	g_stLocalPDCLPara.aafDsgRate2[5][1] = 4;
	g_stLocalPDCLPara.aafDsgRate2[5][2] = 3;
	g_stLocalPDCLPara.aafDsgRate2[5][3] = 2.75;
	g_stLocalPDCLPara.aafDsgRate2[5][4] = 2.5;
	g_stLocalPDCLPara.aafDsgRate2[5][5] = 2;
	g_stLocalPDCLPara.aafDsgRate2[5][6] = 1.75;
	g_stLocalPDCLPara.aafDsgRate2[5][7] = 1.5;
	g_stLocalPDCLPara.aafDsgRate2[5][8] = 1;
	g_stLocalPDCLPara.aafDsgRate2[6][0] = 4;
	g_stLocalPDCLPara.aafDsgRate2[6][1] = 4;
	g_stLocalPDCLPara.aafDsgRate2[6][2] = 3;
	g_stLocalPDCLPara.aafDsgRate2[6][3] = 2.75;
	g_stLocalPDCLPara.aafDsgRate2[6][4] = 2.5;
	g_stLocalPDCLPara.aafDsgRate2[6][5] = 2;
	g_stLocalPDCLPara.aafDsgRate2[6][6] = 1.75;
	g_stLocalPDCLPara.aafDsgRate2[6][7] = 1.5;
	g_stLocalPDCLPara.aafDsgRate2[6][8] = 0.75;
	for(uint8_t i = 0; i < 6; i++) {
		g_stLocalPDCLPara.aafDsgRate3[0][i] = 4;
		g_stLocalPDCLPara.aafDsgRate3[1][i] = 4;
		g_stLocalPDCLPara.aafDsgRate3[2][i] = 4;
		g_stLocalPDCLPara.aafDsgRate3[3][i] = 3.5;
		g_stLocalPDCLPara.aafDsgRate3[4][i] = 2.5;
		g_stLocalPDCLPara.aafDsgRate3[5][i] = 1.5;
	}
	for(uint8_t i = 0; i < 3; i++) {
		g_stLocalPDCLPara.aafDsgRate4[i][0] = 4;
		g_stLocalPDCLPara.aafDsgRate4[i][1] = 4;
		g_stLocalPDCLPara.aafDsgRate4[i][2] = 3;
		g_stLocalPDCLPara.aafDsgRate4[i][3] = 2.75;
		g_stLocalPDCLPara.aafDsgRate4[i][4] = 2.5;
		g_stLocalPDCLPara.aafDsgRate4[i][5] = 2;
		g_stLocalPDCLPara.aafDsgRate4[i][6] = 1.75;
		g_stLocalPDCLPara.aafDsgRate4[i][7] = 1.5;
		g_stLocalPDCLPara.aafDsgRate4[i][8] = 0.5;
	}
	g_stLocalPDCLPara.aafDsgRate4[3][0] = 3.5;
	g_stLocalPDCLPara.aafDsgRate4[3][1] = 3.5;
	g_stLocalPDCLPara.aafDsgRate4[3][2] = 3;
	g_stLocalPDCLPara.aafDsgRate4[3][3] = 2.75;
	g_stLocalPDCLPara.aafDsgRate4[3][4] = 2.5;
	g_stLocalPDCLPara.aafDsgRate4[3][5] = 2;
	g_stLocalPDCLPara.aafDsgRate4[3][6] = 1.75;
	g_stLocalPDCLPara.aafDsgRate4[3][7] = 1.5;
	g_stLocalPDCLPara.aafDsgRate4[3][8] = 0.5;
	g_stLocalPDCLPara.aafDsgRate4[4][0] = 2.5;
	g_stLocalPDCLPara.aafDsgRate4[4][1] = 2.5;
	g_stLocalPDCLPara.aafDsgRate4[4][2] = 2.5;
	g_stLocalPDCLPara.aafDsgRate4[4][3] = 2.5;
	g_stLocalPDCLPara.aafDsgRate4[4][4] = 1.5;
	g_stLocalPDCLPara.aafDsgRate4[4][5] = 1;
	g_stLocalPDCLPara.aafDsgRate4[4][6] = 0.75;
	g_stLocalPDCLPara.aafDsgRate4[4][7] = 0.5;
	g_stLocalPDCLPara.aafDsgRate4[4][8] = 0.5;
	g_stLocalPDCLPara.aafDsgRate4[5][0] = 1.5;
	g_stLocalPDCLPara.aafDsgRate4[5][1] = 1.5;
	g_stLocalPDCLPara.aafDsgRate4[5][2] = 1.5;
	g_stLocalPDCLPara.aafDsgRate4[5][3] = 1.5;
	g_stLocalPDCLPara.aafDsgRate4[5][4] = 1.5;
	g_stLocalPDCLPara.aafDsgRate4[5][5] = 1;
	g_stLocalPDCLPara.aafDsgRate4[5][6] = 0.75;
	g_stLocalPDCLPara.aafDsgRate4[5][7] = 0.5;
	g_stLocalPDCLPara.aafDsgRate4[5][8] = 0.5;

	/* PEAK discharge limit (based upon MOS temperature) init */
	g_stLocalPDCLMOSPara.afTemp[0] = -35;
	g_stLocalPDCLMOSPara.afTemp[1] = -10;
	g_stLocalPDCLMOSPara.afTemp[2] = 0;
	g_stLocalPDCLMOSPara.afTemp[3] = 10;
	g_stLocalPDCLMOSPara.afTemp[4] = 25;
	g_stLocalPDCLMOSPara.afTemp[5] = 50;
	g_stLocalPDCLMOSPara.afTemp[6] = 70;
	g_stLocalPDCLMOSPara.afTemp[7] = 80;
	g_stLocalPDCLMOSPara.afTemp[8] = 90;
	g_stLocalPDCLMOSPara.afTemp[9] = 100;
	g_stLocalPDCLMOSPara.afRate[0] = 1;
	g_stLocalPDCLMOSPara.afRate[1] = 1;
	g_stLocalPDCLMOSPara.afRate[2] = 1;
	g_stLocalPDCLMOSPara.afRate[3] = 1;
	g_stLocalPDCLMOSPara.afRate[4] = 1;
	g_stLocalPDCLMOSPara.afRate[5] = 1;
	g_stLocalPDCLMOSPara.afRate[6] = 0.75;
	g_stLocalPDCLMOSPara.afRate[7] = 0.5;
	g_stLocalPDCLMOSPara.afRate[8] = 0.25;
	g_stLocalPDCLMOSPara.afRate[9] = 0;
	/* Standby soc calibration */
	g_stLocalSocCali.afTemp[0] = -10;
	g_stLocalSocCali.afTemp[1] = 0;
	g_stLocalSocCali.afTemp[2] = 10;
	g_stLocalSocCali.afTemp[3] = 25;
	g_stLocalSocCali.afTemp[4] = 45;
	g_stLocalSocCali.afSoc[0] = 100;
	g_stLocalSocCali.afSoc[1] = 95;
	g_stLocalSocCali.afSoc[2] = 90;
	g_stLocalSocCali.afSoc[3] = 85;
	g_stLocalSocCali.afSoc[4] = 80;
	g_stLocalSocCali.afSoc[5] = 75;
	g_stLocalSocCali.afSoc[6] = 70;
	g_stLocalSocCali.afSoc[7] = 65;
	g_stLocalSocCali.afSoc[8] = 60;
	g_stLocalSocCali.afSoc[9] = 55;
	g_stLocalSocCali.afSoc[10] = 50;
	g_stLocalSocCali.afSoc[11] = 45;
	g_stLocalSocCali.afSoc[12] = 40;
	g_stLocalSocCali.afSoc[13] = 35;
	g_stLocalSocCali.afSoc[14] = 30;
	g_stLocalSocCali.afSoc[15] = 25;
	g_stLocalSocCali.afSoc[16] = 20;
	g_stLocalSocCali.afSoc[17] = 15;
	g_stLocalSocCali.afSoc[18] = 10;
	g_stLocalSocCali.afSoc[19] = 5;
	g_stLocalSocCali.afSoc[20] = 0;
	g_stLocalSocCali.aafCellU[0][0] = 3.35;
	g_stLocalSocCali.aafCellU[0][1] = 3.3652;
	g_stLocalSocCali.aafCellU[0][2] = 3.375;
	g_stLocalSocCali.aafCellU[0][3] = 3.37;
	g_stLocalSocCali.aafCellU[0][4] = 3.365;
	g_stLocalSocCali.aafCellU[1][0] = 3.2951;
	g_stLocalSocCali.aafCellU[1][1] = 3.311;
	g_stLocalSocCali.aafCellU[1][2] = 3.315;
	g_stLocalSocCali.aafCellU[1][3] = 3.3175;
	g_stLocalSocCali.aafCellU[1][4] = 3.3225;
	g_stLocalSocCali.aafCellU[2][0] = 3.2915;
	g_stLocalSocCali.aafCellU[2][1] = 3.3074;
	g_stLocalSocCali.aafCellU[2][2] = 3.3125;
	g_stLocalSocCali.aafCellU[2][3] = 3.315;
	g_stLocalSocCali.aafCellU[2][4] = 3.32;
	g_stLocalSocCali.aafCellU[3][0] = 3.2878;
	g_stLocalSocCali.aafCellU[3][1] = 3.3038;
	g_stLocalSocCali.aafCellU[3][2] = 3.31;
	g_stLocalSocCali.aafCellU[3][3] = 3.3125;
	g_stLocalSocCali.aafCellU[3][4] = 3.3185;
	g_stLocalSocCali.aafCellU[4][0] = 3.2842;
	g_stLocalSocCali.aafCellU[4][1] = 3.3002;
	g_stLocalSocCali.aafCellU[4][2] = 3.3079;
	g_stLocalSocCali.aafCellU[4][3] = 3.31;
	g_stLocalSocCali.aafCellU[4][4] = 3.316;
	g_stLocalSocCali.aafCellU[5][0] = 3.2806;
	g_stLocalSocCali.aafCellU[5][1] = 3.2965;
	g_stLocalSocCali.aafCellU[5][2] = 3.304;
	g_stLocalSocCali.aafCellU[5][3] = 3.3085;
	g_stLocalSocCali.aafCellU[5][4] = 3.3123;
	g_stLocalSocCali.aafCellU[6][0] = 3.277;
	g_stLocalSocCali.aafCellU[6][1] = 3.2929;
	g_stLocalSocCali.aafCellU[6][2] = 3.3002;
	g_stLocalSocCali.aafCellU[6][3] = 3.3045;
	g_stLocalSocCali.aafCellU[6][4] = 3.3085;
	g_stLocalSocCali.aafCellU[7][0] = 3.2734;
	g_stLocalSocCali.aafCellU[7][1] = 3.2893;
	g_stLocalSocCali.aafCellU[7][2] = 3.2963;
	g_stLocalSocCali.aafCellU[7][3] = 3.3005;
	g_stLocalSocCali.aafCellU[7][4] = 3.3047;
	g_stLocalSocCali.aafCellU[8][0] = 3.2661;
	g_stLocalSocCali.aafCellU[8][1] = 3.282;
	g_stLocalSocCali.aafCellU[8][2] = 3.2888;
	g_stLocalSocCali.aafCellU[8][3] = 3.2929;
	g_stLocalSocCali.aafCellU[8][4] = 3.2973;
	g_stLocalSocCali.aafCellU[9][0] = 3.2576;
	g_stLocalSocCali.aafCellU[9][1] = 3.2735;
	g_stLocalSocCali.aafCellU[9][2] = 3.2801;
	g_stLocalSocCali.aafCellU[9][3] = 3.284;
	g_stLocalSocCali.aafCellU[9][4] = 3.2886;
	g_stLocalSocCali.aafCellU[10][0] = 3.2491;
	g_stLocalSocCali.aafCellU[10][1] = 3.2651;
	g_stLocalSocCali.aafCellU[10][2] = 3.2725;
	g_stLocalSocCali.aafCellU[10][3] = 3.2771;
	g_stLocalSocCali.aafCellU[10][4] = 3.2826;
	g_stLocalSocCali.aafCellU[11][0] = 3.2397;
	g_stLocalSocCali.aafCellU[11][1] = 3.2556;
	g_stLocalSocCali.aafCellU[11][2] = 3.2617;
	g_stLocalSocCali.aafCellU[11][3] = 3.2673;
	g_stLocalSocCali.aafCellU[11][4] = 3.2719;
	g_stLocalSocCali.aafCellU[12][0] = 3.2319;
	g_stLocalSocCali.aafCellU[12][1] = 3.2478;
	g_stLocalSocCali.aafCellU[12][2] = 3.2539;
	g_stLocalSocCali.aafCellU[12][3] = 3.2585;
	g_stLocalSocCali.aafCellU[12][4] = 3.2643;
	g_stLocalSocCali.aafCellU[13][0] = 3.225;
	g_stLocalSocCali.aafCellU[13][1] = 3.2409;
	g_stLocalSocCali.aafCellU[13][2] = 3.2484;
	g_stLocalSocCali.aafCellU[13][3] = 3.2517;
	g_stLocalSocCali.aafCellU[13][4] = 3.2562;
	g_stLocalSocCali.aafCellU[14][0] = 3.2177;
	g_stLocalSocCali.aafCellU[14][1] = 3.2336;
	g_stLocalSocCali.aafCellU[14][2] = 3.241;
	g_stLocalSocCali.aafCellU[14][3] = 3.2403;
	g_stLocalSocCali.aafCellU[14][4] = 3.2424;
	g_stLocalSocCali.aafCellU[15][0] = 3.2118;
	g_stLocalSocCali.aafCellU[15][1] = 3.2277;
	g_stLocalSocCali.aafCellU[15][2] = 3.2312;
	g_stLocalSocCali.aafCellU[15][3] = 3.2268;
	g_stLocalSocCali.aafCellU[15][4] = 3.2276;
	g_stLocalSocCali.aafCellU[16][0] = 3.1986;
	g_stLocalSocCali.aafCellU[16][1] = 3.2145;
	g_stLocalSocCali.aafCellU[16][2] = 3.2144;
	g_stLocalSocCali.aafCellU[16][3] = 3.2106;
	g_stLocalSocCali.aafCellU[16][4] = 3.2078;
	g_stLocalSocCali.aafCellU[17][0] = 3.1774;
	g_stLocalSocCali.aafCellU[17][1] = 3.1933;
	g_stLocalSocCali.aafCellU[17][2] = 3.1894;
	g_stLocalSocCali.aafCellU[17][3] = 3.1843;
	g_stLocalSocCali.aafCellU[17][4] = 3.1843;
	g_stLocalSocCali.aafCellU[18][0] = 3.1512;
	g_stLocalSocCali.aafCellU[18][1] = 3.1671;
	g_stLocalSocCali.aafCellU[18][2] = 3.1635;
	g_stLocalSocCali.aafCellU[18][3] = 3.1602;
	g_stLocalSocCali.aafCellU[18][4] = 3.1555;
	g_stLocalSocCali.aafCellU[19][0] = 3.1084;
	g_stLocalSocCali.aafCellU[19][1] = 3.1243;
	g_stLocalSocCali.aafCellU[19][2] = 3.1236;
	g_stLocalSocCali.aafCellU[19][3] = 3.1258;
	g_stLocalSocCali.aafCellU[19][4] = 3.1132;
	g_stLocalSocCali.aafCellU[20][0] = 2.8505;
	g_stLocalSocCali.aafCellU[20][1] = 2.8508;
	g_stLocalSocCali.aafCellU[20][2] = 2.7955;
	g_stLocalSocCali.aafCellU[20][3] = 2.7038;
	g_stLocalSocCali.aafCellU[20][4] = 2.6914;
	LOCAL_RETURN_TRUE;
}

void local_soc_cali_save(void) {
	LOCAL_PACK_RVAL_S* pstPackRVal = g_stLocalArrayRVal.astPackRVal + g_stPrl.ucSelfId;
	eco_refresh_RTV();
	eco_refresh_CD();
	eco_refresh_log();
	g_stEcoRtv.bSocCali = 1;
	g_stEcoLog.bSocCali = 1;
	g_eBaseStat = eBStatSOCRecalc;
	his_data_write();
	his_log_write();
	g_fPackSoc = pstPackRVal->fPackSoc;
	g_stEcoRtv.bSocCali = 0;
	g_stEcoLog.bSocCali = 0;
}

void local_standby_leftAH_cali(void) {
	LOCAL_PACK_RVAL_S* pstPackRVal = g_stLocalArrayRVal.astPackRVal + g_stPrl.ucSelfId;
	if(pstPackRVal->uErrCode.ucErrCode != 0) {
		return;
	}
	bool bSave = false;
		uint8_t ucCol = 0;
		for(int j=0;j<5;j++) {
			if(pstPackRVal->fCellTMin >= g_stLocalSocCali.afTemp[j]) {
				ucCol = j;
			}
		}
		uint8_t ucRow = 0;
		float usCellVoltAvg = 0;
		for(uint8_t i=0;i<CFG_CELL_NUM;i++) {
			usCellVoltAvg += (pstPackRVal->afCellU[i]);
		}
		usCellVoltAvg /= CFG_CELL_NUM;
		for(int j=0;j<20;j++) {
			if(usCellVoltAvg <= g_stLocalSocCali.aafCellU[j][ucCol] && usCellVoltAvg > g_stLocalSocCali.aafCellU[j + 1][ucCol]) {
				ucRow = j;
				break;
			}
			if(usCellVoltAvg < g_stLocalSocCali.aafCellU[20][ucCol]) {
				ucRow = 20;
				break;
			}
			if(usCellVoltAvg >= g_stLocalSocCali.aafCellU[0][ucCol]) {
				ucRow = 0;
				break;
			}
		}
		float fSocLeft = (g_stLocalSocCali.afSoc[ucRow] - g_stLocalSocCali.afSoc[ucRow + 1])
			* (usCellVoltAvg - g_stLocalSocCali.aafCellU[ucRow + 1][ucCol])
			/ (g_stLocalSocCali.aafCellU[ucRow][ucCol] - g_stLocalSocCali.aafCellU[ucRow + 1][ucCol])
			+ g_stLocalSocCali.afSoc[ucRow + 1];
		float fSocRight = (g_stLocalSocCali.afSoc[ucRow] - g_stLocalSocCali.afSoc[ucRow + 1])
			* (usCellVoltAvg - g_stLocalSocCali.aafCellU[ucRow + 1][ucCol + 1])
			/ (g_stLocalSocCali.aafCellU[ucRow][ucCol + 1] - g_stLocalSocCali.aafCellU[ucRow + 1][ucCol + 1])
			+ g_stLocalSocCali.afSoc[ucRow + 1];
		float fSoc = (fSocRight - fSocLeft) * (pstPackRVal->fCellTMin - g_stLocalSocCali.afTemp[ucCol])
			/ (g_stLocalSocCali.afTemp[ucCol + 1] - g_stLocalSocCali.afTemp[ucCol]) + fSocLeft + 0.5; //2024.5.10
		if(((pstPackRVal->fPackSoc > fSoc && pstPackRVal->fPackSoc == 100 && fSoc < 95) || (pstPackRVal->fPackSoc > fSoc && pstPackRVal->fPackSoc < 100)) 
				|| (fSoc - pstPackRVal->fPackSoc  > 50 && pstPackRVal->fCellUMin > 3.05)) {
				if(fSoc > pstPackRVal->fPackSoc) {
					if(fSoc - pstPackRVal->fPackSoc  < 50) {
						return;
					}
				}
//				if(fSoc - pstPackRVal->fPackSoc  < 50 && fSoc > pstPackRVal->fPackSoc) {
//					return;
//				}
				pstPackRVal->fPackSoc = fSoc;
				for(int i=0;i<CFG_CELL_NUM;i++) {
					pstPackRVal->afCellLeftAH[i] = round(pstPackRVal->afCellRealAH[i] * fSoc / 100);
				}
				bSave = true;
		}
	if(bSave) {
		/* Pack dischargeable capacity update */
		float fPackLeftAH = pstPackRVal->afCellLeftAH[0];
		for(uint8_t i=1;i<CFG_CELL_NUM;i++) {
			if(pstPackRVal->afCellLeftAH[i] > g_stCfg.stLocal.usDesignAH){
				pstPackRVal->afCellLeftAH[i] = g_stCfg.stLocal.usDesignAH;
			}
			if(fPackLeftAH > pstPackRVal->afCellLeftAH[i]) {
				fPackLeftAH = pstPackRVal->afCellLeftAH[i];
			}
		}
		pstPackRVal->fPackLeftAH = fPackLeftAH;
		if(pstPackRVal->fPackLeftAH < 0){
			pstPackRVal->fPackLeftAH = 0;
		}
		if(pstPackRVal->fPackLeftAH > g_stCfg.stLocal.usDesignAH){
			pstPackRVal->fPackLeftAH = g_stCfg.stLocal.usDesignAH;
		}
		/* PACK Soc Update */
		pstPackRVal->fPackSoc = pstPackRVal->fPackLeftAH * 100 / pstPackRVal->fPackRealAH;
		
		if(round(pstPackRVal->fPackSoc) != round(g_fPackSoc)) {
			local_soc_cali_save();
		}
	}
}

/*******************************************************************************
* Function Name  : local_info_refresh
* Description    : Collect data and calculate
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void local_info_refresh(void) {
	ERR_CODE_U uErrCode = {0};
	LOCAL_PACK_RVAL_S* pstPackRVal = g_stLocalArrayRVal.astPackRVal + g_stPrl.ucSelfId;
	/* Cell voltage update */
	for(uint8_t i=0;i<CFG_CELL_NUM;i++) {
		pstPackRVal->afCellU[i] = g_stAfe.stRamApp.fCELLVol[i] * 0.001 + g_stCfg.stLocal.sVoltSnrCoeB;
	}
	/* Voltage sensor status update */
	for(uint8_t i=0;i<CFG_CELL_NUM;i++) {
		if(pstPackRVal->afCellU[i] > 5 || pstPackRVal->afCellU[i] < 1) {
			uErrCode.stErrCode.bVoltSensor = 1;
		}
	}
	/* PACK voltage updated */
	float fPackU = 0;
	for(uint8_t i=0;i<CFG_CELL_NUM;i++) {
		fPackU += pstPackRVal->afCellU[i];
	}
	pstPackRVal->fPackU = fPackU;
	/* The maximum and minimum cell voltage is updated */
	uint8_t ucCellUMaxId = 0;
	uint8_t ucCellUMinId = 0;
	for(uint8_t i=0;i<CFG_CELL_NUM;i++) {
		if(pstPackRVal->afCellU[ucCellUMaxId] < pstPackRVal->afCellU[i]) {
			ucCellUMaxId = i;
		}
		if(pstPackRVal->afCellU[ucCellUMinId] > pstPackRVal->afCellU[i]) {
			ucCellUMinId = i;
		}
	}
	pstPackRVal->ucCellUMaxId = ucCellUMaxId;
	pstPackRVal->ucCellUMinId = ucCellUMinId;
	pstPackRVal->fCellUMax = pstPackRVal->afCellU[ucCellUMaxId];
	pstPackRVal->fCellUMin = pstPackRVal->afCellU[ucCellUMinId];
	/* Turn on Equalization */
	if(g_stCfg.stLocal.sBalanceOn != -1
		&& ((pstPackRVal->fCellUMax - pstPackRVal->fCellUMin) * 1000 > g_stCfg.stLocal.usBalanceDiff)
		&& (pstPackRVal->fCellUMax * 1000) > g_stCfg.stLocal.usBalanceValue
		&& (pstPackRVal->uBaseStat.stBaseStat.ucChg || g_eBaseStat == eBStatStewing)) {
		static uint16_t s_uiBal_Tick = 0;
		static uint16_t s_uiBal_Tick1 = 0;
		if(s_uiBal_Tick * g_stCfg.stLocal.usCyclePeriod > 2000){
			if(s_uiBal_Tick1 * g_stCfg.stLocal.usCyclePeriod <= 8000) {
				s_uiBal_Tick1++;
				g_BalanceFlag = 1;
				g_BalanceFlag_2 = 1;
				Bat_Balance(ucCellUMaxId);
			} else {
				s_uiBal_Tick = 0;
				s_uiBal_Tick1 = 0;
				g_BalanceFlag = 0;
			}
    } else {
			s_uiBal_Tick ++;
		}
	} else{
		g_BalanceFlag = 0;
		g_BalanceFlag_2 = 0;
	}
	/* Current updates */
	if(g_stAfe.stRamApp.fCDATA <= 1000/*LOCAL_CUR_DZ*/ && g_stAfe.stRamApp.fCDATA >= -1000/*LOCAL_CUR_DZ*/) {
		pstPackRVal->fPackCur = 0;
		if(g_eBaseStat == eBStatWorking){
			static uint16_t s_TickEnd = 0;
			if(s_TickEnd * g_stCfg.stLocal.usCyclePeriod > 3000) {
				g_eBaseStat = eBStatWorkEnd; 
				his_data_write();
				s_TickEnd = 0;
			} else {
				s_TickEnd++;
			}
		} else {
			static uint16_t s_TickStewing = 0;
			if(g_eBaseStat == eBStatHeating) {
				s_TickStewing = 0;
			}
			if(s_TickStewing * g_stCfg.stLocal.usCyclePeriod > 3000) {
				g_eBaseStat = eBStatStewing;		
				s_TickStewing = 0;
			} else {
				s_TickStewing++;
			}				
		}
	} 
	else if((g_stAfe.stRamApp.fCDATA >= 500 && g_stAfe.stRamApp.fCDATA <= 1000/*LOCAL_CUR_DZ*/) || (g_stAfe.stRamApp.fCDATA <= -500 && g_stAfe.stRamApp.fCDATA >= -1000/*-LOCAL_CUR_DZ*/)) {
		pstPackRVal->fPackCur = g_stAfe.stRamApp.fCDATA * 0.001;
	}
	else {
		pstPackRVal->fPackCur = g_stAfe.stRamApp.fCDATA * 0.001;
		if(g_eBaseStat == eBStatStewing){
			static uint16_t s_TickStart = 0;
			if(s_TickStart * g_stCfg.stLocal.usCyclePeriod > 3000) {
				g_eBaseStat = eBStatWorkStart;
				his_data_write();
				s_TickStart = 0;
			} else {
				s_TickStart++;
			}
		}else{
		  g_eBaseStat = eBStatWorking; 
		}
	}
	if(pstPackRVal->fPackCurMax < pstPackRVal->fPackCur) {
		pstPackRVal->fPackCurMax = pstPackRVal->fPackCur;
	}
	{
		static uint16_t s_Tick = 0;
		if(pstPackRVal->fPackCur > 1000 || pstPackRVal->fPackCur < -1000) {
		  s_Tick ++;
		  if(s_Tick * g_stCfg.stLocal.usCyclePeriod > 3000){
			  pstPackRVal->uErrCode.stErrCode.bCurSensor = 1;
				his_log_write();
				s_Tick = 0;
			}
		} else {
			pstPackRVal->uErrCode.stErrCode.bCurSensor = 0;
			s_Tick = 0;
		}
  }
	/* Current B coefficient calibration */
	{
		static uint8_t s_ucTick = 0;
		if((g_stAfe.uRam.stCode.CHG_FET == 0 && g_stAfe.stRamApp.fCDATA > 1000/*LOCAL_CUR_DZ*/ && g_stAfe.stRamApp.fCDATA < /*LOCAL_CUR_DZ*/1000 * 5)
			|| (g_stAfe.uRam.stCode.DSG_FET == 0 && g_stAfe.stRamApp.fCDATA < -1000/*-LOCAL_CUR_DZ*/ && g_stAfe.stRamApp.fCDATA < /*-LOCAL_CUR_DZ*/-1000 * 5)) {
			if(s_ucTick * g_stCfg.stLocal.usCyclePeriod < 10000) {
				s_ucTick++;
			} else {
				g_stCfg.stAfe.fCDATACaliB -= g_stAfe.stRamApp.fCDATA;
				cfg_save();
			}
		} else {
			s_ucTick = 0;
		}
	}
	/* The remaining capacity of the cell is updated, and the ampere-hour integration method is used */
	float fDeltaAH = (pstPackRVal->fPackCur/*g_stAfe.stRamApp.fCDATA * 0.001 - 0.04 dgx*/) * g_stCfg.stLocal.usCyclePeriod / 1000 / 3600;	//-0.05A, becuase of including bms self energe using
	for(uint8_t i=0;i<CFG_CELL_NUM;i++) {
		pstPackRVal->afCellLeftAH[i] += fDeltaAH;
	}
	
	if(g_stCfg.stLocal.usDesignAH < 40) { // 2024/12/10 dgx
		g_stCfg.stLocal.usDesignAH = 95;
		cfg_save();
	}
	
	/* Cell Soc update */
	for(uint8_t i=0;i<CFG_CELL_NUM;i++) {
		float fSoc = pstPackRVal->afCellLeftAH[i] * 100 / pstPackRVal->afCellRealAH[i];
		if(pstPackRVal->afCellSoc[i] >= 100) {
			if(fSoc < 99.5) {
				pstPackRVal->afCellSoc[i] = fSoc;
			}
		} else if(pstPackRVal->afCellSoc[i] <= 0) {
			if(fSoc > 0.5) {
				pstPackRVal->afCellSoc[i] = fSoc;
			}
		} else {
			if(fSoc > 99) {
				pstPackRVal->afCellSoc[i] = 99;
			} else if(fSoc < 1) {
				pstPackRVal->afCellSoc[i] = 1;
			} else {
				pstPackRVal->afCellSoc[i] = fSoc;
			}
		}
	}
	/* Cell SOH update */
	for(uint8_t i=0;i<CFG_CELL_NUM;i++) {
		//float fSoh = pstPackRVal->afCellRealAH[i] * 100 / g_stCfg.stLocal.usDesignAH;
		if(pstPackRVal->usCycle <= 50) {
			pstPackRVal->afCellSoh[i] = 100;
		} else if(pstPackRVal->usCycle > 50 && pstPackRVal->usCycle <= 500) {
			pstPackRVal->afCellSoh[i] = 100 - (pstPackRVal->usCycle - 50) / 450 * 5;
		} else if(pstPackRVal->usCycle > 500 && pstPackRVal->usCycle <= 1000) {
			pstPackRVal->afCellSoh[i] = 95 - (pstPackRVal->usCycle - 500) / 500 * 5;
		} else if(pstPackRVal->usCycle > 1000 && pstPackRVal->usCycle <= 1500) {
			pstPackRVal->afCellSoh[i] = 90 - (pstPackRVal->usCycle - 1000) / 500 * 5;
		} else if(pstPackRVal->usCycle > 1500 && pstPackRVal->usCycle <= 2000) {
			pstPackRVal->afCellSoh[i] = 85 - (pstPackRVal->usCycle - 1500) / 500 * 5;
		} else if(pstPackRVal->usCycle > 2000 && pstPackRVal->usCycle <= 3000) {
			pstPackRVal->afCellSoh[i] = 80 - (pstPackRVal->usCycle - 2000) / 1000 * 10;
		} else {
			pstPackRVal->afCellSoh[i] = 70;
		}
	}
	/* Cell temperature update */
	uint8_t ucCellTMaxId = 0;
	uint8_t ucCellTMinId = 0;
	for(uint8_t i=0;i<CFG_TMP_NUM;i++) {
		pstPackRVal->afCellT[i] = g_stAfe.stRamApp.fTEMP[i];
		if(pstPackRVal->afCellT[ucCellTMaxId] < pstPackRVal->afCellT[i]) {
			ucCellTMaxId = i;
		}
		if(pstPackRVal->afCellT[ucCellTMinId] > pstPackRVal->afCellT[i]) {
			ucCellTMinId = i;
		}
	}
	pstPackRVal->ucCellTMaxId = ucCellTMaxId;
	pstPackRVal->ucCellTMinId = ucCellTMinId;
	pstPackRVal->fCellTMax = pstPackRVal->afCellT[ucCellTMaxId];
	pstPackRVal->fCellTMin = pstPackRVal->afCellT[ucCellTMinId];
	/* The temperature sensor is faulty */
	for(uint8_t i=0;i<CFG_TMP_NUM;i++) {
		if(pstPackRVal->afCellT[i] >= 110 || pstPackRVal->afCellT[i] < -50) {
			uErrCode.stErrCode.bTempSensor = 1;
		}
	}
	/* MOS temperature update */
	pstPackRVal->fMosT = (int16_t)ADC_Get_Temp(g_auiAdcBuf[1]);
//	pstPackRVal->fMosT = -10;
	/* The temperature sensor is faulty */
	if(pstPackRVal->fMosT > 150 || pstPackRVal->fMosT < -50) {
		uErrCode.stErrCode.bTempSensor = 1;
	}
	/* Standby left AH calibration */
	{
		static uint32_t s_usTick = 0;
		if(pstPackRVal->fPackCur < g_stCfg.stLocal.usSocCaliCurVal && pstPackRVal->fPackCur > ( - g_stCfg.stLocal.usSocCaliCurVal)) {
			if(s_usTick * g_stCfg.stLocal.usCyclePeriod < g_stCfg.stLocal.usSocCaliDelay * 3600000) {
				s_usTick++;
			}
		} else {
			s_usTick = 0;
		}
		if(g_bSocCaliFlag) {
			s_usTick = 0;
			g_bSocCaliFlag = false;
		}
		if(s_usTick * g_stCfg.stLocal.usCyclePeriod >= g_stCfg.stLocal.usSocCaliDelay * 3600000 || g_ucStandbyCali != 0) {
//			if(pstPackRVal->fPackSoc < 95) {
				local_standby_leftAH_cali();
//			}
			s_usTick = 0;
			g_ucStandbyCali = 0;
		}
	}
	/* Pack dischargeable capacity update */
	float fPackLeftAH = pstPackRVal->afCellLeftAH[0];
	for(uint8_t i=1;i<CFG_CELL_NUM;i++) {
		if(pstPackRVal->afCellLeftAH[i] > g_stCfg.stLocal.usDesignAH){
			pstPackRVal->afCellLeftAH[i] = g_stCfg.stLocal.usDesignAH;
		}
		if(fPackLeftAH > pstPackRVal->afCellLeftAH[i]) {
			fPackLeftAH = pstPackRVal->afCellLeftAH[i];
		}
	}
	pstPackRVal->fPackLeftAH = fPackLeftAH;
	if(pstPackRVal->fPackLeftAH < 0){
		pstPackRVal->fPackLeftAH = 0;
	}
	if(pstPackRVal->fPackLeftAH > g_stCfg.stLocal.usDesignAH){
		pstPackRVal->fPackLeftAH = g_stCfg.stLocal.usDesignAH;
	}
	/* Pack rechargeable capacity updated */
	float fPackChgEnAH = pstPackRVal->afCellRealAH[0] - pstPackRVal->afCellLeftAH[0];
	for(uint8_t i=1;i<CFG_CELL_NUM;i++) {
		if(fPackChgEnAH > pstPackRVal->afCellRealAH[i] - pstPackRVal->afCellLeftAH[i]) {
			fPackChgEnAH = pstPackRVal->afCellRealAH[i] - pstPackRVal->afCellLeftAH[i];
		}
	}
	if(fPackChgEnAH < 0){
		fPackChgEnAH = 0;
	}
	if(fPackChgEnAH > g_stCfg.stLocal.usDesignAH){
		fPackChgEnAH = g_stCfg.stLocal.usDesignAH;
	}

//  pstPackRVal->fPackRealAH = fPackChgEnAH + fPackLeftAH;
//	pstPackRVal->fPackRealAH = pstPackRVal->afCellRealAH[0];
//	for(uint8_t i=0;i<CFG_CELL_NUM;i++) {
//		if(pstPackRVal->afCellRealAH[i] > 100){
//			pstPackRVal->afCellRealAH[i] = 100;
//		}
//		if(pstPackRVal->fPackRealAH > pstPackRVal->afCellRealAH[i]) {
//			pstPackRVal->fPackRealAH = pstPackRVal->afCellRealAH[i];
//		}
//	}
//  if(pstPackRVal->fPackRealAH < fPackChgEnAH + fPackLeftAH)	{
//		pstPackRVal->fPackRealAH = fPackChgEnAH + fPackLeftAH;
//	}
//	if(pstPackRVal->fPackRealAH > g_stCfg.stLocal.usDesignAH){
//		pstPackRVal->fPackRealAH = g_stCfg.stLocal.usDesignAH;
//	}

	/* PACK Soh Update */
	float fPackSoh = pstPackRVal->afCellSoh[0];
	for(uint8_t i=1;i<CFG_CELL_NUM;i++) {
		if(fPackSoh > pstPackRVal->afCellSoh[i]) {
			fPackSoh = pstPackRVal->afCellSoh[i];
		}
	}
	pstPackRVal->fPackSoh = fPackSoh;
	
	/* Pack real capacitance updated */
	pstPackRVal->fPackRealAH = g_stCfg.stLocal.usDesignAH * pstPackRVal->fPackSoh /100 ;
	for(uint8_t i=0;i<CFG_CELL_NUM;i++) {
    pstPackRVal->afCellRealAH[i] = pstPackRVal->fPackRealAH;
	}
	if(pstPackRVal->fPackRealAH < g_stCfg.stLocal.usDesignAH * 0.7 ) {
			pstPackRVal->fPackRealAH = g_stCfg.stLocal.usDesignAH * 0.7;
		}
	if(pstPackRVal->fPackRealAH > g_stCfg.stLocal.usDesignAH) {
			pstPackRVal->fPackRealAH = g_stCfg.stLocal.usDesignAH;
		}
		/* PACK Soc Update */
	float fSoc = pstPackRVal->fPackLeftAH * 100 / pstPackRVal->fPackRealAH;
	if(pstPackRVal->fPackSoc >= 100) {
		if(fSoc < 99.5) {
			pstPackRVal->fPackSoc = 99;
		}
	} else if(pstPackRVal->fPackSoc <= 0) {
		if(fSoc > 0.5) {
			pstPackRVal->fPackSoc = 1;
		}
	}	else {
		if(fSoc > 99) {
			pstPackRVal->fPackSoc = 99;
		} else if(fSoc < 1) {
			pstPackRVal->fPackSoc = 1;
		} else {
			pstPackRVal->fPackSoc = fSoc;
		}
	}
	if(pstPackRVal->fPackSoc <= 0 && pstPackRVal->fCellUMin > g_stCfg.stLocal.ausSocECaliU[2] * 0.001){
		if(g_ucDsgZero != 1) {
				g_ucDsgZero = 1;
				local_standby_leftAH_cali();
		}
	}
	
//	if(fSoc > 99 && pstPackRVal->fCellUMax < g_stCfg.stLocal.ausSocFCaliU[1] * 0.001 ) {
//			pstPackRVal->fPackSoc = 99;
//		}else if((fSoc < 1 && pstPackRVal->fCellUMin > g_stCfg.stLocal.ausSocECaliU[2] * 0.001)){	
//			pstPackRVal->fPackSoc = 1;
//	  }else {
//			pstPackRVal->fPackSoc = fSoc;
//		}
		
	/* Basic status updates */
	//pstPackRVal->uBaseStat.stBaseStat.ucChgerON = ??		//12VDC input jugde
	if(g_stAfe.stRamApp.fCDATA >= 1000/*LOCAL_CUR_DZ*/) {		/* State of charge */
		static uint16_t s_usTick = 0;
		if(s_usTick * g_stCfg.stLocal.usCyclePeriod < 1000) {
			s_usTick++;
		} else {
			s_usTick = 0;
		}
		if(s_usTick * g_stCfg.stLocal.usCyclePeriod >= 1000) {
			pstPackRVal->uBaseStat.stBaseStat.ucChg = 1;
			pstPackRVal->uBaseStat.stBaseStat.ucDsg = 0;
			pstPackRVal->uBaseStat.stBaseStat.ucStandby = 0;
		}
	} else if(g_stAfe.stRamApp.fCDATA <= -1000/*LOCAL_CUR_DZ*/) {		/* Discharge status */
		static uint16_t s_usTick = 0;
		if(s_usTick * g_stCfg.stLocal.usCyclePeriod < 1000) {
			s_usTick++;
		} else {
			s_usTick = 0;
		}
		if(s_usTick * g_stCfg.stLocal.usCyclePeriod >= 1000) {
			pstPackRVal->uBaseStat.stBaseStat.ucChg = 0;
			pstPackRVal->uBaseStat.stBaseStat.ucDsg = 1;
			pstPackRVal->uBaseStat.stBaseStat.ucStandby = 0;
		}
	} else {		/* Resting state */
		/* keeping over 60s, then change to standby mode */
//		static uint16_t s_usTick = 0;
//		if(s_usTick * g_stCfg.stLocal.usCyclePeriod < 60000) {
//			s_usTick++;
//		} else {
//			s_usTick = 0;
//		}
//		if(s_usTick * g_stCfg.stLocal.usCyclePeriod >= 60000) {
			pstPackRVal->uBaseStat.stBaseStat.ucChg = 0;
			pstPackRVal->uBaseStat.stBaseStat.ucDsg = 0;
			pstPackRVal->uBaseStat.stBaseStat.ucStandby = 1;
//		}
	}
	/* Update the number of charge/discharge cycles */
	static float fCycle = 0;
	fCycle += __fabs(g_stAfe.stRamApp.fCDATA * 0.001 / 3600 / g_stCfg.stLocal.usDesignAH * g_stCfg.stLocal.usCyclePeriod / 1000) * 0.5;
	if(fCycle >= 1) {
		if(pstPackRVal->usCycle + 1 < 0xFFFF) {
			pstPackRVal->usCycle ++;
		}
		fCycle -= 1;
	}
	/* Error code update */
	if(GET_ALM0_CODE(6) || GET_ALM1_CODE(6) || GET_ALM1_CODE(56)) {
		uErrCode.stErrCode.bInputOV = 1;
	} else {
		uErrCode.stErrCode.bInputOV = 0;
	}
	if((pstPackRVal->uErrCode.stErrCode.bVoltSensor != uErrCode.stErrCode.bVoltSensor)
		|| (pstPackRVal->uErrCode.stErrCode.bTempSensor != uErrCode.stErrCode.bTempSensor)
		|| (pstPackRVal->uErrCode.stErrCode.bInterComm != uErrCode.stErrCode.bInterComm)
		|| (pstPackRVal->uErrCode.stErrCode.bInputOV != uErrCode.stErrCode.bInputOV)) {
		his_log_write();
	}
	pstPackRVal->uErrCode = uErrCode;
	/* When a cell reaches the high limit voltage, the SOC of the cell is calibrated to 100% */
	if(pstPackRVal->fCellUMax < g_stCfg.stLocal.ausSocFCaliU[1] * 0.001) {
		s_iTick = 0;
	}
	if(/* g_stLocalArrayRVal.uBaseStat.stBaseStat.ucChg && */g_ucChgFull == 0 && s_iTick == 0) {
//		for(uint8_t i=0;i<CFG_CELL_NUM;i++) {
		static uint16_t s_usTick1/*[CFG_CELL_NUM]*/ = {0};
		static uint16_t s_usTick2/*[CFG_CELL_NUM]*/ = {0};
		if(/*pstPackRVal->afCellU[i]*/pstPackRVal->fCellUMax > g_stCfg.stLocal.ausSocFCaliU[1] * 0.001 && pstPackRVal->fPackCur < 25 && pstPackRVal->fPackCur > 1) {
			if(s_usTick1/*[i]*/ * g_stCfg.stLocal.usCyclePeriod < 5000) {
				s_usTick1/*[i]*/++;
			}
		} else {
			s_usTick1/*[i]*/ = 0;
		}
		if(/*pstPackRVal->afCellU[i]*/pstPackRVal->fCellUMax > g_stCfg.stLocal.ausSocFCaliU[0] * 0.001) {
			if(s_usTick2/*[i]*/ * g_stCfg.stLocal.usCyclePeriod < 5000) {
				s_usTick2/*[i]*/++;
			}
		} else {
			s_usTick2/*[i]*/ = 0;
		}
		if(s_usTick1/*[i]*/ * g_stCfg.stLocal.usCyclePeriod >= 5000 || s_usTick2/*[i]*/ * g_stCfg.stLocal.usCyclePeriod >= 5000) {
			for(uint8_t j=0;j<CFG_CELL_NUM;j++){
				pstPackRVal->afCellLeftAH[j] = pstPackRVal->fPackRealAH;
			}
			for(uint8_t i=0;i<CFG_CELL_NUM;i++){
				pstPackRVal->fPackLeftAH = pstPackRVal->fPackRealAH;
				pstPackRVal->afCellSoc[i] = 100.5;
				pstPackRVal->fPackSoc = 100.5;
				s_iTick++;
				s_usTick1/*[i]*/ = 0;
				s_usTick2/*[i]*/ = 0;
				pstPackRVal->fReqChgI=0;
				g_stCfg.stLocal.sBalanceOn = 0;
				if(pstPackRVal->afCellRealAH[i] < g_stCfg.stLocal.usDesignAH * 0.7) {
					pstPackRVal->afCellRealAH[i] = g_stCfg.stLocal.usDesignAH * 0.7;
					pstPackRVal->afCellLeftAH[i] = pstPackRVal->afCellRealAH[i];
				}
				if(pstPackRVal->afCellRealAH[i] > g_stCfg.stLocal.usDesignAH) {
					pstPackRVal->afCellRealAH[i] = g_stCfg.stLocal.usDesignAH * 1.005;
					pstPackRVal->afCellLeftAH[i] = pstPackRVal->afCellRealAH[i];
				}
				if(g_ucChgFull != 1) {
					g_ucChgFull= 1;
//					ucNeedSave = 1;
				}
				g_ucFullChgRec = 1;
				break;
			}
		}
//		}
	}
	/* When a cell reaches the low limit of nuclear capacitance, the SOC of the cell is calibrated to 15%*/
	{
		if(pstPackRVal->fPackSoc > 16) {
//			for(uint8_t i=0;i<CFG_CELL_NUM;i++) {
				static uint8_t s_usTick1/*[CFG_CELL_NUM] */= /*{0}*/0;
				if((pstPackRVal->fCellTMin >= 15 && /*pstPackRVal->afCellU[i]*/pstPackRVal->fCellUMin <= 3.05 && ((pstPackRVal->fPackCur > -5 && pstPackRVal->fPackCur < 0) || g_eBaseStat == eBStatStewing))
					|| (pstPackRVal->fCellTMin < 15 && pstPackRVal->fCellTMin >= 5 &&/*pstPackRVal->afCellU[i]*/pstPackRVal->fCellUMin <= 3.00 && ((pstPackRVal->fPackCur > -5 && pstPackRVal->fPackCur < 0) || g_eBaseStat == eBStatStewing))
					|| (pstPackRVal->fCellTMin < 5 && /*pstPackRVal->afCellU[i]*/pstPackRVal->fCellUMin <= 2.95 && ((pstPackRVal->fPackCur > -5 && pstPackRVal->fPackCur < 0) || g_eBaseStat == eBStatStewing))) {
					if(s_usTick1/*[i]*/ * g_stCfg.stLocal.usCyclePeriod < 5000) {
						s_usTick1/*[i]*/++;
					}
				} else {
					s_usTick1/*[i]*/ = 0;
				}
				if(s_usTick1/*[i]*/ * g_stCfg.stLocal.usCyclePeriod >= 5000) {
					for(uint8_t j=0;j<CFG_CELL_NUM;j++) {
						pstPackRVal->afCellLeftAH[j] = pstPackRVal->afCellRealAH[j] * 0.155;
						pstPackRVal->afCellSoc[j] = 15.5;
						pstPackRVal->fPackSoc = 15.5;
						s_usTick1/*[i]*/ = 0;
					}
					local_soc_cali_save();	//Added by h00205922, 2024.04.01
					g_bSocCaliFlag = true;
//					break;
				}
//			}
		}
	}
	/* When a cell reaches the low limit of nuclear capacitance, the SOC of the cell is calibrated to 10% */
	{
		if(pstPackRVal->fPackSoc > 11) {
//			for(uint8_t i=0;i<CFG_CELL_NUM;i++) {
				static uint8_t s_usTick1/*[CFG_CELL_NUM]*/ = /*{0}*/0;
				if((pstPackRVal->fCellTMin >= 15 && /*pstPackRVal->afCellU[i]*/pstPackRVal->fCellUMin <= g_stCfg.stLocal.ausSocECaliU[0] * 0.001 && pstPackRVal->fPackCur > -100 && pstPackRVal->fPackCur < -5)
					|| (pstPackRVal->fCellTMin < 15 && pstPackRVal->fCellTMin >= 5 && /*pstPackRVal->afCellU[i]*/pstPackRVal->fCellUMin <= 2750 * 0.001 && pstPackRVal->fPackCur > -100 && pstPackRVal->fPackCur < -5)
					|| (pstPackRVal->fCellTMin < 5 && /*pstPackRVal->afCellU[i]*/pstPackRVal->fCellUMin <= 2650 * 0.001 && pstPackRVal->fPackCur > -100 && pstPackRVal->fPackCur < -5)) {
					if(s_usTick1/*[i]*/ * g_stCfg.stLocal.usCyclePeriod < 5000) {
						s_usTick1/*[i]*/++;
					}
				} else {
					s_usTick1/*[i]*/ = 0;
				}
				if(s_usTick1/*[i]*/ * g_stCfg.stLocal.usCyclePeriod >= 5000) {
					for(uint8_t j=0;j<CFG_CELL_NUM;j++) {
						pstPackRVal->afCellLeftAH[j] = pstPackRVal->afCellRealAH[j] * 0.105;
						pstPackRVal->afCellSoc[j] = 10.5;
						pstPackRVal->fPackSoc = 10.5;
						s_usTick1/*[i]*/ = 0;
					}
					local_soc_cali_save();	//Added by h00205922, 2024.04.01
					g_bSocCaliFlag = true;
//					break;
				}
//			}
		}
	}
	/* When a cell reaches the low limit of nuclear capacitance, the SOC of the cell is calibrated to 5% */
	{
		if(pstPackRVal->fPackSoc > 6) {
//			for(uint8_t i=0;i<CFG_CELL_NUM;i++) {
				static uint8_t s_usTick1/*[CFG_CELL_NUM]*/ = /*{0}*/0;
				if((pstPackRVal->fCellTMin >= 15 && /*pstPackRVal->afCellU[i]*/pstPackRVal->fCellUMin <= g_stCfg.stLocal.ausSocECaliU[1] * 0.001)
					|| (pstPackRVal->fCellTMin < 15 && pstPackRVal->fCellTMin >= 5 && /*pstPackRVal->afCellU[i]*/pstPackRVal->fCellUMin <= 2600 * 0.001)
					|| (pstPackRVal->fCellTMin < 5 && /*pstPackRVal->afCellU[i]*/pstPackRVal->fCellUMin <= 2500 * 0.001)) {
					if(s_usTick1/*[i]*/ * g_stCfg.stLocal.usCyclePeriod < 5000) {
						s_usTick1/*[i]*/++;
					}
				} else {
					s_usTick1/*[i]*/= 0;
				}
				if(s_usTick1/*[i]*/ * g_stCfg.stLocal.usCyclePeriod >= 5000) {
					for(uint8_t j=0;j<CFG_CELL_NUM;j++) {
						pstPackRVal->afCellLeftAH[j] = pstPackRVal->afCellRealAH[j] * 0.055;
						pstPackRVal->afCellSoc[j] = 5.5;
						pstPackRVal->fPackSoc = 5.5;
						s_usTick1/*[i]*/ = 0;
					}
					local_soc_cali_save();	//Added by h00205922, 2024.04.01
					g_bSocCaliFlag = true;
//					break;
				}
//			}
		}
	}
	/* When a cell reaches the low limit of nuclear capacitance, the SOC of the cell is calibrated to 0%*/
	{
		static uint16_t s_usTick = 0;
		if((pstPackRVal->fCellTMin >= 10 && pstPackRVal->fCellUMin <= (g_stCfg.stLocal.ausSocECaliU[2]) * 0.001)
			|| (pstPackRVal->fCellTMin < 10 && pstPackRVal->fCellUMin <= (g_stCfg.stLocal.ausSocECaliU[2] - 200) * 0.001)) {
			if(s_usTick * g_stCfg.stLocal.usCyclePeriod < 5000) {
				s_usTick++;
			}
			} else {
				s_usTick = 0;
		}
		if(s_usTick * g_stCfg.stLocal.usCyclePeriod >= 5000) {
//			pstPackRVal->afCellLeftAH[ucCellUMinId] = 0;
//			pstPackRVal->afCellSoc[ucCellUMinId] = 0;
//			pstPackRVal->fPackSoc = 0;
			s_usTick = 0;
			for(uint8_t j=0; j<CFG_CELL_NUM; j++) {
				pstPackRVal->afCellLeftAH[j] = 0;
				pstPackRVal->afCellSoc[j] = 0;
				pstPackRVal->fPackSoc = 0;
				if(pstPackRVal->afCellLeftAH[j] < 0){
					pstPackRVal->afCellLeftAH[j] = 0;
				}
			}
			g_ucDsgZero = 0;
			if(g_ucDsgEmpty != 1) {
				g_ucDsgEmpty= 1;
				local_soc_cali_save();
				g_bSocCaliFlag = true;
			}
		}
	}

//	if(ucNeedSave != 0) {
//		his_data_write();
//		ucNeedSave = 0;
//	}
	if(pstPackRVal->fPackSoc < 95) {//95
		g_ucChgFull = 0;
	}
	if((pstPackRVal->fPackSoc < 100 && !(g_bMChgerComAct || g_bSChgerComAct))
		 || (pstPackRVal->fPackSoc < 90 && (g_bMChgerComAct || g_bSChgerComAct))) {
		g_ucFullChgRec = 0;
	}
	if(pstPackRVal->fPackSoc > 0) {//>=20
		g_ucDsgEmpty = 0;
	}
	
	return;
}

/*******************************************************************************
* Function Name  : local_almptct
* Description    : Alarm and protection detection
* Input          : None
* Output         : None
* Return         : result, 1Write successfully 0Write failed
*******************************************************************************/
void local_signal_ptct(void) {
	LOCAL_PACK_RVAL_S* pstPackRVal = g_stLocalArrayRVal.astPackRVal + g_stPrl.ucSelfId;
	float fChgLmtCur = pstPackRVal->fLmtChgI;
	float fDsgLmtCur = pstPackRVal->fLmtDsgI;
	uint8_t usAlmLevel = 0;
	
	/* trigger: External Short-circuit */
	{
		static uint8_t s_ucTick = 0;
		if(g_stAfe.uRam.stCode.SC == 1 || pstPackRVal->fPackCur >= 2000 || pstPackRVal->fPackCur <= -2000) {//1000
			if(s_ucTick * g_stCfg.stLocal.usCyclePeriod >= 1000) {
				SET_ALM0_CODE(2);
				SET_ALM1_CODE(2);
				g_eBaseAlm.ALM_CODE2 = 1;
				g_ChgDisable[2] = 1;
				g_DsgDisable[2] = 1;
				pstPackRVal->ucChgEn = 0xAA;
				pstPackRVal->ucDsgEn = 0xAA;
				fDsgLmtCur = 0;
				fChgLmtCur = 0;
				usAlmLevel = 1;
			} else {
				s_ucTick++;
			}
		} else {
			s_ucTick = 0;
		}
	}
	/* trigger: Volt. Difference >1000mV */
	{
		static uint8_t s_ucTick = 0;
		if(pstPackRVal->fCellUMax - pstPackRVal->fCellUMin > 1) {
			if(s_ucTick * g_stCfg.stLocal.usCyclePeriod >= 3000) {
				SET_ALM0_CODE(3);
				SET_ALM1_CODE(3);
				g_eBaseAlm.ALM_CODE3 = 1;
				g_ChgDisable[3] = 1;
				g_DsgDisable[3] = 1;
				pstPackRVal->ucChgEn = 0xAA;
				pstPackRVal->ucDsgEn = 0xAA;
				fDsgLmtCur = 0;
				usAlmLevel = 1;
			} else {
				s_ucTick++;
			}
		} else {
			s_ucTick = 0;
		}
	}
	/* recover: Volt. Difference < 1000mV */
	if(pstPackRVal->fCellUMax - pstPackRVal->fCellUMin < 1 && GET_ALM0_CODE(3)&& GET_ALM1_CODE(3)) {
		RESET_ALM0_CODE(3);
		RESET_ALM1_CODE(3);
		g_eBaseAlm.ALM_CODE3 = 0;
		usAlmLevel = 0;
		g_ChgDisable[3] = 0;
		g_DsgDisable[3] = 0;
		pstPackRVal->ucChgEn = 0x55;
		pstPackRVal->ucDsgEn = 0x55;
	}
	/* trigger: When charging, the voltage of the unit is greater than 3.7V */
	{
		static uint8_t s_ucTick = 0;
		if(pstPackRVal->fCellUMax > g_stCfg.stLocal.usCellOVTVThr1 * 0.001/* && pstPackRVal->uBaseStat.stBaseStat.ucChg*/) {
			if(s_ucTick * g_stCfg.stLocal.usCyclePeriod >= g_stCfg.stLocal.usCellOVTTThr2) {
				SET_ALM0_CODE(4);
				SET_ALM1_CODE(4);
				g_eBaseAlm.ALM_CODE4 = 1;
				g_ChgDisable[4] = 1;
				pstPackRVal->ucChgEn = 0xAA;
				for(uint8_t j=0;j<CFG_CELL_NUM;j++){
				pstPackRVal->afCellLeftAH[j] = pstPackRVal->fPackRealAH;
				}
				for(uint8_t i=0;i<CFG_CELL_NUM;i++){
					pstPackRVal->fPackLeftAH = pstPackRVal->fPackRealAH;
					pstPackRVal->afCellSoc[i] = 100.5;
					pstPackRVal->fPackSoc = 100.5;
					if(pstPackRVal->afCellRealAH[i] < g_stCfg.stLocal.usDesignAH * 0.7) {
						pstPackRVal->afCellRealAH[i] = g_stCfg.stLocal.usDesignAH * 0.7;
						pstPackRVal->afCellLeftAH[i] = pstPackRVal->afCellRealAH[i];
					}
					if(pstPackRVal->afCellRealAH[i] > g_stCfg.stLocal.usDesignAH) {
						pstPackRVal->afCellRealAH[i] = g_stCfg.stLocal.usDesignAH * 1.005;
						pstPackRVal->afCellLeftAH[i] = pstPackRVal->afCellRealAH[i];
					}
					if(g_ucChgFull != 1) {
						g_ucChgFull= 1;
					}
				}
				fChgLmtCur = 0;
				usAlmLevel = 1;
			} else {
				s_ucTick++;
			}
		} else {
			s_ucTick = 0;
		}
	}
	/* recover: When charging, the voltage of the unit is greater than 3.55V */
	if(pstPackRVal->uBaseStat.stBaseStat.ucChg == 0 && pstPackRVal->fCellUMax < g_stCfg.stLocal.usCellOVRVThr1 * 0.001 && GET_ALM0_CODE(4) && GET_ALM1_CODE(4)) {
		RESET_ALM0_CODE(4);
		RESET_ALM1_CODE(4);
		g_eBaseAlm.ALM_CODE4 = 0;
		usAlmLevel = 0;
		pstPackRVal->ucChgEn = 0x55;
		g_ChgDisable[4] = 0;
	}

	/* trigger: When charging, the voltage of the unit is greater than 3.65V */
	{
		static uint8_t s_ucTick1 = 0;
		{
			static uint8_t s_ucTick = 0;
			if(pstPackRVal->fCellUMax < g_stCfg.stLocal.usCellOVTVThr1 && pstPackRVal->fCellUMax > (g_stCfg.stLocal.usCellOVTVThr2 + 150) * 0.001/* && pstPackRVal->uBaseStat.stBaseStat.ucChg*/) {
				if(s_ucTick * g_stCfg.stLocal.usCyclePeriod >= g_stCfg.stLocal.usCellOVTTThr1) {
					if(s_ucTick1 == 0) {
						g_auiAlmCode1[(58-1)/32]|=((uint32_t)0x00000001 << ((58-1)%32));
						GET_ALM1_CODE(58);
						eco_refresh_CD();
						eco_refresh_log();
						eco_refresh_RTV();
						his_log_write();
						his_data_write();
						s_ucTick1++;
					}
					g_ChgDisable[4] = 1;
					pstPackRVal->ucChgEn = 0xAA;
					fChgLmtCur = 0;
				} else {
					s_ucTick++;
				}
			} else {
				s_ucTick = 0;
			}
		}
		/* recover: When charging, the voltage of the unit is greater than 3.55V */
		if(pstPackRVal->uBaseStat.stBaseStat.ucChg == 0 && pstPackRVal->fCellUMax < g_stCfg.stLocal.usCellOVRVThr1 * 0.001 && GET_ALM1_CODE(58)) {
			RESET_ALM1_CODE(58);
			g_ChgDisable[4] = 0;
			s_ucTick1 = 0;
		}
	}
	
	{
		static uint8_t s_ucTick2 = 0;
/* trigger: Temperature =10? and Cell volt.<2.45V or Temperature <10? and Cell volt.<2.30V, BMS turn off  */
		{
			static uint8_t s_ucTick = 0;
			static uint16_t s_ucTick1 = 0;
			if((pstPackRVal->fCellTMin >= 10 && pstPackRVal->fCellUMin < g_stCfg.stLocal.usCellUVTVThr1 * 0.001)
				|| (pstPackRVal->fCellTMin < 10 && pstPackRVal->fCellUMin < 2300 * 0.001)) {
				if(s_ucTick * g_stCfg.stLocal.usCyclePeriod >= g_stCfg.stLocal.usCellUVTTThr1) {
					if(s_ucTick2 == 0) {
						RESET_ALM0_CODE(5);
						RESET_ALM1_CODE(5);
//						pstPackRVal->afCellLeftAH[pstPackRVal->ucCellUMinId] = 0;
//						pstPackRVal->afCellSoc[pstPackRVal->ucCellUMinId] = 0;
//						pstPackRVal->fPackSoc = 0;
						for(uint8_t j=0; j<CFG_CELL_NUM; j++) {
							pstPackRVal->afCellLeftAH[j] = 0;
							pstPackRVal->afCellSoc[j] = 0;
							pstPackRVal->fPackSoc = 0;
							if(pstPackRVal->afCellLeftAH[j] < 0){
								pstPackRVal->afCellLeftAH[j] = 0;
							}
						}
						s_ucTick2++;
					}
					SET_ALM0_CODE(5);
					SET_ALM1_CODE(5);
					g_eBaseAlm.ALM_CODE5 = 1;
					g_DsgDisable[5] = 1;
					pstPackRVal->ucDsgEn = 0xAA;
					fDsgLmtCur = 0;
					usAlmLevel = 1;
				} else {
					s_ucTick++;
				}
				
				if(s_ucTick1 * g_stCfg.stLocal.usCyclePeriod >= 60 * 1000) {
						g_stCfg.usGoRun = 0;
						cfg_save();
						g_bNeedSleep = true;
					} else{
							s_ucTick1++;
					}
					
			} else {
				s_ucTick = 0;
				s_ucTick1 = 0;
				s_ucTick2 = 0;
			}
		}
	/* recover: Cell vot. > 2.90V */
		if((pstPackRVal->fCellUMin > g_stCfg.stLocal.usCellUVRVThr1 * 0.001 || pstPackRVal->uBaseStat.stBaseStat.ucChg == 1)&& GET_ALM0_CODE(5) && GET_ALM1_CODE(5)) {
			s_ucTick2 = 0;
		  RESET_ALM0_CODE(5);
		  RESET_ALM1_CODE(5);
			g_eBaseAlm.ALM_CODE5 = 0;
			pstPackRVal->ucDsgEn = 0x55;
		  usAlmLevel = 0;
			g_DsgDisable[5] = 0;
		}
	}
	/* trigger: Temperature =10? and Cell volt.<2.55V or Temperature <10? and Cell volt.<2.35V */
	{
		static uint8_t s_ucTick = 0;
		if((pstPackRVal->fCellTMin >= 10 && pstPackRVal->fCellUMin < g_stCfg.stLocal.usCellUVTVThr2 * 0.001 && pstPackRVal->fCellUMin > g_stCfg.stLocal.usCellUVTVThr1 * 0.001)
			|| (pstPackRVal->fCellTMin < 10 && pstPackRVal->fCellUMin < 2350 * 0.001 && pstPackRVal->fCellUMin > 2300 * 0.001)) {
			if(s_ucTick * g_stCfg.stLocal.usCyclePeriod >= g_stCfg.stLocal.usCellUVTTThr2) {
				SET_ALM0_CODE(5);
				SET_ALM1_CODE(5);
				g_eBaseAlm.ALM_CODE5 = 1;
				fDsgLmtCur = 0;
				usAlmLevel = 1;
			} else {
				s_ucTick++;
			}
		} else {
			s_ucTick = 0;
		}
	}
	/* trigger: When charging, PACK volt.>3.60*S(cells in series) */
	{
		static uint8_t s_ucTick = 0;
		if(pstPackRVal->fPackU > g_stCfg.stLocal.usPackOVTVThr1 * 0.1/* && pstPackRVal->fPackCur > 2*/) {
			if(s_ucTick * g_stCfg.stLocal.usCyclePeriod >= g_stCfg.stLocal.usPackOVTTThr1) {
				SET_ALM0_CODE(6);
				SET_ALM1_CODE(6);
				g_eBaseAlm.ALM_CODE6 = 1;
				g_ChgDisable[6] = 1;
				pstPackRVal->ucChgEn = 0xAA;
				fChgLmtCur = 0;
				usAlmLevel = 1;
			} else {
				s_ucTick++;
			}
		} else {
			s_ucTick = 0;
		}
	}
	/* recover: When charging, PACK volt.>3.65*S(cells in series) */
	if(pstPackRVal->uBaseStat.stBaseStat.ucChg == 0 && (pstPackRVal->fPackU <= g_stCfg.stLocal.usPackOVRVThr1 * 0.1) && GET_ALM0_CODE(6) && GET_ALM1_CODE(6)) {
		RESET_ALM0_CODE(6);
		RESET_ALM1_CODE(6);
		g_eBaseAlm.ALM_CODE6 = 0;
		usAlmLevel = 0;
		g_ChgDisable[6] = 0;
		pstPackRVal->ucChgEn = 0x55;
	}

	{
		static uint8_t s_ucTick1 = 0;
	/* trigger: Temperature =10? and PACK vot.<2.50*SV or Temperature < 10? and PACK vot.<2.35*SV(S is number of cells in series) */
		{
			static uint8_t s_ucTick = 0;
			if((pstPackRVal->fCellTMin >= 10 && pstPackRVal->fPackU < g_stCfg.stLocal.usPackUVTVThr1 * 0.1)
				|| (pstPackRVal->fCellTMin < 10 && pstPackRVal->fPackU < 376 * 0.1)) {
				if(s_ucTick * g_stCfg.stLocal.usCyclePeriod >= g_stCfg.stLocal.usPackUVTTThr1) {
					if(s_ucTick1 == 0) {
						RESET_ALM0_CODE(7);
						RESET_ALM1_CODE(7);
						s_ucTick1++;
					}
					for(uint8_t j=0; j<CFG_CELL_NUM; j++) {
						pstPackRVal->afCellLeftAH[j] = 0;
						pstPackRVal->afCellSoc[j] = 0;
						pstPackRVal->fPackSoc = 0;
						if(pstPackRVal->afCellLeftAH[j] < 0){
							pstPackRVal->afCellLeftAH[j] = 0;
						}
					}
					SET_ALM0_CODE(7);
					SET_ALM1_CODE(7);
					g_eBaseAlm.ALM_CODE7 = 1;
					g_DsgDisable[7] = 1;
					pstPackRVal->ucDsgEn = 0xAA;
					fDsgLmtCur = 0;
					usAlmLevel = 1;
				} else {
					s_ucTick++;
				}
			} else {
				s_ucTick = 0;
			}
		}
	/* recover: PACK vot.>2.9*SV(S is number of cells in series) */
		if((pstPackRVal->fPackU > g_stCfg.stLocal.usPackUVRVThr1 * 0.1  || pstPackRVal->uBaseStat.stBaseStat.ucChg == 1) && GET_ALM0_CODE(7)&& GET_ALM1_CODE(7)) {
			s_ucTick1 = 0;
		  RESET_ALM0_CODE(7);
		  RESET_ALM1_CODE(7);
		  g_eBaseAlm.ALM_CODE7 = 0;
		  g_DsgDisable[7] = 0;
			usAlmLevel = 0;
		}
	}
	/* trigger: Temperature =10? and PACK vot.<2.6*SV or Temperature < 10? and PACK vot.<2.4*SV(S is number of cells in series) */
	{
		static uint8_t s_ucTick = 0;
		if((pstPackRVal->fCellTMin >= 10 && pstPackRVal->fPackU < g_stCfg.stLocal.usPackUVTVThr2 * 0.1 && pstPackRVal->fPackU > g_stCfg.stLocal.usPackUVTVThr1 * 0.1)
			|| (pstPackRVal->fCellTMin < 10 && pstPackRVal->fPackU < 384 * 0.1 && pstPackRVal->fPackU > 376 * 0.1)) {
			if(s_ucTick * g_stCfg.stLocal.usCyclePeriod >= g_stCfg.stLocal.usPackUVTTThr2) {
				SET_ALM0_CODE(7);
				SET_ALM1_CODE(7);
				g_eBaseAlm.ALM_CODE7 = 1;
				fDsgLmtCur = 0;
				usAlmLevel = 1;
			} else {
				s_ucTick++;
			}
		} else {
			s_ucTick = 0;
		}
	}
	/* trigger: under -400A when discharging */
	{
		static uint8_t s_ucTick = 0;
		if(pstPackRVal->fPackCur < -g_stCfg.stLocal.usODCTVThr1) {
			if(s_ucTick * g_stCfg.stLocal.usCyclePeriod >= g_stCfg.stLocal.usODCTTThr1) {
				SET_ALM0_CODE(8);
				SET_ALM1_CODE(8);
				g_eBaseAlm.ALM_CODE8 = 1;
				g_DsgDisable[8] = 1;
				pstPackRVal->ucDsgEn = 0xAA;
				fDsgLmtCur = 0;
			} else {
				s_ucTick++;
			}
		} else {
			s_ucTick = 0;
		}
	}
	/* recover: over 400A when discharging */
	{
		static uint8_t s_ucTick = 0;
		if(pstPackRVal->fPackCur > -g_stCfg.stLocal.usODCRVThr1 && GET_ALM0_CODE(8) && GET_ALM1_CODE(8)) {
			if(s_ucTick * g_stCfg.stLocal.usCyclePeriod >= 1000) {
			  RESET_ALM0_CODE(8);
				RESET_ALM1_CODE(8);
				g_eBaseAlm.ALM_CODE8 = 0;
				usAlmLevel = 0;
				g_DsgDisable[8] = 0;
			} else {
				s_ucTick++;
			}
		} else {
			s_ucTick = 0;
		}
  	}
	/* trigger: over 250A when charging */
	{
		static uint8_t s_ucTick = 0;
		if(pstPackRVal->fPackCur > g_stCfg.stLocal.usOCCTVThr1) {
			if(s_ucTick * g_stCfg.stLocal.usCyclePeriod >= g_stCfg.stLocal.usOCCTTThr1) {
				SET_ALM0_CODE(9);
				SET_ALM1_CODE(9);
				g_eBaseAlm.ALM_CODE9 = 1;
				g_ChgDisable[9] = 1;
				pstPackRVal->ucChgEn = 0xAA;
				fChgLmtCur = 0;
			} else {
				s_ucTick++;
			}
		} else {
			s_ucTick = 0;
		}
	}
	/* recover: over 250A when charging */
	{
		static uint8_t s_ucTick = 0;
		if(pstPackRVal->fPackCur < g_stCfg.stLocal.usOCCRVThr1 && GET_ALM0_CODE(9) && GET_ALM1_CODE(9)) {
			if(s_ucTick * g_stCfg.stLocal.usCyclePeriod >= 1000) {
			  RESET_ALM0_CODE(9);
				RESET_ALM1_CODE(9);
				g_eBaseAlm.ALM_CODE9 = 0;
				usAlmLevel = 0;
				g_ChgDisable[9] = 0;
			} else {
				s_ucTick++;
			}
		} else {
			s_ucTick = 0;
		}
  }
	/* trigger: Discharge: Over Temper. 65?? */
	{
		static uint8_t s_ucTick = 0;
		if(pstPackRVal->fCellTMax >= g_stCfg.stLocal.sCellODTTVThr1 && (pstPackRVal->uBaseStat.stBaseStat.ucDsg || pstPackRVal->uBaseStat.stBaseStat.ucStandby)) {
			if(s_ucTick * g_stCfg.stLocal.usCyclePeriod >= g_stCfg.stLocal.usCellODTTTThr1) {
				SET_ALM0_CODE(10);
				SET_ALM1_CODE(10);
				g_eBaseAlm.ALM_CODE10 = 1;
				fDsgLmtCur = 0;
				usAlmLevel = 1;
			} else {
				s_ucTick++;
			}
		} else {
			s_ucTick = 0;
		}
	}
	{	//delay 10s disconnect discharge MOS.
		static uint8_t s_ucTick = 0;
		if(GET_ALM0_CODE(10)) {
			if(s_ucTick * g_stCfg.stLocal.usCyclePeriod >= 10000) {
				pstPackRVal->ucDsgEn = 0xAA;
			} else {
				s_ucTick++;
			}
		} else {
			s_ucTick = 0;
		}
	}
	/* recover: Discharge: Over Temper. 60? */
	if(pstPackRVal->fCellTMax < g_stCfg.stLocal.sCellODTRVThr1 && GET_ALM0_CODE(10) && GET_ALM1_CODE(10)) {
		RESET_ALM0_CODE(10);
		RESET_ALM1_CODE(10);
		g_eBaseAlm.ALM_CODE10 = 0;
		pstPackRVal->ucDsgEn = 0x55;
		usAlmLevel = 0;
	}
	/* trigger: Charge: Over Temper. 55? */
	{
		static uint8_t s_ucTick = 0;
		if(pstPackRVal->fCellTMax >= g_stCfg.stLocal.sCellOCTTVThr1 && (pstPackRVal->uBaseStat.stBaseStat.ucChg || pstPackRVal->uBaseStat.stBaseStat.ucStandby)) {
			if(s_ucTick * g_stCfg.stLocal.usCyclePeriod >= g_stCfg.stLocal.usCellOCTTTThr1) {
				SET_ALM0_CODE(11);
				SET_ALM1_CODE(11);
				g_eBaseAlm.ALM_CODE11 = 1;
				g_ChgDisable[11] = 1;
				pstPackRVal->ucChgEn = 0xAA;
				fChgLmtCur = 0;
				usAlmLevel = 1;
			} else {
				s_ucTick++;
			}
		} else {
			s_ucTick = 0;
		}
	}
	/* recover: Charge: Over Temper. 55? */
	if(pstPackRVal->fCellTMax < g_stCfg.stLocal.sCellOCTRVThr1 && GET_ALM0_CODE(11) && GET_ALM1_CODE(11)) {
		RESET_ALM0_CODE(11);
		RESET_ALM1_CODE(11);
		g_eBaseAlm.ALM_CODE11 = 0;
		usAlmLevel = 0;
		g_ChgDisable[11] = 0;
		pstPackRVal->ucChgEn = 0x55;
	}
	
	/* new trigger: Low Temper. 0℃ 2024.12.28 */
	{
		static uint16_t s_ucTick = 0;
		static bool b_LowTemp = false;
		if(round(pstPackRVal->fCellTMin) <= g_stCfg.stLocal.sCellUCTTVThr1) {
			if(s_ucTick * g_stCfg.stLocal.usCyclePeriod >= g_stCfg.stLocal.usCellUCTTTThr1) {
				if(b_LowTemp == false) {
					g_eBaseStat = eBStatLowTemp;
					b_LowTemp = true;
				}
				SET_ALM0_CODE(12);
				SET_ALM1_CODE(12);
			} else {
				s_ucTick++;
			}
		} else {
			s_ucTick = 0;
		}
		
		if(round(pstPackRVal->fCellTMin) > g_stCfg.stLocal.sCellUCTRVThr1 && GET_ALM0_CODE(12) && GET_ALM1_CODE(12) && g_bChgLowTempFlag == false) {
			b_LowTemp = false;
			RESET_ALM0_CODE(12);
			RESET_ALM1_CODE(12);
		}
	}
	
	/* new trigger: Charge: Low Temper. 0? 2024.6.15 */
	{
		static uint16_t s_ucTick = 0;
		static uint16_t s_ucTick2 = 0;
		static uint16_t s_ucTick3 = 0;
		static uint16_t s_ucTick4 = 0;
		static uint16_t s_ucTick5 = 0;
		static uint16_t s_ucKeepTick1 = 0;
		static uint16_t s_ucKeepTick2 = 0;
		static uint8_t s_ucTick6 = 0;
		if(round(pstPackRVal->fCellTMin) <= g_stCfg.stLocal.sCellUCTTVThr1) {
			if(pstPackRVal->fPackCur > 120) {
				if(s_ucTick * g_stCfg.stLocal.usCyclePeriod >= 5 * 1000) {
					g_bChgLowTempFlag = true;
				} else {
					s_ucTick++;
				}
			} else {
				s_ucTick = 0;
			}
			if(pstPackRVal->fPackCur > 80 && pstPackRVal->fPackCur <= 120) {
				if(s_ucTick2 * g_stCfg.stLocal.usCyclePeriod >= 15 * 1000) {
					g_bChgLowTempFlag = true;
				} else {
					s_ucTick2++;
				}
			} else {
				s_ucTick2 = 0;
			}
			if(pstPackRVal->fPackCur > 50 && pstPackRVal->fPackCur <= 80) {
				if(s_ucTick3 * g_stCfg.stLocal.usCyclePeriod >= 20 * 1000) {
					g_bChgLowTempFlag = true;
				} else {
					s_ucTick3++;
				}
			} else {
				s_ucTick3 = 0;
			}
			if(pstPackRVal->fPackCur > 10 && pstPackRVal->fPackCur <= 50) {
				if(s_ucTick4 * g_stCfg.stLocal.usCyclePeriod >= 30 * 1000) {
					g_bChgLowTempFlag = true;
				} else {
					s_ucTick4++;
				}
			} else {
				s_ucTick4 = 0;
			}
			if(pstPackRVal->fPackCur > 2 && pstPackRVal->fPackCur <= 10) {
				if(s_ucTick5 * g_stCfg.stLocal.usCyclePeriod >= 60 * 1000) {
					g_bChgLowTempFlag = true;
				} else {
					s_ucTick5++;
				}
			} else {
				s_ucTick5 = 0;
			}
			if(g_bChgLowTempFlag == true) {
				if(s_ucKeepTick1 * g_stCfg.stLocal.usCyclePeriod <= 60 * 1000) {
					s_ucKeepTick1++;
					if(s_ucTick6 == 0) {
						SET_ALM0_CODE(12);
						SET_ALM1_CODE(12);
						s_ucTick6++;
					}
					g_ChgDisable[12] = 1;
					pstPackRVal->ucChgEn = 0xAA;
					fChgLmtCur = 0;
					usAlmLevel = 1;
				} 
			}
		}
		/* recover: Charge: Low Temper. 0? */
		if(/* round(pstPackRVal->fCellTMin) > g_stCfg.stLocal.sCellUCTTVThr1 */ g_bChgLowTempFlag == true && GET_ALM0_CODE(12) && GET_ALM1_CODE(12)) {
			if(s_ucKeepTick2 * g_stCfg.stLocal.usCyclePeriod <= 60 * 1000) {
				s_ucKeepTick2++;
			} else {
				s_ucTick6 = 0;
				RESET_ALM0_CODE(12);
				RESET_ALM1_CODE(12);
				usAlmLevel = 0;
				g_ChgDisable[12] = 0;
				pstPackRVal->ucChgEn = 0x55;
				s_ucKeepTick1 = 0;
				s_ucKeepTick2 = 0;
				g_bChgLowTempFlag = false;
			}
		}
	}
	/* trigger: Discharge: Low Temper. -15? */
	{
		static uint8_t s_ucTick = 0;
		if(pstPackRVal->fCellTMin <= g_stCfg.stLocal.sCellUDTTVThr1) {
			if(s_ucTick * g_stCfg.stLocal.usCyclePeriod >= g_stCfg.stLocal.usCellUDTTTThr1) {
				SET_ALM0_CODE(13);
				SET_ALM1_CODE(13);
				g_eBaseAlm.ALM_CODE13 = 1;
				g_DsgDisable[13] = 1;
				pstPackRVal->ucDsgEn = 0xAA;
				fDsgLmtCur = 0;
				usAlmLevel = 1;
			} else {
				s_ucTick++;
			}
		} else {
			s_ucTick = 0;
		}
	}
	/* recover: Discharge: Low Temper. -10? */
	if(pstPackRVal->fCellTMin > g_stCfg.stLocal.sCellUDTRVThr1 && GET_ALM0_CODE(13) && GET_ALM1_CODE(13)) {
		RESET_ALM0_CODE(13);
		RESET_ALM1_CODE(13);
		g_eBaseAlm.ALM_CODE13 = 0;
		usAlmLevel = 0;
		g_DsgDisable[13] = 0;
		pstPackRVal->ucDsgEn = 0x55;
	}
	{
		static uint8_t s_ucTick90 = 0;
	/* trigger: Hi-temper. of MOS 90? */
		{
			static uint8_t s_ucTick = 0;
			if(pstPackRVal->fMosT >= g_stCfg.stLocal.sMOSOTTVThr1) {
				if(s_ucTick * g_stCfg.stLocal.usCyclePeriod >= g_stCfg.stLocal.usMOSOTTTThr1) {
					if(s_ucTick90 == 0) {
						RESET_ALM0_CODE(14);
						RESET_ALM1_CODE(14);
						s_ucTick90++;
					}
					SET_ALM0_CODE(14);
					SET_ALM1_CODE(14);
					g_eBaseAlm.ALM_CODE14 = 1;
					usAlmLevel = 1;
					g_DsgDisable[14] = 1;
					g_ChgDisable[14] = 1;
					pstPackRVal->ucDsgEn = 0xAA;
					pstPackRVal->ucChgEn = 0xAA;
					fDsgLmtCur = 0;
					fChgLmtCur = 0;
				} else {
					s_ucTick++;
				}
			} else {
				s_ucTick = 0;
			}
		}
//	/* recover: Hi-temper. of MOS 80? */
//	if(pstPackRVal->fMosT < g_stCfg.stLocal.sMOSOTRVThr1 && GET_ALM0_CODE(14)) {
//		 RESET_ALM0_CODE(14);
//		 RESET_ALM1_CODE(14);
//		 usAlmLevel = 0;
//		 g_DsgDisable[14] = 0;
//	}
	/* trigger: Hi-temper. of MOS 88? */
		{
			static uint8_t s_ucTick = 0;
			if(pstPackRVal->fMosT >= g_stCfg.stLocal.sMOSOTTVThr2 && pstPackRVal->fMosT < g_stCfg.stLocal.sMOSOTTVThr1) {
				if(s_ucTick * g_stCfg.stLocal.usCyclePeriod >= g_stCfg.stLocal.usMOSOTTTThr2) {
					SET_ALM0_CODE(14);
					SET_ALM1_CODE(14);
					g_eBaseAlm.ALM_CODE14 = 1;
					fDsgLmtCur = 0;
					fChgLmtCur = 0;
					usAlmLevel = 1;
				} else {
					s_ucTick++;
				}
			} else {
				s_ucTick = 0;
			}
		}
//		{	//delay 10s disconnect discharge MOS.
//			static uint8_t s_ucTick = 0;
//			if(GET_ALM0_CODE(14) && GET_ALM1_CODE(14)) {
//				if(s_ucTick * g_stCfg.stLocal.usCyclePeriod >= 10000) {
//					pstPackRVal->ucDsgEn = 0xAA;
//				} else {
//					s_ucTick++;
//				}
//			} else {
//				s_ucTick = 0;
//			}
//		}
	/* recover: Hi-temper. of MOS 85? */
		if(pstPackRVal->fMosT < g_stCfg.stLocal.sMOSOTRVThr2 && GET_ALM0_CODE(14) && GET_ALM1_CODE(14)) {
			s_ucTick90 = 0;
			RESET_ALM0_CODE(14);
			RESET_ALM1_CODE(14);
			g_eBaseAlm.ALM_CODE14 = 0;
			usAlmLevel = 0;
			g_DsgDisable[14] = 0;
			g_ChgDisable[14] = 0;
			pstPackRVal->ucChgEn = 0x55;
			pstPackRVal->ucDsgEn = 0x55;
		}
	}
//	/* trigger: Hi-temper. of MOS 70? */
//	{
//		static uint8_t s_ucTick = 0;
//		if(pstPackRVal->fMosT > g_stCfg.stLocal.sMOSOTTVThr3 && pstPackRVal->fMosT < g_stCfg.stLocal.sMOSOTTVThr2) {
//			if(s_ucTick * g_stCfg.stLocal.usCyclePeriod >= g_stCfg.stLocal.usMOSOTTTThr3) {
//				g_auiAlmCode1[(64-1)/32]|=((uint32_t)0x00000001 << ((64-1)%32));
//				GET_ALM1_CODE(64);
//				g_eBaseAlm.ALM_CODE64 = 1;
//				if(usAlmLevel != 1) {
//					usAlmLevel = 2;
//				}
//			} else {
//				s_ucTick++;
//			}
//		} else {
//			s_ucTick = 0;
//		}
//	}
//	/* recover: Hi-temper. of MOS 65? */
//	if(pstPackRVal->fMosT < g_stCfg.stLocal.sMOSOTRVThr3 && GET_ALM1_CODE(64)) {
//		RESET_ALM1_CODE(64);
//		g_eBaseAlm.ALM_CODE64 = 0;
//		usAlmLevel = 0;
//	}
	/* trigger: Too Low SOC <1% */
	{
		static uint8_t s_ucTick = 0;
		if(pstPackRVal->fPackSoc < 1 && pstPackRVal->uBaseStat.stBaseStat.ucChg == 0) {
			if(s_ucTick * g_stCfg.stLocal.usCyclePeriod >= 3000) {
				SET_ALM0_CODE(15);
				SET_ALM1_CODE(15);
				g_eBaseAlm.ALM_CODE15 = 1;
				fDsgLmtCur = 0;
			} else {
				s_ucTick++;
			}
		} else {
			s_ucTick = 0;
		}
	}
	/* recover: Too Low SOC <1% */
	if((pstPackRVal->uBaseStat.stBaseStat.ucChg || pstPackRVal->fPackSoc >= 7) && GET_ALM0_CODE(15) && GET_ALM1_CODE(15)) {
		RESET_ALM0_CODE(15);
		RESET_ALM1_CODE(15);
		g_eBaseAlm.ALM_CODE15 = 0;
	}
	/* trigger: Too Low SOC <7% */
	{
		static uint8_t s_ucTick = 0;
		static uint8_t s_ucTick7 = 0;
		static uint16_t s_usKeep = 200;
		if(pstPackRVal->fPackSoc < 7 && pstPackRVal->fPackSoc >= 1) {
			if(s_ucTick * g_stCfg.stLocal.usCyclePeriod >= 3000) {
				if(s_usKeep * g_stCfg.stLocal.usCyclePeriod >= 10000) {
					if(s_ucTick7 == 0) {
						SET_ALM0_CODE(15);
						SET_ALM1_CODE(15);
						s_ucTick7++;
					}
					g_eBaseAlm.ALM_CODE15 = 1;
					usAlmLevel = 1;
				}
			} else {
				s_ucTick++;
			}
		} else {
			s_ucTick = 0;
			s_usKeep = 200;
		}
		if(GET_ALM0_CODE(15)) {
			if(s_usKeep * g_stCfg.stLocal.usCyclePeriod >= 0) {
				if(s_usKeep > 0) {
					s_usKeep --;
				} else {
					s_usKeep = 0;
					s_ucTick7 =0;
					RESET_ALM0_CODE(15);
					RESET_ALM1_CODE(15);
					usAlmLevel = 0;
				}
			}
		} 
	}
		/* recover: Too Low SOC <7% */
	if(pstPackRVal->fPackSoc >= 7 && GET_ALM0_CODE(15) && GET_ALM1_CODE(15)) {
		RESET_ALM0_CODE(15);
		RESET_ALM1_CODE(15);
		g_eBaseAlm.ALM_CODE15 = 0;
		g_DsgDisable[15] = 0;
		usAlmLevel = 0;
	}
	//external communication fault
	if(prl_client() && g_ausPrlComTick[0] == 0) {
		SET_ALM0_CODE(16);
		SET_ALM1_CODE(16);
//		g_auiAlmCode0[(16-1)/32]|=((uint32_t)0x00000001 << ((16-1)%32));
//		GET_ALM0_CODE(16);
//		g_auiAlmCode1[(16-1)/32]|=((uint32_t)0x00000001 << ((16-1)%32));
//		GET_ALM1_CODE(16);
		g_eBaseAlm.ALM_CODE16 = 1;
		usAlmLevel = 1;
	} else {
		RESET_ALM0_CODE(16);
		RESET_ALM1_CODE(16);
		g_eBaseAlm.ALM_CODE16 = 0;
		usAlmLevel = 0;
	}
	//internal communication fault
	//...
	
	/* trigger: cell under temp. heater on */
	if(round(pstPackRVal->fCellTMin) <= g_stCfg.stLocal.sHeaterUTTVThr
		&& round(pstPackRVal->fCellTMax) < g_stCfg.stLocal.sCellOCTTVThr1) {
		g_bNeedHeat = true;
	}
	/* trigger: cell temp. diff. heater off */
	if(round(pstPackRVal->fCellTMax) - round(pstPackRVal->fCellTMin) > 10){
		g_bNeedHeat = false;
	}
	/* recover: cell under temp. heater off or heating count limited */
	if(round(pstPackRVal->fCellTMin) >= g_stCfg.stLocal.sHeaterUTRVThr) {
		g_bNeedHeat = false;
	}
	/* trigger: MOS Abnormality */
//	pstPackRVal->fPackCur = 100;
	if(!pstPackRVal->uBaseStat.stBaseStat.ucHeating) {
		static uint8_t s_ucTick = 0;
			if((pstPackRVal->ucChgEn == 0xAA && pstPackRVal->ucDsgEn == 0xAA && fabs(pstPackRVal->fPackCur) > 1)
				|| (pstPackRVal->ucChgEn == 0xAA && pstPackRVal->fPackCur > 1)
				|| (pstPackRVal->ucDsgEn == 0xAA && pstPackRVal->fPackCur < -1)
				|| (pstPackRVal->ucChgForceEn == 0xAA && pstPackRVal->fPackCur > 1)
				|| (pstPackRVal->ucDsgForceEn == 0xAA && pstPackRVal->fPackCur < -1)
				|| (pstPackRVal->ucChgEn == 0xAA && pstPackRVal->ucDsgForceEn == 0xAA && fabs(pstPackRVal->fPackCur) > 1)
			  || (pstPackRVal->ucDsgEn == 0xAA && pstPackRVal->ucChgForceEn == 0xAA && fabs(pstPackRVal->fPackCur) > 1)
				|| (pstPackRVal->ucChgForceEn == 0xAA && pstPackRVal->ucDsgForceEn == 0xAA && fabs(pstPackRVal->fPackCur) > 1)) {	
			if(s_ucTick * g_stCfg.stLocal.usCyclePeriod >= 3000) {
				SET_ALM0_CODE(1);
				SET_ALM1_CODE(1);
				g_eBaseAlm.ALM_CODE1 = 1;
				g_ChgDisable[1] = 1;
				g_DsgDisable[1] = 1;
				usAlmLevel = 1;
				fDsgLmtCur = 0;
				fChgLmtCur = 0;
				DECOUPLER_H;            //The decoupler starts(二次保护继电器关闭)
			} else {
				s_ucTick++;
			}
		} else {
			s_ucTick = 0;
		}
	}
	pstPackRVal->fLmtChgI = fChgLmtCur;
	pstPackRVal->fLmtDsgI = fDsgLmtCur;
	
	{
		uint16_t usValalm;
		memset(g_aucCanSBuf, 0, sizeof(BASE_ALM_CODE_E));
		for(uint16_t i=0;i<sizeof(BASE_ALM_CODE_E) / 2;i++) {
			usValalm = local_read_reg(g_stPrl.ucSelfId, 1, 0 + i);
			g_aucCanSBuf[i * 2] = usValalm & 0xFF;
			g_aucCanSBuf[i * 2 + 1] = usValalm >> 8;
		}
		if(g_stPrl.ucSelfId > 0 && g_stPrl.ucSelfId < 4 && (g_stLocalArrayRVal.eLocalStat != eLocalStatCanUpgrade)) {
			eco_can_data_send(g_stPrl.ucSelfId, 1, 0x64, 0, g_aucCanSBuf, sizeof(BASE_ALM_CODE_E));
		}
	}
//	if() {
//		eco_can_data_send(g_stPrl.ucSelfId, 1, 0x65, 0, g_aucCanSBuf, sizeof(ECO_RTV_S));
//	}
//	uint8_t a0[60]={0};
//  uint8_t a1[60]={0};
//	for(int i =0; i<60; i++){
//		a0[i]=GET_ALM0_CODE(i);
//		a1[i]=GET_ALM1_CODE(i);
//	}
	/* alarm level process */
	if((GET_ALM0_CODE(1) && GET_ALM1_CODE(1)) || (GET_ALM0_CODE(2) && GET_ALM1_CODE(2))
			|| (GET_ALM0_CODE(3) && GET_ALM1_CODE(3)) || (GET_ALM0_CODE(4) && GET_ALM1_CODE(4))
			|| (GET_ALM0_CODE(5) && GET_ALM1_CODE(5)) || (GET_ALM0_CODE(6) && GET_ALM0_CODE(6))
			|| (GET_ALM0_CODE(7) && GET_ALM1_CODE(7)) || (GET_ALM0_CODE(10) && GET_ALM0_CODE(10))
			|| (GET_ALM0_CODE(11) && GET_ALM1_CODE(11)) || (GET_ALM0_CODE(12) && GET_ALM0_CODE(12))
			|| (GET_ALM0_CODE(13) && GET_ALM1_CODE(13)) || (GET_ALM0_CODE(14) && GET_ALM0_CODE(14))
			|| (GET_ALM0_CODE(15) && GET_ALM1_CODE(15)) || (GET_ALM0_CODE(16) && GET_ALM0_CODE(16))
			|| (GET_ALM0_CODE(17) && GET_ALM1_CODE(17))) {
				g_ucAlmLevel = 1;
				g_ucAlmLevel2 = 3;
				if(GET_ALM0_CODE(12) && GET_ALM0_CODE(12)) {
					g_ucAlmLeve3 = 0;
				}
	} else if(GET_ALM1_CODE(53) || GET_ALM1_CODE(54) || GET_ALM1_CODE(55) || GET_ALM1_CODE(56)
		|| GET_ALM1_CODE(57) || GET_ALM1_CODE(64)) {
			g_ucAlmLevel = 2;
			g_ucAlmLevel2 = 2;
			g_ucAlmLeve3 = 2;
	} else if(GET_ALM1_CODE(80) || GET_ALM1_CODE(81) || GET_ALM1_CODE(82) || GET_ALM1_CODE(83)
		|| GET_ALM1_CODE(84) || GET_ALM1_CODE(85) || GET_ALM1_CODE(86) || GET_ALM1_CODE(87)) {
			g_ucAlmLevel = 3;
			g_ucAlmLevel2 = 1;
			g_ucAlmLeve3 = 3;
	} else {
			g_ucAlmLevel = 0;
			g_ucAlmLevel2 = 0;
			g_ucAlmLeve3 = 0;
	}
	/* ECO special limit */
	if(g_bNeedSleep) {
		pstPackRVal->ucChgEn = 0xAA;
		pstPackRVal->ucDsgEn = 0xAA;
	} else {
		for(uint8_t i=0;i<16;i++) {
			if(g_ChgDisable[i] != 0) {
				pstPackRVal->ucChgEn = 0xAA;
				break;
			}
		}
		for(uint8_t i=0;i<16;i++) {
			if(g_DsgDisable[i]) {
				pstPackRVal->ucDsgEn = 0xAA;
				break;
			}
		}
	}
}

uint16_t local_read_reg(uint8_t ucSelfId, uint8_t ucDevAddr, uint16_t usRegAddr) {
	uint16_t usRet = 0;
	if(usRegAddr < 0xFF) {
		if(usRegAddr * 2 >= sizeof(BASE_ALM_CODE_E)) {	//not enough registers
			return 0;
		}
	}
	if(g_stPrl.ucDevNum == 1) {
			memcpy(&usRet, ((uint8_t*)&g_eBaseAlm) + usRegAddr * sizeof(uint16_t), sizeof(uint16_t));
		} else if(ucSelfId == 0 && ucDevAddr == 0) {
			memcpy(&usRet, ((uint8_t*)&g_eBaseAlm_Parallel[PRL_MAX_NODE_NUM]) + usRegAddr * sizeof(uint16_t), sizeof(uint16_t));
		} else if(ucSelfId < PRL_MAX_NODE_NUM && ucDevAddr == 1) {
			memcpy(&usRet, ((uint8_t*)&g_eBaseAlm_Parallel[ucSelfId]) + usRegAddr * sizeof(uint16_t), sizeof(uint16_t));
		} else {
			return 0;
		}
	return usRet;
}

//void local_fsm(void) {
//	uint8_t ucInput = 0;
//	LOCAL_PACK_RVAL_S* pstPackRVal = g_stLocalArrayRVal.astPackRVal + g_stPrl.ucSelfId;
//	/* P+ & P- voltage valid */
//	if(CHG_WK_50V_READ == SET) {
//		ucInput |= 0x01;
//	}
//	/* charger can frame coming */
//	if(pstPackRVal->uBaseStat.stBaseStat.ucChgerON) {
//		ucInput |= 0x02;
//	}
//	if(pstPackRVal->uBaseStat.stBaseStat.ucChg) {
//		ucInput |= 0x04;
//	}
//	if(g_stAfe.uRam.stCode.CHG_FET) {
//		ucInput |= 0x08;
//	}
//	if(g_stAfe.uRam.stCode.DSG_FET) {
//		ucInput |= 0x10;
//	}
//	if(pstPackRVal->ucHeatEn == 0x55) {
//		ucInput |= 0x20;
//	}
//	static uint8_t s_aucOutput[64] = {
//		0x25,	0x25,	0x29,	0x29,	0x6A,	0x6A,	0x6A,	0x6A,
//		0x25,	0x25,	0x29,	0x29,	0x25,	0x25,	0x29,	0x29,
//		0x25,	0x25,	0x29,	0x29,	0x6A,	0x6A,	0x6A,	0x6A,
//		0x25,	0x25,	0x29,	0x29,	0x25,	0x25,	0x29,	0x29,
//		0x25,	0x1A,	0x1A,	0x1A,	0x6A,	0x6A,	0x6A,	0x6A,
//		0x25,	0x25,	0x1A,	0x1A,	0x25,	0x25,	0x1A,	0x1A,
//		0x25,	0x25,	0x1A,	0x1A,	0x6A, 0x6A, 0x6A, 0x1A,
//		0x25,	0x25,	0x1A, 0x1A,	0x25,	0x25,	0x1A,	0x1A};
//	static uint16_t s_usIllegalCnt = 0;
//	if(s_usIllegalCnt > g_stCfg.stLocal.usCyclePeriod) {
//		s_usIllegalCnt -= g_stCfg.stLocal.usCyclePeriod;
//	} else {
//		s_usIllegalCnt = 0;
//	}
//	if(s_usIllegalCnt > 0) {
//			pstPackRVal->ucChgEn = 0xAA;
//			pstPackRVal->ucDsgEn = 0xAA;
//			pstPackRVal->ucHeatEn = 0xAA;
//			return;
//	}
//	static uint8_t s_ucInput = 0;
//	static uint8_t s_ucTick = 0;
//	if(s_ucInput == ucInput) {
//		if(s_ucTick * g_stCfg.stLocal.usCyclePeriod < 300) {	//state of statement machine keeping more then 300ms
//			s_ucTick++;
//			return;
//		}
//	} else {
//		s_ucInput = ucInput;
//		s_ucTick = 0;
//		return;
//	}
//	static uint16_t s_usDMosOffDelayTick = 0;
//	static uint16_t s_usCMosOffDelayTick = 0;
//	if(s_aucOutput[s_ucInput] & 0x02) {	//when charge MOS should be off
//		if(s_aucOutput[s_ucInput] == 0x1A && pstPackRVal->ucHeatEn == 0x55) {	//when heat should be on
//			if(s_usCMosOffDelayTick * g_stCfg.stLocal.usCyclePeriod < 10000) {
//				s_usCMosOffDelayTick ++;
//				pstPackRVal->ucChgEn = 0x55;
//			} else {
//				pstPackRVal->ucChgEn = 0xAA;
//			}
//		} else {
//			s_usCMosOffDelayTick = 0;
//			pstPackRVal->ucChgEn = 0xAA;
//		}
//	} else {
//		s_usCMosOffDelayTick = 0;
//	}
//	if(s_aucOutput[s_ucInput] & 0x08) {	//when discharge MOS should be off
//		if(s_aucOutput[s_ucInput] == 0x1A && pstPackRVal->ucHeatEn == 0x55) {	//when heat should be on
//			if(s_usDMosOffDelayTick * g_stCfg.stLocal.usCyclePeriod < 15000) {
//				s_usDMosOffDelayTick ++;
//				pstPackRVal->ucDsgEn = 0x55;
//			} else {
//				pstPackRVal->ucDsgEn = 0xAA;
//			}
//		} else {
//			s_usDMosOffDelayTick = 0;
//			pstPackRVal->ucDsgEn = 0xAA;
//		}
//	} else {
//		s_usDMosOffDelayTick = 0;
//	}
//	if(s_aucOutput[s_ucInput] & 0x20) {	//when heater should be off
//		pstPackRVal->ucHeatEn = 0xAA;
//	}
//	if(pstPackRVal->uBaseStat.stBaseStat.ucChgerON && pstPackRVal->ucHeatEn == 0xAA && pstPackRVal->fCellTMin > 0 && pstPackRVal->fCellTMin < 5) {
//		pstPackRVal->ucChgEn = 0x55;
//	}
//	if(s_aucOutput[s_ucInput] & 0x40) {
//		s_usIllegalCnt = 30000;
//	}
//	if(g_stPrl.ucChgEn == 0xAA && prl_client()) {
//		pstPackRVal->ucChgEn = 0xAA;
//	}
//	if(g_stPrl.ucDsgEn == 0xAA && prl_client()) {
//		pstPackRVal->ucDsgEn = 0xAA;
//	}
//	if(g_stPrl.ucHeatEn == 0xAA && prl_client()) {
//		pstPackRVal->ucHeatEn = 0xAA;
//	}
//	if(pstPackRVal->ucHeatEn == 0x55) {	//charge disable when heat requested, and heating request current is 5A
//		pstPackRVal->fReqChgI = 5;
//	}
//	
//	local_mos_ctl();
//}

void local_state_machine(void) {
	/* 1. Using state machine */
	/* 1.0. Init */
	static uint16_t s_usSwTick = 0;		//Charger switching tick
	typedef enum {
		eUStat_I = 1,	//no comm. supported charger connected
		eUStat_II,		//Main charger connected
		eUStat_III,		//Only second charger connected
		eUStat_IV			//charger switching from second one to main one
	} E_USING_STAT;
	static E_USING_STAT s_eUStat = eUStat_I;
	/* 1.1. Input refresh */
	bool abUStatInput[2] = {0};
	abUStatInput[0] = g_bMChgerComAct;
	abUStatInput[1] = g_bSChgerComAct;
	/* 1.2. Logic array process */
#define CHGER_SW_MS	5000
	static bool s_abUStatInput[2] = {0};
	bool abUStatOutput[4] = {0};
	if(!s_abUStatInput[0] && abUStatInput[0]) {	//Main charger comm. connected
		switch(s_eUStat) {
			case eUStat_I:	abUStatOutput[0] = true;	abUStatOutput[3] = true;	break;
			case eUStat_II:	abUStatOutput[0] = true;	abUStatOutput[3] = true;	break;
			case eUStat_III:abUStatOutput[1] = true;	abUStatOutput[3] = true;	break;
			case eUStat_IV:	abUStatOutput[1] = true;	abUStatOutput[3] = true;	break;
			default:	return;
		}
	} else if(s_abUStatInput[0] && !abUStatInput[0]) {	//Main charger comm. disconnected
		switch(s_eUStat) {
			case eUStat_I:	abUStatOutput[1] = true;	abUStatOutput[3] = true;	break;
			case eUStat_II:	abUStatOutput[1] = true;	abUStatOutput[3] = true;	break;
			case eUStat_III:abUStatOutput[1] = true;	abUStatOutput[2] = true;	break;
			case eUStat_IV:	abUStatOutput[1] = true;	abUStatOutput[2] = true;	break;
			default:	return;
		}
	} else if(!s_abUStatInput[1] && abUStatInput[1]) {	//Second charger comm. connected
		switch(s_eUStat) {
			case eUStat_I:	abUStatOutput[1] = true;	abUStatOutput[2] = true;	break;
			case eUStat_II:	abUStatOutput[0] = true;	abUStatOutput[3] = true;	break;
			case eUStat_III:abUStatOutput[1] = true;	abUStatOutput[2] = true;	break;
			case eUStat_IV:	abUStatOutput[1] = true;	abUStatOutput[3] = true;	break;
			default:	return;
		}
	} else if(s_abUStatInput[1] && !abUStatInput[1]) {	//Second charger comm. disconnected
		switch(s_eUStat) {
			case eUStat_I:	abUStatOutput[1] = true;	abUStatOutput[3] = true;	break;
			case eUStat_II:	abUStatOutput[0] = true;	abUStatOutput[3] = true;	break;
			case eUStat_III:abUStatOutput[1] = true;	abUStatOutput[3] = true;	break;
			case eUStat_IV:	abUStatOutput[1] = true;	abUStatOutput[3] = true;	break;
			default:	return;
		}
	} else if(s_usSwTick >= CHGER_SW_MS) {
		s_usSwTick = 0;
		switch(s_eUStat) {
			case eUStat_I:	abUStatOutput[1] = true;	abUStatOutput[3] = true;	break;
			case eUStat_II:	abUStatOutput[0] = true;	abUStatOutput[3] = true;	break;
			case eUStat_III:abUStatOutput[1] = true;	abUStatOutput[2] = true;	break;
			case eUStat_IV:	abUStatOutput[0] = true;	abUStatOutput[3] = true;	break;
			default:	return;
		}
	}
	/* 1.3. Output proccess */
	if(abUStatOutput[0]) {
		g_bSetMChgerAct = true;
	}
	if(abUStatOutput[1]) {
		g_bSetMChgerAct = false;
	}
	if(abUStatOutput[2]) {
		g_bSetSChgerAct = true;
	}
	if(abUStatOutput[3]) {
		g_bSetSChgerAct = false;
		if(s_usSwTick == 0) {
			s_usSwTick += g_stCfg.stLocal.usCyclePeriod;
		}
	}
	/* 1.4. Using state refresh */
	bool abUStat[2] = {0};
	abUStat[0] = g_bMChgerComAct;
	abUStat[1] = g_bSChgerComAct;
	E_USING_STAT eUStat;
	if(!abUStat[0] && !abUStat[1]) {
		eUStat = eUStat_I;
	} else if(abUStat[0] && !abUStat[1]) {
		eUStat = eUStat_II;
	} else if(!abUStat[0] && abUStat[1]) {
		eUStat = eUStat_III;
	} else {
		if(s_usSwTick == 0) {
			eUStat = eUStat_II;
		} else {
			eUStat = eUStat_IV;
		}
	}
	/* 1.5. tick count process */
	if(s_usSwTick > 0 && s_usSwTick < CHGER_SW_MS) {
		s_usSwTick += g_stCfg.stLocal.usCyclePeriod;
	}
	/* 1.6. Input info backup */
	s_abUStatInput[0] = abUStatInput[0];
	s_abUStatInput[1] = abUStatInput[1];
	/* 1.7. If state changed, clear all input event */
	if(eUStat != s_eUStat) {
		s_abUStatInput[0] = 0;
		s_abUStatInput[1] = 0;
	}
	s_eUStat = eUStat;
	
	/* 2. charging & heating state machine */
	/* 2.0. Init */
	LOCAL_PACK_RVAL_S* pstPackRVal = g_stLocalArrayRVal.astPackRVal + g_stPrl.ucSelfId;
	static uint8_t s_ucHeatTick = 0;			//times of heat over
	static uint32_t s_uiHeatingTime = 0;	//continuously heating time, unit: ms
	typedef enum {
		eCStat_I = 1,
		eCStat_II,
		eCStat_III,
		eCStat_IV
	} E_CHGING_STAT;
	static E_CHGING_STAT s_eCStat = eCStat_I;
	/* 2.1. Input refresh */
#define MAX_HEAT_TICK 10
#define MAX_HEAT_TIME	90 * 60 * 1000 //30min -> 60min
	bool abCStatInput[3] = {0};
	static int8_t s_cStep = -1;		//step index of abCStatOutput[10] flow
	if(g_bNeedHeat && s_ucHeatTick < MAX_HEAT_TICK && s_uiHeatingTime < MAX_HEAT_TIME && s_cStep == -1) {
		abCStatInput[0] = true;
	}
	if((!g_bNeedHeat || s_ucHeatTick >= MAX_HEAT_TICK || s_uiHeatingTime >= MAX_HEAT_TIME || !g_bMChgerComAct) && pstPackRVal->ucChgEn == 0x55) { //2024.7.3 ????
		abCStatInput[1] = true;
	}
	if((!g_bNeedHeat || s_ucHeatTick >= MAX_HEAT_TICK || s_uiHeatingTime >= MAX_HEAT_TIME || !g_bMChgerComAct) && pstPackRVal->ucChgEn != 0x55) {
		abCStatInput[2] = true;
	}
	/* 2.2. Logic array process */
	bool abCStatOutput[12] = {0};
	if(abCStatInput[0]) {
		switch(s_eCStat) {
			case eCStat_I:
				abCStatOutput[5] = true;
				abCStatOutput[7] = true;
				abCStatOutput[9] = true;
			break;
			case eCStat_II:
				abCStatOutput[8] = true;
				if(g_bHeatingOverNumberFlag == false) {
					abCStatOutput[10] = true;
				}
			break;
			case eCStat_III:
				abCStatOutput[3] = true;
				abCStatOutput[5] = true;
			break;
			case eCStat_IV:
				abCStatOutput[1] = true;
				abCStatOutput[3] = true;
				abCStatOutput[4] = true;
			break;
			default:
				return;
		}
	} else if(abCStatInput[1]) {
		switch(s_eCStat) {
			case eCStat_I:
				abCStatOutput[5] = true;
				abCStatOutput[7] = true;
				abCStatOutput[9] = true;
			break;
			case eCStat_II:
				abCStatOutput[0] = true;
//				abCStatOutput[3] = true;   //2024.7.2??????????MOS
				abCStatOutput[5] = true;
				abCStatOutput[11] = true;
			break;
			case eCStat_III:
				abCStatOutput[0] = true;
				abCStatOutput[3] = true;
				abCStatOutput[5] = true;
				abCStatOutput[11] = true;
			break;
			case eCStat_IV:
				abCStatOutput[0] = true;
				abCStatOutput[3] = true;
				abCStatOutput[5] = true;
				abCStatOutput[11] = true;
			break;
			default:
				return;
		}
	} else if(abCStatInput[2]) {
		switch(s_eCStat) {
			case eCStat_I:
				abCStatOutput[5] = true;
				abCStatOutput[7] = true;
				abCStatOutput[9] = true;
			break;
			case eCStat_II:
				abCStatOutput[1] = true;
				abCStatOutput[3] = true;
				abCStatOutput[5] = true;
				abCStatOutput[11] = true;
			break;
			case eCStat_III:
				abCStatOutput[1] = true;
				abCStatOutput[3] = true;
				abCStatOutput[5] = true;
				abCStatOutput[11] = true;
			break;
			case eCStat_IV:
				abCStatOutput[1] = true;
				abCStatOutput[3] = true;
				abCStatOutput[5] = true;
				abCStatOutput[11] = true;
			break;
			default:
				return;
		}
	}
	/* 2.3. Output proccess */
	if(abCStatOutput[1]) {
		pstPackRVal->ucChgEn = 0xAA;
	}
	if(abCStatOutput[3]) {
		pstPackRVal->ucDsgEn = 0xAA;
	}
	if(abCStatOutput[5]) {
		pstPackRVal->ucHeatEn = 0xAA;
		s_cStep = -1;
	}
	if(abCStatOutput[6]) {
		if(s_ucHeatTick < MAX_HEAT_TICK) {
			s_ucHeatTick++;
		}
	}
	if(abCStatOutput[7]) {
		s_ucHeatTick = 0;
		g_bHeatingOverNumberFlag = false;
	}
	if(abCStatOutput[8]) {
		if(s_uiHeatingTime == 0) {
			s_uiHeatingTime += g_stCfg.stLocal.usCyclePeriod;
		}
	}
	if(abCStatOutput[9]) {
		s_uiHeatingTime = 0;
	}
//	if((s_cStep == 10 || s_cStep == 20 || g_bMChging) && s_uiHeatingTime < MAX_HEAT_TIME) {
//		pstPackRVal->ucChgEn = 0x55;
//		pstPackRVal->ucDsgEn = 0x55;
//		abCStatOutput[10] = true;
//	}
//	if(s_eCStat == eUStat_I || !g_bNeedHeat) {
//		abCStatInput[0] = false;
//		abCStatOutput[10] = false;
//		pstPackRVal->ucHeatEn = 0xAA;
//		s_cStep = -1;
//	}
	static uint16_t s_usStepTick = 0;
	if(abCStatOutput[10]) {
		if(s_cStep == -1) {
			s_ucHeatTick++;
			s_cStep = 0;
			s_usStepTick = 0;
		}
	}
	if(abCStatOutput[11]) {
		if(s_uiHeatingTime < MAX_HEAT_TIME) {
			s_uiHeatingTime = 0;
		}
	}
#define HMOS_ON_DELAY		1000
#define CMOS_OFF_DELAY	3000
#define DMOS_OFF_DELAY	5000
	switch(s_cStep) {
		case 0:
			pstPackRVal->ucChgEn = 0x55;
			pstPackRVal->ucDsgEn = 0x55;
			pstPackRVal->ucHeatEn = 0xAA;
			s_usStepTick += g_stCfg.stLocal.usCyclePeriod;
			if(s_usStepTick >= HMOS_ON_DELAY) {
				s_cStep = 10;
			}
			break;
		case 10:
			pstPackRVal->ucChgEn = 0x55;
			pstPackRVal->ucDsgEn = 0x55;
			pstPackRVal->ucHeatEn = 0x55;
			g_bTempIsNotRisingFlag = false;
			s_usStepTick += g_stCfg.stLocal.usCyclePeriod;
			if(s_usStepTick >= CMOS_OFF_DELAY) {
				s_cStep = 20;
			}
			break;
		case 20:
			pstPackRVal->ucChgEn = 0xAA;
			pstPackRVal->ucDsgEn = 0x55;
			pstPackRVal->ucHeatEn = 0x55;
			if(s_usStepTick < DMOS_OFF_DELAY) {
				s_usStepTick += g_stCfg.stLocal.usCyclePeriod;
			}
			if(s_usStepTick >= DMOS_OFF_DELAY) {
				s_cStep = 30;
			}
			break;
		case 30:
			pstPackRVal->ucChgEn = 0xAA;
			if(pstPackRVal->fCellTMin > 2) {
				pstPackRVal->ucChgEn = 0x55;
			}
			pstPackRVal->ucDsgEn = 0xAA;
			pstPackRVal->ucHeatEn = 0x55;
		default:
			break;
	}
	/* 2.4. Using state refresh */
	bool abCStat[3] = {0};
	if(s_eUStat == eUStat_I) {
		abCStat[0] = true;
	}
	if(pstPackRVal->uBaseStat.stBaseStat.ucHeating != 0) {
		abCStat[1] = true;
	}
	if(pstPackRVal->uBaseStat.stBaseStat.ucHeating == 0 && !g_bMChgerComAct && s_cStep == -1) {
//	if(pstPackRVal->uBaseStat.stBaseStat.ucHeating == 0 && !g_bMChgerComAct && s_uiHeatingTime >= MAX_HEAT_TIME && s_cStep == -1) {
		g_bHeatingTimeoutFlag = false;
		s_uiHeatingTime = 0;
	}
	if(s_ucHeatTick >= MAX_HEAT_TICK){
		if(g_bHeatingOverNumberFlag == false) {
			g_bHeatingOverNumberFlag = true;
			eco_refresh_RTV();
			eco_refresh_CD();
			eco_refresh_log();
			g_eBaseStat = eBStatHeatingOverNumber;
			his_data_write();
			his_log_write();
		}
		abCStat[2] = true;
	}
	if(s_uiHeatingTime >= MAX_HEAT_TIME) {
		if(g_bHeatingTimeoutFlag == false) {
			g_bHeatingTimeoutFlag = true;
			eco_refresh_RTV();
			eco_refresh_CD();
			eco_refresh_log();
			g_eBaseStat = eBStatHeatingTimeout;
			his_data_write();
			his_log_write();
		}
		abCStat[2] = true;
	}
	if(abCStat[0]) {
		s_eCStat = eCStat_I;
	}
	if(!abCStat[0] && !abCStat[1] && !abCStat[2]) {
		s_eCStat = eCStat_II;
	}
	if(!abCStat[0] && abCStat[2]) {
		s_eCStat = eCStat_III;
	}
	if(!abCStat[0] && abCStat[1] && !abCStat[2]) {
		s_eCStat = eCStat_IV;
	}
	/* 2.5. tick count process */
	if(s_uiHeatingTime > 0 && s_uiHeatingTime < MAX_HEAT_TIME) {
		s_uiHeatingTime += g_stCfg.stLocal.usCyclePeriod;
	}
	g_stEcoRtv.usHeatCnt = s_ucHeatTick;
	g_stEcoRtv.usHeatLastTime = s_uiHeatingTime / 1000;
}

/* Charge and discharge MOS on-off update */
void local_mos_ctl(void) {
	LOCAL_PACK_RVAL_S* pstPackRVal = g_stLocalArrayRVal.astPackRVal + g_stPrl.ucSelfId;
	float fTrue = 1;
	float fFalse = 0;
	if(((pstPackRVal->ucChgEn == 0x55 && pstPackRVal->ucChgForceEn == 0) || pstPackRVal->ucChgForceEn == 0x55) && g_stAfe.uRam.stCode.CHG_FET == 0) {
		afe_set_ao(eAfeRamCodeCHGMOS, 1, &fTrue);
	}
	if((pstPackRVal->ucChgEn == 0xAA || pstPackRVal->ucChgForceEn == 0xAA) && g_stAfe.uRam.stCode.CHG_FET != 0) {
		afe_set_ao(eAfeRamCodeCHGMOS, 1, &fFalse);
	}
	if(((pstPackRVal->ucDsgEn == 0x55 && pstPackRVal->ucDsgForceEn == 0)|| pstPackRVal->ucDsgForceEn == 0x55) && g_stAfe.uRam.stCode.DSG_FET == 0) {
		static uint8_t ucCnt = 0;
		if(ucCnt * g_stCfg.stLocal.usCyclePeriod < 1000) {
			PRE_DSG_H;
			ucCnt++;
		} else {
			ucCnt = 0;
			afe_set_ao(eAfeRamCodeDSGMOS, 1, &fTrue);
		}
	}
	if((pstPackRVal->ucDsgEn == 0xAA || pstPackRVal->ucDsgForceEn == 0xAA) && g_stAfe.uRam.stCode.DSG_FET != 0) {
		PRE_DSG_L;
		afe_set_ao(eAfeRamCodeDSGMOS, 1, &fFalse);
	}
	if(pstPackRVal->ucDsgEn == 0xAA || g_stAfe.uRam.stCode.DSG_FET != 0) {
		PRE_DSG_L;
	}
//	{
//		static uint16_t s_tick = 0;
//		if(s_tick * g_stCfg.stLocal.usCyclePeriod < 3000) {
//			s_tick++;
//			pstPackRVal->ucHeatEn = 0x55;
//		}
//	}
	/* heater on/off refresh */
	if((pstPackRVal->ucHeatEn == 0x55 || pstPackRVal->ucHeatForceEn) && pstPackRVal->uBaseStat.stBaseStat.ucHeating == 0) {
		HEAT_RELAY_H;
		pstPackRVal->uBaseStat.stBaseStat.ucHeating = 1;
	}
	if((pstPackRVal->ucHeatEn == 0xAA && pstPackRVal->ucHeatForceEn == 0) && pstPackRVal->uBaseStat.stBaseStat.ucHeating == 1) {
		HEAT_RELAY_L;
		pstPackRVal->uBaseStat.stBaseStat.ucHeating = 0;
		g_bNeedHeat = false;
	}
	/* Heating rules */
	uint8_t g_ucExtrinsicFlag = g_eBaseStat;
	{	
		if(g_ucHeatingStat != eBStatHeatingEnd) {
			g_eBaseStat = g_ucHeatingStat;
		}
		if(g_bNeedHeat && (g_bMChgerComAct || g_bSChgerComAct) && !g_bMChging) {
			if(g_eBaseStat != eBStatHeatingWaiting) {
				g_eBaseStat = eBStatHeatingWaiting;
				his_data_write();
			}
		}
		if((g_bMChgerComAct || g_bSChgerComAct) && pstPackRVal->uBaseStat.stBaseStat.ucHeating == 1 && (g_bMChging || g_bSChging)) {
			if(g_eBaseStat != eBStatHeating) {
				g_eBaseStat = eBStatHeating;
				his_data_write();
			}
		}
		if(!(g_bMChgerComAct || g_bSChgerComAct) || !g_bNeedHeat) {
			if(g_eBaseStat != eBStatHeatingEnd && (g_ucHeatingStat == eBStatHeatingWaiting || g_ucHeatingStat == eBStatHeating)) {
				g_eBaseStat = eBStatHeatingEnd;
				his_data_write();
			}
		}
	}
	if(g_eBaseStat == eBStatHeatingWaiting /*|| g_eBaseStat == eBStatHeating || g_eBaseStat == eBStatHeatingEnd*/) {
		g_ucHeatingStat = g_eBaseStat;
		g_eBaseStat = g_ucExtrinsicFlag;
	}	else if(g_eBaseStat == eBStatHeating || g_eBaseStat == eBStatHeatingEnd) {
		g_ucHeatingStat = g_eBaseStat;
	} else { 
		g_ucHeatingStat = g_ucExtrinsicFlag;
		g_eBaseStat = g_ucExtrinsicFlag;
	}
	/* Heating failure judgment */
	{
		float afCellTRec[CFG_TMP_NUM] = {0};
		static uint16_t s_ucTick = 0;
		if(s_ucTick == 0) {
			for(uint8_t i = 0; i < CFG_TMP_NUM; i++) {
				afCellTRec[i] = pstPackRVal->afCellT[i];
			}
		}
		if((g_bMChgerComAct || g_bSChgerComAct) && pstPackRVal->uBaseStat.stBaseStat.ucHeating == 1 && (g_bMChging || g_bSChging)) {
			if(s_ucTick * g_stCfg.stLocal.usCyclePeriod >= 15 * 60 * 1000) {
				for(uint8_t i = 0; i < CFG_TMP_NUM; i++) {
					if(pstPackRVal->afCellT[i] - afCellTRec[i] <= 2) {
						if(g_bTempIsNotRisingFlag == false) {
							g_bTempIsNotRisingFlag = true;
							eco_refresh_RTV();
							eco_refresh_CD();
							eco_refresh_log();
							g_eBaseStat = eBStatTempIsNotRising;
							his_data_write();
							his_log_write();
						}
						g_bTempIsNotRisingFlag = false;
						s_ucTick = 0;
						break;
					}
				}
			} else {
				s_ucTick++;
			}
		}
	}
}

void local_cur_lmt(float* pfLmtChgI, float* pfLmtDsgI, float* pfReqChgI, float* pfPeakLmtDsgI) {
	int8_t cRow;
	uint8_t ucCol;
	LOCAL_PACK_RVAL_S* pstPackRVal = g_stLocalArrayRVal.astPackRVal + g_stPrl.ucSelfId;
	/* charge request, min. temp. cell */
	cRow = -1;
	ucCol = 0;
	for(uint8_t i=0;i<LOCAL_CR_ROWS;i++) {
		if(pstPackRVal->fCellTMin >= g_stLocalCRPara.afTemp[i] && pstPackRVal->fCellTMin < g_stLocalCRPara.afTemp[i + 1]) {
			cRow = i;
			break;
		}
	}
	for(uint8_t i=0;i<LOCAL_CR_COLUMNS;i++) {
		if(pstPackRVal->fPackSoc >= g_stLocalCRPara.afSoc[i] && pstPackRVal->fPackSoc < g_stLocalCRPara.afSoc[i + 1]) {
			ucCol = i;
			break;
		}
	}
	if(cRow < 0) {
		*pfReqChgI = 0;
	} else if(*pfReqChgI > g_stLocalCRPara.aafChgRate[cRow][ucCol] * g_stCfg.stLocal.usDesignAH) {
		*pfReqChgI = g_stLocalCRPara.aafChgRate[cRow][ucCol] * g_stCfg.stLocal.usDesignAH;
	}
	/* charge request, max. temp. cell */
	cRow = -1;
	ucCol = 0;
	for(uint8_t i=0;i<LOCAL_CR_ROWS;i++) {
		if(pstPackRVal->fCellTMax >= g_stLocalCRPara.afTemp[i] && pstPackRVal->fCellTMax < g_stLocalCRPara.afTemp[i + 1]) {
			cRow = i;
			break;
		}
	}
	for(uint8_t i=0;i<LOCAL_CR_COLUMNS;i++) {
		if(pstPackRVal->fPackSoc >= g_stLocalCRPara.afSoc[i] && pstPackRVal->fPackSoc < g_stLocalCRPara.afSoc[i + 1]) {
			ucCol = i;
			break;
		}
	}
	if(cRow < 0) {
		*pfReqChgI = 0;
	} else if(*pfReqChgI > g_stLocalCRPara.aafChgRate[cRow][ucCol] * g_stCfg.stLocal.usDesignAH) {
		*pfReqChgI = g_stLocalCRPara.aafChgRate[cRow][ucCol] * g_stCfg.stLocal.usDesignAH;
	}
	/* charge limit */
	float pfLmtChgI1;
	float pfLmtChgI2;
	float pfLmtChgI3;
	float pfLmtChgI4;
	if(pstPackRVal->fCellTMin < g_stLocalCLPara.afTempMin[6] && pstPackRVal->fCellUMax * 1000 > g_stLocalCLPara.afCellVMax[5]) {
		cRow = -1;
		ucCol = 0;
		for(uint8_t i = 1; i < LOCAL_CL_ROWS1 + 1; i++) {
			if(pstPackRVal->fCellTMin < g_stLocalCLPara.afTempMin[i] && pstPackRVal->fCellTMin >= g_stLocalCLPara.afTempMin[i - 1]) {
				cRow = i;
				break;
			}
			if(pstPackRVal->fCellTMin <= g_stLocalCLPara.afTempMin[0]) {
				cRow = 0;
				break;
			}
		}
		for(uint8_t i = 1; i < LOCAL_CL_COLUMNS1; i++) {
			if(pstPackRVal->fCellUMax * 1000 > g_stLocalCLPara.afCellVMax[i] && pstPackRVal->fCellUMax * 1000 <= g_stLocalCLPara.afCellVMax[i - 1]) {
				ucCol = i;
				break;
			}
			if(pstPackRVal->fCellUMax * 1000 > g_stLocalCLPara.afCellVMax[0]) {
				ucCol = 0;
				break;
			}
		}
		if(cRow < 0) {
			pfLmtChgI1 = 0;
		} else if(*pfLmtChgI >= g_stLocalCLPara.aafChgRate1[cRow][ucCol] * g_stCfg.stLocal.usDesignAH) {
			pfLmtChgI1 = g_stLocalCLPara.aafChgRate1[cRow][ucCol] * g_stCfg.stLocal.usDesignAH;
		}
	}
	if(pstPackRVal->fCellTMin < g_stLocalCLPara.afTempMin[6] && pstPackRVal->fCellUMax * 1000 <= g_stLocalCLPara.afCellVMax[5]) {
		cRow = -1;
		ucCol = 0;
		for(uint8_t i = 1; i < LOCAL_CL_ROWS1 + 1; i++) {
			if(pstPackRVal->fCellTMin <= g_stLocalCLPara.afTempMin[i] && pstPackRVal->fCellTMin > g_stLocalCLPara.afTempMin[i - 1]) {
				cRow = i;
				break;
			}
			if(pstPackRVal->fCellTMin <= g_stLocalCLPara.afTempMin[0]) {
				cRow = 0;
				break;
			}
//			if(pstPackRVal->fCellTMin > g_stLocalCLPara.afTempMin[6]) {
//				cRow = 6;
//				break;
//			}
		}
		for(uint8_t i = 0; i < LOCAL_CL_COLUMNS2; i++) {
			if(i == 0) {
				if(pstPackRVal->fCellUMin * 1000 > g_stLocalCLPara.afCellVMin[i] && pstPackRVal->fCellUMin * 1000 <= g_stLocalCLPara.afCellVMax[5]) {
					ucCol = i;
					break;
				}
			} else {
				if(pstPackRVal->fCellUMin * 1000 > g_stLocalCLPara.afCellVMin[i] && pstPackRVal->fCellUMin * 1000 <= g_stLocalCLPara.afCellVMin[i - 1]) {
					ucCol = i;
					break;
				}
			}
			if(pstPackRVal->fCellUMin * 1000 <= g_stLocalCLPara.afCellVMin[7]) {
				ucCol = 8;
				break;
			}
		}
		if(cRow < 0) {
			pfLmtChgI2 = 0;
		} else if(*pfLmtChgI >= g_stLocalCLPara.aafChgRate2[cRow][ucCol] * g_stCfg.stLocal.usDesignAH) {
			pfLmtChgI2 = g_stLocalCLPara.aafChgRate2[cRow][ucCol] * g_stCfg.stLocal.usDesignAH;
		}
	}
	if(pstPackRVal->fCellTMin >= g_stLocalCLPara.afTempMin[6] && pstPackRVal->fCellUMax * 1000 > g_stLocalCLPara.afCellVMax[5]) {
		cRow = -1;
		ucCol = 0;
		for(uint8_t i = 0; i < LOCAL_CL_ROWS2 - 1; i++) {
			if(i == 0) {
				if(pstPackRVal->fCellTMax < g_stLocalCLPara.afTempMax[i] && pstPackRVal->fCellTMax >= g_stLocalCLPara.afTempMin[6]) {
					cRow = i;
					break;
				}
			} else {
				if(pstPackRVal->fCellTMax < g_stLocalCLPara.afTempMax[i] && pstPackRVal->fCellTMax >= g_stLocalCLPara.afTempMax[i - 1]) {
					cRow = i;
					break;
				}
			}
			if(pstPackRVal->fCellTMax >= g_stLocalCLPara.afTempMax[4]) {
				cRow = 5;
				break;
			}
		}
		for(uint8_t i = 1; i < LOCAL_CL_COLUMNS1; i++) {
			if(pstPackRVal->fCellUMax * 1000 > g_stLocalCLPara.afCellVMax[i] && pstPackRVal->fCellUMax * 1000 <= g_stLocalCLPara.afCellVMax[i - 1]) {
				ucCol = i;
				break;
			}
			if(pstPackRVal->fCellUMax * 1000 > g_stLocalCLPara.afCellVMax[0]) {
				ucCol = 0;
				break;
			}
		}
		if(cRow < 0) {
			pfLmtChgI3 = 0;
		} else if(*pfLmtChgI >= g_stLocalCLPara.aafChgRate3[cRow][ucCol] * g_stCfg.stLocal.usDesignAH) {
			pfLmtChgI3 = g_stLocalCLPara.aafChgRate3[cRow][ucCol] * g_stCfg.stLocal.usDesignAH;
		}
	}
	if(pstPackRVal->fCellTMin >= g_stLocalCLPara.afTempMin[6] && pstPackRVal->fCellUMax * 1000 <= g_stLocalCLPara.afCellVMax[5]) {
		cRow = -1;
		ucCol = 0;
		for(uint8_t i = 0; i < LOCAL_CL_ROWS2 - 1; i++) {
			if(i == 0) {
				if(pstPackRVal->fCellTMax < g_stLocalCLPara.afTempMax[i] && pstPackRVal->fCellTMax >= g_stLocalCLPara.afTempMin[6]) {
					cRow = i;
					break;
				}
			} else {
				if(pstPackRVal->fCellTMax < g_stLocalCLPara.afTempMax[i] && pstPackRVal->fCellTMax >= g_stLocalCLPara.afTempMax[i - 1]) {
					cRow = i;
					break;
				}
			}
			if(pstPackRVal->fCellTMax >= g_stLocalCLPara.afTempMax[4]) {
				cRow = 5;
				break;
			}
		}
		for(uint8_t i = 0; i < LOCAL_CL_COLUMNS2; i++) {
			if(i == 0) {
				if(pstPackRVal->fCellUMin * 1000 > g_stLocalCLPara.afCellVMin[i] && pstPackRVal->fCellUMin * 1000 <= g_stLocalCLPara.afCellVMax[5]) {
					ucCol = i;
					break;
				}
			} else {
				if(pstPackRVal->fCellUMin * 1000 > g_stLocalCLPara.afCellVMin[i] && pstPackRVal->fCellUMin * 1000 <= g_stLocalCLPara.afCellVMin[i - 1]) {
					ucCol = i;
					break;
				}
			}
			if(pstPackRVal->fCellUMin * 1000 <= g_stLocalCLPara.afCellVMin[7]) {
				ucCol = 8;
				break;
			}
		}
		if(cRow < 0) {
			pfLmtChgI4 = 0;
		} else if(*pfLmtChgI >= g_stLocalCLPara.aafChgRate4[cRow][ucCol] * g_stCfg.stLocal.usDesignAH) {
			pfLmtChgI4 = g_stLocalCLPara.aafChgRate4[cRow][ucCol] * g_stCfg.stLocal.usDesignAH;
		}
	}
	if((pstPackRVal->fCellTMin < g_stLocalCLPara.afTempMin[6] && pstPackRVal->fCellUMax * 1000 > g_stLocalCLPara.afCellVMax[5])
		&& (pstPackRVal->fCellTMin < g_stLocalCLPara.afTempMin[6] && pstPackRVal->fCellUMax * 1000 <= g_stLocalCLPara.afCellVMax[5])
		&& (pstPackRVal->fCellTMin >= g_stLocalCLPara.afTempMin[6] && pstPackRVal->fCellUMax * 1000 > g_stLocalCLPara.afCellVMax[5])
		&& (pstPackRVal->fCellTMin >= g_stLocalCLPara.afTempMin[6] && pstPackRVal->fCellUMax * 1000 <= g_stLocalCLPara.afCellVMax[5])) {
		if(pfLmtChgI1 < pfLmtChgI2) {
		*pfLmtChgI = pfLmtChgI1;
		} else {
			*pfLmtChgI = pfLmtChgI2;
		}
		if(*pfLmtChgI > pfLmtChgI3) {
			*pfLmtChgI = pfLmtChgI3;
		}
		if(*pfLmtChgI > pfLmtChgI4) {
			*pfLmtChgI = pfLmtChgI4;
		}
	} else if((pstPackRVal->fCellTMin < g_stLocalCLPara.afTempMin[6] && pstPackRVal->fCellUMax * 1000 <= g_stLocalCLPara.afCellVMax[5])
		&& (pstPackRVal->fCellTMin >= g_stLocalCLPara.afTempMin[6] && pstPackRVal->fCellUMax * 1000 > g_stLocalCLPara.afCellVMax[5])
		&& (pstPackRVal->fCellTMin >= g_stLocalCLPara.afTempMin[6] && pstPackRVal->fCellUMax * 1000 <= g_stLocalCLPara.afCellVMax[5])) {
		if(pfLmtChgI2 < pfLmtChgI3) {
		*pfLmtChgI = pfLmtChgI2;
		} else {
			*pfLmtChgI = pfLmtChgI3;
		}
		if(*pfLmtChgI > pfLmtChgI4) {
			*pfLmtChgI = pfLmtChgI4;
		}
	} else if((pstPackRVal->fCellTMin < g_stLocalCLPara.afTempMin[6] && pstPackRVal->fCellUMax * 1000 > g_stLocalCLPara.afCellVMax[5])
		&& (pstPackRVal->fCellTMin >= g_stLocalCLPara.afTempMin[6] && pstPackRVal->fCellUMax * 1000 > g_stLocalCLPara.afCellVMax[5])
		&& (pstPackRVal->fCellTMin >= g_stLocalCLPara.afTempMin[6] && pstPackRVal->fCellUMax * 1000 <= g_stLocalCLPara.afCellVMax[5])) {
		if(pfLmtChgI1 < pfLmtChgI3) {
		*pfLmtChgI = pfLmtChgI1;
		} else {
			*pfLmtChgI = pfLmtChgI3;
		}
		if(*pfLmtChgI > pfLmtChgI4) {
			*pfLmtChgI = pfLmtChgI4;
		}
	} else if((pstPackRVal->fCellTMin < g_stLocalCLPara.afTempMin[6] && pstPackRVal->fCellUMax * 1000 > g_stLocalCLPara.afCellVMax[5])
		&& (pstPackRVal->fCellTMin < g_stLocalCLPara.afTempMin[6] && pstPackRVal->fCellUMax * 1000 <= g_stLocalCLPara.afCellVMax[5])
		&& (pstPackRVal->fCellTMin >= g_stLocalCLPara.afTempMin[6] && pstPackRVal->fCellUMax * 1000 <= g_stLocalCLPara.afCellVMax[5])) {
		if(pfLmtChgI1 < pfLmtChgI2) {
		*pfLmtChgI = pfLmtChgI1;
		} else {
			*pfLmtChgI = pfLmtChgI2;
		}
		if(*pfLmtChgI > pfLmtChgI4) {
			*pfLmtChgI = pfLmtChgI4;
		}
	} else if((pstPackRVal->fCellTMin < g_stLocalCLPara.afTempMin[6] && pstPackRVal->fCellUMax * 1000 > g_stLocalCLPara.afCellVMax[5])
		&& (pstPackRVal->fCellTMin < g_stLocalCLPara.afTempMin[6] && pstPackRVal->fCellUMax * 1000 <= g_stLocalCLPara.afCellVMax[5])
		&& (pstPackRVal->fCellTMin >= g_stLocalCLPara.afTempMin[6] && pstPackRVal->fCellUMax * 1000 > g_stLocalCLPara.afCellVMax[5])) {
		if(pfLmtChgI1 < pfLmtChgI2) {
		*pfLmtChgI = pfLmtChgI1;
		} else {
			*pfLmtChgI = pfLmtChgI2;
		}
		if(*pfLmtChgI > pfLmtChgI3) {
			*pfLmtChgI = pfLmtChgI3;
		}
	} else if((pstPackRVal->fCellTMin >= g_stLocalCLPara.afTempMin[6] && pstPackRVal->fCellUMax * 1000 > g_stLocalCLPara.afCellVMax[5])
		&& (pstPackRVal->fCellTMin >= g_stLocalCLPara.afTempMin[6] && pstPackRVal->fCellUMax * 1000 <= g_stLocalCLPara.afCellVMax[5])) {
		if(pfLmtChgI3 < pfLmtChgI4) {
		*pfLmtChgI = pfLmtChgI3;
		} else {
			*pfLmtChgI = pfLmtChgI4;
		}
	} else if((pstPackRVal->fCellTMin < g_stLocalCLPara.afTempMin[6] && pstPackRVal->fCellUMax * 1000 <= g_stLocalCLPara.afCellVMax[5])
		&& (pstPackRVal->fCellTMin >= g_stLocalCLPara.afTempMin[6] && pstPackRVal->fCellUMax * 1000 <= g_stLocalCLPara.afCellVMax[5])) {
		if(pfLmtChgI2 < pfLmtChgI4) {
		*pfLmtChgI = pfLmtChgI2;
		} else {
			*pfLmtChgI = pfLmtChgI4;
		}
	} else if((pstPackRVal->fCellTMin < g_stLocalCLPara.afTempMin[6] && pstPackRVal->fCellUMax * 1000 <= g_stLocalCLPara.afCellVMax[5])
		&& (pstPackRVal->fCellTMin >= g_stLocalCLPara.afTempMin[6] && pstPackRVal->fCellUMax * 1000 > g_stLocalCLPara.afCellVMax[5])) {
		if(pfLmtChgI2 < pfLmtChgI3) {
		*pfLmtChgI = pfLmtChgI2;
		} else {
			*pfLmtChgI = pfLmtChgI3;
		}
	} else if((pstPackRVal->fCellTMin < g_stLocalCLPara.afTempMin[6] && pstPackRVal->fCellUMax * 1000 > g_stLocalCLPara.afCellVMax[5])
		&& (pstPackRVal->fCellTMin >= g_stLocalCLPara.afTempMin[6] && pstPackRVal->fCellUMax * 1000 <= g_stLocalCLPara.afCellVMax[5])) {
		if(pfLmtChgI1 < pfLmtChgI4) {
		*pfLmtChgI = pfLmtChgI1;
		} else {
			*pfLmtChgI = pfLmtChgI4;
		}
	} else if((pstPackRVal->fCellTMin < g_stLocalCLPara.afTempMin[6] && pstPackRVal->fCellUMax * 1000 > g_stLocalCLPara.afCellVMax[5])
		&& (pstPackRVal->fCellTMin >= g_stLocalCLPara.afTempMin[6] && pstPackRVal->fCellUMax * 1000 > g_stLocalCLPara.afCellVMax[5])) {
		if(pfLmtChgI1 < pfLmtChgI3) {
		*pfLmtChgI = pfLmtChgI1;
		} else {
			*pfLmtChgI = pfLmtChgI3;
		}
	} else if((pstPackRVal->fCellTMin < g_stLocalCLPara.afTempMin[6] && pstPackRVal->fCellUMax * 1000 > g_stLocalCLPara.afCellVMax[5])
		&& (pstPackRVal->fCellTMin < g_stLocalCLPara.afTempMin[6] && pstPackRVal->fCellUMax * 1000 <= g_stLocalCLPara.afCellVMax[5])) {
		if(pfLmtChgI1 < pfLmtChgI2) {
		*pfLmtChgI = pfLmtChgI1;
		} else {
			*pfLmtChgI = pfLmtChgI2;
		}
	} else if(pstPackRVal->fCellTMin >= g_stLocalCLPara.afTempMin[6] && pstPackRVal->fCellUMax * 1000 <= g_stLocalCLPara.afCellVMax[5]) {
		*pfLmtChgI = pfLmtChgI4;
	} else if(pstPackRVal->fCellTMin >= g_stLocalCLPara.afTempMin[6] && pstPackRVal->fCellUMax * 1000 > g_stLocalCLPara.afCellVMax[5]) {
		*pfLmtChgI = pfLmtChgI3;
	} else if(pstPackRVal->fCellTMin < g_stLocalCLPara.afTempMin[6] && pstPackRVal->fCellUMax * 1000 <= g_stLocalCLPara.afCellVMax[5]) {
		*pfLmtChgI = pfLmtChgI2;
	} else if(pstPackRVal->fCellTMin < g_stLocalCLPara.afTempMin[6] && pstPackRVal->fCellUMax * 1000 > g_stLocalCLPara.afCellVMax[5]) {
		*pfLmtChgI = pfLmtChgI1;
	}
	
	/* Discharge limit */
	float pfLmtDsgI1;
	float pfLmtDsgI2;
	float pfLmtDsgI3;
	float pfLmtDsgI4;
	if(pstPackRVal->fCellTMin < g_stLocalDLPara.afTempMin[6] && pstPackRVal->fCellUMax * 1000 > g_stLocalDLPara.afCellVMax[5]) {
		cRow = -1;
		ucCol = 0;
		for(uint8_t i = 1; i < LOCAL_DL_ROWS1 + 1; i++) {
			if(pstPackRVal->fCellTMin < g_stLocalDLPara.afTempMin[i] && pstPackRVal->fCellTMin >= g_stLocalDLPara.afTempMin[i - 1]) {
				cRow = i;
				break;
			}
			if(pstPackRVal->fCellTMin <= g_stLocalDLPara.afTempMin[0]) {
				cRow = 0;
				break;
			}
		}
		for(uint8_t i = 1; i < LOCAL_DL_COLUMNS1; i++) {
			if(pstPackRVal->fCellUMax * 1000 > g_stLocalDLPara.afCellVMax[i] && pstPackRVal->fCellUMax * 1000 <= g_stLocalDLPara.afCellVMax[i - 1]) {
				ucCol = i;
				break;
			}
			if(pstPackRVal->fCellUMax * 1000 > g_stLocalDLPara.afCellVMax[0]) {
				ucCol = 0;
				break;
			}
		}
		if(cRow < 0) {
			pfLmtDsgI1 = 0;
		} else if(*pfLmtDsgI >= g_stLocalDLPara.aafDsgRate1[cRow][ucCol] * g_stCfg.stLocal.usDesignAH) {
			pfLmtDsgI1 = g_stLocalDLPara.aafDsgRate1[cRow][ucCol] * g_stCfg.stLocal.usDesignAH;
		}
	}
	if(pstPackRVal->fCellTMin < g_stLocalDLPara.afTempMin[6] && pstPackRVal->fCellUMax * 1000 <= g_stLocalDLPara.afCellVMax[5]) {
		cRow = -1;
		ucCol = 0;
		for(uint8_t i = 1; i < LOCAL_DL_ROWS1 + 1; i++) {
			if(pstPackRVal->fCellTMin <= g_stLocalDLPara.afTempMin[i] && pstPackRVal->fCellTMin > g_stLocalDLPara.afTempMin[i - 1]) {
				cRow = i;
				break;
			}
			if(pstPackRVal->fCellTMin <= g_stLocalDLPara.afTempMin[0]) {
				cRow = 0;
				break;
			}
//			if(pstPackRVal->fCellTMin > g_stLocalDLPara.afTempMin[6]) {
//				cRow = 6;
//				break;
//			}
		}
		for(uint8_t i = 0; i < LOCAL_DL_COLUMNS2; i++) {
			if(i == 0) {
				if(pstPackRVal->fCellUMin * 1000 > g_stLocalDLPara.afCellVMin[i] && pstPackRVal->fCellUMin * 1000 <= g_stLocalDLPara.afCellVMax[5]) {
					ucCol = i;
					break;
				}
			} else {
				if(pstPackRVal->fCellUMin * 1000 > g_stLocalDLPara.afCellVMin[i] && pstPackRVal->fCellUMin * 1000 <= g_stLocalDLPara.afCellVMin[i - 1]) {
					ucCol = i;
					break;
				}
			}
			if(pstPackRVal->fCellUMin * 1000 <= g_stLocalDLPara.afCellVMin[7]) {
				ucCol = 8;
				break;
			}
		}
		if(cRow < 0) {
			pfLmtDsgI2 = 0;
		} else if(*pfLmtDsgI >= g_stLocalDLPara.aafDsgRate2[cRow][ucCol] * g_stCfg.stLocal.usDesignAH) {
			pfLmtDsgI2 = g_stLocalDLPara.aafDsgRate2[cRow][ucCol] * g_stCfg.stLocal.usDesignAH;
		}
	}
	if(pstPackRVal->fCellTMin >= g_stLocalDLPara.afTempMin[6] && pstPackRVal->fCellUMax * 1000 > g_stLocalDLPara.afCellVMax[5]) {
		cRow = -1;
		ucCol = 0;
		for(uint8_t i = 0; i < LOCAL_DL_ROWS2 - 1; i++) {
			if(i == 0) {
				if(pstPackRVal->fCellTMax < g_stLocalDLPara.afTempMax[i] && pstPackRVal->fCellTMax >= g_stLocalDLPara.afTempMin[6]) {
					cRow = i;
					break;
				}
			} else {
				if(pstPackRVal->fCellTMax < g_stLocalDLPara.afTempMax[i] && pstPackRVal->fCellTMax >= g_stLocalDLPara.afTempMax[i - 1]) {
					cRow = i;
					break;
				}
			}
			if(pstPackRVal->fCellTMax >= g_stLocalDLPara.afTempMax[4]) {
				cRow = 5;
				break;
			}
		}
		for(uint8_t i = 1; i < LOCAL_DL_COLUMNS1; i++) {
			if(pstPackRVal->fCellUMax * 1000 > g_stLocalDLPara.afCellVMax[i] && pstPackRVal->fCellUMax * 1000 <= g_stLocalDLPara.afCellVMax[i - 1]) {
				ucCol = i;
				break;
			}
			if(pstPackRVal->fCellUMax * 1000 > g_stLocalDLPara.afCellVMax[0]) {
				ucCol = 0;
				break;
			}
		}
		if(cRow < 0) {
			pfLmtDsgI3 = 0;
		} else if(*pfLmtDsgI >= g_stLocalDLPara.aafDsgRate3[cRow][ucCol] * g_stCfg.stLocal.usDesignAH) {
			pfLmtDsgI3 = g_stLocalDLPara.aafDsgRate3[cRow][ucCol] * g_stCfg.stLocal.usDesignAH;
		}
	}
	if(pstPackRVal->fCellTMin >= g_stLocalDLPara.afTempMin[6] && pstPackRVal->fCellUMax * 1000 <= g_stLocalDLPara.afCellVMax[5]) {
		cRow = -1;
		ucCol = 0;
		for(uint8_t i = 0; i < LOCAL_DL_ROWS2 - 1; i++) {
			if(i == 0) {
				if(pstPackRVal->fCellTMax < g_stLocalDLPara.afTempMax[i] && pstPackRVal->fCellTMax >= g_stLocalDLPara.afTempMin[6]) {
					cRow = i;
					break;
				}
			} else {
				if(pstPackRVal->fCellTMax < g_stLocalDLPara.afTempMax[i] && pstPackRVal->fCellTMax >= g_stLocalDLPara.afTempMax[i - 1]) {
					cRow = i;
					break;
				}
			}
			if(pstPackRVal->fCellTMax >= g_stLocalDLPara.afTempMax[4]) {
				cRow = 5;
				break;
			}
		}
		for(uint8_t i = 0; i < LOCAL_DL_COLUMNS2; i++) {
			if(i == 0) {
				if(pstPackRVal->fCellUMin * 1000 > g_stLocalDLPara.afCellVMin[i] && pstPackRVal->fCellUMin * 1000 <= g_stLocalDLPara.afCellVMax[5]) {
					ucCol = i;
					break;
				}
			} else {
				if(pstPackRVal->fCellUMin * 1000 > g_stLocalDLPara.afCellVMin[i] && pstPackRVal->fCellUMin * 1000 <= g_stLocalDLPara.afCellVMin[i - 1]) {
					ucCol = i;
					break;
				}
			}
			if(pstPackRVal->fCellUMin * 1000 <= g_stLocalDLPara.afCellVMin[7]) {
				ucCol = 8;
				break;
			}
		}
		if(cRow < 0) {
			pfLmtDsgI4 = 0;
		} else if(*pfLmtDsgI >= g_stLocalDLPara.aafDsgRate4[cRow][ucCol] * g_stCfg.stLocal.usDesignAH) {
			pfLmtDsgI4 = g_stLocalDLPara.aafDsgRate4[cRow][ucCol] * g_stCfg.stLocal.usDesignAH;
		}
	}
	if((pstPackRVal->fCellTMin < g_stLocalDLPara.afTempMin[6] && pstPackRVal->fCellUMax * 1000 > g_stLocalDLPara.afCellVMax[5])
		&& (pstPackRVal->fCellTMin < g_stLocalDLPara.afTempMin[6] && pstPackRVal->fCellUMax * 1000 <= g_stLocalDLPara.afCellVMax[5])
		&& (pstPackRVal->fCellTMin >= g_stLocalDLPara.afTempMin[6] && pstPackRVal->fCellUMax * 1000 > g_stLocalDLPara.afCellVMax[5])
		&& (pstPackRVal->fCellTMin >= g_stLocalDLPara.afTempMin[6] && pstPackRVal->fCellUMax * 1000 <= g_stLocalDLPara.afCellVMax[5])) {
		if(pfLmtDsgI1 < pfLmtDsgI2) {
		*pfLmtDsgI = pfLmtDsgI1;
		} else {
			*pfLmtDsgI = pfLmtDsgI2;
		}
		if(*pfLmtDsgI > pfLmtDsgI3) {
			*pfLmtDsgI = pfLmtDsgI3;
		}
		if(*pfLmtDsgI > pfLmtDsgI4) {
			*pfLmtDsgI = pfLmtDsgI4;
		}
	} else if((pstPackRVal->fCellTMin < g_stLocalDLPara.afTempMin[6] && pstPackRVal->fCellUMax * 1000 <= g_stLocalDLPara.afCellVMax[5])
		&& (pstPackRVal->fCellTMin >= g_stLocalDLPara.afTempMin[6] && pstPackRVal->fCellUMax * 1000 > g_stLocalDLPara.afCellVMax[5])
		&& (pstPackRVal->fCellTMin >= g_stLocalDLPara.afTempMin[6] && pstPackRVal->fCellUMax * 1000 <= g_stLocalDLPara.afCellVMax[5])) {
		if(pfLmtDsgI2 < pfLmtDsgI3) {
		*pfLmtDsgI = pfLmtDsgI2;
		} else {
			*pfLmtDsgI = pfLmtDsgI3;
		}
		if(*pfLmtDsgI > pfLmtDsgI4) {
			*pfLmtDsgI = pfLmtDsgI4;
		}
	} else if((pstPackRVal->fCellTMin < g_stLocalDLPara.afTempMin[6] && pstPackRVal->fCellUMax * 1000 > g_stLocalDLPara.afCellVMax[5])
		&& (pstPackRVal->fCellTMin >= g_stLocalDLPara.afTempMin[6] && pstPackRVal->fCellUMax * 1000 > g_stLocalDLPara.afCellVMax[5])
		&& (pstPackRVal->fCellTMin >= g_stLocalDLPara.afTempMin[6] && pstPackRVal->fCellUMax * 1000 <= g_stLocalDLPara.afCellVMax[5])) {
		if(pfLmtDsgI1 < pfLmtDsgI3) {
		*pfLmtDsgI = pfLmtDsgI1;
		} else {
			*pfLmtDsgI = pfLmtDsgI3;
		}
		if(*pfLmtDsgI > pfLmtDsgI4) {
			*pfLmtDsgI = pfLmtDsgI4;
		}
	} else if((pstPackRVal->fCellTMin < g_stLocalDLPara.afTempMin[6] && pstPackRVal->fCellUMax * 1000 > g_stLocalDLPara.afCellVMax[5])
		&& (pstPackRVal->fCellTMin < g_stLocalDLPara.afTempMin[6] && pstPackRVal->fCellUMax * 1000 <= g_stLocalDLPara.afCellVMax[5])
		&& (pstPackRVal->fCellTMin >= g_stLocalDLPara.afTempMin[6] && pstPackRVal->fCellUMax * 1000 <= g_stLocalDLPara.afCellVMax[5])) {
		if(pfLmtDsgI1 < pfLmtDsgI2) {
		*pfLmtDsgI = pfLmtDsgI1;
		} else {
			*pfLmtDsgI = pfLmtDsgI2;
		}
		if(*pfLmtDsgI > pfLmtDsgI4) {
			*pfLmtDsgI = pfLmtDsgI4;
		}
	} else if((pstPackRVal->fCellTMin < g_stLocalDLPara.afTempMin[6] && pstPackRVal->fCellUMax * 1000 > g_stLocalDLPara.afCellVMax[5])
		&& (pstPackRVal->fCellTMin < g_stLocalDLPara.afTempMin[6] && pstPackRVal->fCellUMax * 1000 <= g_stLocalDLPara.afCellVMax[5])
		&& (pstPackRVal->fCellTMin >= g_stLocalDLPara.afTempMin[6] && pstPackRVal->fCellUMax * 1000 > g_stLocalDLPara.afCellVMax[5])) {
		if(pfLmtDsgI1 < pfLmtDsgI2) {
		*pfLmtDsgI = pfLmtDsgI1;
		} else {
			*pfLmtDsgI = pfLmtDsgI2;
		}
		if(*pfLmtDsgI > pfLmtDsgI3) {
			*pfLmtDsgI = pfLmtDsgI3;
		}
	} else if((pstPackRVal->fCellTMin >= g_stLocalDLPara.afTempMin[6] && pstPackRVal->fCellUMax * 1000 > g_stLocalDLPara.afCellVMax[5])
		&& (pstPackRVal->fCellTMin >= g_stLocalDLPara.afTempMin[6] && pstPackRVal->fCellUMax * 1000 <= g_stLocalDLPara.afCellVMax[5])) {
		if(pfLmtDsgI3 < pfLmtDsgI4) {
		*pfLmtDsgI = pfLmtDsgI3;
		} else {
			*pfLmtDsgI = pfLmtDsgI4;
		}
	} else if((pstPackRVal->fCellTMin < g_stLocalDLPara.afTempMin[6] && pstPackRVal->fCellUMax * 1000 <= g_stLocalDLPara.afCellVMax[5])
		&& (pstPackRVal->fCellTMin >= g_stLocalDLPara.afTempMin[6] && pstPackRVal->fCellUMax * 1000 <= g_stLocalDLPara.afCellVMax[5])) {
		if(pfLmtDsgI2 < pfLmtDsgI4) {
		*pfLmtDsgI = pfLmtDsgI2;
		} else {
			*pfLmtDsgI = pfLmtDsgI4;
		}
	} else if((pstPackRVal->fCellTMin < g_stLocalDLPara.afTempMin[6] && pstPackRVal->fCellUMax * 1000 <= g_stLocalDLPara.afCellVMax[5])
		&& (pstPackRVal->fCellTMin >= g_stLocalDLPara.afTempMin[6] && pstPackRVal->fCellUMax * 1000 > g_stLocalDLPara.afCellVMax[5])) {
		if(pfLmtDsgI2 < pfLmtDsgI3) {
		*pfLmtDsgI = pfLmtDsgI2;
		} else {
			*pfLmtDsgI = pfLmtDsgI3;
		}
	} else if((pstPackRVal->fCellTMin < g_stLocalDLPara.afTempMin[6] && pstPackRVal->fCellUMax * 1000 > g_stLocalDLPara.afCellVMax[5])
		&& (pstPackRVal->fCellTMin >= g_stLocalDLPara.afTempMin[6] && pstPackRVal->fCellUMax * 1000 <= g_stLocalDLPara.afCellVMax[5])) {
		if(pfLmtDsgI1 < pfLmtDsgI4) {
		*pfLmtDsgI = pfLmtDsgI1;
		} else {
			*pfLmtDsgI = pfLmtDsgI4;
		}
	} else if((pstPackRVal->fCellTMin < g_stLocalDLPara.afTempMin[6] && pstPackRVal->fCellUMax * 1000 > g_stLocalDLPara.afCellVMax[5])
		&& (pstPackRVal->fCellTMin >= g_stLocalDLPara.afTempMin[6] && pstPackRVal->fCellUMax * 1000 > g_stLocalDLPara.afCellVMax[5])) {
		if(pfLmtDsgI1 < pfLmtDsgI3) {
		*pfLmtDsgI = pfLmtDsgI1;
		} else {
			*pfLmtDsgI = pfLmtDsgI3;
		}
	} else if((pstPackRVal->fCellTMin < g_stLocalDLPara.afTempMin[6] && pstPackRVal->fCellUMax * 1000 > g_stLocalDLPara.afCellVMax[5])
		&& (pstPackRVal->fCellTMin < g_stLocalDLPara.afTempMin[6] && pstPackRVal->fCellUMax * 1000 <= g_stLocalDLPara.afCellVMax[5])) {
		if(pfLmtDsgI1 < pfLmtDsgI2) {
		*pfLmtDsgI = pfLmtDsgI1;
		} else {
			*pfLmtDsgI = pfLmtDsgI2;
		}
	} else if(pstPackRVal->fCellTMin >= g_stLocalDLPara.afTempMin[6] && pstPackRVal->fCellUMax * 1000 <= g_stLocalDLPara.afCellVMax[5]) {
		*pfLmtDsgI = pfLmtDsgI4;
	} else if(pstPackRVal->fCellTMin >= g_stLocalDLPara.afTempMin[6] && pstPackRVal->fCellUMax * 1000 > g_stLocalDLPara.afCellVMax[5]) {
		*pfLmtDsgI = pfLmtDsgI3;
	} else if(pstPackRVal->fCellTMin < g_stLocalDLPara.afTempMin[6] && pstPackRVal->fCellUMax * 1000 <= g_stLocalDLPara.afCellVMax[5]) {
		*pfLmtDsgI = pfLmtDsgI2;
	} else if(pstPackRVal->fCellTMin < g_stLocalDLPara.afTempMin[6] && pstPackRVal->fCellUMax * 1000 > g_stLocalDLPara.afCellVMax[5]) {
		*pfLmtDsgI = pfLmtDsgI1;
	}

	/* PEAK discharge current limit */
	float pfPeakLmtDsgI1;
	float pfPeakLmtDsgI2;
	float pfPeakLmtDsgI3;
	float pfPeakLmtDsgI4;
	if(pstPackRVal->fCellTMin < g_stLocalPDCLPara.afTempMin[6] && pstPackRVal->fCellUMax * 1000 > g_stLocalPDCLPara.afCellVMax[5]) {
		cRow = -1;
		ucCol = 0;
		for(uint8_t i = 1; i < LOCAL_PDCL_ROWS1 + 1; i++) {
			if(pstPackRVal->fCellTMin < g_stLocalPDCLPara.afTempMin[i] && pstPackRVal->fCellTMin >= g_stLocalPDCLPara.afTempMin[i - 1]) {
				cRow = i;
				break;
			}
			if(pstPackRVal->fCellTMin <= g_stLocalPDCLPara.afTempMin[0]) {
				cRow = 0;
				break;
			}
		}
		for(uint8_t i = 1; i < LOCAL_PDCL_COLUMNS1; i++) {
			if(pstPackRVal->fCellUMax * 1000 > g_stLocalPDCLPara.afCellVMax[i] && pstPackRVal->fCellUMax * 1000 <= g_stLocalPDCLPara.afCellVMax[i - 1]) {
				ucCol = i;
				break;
			}
			if(pstPackRVal->fCellUMax * 1000 > g_stLocalPDCLPara.afCellVMax[0]) {
				ucCol = 0;
				break;
			}
		}
		if(cRow < 0) {
			pfPeakLmtDsgI1 = 0;
		} else if(*pfPeakLmtDsgI >= g_stLocalPDCLPara.aafDsgRate1[cRow][ucCol] * g_stCfg.stLocal.usDesignAH) {
			pfPeakLmtDsgI1 = g_stLocalPDCLPara.aafDsgRate1[cRow][ucCol] * g_stCfg.stLocal.usDesignAH;
		}
	}
	if(pstPackRVal->fCellTMin < g_stLocalPDCLPara.afTempMin[6] && pstPackRVal->fCellUMax * 1000 <= g_stLocalPDCLPara.afCellVMax[5]) {
		cRow = -1;
		ucCol = 0;
		for(uint8_t i = 1; i < LOCAL_PDCL_ROWS1 + 1; i++) {
			if(pstPackRVal->fCellTMin <= g_stLocalPDCLPara.afTempMin[i] && pstPackRVal->fCellTMin > g_stLocalPDCLPara.afTempMin[i - 1]) {
				cRow = i;
				break;
			}
			if(pstPackRVal->fCellTMin <= g_stLocalPDCLPara.afTempMin[0]) {
				cRow = 0;
				break;
			}
//			if(pstPackRVal->fCellTMin > g_stLocalPDCLPara.afTempMin[6]) {
//				cRow = 6;
//				break;
//			}
		}
		for(uint8_t i = 0; i < LOCAL_PDCL_COLUMNS2; i++) {
			if(i == 0) {
				if(pstPackRVal->fCellUMin * 1000 > g_stLocalPDCLPara.afCellVMin[i] && pstPackRVal->fCellUMin * 1000 <= g_stLocalPDCLPara.afCellVMax[5]) {
					ucCol = i;
					break;
				}
			} else {
				if(pstPackRVal->fCellUMin * 1000 > g_stLocalPDCLPara.afCellVMin[i] && pstPackRVal->fCellUMin * 1000 <= g_stLocalPDCLPara.afCellVMin[i - 1]) {
					ucCol = i;
					break;
				}
			}
			if(pstPackRVal->fCellUMin * 1000 <= g_stLocalPDCLPara.afCellVMin[7]) {
				ucCol = 8;
				break;
			}
		}
		if(cRow < 0) {
			pfPeakLmtDsgI2 = 0;
		} else if(*pfPeakLmtDsgI >= g_stLocalPDCLPara.aafDsgRate2[cRow][ucCol] * g_stCfg.stLocal.usDesignAH) {
			pfPeakLmtDsgI2 = g_stLocalPDCLPara.aafDsgRate2[cRow][ucCol] * g_stCfg.stLocal.usDesignAH;
		}
	}
	if(pstPackRVal->fCellTMin >= g_stLocalPDCLPara.afTempMin[6] && pstPackRVal->fCellUMax * 1000 > g_stLocalPDCLPara.afCellVMax[5]) {
		cRow = -1;
		ucCol = 0;
		for(uint8_t i = 0; i < LOCAL_PDCL_ROWS2 - 1; i++) {
			if(i == 0) {
				if(pstPackRVal->fCellTMax < g_stLocalPDCLPara.afTempMax[i] && pstPackRVal->fCellTMax >= g_stLocalPDCLPara.afTempMin[6]) {
					cRow = i;
					break;
				}
			} else {
				if(pstPackRVal->fCellTMax < g_stLocalPDCLPara.afTempMax[i] && pstPackRVal->fCellTMax >= g_stLocalPDCLPara.afTempMax[i - 1]) {
					cRow = i;
					break;
				}
			}
			if(pstPackRVal->fCellTMax >= g_stLocalPDCLPara.afTempMax[4]) {
				cRow = 5;
				break;
			}
		}
		for(uint8_t i = 1; i < LOCAL_PDCL_COLUMNS1; i++) {
			if(pstPackRVal->fCellUMax * 1000 > g_stLocalPDCLPara.afCellVMax[i] && pstPackRVal->fCellUMax * 1000 <= g_stLocalPDCLPara.afCellVMax[i - 1]) {
				ucCol = i;
				break;
			}
			if(pstPackRVal->fCellUMax * 1000 > g_stLocalPDCLPara.afCellVMax[0]) {
				ucCol = 0;
				break;
			}
		}
		if(cRow < 0) {
			pfPeakLmtDsgI3 = 0;
		} else if(*pfPeakLmtDsgI >= g_stLocalPDCLPara.aafDsgRate3[cRow][ucCol] * g_stCfg.stLocal.usDesignAH) {
			pfPeakLmtDsgI3 = g_stLocalPDCLPara.aafDsgRate3[cRow][ucCol] * g_stCfg.stLocal.usDesignAH;
		}
	}
	if(pstPackRVal->fCellTMin >= g_stLocalPDCLPara.afTempMin[6] && pstPackRVal->fCellUMax * 1000 <= g_stLocalPDCLPara.afCellVMax[5]) {
		cRow = -1;
		ucCol = 0;
		for(uint8_t i = 0; i < LOCAL_PDCL_ROWS2 - 1; i++) {
			if(i == 0) {
				if(pstPackRVal->fCellTMax < g_stLocalPDCLPara.afTempMax[i] && pstPackRVal->fCellTMax >= g_stLocalPDCLPara.afTempMin[6]) {
					cRow = i;
					break;
				}
			} else {
				if(pstPackRVal->fCellTMax < g_stLocalPDCLPara.afTempMax[i] && pstPackRVal->fCellTMax >= g_stLocalPDCLPara.afTempMax[i - 1]) {
					cRow = i;
					break;
				}
			}
			if(pstPackRVal->fCellTMax >= g_stLocalPDCLPara.afTempMax[4]) {
				cRow = 5;
				break;
			}
		}
		for(uint8_t i = 0; i < LOCAL_PDCL_COLUMNS2; i++) {
			if(i == 0) {
				if(pstPackRVal->fCellUMin * 1000 > g_stLocalPDCLPara.afCellVMin[i] && pstPackRVal->fCellUMin * 1000 <= g_stLocalPDCLPara.afCellVMax[5]) {
					ucCol = i;
					break;
				}
			} else {
				if(pstPackRVal->fCellUMin * 1000 > g_stLocalPDCLPara.afCellVMin[i] && pstPackRVal->fCellUMin * 1000 <= g_stLocalPDCLPara.afCellVMin[i - 1]) {
					ucCol = i;
					break;
				}
			}
			if(pstPackRVal->fCellUMin * 1000 <= g_stLocalPDCLPara.afCellVMin[7]) {
				ucCol = 8;
				break;
			}
		}
		if(cRow < 0) {
			pfPeakLmtDsgI4 = 0;
		} else if(*pfPeakLmtDsgI >= g_stLocalPDCLPara.aafDsgRate4[cRow][ucCol] * g_stCfg.stLocal.usDesignAH) {
			pfPeakLmtDsgI4 = g_stLocalPDCLPara.aafDsgRate4[cRow][ucCol] * g_stCfg.stLocal.usDesignAH;
		}
	}
	if((pstPackRVal->fCellTMin < g_stLocalPDCLPara.afTempMin[6] && pstPackRVal->fCellUMax * 1000 > g_stLocalPDCLPara.afCellVMax[5])
		&& (pstPackRVal->fCellTMin < g_stLocalPDCLPara.afTempMin[6] && pstPackRVal->fCellUMax * 1000 <= g_stLocalPDCLPara.afCellVMax[5])
		&& (pstPackRVal->fCellTMin >= g_stLocalPDCLPara.afTempMin[6] && pstPackRVal->fCellUMax * 1000 > g_stLocalPDCLPara.afCellVMax[5])
		&& (pstPackRVal->fCellTMin >= g_stLocalPDCLPara.afTempMin[6] && pstPackRVal->fCellUMax * 1000 <= g_stLocalPDCLPara.afCellVMax[5])) {
		if(pfPeakLmtDsgI1 < pfPeakLmtDsgI2) {
		*pfPeakLmtDsgI = pfPeakLmtDsgI1;
		} else {
			*pfPeakLmtDsgI = pfPeakLmtDsgI2;
		}
		if(*pfPeakLmtDsgI > pfPeakLmtDsgI3) {
			*pfPeakLmtDsgI = pfPeakLmtDsgI3;
		}
		if(*pfPeakLmtDsgI > pfPeakLmtDsgI4) {
			*pfPeakLmtDsgI = pfPeakLmtDsgI4;
		}
	} else if((pstPackRVal->fCellTMin < g_stLocalPDCLPara.afTempMin[6] && pstPackRVal->fCellUMax * 1000 <= g_stLocalPDCLPara.afCellVMax[5])
		&& (pstPackRVal->fCellTMin >= g_stLocalPDCLPara.afTempMin[6] && pstPackRVal->fCellUMax * 1000 > g_stLocalPDCLPara.afCellVMax[5])
		&& (pstPackRVal->fCellTMin >= g_stLocalPDCLPara.afTempMin[6] && pstPackRVal->fCellUMax * 1000 <= g_stLocalPDCLPara.afCellVMax[5])) {
		if(pfPeakLmtDsgI2 < pfPeakLmtDsgI3) {
		*pfPeakLmtDsgI = pfPeakLmtDsgI2;
		} else {
			*pfPeakLmtDsgI = pfPeakLmtDsgI3;
		}
		if(*pfPeakLmtDsgI > pfPeakLmtDsgI4) {
			*pfPeakLmtDsgI = pfPeakLmtDsgI4;
		}
	} else if((pstPackRVal->fCellTMin < g_stLocalPDCLPara.afTempMin[6] && pstPackRVal->fCellUMax * 1000 > g_stLocalPDCLPara.afCellVMax[5])
		&& (pstPackRVal->fCellTMin >= g_stLocalPDCLPara.afTempMin[6] && pstPackRVal->fCellUMax * 1000 > g_stLocalPDCLPara.afCellVMax[5])
		&& (pstPackRVal->fCellTMin >= g_stLocalPDCLPara.afTempMin[6] && pstPackRVal->fCellUMax * 1000 <= g_stLocalPDCLPara.afCellVMax[5])) {
		if(pfPeakLmtDsgI1 < pfPeakLmtDsgI3) {
		*pfPeakLmtDsgI = pfPeakLmtDsgI1;
		} else {
			*pfPeakLmtDsgI = pfPeakLmtDsgI3;
		}
		if(*pfPeakLmtDsgI > pfPeakLmtDsgI4) {
			*pfPeakLmtDsgI = pfPeakLmtDsgI4;
		}
	} else if((pstPackRVal->fCellTMin < g_stLocalPDCLPara.afTempMin[6] && pstPackRVal->fCellUMax * 1000 > g_stLocalPDCLPara.afCellVMax[5])
		&& (pstPackRVal->fCellTMin < g_stLocalPDCLPara.afTempMin[6] && pstPackRVal->fCellUMax * 1000 <= g_stLocalPDCLPara.afCellVMax[5])
		&& (pstPackRVal->fCellTMin >= g_stLocalPDCLPara.afTempMin[6] && pstPackRVal->fCellUMax * 1000 <= g_stLocalPDCLPara.afCellVMax[5])) {
		if(pfPeakLmtDsgI1 < pfPeakLmtDsgI2) {
		*pfPeakLmtDsgI = pfPeakLmtDsgI1;
		} else {
			*pfPeakLmtDsgI = pfPeakLmtDsgI2;
		}
		if(*pfPeakLmtDsgI > pfPeakLmtDsgI4) {
			*pfPeakLmtDsgI = pfPeakLmtDsgI4;
		}
	} else if((pstPackRVal->fCellTMin < g_stLocalPDCLPara.afTempMin[6] && pstPackRVal->fCellUMax * 1000 > g_stLocalPDCLPara.afCellVMax[5])
		&& (pstPackRVal->fCellTMin < g_stLocalPDCLPara.afTempMin[6] && pstPackRVal->fCellUMax * 1000 <= g_stLocalPDCLPara.afCellVMax[5])
		&& (pstPackRVal->fCellTMin >= g_stLocalPDCLPara.afTempMin[6] && pstPackRVal->fCellUMax * 1000 > g_stLocalPDCLPara.afCellVMax[5])) {
		if(pfPeakLmtDsgI1 < pfPeakLmtDsgI2) {
		*pfPeakLmtDsgI = pfPeakLmtDsgI1;
		} else {
			*pfPeakLmtDsgI = pfPeakLmtDsgI2;
		}
		if(*pfPeakLmtDsgI > pfPeakLmtDsgI3) {
			*pfPeakLmtDsgI = pfPeakLmtDsgI3;
		}
	} else if((pstPackRVal->fCellTMin >= g_stLocalPDCLPara.afTempMin[6] && pstPackRVal->fCellUMax * 1000 > g_stLocalPDCLPara.afCellVMax[5])
		&& (pstPackRVal->fCellTMin >= g_stLocalPDCLPara.afTempMin[6] && pstPackRVal->fCellUMax * 1000 <= g_stLocalPDCLPara.afCellVMax[5])) {
		if(pfPeakLmtDsgI3 < pfPeakLmtDsgI4) {
		*pfPeakLmtDsgI = pfPeakLmtDsgI3;
		} else {
			*pfPeakLmtDsgI = pfPeakLmtDsgI4;
		}
	} else if((pstPackRVal->fCellTMin < g_stLocalPDCLPara.afTempMin[6] && pstPackRVal->fCellUMax * 1000 <= g_stLocalPDCLPara.afCellVMax[5])
		&& (pstPackRVal->fCellTMin >= g_stLocalPDCLPara.afTempMin[6] && pstPackRVal->fCellUMax * 1000 <= g_stLocalPDCLPara.afCellVMax[5])) {
		if(pfPeakLmtDsgI2 < pfPeakLmtDsgI4) {
		*pfPeakLmtDsgI = pfPeakLmtDsgI2;
		} else {
			*pfPeakLmtDsgI = pfPeakLmtDsgI4;
		}
	} else if((pstPackRVal->fCellTMin < g_stLocalPDCLPara.afTempMin[6] && pstPackRVal->fCellUMax * 1000 <= g_stLocalPDCLPara.afCellVMax[5])
		&& (pstPackRVal->fCellTMin >= g_stLocalPDCLPara.afTempMin[6] && pstPackRVal->fCellUMax * 1000 > g_stLocalPDCLPara.afCellVMax[5])) {
		if(pfPeakLmtDsgI2 < pfPeakLmtDsgI3) {
		*pfPeakLmtDsgI = pfPeakLmtDsgI2;
		} else {
			*pfPeakLmtDsgI = pfPeakLmtDsgI3;
		}
	} else if((pstPackRVal->fCellTMin < g_stLocalPDCLPara.afTempMin[6] && pstPackRVal->fCellUMax * 1000 > g_stLocalPDCLPara.afCellVMax[5])
		&& (pstPackRVal->fCellTMin >= g_stLocalPDCLPara.afTempMin[6] && pstPackRVal->fCellUMax * 1000 <= g_stLocalPDCLPara.afCellVMax[5])) {
		if(pfPeakLmtDsgI1 < pfPeakLmtDsgI4) {
		*pfPeakLmtDsgI = pfPeakLmtDsgI1;
		} else {
			*pfPeakLmtDsgI = pfPeakLmtDsgI4;
		}
	} else if((pstPackRVal->fCellTMin < g_stLocalPDCLPara.afTempMin[6] && pstPackRVal->fCellUMax * 1000 > g_stLocalPDCLPara.afCellVMax[5])
		&& (pstPackRVal->fCellTMin >= g_stLocalPDCLPara.afTempMin[6] && pstPackRVal->fCellUMax * 1000 > g_stLocalPDCLPara.afCellVMax[5])) {
		if(pfPeakLmtDsgI1 < pfPeakLmtDsgI3) {
		*pfPeakLmtDsgI = pfPeakLmtDsgI1;
		} else {
			*pfPeakLmtDsgI = pfPeakLmtDsgI3;
		}
	} else if((pstPackRVal->fCellTMin < g_stLocalPDCLPara.afTempMin[6] && pstPackRVal->fCellUMax * 1000 > g_stLocalPDCLPara.afCellVMax[5])
		&& (pstPackRVal->fCellTMin < g_stLocalPDCLPara.afTempMin[6] && pstPackRVal->fCellUMax * 1000 <= g_stLocalPDCLPara.afCellVMax[5])) {
		if(pfPeakLmtDsgI1 < pfPeakLmtDsgI2) {
		*pfPeakLmtDsgI = pfPeakLmtDsgI1;
		} else {
			*pfPeakLmtDsgI = pfPeakLmtDsgI2;
		}
	} else if(pstPackRVal->fCellTMin >= g_stLocalPDCLPara.afTempMin[6] && pstPackRVal->fCellUMax * 1000 <= g_stLocalPDCLPara.afCellVMax[5]) {
		*pfPeakLmtDsgI = pfPeakLmtDsgI4;
	} else if(pstPackRVal->fCellTMin >= g_stLocalPDCLPara.afTempMin[6] && pstPackRVal->fCellUMax * 1000 > g_stLocalPDCLPara.afCellVMax[5]) {
		*pfPeakLmtDsgI = pfPeakLmtDsgI3;
	} else if(pstPackRVal->fCellTMin < g_stLocalPDCLPara.afTempMin[6] && pstPackRVal->fCellUMax * 1000 <= g_stLocalPDCLPara.afCellVMax[5]) {
		*pfPeakLmtDsgI = pfPeakLmtDsgI2;
	} else if(pstPackRVal->fCellTMin < g_stLocalPDCLPara.afTempMin[6] && pstPackRVal->fCellUMax * 1000 > g_stLocalPDCLPara.afCellVMax[5]) {
		*pfPeakLmtDsgI = pfPeakLmtDsgI1;
	}

	/* PEAK discharge limit (based upon MOS temperature) */
	for(int8_t i=LOCAL_PDCLMOS_ROWS - 1; i >= 0; i--) {
		if(pstPackRVal->fMosT >= g_stLocalPDCLMOSPara.afTemp[i]) {
			*pfPeakLmtDsgI *= g_stLocalPDCLMOSPara.afRate[i + 1];
			*pfReqChgI *= g_stLocalPDCLMOSPara.afRate[i + 1];
			*pfLmtChgI *= g_stLocalPDCLMOSPara.afRate[i + 1];
			*pfLmtDsgI *= g_stLocalPDCLMOSPara.afRate[i + 1];
			break;
		}
	}
	if(pstPackRVal->fCellTMin > 10 && pstPackRVal->fCellTMax < 63) {
	/* The charger requests the allowable value of the current*/
		static bool b_ReqChgFlag = 0;
		if(pstPackRVal->fCellUMax < 3.39){
			if(pstPackRVal->fCellUMax < 3.38 && pstPackRVal->fPackCur < 1 && b_ReqChgFlag == 1) {
//				memset(fReqChgI100, 0, sizeof fReqChgI100);
				b_ReqChgFlag = 0;
			} else if(pstPackRVal->fCellUMax < 3.38 && pstPackRVal->fPackCur >= 1 && b_ReqChgFlag == 1) {
				*pfReqChgI = 40;
			}	else if(pstPackRVal->fCellUMax >= 3.38 && b_ReqChgFlag == 1) {
				*pfReqChgI = 40;
			}
		}else if(pstPackRVal->fCellUMax >= 3.39 && pstPackRVal->fCellUMax < 3.395 /*&& b_ReqChgFlag == 1*/){
			*pfReqChgI = 40;
		   b_ReqChgFlag = 1;
		}else if(pstPackRVal->fCellUMax >= 3.395 && pstPackRVal->fCellUMax < 3.4){
			*pfReqChgI = 30;
			 b_ReqChgFlag = 1;
		}else if(pstPackRVal->fCellUMax >= 3.4 && pstPackRVal->fCellUMax < 3.42){
			*pfReqChgI = 20;
			 b_ReqChgFlag = 1;
		}else if(pstPackRVal->fCellUMax >= 3.42 && pstPackRVal->fCellUMax < 3.435){
			*pfReqChgI = 10;
			 b_ReqChgFlag = 1;
		}else if(pstPackRVal->fCellUMax >= 3.435 && pstPackRVal->fCellUMax < 3.445){
			*pfReqChgI = 7;
			 b_ReqChgFlag = 1;
		}else if(pstPackRVal->fCellUMax >= 3.445) {
			*pfReqChgI = 5;
			 b_ReqChgFlag = 1;
		}
  }
	/* 0x18ff50e5 allowable discharge voltage when there is a value */
		if(g_bMChgerAbNor) {
			*pfLmtDsgI =0;
			*pfPeakLmtDsgI = 0;
	}
	/* Balancing on request current allowable value (special balancing mode) */
	if(g_stCfg.stLocal.sBalanceOn == 1 && g_BalanceFlag_2 == 1 && pstPackRVal->fCellUMax >= 3.35){
//		g_stCfg.stLocal.sBalanceOn = 1;
		*pfReqChgI = 3;
	}
	/* The SOC is 100% of the allowable value of the requested current*/
	if(pstPackRVal->fPackSoc == 100){
		g_stCfg.stLocal.sBalanceOn = 0;
		*pfReqChgI = 0;
	}
	
	if(pstPackRVal->ucHeatEn == 0x55 && pstPackRVal->fCellTMin <= 2) {	//charge disable when heat requested, and heating request current is 5A
		*pfReqChgI = 5;
	}
	
	/* Alarm or protection limit */
	
	if(GET_ALM0_CODE(1) || GET_ALM1_CODE(1) || GET_ALM0_CODE(2) || GET_ALM1_CODE(2) || GET_ALM0_CODE(3) || GET_ALM1_CODE(3)) {
		*pfLmtDsgI =0;
		*pfPeakLmtDsgI = 0;
	}
	if(GET_ALM1_CODE(53) || GET_ALM1_CODE(55) || GET_ALM1_CODE(57)) {
		*pfLmtDsgI *= 0.4;
		*pfPeakLmtDsgI *= 0.4;
	}
	if(GET_ALM0_CODE(1) || GET_ALM1_CODE(1) || GET_ALM0_CODE(2) || GET_ALM1_CODE(2) || GET_ALM0_CODE(4) || GET_ALM1_CODE(4) || GET_ALM0_CODE(6) || GET_ALM1_CODE(6) || GET_ALM1_CODE(56)
		|| GET_ALM0_CODE(11) || GET_ALM1_CODE(11) || GET_ALM0_CODE(12) || GET_ALM1_CODE(12) 
		|| GET_ALM0_CODE(14) || GET_ALM1_CODE(14) /*|| g_ucFullChgRec != 0g_ucChgFull != 0*/) {
		*pfLmtChgI = 0;
		*pfReqChgI = 0;
	}
	if(g_ucChgFull != 0) {
		*pfReqChgI = 0;
	}
	if(GET_ALM0_CODE(5) || GET_ALM1_CODE(5) || GET_ALM0_CODE(7) || GET_ALM1_CODE(7)
		|| GET_ALM0_CODE(10) || GET_ALM1_CODE(10) || GET_ALM0_CODE(13) || GET_ALM1_CODE(13)
		|| GET_ALM0_CODE(14) || GET_ALM1_CODE(14) || GET_ALM0_CODE(15) || GET_ALM1_CODE(15) || g_ucDsgEmpty != 0) {
		*pfLmtDsgI =0;
		*pfPeakLmtDsgI = 0;
	}
//	if(GET_ALM1_CODE(55)) {
//		*pfLmtDsgI *= (pstPackRVal->fCellUMin - 2.7) / 0.1;
//		if(*pfLmtDsgI < 0) {
//			*pfLmtDsgI = 0;
//		}
//		*pfPeakLmtDsgI *= (pstPackRVal->fCellUMin - 2.7) / 0.1;
//		if(*pfPeakLmtDsgI < 0) {
//			*pfPeakLmtDsgI = 0;
//		}
//	}
	if(GET_ALM0_CODE(1) || GET_ALM1_CODE(1) || pstPackRVal->uErrCode.stErrCode.bTempSensor || pstPackRVal->uErrCode.stErrCode.bVoltSensor) {
		*pfLmtChgI = 0;
		*pfReqChgI = 0;
		*pfLmtDsgI = 0;
		*pfPeakLmtDsgI = 0;
		pstPackRVal->ucDsgEn = 0xAA;
		pstPackRVal->ucChgEn = 0xAA;
	}
}

/*******************************************************************************
* Function Name  : local_balance_proc
* Description    : Cell balancing
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/ 
void local_balance_proc(void) {
	LOCAL_PACK_RVAL_S* pstPackRVal = g_stLocalArrayRVal.astPackRVal + g_stPrl.ucSelfId;
	static uint8_t s_ucTick[16] = {0};
	for(uint8_t i=0;i<CFG_CELL_NUM;i++) {
		if(pstPackRVal->afCellU[i] >= g_stCfg.stLocal.usBALActVolt && (pstPackRVal->afCellU[i]-pstPackRVal->fCellUMin > g_stCfg.stLocal.usBALActDVolt)) {
			s_ucTick[i]++;
			if(s_ucTick[i] * g_stCfg.stLocal.usCyclePeriod > 1000) {
				Bat_Balance(i);
				balanc[i] = 1; 
				s_ucTick[i] = 0;
			}
		} else {
			s_ucTick[i] = 0;
			balanc[i] = 0; 
		}
	}
}

bool local_proc(void) {
	/* If the negotiation is not completed, it will not work properly*/
	LOCAL_PACK_RVAL_S* pstPackRVal = g_stLocalArrayRVal.astPackRVal + g_stPrl.ucSelfId;
	pstPackRVal->fLmtChgI = 2.5 * g_stCfg.stLocal.usDesignAH;
	pstPackRVal->fLmtDsgI = 4 * g_stCfg.stLocal.usDesignAH;
	pstPackRVal->fReqChgI = 2.5 * g_stCfg.stLocal.usDesignAH;
	pstPackRVal->fPeakLmtDsgI = 4 * g_stCfg.stLocal.usDesignAH;
	if(g_stLocalArrayRVal.eLocalStat != eLocalStatRun || (pstPackRVal->uErrCode.ucErrCode != 0 && pstPackRVal->uErrCode.stErrCode.bInputOV != 1)) {
		pstPackRVal->ucChgEn = 0xAA;
		pstPackRVal->ucDsgEn = 0xAA;
		pstPackRVal->ucHeatEn = 0xAA;
		float fFlase = 0;
		if(g_stAfe.uRam.stCode.CHG_FET != 0) {
			afe_set_ao(eAfeRamCodeCHGMOS, 1, &fFlase);
		}
		if(g_stAfe.uRam.stCode.DSG_FET != 0) {
			afe_set_ao(eAfeRamCodeDSGMOS, 1, &fFlase);
		}
		if(pstPackRVal->uBaseStat.stBaseStat.ucHeating != 0) {
			pstPackRVal->uBaseStat.stBaseStat.ucHeating = 0;
		}
//		LOCAL_RETURN_TRUE; //?????????????
	}
	/* Calculation of collected data and intermediate variables */
	local_info_refresh();
	/* Alarm and soft protection detection */
	pstPackRVal->ucChgEn = 0x55;
	pstPackRVal->ucDsgEn = 0x55;
//	if(g_bNeedHeat) {
//		pstPackRVal->ucHeatEn = 0x55;
//	} else {
//		pstPackRVal->ucHeatEn = 0xAA;
//	}
	local_signal_ptct();
	{
		static uint32_t s_uiTick = 0;
		if(pstPackRVal->fPackSoc < 10 && g_eBaseStat == eBStatStewing) {
			if(s_uiTick * g_stCfg.stLocal.usCyclePeriod >= 60*60*1000) {
				g_stCfg.usGoRun = 0;
				cfg_save();
				g_bNeedSleep = true;
				s_uiTick = 0;
			} else {
				s_uiTick++;
			}
		} else {
			s_uiTick = 0;
		}
	}
	{
		static uint16_t s_uiTick = 0;
		static uint16_t s_uiTick1 = 0;
		if(eco_refresh_log() && g_bNeedSleep != true) {
			s_uiTick++;
			if(s_uiTick * g_stCfg.stLocal.usCyclePeriod < 500) {
				SW_EN_LED_H;
			} else if(s_uiTick * g_stCfg.stLocal.usCyclePeriod >= 500 && s_uiTick * g_stCfg.stLocal.usCyclePeriod < 1500) {
				SW_EN_LED_L;
				s_uiTick = 0;
			}
		} else if(g_bNeedSleep == true) {
			s_uiTick1++;
			if(s_uiTick1 * g_stCfg.stLocal.usCyclePeriod < 500) {
				SW_EN_LED_H;
			} else if(s_uiTick1 * g_stCfg.stLocal.usCyclePeriod >= 500 && s_uiTick1 * g_stCfg.stLocal.usCyclePeriod < 1500) {
				SW_EN_LED_L;
				s_uiTick1 = 0;
			}
		} else {
			SW_EN_LED_H;
		}
	}
	/* TUI?? */
//	float fLmtChgI = g_stCfg.stLocal.usOCCTVThr1;
//	float fLmtDsgI = g_stCfg.stLocal.usODCTVThr1;
//	float fReqChgI = g_stCfg.stLocal.usOCCTVThr1;
//	float fPeakLmtDsgI = g_stCfg.stLocal.usODCTVThr1;
//	local_cur_lmt(&fLmtChgI, &fLmtDsgI, &fReqChgI, &fPeakLmtDsgI);
//	pstPackRVal->fLmtChgI = fLmtChgI;	/* Over-charge current threshold */
//	pstPackRVal->fLmtDsgI = fLmtDsgI;	/* Over-discharge current threshold */
//	pstPackRVal->fReqChgI = fReqChgI;
//	if(fReqChgI <= 0 && pstPackRVal->fPackSoc < 100 && pstPackRVal->ucChgEn == 0x55) {
//		pstPackRVal->fReqChgI = 0.1 * g_stCfg.stLocal.usDesignAH;
//	} else {
//		pstPackRVal->fReqChgI = fReqChgI;
//	}
//	pstPackRVal->fPeakLmtDsgI = fPeakLmtDsgI;
	/* TUI?? */
//	float fLmtChgI = g_stCfg.stLocal.usOCCTVThr1;
//	float fLmtDsgI = g_stCfg.stLocal.usODCTVThr1;
//	float fReqChgI = g_stCfg.stLocal.usOCCTVThr1;
//	float fPeakLmtDsgI = g_stCfg.stLocal.usODCTVThr1;
	float fLmtChgI = 2.5 * g_stCfg.stLocal.usDesignAH;
	float fLmtDsgI = 4 * g_stCfg.stLocal.usDesignAH;
	float fReqChgI = 2.5 * g_stCfg.stLocal.usDesignAH;
	float fPeakLmtDsgI = 4 * g_stCfg.stLocal.usDesignAH;
	local_cur_lmt(&fLmtChgI, &fLmtDsgI, &fReqChgI, &fPeakLmtDsgI);
	if(pstPackRVal->fLmtChgI > fLmtChgI) {
		pstPackRVal->fLmtChgI = fLmtChgI;	/* Over-charge current threshold */
	}
	if(pstPackRVal->fLmtDsgI > fLmtDsgI) {
		pstPackRVal->fLmtDsgI = fLmtDsgI;	/* Over-discharge current threshold */
	}
	if(fReqChgI <= 0 && pstPackRVal->fPackSoc < 100 && pstPackRVal->ucChgEn == 0x55 && g_ucChgFull == 0 && GET_ALM0_CODE(14) == 0 && GET_ALM1_CODE(14) == 0) {
		fReqChgI = 0.1 * g_stCfg.stLocal.usDesignAH;
	}
	if(pstPackRVal->fReqChgI > fReqChgI) {
		pstPackRVal->fReqChgI = fReqChgI;
	}
	if(pstPackRVal->fPeakLmtDsgI > fPeakLmtDsgI) {
		pstPackRVal->fPeakLmtDsgI = fPeakLmtDsgI;
	}
	
//	if(pstPackRVal->fCellUMax >= 3.36) {
		#define REQCHGI_TICK 80
		if(g_iReqChgITick < REQCHGI_TICK) {
			fReqChgI100[g_iReqChgITick] = pstPackRVal->fReqChgI;
			g_iReqChgITick++;
		} else if(g_iReqChgITick >= REQCHGI_TICK) {
			memcpy(fReqChgI100,fReqChgI100+1,sizeof fReqChgI100 - 4);
			fReqChgI100[79] = pstPackRVal->fReqChgI;
		}
//	}
	if(pstPackRVal->fCellUMax >= 3.37) {
		float fReqChgISum = 0;
		for(uint8_t i = 0; i < REQCHGI_TICK; i++) {
			fReqChgISum += fReqChgI100[i];
		}
		pstPackRVal->fReqChgI = fReqChgISum / REQCHGI_TICK;
		memcpy(fReqChgI100,fReqChgI100+1,sizeof fReqChgI100 - 4);
		fReqChgI100[79] = pstPackRVal->fReqChgI;
	}
	
	/* ECO FSM */
	//local_fsm();
	local_state_machine();
	/* MOS control */
	local_mos_ctl();
	
	LOCAL_RETURN_TRUE;
}
