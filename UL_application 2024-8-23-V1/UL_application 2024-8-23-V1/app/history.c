#include "history.h"
#include "bsp_gd25q32.h"
#include "bsp_rtc.h"
#include "parallel.h"
#include "local.h"

#include <stdio.h>
#include <string.h>
#include <math.h>


uint16_t g_usHDataIdx;
uint16_t g_usHLogIdx;
uint8_t g_ucWaveIdx;
uint8_t  g_AlmVal = 0;
uint8_t g_AlmValold = 0;
extern bool g_bWriteWave;

/*******************************************************************************
* Function Name  : his_data_read
* Description    : 读取历史数据
* Input          : usPStart, 需要读取的历史数据的页号
* Output         : pstData, 读取的历史数据
* Return         : result, 1写成功 0写失败
*******************************************************************************/
bool his_data_read(uint16_t usIdx, HIS_DATA_S* pstData) {
	if(0 == pstData) {
		HIS_RETURN_FALSE;
	}
	//MEM_FlashRead((((g_usHDataIdx + 5120 - usIdx - 1) % 5120) * 2 + eHisDataPStart) * PAGE_SIZE, (uint8_t*)pstData, sizeof(HIS_DATA_S));
	MEM_FlashRead((usIdx + eHisDataPStart) * PAGE_SIZE, (uint8_t*)pstData, sizeof(HIS_DATA_S));
    if(pstData->dt ==0xFFFFFFFF || pstData->dt == 0) {
		HIS_RETURN_FALSE;
	}
//	for(uint8_t i = 0; i < CFG_CELL_NUM; i++) {
//		if(pstData->ausCellRealAh[i] <= 0) {
//			HIS_RETURN_FALSE;
//		}
//		if(pstData->stRtv.ausCellRTVolt[i] == 0) {
//			HIS_RETURN_FALSE;
//		}
//	}
	
	HIS_RETURN_TRUE;
}

/*******************************************************************************
* Function Name  : his_data_write
* Description    : 写入一页历史数据
* Input          : None
* Output         : None
* Return         : result, 1写成功 0写失败
*******************************************************************************/
bool his_data_write(void) {
	if(g_stPrl.ucSelfId > PRL_MAX_NODE_NUM) {
		HIS_RETURN_FALSE;
	}
	HIS_DATA_S stHData;
    //LOCAL_PACK_RVAL_S* pstRVal = &g_stLocalArrayRVal.astPackRVal[g_stPrl.ucSelfId];
    LOCAL_PACK_RVAL_S* pstRVal = &g_stLocalPackRVal[g_stPrl.ucSelfId];
	stHData.dt = DS1302_gtime();
	//stHData.stRtv = g_stEcoRtv;
//	if(stHData.stRtv.ausCellRTVolt[0] == 0) {
//		HIS_RETURN_FALSE;
//	}
//	stHData.state = g_eBaseStat;
//	for(uint8_t i=0;i<CFG_CELL_NUM;i++) {
//		stHData.ausCellRealAh[i] = g_stLocalArrayRVal.astPackRVal[g_stPrl.ucSelfId].afCellRealAH[i];
//	}
//	for(uint8_t i=0;i<CFG_CELL_NUM;i++) {
//		stHData.aucCellSoc[i] = round(g_stLocalArrayRVal.astPackRVal[g_stPrl.ucSelfId].afCellLeftAH[i] * 100 / stHData.ausCellRealAh[i]);
//	}
    
    
    stHData.usPackVol = (uint16_t)(pstRVal->astPackReport.fPackU * 10);
	stHData.usPackCur = (uint16_t)(pstRVal->astPackReport.fPackCur * 10) + 3000;
	//stHData.usPackEvnT = (uint16_t)(pstRVal->fEnvT * 10) + 100;
	stHData.ucPackSoc = (uint16_t)pstRVal->astPackReport.fPackSoc;
	stHData.ucPackSoh = (uint16_t)pstRVal->astPackReport.fPackSoh;
	stHData.usPackOV = (uint16_t)(pstRVal->astPackReport.fOV * 10);
	stHData.usPackUV = (uint16_t)(pstRVal->astPackReport.fUV * 10);
	stHData.usPackOCC = (uint16_t)(pstRVal->astPackReport.fOCC * 10) + 3000;
	stHData.usPackODC = (uint16_t)(pstRVal->astPackReport.fODC * 10) + 3000;
	stHData.usCellMaxVol = (uint16_t)(pstRVal->astPackReport.fCellUMax * 1000);
	stHData.usCellMinVol = (uint16_t)(pstRVal->astPackReport.fCellUMin * 1000);
	stHData.usCellMaxVolId = pstRVal->astPackReport.ucCellUMaxId;
	stHData.usCellMinVolId = pstRVal->astPackReport.ucCellUMinId;
	stHData.usCellMaxT = (uint16_t)(pstRVal->astPackReport.fCellTMax * 10) + 100;
	stHData.usCellMinT = (uint16_t)(pstRVal->astPackReport.fCellTMin * 10) + 100;
	stHData.usCellMaxTId = pstRVal->astPackReport.ucCellTMaxId;
	stHData.usCellMinTId = pstRVal->astPackReport.ucCellTMinId;
	//stHData.ucBaseStat = pstRVal->uBaseStat.stBaseStat;
	stHData.usCycle= pstRVal->astPackReport.usCycle;
	stHData.ucErrCode = pstRVal->astPackReport.uErrCode.ucErrCode;
	stHData.usAlmCode = pstRVal->astPackReport.uAlmCode.usAlmCode;
	stHData.usPtctCode = pstRVal->astPackReport.uPtctCode.usPtctCode;
	stHData.ucChgEn = pstRVal->astPackReport.ucChgEn;
	stHData.ucDsgEn = pstRVal->astPackReport.ucDsgEn;
	stHData.ucErrCodeEx = pstRVal->astPackReport.ucErrCodeEx;
	for(uint8_t i=0;i<CFG_CELL_NUM;i++) {
		stHData.usCellVol[i] = (uint16_t)(pstRVal->astPackReport.afCellU[i] * 1000);
	}
	for(uint8_t i=0;i<CFG_CELL_NUM;i++) {
		stHData.usCellSoc[i] = pstRVal->afCellSoc[i];
	}
	for(uint8_t i=0;i<CFG_CELL_NUM;i++) {
		stHData.usCellSoh[i] = pstRVal->afCellSoh[i];
	}
	for(uint8_t i=0;i<CFG_CELL_NUM;i++) {
		stHData.usCellT[i] = (uint16_t)(pstRVal->astPackReport.afCellT[i] * 10) + 100;
	}
	stHData.usMosT = (uint16_t)(pstRVal->astPackReport.fMosT * 10) + 100;
    
    
    
    MEM_FlashWrite((g_usHDataIdx + eHisDataPStart) * PAGE_SIZE, (uint8_t*)&stHData, sizeof(HIS_DATA_S));

	if(g_usHDataIdx >= HIS_DATA_PLEN - 1) {
		g_usHDataIdx = 0;
	} else {
		g_usHDataIdx++;
	}
	
	HIS_RETURN_TRUE;
}

/*******************************************************************************
* Function Name  : his_data_clear
* Description    : 清除所有历史数据
* Input          : None
* Output         : None
* Return         : result, 1写成功 0写失败
*******************************************************************************/
bool his_data_clear(void) {
	for(uint16_t i = eHisDataPStart / 16; i < (eHisDataPStart + HIS_DATA_PLEN) / 16; i++) {
		BSP_W25Qx_Erase_Sector(i);
		gd32_wdgt_feed_dog(EWdgType_fwdg);	//feed dog
	}
	g_usHDataIdx = 0;
	
	HIS_RETURN_TRUE;
}

/*******************************************************************************
* Function Name  : his_log_read
* Description    : 读取一条历史日志
* Input          : usIdx, 需要读取历史日志的序号
* Output         : pstLog, 读取的历史日志
* Return         : result, 1写成功 0写失败
*******************************************************************************/
bool his_log_read(uint16_t usIdx, HIS_LOG_S* pstLog) {
	if(0 == pstLog) {
		HIS_RETURN_FALSE;
	}
	MEM_FlashRead(usIdx * PAGE_SIZE / 4 + PAGE_SIZE * eHisLogPStart, (uint8_t*)pstLog, sizeof(HIS_LOG_S));
	if(pstLog->dt == 0xFFFFFFFF || pstLog->dt == 0) {
		HIS_RETURN_FALSE;
	}
	
	HIS_RETURN_TRUE;
}

/*******************************************************************************
* Function Name  : his_log_write
* Description    : 写入一条历史日志
* Input          : pstLog, 需要写入历史日志
* Output         : None
* Return         : result, 1写成功 0写失败
*******************************************************************************/
bool his_log_write(HIS_LOG_S* pstLog) {
	if(0 == pstLog) {
		HIS_RETURN_FALSE;
	}
	MEM_FlashWrite(g_usHLogIdx * PAGE_SIZE / 4 + PAGE_SIZE * eHisLogPStart, (uint8_t*)pstLog, sizeof(HIS_LOG_S));
	if(g_usHLogIdx >= HIS_LOG_PLEN * 4 - 1) {
		g_usHLogIdx = 0;
	} else {
		g_usHLogIdx++;
	}
	
	HIS_RETURN_TRUE;
}

/*******************************************************************************
* Function Name  : his_log_clear
* Description    : 清除所有历史日志
* Input          : None
* Output         : None
* Return         : result, 1写成功 0写失败
*******************************************************************************/
bool his_log_clear(void) {
	for(uint16_t i = eHisLogPStart / 16; i < (eHisLogPStart + HIS_LOG_PLEN) / 16; i++) {
		BSP_W25Qx_Erase_Sector(i);
		gd32_wdgt_feed_dog(EWdgType_fwdg);	//feed dog
	}
	g_usHLogIdx = 0;
	
	HIS_RETURN_TRUE;
}

/*******************************************************************************
* Function Name  : his_clear
* Description    : Clear all historical information
* Input          : None
* Output         : None
* Return         : result, 1 write succeeded, 0 write failed
*******************************************************************************/
bool his_clear(void) {
	if(!his_data_clear()) {
		HIS_RETURN_FALSE;
	}
	if(!his_log_clear()) {
		HIS_RETURN_FALSE;
	}

	g_usHDataIdx = 0;
	g_usHLogIdx = 0;
	g_ucWaveIdx = 0;
	
	HIS_RETURN_TRUE;
}

/*******************************************************************************
* Function Name  : his_init
* Description    : 历史服务初始化
* Input          : None
* Output         : None
* Return         : result, 1写成功 0写失败
*******************************************************************************/
bool his_init(void) {
	uint8_t aucDt1[4] = {0};
	uint8_t aucDt2[4] = {0};
	bool ucFound;
	/* 寻找历史数据写入位置 */
	g_usHDataIdx = 0;
	for(uint16_t i=0;i<HIS_DATA_PLEN / 2;i++) {
		MEM_FlashRead((eHisDataPStart + i * 2) * PAGE_SIZE, aucDt2, 4);
		if(0xFF == aucDt2[0] && 0xFF == aucDt2[1] && 0xFF == aucDt2[2] && 0xFF == aucDt2[3]) {
			g_usHDataIdx = i;
			break;
		}
		if(*(uint32_t*)aucDt2 < *(uint32_t*)aucDt1 && *(uint32_t*)aucDt2 > 1000000000) {
			g_usHDataIdx = i;
			break;
		}
		memcpy(aucDt1, aucDt2, 4);
	}
	/* 寻找历史日志写入位置 */
	ucFound = false;
	memset(aucDt1, 0, 4);
	g_usHLogIdx = 0;
	for(uint16_t i=eHisLogPStart;i<eHisLogPStart + HIS_LOG_PLEN;i++) {
		if(ucFound) {
			break;
		}
		for(uint8_t j=0;j<4;j++) {
			MEM_FlashRead(i * PAGE_SIZE + PAGE_SIZE / 4 * j, aucDt2, 4);
			if(0xFF == aucDt2[0] && 0xFF == aucDt2[1] && 0xFF == aucDt2[2] && 0xFF == aucDt2[3]) {
				g_usHLogIdx = (i - eHisLogPStart) * 4 + j;
				ucFound = true;
				break;
			}
			if(*(uint32_t*)aucDt2 < *(uint32_t*)aucDt1) {
				g_usHLogIdx = (i - eHisLogPStart) * 4 + j;
				ucFound = true;
				break;
			}
			memcpy(aucDt1, aucDt2, 4);
		}
	}
	/* Find the location where the recording file is written */
//	g_ucWaveIdx = 0;
//	memset(aucDt1, 0, 4);
//	for(uint8_t i=0;i<8;i++) {
//		MEM_FlashRead((eHisWavePStart + i * HIS_WAVE_PLEN / 8) * PAGE_SIZE, aucDt2, 4);
//		if(0xFF == aucDt2[0] && 0xFF == aucDt2[1] && 0xFF == aucDt2[2] && 0xFF == aucDt2[3]) {
//			g_ucWaveIdx = i;
//			break;
//		}
//		if(*(uint32_t*)aucDt2 < *(uint32_t*)aucDt1) {
//			g_ucWaveIdx = i;
//			break;
//		}
//		memcpy(aucDt1, aucDt2, 4);
//	}
	/* Clear the waveform cache */
	//memset(&g_stWave, 0, sizeof(HIS_WAVE_S));
	
	HIS_RETURN_TRUE;
}

/*******************************************************************************
* Function Name  : his_proc
* Description    : 历史信息服务, 每次主循环调用
* Input          : None
* Output         : None
* Return         : result, 1写成功 0写失败
*******************************************************************************/
bool his_proc(void) {
	//LOCAL_PACK_RVAL_S* pstPackRVal = g_stLocalArrayRVal.astPackRVal + g_stPrl.ucSelfId;
	LOCAL_PACK_RVAL_S* pstPackRVal = &g_stLocalPackRVal[g_stPrl.ucSelfId];
    if(pstPackRVal->astPackReport.fPackCur < -2 && g_eBaseStat == eBStatWorking) {
		if(g_stCfg.stHis.ucDataEn) {//When discharging, it is recorded once a minute
			static uint32_t s_uiTick3 = 0;
			if(s_uiTick3 * g_stCfg.stLocal.usCyclePeriod >= g_stCfg.stLocal.usCyclePeriod * 200) {
				s_uiTick3 = 0;
				his_data_write();
			} else {
				s_uiTick3++;
			}
		}
	} else if(g_eBaseStat == eBStatStewing) {//In the standing state, it is recorded once an hour
		if(g_stCfg.stHis.ucDataEn) {
			static uint32_t s_uiTick1 = 0;
			if(s_uiTick1 * g_stCfg.stLocal.usCyclePeriod >= g_stCfg.stHis.usDataPeriod * 60000 * 12) {
				s_uiTick1 = 0;
				his_data_write();
			} else {
				s_uiTick1++;
			}
		}
	}	else {
		if(g_stCfg.stHis.ucDataEn) {
			static uint32_t s_uiTick1 = 0;
			if(s_uiTick1 * g_stCfg.stLocal.usCyclePeriod >= g_stCfg.stHis.usDataPeriod * 5000) {
				s_uiTick1 = 0;
				his_data_write();
			} else {
				s_uiTick1++;
			}
		}
	}
//	if(g_stCfg.stHis.ucLogEn) {
//		static uint32_t s_uiTick = 0;
//		if(s_uiTick * g_stCfg.stLocal.usCyclePeriod > g_stCfg.stHis.usLogPeriod * 1000) {
//			s_uiTick = 0;
//			his_log_write();
//		} else {
//			s_uiTick++;
//		}
//	}
	if(g_stCfg.stHis.ucWaveEn) {
		static uint32_t s_uiTick2 = 0;
		if(s_uiTick2 == 19) {
			//his_wave_proc();
			s_uiTick2 = 0;
		} else {
			s_uiTick2++;
		}
	}

	HIS_RETURN_TRUE;
}
