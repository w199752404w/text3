#include "history.h"
#include "bsp_gd25q32.h"
#include "bsp_rtc.h"
#include "parallel.h"
#include "local.h"

#include <stdio.h>
#include <string.h>
#include <math.h>

HIS_WAVE_S g_stWave = {0};
uint16_t g_usHDataIdx;
uint16_t g_usHLogIdx;
uint8_t g_ucWaveIdx;
uint8_t  g_AlmVal = 0;
uint8_t g_AlmValold = 0;
extern bool g_bWriteWave;

/*******************************************************************************
* Function Name  : his_data_read
* Description    : Read historical data
* Input          : usIdx, The historical data number that needs to be read
* Output         : pstData, Historical data read
* Return         : result, 1 write succeeded, 0 write failed
*******************************************************************************/
bool his_data_read(uint16_t usIdx, HIS_DATA_S* pstData) {
	if(0 == pstData) {
		HIS_RETURN_FALSE;
	}
	MEM_FlashRead((((g_usHDataIdx + 5120 - usIdx - 1) % 5120) * 2 + eHisDataPStart) * PAGE_SIZE, (uint8_t*)pstData, sizeof(HIS_DATA_S));
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
* Description    : Write a page of historical data
* Input          : None
* Output         : None
* Return         : result, 1 write succeeded, 0 write failed
*******************************************************************************/
bool his_data_write(void) {
	if(g_stPrl.ucSelfId > PRL_MAX_NODE_NUM) {
		HIS_RETURN_FALSE;
	}
	HIS_DATA_S stHData;
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
	MEM_FlashWrite((g_usHDataIdx * 2 + eHisDataPStart) * PAGE_SIZE, (uint8_t*)&stHData, sizeof(HIS_DATA_S));
	if(g_usHDataIdx * 2 >= HIS_DATA_PLEN - 2) {
		g_usHDataIdx = 0;
	} else {
		g_usHDataIdx++;
	}
	
	HIS_RETURN_TRUE;
}

/*******************************************************************************
* Function Name  : his_data_clear
* Description    : Erase all historical data
* Input          : None
* Output         : None
* Return         : result, 1 write succeeded, 0 write failed
*******************************************************************************/
bool his_data_clear(void) {
	for(uint16_t i = eHisDataPStart / 16; i < (eHisDataPStart + HIS_DATA_PLEN) / 16; i++) {
		BSP_W25Qx_Erase_Sector(i);
//		WDG_DONE_H;
//		WDG_DONE_L;
		gd32_wdgt_feed_dog(EWdgType_fwdg);	//feed dog
	}
	g_usHDataIdx = 0;
	
	HIS_RETURN_TRUE;
}

/*******************************************************************************
* Function Name  : his_log_read
* Description    : Read a history log
* Input          : usIdx, You need to read the sequence number of the history log
* Output         : pstLog, A history log of reads
* Return         : result, 1 write succeeded, 0 write failed
*******************************************************************************/
bool his_log_read(uint16_t usIdx, HIS_LOG_S* pstLog) {
	if(0 == pstLog) {
		HIS_RETURN_FALSE;
	}
	MEM_FlashRead(((g_usHLogIdx + 8192 - usIdx - 1) % 8192) * PAGE_SIZE / 4 + PAGE_SIZE * eHisLogPStart, (uint8_t*)pstLog, sizeof(HIS_LOG_S));
	if(pstLog->dt == 0xFFFFFFFF || pstLog->dt == 0) {
		HIS_RETURN_FALSE;
	}
	
	HIS_RETURN_TRUE;
}

/*******************************************************************************
* Function Name  : his_log_write
* Description    : Write a history log
* Input          : None
* Output         : None
* Return         : result, 1 write succeeded, 0 write failed
*******************************************************************************/
bool his_log_write(void) {
	if(g_stCfg.stHis.ucLogEn == 0) {
		HIS_RETURN_FALSE;
	}
	HIS_LOG_S stLog;
	stLog.dt = DS1302_gtime();
	//stLog.stEcoLog = g_stEcoLog;
	MEM_FlashWrite(g_usHLogIdx * PAGE_SIZE / 4 + PAGE_SIZE * eHisLogPStart, (uint8_t*)&stLog, sizeof(HIS_LOG_S));
	if(g_usHLogIdx >= HIS_LOG_PLEN * 4 - 1) {
		g_usHLogIdx = 0;
	} else {
		g_usHLogIdx++;
	}
	
	HIS_RETURN_TRUE;
}

/*******************************************************************************
* Function Name  : his_log_clear
* Description    : Clear all historical logs
* Input          : None
* Output         : None
* Return         : result, 1 write succeeded, 0 write failed
*******************************************************************************/
bool his_log_clear(void) {
	for(uint16_t i = eHisLogPStart / 16; i < (eHisLogPStart + HIS_LOG_PLEN) / 16; i++) {
		BSP_W25Qx_Erase_Sector(i);
//		WDG_DONE_H;
//		WDG_DONE_L;
		gd32_wdgt_feed_dog(EWdgType_fwdg);	//feed dog
	}
	g_usHLogIdx = 0;
	
	HIS_RETURN_TRUE;
}

/*******************************************************************************
* Function Name  : his_wave_read
* Description    : The recording function service, which is called every time in the main loop, assumes that the main loop period is 50ms
* Input          : ucIdx, There are 4 recording files saved in Flash, 0~3 indicates the serial number of the recording file to be read, and 0xFF indicates the waveform in memory
								 : ucOffset, Due to the large content of the waveform file, it needs to be read step by step, and each read is 1024 bytes
* Output         : pucBuf, Fragments of the contents of the output waveform file
* Return         : result, 1 write succeeded, 0 write failed
*******************************************************************************/
bool his_wave_read(uint8_t ucIdx, uint8_t ucOffset, uint8_t* pucBuf) {
	if(0 == pucBuf) {
		HIS_RETURN_FALSE;
	}
	uint16_t usLen = sizeof(HIS_WAVE_S) - ucOffset * 512;
	if(usLen >= 512) {
		MEM_FlashRead((((g_ucWaveIdx + 8 - ucIdx - 1) % 8) * HIS_WAVE_PLEN / 8 + eHisWavePStart) * PAGE_SIZE + ucOffset * 512, pucBuf, 512);
	} else {
		MEM_FlashRead((((g_ucWaveIdx + 8 - ucIdx - 1) % 8) * HIS_WAVE_PLEN / 8 + eHisWavePStart) * PAGE_SIZE + ucOffset * 512, pucBuf, usLen);
	}
	HIS_RETURN_TRUE;
}

/*******************************************************************************
* Function Name  : his_wave_write
* Description    : Write history waveforms
* Input          : None
* Output         : None
* Return         : result, 1 write succeeded, 0 write failed
*******************************************************************************/
bool his_wave_write(void) {
	MEM_FlashWrite((eHisWavePStart + HIS_WAVE_PLEN / 8 * g_ucWaveIdx) * PAGE_SIZE, (uint8_t*)&g_stWave, sizeof(HIS_WAVE_S));
	if(g_ucWaveIdx >= 8) {
		g_ucWaveIdx = 0;
	} else {
		g_ucWaveIdx++;
	}
	
	HIS_RETURN_TRUE;
}

/*******************************************************************************
* Function Name  : his_wave_clear
* Description    : Clear all historical waveforms
* Input          : None
* Output         : None
* Return         : result, 1 write succeeded, 0 write failed
*******************************************************************************/
//bool his_wave_clear(void) {
//	for(uint8_t i=0;i<4;i++) {
//		BSP_W25Qx_Erase_Sector((eHisWavePStart + HIS_WAVE_PLEN / 4 * i) / 16);
//	}
//	g_ucWaveIdx = 0;
//	
//	HIS_RETURN_TRUE;
//}

/*******************************************************************************
* Function Name  : his_wave_proc
* Description    : The recording function service is called in the main loop, assuming that the main loop period is 50ms, and it is called once in 1s
* Input          : None
* Output         : None
* Return         : result, 1 write succeeded, 0 write failed
*******************************************************************************/
bool his_wave_proc(void) {
	g_stWave.dt = DS1302_gtime();
	//g_stWave.astEcoRtv[g_stWave.usIdx] = g_stEcoRtv;
	g_stWave.usIdx++;
	if(g_stWave.usIdx >= WAVE_PEICES) {
	g_stWave.usIdx = 0;
    //g_AlmVal = eco_refresh_log();
//		if(g_AlmVal != g_AlmValold && g_AlmVal > 0) {
//			g_bWriteWave = true;
//		}
		if(1){
			//g_bWriteWave = false;
		  his_wave_write();
	  }
	}
	g_AlmValold = g_AlmVal;
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
//	if(!his_wave_clear()) {
//		HIS_RETURN_FALSE;
//	}
	g_usHDataIdx = 0;
	g_usHLogIdx = 0;
	g_ucWaveIdx = 0;
	
	HIS_RETURN_TRUE;
}

/*******************************************************************************
* Function Name  : his_init
* Description    : Historical service initialization
* Input          : None
* Output         : None
* Return         : result, 1 write succeeded, 0 write failed
*******************************************************************************/
bool his_init(void) {
	uint8_t aucDt1[4] = {0};
	uint8_t aucDt2[4] = {0};
	bool ucFound;
	/* Look for where historical data is written */
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
	/* Look for the location where the history log is written */
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
	g_ucWaveIdx = 0;
	memset(aucDt1, 0, 4);
	for(uint8_t i=0;i<8;i++) {
		MEM_FlashRead((eHisWavePStart + i * HIS_WAVE_PLEN / 8) * PAGE_SIZE, aucDt2, 4);
		if(0xFF == aucDt2[0] && 0xFF == aucDt2[1] && 0xFF == aucDt2[2] && 0xFF == aucDt2[3]) {
			g_ucWaveIdx = i;
			break;
		}
		if(*(uint32_t*)aucDt2 < *(uint32_t*)aucDt1) {
			g_ucWaveIdx = i;
			break;
		}
		memcpy(aucDt1, aucDt2, 4);
	}
	/* Clear the waveform cache */
	memset(&g_stWave, 0, sizeof(HIS_WAVE_S));
	
	HIS_RETURN_TRUE;
}

/*******************************************************************************
* Function Name  : his_proc
* Description    : Historical information service, every time the main loop is called
* Input          : None
* Output         : None
* Return         : result, 1 write succeeded, 0 write failed
*******************************************************************************/
bool his_proc(void) {
	LOCAL_PACK_RVAL_S* pstPackRVal = g_stLocalArrayRVal.astPackRVal + g_stPrl.ucSelfId;
	if(pstPackRVal->fPackCur < -2 && g_eBaseStat == eBStatWorking) {
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
			his_wave_proc();
			s_uiTick2 = 0;
		} else {
			s_uiTick2++;
		}
	}

	HIS_RETURN_TRUE;
}
