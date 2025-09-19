#include "local.h" 


#include "modbus_rtu.h"
#include "history.h"
#include "bsp_rtc.h"
//#include "parallel.h"
//#include "protocol.h"
#include "bsp_adc.h"
#include "bsp_gpio.h"
//#include "bsp_exti.h"
#include "bsp_sh367303.h"

//#include "parallel.h"

#include <stdio.h>
#include <string.h>
#include <math.h>

LOCAL_PACK_RVAL_S g_stLocalPackRVal[PRL_MAX_NODE_NUM] = {0};;


LOCAL_STAT_E g_stLocalStat;


LOCAL_FLAG_S g_stLocalFlagWVal = {0};
//LOCAL_ARRAY_RVAL_S g_stLocalArrayRVal = {0};
BASE_STATU_E g_eBaseStat;


uint8_t g_ucOCCTick = 0;	/* 连续充电过流次数 */
uint8_t g_ucOCDTick = 0;	/* 连续放电过流次数 */

/*******************************************************************************
* Function Name  : local_pch2chg
* Description    : 预放和主放的通断
* Input          : ucChgEn, 0x55-允许充电, 0xAA-禁止充电
									 ucDsgEn, 0x55-允许放电, 0xAA-禁止放电
* Output         : None
* Return         : None
*******************************************************************************/
void local_pch2chg (uint8_t ucChgEn, uint8_t ucDsgEn) {
	float fTrue = 1;
	float fFalse = 0;
	if(ucChgEn == 0xAA && g_stAfe.uRom.stCode.CHG_G == 1) {
		afe_set_ao(eAfeRomCodeCHG_C, 1, &fFalse);
	}
	if(ucChgEn == 0x55 && g_stAfe.uRom.stCode.CHG_G == 0) {
		afe_set_ao(eAfeRomCodeCHG_C, 1, &fTrue);
	}
	static uint8_t s_ucTick = 0;
	if(ucDsgEn == 0xAA ) {
		PRE_DSG_L;
		s_ucTick = 0;
		LOCAL_DEBUG("Pre dsg disabled");
	}
	if(ucDsgEn == 0xAA && g_stAfe.uRom.stCode.DSG_G == 1) {
		afe_set_ao(eAfeRomCodeDSG_C, 1, &fFalse);
	}
	if(ucDsgEn == 0x55 && g_stAfe.uRom.stCode.DSG_G == 0) {
		if(g_stCfg.stLocal.ucPDSGDly > 0) {
			PRE_DSG_H;
			LOCAL_DEBUG("Pre dsg enabled");
		}
		if(s_ucTick < g_stCfg.stLocal.ucPDSGDly * 1000 / g_stCfg.stLocal.usCyclePeriod ) {
			s_ucTick++;
		}
		if(s_ucTick >= g_stCfg.stLocal.ucPDSGDly * 1000 / g_stCfg.stLocal.usCyclePeriod) {
			afe_set_ao(eAfeRomCodeDSG_C, 1, &fTrue);
		}
	}
	if(g_stAfe.uRom.stCode.DSG_G == 1) {
		PRE_DSG_L;
		s_ucTick = 0;
		LOCAL_DEBUG("Pre dsg disabled");
	}
}



/*******************************************************************************
* Function Name  : local_init
* Description    : 本机服务初始化, 值得注意的是, 这里所有数据的刷新, 都必须具备原子操作性质, 因为CAN等外部中断会打断本函数并随时获取中间处理状态
* Input          : None
* Output         : None
* Return         : result, 1写成功 0写失败
*******************************************************************************/
bool local_init(void) {
//    LOCAL_RETURN_TRUE;

	LOCAL_PACK_RVAL_S* pstPackRVal = &g_stLocalPackRVal[g_stPrl.ucSelfId];
	HIS_DATA_S stHData;
	his_data_read((HIS_DATA_PLEN + g_usHDataIdx - 1) % HIS_DATA_PLEN, &stHData);
	for(uint8_t i=0;i<CFG_CELL_NUM;i++) {
		/* 电芯剩余容量, 实际容量, Soc, Soh初始化 */
		if(stHData.dt != 0xFFFFFFFF) {
			//pstPackRVal->afCellRealAH[i] = stHData.usCellSoh[i] * g_stCfg.stLocal.usDesignAH * 0.01;
			//pstPackRVal->afCellLeftAH[i] = stHData.usCellSoc[i] * pstPackRVal->afCellRealAH[i] * 0.01;
			//pstPackRVal->afCellSoc[i] = stHData.usCellSoc[i];
			//pstPackRVal->usCycle = stHData.usCycle;
		} else {
			pstPackRVal->afCellRealAH[i] = g_stCfg.stLocal.usDesignAH;
			pstPackRVal->afCellLeftAH[i] = g_stCfg.stLocal.usDesignAH * 0.5;
			pstPackRVal->afCellSoc[i] = 50;
			pstPackRVal->astPackReport.usCycle = 0;
		}
	}
	float fPackDsgAH = pstPackRVal->afCellLeftAH[0];
	float fPackChgAH = pstPackRVal->afCellRealAH[0] - pstPackRVal->afCellLeftAH[0];
	for(uint8_t i=1;i<CFG_CELL_NUM;i++) {
		if(fPackDsgAH > pstPackRVal->afCellLeftAH[i]) {
			fPackDsgAH = pstPackRVal->afCellLeftAH[i];
		}
		if(fPackChgAH > pstPackRVal->afCellRealAH[i] - pstPackRVal->afCellLeftAH[i]) {
			fPackChgAH = pstPackRVal->afCellRealAH[i] - pstPackRVal->afCellLeftAH[i];
		}
	}
	pstPackRVal->astPackReport.fPackSoc = fPackDsgAH * 100 / (fPackDsgAH + fPackChgAH);
	LOCAL_RETURN_TRUE;
    
}

void local_set_soc(float fVal, uint8_t ucIdx) {
	float fSoc = 0;
	if(ucIdx > CFG_CELL_NUM) {
		return;
	} else if(ucIdx == CFG_CELL_NUM) {
		fSoc = g_stLocalPackRVal[g_stPrl.ucSelfId].astPackReport.fPackSoc;
	} else {
		fSoc = g_stLocalPackRVal[g_stPrl.ucSelfId].afCellSoc[ucIdx];
	}
	if(fSoc == 100 && fVal > 99.5) {
		return;
	}
	if(fSoc == 0 && fVal < 0.5) {
		return;
	}
	float fSocBak = fSoc;
	if(fVal - fSoc > 0.25) {
		fSoc += 0.25;
	} else if(fVal - fSoc < -0.25) {
		fSoc -= 0.25;
	} else {
		fSoc = fVal;
	}
	if(fSoc > 100) {
		fSoc = 100;
	}
	if(fSoc < 0) {
		fSoc = 0;
	}
	if(fSocBak < 100 && fSoc >=99) {
		fSoc = 99;
	} else if(fSocBak > 0 && fSoc <= 1) {
		fSoc = 1;
	} else {
		fSoc = fSoc;
	}
	if(ucIdx == CFG_CELL_NUM) {
		g_stLocalPackRVal[g_stPrl.ucSelfId].astPackReport.fPackSoc = fSoc;
	} else {
		g_stLocalPackRVal[g_stPrl.ucSelfId].afCellSoc[ucIdx] = fSoc;
	}
}

uint8_t local_get_soc(uint8_t ucIdx) {
	if(ucIdx > 16) {
		return 0;
	} else if(ucIdx == 16) {
		float fSoc = g_stLocalPackRVal[g_stPrl.ucSelfId].astPackReport.fPackSoc;
		if(fSoc - (uint8_t)fSoc > 0.5) {
			return fSoc + 1;
		} else {
			return fSoc;
		}
	} else {
		float fSoc = g_stLocalPackRVal[g_stPrl.ucSelfId].afCellSoc[ucIdx];
		if(fSoc - (uint8_t)fSoc > 0.5) {
			return fSoc + 1;
		} else {
			return fSoc;
		}
	}
}


/*******************************************************************************
* Function Name  : local_refresh
* Description    : 采集数据和计算
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void local_refresh(void) {
	LOCAL_PACK_RVAL_S* pstPackRVal = &g_stLocalPackRVal[g_stPrl.ucSelfId];
    ERR_CODE_U uErrCode = {0};
    float fPackU = 0;
    uint8_t ucCellUMaxId = 0;
	uint8_t ucCellUMinId = 0;
    /* 电流更新 */
   	if(g_stAfe.stRamApp.fCUR <= LOCAL_CUR_DZ && g_stAfe.stRamApp.fCUR >= -LOCAL_CUR_DZ) {
		pstPackRVal->astPackReport.fPackCur = 0;
	} else {
		pstPackRVal->astPackReport.fPackCur = g_stAfe.stRamApp.fCUR * 0.01;
	} 
    float fDeltaAH = pstPackRVal->astPackReport.fPackCur * g_stCfg.stLocal.usCyclePeriod / 1000 / 3600;
    for(uint8_t i=0;i<CFG_CELL_NUM;i++) {
        /* 电芯电压更新 */
		pstPackRVal->astPackReport.afCellU[i] = g_stAfe.stRamApp.fCELL[i];
        /* 电压传感器状态更新 */
        if(pstPackRVal->astPackReport.afCellU[i] > 5000 || pstPackRVal->astPackReport.afCellU[i] < 1000) {
			uErrCode.stErrCode.bVoltSensor = 1;
		}
        fPackU += pstPackRVal->astPackReport.afCellU[i];
        /* 最高最低电芯电压更新 */
		if(pstPackRVal->astPackReport.afCellU[ucCellUMaxId] < pstPackRVal->astPackReport.afCellU[i]) {
			ucCellUMaxId = i;
		}
		if(pstPackRVal->astPackReport.afCellU[ucCellUMinId] > pstPackRVal->astPackReport.afCellU[i]) {
			ucCellUMinId = i;
		}
        
        /* 方法: 用充放电Ah积分换算成充放电次数, 再用充放电次数映射Soh */
        if(pstPackRVal->astPackReport.usCycle <= 50) {
			pstPackRVal->afCellSoh[i] = 100;
		} else if(pstPackRVal->astPackReport.usCycle > 50 && pstPackRVal->astPackReport.usCycle <= 500) {
			pstPackRVal->afCellSoh[i] = 100 - (pstPackRVal->astPackReport.usCycle - 50) / 450 * 5;
		} else if(pstPackRVal->astPackReport.usCycle > 500 && pstPackRVal->astPackReport.usCycle <= 1000) {
			pstPackRVal->afCellSoh[i] = 95 - (pstPackRVal->astPackReport.usCycle - 500) / 500 * 5;
		} else if(pstPackRVal->astPackReport.usCycle > 1000 && pstPackRVal->astPackReport.usCycle <= 1500) {
			pstPackRVal->afCellSoh[i] = 90 - (pstPackRVal->astPackReport.usCycle - 1000) / 500 * 5;
		} else if(pstPackRVal->astPackReport.usCycle > 1500 && pstPackRVal->astPackReport.usCycle <= 2000) {
			pstPackRVal->afCellSoh[i] = 85 - (pstPackRVal->astPackReport.usCycle - 1500) / 500 * 5;
		} else if(pstPackRVal->astPackReport.usCycle > 2000 && pstPackRVal->astPackReport.usCycle <= 3000) {
			pstPackRVal->afCellSoh[i] = 80 - (pstPackRVal->astPackReport.usCycle - 2000) / 1000 * 10;
		} else {
			pstPackRVal->afCellSoh[i] = 70;
		}
	}

    pstPackRVal->astPackReport.fPackU = fPackU;
    pstPackRVal->astPackReport.ucCellUMaxId = ucCellUMaxId+1;
	pstPackRVal->astPackReport.ucCellUMinId = ucCellUMinId+1;
	pstPackRVal->astPackReport.fCellUMax = pstPackRVal->astPackReport.afCellU[ucCellUMaxId];
	pstPackRVal->astPackReport.fCellUMin = pstPackRVal->astPackReport.afCellU[ucCellUMinId];
    
    
    
    
    
    /* 电芯温度更新 */
    uint8_t ucCellTMaxId = 0;
	uint8_t ucCellTMinId = 0;
    for(uint8_t i=0;i<2;i++) {
        pstPackRVal->astPackReport.afCellT[i] = g_stAfe.stRamApp.fTS[i];
    }
    pstPackRVal->astPackReport.afCellT[2]= ADC_Get_Temp(g_auiAdcBuf[0]);
    pstPackRVal->astPackReport.afCellT[3] = ADC_Get_Temp(g_auiAdcBuf[2]);
    
    for(uint8_t i=0;i < 4;i++) {
        if(pstPackRVal->astPackReport.afCellT[i] >= 110 || pstPackRVal->astPackReport.afCellT[i] < -50) {
            uErrCode.stErrCode.bTempSensor = 1;	/* 温度传感器故障 */
        }
        
        /* 最高最低电芯温度更新 */
		if(pstPackRVal->astPackReport.afCellT[ucCellUMaxId] < pstPackRVal->astPackReport.afCellT[i]) {
			ucCellUMaxId = i;
		}
		if(pstPackRVal->astPackReport.afCellT[ucCellUMinId] > pstPackRVal->astPackReport.afCellT[i]) {
			ucCellUMinId = i;
		}
    }
    //ucCellTMaxId = g_stCfg.stLocal.ucSVerRMinor;
    pstPackRVal->astPackReport.ucCellTMaxId = ucCellTMaxId+1;
	pstPackRVal->astPackReport.ucCellTMinId = ucCellTMinId+1;
	pstPackRVal->astPackReport.fCellTMax = pstPackRVal->astPackReport.afCellT[ucCellTMaxId];
	pstPackRVal->astPackReport.fCellTMin = pstPackRVal->astPackReport.afCellT[ucCellTMinId];

    /* MOS温度更新 */
	pstPackRVal->astPackReport.fMosT = ADC_Get_Temp(g_auiAdcBuf[1]);
    if(pstPackRVal->astPackReport.fMosT > 150 || pstPackRVal->astPackReport.fMosT < -50) {
		uErrCode.stErrCode.bTempSensor = 1;	/* 温度传感器故障 */
	}

    /* 环境温度更新 */
	pstPackRVal->astPackReport.fEnvT = ADC_Get_Temp(g_auiAdcBuf[1]);
    if(pstPackRVal->astPackReport.fEnvT > 150 || pstPackRVal->astPackReport.fEnvT < -50) {
		uErrCode.stErrCode.bTempSensor = 1;	/* 温度传感器故障 */
	}
    
    
    
    /* PACK剩余容量更新 */
	float fPackLeftAH = pstPackRVal->afCellLeftAH[0];
	for(uint8_t i=1;i<CFG_CELL_NUM;i++) {
		if(fPackLeftAH > pstPackRVal->afCellLeftAH[i]) {
			fPackLeftAH = pstPackRVal->afCellLeftAH[i];
		}
	}
	pstPackRVal->astPackReport.fPackLeftAH = fPackLeftAH;
    /* PACK实际额定容量更新 */
	float fChgAH = pstPackRVal->afCellRealAH[0] - pstPackRVal->afCellLeftAH[0];
	for(uint8_t i=1;i<CFG_CELL_NUM;i++) {
		if(fChgAH > pstPackRVal->afCellRealAH[i] - pstPackRVal->afCellLeftAH[i]) {
			fChgAH = pstPackRVal->afCellRealAH[i] - pstPackRVal->afCellLeftAH[i];
		}
	}
    /* PACK Soh更新 */
    float fPackSoh = pstPackRVal->afCellSoh[0];
	for(uint8_t i=1;i<CFG_CELL_NUM;i++) {
		if(fPackSoh > pstPackRVal->afCellSoh[i]) {
			fPackSoh = pstPackRVal->afCellSoh[i];
		}
	}
    
    pstPackRVal->astPackReport.fPackSoh = fPackSoh;
    
    /* Pack真实电容量更新 */
    pstPackRVal->astPackReport.fPackRealAH = g_stCfg.stLocal.usDesignAH * pstPackRVal->astPackReport.fPackSoh /100 ;
    for(uint8_t i=0;i<CFG_CELL_NUM;i++) {
    pstPackRVal->afCellRealAH[i] = pstPackRVal->astPackReport.fPackRealAH;
    }
    if(pstPackRVal->astPackReport.fPackRealAH < g_stCfg.stLocal.usDesignAH * 0.7 ) {
            pstPackRVal->astPackReport.fPackRealAH = g_stCfg.stLocal.usDesignAH * 0.7;
    }
    if(pstPackRVal->astPackReport.fPackRealAH > g_stCfg.stLocal.usDesignAH) {
            pstPackRVal->astPackReport.fPackRealAH = g_stCfg.stLocal.usDesignAH;
    }
    
    
    
    
    /* 基本状态更新 */
	if(g_stAfe.stRamApp.fCUR >= LOCAL_CUR_DZ) {		/* 充电状态 */
		pstPackRVal->astPackReport.uBaseStat.stBaseStat.ucStat = 1;
	} else if(g_stAfe.stRamApp.fCUR <= -LOCAL_CUR_DZ) {		/* 放电状态 */
		pstPackRVal->astPackReport.uBaseStat.stBaseStat.ucStat  = 2;
	} else {		/* 搁置状态 */
		pstPackRVal->astPackReport.uBaseStat.stBaseStat.ucStat  = 3;
	}
    
    

    
    
    /* 更新充放电循环次数 */
	static float fCycle = 0;
	fCycle += __fabs(g_stAfe.stRamApp.fCUR * 0.001 / 3600 / g_stCfg.stLocal.usDesignAH * g_stCfg.stLocal.usCyclePeriod / 1000) * 0.5;
	if(fCycle >= 1) {
		if(pstPackRVal->astPackReport.usCycle + 1 < 0xFFFF) {
			pstPackRVal->astPackReport.usCycle ++;
		}
		fCycle -= 1;
	}
    
    if(prl_client() && 0 == g_ausPrlComTick[0]) {	/* 内部通信故障 */
		uErrCode.stErrCode.bInterComm = 1;
	} else {
		uErrCode.stErrCode.bInterComm = 0;
	}
    
    
    HIS_LOG_S stLog;
	stLog.dt = DS1302_gtime();
	stLog.eLv = eHisLvFault;
	stLog.eSrc = eHisSrcApp;
	stLog.fVal = 0;
	if(pstPackRVal->astPackReport.uErrCode.stErrCode.bVoltSensor == 0 && uErrCode.stErrCode.bVoltSensor != 0) {
		stLog.eObj = eHisObjVoltSensorErr;
		stLog.eAct = eHisActAct;
		his_log_write(&stLog);
	}
	if(pstPackRVal->astPackReport.uErrCode.stErrCode.bVoltSensor != 0 && uErrCode.stErrCode.bVoltSensor == 0) {
		stLog.eObj = eHisObjVoltSensorErr;
		stLog.eAct = eHisActReact;
		his_log_write(&stLog);
	}
	if(pstPackRVal->astPackReport.uErrCode.stErrCode.bTempSensor == 0 && uErrCode.stErrCode.bTempSensor != 0) {
		stLog.eObj = eHisObjTempSensorErr;
		stLog.eAct = eHisActAct;
		his_log_write(&stLog);
	}
	if(pstPackRVal->astPackReport.uErrCode.stErrCode.bTempSensor != 0 && uErrCode.stErrCode.bTempSensor == 0) {
		stLog.eObj = eHisObjTempSensorErr;
		stLog.eAct = eHisActReact;
		his_log_write(&stLog);
	}
	if(pstPackRVal->astPackReport.uErrCode.stErrCode.bInterComm == 0 && uErrCode.stErrCode.bInterComm != 0) {
		stLog.eObj = eHisObjInterComErr;
		stLog.eAct = eHisActAct;
		his_log_write(&stLog);
	}
	if(pstPackRVal->astPackReport.uErrCode.stErrCode.bInterComm != 0 && uErrCode.stErrCode.bInterComm == 0) {
		stLog.eObj = eHisObjInterComErr;
		stLog.eAct = eHisActReact;
		his_log_write(&stLog);
	}
	if(pstPackRVal->astPackReport.uErrCode.stErrCode.bInputOV == 0 && uErrCode.stErrCode.bInputOV != 0) {
		stLog.eObj = eHisObjInputOV;
		stLog.eAct = eHisActAct;
		his_log_write(&stLog);
	}
	if(pstPackRVal->astPackReport.uErrCode.stErrCode.bInputOV != 0 && uErrCode.stErrCode.bInputOV == 0) {
		stLog.eObj = eHisObjInputOV;
		stLog.eAct = eHisActReact;
		his_log_write(&stLog);
	}
	pstPackRVal->astPackReport.uErrCode = uErrCode;
    
    
    if(pstPackRVal->astPackReport.uBaseStat.stBaseStat.ucStat == 1)
    {
        for(uint8_t i=0;i<CFG_CELL_NUM;i++) {
            static uint16_t s_usTick1[CFG_CELL_NUM] = {0};
            if(pstPackRVal->astPackReport.afCellU[i] * 1000 > g_stCfg.stLocal.usChgAdjEVolt)
            {
                if(s_usTick1[i] * g_stCfg.stLocal.usCyclePeriod < 5000) 
                {
                    s_usTick1[i]++;
                }
                else {
                s_usTick1[i] = 0;
                }
                
                if(s_usTick1[i] * g_stCfg.stLocal.usCyclePeriod >= 5000) 
                {
                    for(uint8_t j=0;j<CFG_CELL_NUM;j++)
                    {
                        pstPackRVal->afCellLeftAH[j] = pstPackRVal->astPackReport.fPackRealAH;
                    }
                    pstPackRVal->afCellLeftAH[i] = pstPackRVal->astPackReport.fPackRealAH;
                    pstPackRVal->afCellSoc[i] = 100;
                    pstPackRVal->astPackReport.fPackSoc = 100;
                    if(pstPackRVal->afCellRealAH[i] < g_stCfg.stLocal.usDesignAH * 0.7) {
                        pstPackRVal->afCellRealAH[i] = g_stCfg.stLocal.usDesignAH * 0.7;
                        pstPackRVal->afCellLeftAH[i] = pstPackRVal->afCellRealAH[i];
                        }
                    if(pstPackRVal->afCellRealAH[i] > g_stCfg.stLocal.usDesignAH) {
                        pstPackRVal->afCellRealAH[i] = g_stCfg.stLocal.usDesignAH;
                        pstPackRVal->afCellLeftAH[i] = pstPackRVal->afCellRealAH[i];
                        }
                    his_data_write();
                    break;
                }
            }
        }
    }
        /* 某一颗电芯达到核容低限, 则该电芯SOC均校准为5% */
	{
		if(pstPackRVal->astPackReport.fPackSoc > 5) {
			for(uint8_t i=0;i<CFG_CELL_NUM;i++) {
				static uint8_t s_usTick1[CFG_CELL_NUM] = {0};
				if(pstPackRVal->astPackReport.afCellU[i] <= 2750 * 0.001) {
					if(s_usTick1[i] * g_stCfg.stLocal.usCyclePeriod < 5000) {
						s_usTick1[i]++;
					}
				} else {
					s_usTick1[i] = 0;
				}
				if(s_usTick1[i] * g_stCfg.stLocal.usCyclePeriod >= 5000) {
					for(uint8_t j=0;j<CFG_CELL_NUM;j++) {
						pstPackRVal->afCellLeftAH[j] = pstPackRVal->afCellRealAH[j] * 0.05;
						pstPackRVal->afCellSoc[j] = 5;
						pstPackRVal->astPackReport.fPackSoc = 5;
						s_usTick1[i] = 0;
					}
					his_data_write();
					break;
				}
			}
		}
	}
    /* 某一颗电芯达到核容低限, 则该电芯SOC均校准为0% */
    for(uint8_t i=0;i<CFG_CELL_NUM;i++) {
            static uint16_t s_usTick[CFG_CELL_NUM] = {0};
            if(pstPackRVal->astPackReport.afCellU[i] <= g_stCfg.stLocal.usDsgAdjEVolt * 0.001) {
                if(s_usTick[i] * g_stCfg.stLocal.usCyclePeriod < 3000) {
					s_usTick[i]++;
				}
			} else {
				s_usTick[i] = 0;
			}
			if(s_usTick[i] * g_stCfg.stLocal.usCyclePeriod >= 3000) {
				pstPackRVal->afCellLeftAH[i] = 0;
				pstPackRVal->afCellSoc[i] = 0;
				pstPackRVal->astPackReport.fPackSoc = 0;
				s_usTick[i] = 0;
				for(uint8_t j=0; j<CFG_CELL_NUM; j++) {
					if(pstPackRVal->afCellLeftAH[j] < 0){
						pstPackRVal->afCellLeftAH[j] = 0;
					}
				}
                his_data_write();
				break;
			}
		}
    
    for(uint8_t i=0;i<CFG_CELL_NUM;i++) {
        local_set_soc(pstPackRVal->afCellLeftAH[i] * 100 / pstPackRVal->afCellRealAH[i], i);
    }
    
    /* PACK剩余容量更新 */
	fPackLeftAH = pstPackRVal->afCellLeftAH[0];
	for(uint8_t i=1;i<CFG_CELL_NUM;i++) {
		if(fPackLeftAH > pstPackRVal->afCellLeftAH[i]) {
			fPackLeftAH = pstPackRVal->afCellLeftAH[i];
		}
	}
	if(fPackLeftAH < 0) {
		fPackLeftAH = 0;
	}
	fChgAH = pstPackRVal->afCellRealAH[0] - pstPackRVal->afCellLeftAH[0];
	for(uint8_t i=1;i<CFG_CELL_NUM;i++) {
		if(fChgAH > pstPackRVal->afCellRealAH[i] - pstPackRVal->afCellLeftAH[i]) {
			fChgAH = pstPackRVal->afCellRealAH[i] - pstPackRVal->afCellLeftAH[i];
		}
	}
	if(fChgAH < 0) {
		fChgAH = 0;
	}
//	pstPackRVal->fPackRealAH = pstPackRVal->fPackLeftAH + fChgAH;
	local_set_soc(pstPackRVal->astPackReport.fPackLeftAH * 100 / pstPackRVal->astPackReport.fPackRealAH, CFG_CELL_NUM);
		//}
//	}
    //按钮状态更新
    pstPackRVal->astPackReport.ucBtn_status = (uint16_t)SW_WK_READ;
    
    
	/* 扩展故障码更新 */
	pstPackRVal->astPackReport.ucErrCodeEx = 0;
	
	return;
}


/*******************************************************************************
* Function Name  : local_alm_ptct_proc
* Description    : 告警和保护检测通用函数
* Input          : fRTV, 被判断的业务变量; stThr, 物理量和时间阈值; pucCnt, 触发计数; pucRetCnt, 恢复计数; eLv, 告警或保护级别; eObj, 日志记录的动作对象; ucUpDown, 0-上升触发, 1-下降触发
* Output         : pucCnt, 触发计数; pucRetCnt, 恢复计数. 这两个同时也是输入参数
* Return         : result, 0-无触发或恢复, 1-恢复, 2-触发
*******************************************************************************/
uint8_t local_alm_ptct_proc(float fRTV, THR_S stThr, uint8_t* pucCnt, uint8_t* pucRetCnt, HIS_LV_E eLv, HIS_OBJ_E eObj, uint8_t ucUpDown) {
	if(ucUpDown != 0) {
		if(fRTV >= stThr.fValThr && *pucCnt != 0xFF) {
			if((*pucCnt) < stThr.ucTmThr * 1000 / g_stCfg.stLocal.usCyclePeriod) {
				(*pucCnt)++;
			}
			if(*pucCnt >= stThr.ucTmThr * 1000 / g_stCfg.stLocal.usCyclePeriod) {
				HIS_LOG_S stLog;
				stLog.dt = DS1302_gtime();
				stLog.eLv = eLv;
				stLog.eSrc = eHisSrcApp;
				stLog.eObj = eObj;
				stLog.eAct = eHisActAct;
				stLog.fVal = fRTV;
				his_log_write(&stLog);
				*pucCnt = 0xFF;
				*pucRetCnt = 0;
				return 2;
			}
		}
		if(fRTV <= stThr.fRetValThr && *pucRetCnt != 0xFF) {
			if(*pucRetCnt < stThr.ucRetTmThr * 1000 / g_stCfg.stLocal.usCyclePeriod) {
				(*pucRetCnt)++;
			}
			if(*pucRetCnt >= stThr.ucRetTmThr * 1000 / g_stCfg.stLocal.usCyclePeriod) {
				HIS_LOG_S stLog;
				stLog.dt = DS1302_gtime();
				stLog.eLv = eLv;
				stLog.eSrc = eHisSrcApp;
				stLog.eObj = eObj;
				stLog.eAct = eHisActReact;
				stLog.fVal = fRTV;
				his_log_write(&stLog);
				*pucCnt = 0;
				*pucRetCnt = 0xFF;
				return 1;
			}
		}
	} else {
		if(fRTV <= stThr.fValThr && *pucCnt != 0xFF) {
			if(*pucCnt < stThr.ucTmThr * 1000 / g_stCfg.stLocal.usCyclePeriod) {
				(*pucCnt)++;
			}
			if(*pucCnt >= stThr.ucTmThr * 1000 / g_stCfg.stLocal.usCyclePeriod) {
				HIS_LOG_S stLog;
				stLog.dt = DS1302_gtime();
				stLog.eLv = eLv;
				stLog.eSrc = eHisSrcApp;
				stLog.eObj = eObj;
				stLog.eAct = eHisActAct;
				stLog.fVal = fRTV;
				his_log_write(&stLog);
				*pucCnt = 0xFF;
				*pucRetCnt = 0;
				return 2;
			}
		} else if(fRTV >= stThr.fRetValThr && *pucRetCnt != 0xFF) {
			if(*pucRetCnt < stThr.ucRetTmThr * 1000 / g_stCfg.stLocal.usCyclePeriod) {
				(*pucRetCnt)++;
			}
			if(*pucRetCnt >= stThr.ucRetTmThr * 1000 / g_stCfg.stLocal.usCyclePeriod) {
				HIS_LOG_S stLog;
				stLog.dt = DS1302_gtime();
				stLog.eLv = eLv;
				stLog.eSrc = eHisSrcApp;
				stLog.eObj = eObj;
				stLog.eAct = eHisActReact;
				stLog.fVal = fRTV;
				his_log_write(&stLog);
				*pucCnt = 0;
				*pucRetCnt = 0xFF;
				return 1;
			}
		}
	}
	
	return 0;
}

/*******************************************************************************
* Function Name  : local_almptct
* Description    : 告警和保护检测
* Input          : None
* Output         : None
* Return         : result, 1写成功 0写失败
*******************************************************************************/
void local_alm_ptct(void) {
	LOCAL_PACK_RVAL_S* pstPackRVal = &g_stLocalPackRVal[g_stPrl.ucSelfId];
	uint8_t ucRet;
	/* 电芯过压告警 */
	{
		static uint8_t ucCnt = 0;
		static uint8_t ucRetCnt = 0xFF;
		ucRet = local_alm_ptct_proc((float)(pstPackRVal->astPackReport.fCellUMax)/1000, g_stCfg.stLocal.stCellOVAlm, &ucCnt, &ucRetCnt, eHisLvAlarm, eHisObjCellOV, 1);
		if(ucRet != 0) pstPackRVal->astPackReport.uAlmCode.stAlmCode.bCellOV = ucRet - 1;
	}
	/* 电芯过压保护 */
	{
		static uint8_t ucCnt = 0;
		static uint8_t ucRetCnt = 0xFF;
		ucRet = local_alm_ptct_proc((float)(pstPackRVal->astPackReport.fCellUMax)/1000, g_stCfg.stLocal.stCellOVPtct, &ucCnt, &ucRetCnt, eHisLvPtct, eHisObjCellOV, 1);
		if(ucRet != 0) pstPackRVal->astPackReport.uPtctCode.stPtctCode.bCellOV = ucRet - 1;
	}
	/* 电芯欠压告警 */
	{
		static uint8_t ucCnt = 0;
		static uint8_t ucRetCnt = 0xFF;
		ucRet = local_alm_ptct_proc((float)(pstPackRVal->astPackReport.fCellUMin)/1000, g_stCfg.stLocal.stCellUVAlm, &ucCnt, &ucRetCnt, eHisLvAlarm, eHisObjCellUV, 0);
		if(ucRet != 0) pstPackRVal->astPackReport.uAlmCode.stAlmCode.bCellUV = ucRet - 1;
	}
	/* 电芯欠压保护 */
	{
		static uint8_t ucCnt = 0;
		static uint8_t ucRetCnt = 0xFF;
		ucRet = local_alm_ptct_proc((float)(pstPackRVal->astPackReport.fCellUMin)/1000, g_stCfg.stLocal.stCellUVPtct, &ucCnt, &ucRetCnt, eHisLvPtct, eHisObjCellUV, 0);
		if(ucRet != 0) pstPackRVal->astPackReport.uPtctCode.stPtctCode.bCellUV = ucRet - 1;
	}
	/* PACK过压告警 */
	{
		static uint8_t ucCnt = 0;
		static uint8_t ucRetCnt = 0xFF;
		ucRet = local_alm_ptct_proc((float)(pstPackRVal->astPackReport.fPackU)/1000, g_stCfg.stLocal.stPackOVAlm, &ucCnt, &ucRetCnt, eHisLvAlarm, eHisObjPackOV, 1);
		if(ucRet != 0) pstPackRVal->astPackReport.uAlmCode.stAlmCode.bPackOV = ucRet - 1;
	}
	/* PACK过压保护 */
	{
		static uint8_t ucCnt = 0;
		static uint8_t ucRetCnt = 0xFF;
		ucRet = local_alm_ptct_proc((float)(pstPackRVal->astPackReport.fPackU)/1000, g_stCfg.stLocal.stPackOVPtct, &ucCnt, &ucRetCnt, eHisLvPtct, eHisObjPackOV, 1);
		if(ucRet != 0) pstPackRVal->astPackReport.uPtctCode.stPtctCode.bPackOV = ucRet - 1;
	}
	/* PACK低压告警 */
	{
		static uint8_t ucCnt = 0;
		static uint8_t ucRetCnt = 0xFF;
		ucRet = local_alm_ptct_proc((float)(pstPackRVal->astPackReport.fPackU)/1000, g_stCfg.stLocal.stPackUVAlm, &ucCnt, &ucRetCnt, eHisLvAlarm, eHisObjPackUV, 0);
		if(ucRet != 0) pstPackRVal->astPackReport.uAlmCode.stAlmCode.bPackUV = ucRet - 1;
	}
	/* PACK低压保护 */
	{
		static uint8_t ucCnt = 0;
		static uint8_t ucRetCnt = 0xFF;
		ucRet = local_alm_ptct_proc((float)(pstPackRVal->astPackReport.fPackU)/1000, g_stCfg.stLocal.stPackUVPtct, &ucCnt, &ucRetCnt, eHisLvPtct, eHisObjPackUV, 0);
		if(ucRet != 0) pstPackRVal->astPackReport.uPtctCode.stPtctCode.bPackUV = ucRet - 1;
	}
	/* 充电过温告警 */
	{
		static uint8_t ucCnt = 0;
		static uint8_t ucRetCnt = 0xFF;
		if((pstPackRVal->astPackReport.uBaseStat.stBaseStat.ucStat == 1 && ucCnt != 0xFF) || ucRetCnt != 0xFF) {
			ucRet = local_alm_ptct_proc((float)(pstPackRVal->astPackReport.fCellTMax), g_stCfg.stLocal.stChgOTAlm, &ucCnt, &ucRetCnt, eHisLvAlarm, eHisObjOCT, 1);
			if(ucRet != 0) pstPackRVal->astPackReport.uAlmCode.stAlmCode.bChgOT = ucRet - 1;
		}
	}
	/* 充电过温保护 */
	{
		static uint8_t ucCnt = 0;
		static uint8_t ucRetCnt = 0xFF;
		if((pstPackRVal->astPackReport.uBaseStat.stBaseStat.ucStat == 1 && ucCnt != 0xFF) || ucRetCnt != 0xFF) {
			ucRet = local_alm_ptct_proc(pstPackRVal->astPackReport.fCellTMax, g_stCfg.stLocal.stChgOTPtct, &ucCnt, &ucRetCnt, eHisLvPtct, eHisObjOCT, 1);
			if(ucRet != 0) pstPackRVal->astPackReport.uPtctCode.stPtctCode.bChgOT = ucRet - 1;
		}
	}
	/* 充电低温告警 */
	{
		static uint8_t ucCnt = 0;
		static uint8_t ucRetCnt = 0xFF;
		if((pstPackRVal->astPackReport.uBaseStat.stBaseStat.ucStat == 1 && ucCnt != 0xFF) || ucRetCnt != 0xFF) {
			ucRet = local_alm_ptct_proc(pstPackRVal->astPackReport.fCellTMin, g_stCfg.stLocal.stChgUTAlm, &ucCnt, &ucRetCnt, eHisLvAlarm, eHisObjUCT, 0);
			if(ucRet != 0) pstPackRVal->astPackReport.uAlmCode.stAlmCode.bChgUT = ucRet - 1;
		}
	}
	/* 充电低温保护 */
	{
		static uint8_t ucCnt = 0;
		static uint8_t ucRetCnt = 0xFF;
		if((pstPackRVal->astPackReport.uBaseStat.stBaseStat.ucStat == 1 && ucCnt != 0xFF) || ucRetCnt != 0xFF) {
			ucRet = local_alm_ptct_proc(pstPackRVal->astPackReport.fCellTMin, g_stCfg.stLocal.stChgUTPtct, &ucCnt, &ucRetCnt, eHisLvPtct, eHisObjUCT, 0);
			if(ucRet != 0) pstPackRVal->astPackReport.uPtctCode.stPtctCode.bChgUT = ucRet - 1;
		}
	}
	/* 放电过温告警 */
	{
		static uint8_t ucCnt = 0;
		static uint8_t ucRetCnt = 0xFF;
		if((pstPackRVal->astPackReport.uBaseStat.stBaseStat.ucStat == 2 && ucCnt != 0xFF) || ucRetCnt != 0xFF) {
			ucRet = local_alm_ptct_proc(pstPackRVal->astPackReport.fCellTMax, g_stCfg.stLocal.stDsgOTAlm, &ucCnt, &ucRetCnt, eHisLvAlarm, eHisObjODT, 1);
			if(ucRet != 0) pstPackRVal->astPackReport.uAlmCode.stAlmCode.bDsgOT = ucRet - 1;
		}
	}
	/* 放电过温保护 */
	{
		static uint8_t ucCnt = 0;
		static uint8_t ucRetCnt = 0xFF;
		if((pstPackRVal->astPackReport.uBaseStat.stBaseStat.ucStat == 2 && ucCnt != 0xFF) || ucRetCnt != 0xFF) {
			ucRet = local_alm_ptct_proc(pstPackRVal->astPackReport.fCellTMax, g_stCfg.stLocal.stDsgOTPtct, &ucCnt, &ucRetCnt, eHisLvPtct, eHisObjODT, 1);
			if(ucRet != 0) pstPackRVal->astPackReport.uPtctCode.stPtctCode.bDsgOT = ucRet - 1;
		}
	}
	/* 放电低温告警 */
	{
		static uint8_t ucCnt = 0;
		static uint8_t ucRetCnt = 0xFF;
		if((pstPackRVal->astPackReport.uBaseStat.stBaseStat.ucStat == 2 && ucCnt != 0xFF) || ucRetCnt != 0xFF) {
			ucRet = local_alm_ptct_proc(pstPackRVal->astPackReport.fCellTMin, g_stCfg.stLocal.stDsgUTAlm, &ucCnt, &ucRetCnt, eHisLvAlarm, eHisObjUDT, 0);
			if(ucRet != 0) pstPackRVal->astPackReport.uAlmCode.stAlmCode.bDsgUT = ucRet - 1;
		}
	}
	/* 放电低温保护 */
	{
		static uint8_t ucCnt = 0;
		static uint8_t ucRetCnt = 0xFF;
		if((pstPackRVal->astPackReport.uBaseStat.stBaseStat.ucStat == 2 && ucCnt != 0xFF) || ucRetCnt != 0xFF) {
			ucRet = local_alm_ptct_proc(pstPackRVal->astPackReport.fCellTMin, g_stCfg.stLocal.stDsgUTPtct, &ucCnt, &ucRetCnt, eHisLvPtct, eHisObjUDT, 0);
			if(ucRet != 0) pstPackRVal->astPackReport.uPtctCode.stPtctCode.bDsgUT = ucRet - 1;
		}
	}

	/* 充电过流告警 */
	{
		static uint8_t ucCnt = 0;
		static uint8_t ucRetCnt = 0xFF;
		ucRet = local_alm_ptct_proc((float)(pstPackRVal->astPackReport.fPackCur)/10, g_stCfg.stLocal.stOCCAlm, &ucCnt, &ucRetCnt, eHisLvAlarm, eHisObjOCC, 1);
		if(ucRet != 0) pstPackRVal->astPackReport.uAlmCode.stAlmCode.bChgOCC = ucRet - 1;
	}
	/* 充电过流保护 */
	uint8_t ucChgOCC = pstPackRVal->astPackReport.uPtctCode.stPtctCode.bChgOCC;
	{
		static uint8_t ucCnt = 0;
		static uint8_t ucRetCnt = 0xFF;
		ucRet = local_alm_ptct_proc((float)(pstPackRVal->astPackReport.fPackCur)/10, g_stCfg.stLocal.stOCCPtct, &ucCnt, &ucRetCnt, eHisLvPtct, eHisObjOCC, 1);
		if(ucRet != 0) pstPackRVal->astPackReport.uPtctCode.stPtctCode.bChgOCC = ucRet - 1;
	}
	if(ucChgOCC == 0 && pstPackRVal->astPackReport.uPtctCode.stPtctCode.bChgOCC != 0 && g_ucOCCTick < 3) {
		g_ucOCCTick++;
	}
	/* 放电过流告警 */
	{
		static uint8_t ucCnt = 0;
		static uint8_t ucRetCnt = 0xFF;
		ucRet = local_alm_ptct_proc((float)(pstPackRVal->astPackReport.fPackCur)/10, g_stCfg.stLocal.stODCAlm, &ucCnt, &ucRetCnt, eHisLvAlarm, eHisObjODC, 0);
		if(ucRet != 0) pstPackRVal->astPackReport.uAlmCode.stAlmCode.bDsgODC = ucRet - 1;
	}
	/* 放电过流保护 */
	uint8_t ucChgODC = pstPackRVal->astPackReport.uPtctCode.stPtctCode.bDsgODC;
	{
		static uint8_t ucCnt = 0;
		static uint8_t ucRetCnt = 0xFF;
		ucRet = local_alm_ptct_proc((float)(pstPackRVal->astPackReport.fPackCur)/10, g_stCfg.stLocal.stODCPtct, &ucCnt, &ucRetCnt, eHisLvPtct, eHisObjODC, 0);
		if(ucRet != 0) pstPackRVal->astPackReport.uPtctCode.stPtctCode.bDsgODC = ucRet - 1;
	}
	if(ucChgODC == 0 && pstPackRVal->astPackReport.uPtctCode.stPtctCode.bDsgODC != 0 && g_ucOCDTick < 3) {
		g_ucOCDTick++;
	}
	/* MOS高温告警 */
	{
		static uint8_t ucCnt = 0;
		static uint8_t ucRetCnt = 0xFF;
		ucRet = local_alm_ptct_proc(pstPackRVal->astPackReport.fMosT, g_stCfg.stLocal.stMosOTAlm, &ucCnt, &ucRetCnt, eHisLvAlarm, eHisObjMosOT, 1);
		if(ucRet != 0) pstPackRVal->astPackReport.uAlmCode.stAlmCode.bMosOT = ucRet - 1;
	}
	/* MOS高温保护 */
	{
		static uint8_t ucCnt = 0;
		static uint8_t ucRetCnt = 0xFF;
		ucRet = local_alm_ptct_proc(pstPackRVal->astPackReport.fMosT, g_stCfg.stLocal.stMosOTPtct, &ucCnt, &ucRetCnt, eHisLvPtct, eHisObjMosOT, 1);
		if(ucRet != 0) pstPackRVal->astPackReport.uPtctCode.stPtctCode.bMosOT = ucRet - 1;
	}
	/* 短路保护 */
	{
		pstPackRVal->astPackReport.uPtctCode.stPtctCode.bSC = g_stAfe.uRom.stCode.SC;
		if(pstPackRVal->astPackReport.uPtctCode.stPtctCode.bSC) {
			HIS_LOG_S stLog;
			stLog.dt = DS1302_gtime();
			stLog.eLv = eHisLvPtct;
			stLog.eSrc = eHisSrcApp;
			stLog.eObj = eHisObjSC;
			stLog.eAct = eHisActAct;
			stLog.fVal = 0;
			his_log_write(&stLog);
		}
	}
	/* PACK组低压告警 */
	uint8_t bArrayUV = 0;
	for(int i=0;i<g_stPrl.ucDevNum;i++) {
		if(g_ausPrlComTick[i] == 0) {
			continue;
		}
		if(g_stLocalPackRVal[g_stPrl.ucSelfId].astPackReport.uAlmCode.stAlmCode.bPackUV != 0) {
			bArrayUV = 1;
			break;
		}
	}
	pstPackRVal->astPackReport.uAlmCode.stAlmCode.bArrayUV = bArrayUV;
	g_stLocalPackRVal[g_stPrl.ucSelfId].astPackReport.uPtctCode.stPtctCode.bArrayUV = bArrayUV;
	/* PACK组高压告警 */
	uint8_t bArrayOV = 0;
	for(int i=0;i<g_stPrl.ucDevNum;i++) {
		if(g_ausPrlComTick[i] == 0) {
			continue;
		}
		if(g_stLocalPackRVal[g_stPrl.ucSelfId].astPackReport.uAlmCode.stAlmCode.bPackOV != 0) {
			bArrayOV = 1;
			break;
		}
	}
	pstPackRVal->astPackReport.uAlmCode.stAlmCode.bArrayOV = bArrayOV;
	g_stLocalPackRVal[g_stPrl.ucSelfId].astPackReport.uPtctCode.stPtctCode.bArrayOV = bArrayOV;
	/* PACK组低压保护 */
	bArrayUV = 0;
	for(int i=0;i<g_stPrl.ucDevNum;i++) {
		if(g_ausPrlComTick[i] == 0) {
			continue;
		}
		if(g_stLocalPackRVal[g_stPrl.ucSelfId].astPackReport.uPtctCode.stPtctCode.bPackUV != 0) {
			bArrayUV = 1;
			break;
		}
	}
	pstPackRVal->astPackReport.uPtctCode.stPtctCode.bArrayUV = bArrayUV;
	g_stLocalPackRVal[g_stPrl.ucSelfId].astPackReport.uPtctCode.stPtctCode.bArrayUV = bArrayUV;
	/* PACK组高压保护 */
	bArrayOV = 0;
	for(int i=0;i<g_stPrl.ucDevNum;i++) {
		if(g_ausPrlComTick[i] == 0) {
			continue;
		}
		if(g_stLocalPackRVal[g_stPrl.ucSelfId].astPackReport.uPtctCode.stPtctCode.bPackOV != 0) {
			bArrayOV = 1;
			break;
		}
	}
	pstPackRVal->astPackReport.uPtctCode.stPtctCode.bArrayOV = bArrayOV;
	g_stLocalPackRVal[g_stPrl.ucSelfId].astPackReport.uPtctCode.stPtctCode.bArrayOV = bArrayOV;
}

void local_chg_dsg_ctrl(void) {
	uint8_t ucChgEn = 0x55;
	uint8_t ucDsgEn = 0x55;
	LOCAL_PACK_RVAL_S* pstPackRVal = &g_stLocalPackRVal[g_stPrl.ucSelfId];
    
	if(pstPackRVal->astPackReport.uErrCode.ucErrCode != 0) {
		ucChgEn = 0xAA;
		ucDsgEn = 0xAA;
        AFE_CTLD_L;
	} else {
        AFE_CTLD_H;
	}
	if(pstPackRVal->astPackReport.uPtctCode.stPtctCode.bCellUV != 0) {
		ucDsgEn = 0xAA;
	}
	if(pstPackRVal->astPackReport.uPtctCode.stPtctCode.bCellOV != 0) {
		ucChgEn = 0xAA;
	}
	if(pstPackRVal->astPackReport.uPtctCode.stPtctCode.bChgUT != 0 || pstPackRVal->astPackReport.uPtctCode.stPtctCode.bChgOT != 0) {
		ucChgEn = 0xAA;
	}
	if(pstPackRVal->astPackReport.uPtctCode.stPtctCode.bDsgUT != 0 || pstPackRVal->astPackReport.uPtctCode.stPtctCode.bDsgOT != 0) {
		ucDsgEn = 0xAA;
	}
	if(pstPackRVal->astPackReport.uPtctCode.stPtctCode.bChgOCC != 0) {
		ucChgEn = 0xAA;
	}
	if(pstPackRVal->astPackReport.uPtctCode.stPtctCode.bDsgODC != 0) {
		ucDsgEn = 0xAA;
	}
	if(pstPackRVal->astPackReport.uPtctCode.stPtctCode.bPackUV != 0) {
		ucDsgEn = 0xAA;
	}
	if(pstPackRVal->astPackReport.uPtctCode.stPtctCode.bPackOV != 0) {
		ucChgEn = 0xAA;
	}
	if(pstPackRVal->astPackReport.uPtctCode.stPtctCode.bSC != 0) {
		ucChgEn = 0xAA;
		ucDsgEn = 0xAA;
	}
	if(pstPackRVal->astPackReport.uPtctCode.stPtctCode.bEnvUT != 0 || pstPackRVal->astPackReport.uPtctCode.stPtctCode.bEnvOT != 0 || pstPackRVal->astPackReport.uPtctCode.stPtctCode.bMosOT != 0) {
		ucChgEn = 0xAA;
		ucDsgEn = 0xAA;
	}
	static uint16_t usOCTokenTick = 0;	/* 过流保护令牌减少计数 */
	if(usOCTokenTick < 60 * 1000 / g_stCfg.stLocal.usCyclePeriod) {
		usOCTokenTick++;
	} else {
		if(g_ucOCCTick > 0) {
			g_ucOCCTick--;
		}
		if(g_ucOCDTick > 0) {
			g_ucOCDTick--;
		}
		usOCTokenTick = 0;
	}
	if(g_ucOCCTick >= 3) {
		ucChgEn = 0xAA;
	}
	if(g_ucOCDTick >= 3) {
		ucDsgEn = 0xAA;
	}
	if(g_stAfe.stRamApp.fCUR >= 1000) {		/* 恢复充电清除过放计数 */
		g_ucOCDTick = 0;
	}
	if(g_stAfe.stRamApp.fCUR <= -1000) {	/* 恢复放电清除过充计数 */
		g_ucOCCTick = 0;
	}
/* 充放电MOS通断更新 */

	if(g_ausPrlComTick[0] == 0 && prl_client()) {
		ucChgEn = 0xAA;
		ucDsgEn = 0xAA;
	}
	if(pstPackRVal->astPackReport.fPackSoc >= 100 && pstPackRVal->astPackReport.fPackLeftAH / pstPackRVal->astPackReport.fPackRealAH > 1.05 && prl_single() == 0) {
		ucChgEn = 0xAA;
		//pstPackRVal->fOCC = 0;
	}
    
//    if(ucChgEn == 0x55 && g_stLocalFlagWVal.g_ucChgRecvFlag == 1){
//        ucChgEn = 0x55;
//    }else{
//        ucChgEn =0xAA;
//    }
//    if(ucDsgEn == 0x55 && g_stLocalFlagWVal.g_ucDsgRecvFlag == 1){
//        ucDsgEn = 0x55;
//    }else{
//        ucDsgEn =0xAA;
//    }
	pstPackRVal->astPackReport.ucChgEn = ucChgEn;
	pstPackRVal->astPackReport.ucDsgEn = ucDsgEn;
	local_pch2chg(ucChgEn, ucDsgEn);
}


void local_led_proc(void) {
	LOCAL_PACK_RVAL_S* pstPackRVal = &g_stLocalPackRVal[g_stPrl.ucSelfId];
	if(g_stLocalStat == eLocalStatInitFail) {	/* 初始化失败, 待机告警状态 */
		//led_set_stat(eLedStandbyAlm);
		return;
//	} else if(g_stLocalArrayRVal.eLocalStat) {	/* 并机协商状态 */
//		if(g_stPrl.ucSelfId == ePrlPackIdNull) {
//			//led_set_stat(eLedCstUncfm);
//		} else {
//			//led_set_stat(eLedCstCfm);
//		}
	}
    else if(pstPackRVal->astPackReport.uErrCode.ucErrCode != 0 || pstPackRVal->astPackReport.ucErrCodeEx != 0) {	/* 内部错误, 失效状态 */
		//led_set_stat(eLedIPtct);
	} else if(3 == pstPackRVal->astPackReport.uBaseStat.stBaseStat.ucStat) {		/* 待机状态 */
		if(pstPackRVal->astPackReport.uAlmCode.usAlmCode == 0 && pstPackRVal->astPackReport.uPtctCode.usPtctCode == 0) {
			//led_set_stat(eLedStandbyNormal);
		} else {
			//led_set_stat(eLedStandbyAlm);
		}
	} else if(1 == pstPackRVal->astPackReport.uBaseStat.stBaseStat.ucStat) {		/* 充电状态 */
		if(pstPackRVal->astPackReport.uAlmCode.usAlmCode == 0 && pstPackRVal->astPackReport.uPtctCode.usPtctCode == 0) {
			//led_set_stat(eLedChgNormal);
		} else if(pstPackRVal->astPackReport.uPtctCode.stPtctCode.bCellOV != 0 || pstPackRVal->astPackReport.uPtctCode.stPtctCode.bPackOV != 0) {
			//led_set_stat(eLedChgOVPtct);
		} else if(pstPackRVal->astPackReport.uPtctCode.stPtctCode.bChgOT != 0 || pstPackRVal->astPackReport.uPtctCode.stPtctCode.bChgUT || pstPackRVal->astPackReport.uPtctCode.stPtctCode.bChgOCC) {
			//led_set_stat(eLedChgTCIPtct);
		} else {
			//led_set_stat(eLedChgAlm);
		}
	} else if(2 == pstPackRVal->astPackReport.uBaseStat.stBaseStat.ucStat) {		/* 放电状态 */
		if(pstPackRVal->astPackReport.uAlmCode.usAlmCode == 0 && pstPackRVal->astPackReport.uPtctCode.usPtctCode == 0) {
			//led_set_stat(eLedDsgNormal);
		} else if(pstPackRVal->astPackReport.uPtctCode.stPtctCode.bCellUV != 0 && pstPackRVal->astPackReport.uPtctCode.stPtctCode.bPackUV != 0) {
			//led_set_stat(eLedDsgUVPtct);
		} else if(pstPackRVal->astPackReport.uPtctCode.stPtctCode.bDsgOT != 0 || pstPackRVal->astPackReport.uPtctCode.stPtctCode.bDsgUT || pstPackRVal->astPackReport.uPtctCode.stPtctCode.bDsgODC) {
			//led_set_stat(eLedDsgTCIPtct);
		} else {
			//led_set_stat(eLedDsgAlm);
		}
	}
}

void local_proc(void) {

	if(g_stCfg.stPrl.ePrlSelfIdx != g_stPrl.ucSelfId) {
        //ywj
		g_stLocalPackRVal[g_stCfg.stPrl.ePrlSelfIdx] = g_stLocalPackRVal[g_stCfg.stPrl.ePrlSelfIdx];
		memset(g_stLocalPackRVal + g_stCfg.stPrl.ePrlSelfIdx, 0, sizeof(LOCAL_PACK_RVAL_S));
        //ywj
        g_stPrl.ucSelfId = g_stCfg.stPrl.ePrlSelfIdx;
	}
	/* 每g_stCfg.stLocal.usCyclePeriod毫秒调用一次本函数以下部分 */
	static uint32_t uiTick = 0;
	uiTick++;
	if(uiTick < 20 * g_stCfg.stLocal.usCyclePeriod / 1000) {
		return;
	}
	
	
	uiTick = 0;
	/* LED和蜂鸣器状态刷新 */
	local_led_proc();
	/* 未完成协商时, 不能够正常运行 */
	if(g_stPrl.ucSelfId > CFG_MAX_PRL_NUM) {
		local_pch2chg(0xAA, 0xAA);
		return;
	}
	/* 采集数据及中间变量的计算 */
	local_refresh();
	/* 告警和软保护检测 */
	local_alm_ptct();

    
    float fOV = g_stCfg.stLocal.stPackOVPtct.fValThr;
	float fUV = g_stCfg.stLocal.usDsgAdjEVolt * 0.001 * CFG_CELL_NUM;
	float fOCC = g_stCfg.stLocal.usDesignAH;
	float fODC = g_stCfg.stLocal.usDesignAH;
    
    

	LOCAL_PACK_RVAL_S* pstPackRVal = &g_stLocalPackRVal[g_stPrl.ucSelfId];
	if(pstPackRVal->astPackReport.fPackSoc > 90) {
		fOCC *= (100 - pstPackRVal->astPackReport.fPackSoc) / 10;
	}
	/* 告警修正对外上报的OV, UV, OCC, ODC */
//	if(pstPackRVal->uAlmCode.stAlmCode.bCellOV || pstPackRVal->uAlmCode.stAlmCode.bPackOV || pstPackRVal->uAlmCode.stAlmCode.bChgOCC || pstPackRVal->uAlmCode.stAlmCode.bMosOT
//		|| pstPackRVal->uAlmCode.stAlmCode.bChgOT || pstPackRVal->uAlmCode.stAlmCode.bChgUT || pstPackRVal->uAlmCode.stAlmCode.bEnvOT || pstPackRVal->uAlmCode.stAlmCode.bEnvUT
//		|| pstPackRVal->uPtctCode.stPtctCode.bCellOV || pstPackRVal->uPtctCode.stPtctCode.bPackOV || pstPackRVal->uPtctCode.stPtctCode.bChgOCC
//		|| pstPackRVal->uPtctCode.stPtctCode.bChgOT || pstPackRVal->uPtctCode.stPtctCode.bChgUT || pstPackRVal->uPtctCode.stPtctCode.bEnvOT || pstPackRVal->uPtctCode.stPtctCode.bEnvUT
//		|| pstPackRVal->uPtctCode.stPtctCode.bSC || pstPackRVal->uPtctCode.stPtctCode.bMosOT) {
//		fOCC = 0;
//	}
//	if(pstPackRVal->uAlmCode.stAlmCode.bCellUV || pstPackRVal->uAlmCode.stAlmCode.bPackUV || pstPackRVal->uPtctCode.stPtctCode.bDsgODC || pstPackRVal->uAlmCode.stAlmCode.bMosOT
//		|| pstPackRVal->uAlmCode.stAlmCode.bDsgOT || pstPackRVal->uAlmCode.stAlmCode.bDsgUT || pstPackRVal->uAlmCode.stAlmCode.bEnvOT || pstPackRVal->uAlmCode.stAlmCode.bEnvUT
//		|| pstPackRVal->uPtctCode.stPtctCode.bCellUV || pstPackRVal->uPtctCode.stPtctCode.bPackUV || pstPackRVal->uPtctCode.stPtctCode.bDsgODC
//		|| pstPackRVal->uPtctCode.stPtctCode.bDsgOT || pstPackRVal->uPtctCode.stPtctCode.bDsgUT || pstPackRVal->uPtctCode.stPtctCode.bEnvOT || pstPackRVal->uPtctCode.stPtctCode.bEnvUT
//		|| pstPackRVal->uPtctCode.stPtctCode.bSC || pstPackRVal->uPtctCode.stPtctCode.bMosOT) {
//		fODC = 0;
//	}
    if(pstPackRVal->astPackReport.uPtctCode.stPtctCode.bCellOV || pstPackRVal->astPackReport.uPtctCode.stPtctCode.bPackOV || pstPackRVal->astPackReport.uPtctCode.stPtctCode.bChgOCC
        || pstPackRVal->astPackReport.uPtctCode.stPtctCode.bChgOT || pstPackRVal->astPackReport.uPtctCode.stPtctCode.bChgUT || pstPackRVal->astPackReport.uPtctCode.stPtctCode.bEnvOT || pstPackRVal->astPackReport.uPtctCode.stPtctCode.bEnvUT
        || pstPackRVal->astPackReport.uPtctCode.stPtctCode.bSC || pstPackRVal->astPackReport.uPtctCode.stPtctCode.bMosOT) {
        fOCC = 0;
	}
	if(pstPackRVal->astPackReport.uPtctCode.stPtctCode.bCellUV || pstPackRVal->astPackReport.uPtctCode.stPtctCode.bPackUV || pstPackRVal->astPackReport.uPtctCode.stPtctCode.bDsgODC
		|| pstPackRVal->astPackReport.uPtctCode.stPtctCode.bDsgOT || pstPackRVal->astPackReport.uPtctCode.stPtctCode.bDsgUT || pstPackRVal->astPackReport.uPtctCode.stPtctCode.bEnvOT || pstPackRVal->astPackReport.uPtctCode.stPtctCode.bEnvUT
		|| pstPackRVal->astPackReport.uPtctCode.stPtctCode.bSC || pstPackRVal->astPackReport.uPtctCode.stPtctCode.bMosOT) {
		fODC = 0;
	}
	if(local_get_soc(CFG_CELL_NUM) >= 100) {
		fOCC = 0;
	}
	if(local_get_soc(CFG_CELL_NUM) <= 0) {
		fODC = 0;
	}
	if(g_ausPrlComTick[0] == 0 && prl_client()) {
		fOCC = 0;
		fODC = 0;
	}
	pstPackRVal->astPackReport.fOV = fOV;		/* 过压阈值 */
	pstPackRVal->astPackReport.fUV = fUV;		/* 欠压阈值 */
	pstPackRVal->astPackReport.fOCC = fOCC * 0.01;	/* 过充电流阈值 */
	pstPackRVal->astPackReport.fODC = fODC * 0.01;	/* 过放电流阈值 */
    
	/* MOS的控制 */
	local_chg_dsg_ctrl();
	/* 被动均衡 */
	if(pstPackRVal->astPackReport.uAlmCode.usAlmCode != 0 || pstPackRVal->astPackReport.uPtctCode.usPtctCode != 0 || pstPackRVal->astPackReport.uErrCode.ucErrCode != 0) {
		pstPackRVal->astPackReport.uBaseStat.stBaseStat.bBalnceChgReq = 0;
	}
//	if(pstPackRVal->uBaseStat.stBaseStat.bBalnceChgReq != 0 && g_stAfe.uRam.stCode.BALANCEH == 0 && g_stAfe.uRam.stCode.BALANCEL == 0 && pstPackRVal->fCellUMax > g_stCfg.stLocal.usBCHGMaxCellU) {
//		float fVal = 0;
//		fVal = (1 << pstPackRVal->ucCellUMaxId);
//		afe_set_ao(eAfeRamCodeCB, 1, &fVal);
//	}
}

bool local_read_reg(uint8_t ucSelfId, uint8_t ucDevAddr, uint16_t usRegAddr ,uint16_t* pfVal) {

    if(usRegAddr >= R_StartAddr && usRegAddr < W_StartAddr){
        //memcpy(pfVal,&g_stLocalArrayRVal.astPackRVal->afCellU + usRegAddr - R_StartAddr,sizeof(uint16_t));
        memcpy(pfVal,&g_stLocalPackRVal[g_stPrl.ucSelfId].astPackReport.afCellU[0] + usRegAddr - R_StartAddr,2);
        //pfVal = g_stLocalArrayRVal.astPackRVal->afCellU + usRegAddr - R_StartAddr;
        return 0;
    }
    return 1;
}

bool local_Write_reg(uint8_t ucSelfId, uint8_t ucDevAddr, uint16_t usRegAddr ,uint16_t* pfVal){
    
    if(usRegAddr >= W_StartAddr){
        //memcpy((&g_stLocalFlagWVal) + (usRegAddr - W_StartAddr), pfVal, 4);
//        if((usRegAddr - W_StartAddr)==0){
//            g_stLocalFlagWVal.g_ucChgRecvFlag = *pfVal;
//            //memcpy((&g_stLocalFlagWVal.g_ucChgRecvFlag) + (usRegAddr - W_StartAddr), (uint8_t*)pfVal, 1);
//        }else{
//            g_stLocalFlagWVal.g_ucDsgRecvFlag = *pfVal;
//        
//        }
        memcpy(&g_stLocalFlagWVal.g_ucChgRecvFlag + usRegAddr - W_StartAddr, pfVal,2);
    }
    LOCAL_RETURN_TRUE;
}



