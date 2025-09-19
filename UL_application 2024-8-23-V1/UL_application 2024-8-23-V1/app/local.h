#ifndef LOCAL_H
#define LOCAL_H

#include <stdbool.h>
#include <stdint.h>

#include "parallel.h"

#include "main.h"

#ifdef USER_DEBUG
#define LOCAL_DEBUG_EN	1		/* 0: non-DEBUG state, 1: DEBUG status */
#else
#define LOCAL_DEBUG_EN 0
#endif

#define LOCAL_DEBUG(fmt,arg...)	do{if(LOCAL_DEBUG_EN){printf(fmt,##arg);printf("[%s][%d]\r\n",__func__,__LINE__);}}while(0)
#define LOCAL_RETURN_FALSE	do{LOCAL_DEBUG("Return failed");return false;}while(0)
//#define LOCAL_RETURN_TRUE do{LOCAL_DEBUG("Return success");return true;}while(0)
#define LOCAL_RETURN_TRUE do{return true;}while(0)

#pragma pack(1)



extern uint8_t g_ucAlmLevel;
extern bool g_BalanceFlag;
extern bool g_BalanceFlag_2;
extern uint8_t g_ucChgFull;
extern uint8_t g_ucFullChgRec;
extern uint8_t balanc[16]; 
extern bool g_bNeedHeat;


#define LOCAL_CUR_DZ	2000	/* current dead zone, unit: mA */


/* 7. Standby soc calibration */
typedef struct {
	float afTemp[5];
	float afSoc[21];
	float aafCellU[21][5];
} LOCAL_SOC_CALI_S;

/* 8. Realtime data */
/* The external load is calculated based on a minimum start-up current of 3A, 
The limit case battery voltage is 48V, and the external load resistance shall not be greater than 16Ω,
The pre-play circuit resistance is 25.5 ohms, the total series resistance is 41.5 Ω, and the minimum current is not less than 1150mA,
Considering the superimposed current sampling error d, the value shall not be higher than 1150-d and shall not be lower than d
Assuming that the current d is 500mA, the value range is 500~650 */
#define LOCAL_CUR_RESIST	(0.001/22)		/* unit: Ω */

typedef struct {
	uint8_t ucStat:3;
	uint8_t bForceChgReq:1;     //强充
	uint8_t bBalnceChgReq:1;    //均充
} BASE_STAT_S;

typedef enum {
	eBStatWorkEnd = 0,	          /* Charge and discharge end */
	eBStatWorkStart,					    /* Charge and discharge begins */
	eBStatWorking,			          /* Charging and discharging are in progress */
	eBStatSleep_01,			          /* Normal sleep, LCD sleep, SOC > 20% sleep, SOC < 20% sleep */
	eBStatWake,			              /* awaken */
	eBStatStewing,			          /* Let it stand */
	eBStatSleep_02,			          /* Undervoltage sleep */
	eBStatDefault,                /* Restore the default parameters */
	eBStatDownload,               /* Firmware download*/
	eBStatClearCache = 9,         /* Clear the cache */
	eBStatReset_51 = 0x33,			  /* Watchdog reset */
	eBStatReset_52 = 0x34,				/* Software reset */
	eBStatReset_53 = 0x35,        /* Power-off restart (e.g. disconnection of the collection cable) */
	eBStatHeatingWaiting = 0x36,	/* Prepare for heating */
	eBStatHeating = 0x37,					/* Heating in progress */
	eBStatHeatingEnd = 0x38,			/* Heating ends */
	eBStatSOCRecalc = 0x39,			 	/* SOC calibration 1, SOC-OCV static calibration needs to be recorded 2, low power calibration needs to be recorded 3, 3.45V full power calibration does not need to be recorded */
	eBStatHeatFail = 0x40,        /* When heated, the temperature does not rise by 1 degree every 12 minutes, and it fails */
} BASE_STATU_E;
extern BASE_STATU_E g_eBaseStat;

typedef union {
	uint16_t usBaseStat;
	BASE_STAT_S stBaseStat;
} BASE_STAT_U;

typedef struct {
	uint8_t bVoltSensor:1;
	uint8_t bTempSensor:1;
	uint8_t bCurSensor:1;
	uint8_t bInterComm:1;
	uint8_t bInputOV:1;
	uint8_t bInputReverse:1;
	uint8_t bRelay:1;
	uint8_t bBatErr:1;
} ERR_CODE_S;

typedef union {
	ERR_CODE_S stErrCode;
	uint8_t ucErrCode;
} ERR_CODE_U;
typedef struct {
	uint16_t bCellUV:1;
	uint16_t bCellOV:1;
	uint16_t bArrayUV:1;
	uint16_t bArrayOV:1;
	uint16_t bChgUT:1;
	uint16_t bChgOT:1;
	uint16_t bDsgUT:1;
	uint16_t bDsgOT:1;
	uint16_t bChgOCC:1;
	uint16_t bDsgODC:1;
	uint16_t bPackUV:1;
	uint16_t bPackOV:1;
	uint16_t bReserve:1;
	uint16_t bEnvUT:1;
	uint16_t bEnvOT:1;
	uint16_t bMosOT:1;
} ALM_S;

typedef union {
	uint16_t usAlmCode;
	ALM_S stAlmCode;
} ALM_U;

typedef struct {
	uint16_t bCellUV:1;
	uint16_t bCellOV:1;
	uint16_t bArrayUV:1;
	uint16_t bArrayOV:1;
	uint16_t bChgUT:1;
	uint16_t bChgOT:1;
	uint16_t bDsgUT:1;
	uint16_t bDsgOT:1;
	uint16_t bChgOCC:1;
	uint16_t bDsgODC:1;
	uint16_t bPackUV:1;
	uint16_t bPackOV:1;
	uint16_t bSC:1;
	uint16_t bEnvUT:1;
	uint16_t bEnvOT:1;
	uint16_t bMosOT:1;
} PTCT_S;

typedef union {
	uint16_t usPtctCode;
	PTCT_S stPtctCode;
} PTCT_U;

typedef enum {
	eLocalStatInit = 0,
	eLocalStatInitFail,
    eAppStatConsult,
	eLocalStatSlp,
	eLocalStatRun,
	eLocalStatStop,
	eLocalStatCanUpgrade,
	eLocalStatUartBootMode,
	eLocalStatUartUpgrade,
	eLocalStatNum
} LOCAL_STAT_E;






typedef struct{
    uint16_t afCellU[CFG_CELL_NUM];			/* Cell voltage, unit: V */
    int16_t  afCellT[CFG_CELL_NUM];			/* 电芯温度, 单位: ℃ */
    int16_t  fEnvT;
    int16_t  fMosT;											/* MOS temperature, unit: °C */
    int16_t  fPackCur;										/* Total current, unit: A */
    uint16_t fPackU;											/* Total voltage, unit: V */
    uint16_t fPackRealAH;								/* PACK actual rated capacity */
	uint16_t fPackLeftAH;								/* PACK remaining capacity */
    uint16_t fPackSoc;										/* Total Soc, Unit: % */
	uint16_t fPackSoh;										/* Total Soh, Unit: % */
    
    uint16_t fOV;			/* 经过修正的最大充电电压, 单位: V 过压阈值*/
	uint16_t fUV;			/* 经过修正的最低放电电压, 单位：V 欠压阈值*/
	uint16_t fOCC;			/* 经过修正的最大充电电流, 单位: A 充电过流阈值*/
	uint16_t fODC;			/* 经过修正的最大放电电流, 单位: A 放电过流阈值*/

    uint16_t fCellUMax;	/* 最高电芯电压, 单位: V */
	uint16_t fCellUMin;	/* 最低电芯电压, 单位: V */
	uint16_t ucCellUMaxId;	/* 最高电压电芯ID */
	uint16_t ucCellUMinId;	/* 最低电压电芯ID */
	int16_t  fCellTMax;	/* 最高电芯温度, 单位: V */
	int16_t  fCellTMin;	/* 最低电芯温度, 单位: V */
	uint16_t ucCellTMaxId;	/* 最高温度电芯ID */
	uint16_t ucCellTMinId;	/* 最低温度电芯ID */
    ALM_U uAlmCode;				/* 告警码, bit0~11: 单体低压, 单体高压, PACK组低压, PACK组高压, 
													充电低温, 充电高温, 放电低温, 放电高温,
													充电过流, 放电过流, PACK低压, PACK高压,
													备用, 环境低温, 环境高温, MOS高温*/
	PTCT_U uPtctCode;			/* 保护码, bit0~11: 单体低压, 单体高压, PACK组低压, PACK组高压,
													充电低温, 充电高温, 放电低温, 放电高温,
													充电过流, 放电过流, PACK低压, PACK高压,
													短路保护, 环境低温, 环境高温, MOS高温 */
    ERR_CODE_U uErrCode;							/* Error code, bit0~7: voltage sensor failure, temperature sensor failure, internal communication failure, input overvoltage failure,
																				Input Reverse Fault, Relay Detection Fault, Battery Damage Fault, Other Fault (see Fault Extension field for details) */
    uint8_t ucErrCodeEx;	/* 扩展故障码, bit0~5: 关机电路异常, BMIC异常, 内部总线异常, 开机自检异常, 安全功能异常 */
    uint16_t ucChgEn;									/* 0x55 charging allowed, 0xAA means forbidden, other values are invalid */
	uint16_t ucDsgEn;									/* 0x55 discharge allowed, 0xAA means forbidden, other values are invalid */
    uint16_t ucBtn_status;
    BASE_STAT_U uBaseStat;		/* 基本状态, bit0~2: 0休眠, 1充电, 2放电, 3搁置, bit3: 请求强充, bit4: 请求均充 */
    uint16_t usCycle;	        /* Number of cycles, unit: times */


}LOCAL_PACK_REPORT_S;


/*单个包的实时运行状态信息*/
typedef struct {
	
    uint16_t afCellRealAH[CFG_CELL_NUM];	/* The number of cell AH, the actual capacity, the default and design capacity are the same */
	int16_t  afCellLeftAH[CFG_CELL_NUM];	/* Number of cells AH, remaining capacity */
    uint16_t afCellSoc[CFG_CELL_NUM];		/* Cell Soc, Unit: % */
	uint16_t afCellSoh[CFG_CELL_NUM];		/* Cell Soh, Unit: % */
    LOCAL_PACK_REPORT_S astPackReport;						
} LOCAL_PACK_RVAL_S;

//typedef enum {
//	eLocalStatInit = 0,
//	eLocalStatInitFail,
//	eLocalStatSlp,
//	eLocalStatRun,
//	eLocalStatStop,
//	eLocalStatCanUpgrade,
//	eLocalStatUartBootMode,
//	eLocalStatUartUpgrade,
//	eLocalStatNum
//} LOCAL_STAT_E;

/* Real-time operating condition information of the battery pack */
//typedef struct {
//	LOCAL_PACK_RVAL_S astPackRVal[PRL_MAX_NODE_NUM];
//    uint16_t fRealAH;	/* Actual rated capacity */
//	uint16_t fLeftAH;	/* Remaining capacity */
//	uint16_t fU;		/* Total battery pack voltage, unit: V */
//	uint16_t fCur;	/* Total battery pack current, unit: A */
//    
//    
//	uint16_t fSoc;	/* Battery pack average Soc, unit: % */
//	uint16_t fSoh;	/* Battery pack average Soh, unit: % */

//	uint16_t fLmtChgI;	/* Modified maximum charge current, unit: A */
//	uint16_t fLmtDsgI;	/* Modified maximum charge current, unit: A */
//	uint16_t fReqChgI;	/* request charge current, unit: A */
//	uint16_t fPeakLmtDsgI;	/* Modified maximum peak discharge current, unit: A */
//	uint16_t fPackUMax;	/* The maximum PACK voltage in the battery pack, unit: V */
//	uint16_t fPackUMin;	/* The lowest PACK voltage in the battery pack, unit: V */
//	uint16_t usPackUMaxId;	/* The highest voltage in the battery pack PACK ID */
//	uint16_t usPackUMinId;	/* The lowest voltage in the battery pack PACK ID */
//	uint16_t fPackTMax;	/* The highest cell temperature in a battery pack, unit: °C */
//	uint16_t fPackTMin;	/* The lowest cell temperature in a battery pack, uint: °C */
//	uint16_t usPackTMaxId;	/* The highest temperature in the battery pack is the PACK ID, and the quotient after dividing 256 is the PACK ID, and the remainder is the sampling channel ID in the pack */
//	uint16_t usPackTMinId;	/* The lowest temperature in the battery pack is the PACK ID, and the quotient after dividing 256 is the PACK ID, and the remainder is the sampling channel ID in the pack */
//	uint16_t fCellUMax;	/* The highest cell voltage in a battery pack, unit: V */
//	uint16_t fCellUMin;	/* The lowest cell voltage in a battery pack, unit: V */
//	uint16_t usCellUMaxId;	/* The cell ID with the highest voltage in the battery pack, divided by 256, is the PACK ID, and the remainder is the cell ID in the pack */
//	uint16_t usCellUMinId;	/* The cell ID with the lowest voltage in the battery pack, the quotient after dividing 256 is the PACK ID, and the remainder is the cell ID in the pack */
//	uint16_t fUDiff;					/* parallel packs voltage difference, unit: mV */
//	BASE_STAT_U uBaseStat;		/* Basic state, bit0~2: 0 sleep, 1 charge, 2 discharge, 3 shelf, bit3: request for strong charge, bit4: request for equal charge */
//	ERR_CODE_U uErrCode;		/* Error Code, bit0~7: Voltage sensor failure, Temperature sensor failure, Internal communication failure, Input overvoltage failure,
//													Input Reverse Fault, Relay Detection Fault, Battery Damage Fault, Other Fault (see Fault Extension field for details) */
//    ALM_U uAlmCode;				/* 告警码, bit0~11: 单体低压, 单体高压, PACK组低压, PACK组高压, 
//													充电低温, 充电高温, 放电低温, 放电高温,
//													充电过流, 放电过流, PACK低压, PACK高压,
//													备用, 环境低温, 环境高温, MOS高温*/
//	PTCT_U uPtctCode;			/* 保护码, bit0~11: 单体低压, 单体高压, PACK组低压, PACK组高压,
//													充电低温, 充电高温, 放电低温, 放电高温,
//													充电过流, 放电过流, PACK低压, PACK高压,
//													短路保护, 环境低温, 环境高温, MOS高温 */
//    uint8_t ucErrCodeEx;	/* 扩展故障码, bit0~5: 关机电路异常, BMIC异常, 内部总线异常, 开机自检异常, 安全功能异常 */

//    uint16_t ucChgEn;			/* 0x55 charging allowed, 0xAA means forbidden, other values are invalid */
//	uint16_t ucDsgEn;			/* 0x55 discharge allowed, 0xAA means forbidden, other values are invalid */
//	LOCAL_STAT_E eLocalStat;
//} LOCAL_ARRAY_RVAL_S;


typedef struct {
    uint8_t g_ucChgRecvFlag;            //允许充电自恢复
    uint8_t g_ucDsgRecvFlag;            //允许放电自恢复
}LOCAL_FLAG_S;

extern LOCAL_PACK_RVAL_S g_stLocalPackRVal[PRL_MAX_NODE_NUM];
extern LOCAL_FLAG_S g_stLocalFlagWVal;
//extern LOCAL_ARRAY_RVAL_S g_stLocalArrayRVal;
extern LOCAL_STAT_E g_stLocalStat;


//extern uint16_t local_read_reg(uint8_t ucSelfId, uint8_t ucDevAddr, uint16_t usRegAddr);



extern bool local_init(void);
static void local_set_soc(float fVal, uint8_t ucIdx);
extern uint8_t local_get_soc(uint8_t ucIdx);
static void local_refresh(void);
static void local_alm_ptct(void);
static void local_chg_dsg_ctrl(void);
static void local_led_proc(void);
extern void local_proc(void);
extern bool local_read_reg(uint8_t ucSelfId, uint8_t ucDevAddr, uint16_t usRegAddr ,uint16_t* pfVal);
extern bool local_Write_reg(uint8_t ucSelfId, uint8_t ucDevAddr, uint16_t usRegAddr,uint16_t * pfVal);
#pragma pack()

#endif
