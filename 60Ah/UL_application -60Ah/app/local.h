#ifndef LOCAL_H
#define LOCAL_H

#include <stdbool.h>
#include <stdint.h>

#include "parallel.h"
#include "eco.h"
#include "main.h"

#ifdef USER_DEBUG
#define LOCAL_DEBUG_EN	1		/* 0: non-DEBUG state, 1: DEBUG status */
#else
#define LOCAL_DEBUG_EN 0
#endif

#define MAX_RETRIES     2   /* 最大重试次数 */

#define LOCAL_DEBUG(fmt,arg...)	do{if(LOCAL_DEBUG_EN){printf(fmt,##arg);printf("[%s][%d]\r\n",__func__,__LINE__);}}while(0)
#define LOCAL_RETURN_FALSE	do{LOCAL_DEBUG("Return failed");return false;}while(0)
//#define LOCAL_RETURN_TRUE do{LOCAL_DEBUG("Return success");return true;}while(0)
#define LOCAL_RETURN_TRUE do{return true;}while(0)

#pragma pack(1)

extern uint32_t g_auiAlmCode0[4];
extern uint32_t g_auiAlmCode1[4];
extern uint8_t g_ucAlmLevel;
extern uint8_t g_ucAlmLevel2;
extern uint8_t g_ucAlmLeve3;
extern bool g_BalanceFlag;
extern bool g_BalanceFlag_2;
extern uint8_t g_ucChgFull;
extern uint8_t g_ucFullChgRec;
extern uint8_t balanc[16]; 
extern bool g_bNeedHeat;
extern bool g_bTempIsNotRisingFlag;
extern bool g_bHeatingTimeoutFlag;
extern bool g_bHeatingOverNumberFlag;
extern float g_fPackSoc;

#define SET_ALM0_CODE(idx) if(GET_ALM0_CODE(idx)==0){g_auiAlmCode0[(idx-1)/32]|=((uint32_t)0x00000001 << ((idx-1)%32));/*if(eco_refresh_log()){his_log_write();}*/}
#define SET_ALM1_CODE(idx) if(GET_ALM1_CODE(idx)==0){g_auiAlmCode1[(idx-1)/32]|=((uint32_t)0x00000001 << ((idx-1)%32));if(eco_refresh_log()){eco_refresh_RTV();his_data_write();his_log_write();}}
#define RESET_ALM0_CODE(idx) if(GET_ALM0_CODE(idx)==1){g_auiAlmCode0[(idx-1)/32]&=(~((uint32_t)0x00000001 << ((idx-1)%32)));}
#define RESET_ALM1_CODE(idx) if(GET_ALM1_CODE(idx)==1){g_auiAlmCode1[(idx-1)/32]&=(~((uint32_t)0x00000001 << ((idx-1)%32)));}
#define GET_ALM0_CODE(idx) (g_auiAlmCode0[(idx-1)/32]>>((idx-1)%32)&0x01)
#define GET_ALM1_CODE(idx) (g_auiAlmCode1[(idx-1)/32]>>((idx-1)%32)&0x01)

#define LOCAL_CUR_DZ	2000	/* current dead zone, unit: mA */

/* 1. Charge request current */
#define LOCAL_CR_ROWS	9
#define LOCAL_CR_COLUMNS	13

typedef struct {
	float afTemp[LOCAL_CR_ROWS];
	float afSoc[LOCAL_CR_COLUMNS];
	float aafChgRate[LOCAL_CR_ROWS][LOCAL_CR_COLUMNS];
} LOCAL_CR_PARA_S;

/* 2. Charge limit current */
#define LOCAL_CL_ROWS1	7
#define LOCAL_CL_ROWS2	6
#define LOCAL_CL_COLUMNS1	6
#define LOCAL_CL_COLUMNS2	9
//#define LOCAL_CL_ROWS	16
//#define LOCAL_CL_COLUMNS	14

typedef struct {
	float afTempMin[LOCAL_CL_ROWS1];
	float afTempMax[LOCAL_CL_ROWS2];
	float afCellVMax[LOCAL_CL_COLUMNS1];
	float afCellVMin[LOCAL_CL_COLUMNS2];
	float aafChgRate1[LOCAL_CL_ROWS1][LOCAL_CL_COLUMNS1];
	float aafChgRate2[LOCAL_CL_ROWS1][LOCAL_CL_COLUMNS2];
	float aafChgRate3[LOCAL_CL_ROWS2][LOCAL_CL_COLUMNS1];
	float aafChgRate4[LOCAL_CL_ROWS2][LOCAL_CL_COLUMNS2];
//	float afTemp[LOCAL_CL_ROWS];
//	float afSoc[LOCAL_CL_COLUMNS];
//	float aafChgRate[LOCAL_CL_ROWS][LOCAL_CL_COLUMNS];
} LOCAL_CL_PARA_S;

/* 3. Discharge current limit */
#define LOCAL_DL_ROWS1	7
#define LOCAL_DL_ROWS2	6
#define LOCAL_DL_COLUMNS1	6
#define LOCAL_DL_COLUMNS2	9
//#define LOCAL_DL_ROWS	10
//#define LOCAL_DL_COLUMNS	14

typedef struct {
	float afTempMin[LOCAL_DL_ROWS1];
	float afTempMax[LOCAL_DL_ROWS2];
	float afCellVMax[LOCAL_DL_COLUMNS1];
	float afCellVMin[LOCAL_DL_COLUMNS2];
	float aafDsgRate1[LOCAL_DL_ROWS1][LOCAL_DL_COLUMNS1];
	float aafDsgRate2[LOCAL_DL_ROWS1][LOCAL_DL_COLUMNS2];
	float aafDsgRate3[LOCAL_DL_ROWS2][LOCAL_DL_COLUMNS1];
	float aafDsgRate4[LOCAL_DL_ROWS2][LOCAL_DL_COLUMNS2];
//	float afTemp[LOCAL_DL_ROWS];
//	float afSoc[LOCAL_DL_COLUMNS];
//	float aafDsgRate[LOCAL_DL_ROWS][LOCAL_DL_COLUMNS];
} LOCAL_DL_PARA_S;

/* 4. 3s PEAK discharge current limit */
#define LOCAL_PDCL_ROWS1	7
#define LOCAL_PDCL_ROWS2	6
#define LOCAL_PDCL_COLUMNS1	6
#define LOCAL_PDCL_COLUMNS2	9
//#define LOCAL_3SPDCL_ROWS	10
//#define LOCAL_3SPDCL_COLUMNS	14

typedef struct {
	float afTempMin[LOCAL_PDCL_ROWS1];
	float afTempMax[LOCAL_PDCL_ROWS2];
	float afCellVMax[LOCAL_PDCL_COLUMNS1];
	float afCellVMin[LOCAL_PDCL_COLUMNS2];
	float aafDsgRate1[LOCAL_PDCL_ROWS1][LOCAL_PDCL_COLUMNS1];
	float aafDsgRate2[LOCAL_PDCL_ROWS1][LOCAL_PDCL_COLUMNS2];
	float aafDsgRate3[LOCAL_PDCL_ROWS2][LOCAL_PDCL_COLUMNS1];
	float aafDsgRate4[LOCAL_PDCL_ROWS2][LOCAL_PDCL_COLUMNS2];
//	float afTemp[LOCAL_3SPDCL_ROWS];
//	float afSoc[LOCAL_3SPDCL_COLUMNS];
//	float aafDsgRate[LOCAL_3SPDCL_ROWS][LOCAL_3SPDCL_COLUMNS];
} LOCAL_PDCL_PARA_S;/*LOCAL_3SPDCL_PARA_S;*/

///* 5. 30s PEAK discharge current limit */
//#define LOCAL_30SPDCL_ROWS	18
//#define LOCAL_30SPDCL_COLUMNS	14

//typedef struct {
//	float afTemp[LOCAL_30SPDCL_ROWS];
//	float afSoc[LOCAL_30SPDCL_COLUMNS];
//	float aafDsgRate[LOCAL_30SPDCL_ROWS][LOCAL_30SPDCL_COLUMNS];
//} LOCAL_30SPDCL_PARA_S;

/* 6. PEAK discharge limit (based upon MOS temperature) */
#define LOCAL_PDCLMOS_ROWS	10

typedef struct {
	float afTemp[LOCAL_PDCLMOS_ROWS];
	float afRate[LOCAL_PDCLMOS_ROWS];
} LOCAL_PDCLMOS_PARA_S;

/* 7. Standby soc calibration */
//typedef struct {
//	float afTemp[7];
//	float afSoc[21];
//	float aafCellU[21][7];
//} LOCAL_SOC_CALI_S;
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
#define LOCAL_CUR_RESIST	(0.002/24)		/* unit: Ω */

typedef struct {
	uint8_t ucChg:1;					/* charge */
	uint8_t ucDsg:1;					/* discharge */
	uint8_t ucStandby:1;			/* shelve */
	uint8_t ucChgerON:1;			/* charger is connected */
	uint8_t ucHeating:1;			/* heater on */
	uint8_t ucDCOut1:1;				/* 1A DC output on */
	uint8_t ucDCOut2:1;				/* 3A DC output on */
} BASE_STAT_S;

typedef struct{
	uint16_t ALM_CODE1;
	uint16_t ALM_CODE2;
	uint16_t ALM_CODE3;
	uint16_t ALM_CODE4;
	uint16_t ALM_CODE5;
	uint16_t ALM_CODE6;
	uint16_t ALM_CODE7;
	uint16_t ALM_CODE8;
	uint16_t ALM_CODE9;
	uint16_t ALM_CODE10;
	uint16_t ALM_CODE11;
	uint16_t ALM_CODE12;
	uint16_t ALM_CODE13;
	uint16_t ALM_CODE14;
	uint16_t ALM_CODE15;
	uint16_t ALM_CODE16;
	uint16_t ALM_CODE17;
//	uint8_t ALM_CODE18;
	uint16_t ALM_CODE53;
	uint16_t ALM_CODE54;
	uint16_t ALM_CODE55;
	uint16_t ALM_CODE56;
	uint16_t ALM_CODE57;
	uint16_t ALM_CODE64;
} BASE_ALM_CODE_E;
extern BASE_ALM_CODE_E g_eBaseAlm;

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
	eBStatSOCRecalc = 0x14,			 	/* SOC calibration 1, SOC-OCV static calibration needs to be recorded 2, low power calibration needs to be recorded 3, 3.45V full power calibration does not need to be recorded */
	eBStatLowTemp = 0x18,         /* Triggered when the temperature is below 0°C */
	eBStatReset_51 = 0x33,			  /* Watchdog reset */
	eBStatReset_52 = 0x34,				/* Software reset */
	eBStatReset_53 = 0x35,        /* Power-off restart (e.g. disconnection of the collection cable) */
	eBStatHeatingWaiting = 0x36,	/* Prepare for heating */
	eBStatHeating = 0x37,					/* Heating in progress */
	eBStatHeatingEnd = 0x38,			/* Heating ends */
//	eBStatSOCRecalc = 0x39,			 	/* SOC calibration 1, SOC-OCV static calibration needs to be recorded 2, low power calibration needs to be recorded 3, 3.45V full power calibration does not need to be recorded */
	eBStatTempIsNotRising = 0x40,	/* When heated, the temperature does not rise by 2 degree every 15 minutes, and it fails */
	eBStatHeatingTimeout = 0x41, 	/* Heat up for more than 90 minutes */
	eBStatHeatingOverNumber = 0x42,	/* The number of heating times reaches 10 times */
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

/* Real-time operating condition information of a single pack */
typedef struct {
	float afCellU[CFG_CELL_NUM];			/* Cell voltage, unit: V */
	float afCellRealAH[CFG_CELL_NUM];	/* The number of cell AH, the actual capacity, the default and design capacity are the same */
	float afCellLeftAH[CFG_CELL_NUM];	/* Number of cells AH, remaining capacity */
	float afCellSoc[CFG_CELL_NUM];		/* Cell Soc, Unit: % */
	float afCellSoh[CFG_CELL_NUM];		/* Cell Soh, Unit: % */
	float afCellT[CFG_TMP_NUM];				/* Cell temperature, unit: °C */
	float fPackRealAH;								/* PACK actual rated capacity */
	float fPackLeftAH;								/* PACK remaining capacity */
	float fPackU;											/* Total voltage, unit: V */
	float fPackUEx;										/* voltage from B+ and B-, sample from MCU ADC */
	float fPackCur;										/* Total current, unit: A */
	float fMosT;											/* MOS temperature, unit: °C */
	float fPackSoc;										/* Total Soc, Unit: % */
	float fPackSoh;										/* Total Soh, Unit: % */
	float fLmtChgI;										/* Modified maximum charge current, unit: A */
	float fLmtDsgI;										/* Modified maximum discharge current, unit: A */
	float fReqChgI;										/* request charge current, unit: A */
	float fPeakLmtDsgI;								/* Modified maximum peak discharge current, unit: A */
	float fCellUMax;									/* Maximum cell voltage, unit: V */
	float fCellUMin;									/* Minimum cell voltage, unit: V */
	float fPackCurMax;								/* max current in history */
	uint8_t ucCellUMaxId;							/* Highest voltage cell ID */
	uint8_t ucCellUMinId;							/* Minimum voltage cell ID */
	float fCellTMax;									/* Maximum cell temperature, unit: °C */
	float fCellTMin;									/* Minimum cell temperature, unit: °C */
	uint8_t ucCellTMaxId;							/* Maximum temperature cell ID */
	uint8_t ucCellTMinId;							/* Minimum temperature cell ID */
	BASE_STAT_U uBaseStat;						/* Basic state */
	uint16_t usCycle;									/* Number of cycles, unit: times */
	ERR_CODE_U uErrCode;							/* Error code, bit0~7: voltage sensor failure, temperature sensor failure, internal communication failure, input overvoltage failure,
																			Input Reverse Fault, Relay Detection Fault, Battery Damage Fault, Other Fault (see Fault Extension field for details) */
	uint8_t ucChgEn;									/* 0x55 charging allowed, 0xAA means forbidden, other values are invalid */
	uint8_t ucDsgEn;									/* 0x55 discharge allowed, 0xAA means forbidden, other values are invalid */
	uint8_t ucHeatEn;									/* 0x55- heater on enable, 0xAA- heater on disable */
	uint8_t ucChgForceEn;							/* 0x55 charging is forced, 0xAA means forbidden, other values are invalid */
	uint8_t ucDsgForceEn;							/* 0x55 discharge forced, 0xAA means forbidden, other values are invalid */
	uint8_t ucHeatForceEn;						/* 0- not forced, or software force heater MOS on */
	float fCDATA;
} LOCAL_PACK_RVAL_S;

typedef enum {
	eLocalStatInit = 0,
	eLocalStatInitFail,
	eLocalStatSlp,
	eLocalStatRun,
	eLocalStatStop,
	eLocalStatCanUpgrade,
	eLocalStatUartBootMode,
	eLocalStatUartUpgrade,
	eLocalStatNum
} LOCAL_STAT_E;

/* Real-time operating condition information of the battery pack */
typedef struct {
	LOCAL_PACK_RVAL_S astPackRVal[PRL_MAX_NODE_NUM];
	float fRealAH;	/* Actual rated capacity */
	float fLeftAH;	/* Remaining capacity */
	float fU;		/* Total battery pack voltage, unit: V */
	float fCur;	/* Total battery pack current, unit: A */
	float fSoc;	/* Battery pack average Soc, unit: % */
	float fSoh;	/* Battery pack average Soh, unit: % */
	float fLmtChgI;	/* Modified maximum charge current, unit: A */
	float fLmtDsgI;	/* Modified maximum charge current, unit: A */
	float fReqChgI;	/* request charge current, unit: A */
	float fPeakLmtDsgI;	/* Modified maximum peak discharge current, unit: A */
	float fPackUMax;	/* The maximum PACK voltage in the battery pack, unit: V */
	float fPackUMin;	/* The lowest PACK voltage in the battery pack, unit: V */
	uint16_t usPackUMaxId;	/* The highest voltage in the battery pack PACK ID */
	uint16_t usPackUMinId;	/* The lowest voltage in the battery pack PACK ID */
	float fPackTMax;	/* The highest cell temperature in a battery pack, unit: °C */
	float fPackTMin;	/* The lowest cell temperature in a battery pack, uint: °C */
	uint16_t usPackTMaxId;	/* The highest temperature in the battery pack is the PACK ID, and the quotient after dividing 256 is the PACK ID, and the remainder is the sampling channel ID in the pack */
	uint16_t usPackTMinId;	/* The lowest temperature in the battery pack is the PACK ID, and the quotient after dividing 256 is the PACK ID, and the remainder is the sampling channel ID in the pack */
	float fCellUMax;	/* The highest cell voltage in a battery pack, unit: V */
	float fCellUMin;	/* The lowest cell voltage in a battery pack, unit: V */
	uint16_t usCellUMaxId;	/* The cell ID with the highest voltage in the battery pack, divided by 256, is the PACK ID, and the remainder is the cell ID in the pack */
	uint16_t usCellUMinId;	/* The cell ID with the lowest voltage in the battery pack, the quotient after dividing 256 is the PACK ID, and the remainder is the cell ID in the pack */
	float fUDiff;					/* parallel packs voltage difference, unit: mV */
	BASE_STAT_U uBaseStat;		/* Basic state, bit0~2: 0 sleep, 1 charge, 2 discharge, 3 shelf, bit3: request for strong charge, bit4: request for equal charge */
	ERR_CODE_U uErrCode;		/* Error Code, bit0~7: Voltage sensor failure, Temperature sensor failure, Internal communication failure, Input overvoltage failure,
													Input Reverse Fault, Relay Detection Fault, Battery Damage Fault, Other Fault (see Fault Extension field for details) */
	uint8_t ucChgEn;			/* 0x55 charging allowed, 0xAA means forbidden, other values are invalid */
	uint8_t ucDsgEn;			/* 0x55 discharge allowed, 0xAA means forbidden, other values are invalid */
	LOCAL_STAT_E eLocalStat;
} LOCAL_ARRAY_RVAL_S;

extern LOCAL_ARRAY_RVAL_S g_stLocalArrayRVal;
extern BASE_ALM_CODE_E g_eBaseAlm_Parallel[PRL_MAX_NODE_NUM];

static void local_cur_lmt(float* pfLmtChgI, float* pfLmtDsgI, float* pfReqChgI, float* pfPeakLmtDsgI);
extern bool local_init(void);
static void local_soc_cali_save(void);
extern void local_standby_leftAH_cali(void);
static void local_info_refresh(void);
static void local_signal_ptct(void);
//static void local_fsm(void);
static void local_state_machine(void);
static void local_mos_ctl(void);
extern void local_balance_proc(void);
extern bool local_proc(void);
extern uint16_t local_read_reg(uint8_t ucSelfId, uint8_t ucDevAddr, uint16_t usRegAddr);

#pragma pack()

#endif
