#ifndef ECO_CAN_H
#define ECO_CAN_H

#include "bsp_can.h"
#include "config.h"

#include <stdint.h>
#include <time.h>

#pragma pack(1)

typedef struct {
	/* 0x00 */
	uint16_t usPackNominalVolt;			//pack nominal voltage, unit: V
	uint16_t usPackNominalAh;				//pack nominal Ah, unit: 0.1Ah
	uint16_t usCellSeriesNum;				//number of cell series, unit: 1 pcs cell
	uint16_t usPackRTVolt;					//pack realtime volrage, unit: 0.1V
	int16_t  sPackRTCur;					  //pack realtime current, unit: 0.1A
	uint16_t usPackRTSoc;						//pack realtime Soc, unit: %
	uint16_t usPackRTSoh;						//pack realtime Soh, unit: 0.1%
	uint16_t usCycleCnt;						//cycle count, unit: 1 count
	
	uint16_t bSWCellODT:1;					//cell over temperature when discharging from mcu
	uint16_t bVDIFFW:1;							//Cell Volt. Difference is too big
	uint16_t bHWOC:1;								//over current from afe
	uint16_t bHWCellOV:1;						//cell over voltage from afe
	
	uint16_t bHWCellUV:1;						//cell under voltage from afe
	uint16_t bSWPackUV:1;						//pack under voltage from mcu
	uint16_t bSWPackOV:1;						//pack over voltage from mcu
	uint16_t bHWSC:1;								//short circuit from afe
	
	uint16_t bSWOCC:1;							//over current when charging from mcu
	uint16_t bSWODC:1;							//over current when dischaging from mcu
	uint16_t bSWCellUDT:1;					//cell under temperatrue when discharging from mcu
	uint16_t bSWCellUCT:1;					//cell under temperatrue when charging from mcu
	
	uint16_t bSWMOSOT:1;						//MOSFET over temperatrue from mcu
	uint16_t bSWOCT:1;							//cell over temperatrue when charging from mcu
	uint16_t bSWCellOV:1;						//cell over voltage from mcu
	uint16_t bSWCellUV:1;						//cell under voltage from mcu
	
	uint16_t bsWSocLow:1;						//Too Low SOC
	uint16_t bNeedHeat:1;
	uint16_t bSetMChgerAct:1;
	uint16_t bSetSChgerAct:1;
	
	uint16_t bSocCali:1;						//Soc calibration event
	uint16_t bHeaterOn:1;						//heat realy is on
	uint16_t bBAL:1;								//balance active
	uint16_t MosAnomaly:1;					//MOS Anomaly
	
	uint16_t bTempSersorErr:1;			//temperature sensor error
	uint16_t bVoltSensor:1;					//voltage sensor error
	uint16_t bDMOSOn:1;							//discharge MOSFET is on
	uint16_t bCMOSOn:1;							//charge MOSFET is on
	
	uint16_t bPreDsgOn:1;						//bypass is on
	uint16_t bDMOSForceOn:1;				//discharge MOSFET is force on
	uint16_t bCMOSForceOn:1;				//charge MOSFET is force on
	uint16_t bHMOSForceOn:1;				//heat MOSFET is force on
	
	uint16_t usCellUVTVThr1;				//cell under voltage lv.1 trigger value threshold, unit: mV
	uint16_t usCellOVTVThr1;				//cell over voltage lv.1 trigger value threshold, unit: mV
	int16_t  sCellOCTTVThr1;				//cell over temperatrue lv.1 trigger value threshold when charging, unit: ¡æ
	int16_t  sMOSOTTVThr2;					//MOSFET over temperatrue lv.2 trigger value threshold, unit: ¡æ
	int16_t  sCellUCTTVThr1;				//cell under temperatrue lv.1 trigger value threshold when charging, unit: ¡æ
	int16_t  sCellUDTTVThr1;				//cell under temperatrue lv.1 trigger value threshold when discharging, uint: ¡æ
	/* 0x10 */
	uint16_t usODCTVThr1;						//over current lv.1 trigger value threshold when discharging, unit: A
	uint16_t usOCCTVThr1;						//over current lv.1 trigger value threshold when charging, unit: A
	uint16_t usCellUVRVThr1;				//cell under voltage lv.1 recover value threshold, unit: mV
	uint16_t usCellOVRVThr1;				//cell over voltage lv.1 recover value threshold, unit: mV
	int16_t  sCellOCTRVThr1;				//cell over temperatrue lv.1 recover value threshold when charging, unit: ¡æ
	int16_t  sMOSOTRVThr2;					//MOSFET over lv.2 temperatrue recover value threshold, unit: ¡æ
	int16_t  sMOSRTT1;							//MOSFET realtime temperatrue1, unit: ¡æ
	int16_t  sCellRTT1;							//cell realtime temperatrue1, unit: ¡æ
	int16_t  sCellRTT2;							//cell realtime temperatrue2, unit: ¡æ
	int16_t  sCellRTT3;							//cell realtime temperatrue3, unit: ¡æ
	int16_t  sCellRTT4;							//cell realtime temperatrue4, unit: ¡æ
	int16_t  sMOSRTT2;							//MOSFET realtime temperatrue2, unit: ¡æ
	int16_t  sCellUCTRVThr1;				//cell under temperatrue lv.1 recover value threshold when charging, unit: ¡æ
	int16_t  sCellUDTRVThr1;				//cell under temperatrue lv.1 recover value threshold when discharging, unit: ¡æ
	int16_t  sCellODTTVThr1;				//cell over temperatrue lv.1 trigger value threshold when discharging, unit: ¡æ
	int16_t  sCellODTRVThr1;				//cell over temperatrue lv.1 recover value threshold when discharging, unit: ¡æ
	/* 0x20 */
	uint16_t usHWOCTVThr;						//hardware over current trigger value thredshold, SH367309-SC value code, default: 0x01
	uint16_t ausCellRTVolt[CFG_CELL_NUM];		//cell realtime voltage, unit: mV
	/* 0x20 + CFG_CELL_NUM + 2 */
	uint16_t usBALActVolt;					//balance active voltage, unit: mV
	uint16_t usBALActDVolt;					//balance active delta voltage, unit: mV
	uint16_t usVALReactDVolt;				//balance reactive delta voltage, unit: mV
	uint16_t usRTSlpCnt;						//realtime sleep count, unit: s //should usRTSlpCnt & usVALReactDVolt are int16_t?(h00205922)
	int16_t  sStandbySlpCnt;				//standby sleep count, unit: s
	uint16_t usDbgPara;							//debug parameter, unused
	uint16_t usHWSCTVThr;						//hardware short circuit trigger value threshold, SH367309-SC time code, default: 0x00
	uint16_t usMOSForceOn;					//0-do not force charge and discharge MOSFET on, 1-force charge and discharge MOSFET on
	uint16_t usPackUVTVThr1;				//pack under voltage lv.1 trigger value threshold, unit: 0.1V
	uint16_t usPackOVTVThr1;				//pack over voltage lv.1 trigger value threshold, unit: 0.1V
	/* 0x20 + CFG_CELL_NUM + 12 */
	uint16_t usPackUVRVThr1;				//pack under voltage lv.1 recover value threshold, unit: 0.1V
	uint16_t usPackOVRVThr1;				//pack over voltage lv.1 recover value threshold, unit: 0.1V
	int16_t  sChgCurSnrCoeB;				//charge current senser coefficient B, unit: micor ohms
	int16_t  sDsgCurSnrCoeB;				//diacharge current senser coefficient B, unit: micor ohms
	int16_t  sVoltSnrCoeB;					//voltage senser coefficient B, unit: mV
	uint16_t usReset;								//BMS reset 
	int16_t usSetDefault;					  //set default configuration
	uint16_t ausRTC[3];
	/* 0x20 + CFG_CELL_NUM + 22 */
	int16_t  sHeaterUTTVThr;				//heater temperature trigger value threshold, unit: ¡æ
	uint16_t usCellOCTTTThr1;				//cell over temperature lv.1 trigger time threshold when charging, unit: 200ms
	uint16_t usCellODTTTThr1;				//cell over temperature lv.1 trigger time threshold when discharging, unit: 200ms
	uint16_t usMOSOTTTThr2;					//MOSFET over temperature lv.2 trigger time threshold, unit: 200ms
	uint16_t usCellUCTTTThr1;				//cell under temperature lv.1 trigger time threshold when charging, unit: 200ms
	uint16_t usCellUDTTTThr1;				//cell under temperature lv.1 trigger time threshold when discharging, unit: 200ms
	uint16_t usCellUVTTThr2;				//cell under voltage lv.2 trigger time threshold, unit: 200ms
	uint16_t usCellOVTTThr1;				//cell over voltage lv.1 trigger time threshold, unit: 200ms
	uint16_t usPackOVTTThr1;				//pack over voltage lv.1 trigger time threshold, unit: 200ms
	uint16_t usPackUVTTThr2;				//pack under voltage lv.2 trigger time threshold, unit: 200ms
	/* 0x20 + CFG_CELL_NUM + 32 */
	uint16_t usODCTTThr1;						//over current trigger lv.1 time threshold when discharging, unit: 200ms
	uint16_t usOCCTTThr1;						//over current trigger lv.1 time threshold when charging, unit: 200ms
	uint16_t usOCCRTThr1;						//over current recover lv.1 time threshold when charging, unit: 200ms
	uint16_t usODCRTThr1;						//over current recover lv.1 time threshold when discharging, unit: 200ms
	uint16_t usSCRTThr;							//short circuit recover time threshold, unit: 200ms
	uint16_t usCurLmtMode;					//current limit mode
//	uint16_t ausReserve6[2];			//reserve
	uint16_t usReserve6;						//reserve
	uint16_t usDevNum;           	  //Number of parallel machines
	int16_t  sLowSocSlpTVThr;				//low power sleep trigger value threshold, unit: min
//	uint16_t usPrlSelfId;					//parallel self id
//	uint16_t usPrlDiffU;					//parallel different pack voltage, unit: mV
//	uint16_t usStandbySocCaliSec;	//standby mode soc calibration time delay, unit: h
//	uint16_t usStandbySocCaliI;		//standby mode soc calibration curren, unit: A
	uint8_t ausSN[8];								//Record the version number
	/* 0x20 + CFG_CELL_NUM + 45 */
	int16_t  sHeaterOTRVThr;				//heater temperature recover value threshold, unit: ¡æ
	uint16_t usSocFCaliU0;					//soc full calibration voltage threshold0, unit: mV
	uint16_t usSocECaliU0;					//soc 5% empty calibration voltage threshold0, unit: mV
	uint16_t usSocECaliU1;					//soc 5% empty calibration voltage threshold1, unit: mV
	uint16_t usSocFCaliU1;					//soc full calibration voltage threshold1, unit: mV
	uint16_t usSocECaliU2;					//soc 0% empty calibration voltage threshold2, unit: mV
	uint16_t usSleep;								//write only, sleep cmd
	uint16_t usPassword;						//generated by bluetooth name
	uint16_t usPackRealAH;       	  //pack real Ah, unit: 0.1Ah
	uint16_t usPackLeftAH;        	//pack left Ah, unit: 0.1Ah
	uint16_t usClearHistory;				//write only, ClearHistory cmd
	uint16_t usSetVolCali;					//write only, set default VolCali configuration
	uint16_t usSetCurCali;					//write only, set default CurCali configuration
	int16_t  sBalanceOn;				 	  //Balance configuration, 1 is on, 0 is off
	uint16_t usBalanceValue;		    //Balance voltage value, unit: mV
	uint16_t usBalanceDiff;		   	  //Balance diff value, unit: mV
	uint16_t usSocCaliDelay;		  	//SOC calibration time delay, unit: h
	uint16_t usSocCaliCurVal;		  	//SOC calibration current value, unit: A
	uint16_t usReqChgCur;						//Request charge current, unit: 0.1A
	uint16_t usPeakLmtDsgCur;				//Peak discharge current limit, unit: 0.1A
	uint16_t usLmtChgCur;						//Normal charge current limit, unit: 0.1A
	uint16_t usLmtDsgCur;						//Normal discharge current limit, unit: 0.1A
	uint16_t usMChgerActCur;				//Main charger active current, unit: 0.1A
	uint16_t usSChgerActCur;				//Main charger active cuurent, unit: 0.1A
	uint16_t usHeatCnt;							//Heat count, increasing when heat start, clear when charger disconnected
	uint16_t usHeatLastTime;				//Continuously heating time, unit: s
	uint16_t ucCanPtcType;					//0-Default(250k), 1-VERSION 'C'(500k), 2-NEW(500k)
	uint16_t usTempIsNotRising;			//When heated, the temperature does not rise by 1 degree every 15 minutes, and it fails
	uint16_t usHeatingTimeout;			//Heat up for more than 90 minutes
	uint16_t usHeatingOverNumber;		//The number of heating times reaches 10 times
	float fCDATA;                   //Static current
//	uint16_t ausCellRTVolt2[CFG_CELL_NUM];		//cell realtime voltage, unit: mV
} ECO_RTV_S;

extern ECO_RTV_S g_stEcoRtv;

typedef struct {
	//alarm code high byte
//	uint8_t abReserve1:3;		//reserve
	uint8_t bTINR:1;				//When heated, the temperature does not rise by 1 degree every 15 minutes, and it fails
	uint8_t bHT:1;					//Heat up for more than 90 minutes
	uint8_t bHON:1;					//The number of heating times reaches 10 times
	uint8_t bSocCali:1;			//Soc calibration event
	uint8_t bTDIFFW:1;			//Alarm if the temperature difference is too large
	uint8_t bVDIFFW:1;			//Alarm if the differential pressure is too large
	uint8_t abReserve2:2;		//reserve
	//alarm code low byte
	uint8_t bOCDW:1;				//Discharge overcurrent alarm
	uint8_t bOCCW:1;				//Charging overcurrent alarm
	uint8_t bOTCW:1;				//Charging high temperature alarm
	uint8_t bOTDW:1;				//Discharge high temperature alarm
	uint8_t bOTMW:1;				//MOS high temperature alarm
	uint8_t MosAnomaly:1;		//MOS Anomaly
	uint8_t abReserve3:2;		//reserve
	//protection code1 high byte
	uint8_t bSWOCC:1;						//over current when charging from mcu
	uint8_t bSWODC:1;						//over current when dischaging from mcu
	uint8_t bSWCellUDT:1;				//cell under temperatrue when discharging from mcu
	uint8_t bSWCellUCT:1;				//cell under temperatrue when charging from mcu
	uint8_t bSWMOSOT:1;					//MOSFET over temperatrue from mcu
	uint8_t bSWOCT:1;						//cell over temperatrue when charging from mcu
	uint8_t bSWCellOV:1;				//cell over voltage from mcu
	uint8_t bSWCellUV:1;				//cell under voltage from mcu
	//protection code1 low byte
	uint8_t bSWCellODT:1;				//cell over temperature when discharging from mcu
	uint8_t bsWSocLow:1;				//The SOC of the battery cell is too low
	uint8_t bHWOC:1;						//over current from afe
	uint8_t bHWCellOV:1;				//cell over voltage from afe
	uint8_t bHWCellUV:1;				//cell under voltage from afe
	uint8_t bSWPackUV:1;				//pack under voltage from mcu
	uint8_t bSWPackOV:1;				//pack over voltage from mcu
	uint8_t bHWSC:1;						//short circuit from afe
	//protection code2 high byte
	uint8_t bTempSersorErr:1;		//temperature sensor error
	uint8_t bReserve4:1;				//reserve
	uint8_t bDMOSOn:1;					//discharge MOSFET is on
	uint8_t bCMOSOn:1;					//charge MOSFET is on
	uint8_t bHMOSOn:1;					//heater MOSFET is on
	uint8_t bVoltSensor:1;			//voltage sensor error
	uint8_t bReserve5:2;				//reserve
	//protection code2 low byte
	uint8_t  bReserve2:6;				//reserve
	uint8_t  bBAL:1;						//balance active
	uint8_t  bReserve3:1;				//reserve
	uint8_t  ucSoc;							//unit: %
	int8_t   cCurHB;						//current high byte, unit: 0.1A
	uint8_t  ucCurLB;						//current low byte, unit: 0.1A
	uint16_t ausCell[CFG_CELL_NUM];	//cell 0~n voltage high byte & low byte, unit:mV
	int16_t  asTemp[4];					//MOSFET temperature high byte, unit: ¡æ
	uint8_t	 ausSN[8];
} ECO_LOG_S;

extern ECO_LOG_S g_stEcoLog;

typedef struct {
	uint8_t ucCDStat;							//charge & discharge status
	uint16_t bSWOCC:1;						//over current when charging from mcu
	uint16_t bSWODC:1;						//over current when dischaging from mcu
	uint16_t bSWCellUDT:1;				//cell under temperatrue when discharging from mcu
	uint16_t bSWCellUCT:1;				//cell under temperatrue when charging from mcu
	uint16_t bSWMOSOT:1;					//MOSFET over temperatrue from mcu
	uint16_t bSWOCT:1;						//cell over temperatrue when charging from mcu
	uint16_t bSWCellOV:1;					//cell over voltage from mcu
	uint16_t bSWCellUV:1;					//cell under voltage from mcu
	
	uint16_t bSWCellODT:1;				//cell over temperature when discharging from mcu
	uint16_t bReserve1:1;					//reserve
	uint16_t bHWOC:1;							//over current from afe
	uint16_t bHWCellOV:1;					//cell over voltage from afe
	uint16_t bHWCellUV:1;					//cell under voltage from afe
	uint16_t bSWPackUV:1;					//pack under voltage from mcu
	uint16_t bSWPackOV:1;					//pack over voltage from mcu
	uint16_t bHWSC:1;							//short circuit from afe
	
	uint16_t bTempSersorErr:1;		//temperature sensor error
	uint16_t bVoltSensor:1;				//voltage sensor error
	uint16_t bDMOSOn:1;						//discharge MOSFET is on
	uint16_t bCMOSOn:1;						//charge MOSFET is on
	uint16_t bReserve5:4;					//reserve
	
	uint16_t bReserve2:5;					//reserve
	uint16_t MosAnomaly:1; 				//Mos Anomaly
	uint16_t bBAL:1;							//balance active
	uint16_t bHMOSOn:1;						//heating MOSFET is on
	uint8_t ucSoc;								//Soc, unit: %
	uint8_t ucPackRTVoltHB;				//pack realtime voltage high byte, unit: 0.1V
	uint8_t ucPackRTVoltLB;				//pack realtime voltage low byte, unit: 0.1V
	int8_t  cPackRTCurHB;				//pack realtime current high byte, unit: 0.1A
	uint8_t  ucPackRTCurLB;				//pack realtime current low byte, unit: 0.1A
	int8_t  cPackMaxCurHB;				//pack max. current high byte, unit: 0.1A
	uint8_t  ucPackMaxCurLB;				//pack max. current low byte, unit: 0.1A
	uint8_t ucCellMaxVoltHB;			//cell max. voltage high byte, unit: mV
	uint8_t ucCellMaxVoltLB;			//cell max. voltage low byte, unit: mV
	uint8_t ucCellMinVoltHB;			//cell min. voltage high byte, unit: mV
	uint8_t ucCellMinVoltLB;			//cell min. voltage low byte, unit: mV
	int8_t cCellMaxTempHB;			//cell max. temperature high byte, unit: ¡æ
	int8_t cCellMaxTempLB;			//cell max. temperature low byte, unit: ¡æ
	int8_t cCellMinTempHB;			//cell min. temperature high byte, unit: ¡æ
	int8_t cCellMinTempLB;			//cell min. temperature low byte, unit: ¡æ
	int8_t cMOSMaxTempHB;				//MOSFET max. temperature high byte, unit: ¡æ
	int8_t cMOSMaxTempLB;				//MOSFET max. temperature low byte, unit: ¡æ
} ECO_CD_S;

typedef enum {
	eCanIdBmsInf		= 0xE000,			/* BMS Info */
	eCanIdSoc				= 0xE004,			/* Soc */
	eCanIdBMSLmt		= 0xE005,			/* BMS Limits */
	eCanIdCellVolt0 = 0xE006,			/* Cell Voltage 0 */
	eCanIdCellVolt1 = 0xE007,			/* Cell Voltage 1 */
	eCanIdCellVolt2 = 0xE008,			/* Cell Voltage 2 */
	eCanIdCellVolt3 = 0xE009,			/* Cell Voltage 3 */
	eCanIdCellVolt4 = 0xE010,			/* Cell Voltage 4 */
	eCanIdCellVolt5 = 0xE011,			/* Cell Voltage 5 */
	eCanIdCellVoltAvg = 0xE012,		/* Cells Average Voltage */
	eCanIdPackID1    = 0xE013,    /* PACK ID1 */
	eCanIdPackID2    = 0xE014,    /* PACK ID2 */
	eCanIdPackID3    = 0xE015,		/* PACK ID3 */
	eCanIdBMSEqOpenStrNum = 0xF000,    /* BMS equalization opening string number */
	eCanIdFaultInf	= 0xF001,			/* Fault Info. */
	eCanIdChgReq0		= 0x1806E5F4,
	eCanIdChgReq1		= 0x18904010,
	eCanIdChgReq2		= 0x18914010,
	eCanIdChgReq3		= 0x18C828F4,
	eCanIdChgReq4		= 0x18F810F3,
	eCanIdChgReq5		= 0x18F811F3,
	eCanIdChgReq6		= 0x18F812F3,
	eCanIdChgReq7		= 0x18F813F3,
	eCanIdChgReq8		= 0x18F814F3,
	eCanIdChgReq9		= 0x18F815F3,
	eCanIdChgReq10	= 0x18F880F3,
	eCanIdChgReq11	= 0x18FA28F4,
	eCanIdChgReq12	= 0x18FB28F4,
	eCanIdChgReq13	= 0x18FC28F4,
	eCanIdChgReq16	= 0x18FD28F4,
	eCanIdChgReq15	= 0x18FE28F4,
	eCanIdChgReq14	= 0x18FF28F4,
	eCanIdChgReq17	= 0x18FF50E5,  /* The main charger message */
	eCanIdChgReq18	= 0x18FFE5F4,
	eCanIdChgReq19  = 0x773,       /* Second charger message */
  eCanIdChgReq20  = 0x273,       /* The second charger sends a message */   
	eCanIdEx0				= 0x108,
	eCanIdEx1				= 0x110,
	eCanIdEx2				= 0x111,
	eCanIdEx3				= 0x4D4,
	eCanIdEx4				= 0x4D5,
	eCanIdUGMode		= 0x78D,
	eCanIdUGRun			= 0x78E,
	eCanIdUGStop		= 0x78F,
	eCanIdUGRly			= 0x87E,
	eCanIdRexBox    = 0x10086,
	eCanIdParallel0 = 0x200200,       /* The dataframe ID of the host is generated in stand-alone mode or parallel mode */
	eCanIdParallel1 = 0x2200200,		  /* In parallel mode, only slave 1 data frame ID is sent */
	eCanIdParallel2 = 0x4200200,		  /* In parallel mode, only slave 2 dataframe IDs are issued */
	eCanIdParallel3 = 0x6200200,		  /* In parallel mode, only slave 3 data frame IDs are sent */
	eCanIdParallel  = 0x200,					/* In parallel mode, only the integrated data frame ID is generated */
	eCanIdPalCopd		= 0x206500,				/* Parallel communication frames */
	eCanIdEx5    	  = 0x1E1,					/* Info. of battery Status sent from BMS */
	eCanIdEx6 	    = 0x1F5,					/* Info. of battery Status sent from BMS */
	eCanIdEx7  	    = 0x800A6A9,			
	eCanIdEx8       = 0x1000A6A9,
	eCanIdEx9       = 0x1C00A6A9
} ECO_CANID_E;

extern uint16_t g_usMChgerComTick;
extern uint16_t g_usSChgerComTick;
extern uint16_t g_us18F880F3ComTick;
extern uint16_t g_usGbms3ComTick;
extern bool g_bMChgerComAct;
extern bool g_bMChging;
extern bool g_bMChgerAbNor;
extern bool g_bSChgerComAct;
extern bool g_bSChging;
extern bool g_bSetMChgerAct;
extern bool g_bSetSChgerAct;
extern bool g_bs18F880F3Act;
extern bool g_bGbms3;
extern uint8_t g_aucCanSBuf[512];

static uint16_t eco_crc16(const uint8_t* pucBuf, const uint16_t usLen, const uint16_t usDefCRC, const uint16_t usPoly);
static uint32_t eco_crc32(uint8_t* pucBuf, uint32_t uiDataLen, uint32_t uiCRCin, uint32_t uiPoly);
static void eco_can_ug_proc(CAN_RBUF_S stCanRBuf);
extern void eco_can_recv_proc(CAN_RBUF_S stCanRBuf);
extern uint16_t eco_read_reg(uint8_t ucSelfId, uint8_t ucDevAddr, uint16_t usRegAddr);
static void eco_write_reg(uint16_t usRegAddr, uint16_t usData);
static uint16_t eco_get_log(uint8_t ucLogIdx, uint8_t* pucData);
static uint8_t eco_get_cd(uint8_t* pucData);
static void eco_data_proc(uint8_t ucSelfId, uint8_t ucDevAddr, uint8_t ucFunCode, uint16_t usRegAddr, uint8_t* pucData, uint16_t usLen);
extern void eco_can_data_send(uint8_t ucSelfId, uint8_t ucDevAddr, uint8_t ucFunCode, uint8_t ucRegAddr, uint8_t* pucData, uint16_t usLen);
static void eco_uart_ug_proc(uint8_t* pucData, uint16_t usLen);
extern void eco_uart_recv_proc(uint8_t* pucData, uint16_t usLen);
extern void eco_init(void);
extern void eco_proc(void);
extern void eco_refresh_RTV(void);
extern void eco_refresh_CD(void);
extern bool eco_refresh_log(void);

#pragma pack()

#endif
