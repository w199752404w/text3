#ifndef CONFIG_H
#define CONFIG_H

#include <stdbool.h>
#include <stdint.h>
#include <systick.h>

#include "main.h"

#ifdef USER_DEBUG
#define CFG_DEBUG_EN	0		/* 0: non-DEBUG state, 1: DEBUG status */
#else
#define CFG_DEBUG_EN 0
#endif
#define CFG_DEBUG(fmt,arg...)	do{if(CFG_DEBUG_EN){printf(fmt,##arg);printf("[%s][%d]\r\n",__func__,__LINE__);}}while(0)
#define CFG_RETURN_FALSE	do{CFG_DEBUG("Return failed");return false;}while(0)
//#define CFG_RETURN_TRUE do{CFG_DEBUG("Return success");return true;}while(0)
#define CFG_RETURN_TRUE do{return true;}while(0)

#define CFG_CELL_NUM 16		/* The number of cells connected in series for a single BMS */
#define CFG_TMP_NUM 3			/* The number of temperature channels for a single BMS */

#pragma pack(1)

/* 0. OTA configuration */
typedef struct {
	uint32_t uiAddr;	/* Firmware storage address, 0-on-chip, other-off-chip address */
	uint32_t uiLen;		/* Firmware bytes */
	uint8_t ucUpdate;	/* Non-0: The next reboot needs to be upgraded via Flash, and 0 will be cleared after the upgrade */
	uint8_t aucReserve[247];	/* The overall structure size is guaranteed to be 256 bytes */
} CFG_OTA_S;

/* 2. Serial port and CAN */
typedef struct {
	uint32_t uiBaud;
	uint32_t uiWordLen;
	uint32_t uiStopBit;
	uint32_t uiParity;
} CFG_UART_S;	/* sizeof(CFG_UART_S) = 16B */

typedef struct {
	CFG_UART_S stUart;		/* 16B */
	uint32_t uiCanBaud;		/* 4B */
	/* Begin : Modified by h00205922, 2024.09.06 */
	uint8_t ucCanPtcType;				//0-Default(250k), 1-VERSION 'C'(500k), 2-NEW(500k)
	uint8_t aucReserve[235];		/* The overall structure size is guaranteed to be 256 bytes */
	//uint8_t aucReserve[236];		/* The overall structure size is guaranteed to be 256 bytes */
	/* End   : Modified by h00205922, 2024.09.06 */
} CFG_COM_S;

/* 3. Parallel information */

/* 4. Historical service */
typedef struct {
	uint8_t ucDataEn:1;	/* Whether to allow historical data recording, disabling historical data recording will lead to inaccurate SOC estimation after MCU restarts */
	uint8_t ucLogEn:1;	/* Whether or not to allow historical logging */
	uint8_t ucWaveEn:1;	/* Whether fault waveform recording is allowed */
	uint8_t ucReserveBit:5;
	uint16_t usDataPeriod;	/* Unit: s, the interval between historical data recordings */
	uint16_t usLogPeriod;		/* log save periodicity period, unit: s */
	uint8_t aucReserve[251];		/* The overall structure size is guaranteed to be 256 bytes */
} CFG_HIS_S;

/* 5. AFE configuration */
#define CFG_AFE_ROM_BLEN	0x001A

typedef struct {
	uint8_t aucRomByte[CFG_AFE_ROM_BLEN];
	float afTempCali[CFG_TMP_NUM];			/* Temperature Drift Calibration Factor, Unit: °„C */
	float fAvgTempCali;				/* Average temperature drift calibration factor, unit: °„C */
	float fCurCaliA;					/* CUR proportional calibration coefficients */
	float fCurCaliB;					/* CUR drift calibration factor, unit: A */
	float fCDATACaliB;				/* CDATA drift calibration factor, unit: A */
	float afCellVolCali[CFG_CELL_NUM];	/* Cell proportional calibration factor */
	float fCDATACaliA_CHG;		/* Charge CDATA proportional calibration factor */
  float fCDATACaliA_DSG;		/* Discharge CDATA proportional calibration factor */
	float fPackVolCali;				/* Proportional calibration factor for the entire string of battery packs */
	uint8_t aucReserve[126];	/* The overall structure size is guaranteed to be 256 bytes */
} CFG_AFE_S;

/* 6. Native service configuration */
typedef struct {
	uint8_t ucHVer;						/* Hardware version, 0-inactive, 1-A version, 2-B version, Other - reserved */
	uint8_t ucHVerV;					/* Hardware V version */
	uint8_t ucHVerR;					/* Hardware R version */
	uint8_t ucSVerVMajor;			/* Software V major version */
	uint8_t ucSVerVMinor;			/* Software V sub-version */
	uint8_t ucSVerRMajor;			/* Software R major version */
	uint8_t ucSVerRMinor;			/* Software R sub-version */
	uint8_t ucSWPulse;    		/* extern button trigger type, 0- last, or- active trigger */
	uint8_t ucSW2Pulse;    		/* extern button trigger type, 0- last, or- active trigger */
	char aucBleName[20];			/* blue tooth name */
	uint16_t ausHWSN[4];			/* hardware sn */
	uint16_t usCellNum;				/* The number of cells in the PACK */
	uint16_t usSerialCellNum;	/* The number of battery cell strings in the PACK */
	float fNominalU;					/* Voltage platform, i.e. rated voltage*/
	uint16_t usDesignAH;			/* Number of AH of battery pack, design capacity */
	uint16_t usCycle;					/* Unit: times, the number of charge-discharge cycles */
	int32_t iStandbySlpSec;		/* unit: s, In the absence of external communication and charging and discharging, the time threshold for automatic sleep is 0, which means that it does not sleep */
	int32_t iLowPwrSlpSec;		/* unit: s, low power timeout sleep */
	int32_t iSreenSlpSec;			/* uint: s, screen timeout sleep */
	uint16_t usCyclePeriod;		/* Unit: ms, main cycle */
	uint16_t usBALActVolt;		/* balance active voltage, unit: mV */
	uint16_t usBALActDVolt;		/* balance active delta voltage, unit: mV */
	uint16_t usVALReactDVolt;	/* balance reactive delta voltage, unit: mV */
	uint16_t usDbgPara;				/* debug parameter, unused */
	int16_t sChgCurSnrCoeB;		/* charge current senser coefficient B, unit: micor ohms */
	int16_t sDsgCurSnrCoeB;		/* diacharge current senser coefficient B, unit: micor ohms */
	int16_t sVoltSnrCoeB;			/* voltage senser coefficient B, unit: mV */
	uint16_t usCurLmtMode;		/* current limit mode */
	uint16_t ausSocFCaliU[2];	/* Soc full calibration voltage threshold, unit: mV */
	uint16_t ausSocECaliU[3];	/* Soc empty calibration voltage threshold, unit: mV */
	uint16_t usSCRTThr;				/* short circuit recover time threshold */
	uint16_t usCellOVTVThr1;	/* cell over voltage lv.1 trigger value threshold, unit: mV */
	uint16_t usCellOVTTThr1;	/* cell over voltage lv.1 trigger time threshold, unit: ms */
	uint16_t usCellOVRVThr1;	/* cell over voltage lv.1 recover value threshold, unit: mV */
	uint16_t usCellOVRTThr1;	/* cell over voltage lv.1 recover time threshold, unit: ms */
	uint16_t usCellOVTVThr2;	/* cell over voltage lv.2 trigger value threshold, unit: mV */
	uint16_t usCellOVTTThr2;	/* cell over voltage lv.2 trigger time threshold, unit: ms */
	uint16_t usCellOVRVThr2;	/* cell over voltage lv.2 recover value threshold, unit: mV */
	uint16_t usCellOVRTThr2;	/* cell over voltage lv.2 recover time threshold, unit: ms */
	uint16_t usCellUVTVThr1;	/* cell under voltage lv.1 trigger value threshold, unit: mV */
	uint16_t usCellUVTTThr1;	/* cell under voltage lv.1 trigger time threshold, unit: ms */
	uint16_t usCellUVRVThr1;	/* cell under voltage lv.1 recover value threshold, unit: mV */
	uint16_t usCellUVRTThr1;	/* cell under voltage lv.1 recover time threshold, unit: ms */
	uint16_t usCellUVTVThr2;	/* cell under voltage lv.2 trigger value threshold, unit: mV */
	uint16_t usCellUVTTThr2;	/* cell under voltage lv.2 trigger time threshold, unit: ms */
	uint16_t usCellUVRVThr2;	/* cell under voltage lv.2 recover value threshold, unit: mV */
	uint16_t usCellUVRTThr2;	/* cell under voltage lv.2 recover time threshold, unit: ms */
	uint16_t usCellUVTVThr3;	/* cell under voltage lv.3 trigger value threshold, unit: mV */
	uint16_t usCellUVTTThr3;	/* cell under voltage lv.3 trigger time threshold, unit: ms */
	uint16_t usCellUVRVThr3;	/* cell under voltage lv.3 recover value threshold, unit: mV */
	uint16_t usCellUVRTThr3;	/* cell under voltage lv.3 recover time threshold, unit: ms */
	int16_t sCellOCTTVThr1;		/* cell over temperatrue lv.1 trigger value threshold when charging, unit: °Ê */
	uint16_t usCellOCTTTThr1;	/* cell over temperatrue lv.1 trigger time threshold when charging, unit: ms */
	int16_t sCellOCTRVThr1;		/* cell over temperatrue lv.1 recover value threshold when charging, unit: °Ê */
	uint16_t usCellOCTRTThr1;	/* cell over temperatrue lv.1 recover time threshold when charging, unit: ms */
	int16_t sCellODTTVThr1;		/* cell over temperatrue lv.1 trigger value threshold when discharging, unit: °Ê */
	uint16_t usCellODTTTThr1;	/* cell over temperatrue lv.1 trigger time threshold when discharging, unit: ms */
	int16_t sCellODTRVThr1;		/* cell over temperatrue lv.1 recover value threshold when discharging, unit: °Ê */
	uint16_t usCellODTRTThr1;	/* cell over temperatrue lv.1 recover time threshold when discharging, unit: ms */
	int16_t sCellUCTTVThr1;		/* cell under temperatrue lv.1 trigger value threshold when charging, unit: °Ê */
	uint16_t usCellUCTTTThr1;	/* cell under temperatrue lv.1 trigger time threshold when charging, unit: ms */
	int16_t sCellUCTRVThr1;		/* cell under temperatrue lv.1 recover value threshold when charging, unit: °Ê */
	uint16_t usCellUCTRTThr1;	/* cell under temperatrue lv.1 recover time threshold when charging, unit: ms */
	int16_t sCellUDTTVThr1;		/* cell under temperatrue lv.1 trigger value threshold when discharging, unit: °Ê */
	uint16_t usCellUDTTTThr1;	/* cell under temperatrue lv.1 trigger time threshold when discharging, unit: ms */
	int16_t sCellUDTRVThr1;		/* cell under temperatrue lv.1 recover value threshold when discharging, unit: °Ê */
	uint16_t usCellUDTRTThr1;	/* cell under temperatrue lv.1 recover time threshold when discharging, unit: ms */
	int16_t sHeaterUTTVThr;		/* heater temperature trigger value threshold, unit: °Ê */
	uint16_t usHeaterUTTTThr;	/* heater temperature trigger time threshold, unit: ms */
	int16_t sHeaterUTRVThr;		/* heater temperature recover value threshold, unit: °Ê */
	uint16_t usHeaterUTRTThr;	/* heater temperature recover time threshold, unit: ms */
	uint16_t usPackOVTVThr1;	/* pack over voltage lv.1 trigger value threshold, unit: 0.1V */
	uint16_t usPackOVTTThr1;	/* pack over voltage lv.1 trigger time threshold, unit: ms */
	uint16_t usPackOVRVThr1;	/* pack over voltage lv.1 recover value threshold, unit: 0.1V */
	uint16_t usPackOVRTThr1;	/* pack over voltage lv.1 recover time threshold, unit: ms */
	uint16_t usPackOVTVThr2;	/* pack over voltage lv.2 trigger value threshold, unit: 0.1V */
	uint16_t usPackOVTTThr2;	/* pack over voltage lv.2 trigger time threshold, unit: ms */
	uint16_t usPackOVRVThr2;	/* pack over voltage lv.2 recover value threshold, unit: 0.1V */
	uint16_t usPackOVRTThr2;	/* pack over voltage lv.2 recover time threshold, unit: ms */
	uint16_t usPackUVTVThr1;	/* pack under voltage lv.1 trigger value threshold, unit: 0.1V */
	uint16_t usPackUVTTThr1;	/* pack under voltage lv.1 trigger time threshold, unit: ms */
	uint16_t usPackUVRVThr1;	/* pack under voltage lv.1 recover value threshold, unit: 0.1V */
	uint16_t usPackUVRTThr1;	/* pack under voltage lv.1 recover time threshold, unit: ms */
	uint16_t usPackUVTVThr2;	/* pack under voltage lv.2 trigger value threshold, unit: 0.1V */
	uint16_t usPackUVTTThr2;	/* pack under voltage lv.2 trigger time threshold, unit: ms */
	uint16_t usPackUVRVThr2;	/* pack under voltage lv.2 recover value threshold, unit: 0.1V */
	uint16_t usPackUVRTThr2;	/* pack under voltage lv.2 recover time threshold, unit: ms */
	uint16_t usPackUVTVThr3;	/* pack under voltage lv.3 trigger value threshold, unit: 0.1V */
	uint16_t usPackUVTTThr3;	/* pack under voltage lv.3 trigger time threshold, unit: ms */
	uint16_t usPackUVRVThr3;	/* pack under voltage lv.3 recover value threshold, unit: 0.1V */
	uint16_t usPackUVRTThr3;	/* pack under voltage lv.3 recover time threshold, unit: ms */
	int16_t sMOSOTTVThr1;			/* MOSFET over temperatrue lv.1 trigger value threshold, unit: °Ê */
	uint16_t usMOSOTTTThr1;		/* MOSFET over temperatrue lv.1 trigger time threshold, unit: ms */
	int16_t sMOSOTRVThr1;			/* MOSFET over temperatrue lv.1 recover value threshold, unit: °Ê */
	uint16_t usMOSOTRTThr1;		/* MOSFET over temperatrue lv.1 recover time threshold, unit: ms */
	int16_t sMOSOTTVThr2;			/* MOSFET over temperatrue lv.2 trigger value threshold, unit: °Ê */
	uint16_t usMOSOTTTThr2;		/* MOSFET over temperatrue lv.2 trigger time threshold, unit: ms */
	int16_t sMOSOTRVThr2;			/* MOSFET over temperatrue lv.2 recover value threshold, unit: °Ê */
	uint16_t usMOSOTRTThr2;		/* MOSFET over temperatrue lv.2 recover time threshold, unit: ms */
	int16_t sMOSOTTVThr3;			/* MOSFET over temperatrue lv.3 trigger value threshold, unit: °Ê */
	uint16_t usMOSOTTTThr3;		/* MOSFET over temperatrue lv.3 trigger time threshold, unit: ms */
	int16_t sMOSOTRVThr3;			/* MOSFET over temperatrue lv.3 recover value threshold, unit: °Ê */
	uint16_t usMOSOTRTThr3;		/* MOSFET over temperatrue lv.3 recover time threshold, unit: ms */
	uint16_t usOCCTVThr1;			/* over current lv.1 trigger value threshold when charging, unit: A */
	uint16_t usOCCTTThr1;			/* over current lv.1 trigger time threshold when charging, unit: ms */
	uint16_t usOCCRVThr1;			/* over current lv.1 recover value threshold when charging, unit: A */
	uint16_t usOCCRTThr1;			/* over current lv.1 recover time threshold when charging, unit: ms */
	uint16_t usODCTVThr1;			/* over current lv.1 trigger value threshold when discharging, unit: A */
	uint16_t usODCTTThr1;			/* over current lv.1 trigger time threshold when discharging, unit: ms */
	uint16_t usODCRVThr1;			/* over current lv.1 recover value threshold when discharging, unit: A */
	uint16_t usODCRTThr1;			/* over current lv.1 recover time threshold when discharging, unit: ms */
	int16_t  sBalanceOn;      /* Balance configuration, 1 is on, 0 is off */
	uint16_t usBalanceValue;  /* Balance voltage value, unit: mV */
	uint16_t usBalanceDiff;	  /* Balance diff value, unit: mV */
	uint16_t usSocCaliDelay;	/* SOC calibration time delay, unit: h */
	uint16_t usSocCaliCurVal;	/* SOC calibration current value, unit: A */
	char aucBMS_ID[25];			  /* BMS_ID */
	uint16_t ucSVerAgree;			/* Software Version Agreement,0x41 means A protocol, 0x42 represents B protocol, 0x43 represents C protocol, 0x44 represents D protocol, 0x45 represents E protocol, ... */
} CFG_LOCAL_S;

typedef struct {
	/* 0. OTA configuration, 0.25kB */
	CFG_OTA_S stOta;
	/* 1. start after reset, 0.25kB */
	uint16_t usGoRun;		/* 	0- powered off by sleep last round, should be shutdown right now
													1- powered off by software reset last round, should go on
													2- powered off by switch last round, should be shutdown right now
													3- just going to run, should find waken up reason */
	uint8_t ucReserve[254];
	/* 2. UART & CAN configuration, 0.25kB */
	CFG_COM_S stCom;
	/* 4. Historical service configuration, 0.25kB */
	CFG_HIS_S stHis;
	/* 5. AFE configuration, 0.25kB */
	CFG_AFE_S stAfe;
	/* 6. Native service configuration, ??kB */
	CFG_LOCAL_S stLocal;
} CFG_S;

typedef struct{
	char aucBleName_BackUp[20];			/* blue tooth name Back Up*/
	char aucBMS_ID_BackUp[25];			/* BMS_ID Back Up*/
	float fCDATACaliA_CHG_BackUp;		/* Charge CDATA proportional calibration factor */
  float fCDATACaliA_DSG_BackUp;		/* Discharge CDATA proportional calibration factor */
} CFG_BackUp;
typedef enum {
	eCfgBackUpStart = 0x1000,		    /* flash Back Up config page start */
} CfgBack_ADDR_E;

extern CFG_S g_stCfg;
extern CFG_BackUp g_stCfg_BackUp;
extern bool g_SetDefaultFlag;

extern bool cfg_save(void);
extern bool cfg_load(void);
extern bool cfg_set_default(void);
extern bool cfg_init(void);
extern bool cfg_backup_load(void);

#pragma pack()

#endif
