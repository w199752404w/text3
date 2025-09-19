#ifndef HISTORY_H
#define HISTORY_H

#include <stdbool.h>
#include <stdint.h>
#include <time.h>


#include "main.h"
#include "bsp_wdg.h"
#include "bsp_gpio.h"
#include "config.h"
#ifdef USER_DEBUG
#define HIS_DEBUG_EN	1		/* 0: non-DEBUG state, 1: DEBUG status */
#else
#define HIS_DEBUG_EN 0 
#endif
#define HIS_DEBUG(fmt,arg...)	do{if(HIS_DEBUG_EN){printf(fmt,##arg);printf("[%s][%d]\r\n",__func__,__LINE__);}}while(0)
#define HIS_RETURN_FALSE	do{HIS_DEBUG("Return failed");return false;}while(0)
//#define HIS_RETURN_TRUE do{HIS_DEBUG("Return success");return true;}while(0)
#define HIS_RETURN_TRUE do{return true;}while(0)

#pragma pack(1)

/* Encapsulate alarm logs and historical data, and wait for the flash interface to be determined before improvement */

#define HIS_CFG_PLEN	0x0400	/* The configuration file occupies 512 pages with a total of 0.25MB of space */
#define HIS_DATA_PLEN	0x2800	/* Historical data occupies 10,240 pages, a total of 2.5MB, 5k historical data fragments can be written, recorded every 1 minute, and can be recorded for about 3.56 days */
#define HIS_LOG_PLEN	0x0800	/* The historical log occupies 2048 pages, a total of 0.5MB, each page is divided into 4 segments, each segment can write one log, a total of 8192 logs can be written */
#define HIS_WAVE_PLEN	0x0400	/* The recording file occupies 1024 pages, a total of 0.25MB space, and each recording file occupies a fixed occupation of 32kB, which can save a total of 8 recording files */
#define HIS_OTA_PLEN	0x0800	/* OTA firmware occupies 2048 pages, a total of 0.5MB space, divided into two copies, each containing one firmware of no more than 256kB */

typedef enum {
	eHisCfgPStart = 0x0000,		/* flash config page start */
	eHisDataPStart = eHisCfgPStart + HIS_CFG_PLEN,
	eHisLogPStart = eHisDataPStart + HIS_DATA_PLEN,
	eHisWavePStart = eHisLogPStart + HIS_LOG_PLEN,
	eHisOTAPStart = eHisWavePStart + HIS_WAVE_PLEN
} HIS_PADDR_E;

/* Historical data fragmentation structure, Flash 256 bytes per page, each page stores a piece of historical data */
typedef struct {
	time_t dt;
	//ECO_RTV_S stRtv;
	//uint16_t ausCellRealAh[CFG_CELL_NUM];	//cell real Ah
	//uint16_t aucCellSoc[CFG_CELL_NUM];	//cell soc
	//uint16_t state;
    
    
    uint16_t usPackVol;	/* PACK��ѹ, ��λ: 0.1V */
	uint16_t usPackCur;	/* PACK����, ��λ: 0.1A, ƫ����: -3000A */
	uint16_t usPackEvnT;		/* PACK�����¶�, ��λ: 0.1��, ƫ����: -100�� */
	uint8_t ucPackSoc;	/* Soc, ��λ: % */
	uint8_t ucPackSoh;	/* Soh, ��λ: % */
	uint16_t usPackOV;	/* PACK����ֹ��ѹ, ��λ: 0.1V */
	uint16_t usPackUV;	/* PACK�ŵ��ֹ��ѹ, ��λ: 0.1V */
	uint16_t usPackOCC;		/* ��������, ��λ: 0.1A, ƫ����: -3000A */
	uint16_t usPackODC;		/* ���ŵ����, ��λ: 0.1A, ƫ����: -3000A */
	uint16_t usCellMaxVol;	/* ��ߵ�о��ѹ, ��λ: 0.001V */
	uint16_t usCellMinVol;	/* ��͵�о��ѹ, ��λ: 0.001V */
	uint16_t usCellMaxVolId;	/* ��ߵ�ѹ��оID */
	uint16_t usCellMinVolId;	/* ��͵�ѹ��оID */
	uint16_t usCellMaxT;	/* ��ߵ�о�¶�, ��λ: 0.1��, ƫ����: -100�� */
	uint16_t usCellMinT;	/* ��͵�о�¶�, ��λ: 0.1��, ƫ����: -100�� */
	uint16_t usCellMaxTId;	/* ����¶ȵ�оID */
	uint16_t usCellMinTId;	/* ����¶ȵ�оID */
	uint8_t ucBaseStat;		/* ����״̬, bit0~2: 0����, 1���, 2�ŵ�, 3����, bit3: ����ǿ��, bit4: ������� */
	uint16_t usCycle;	/* ѭ������ */
	uint8_t ucErrCode;	/* ������, bit0~7:	��ѹ����������, �¶ȴ���������, �ڲ�ͨ�Ź���, �����ѹ����,
												���뷴�ӹ���, �̵���������, ����𻵹���, �������ϣ������������չ�ֶΣ� */
	uint16_t usAlmCode;	/* �澯��, bit0~11: �����ѹ, �����ѹ, PACK���ѹ, PACK���ѹ, 
												������, ������, �ŵ����, �ŵ����,
												������, �ŵ����, PACK��ѹ, PACK��ѹ */
	uint16_t usPtctCode;	/* ������, bit0~11: �����ѹ, �����ѹ, PACK��ѹ, PACK��ѹ,
												������, ������, �ŵ����, �ŵ����,
												������, �ŵ����, PACK��ѹ, PACK��ѹ */
	uint8_t ucChgEn;	/* ����ֹ, 0xAA��ʾ��ֹ, ����ֵ��Ч */
	uint8_t ucDsgEn;	/* �ŵ��ֹ, 0xAA��ʾ��ֹ, ����ֵ��Ч */
	uint8_t ucErrCodeEx;	/* ��չ������, bit0~5: �ػ���·�쳣, BMIC�쳣, �ڲ������쳣, �����Լ��쳣, ��ȫ�����쳣 */
	uint16_t usCellVol[CFG_CELL_NUM];	/* ��о��ѹ, ��λ: 0.001 */
	uint16_t usCellSoc[CFG_CELL_NUM];	/* ��оSoc, ��λ: % */
	uint16_t usCellSoh[CFG_CELL_NUM];	/* ��оSoh, ��λ: % */
	uint16_t usCellT[CFG_TMP_NUM];	/* ��о�¶�, ��λ: 0.1��, ƫ����: -100�� */
	uint16_t usMosT;	/* MOS�¶�, ��λ: 0.1��, ƫ����: -100�� */
    
} HIS_DATA_S;

typedef enum {
	eHisLvStart = 0,
	eHisLvNormal = eHisLvStart,
	eHisLvAlarm,
	eHisLvPtct,
	eHisLvFault,
	eHisLvNum,
	eHisLvMax = 0x7FFFFFFF
} HIS_LV_E;

typedef enum {
	eHisSrcStart = 0,
	eHisSrcApp = eHisSrcStart,
	eHisSrcAfe,
	eHisSrcCfg,
	eHisSrcHis,
	eHisSrcPrl,
	eHisSrcPtc,
	eHisSrcMBS,
	eHisSrcPYLON,
	eHisSrcMBM,
	eHisSrcNum,
	eHisSrcMax = 0x7FFFFFFF
} HIS_SRC_E;

typedef enum {
	eHisObjStart = 0,
	eHisObjMcu = eHisObjStart,
	eHisObjCellOV,
	eHisObjCellUV,
	eHisObjPackOV,
	eHisObjPackUV,
	eHisObjOCT,
	eHisObjUCT,
	eHisObjODT,
	eHisObjUDT,
	eHisObjEnvOT,
	eHisObjEnvUT,
	eHisObjOCC,
	eHisObjODC,
	eHisObjMosOT,
	eHisObjSC,
	eHisObjArrOV,
	eHisObjArrUV,
	eHisObjVoltSensorErr,
	eHisObjTempSensorErr,
	eHisObjInterComErr,
	eHisObjInputOV,
	eHisObjMax = 0x7FFFFFFF
} HIS_OBJ_E;

typedef enum {
	eHisActStart = 0,
	eHisActInit = eHisActStart,
	eHisActAct,
	eHisActReact,
	eHisActMax = 0x7FFFFFFF
} HIS_ACT_E;


/* ��־�ṹ��, �̶�Ϊ64���ֽ� */
typedef struct {
	time_t dt;							/* ����ʱ�� */
	HIS_LV_E eLv;						/* �¼����� */
	HIS_SRC_E eSrc;					/* �¼���Դ */
	HIS_OBJ_E eObj;					/* �������� */
	HIS_ACT_E eAct;					/* �������� */
	float fVal;							/* ������ */
	uint8_t aucReserve[40];	/* �������ڴ��һЩ���⶯����������ʶ�Ӧ������ */
} HIS_LOG_S;


//typedef enum {
//	eHisLvStart = 0,
//	eHisLvNormal = eHisLvStart,
//	eHisLvAlarm,
//	eHisLvPtct,
//	eHisLvFault,
//	eHisLvNum,
//	eHisLvMax = 0x7FFFFFFF
//} HIS_LV_E;




extern uint16_t g_usHDataIdx;
extern uint16_t g_usHLogIdx;

extern bool his_data_read(uint16_t usIdx, HIS_DATA_S* pstData);
extern bool his_data_write(void);
extern bool his_data_clear(void);
extern bool his_log_read(uint16_t usIdx, HIS_LOG_S* pstLog);
extern bool his_log_write(HIS_LOG_S* pstLog);
extern bool his_log_clear(void);
//extern bool his_wave_read(uint8_t ucIdx, uint8_t ucOffset, uint8_t* pucBuf);
//extern bool his_wave_write(void);
//extern bool his_wave_clear(void);
static bool his_wave_proc(void);
extern bool his_clear(void);




extern bool his_init(void);
extern bool his_proc(void);





#pragma pack()

#endif
