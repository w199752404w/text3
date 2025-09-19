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
    
    
    uint16_t usPackVol;	/* PACK电压, 单位: 0.1V */
	uint16_t usPackCur;	/* PACK电流, 单位: 0.1A, 偏移量: -3000A */
	uint16_t usPackEvnT;		/* PACK环境温度, 单位: 0.1℃, 偏移量: -100℃ */
	uint8_t ucPackSoc;	/* Soc, 单位: % */
	uint8_t ucPackSoh;	/* Soh, 单位: % */
	uint16_t usPackOV;	/* PACK充电截止电压, 单位: 0.1V */
	uint16_t usPackUV;	/* PACK放点截止电压, 单位: 0.1V */
	uint16_t usPackOCC;		/* 最大充电电流, 单位: 0.1A, 偏移量: -3000A */
	uint16_t usPackODC;		/* 最大放电电流, 单位: 0.1A, 偏移量: -3000A */
	uint16_t usCellMaxVol;	/* 最高电芯电压, 单位: 0.001V */
	uint16_t usCellMinVol;	/* 最低电芯电压, 单位: 0.001V */
	uint16_t usCellMaxVolId;	/* 最高电压电芯ID */
	uint16_t usCellMinVolId;	/* 最低电压电芯ID */
	uint16_t usCellMaxT;	/* 最高电芯温度, 单位: 0.1℃, 偏移量: -100℃ */
	uint16_t usCellMinT;	/* 最低电芯温度, 单位: 0.1℃, 偏移量: -100℃ */
	uint16_t usCellMaxTId;	/* 最高温度电芯ID */
	uint16_t usCellMinTId;	/* 最低温度电芯ID */
	uint8_t ucBaseStat;		/* 基本状态, bit0~2: 0休眠, 1充电, 2放电, 3搁置, bit3: 请求强充, bit4: 请求均充 */
	uint16_t usCycle;	/* 循环周期 */
	uint8_t ucErrCode;	/* 错误码, bit0~7:	电压传感器故障, 温度传感器故障, 内部通信故障, 输入过压故障,
												输入反接故障, 继电器检测故障, 电池损坏故障, 其他故障（具体见故障扩展字段） */
	uint16_t usAlmCode;	/* 告警码, bit0~11: 单体低压, 单体高压, PACK组低压, PACK组高压, 
												充电低温, 充电高温, 放电低温, 放电高温,
												充电过流, 放电过流, PACK低压, PACK高压 */
	uint16_t usPtctCode;	/* 保护码, bit0~11: 单体低压, 单体高压, PACK低压, PACK高压,
												充电低温, 充电高温, 放电低温, 放电高温,
												充电过流, 放电过流, PACK低压, PACK高压 */
	uint8_t ucChgEn;	/* 充电禁止, 0xAA表示禁止, 其他值无效 */
	uint8_t ucDsgEn;	/* 放电禁止, 0xAA表示禁止, 其他值无效 */
	uint8_t ucErrCodeEx;	/* 扩展故障码, bit0~5: 关机电路异常, BMIC异常, 内部总线异常, 开机自检异常, 安全功能异常 */
	uint16_t usCellVol[CFG_CELL_NUM];	/* 电芯电压, 单位: 0.001 */
	uint16_t usCellSoc[CFG_CELL_NUM];	/* 电芯Soc, 单位: % */
	uint16_t usCellSoh[CFG_CELL_NUM];	/* 电芯Soh, 单位: % */
	uint16_t usCellT[CFG_TMP_NUM];	/* 电芯温度, 单位: 0.1℃, 偏移量: -100℃ */
	uint16_t usMosT;	/* MOS温度, 单位: 0.1℃, 偏移量: -100℃ */
    
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


/* 日志结构体, 固定为64个字节 */
typedef struct {
	time_t dt;							/* 发生时间 */
	HIS_LV_E eLv;						/* 事件级别 */
	HIS_SRC_E eSrc;					/* 事件来源 */
	HIS_OBJ_E eObj;					/* 动作对象 */
	HIS_ACT_E eAct;					/* 动作性质 */
	float fVal;							/* 动作量 */
	uint8_t aucReserve[40];	/* 可以用于存放一些特殊动作对象和性质对应的数据 */
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
