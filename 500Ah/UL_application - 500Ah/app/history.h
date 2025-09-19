#ifndef HISTORY_H
#define HISTORY_H

#include <stdbool.h>
#include <stdint.h>
#include <time.h>

#include "eco.h"
#include "main.h"
#include "bsp_wdg.h"
#include "bsp_gpio.h"

#ifdef USER_DEBUG
#define HIS_DEBUG_EN	0		/* 0: non-DEBUG state, 1: DEBUG status */
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
	ECO_RTV_S stRtv;
	uint16_t ausCellRealAh[CFG_CELL_NUM];	//cell real Ah
	uint16_t aucCellSoc[CFG_CELL_NUM];	//cell soc
	uint16_t state;
} HIS_DATA_S;

/* The log structure is fixed at 64 bytes */
typedef struct {
	time_t dt;							/* Time of occurrence */
	ECO_LOG_S stEcoLog;
} HIS_LOG_S;

#define WAVE_PEICES	64

typedef struct {
	time_t dt;							/* Time of occurrence */
	ECO_RTV_S astEcoRtv[WAVE_PEICES];
	uint16_t usIdx;
} HIS_WAVE_S;

extern uint16_t g_usHDataIdx;
extern uint16_t g_usHLogIdx;

extern bool his_data_read(uint16_t usIdx, HIS_DATA_S* pstData);
extern bool his_data_write(void);
extern bool his_data_clear(void);
extern bool his_log_read(uint16_t usIdx, HIS_LOG_S* pstLog);
extern bool his_log_write(void);
extern bool his_log_clear(void);
extern bool his_wave_read(uint8_t ucIdx, uint8_t ucOffset, uint8_t* pucBuf);
extern bool his_wave_write(void);
//extern bool his_wave_clear(void);
static bool his_wave_proc(void);
extern bool his_clear(void);
extern bool his_init(void);
extern bool his_proc(void);

#pragma pack()

#endif
