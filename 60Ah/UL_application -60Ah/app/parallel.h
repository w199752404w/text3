#ifndef PARALLEL_H
#define PARALLEL_H

#include <stdbool.h>
#include <stdint.h>

#include "config.h"
#include "main.h"

#ifdef USER_DEBUG
#define PRL_DEBUG_EN	0		/* 0: non-DEBUG state, 1: DEBUG status */
#else
#define PRL_DEBUG_EN 0
#endif

#define PRL_DEBUG(fmt,arg...)	do{if(PRL_DEBUG_EN){printf(fmt,##arg);printf("[%s][%d]\r\n",__func__,__LINE__);}}while(0)
#define PRL_RETURN_FALSE	do{PRL_DEBUG("Return failed");return false;}while(0)
//#define PRL_RETURN_TRUE do{PRL_DEBUG("Return success");return true;}while(0)
#define PRL_RETURN_TRUE do{return true;}while(0)

#pragma pack(1)

#define PRL_MAX_NODE_NUM	4
#define PRL_COM_MSEC	5	//max. parallel communication timeout seconds

typedef struct {
	/* Variables that the host cares about */
	uint8_t ucDevNum;
	/* Variable from the center */
	uint8_t ucSelfId;
	uint8_t ucChgEn;
	uint8_t ucDsgEn;
	uint8_t ucHeatEn;
	uint32_t uiSelfCode;
} PRL_S;

extern PRL_S g_stPrl;
extern uint32_t g_auiIEMI_IDs[PRL_MAX_NODE_NUM];
extern uint16_t g_ausPrlComTick[PRL_MAX_NODE_NUM];	/* The remaining communication terminal descriptions are decremented in the pycan_recv and restored when the communication data is received */
extern uint32_t g_uiSlpTick;
extern bool g_bNeedSleep;
extern uint32_t g_eCanIdParallel;

extern bool prl_host(void);
extern bool prl_client(void);
extern bool prl_single(void);
static bool prl_hproc(void);
static void prl_host_auto_send(void);
static bool prl_cproc(void);
static bool prl_sproc(void);
extern bool prl_init(void);
extern bool prl_proc(void);
extern void prl_cproc_send(void);
extern bool g_eCanIdParallelFlag;

#pragma pack()

#endif
