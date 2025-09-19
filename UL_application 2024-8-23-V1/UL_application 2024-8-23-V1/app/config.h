#ifndef CONFIG_H
#define CONFIG_H

#include <stdbool.h>
#include <stdint.h>
#include <systick.h>

#define CFG_DEBUG_EN	1		/* 0:非DEBUG状态, 1:DEBUG状态 */
#define CFG_DEBUG(fmt,arg...)	do{if(CFG_DEBUG_EN){printf(fmt,##arg);printf("[%s][%d]\r\n",__func__,__LINE__);}}while(0)
#define CFG_RETURN_FALSE	do{CFG_DEBUG("Return failed");return false;}while(0)
//#define CFG_RETURN_TRUE do{CFG_DEBUG("Return success");return true;}while(0)
#define CFG_RETURN_TRUE do{return true;}while(0)

#define CFG_CELL_NUM 4		/* 单个BMS的电芯串联数量 */
#define CFG_TMP_NUM 3			/* 单个BMS的温度通道数量 */

#pragma pack(1)

/* 0. OTA配置 */
typedef struct {
	uint32_t uiAddr;	/* 固件存放地址, 0-片内, 其他-片外地址 */
	uint32_t uiLen;		/* 固件字节数 */
	uint8_t ucUpdate;	/* 非0: 下次重启时需要通过Flash升级, 升级后清0 */
	uint8_t aucReserve[247];	/* 保证整个结构体大小为256字节 */
} CFG_OTA_S;

/* 2. 串口和CAN */
#define CFG_MAX_UART_NUM 3


typedef struct {
	uint32_t uiBaud;
	uint32_t uiWordLen;
	uint32_t uiStopBit;
	uint32_t uiParity;
} CFG_UART_S;	/* sizeof(CFG_UART_S) = 16B */

typedef struct {
	CFG_UART_S astUart[CFG_MAX_UART_NUM];		/* 96B */
	uint32_t auiCanBaud;		/* 8B */
	uint8_t aucReserve[148];		/* 保证整个结构体大小为256字节 */
} CFG_COM_S;

/* 3. 并机信息 */
#define CFG_MAX_PRL_NUM 8

typedef enum {
	ePrlPackId0 = 0,
	ePrlPackId1,
	ePrlPackId2,
	ePrlPackId3,
	ePrlPackId4,
	ePrlPackId5,
	ePrlPackId6,
	ePrlPackId7,
	ePrlPackIdBoard = 0x0E,	/* 广播 */
	ePrlPackIdNull = 0x0F,	/* 无效 */
	ePrlPackIdMax = 0x7FFFFFFF	/* STM32中枚举长度由最大值决定, 该处保证该枚举长度达到4字节 */
} PRL_PACK_E;

typedef struct {
	PRL_PACK_E ePrlSelfIdx;			/* 并机时自身PACK的编号 */
	uint8_t ucPrlPackNum;				/* 并机PACK数量 */
	uint8_t aucReserve[251];		/* 保证整个结构体大小为256字节 */
} CFG_PRL_S;

/* 4. 历史服务 */
typedef struct {
	uint8_t ucDataEn:1;	/* 是否允许历史数据记录, 禁止历史数据记录会导致MCU重启后SOC估算不准 */
	uint8_t ucLogEn:1;	/* 是否允许历史日志记录 */
	uint8_t ucWaveEn:1;	/* 是否允许故障波形记录 */
	uint8_t ucReserveBit:5;
	uint16_t usDataPeriod;	/* 单位: s, 历史数据记录间隔时间 */
    uint16_t usLogPeriod;
	uint8_t aucReserve[251];		/* 保证整个结构体大小为256字节 */
} CFG_HIS_S;

/* 5. AFE配置 */
#define CFG_AFE_ROM_BLEN	0x000E

typedef struct {
	uint8_t aucRomByte[CFG_AFE_ROM_BLEN];
	float afTempCali[CFG_TMP_NUM];			/* 温度漂移校准系数, 单位℃ */
	float fAvgTempCali;				/* 平均温度漂移校准系数, 单位℃ */
	float fCurCaliA;					/* CUR比例校准系数 */
	float fCurCaliB;					/* CUR漂移校准系数, 单位A */
	float fCDATACaliA;				/* CDATA比例校准系数 */
	float fCDATACaliB;				/* CDATA漂移校准系数, 单位A */
	float afCellVolCali[CFG_CELL_NUM];	/* 电芯比例校准系数 */
	float fPackVolCali;				/* 整串电池包比例校准系数 */
	uint8_t aucReserve[130];	/* 保证整个结构体大小为256字节 */
} CFG_AFE_S;

/* 6. 本机服务配置 */
#define CFG_MAX_TUI_ROWS	7
#define CFG_MAX_TUI_COLUMNS	6

typedef struct {
	uint16_t usChgRate;		/* 最大充电倍率, 单位: 0.01C */
	uint16_t usChgCellOV;	/* 最大充电单体电压, 单位: mV */
	uint16_t usChgPackOV;	/* 最大充电PACK电压, 单位: 10mV */
	uint16_t usDsgRate;		/* 最大放电倍率, 单位: 0.01C */
	uint16_t usDsgCellUV;	/* 最大放电单体电压, 单位: mV */
	uint16_t usDsgPackUV;	/* 最大放电PACK电压, 单位: 10mV */
} CFG_TUI_PARA_S;

typedef struct {
	int16_t asTriggerT[CFG_MAX_TUI_ROWS];	/* 触发温度, 单位: ℃ */
	int16_t asRecovery[CFG_MAX_TUI_ROWS];	/* 恢复温度, 单位: ℃ */
	uint16_t ausHV[CFG_MAX_TUI_COLUMNS];		/* 单体电压高限, 单位: mV */
	uint16_t ausLV[CFG_MAX_TUI_COLUMNS];		/* 单体电压低限, 单位: mV */
	CFG_TUI_PARA_S aastTUIPara[CFG_MAX_TUI_ROWS][CFG_MAX_TUI_COLUMNS];
} CFG_TUI_S;

typedef struct {
	float fValThr;
	uint8_t ucTmThr;
	float fRetValThr;
	uint8_t ucRetTmThr;
} THR_S;

typedef struct {
	uint8_t ucHVer;						/* 硬件版本, 0-无效, 1-A版本, 2-B版本, 其他-预留 */
	uint8_t ucHVerV;					/* 硬件V版本 */
	uint8_t ucHVerR;					/* 硬件R版本 */
	uint8_t ucSVerVMajor;			/* 软件V主版本 */
	uint8_t ucSVerVMinor;			/* 软件V子版本 */
	uint8_t ucSVerRMajor;			/* 软件R主版本 */
	uint8_t ucSVerRMinor;			/* 软件R子版本 */
	uint8_t ucReserve;    		/* 为了让后面的变量做到4字节对齐 */
	char acSN[32];						/* BMS硬件序列号 */
	char acBmsMnfInfo[20];		/* BMS制造信息 */
	char acPackMnfInfo[20];		/* PACK制造信息 */
	char acMnfName[16];				/* 制造商名称 */
	uint16_t usCellNum;				/* PACK内电芯个数 */
	uint16_t usSerialCellNum;	/* PACK内电芯串数 */
	float fNominalU;					/* 电压平台, 即额定电压 */
	uint16_t usDesignAH;			/* 电池组AH数, 设计容量 */
	uint16_t usCycle;					/* 单位: 次, 充放电循环次数 */
	THR_S stCellOVAlm;
	THR_S stCellOVPtct;
	THR_S stCellUVAlm;
	THR_S stCellUVPtct;
	THR_S stPackOVAlm;
	THR_S stPackOVPtct;
	THR_S stPackUVAlm;
	THR_S stPackUVPtct;
	THR_S stChgOTAlm;
	THR_S stChgOTPtct;
	THR_S stChgUTAlm;
	THR_S stChgUTPtct;
	THR_S stDsgOTAlm;
	THR_S stDsgOTPtct;
	THR_S stDsgUTAlm;
	THR_S stDsgUTPtct;
	THR_S stOCCAlm;
	THR_S stOCCPtct;
	THR_S stODCAlm;
	THR_S stODCPtct;
	THR_S stEnvOTAlm;
	THR_S stEnvOTPtct;
	THR_S stEnvUTAlm;
	THR_S stEnvUTPtct;
	THR_S stMosOTAlm;
	THR_S stMosOTPtct;
	uint8_t ucLedAlmEn;				/* 使能LED告警功能 */
	uint8_t ucCurLimitLv;			/* 选择限流高档位, 0-高档位, 1-低档位 */
	uint8_t ucCurLimitEn;			/* 使能限流功能 */
	uint8_t ucBeepAlmEn;			/* 使能蜂鸣器告警 */
	uint16_t usFCHGMaxCellU;	/* 单位: mV, 强充请求允许的电压最高的电芯的电压, 因强充使用大电压, 如原有电芯电压高, 可能会充坏, 所以增加该限制 */
	uint16_t usFCHGMinCellU;	/* 单位: mV, 强充请求触发的最低电芯电压条件 */
	uint16_t usBCHGDeltaCellU;/* 单位: mV, 均充请求触发的最小电芯压差条件 */
	uint16_t usChgAdjSVolt;		/* 单位: mV, 充电校准电压, 达到该阈值则开始校准SOC */
	uint16_t usChgAdjEVolt;		/* 单位: mV, 充电校准电压, 达到该阈值则校准为100% SOC */
	uint16_t usDsgAdjSVolt;		/* 单位: mV, 放电校准电压, 达到该阈值则开始校准SOC */
	uint16_t usDsgAdjEVolt;		/* 单位: mV, 放电校准电压, 达到该阈值则校准为0% SOC */
	uint32_t uiSleepSec;			/* 单位: s, 没有外部通信及充放电情况下, 自动休眠的时间阈值, 为0时代表不休眠 */
	uint16_t usCyclePeriod;		/* 单位: ms, 主循环周期 */
	CFG_TUI_S stTUI;					/* 充放电性能修正参数 */
	uint8_t ucPDSGDly;				/* 单位: s, 预放电投入时间, 0表示不使用预放电回路策略 */
	uint16_t usBCHGMaxCellU;	/* 单位: mV, 被动均衡最大电压 */
	uint8_t aucReserve[69];		/* 保证整个结构体大小为256 * 3字节 */
} CFG_LOCAL_S;

typedef struct {
	/* 0. OTA配置, 0.25kB */
	CFG_OTA_S stOta;
	/* 2. UART & CAN配置, 0.25kB */
	CFG_COM_S stCom;
	/* 3. 并机服务配置, 0.25kB */
	CFG_PRL_S stPrl;
	/* 4. 历史服务配置, 0.25kB */
	CFG_HIS_S stHis;
	/* 5. AFE配置, 0.25kB */
	CFG_AFE_S stAfe;
	/* 6. 本机服务配置, 3kB */
	CFG_LOCAL_S stLocal;
} CFG_S;

extern CFG_S g_stCfg;

typedef enum {
	/* 0. 控制命令 */
	eCfgCmdStart = 0x1400,
	eCfgCmdSetDefault = eCfgCmdStart,		/* 设置所有配置为默认值, 如果还有配置相关的命令, 往后加 */
	eCfgCmdSave,
	eCfgCmdTiming,
    eCfgCmdReset,
	eCfgCmdEnd = eCfgCmdStart + 0x00FF,
	/* 1. 改写本地数据 */
	eCfgDataStart,
	eCfgDataEnd = eCfgDataStart + sizeof(CFG_S) / sizeof(float) - 1,
	/* 2. 预留 */
	eCfgRsvStart,
	eCfgRsvReserve = eCfgRsvStart + 0x00FF,
	eCfgRsvEnd = eCfgRsvReserve,
	
	eCfgEnd = eCfgRsvEnd
} CFG_E;

extern bool cfg_ota_save(void) ;
extern bool cfg_prl_save(void);
extern bool cfg_save(void);
extern bool cfg_load(void);
extern bool cfg_set_default(void);
//extern bool cfg_get_ai(uint16_t usAddr, float* pfVal);
//extern bool cfg_set_ao(uint16_t usAddr, uint8_t ucCnt, float* pfVal);
//extern bool cfg_get_di(uint16_t usAddr, bool* pbVal);
//extern bool cfg_set_do(uint16_t usAddr, bool bVal);
extern bool cfg_init(void);
extern bool cfg_proc(void);

#pragma pack()

#endif
