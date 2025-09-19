#ifndef CONFIG_H
#define CONFIG_H

#include <stdbool.h>
#include <stdint.h>
#include <systick.h>

#define CFG_DEBUG_EN	1		/* 0:��DEBUG״̬, 1:DEBUG״̬ */
#define CFG_DEBUG(fmt,arg...)	do{if(CFG_DEBUG_EN){printf(fmt,##arg);printf("[%s][%d]\r\n",__func__,__LINE__);}}while(0)
#define CFG_RETURN_FALSE	do{CFG_DEBUG("Return failed");return false;}while(0)
//#define CFG_RETURN_TRUE do{CFG_DEBUG("Return success");return true;}while(0)
#define CFG_RETURN_TRUE do{return true;}while(0)

#define CFG_CELL_NUM 4		/* ����BMS�ĵ�о�������� */
#define CFG_TMP_NUM 3			/* ����BMS���¶�ͨ������ */

#pragma pack(1)

/* 0. OTA���� */
typedef struct {
	uint32_t uiAddr;	/* �̼���ŵ�ַ, 0-Ƭ��, ����-Ƭ���ַ */
	uint32_t uiLen;		/* �̼��ֽ��� */
	uint8_t ucUpdate;	/* ��0: �´�����ʱ��Ҫͨ��Flash����, ��������0 */
	uint8_t aucReserve[247];	/* ��֤�����ṹ���СΪ256�ֽ� */
} CFG_OTA_S;

/* 2. ���ں�CAN */
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
	uint8_t aucReserve[148];		/* ��֤�����ṹ���СΪ256�ֽ� */
} CFG_COM_S;

/* 3. ������Ϣ */
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
	ePrlPackIdBoard = 0x0E,	/* �㲥 */
	ePrlPackIdNull = 0x0F,	/* ��Ч */
	ePrlPackIdMax = 0x7FFFFFFF	/* STM32��ö�ٳ��������ֵ����, �ô���֤��ö�ٳ��ȴﵽ4�ֽ� */
} PRL_PACK_E;

typedef struct {
	PRL_PACK_E ePrlSelfIdx;			/* ����ʱ����PACK�ı�� */
	uint8_t ucPrlPackNum;				/* ����PACK���� */
	uint8_t aucReserve[251];		/* ��֤�����ṹ���СΪ256�ֽ� */
} CFG_PRL_S;

/* 4. ��ʷ���� */
typedef struct {
	uint8_t ucDataEn:1;	/* �Ƿ�������ʷ���ݼ�¼, ��ֹ��ʷ���ݼ�¼�ᵼ��MCU������SOC���㲻׼ */
	uint8_t ucLogEn:1;	/* �Ƿ�������ʷ��־��¼ */
	uint8_t ucWaveEn:1;	/* �Ƿ�������ϲ��μ�¼ */
	uint8_t ucReserveBit:5;
	uint16_t usDataPeriod;	/* ��λ: s, ��ʷ���ݼ�¼���ʱ�� */
    uint16_t usLogPeriod;
	uint8_t aucReserve[251];		/* ��֤�����ṹ���СΪ256�ֽ� */
} CFG_HIS_S;

/* 5. AFE���� */
#define CFG_AFE_ROM_BLEN	0x000E

typedef struct {
	uint8_t aucRomByte[CFG_AFE_ROM_BLEN];
	float afTempCali[CFG_TMP_NUM];			/* �¶�Ư��У׼ϵ��, ��λ�� */
	float fAvgTempCali;				/* ƽ���¶�Ư��У׼ϵ��, ��λ�� */
	float fCurCaliA;					/* CUR����У׼ϵ�� */
	float fCurCaliB;					/* CURƯ��У׼ϵ��, ��λA */
	float fCDATACaliA;				/* CDATA����У׼ϵ�� */
	float fCDATACaliB;				/* CDATAƯ��У׼ϵ��, ��λA */
	float afCellVolCali[CFG_CELL_NUM];	/* ��о����У׼ϵ�� */
	float fPackVolCali;				/* ������ذ�����У׼ϵ�� */
	uint8_t aucReserve[130];	/* ��֤�����ṹ���СΪ256�ֽ� */
} CFG_AFE_S;

/* 6. ������������ */
#define CFG_MAX_TUI_ROWS	7
#define CFG_MAX_TUI_COLUMNS	6

typedef struct {
	uint16_t usChgRate;		/* ����籶��, ��λ: 0.01C */
	uint16_t usChgCellOV;	/* ����絥���ѹ, ��λ: mV */
	uint16_t usChgPackOV;	/* �����PACK��ѹ, ��λ: 10mV */
	uint16_t usDsgRate;		/* ���ŵ籶��, ��λ: 0.01C */
	uint16_t usDsgCellUV;	/* ���ŵ絥���ѹ, ��λ: mV */
	uint16_t usDsgPackUV;	/* ���ŵ�PACK��ѹ, ��λ: 10mV */
} CFG_TUI_PARA_S;

typedef struct {
	int16_t asTriggerT[CFG_MAX_TUI_ROWS];	/* �����¶�, ��λ: �� */
	int16_t asRecovery[CFG_MAX_TUI_ROWS];	/* �ָ��¶�, ��λ: �� */
	uint16_t ausHV[CFG_MAX_TUI_COLUMNS];		/* �����ѹ����, ��λ: mV */
	uint16_t ausLV[CFG_MAX_TUI_COLUMNS];		/* �����ѹ����, ��λ: mV */
	CFG_TUI_PARA_S aastTUIPara[CFG_MAX_TUI_ROWS][CFG_MAX_TUI_COLUMNS];
} CFG_TUI_S;

typedef struct {
	float fValThr;
	uint8_t ucTmThr;
	float fRetValThr;
	uint8_t ucRetTmThr;
} THR_S;

typedef struct {
	uint8_t ucHVer;						/* Ӳ���汾, 0-��Ч, 1-A�汾, 2-B�汾, ����-Ԥ�� */
	uint8_t ucHVerV;					/* Ӳ��V�汾 */
	uint8_t ucHVerR;					/* Ӳ��R�汾 */
	uint8_t ucSVerVMajor;			/* ���V���汾 */
	uint8_t ucSVerVMinor;			/* ���V�Ӱ汾 */
	uint8_t ucSVerRMajor;			/* ���R���汾 */
	uint8_t ucSVerRMinor;			/* ���R�Ӱ汾 */
	uint8_t ucReserve;    		/* Ϊ���ú���ı�������4�ֽڶ��� */
	char acSN[32];						/* BMSӲ�����к� */
	char acBmsMnfInfo[20];		/* BMS������Ϣ */
	char acPackMnfInfo[20];		/* PACK������Ϣ */
	char acMnfName[16];				/* ���������� */
	uint16_t usCellNum;				/* PACK�ڵ�о���� */
	uint16_t usSerialCellNum;	/* PACK�ڵ�о���� */
	float fNominalU;					/* ��ѹƽ̨, �����ѹ */
	uint16_t usDesignAH;			/* �����AH��, ������� */
	uint16_t usCycle;					/* ��λ: ��, ��ŵ�ѭ������ */
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
	uint8_t ucLedAlmEn;				/* ʹ��LED�澯���� */
	uint8_t ucCurLimitLv;			/* ѡ�������ߵ�λ, 0-�ߵ�λ, 1-�͵�λ */
	uint8_t ucCurLimitEn;			/* ʹ���������� */
	uint8_t ucBeepAlmEn;			/* ʹ�ܷ������澯 */
	uint16_t usFCHGMaxCellU;	/* ��λ: mV, ǿ����������ĵ�ѹ��ߵĵ�о�ĵ�ѹ, ��ǿ��ʹ�ô��ѹ, ��ԭ�е�о��ѹ��, ���ܻ�仵, �������Ӹ����� */
	uint16_t usFCHGMinCellU;	/* ��λ: mV, ǿ�����󴥷�����͵�о��ѹ���� */
	uint16_t usBCHGDeltaCellU;/* ��λ: mV, �������󴥷�����С��оѹ������ */
	uint16_t usChgAdjSVolt;		/* ��λ: mV, ���У׼��ѹ, �ﵽ����ֵ��ʼУ׼SOC */
	uint16_t usChgAdjEVolt;		/* ��λ: mV, ���У׼��ѹ, �ﵽ����ֵ��У׼Ϊ100% SOC */
	uint16_t usDsgAdjSVolt;		/* ��λ: mV, �ŵ�У׼��ѹ, �ﵽ����ֵ��ʼУ׼SOC */
	uint16_t usDsgAdjEVolt;		/* ��λ: mV, �ŵ�У׼��ѹ, �ﵽ����ֵ��У׼Ϊ0% SOC */
	uint32_t uiSleepSec;			/* ��λ: s, û���ⲿͨ�ż���ŵ������, �Զ����ߵ�ʱ����ֵ, Ϊ0ʱ�������� */
	uint16_t usCyclePeriod;		/* ��λ: ms, ��ѭ������ */
	CFG_TUI_S stTUI;					/* ��ŵ������������� */
	uint8_t ucPDSGDly;				/* ��λ: s, Ԥ�ŵ�Ͷ��ʱ��, 0��ʾ��ʹ��Ԥ�ŵ��·���� */
	uint16_t usBCHGMaxCellU;	/* ��λ: mV, ������������ѹ */
	uint8_t aucReserve[69];		/* ��֤�����ṹ���СΪ256 * 3�ֽ� */
} CFG_LOCAL_S;

typedef struct {
	/* 0. OTA����, 0.25kB */
	CFG_OTA_S stOta;
	/* 2. UART & CAN����, 0.25kB */
	CFG_COM_S stCom;
	/* 3. ������������, 0.25kB */
	CFG_PRL_S stPrl;
	/* 4. ��ʷ��������, 0.25kB */
	CFG_HIS_S stHis;
	/* 5. AFE����, 0.25kB */
	CFG_AFE_S stAfe;
	/* 6. ������������, 3kB */
	CFG_LOCAL_S stLocal;
} CFG_S;

extern CFG_S g_stCfg;

typedef enum {
	/* 0. �������� */
	eCfgCmdStart = 0x1400,
	eCfgCmdSetDefault = eCfgCmdStart,		/* ������������ΪĬ��ֵ, �������������ص�����, ����� */
	eCfgCmdSave,
	eCfgCmdTiming,
    eCfgCmdReset,
	eCfgCmdEnd = eCfgCmdStart + 0x00FF,
	/* 1. ��д�������� */
	eCfgDataStart,
	eCfgDataEnd = eCfgDataStart + sizeof(CFG_S) / sizeof(float) - 1,
	/* 2. Ԥ�� */
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
