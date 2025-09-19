#include "config.h"
#include "gd32f10x.h"
#include "bsp_gd25q32.h"
#include "history.h"
#include "bsp_gpio.h"
#include <stdio.h>
#include <string.h>
#include <time.h>

CFG_S g_stCfg;

/*******************************************************************************
* Function Name  : cfg_ota_save
* Description    : 将OTA服务相关的配置信息写入Flash
* Input          : None
* Output         : None
* Return         : result, 1写成功 0写失败
*******************************************************************************/
bool cfg_ota_save(void) {
	MEM_FlashWrite(eHisCfgPStart * PAGE_SIZE + (uint32_t)&g_stCfg.stOta - (uint32_t)&g_stCfg, (uint8_t*)&g_stCfg.stOta, sizeof(CFG_OTA_S));
	CFG_RETURN_TRUE;
}

/*******************************************************************************
* Function Name  : cfg_prl_save
* Description    : 将并机相关的配置信息写入Flash
* Input          : None
* Output         : None
* Return         : result, 1写成功 0写失败
*******************************************************************************/
bool cfg_prl_save(void) {
	MEM_FlashWrite(eHisCfgPStart * PAGE_SIZE + (uint32_t)&g_stCfg.stPrl - (uint32_t)&g_stCfg, (uint8_t*)&g_stCfg.stPrl, sizeof(CFG_PRL_S));
	CFG_RETURN_TRUE;
}

/*******************************************************************************
* Function Name  : cfg_save
* Description    : 将所有配置相关的配置信息写入flash
* Input          : None
* Output         : None
* Return         : result, 1写成功 0写失败
*******************************************************************************/
bool cfg_save(void) {
	MEM_FlashWrite(eHisCfgPStart * PAGE_SIZE, (uint8_t*)&g_stCfg, sizeof(CFG_S));
	delay_1ms(100);
	CFG_RETURN_TRUE;
}

/*******************************************************************************
* Function Name  : cfg_load
* Description    : 将所有配置信息从Flash中读出
* Input          : None
* Output         : None
* Return         : result, 1写成功 0写失败
*******************************************************************************/
bool cfg_load(void) {
    MEM_FlashRead(eHisCfgPStart * PAGE_SIZE, (uint8_t*)&g_stCfg, sizeof(CFG_S));
	//cfg_backup_load();
	if((g_stCfg.stCom.auiCanBaud == 250 || g_stCfg.stCom.auiCanBaud == 500)){
	 	CFG_RETURN_TRUE;
	}
    CFG_RETURN_FALSE;

}

/*******************************************************************************
* Function Name  : cfg_set_default
* Description    : 将内存中的配置信息恢复默认值
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
bool cfg_set_default(void) {
	/* 0. OTA配置 */
	memset(&g_stCfg.stOta, 0, sizeof(CFG_OTA_S));

	/* 2. UART & CAN配置 */
	for(uint8_t i=0;i<CFG_MAX_UART_NUM;i++) {
		g_stCfg.stCom.astUart[i].uiBaud = 115200;//串口波特率
        g_stCfg.stCom.astUart[i].uiWordLen = USART_WL_8BIT;//字长为8位数据格式
        g_stCfg.stCom.astUart[i].uiStopBit = USART_STB_1BIT;//一个停止位
        g_stCfg.stCom.astUart[i].uiParity = USART_PM_NONE;//无奇偶校验位
	}

    g_stCfg.stCom.auiCanBaud = 250;

	memset(g_stCfg.stCom.aucReserve, 0, sizeof(g_stCfg.stCom.aucReserve));
	/* 3. 并机服务配置 */
	g_stCfg.stPrl.ePrlSelfIdx = ePrlPackId0;
	g_stCfg.stPrl.ucPrlPackNum = 1;
	memset(g_stCfg.stPrl.aucReserve, 0, sizeof(g_stCfg.stPrl.aucReserve));
	
	/* 4. 历史服务配置 */
	g_stCfg.stHis.ucDataEn = 1;
	g_stCfg.stHis.ucLogEn = 1;
	g_stCfg.stHis.ucWaveEn = 1;
	g_stCfg.stHis.ucReserveBit = 0;
	g_stCfg.stHis.usDataPeriod = 60;
    g_stCfg.stHis.usLogPeriod = 30;
	memset(g_stCfg.stHis.aucReserve, 0, sizeof(g_stCfg.stHis.aucReserve));
	/* 5. AFE配置 */
	/* 这里rom的值因为大小端原因, 奇数字节和偶数字节需要调换 */
    g_stCfg.stAfe.aucRomByte[1] = 0x50;
	g_stCfg.stAfe.aucRomByte[0] = 0x0E;
	g_stCfg.stAfe.aucRomByte[3] = 0x6B;   	
	g_stCfg.stAfe.aucRomByte[2] = 0x00;
	g_stCfg.stAfe.aucRomByte[5] = 0x00;   	
	g_stCfg.stAfe.aucRomByte[4] = 0x00;
	g_stCfg.stAfe.aucRomByte[7] = 0xD8;     //开启电流检测 连续采集 10bit VADC 温度 VADC转换周期50ms	
	g_stCfg.stAfe.aucRomByte[6] = 0x00;  	
	g_stCfg.stAfe.aucRomByte[9] = 0x00;
	g_stCfg.stAfe.aucRomByte[8] = 0xFF;     //CADC范围0~50mv 复位MCU脉冲宽度1S 短路保护电压400mv 短路保护延时500us
	g_stCfg.stAfe.aucRomByte[11] = 0x01;    //硬件过充保护 1个VADC周期 充电状态检测 1.4mv 看门狗溢出时间10S
	g_stCfg.stAfe.aucRomByte[10] = 0xE1; 	
	g_stCfg.stAfe.aucRomByte[12] = 0x42;
	for(uint8_t i=0;i<CFG_TMP_NUM;i++) {
		g_stCfg.stAfe.afTempCali[i] = 0;
	}
	g_stCfg.stAfe.fAvgTempCali = 0.5;
	g_stCfg.stAfe.fCurCaliA = 1;
	g_stCfg.stAfe.fCurCaliB = 0;
	//g_stCfg.stAfe.fCDATACaliA = 0.545;
	//g_stCfg.stAfe.fCDATACaliB = -755.2;
	g_stCfg.stAfe.fCDATACaliA = 0.1;
	g_stCfg.stAfe.fCDATACaliB = 0;
	for(uint8_t i=0;i<CFG_CELL_NUM;i++) {
		g_stCfg.stAfe.afCellVolCali[i] = 1;
	}
	g_stCfg.stAfe.fPackVolCali = 0;
	memset(g_stCfg.stAfe.aucReserve, 0, sizeof(g_stCfg.stAfe.aucReserve));
	/* 6. 本机服务配置 */
	memset(g_stCfg.stLocal.acSN, 0, 32);
	strcpy(g_stCfg.stLocal.acBmsMnfInfo, "Empty BMS mnf info");
	strcpy(g_stCfg.stLocal.acPackMnfInfo, "Empty PACK mnf info");
	strcpy(g_stCfg.stLocal.acMnfName, "Hybrid Tech.");
	g_stCfg.stLocal.usCellNum = CFG_CELL_NUM;
	g_stCfg.stLocal.usSerialCellNum = CFG_CELL_NUM;
	g_stCfg.stLocal.fNominalU = 14.4;
	g_stCfg.stLocal.usDesignAH = 100;
	g_stCfg.stLocal.usCycle = 0;
	/* 电芯过压告警 */
	g_stCfg.stLocal.stCellOVAlm.fValThr = 3.6;
	g_stCfg.stLocal.stCellOVAlm.ucTmThr = 3;
	g_stCfg.stLocal.stCellOVAlm.fRetValThr = 3.55;
	g_stCfg.stLocal.stCellOVAlm.ucRetTmThr = 3;
	/* 电芯过压保护 */
	g_stCfg.stLocal.stCellOVPtct.fValThr = 3.7;
	g_stCfg.stLocal.stCellOVPtct.ucTmThr = 3;
	g_stCfg.stLocal.stCellOVPtct.fRetValThr = 3.6;
	g_stCfg.stLocal.stCellOVPtct.ucRetTmThr = 3;
	/* 电芯欠压告警 */
	g_stCfg.stLocal.stCellUVAlm.fValThr = 2.8;
	g_stCfg.stLocal.stCellUVAlm.ucTmThr = 3;
	g_stCfg.stLocal.stCellUVAlm.fRetValThr = 2.85;
	g_stCfg.stLocal.stCellUVAlm.ucRetTmThr = 3;
	/* 电芯欠压保护 */
	g_stCfg.stLocal.stCellUVPtct.fValThr = 2.6;
	g_stCfg.stLocal.stCellUVPtct.ucTmThr = 3;
	g_stCfg.stLocal.stCellUVPtct.fRetValThr = 2.75;
	g_stCfg.stLocal.stCellUVPtct.ucRetTmThr = 3;
	/* PACK过压告警 */
	g_stCfg.stLocal.stPackOVAlm.fValThr = 14.4;
	g_stCfg.stLocal.stPackOVAlm.ucTmThr = 3;
	g_stCfg.stLocal.stPackOVAlm.fRetValThr = 14.4;
	g_stCfg.stLocal.stPackOVAlm.ucRetTmThr = 3;
	/* PACK过压保护 */
	g_stCfg.stLocal.stPackOVPtct.fValThr = 14.8;
	g_stCfg.stLocal.stPackOVPtct.ucTmThr = 3;
	g_stCfg.stLocal.stPackOVPtct.fRetValThr = 14.8;
	g_stCfg.stLocal.stPackOVPtct.ucRetTmThr = 3;
	/* PACK低压告警 */
	g_stCfg.stLocal.stPackUVAlm.fValThr = 11.2;
	g_stCfg.stLocal.stPackUVAlm.ucTmThr = 3;
	g_stCfg.stLocal.stPackUVAlm.fRetValThr = 11.2;
	g_stCfg.stLocal.stPackUVAlm.ucRetTmThr = 3;
	/* PACK低压保护 */
	g_stCfg.stLocal.stPackUVPtct.fValThr = 10.4;
	g_stCfg.stLocal.stPackUVPtct.ucTmThr = 3;
	g_stCfg.stLocal.stPackUVPtct.fRetValThr = 10.4;
	g_stCfg.stLocal.stPackUVPtct.ucRetTmThr = 3;
	/* 充电过温告警 */
	g_stCfg.stLocal.stChgOTAlm.fValThr = 40;
	g_stCfg.stLocal.stChgOTAlm.ucTmThr = 3;
	g_stCfg.stLocal.stChgOTAlm.fRetValThr = 40;
	g_stCfg.stLocal.stChgOTAlm.ucRetTmThr = 3;
	/* 充电过温保护 */
	g_stCfg.stLocal.stChgOTPtct.fValThr = 60;
	g_stCfg.stLocal.stChgOTPtct.ucRetTmThr = 3;
	g_stCfg.stLocal.stChgOTPtct.fRetValThr = 60;
	g_stCfg.stLocal.stChgOTPtct.ucRetTmThr = 3;
	/* 充电低温告警 */
	g_stCfg.stLocal.stChgUTAlm.fValThr = 2;
	g_stCfg.stLocal.stChgUTAlm.ucTmThr = 3;
	g_stCfg.stLocal.stChgUTAlm.fRetValThr = 2;
	g_stCfg.stLocal.stChgUTAlm.ucRetTmThr = 3;
	/* 充电低温保护 */
	g_stCfg.stLocal.stChgUTPtct.fValThr = 0;
	g_stCfg.stLocal.stChgUTPtct.ucTmThr = 3;
	g_stCfg.stLocal.stChgUTPtct.fRetValThr = 0;
	g_stCfg.stLocal.stChgUTPtct.ucRetTmThr = 3;
	/* 放电过温告警 */
	g_stCfg.stLocal.stDsgOTAlm.fValThr = 40;
	g_stCfg.stLocal.stDsgOTAlm.ucTmThr = 3;
	g_stCfg.stLocal.stDsgOTAlm.fRetValThr = 40;
	g_stCfg.stLocal.stDsgOTAlm.ucRetTmThr = 3;
	/* 放电过温保护 */
	g_stCfg.stLocal.stDsgOTPtct.fValThr = 65;
	g_stCfg.stLocal.stDsgOTPtct.ucTmThr = 3;
	g_stCfg.stLocal.stDsgOTPtct.fRetValThr = 65;
	g_stCfg.stLocal.stDsgOTPtct.ucRetTmThr = 3;
	/* 放电低温告警 */
	g_stCfg.stLocal.stDsgUTAlm.fValThr = -10;
	g_stCfg.stLocal.stDsgUTAlm.ucTmThr = 3;
	g_stCfg.stLocal.stDsgUTAlm.fRetValThr = -10;
	g_stCfg.stLocal.stDsgUTAlm.ucRetTmThr = 3;
	/* 放电低温保护 */
	g_stCfg.stLocal.stDsgUTPtct.fValThr = -20;
	g_stCfg.stLocal.stDsgUTPtct.ucTmThr = 3;
	g_stCfg.stLocal.stDsgUTPtct.fRetValThr = -20;
	g_stCfg.stLocal.stDsgUTPtct.ucRetTmThr = 3;
	/* 环境过温告警 */
	g_stCfg.stLocal.stEnvOTAlm.fValThr = 40;
	g_stCfg.stLocal.stEnvOTAlm.ucTmThr = 3;
	g_stCfg.stLocal.stEnvOTAlm.fRetValThr = 40;
	g_stCfg.stLocal.stEnvOTAlm.ucRetTmThr = 3;
	/* 环境过温保护 */
	g_stCfg.stLocal.stEnvOTPtct.fValThr = 60;
	g_stCfg.stLocal.stEnvOTPtct.ucTmThr = 3;
	g_stCfg.stLocal.stEnvOTPtct.fRetValThr = 60;
	g_stCfg.stLocal.stEnvOTPtct.ucRetTmThr = 3;
	/* 环境低温告警 */
	g_stCfg.stLocal.stEnvUTAlm.fValThr = -10;
	g_stCfg.stLocal.stEnvUTAlm.ucTmThr = 3;
	g_stCfg.stLocal.stEnvUTAlm.fRetValThr = -10;
	g_stCfg.stLocal.stEnvUTAlm.ucRetTmThr = 3;
	/* 环境低温保护 */
	g_stCfg.stLocal.stEnvUTPtct.fValThr = -20;
	g_stCfg.stLocal.stEnvUTPtct.ucTmThr = 3;
	g_stCfg.stLocal.stEnvUTPtct.fRetValThr = -20;
	g_stCfg.stLocal.stEnvUTPtct.ucRetTmThr = 3;
	/* 充电过流告警 */
	g_stCfg.stLocal.stOCCAlm.fValThr = 300;
	g_stCfg.stLocal.stOCCAlm.ucTmThr = 3;
	g_stCfg.stLocal.stOCCAlm.fRetValThr = 300;
	g_stCfg.stLocal.stOCCAlm.ucRetTmThr = 3;
	/* 充电过流保护 */
	g_stCfg.stLocal.stOCCPtct.fValThr = 350;
	g_stCfg.stLocal.stOCCPtct.ucTmThr = 3;
	g_stCfg.stLocal.stOCCPtct.fRetValThr = 350;
	g_stCfg.stLocal.stOCCPtct.ucRetTmThr = 3;
	/* 放电过流告警 */
	g_stCfg.stLocal.stODCAlm.fValThr = -550;
	g_stCfg.stLocal.stODCAlm.ucTmThr = 3;
	g_stCfg.stLocal.stODCAlm.fRetValThr = -550;
	g_stCfg.stLocal.stODCAlm.ucRetTmThr = 3;
	/* 放电过流保护 */
	g_stCfg.stLocal.stODCPtct.fValThr = -600;
	g_stCfg.stLocal.stODCPtct.ucTmThr = 3;
	g_stCfg.stLocal.stODCPtct.fRetValThr = -600;
	g_stCfg.stLocal.stODCPtct.ucRetTmThr = 3;
	/* MOS高温告警 */
	g_stCfg.stLocal.stMosOTAlm.fValThr = 70;
	g_stCfg.stLocal.stMosOTAlm.ucTmThr = 3;
	g_stCfg.stLocal.stMosOTAlm.fRetValThr = 70;
	g_stCfg.stLocal.stMosOTAlm.ucRetTmThr = 3;
	/* MOS高温保护 */
	g_stCfg.stLocal.stMosOTPtct.fValThr = 90;
	g_stCfg.stLocal.stMosOTPtct.ucTmThr = 3;
	g_stCfg.stLocal.stMosOTPtct.fRetValThr = 85;
	g_stCfg.stLocal.stMosOTPtct.ucRetTmThr = 3;
	
	g_stCfg.stLocal.ucLedAlmEn = 1;
	g_stCfg.stLocal.ucCurLimitLv = 0;
	g_stCfg.stLocal.ucCurLimitEn = 1;
	g_stCfg.stLocal.ucBeepAlmEn = 1;
	g_stCfg.stLocal.usFCHGMaxCellU = 3000;
	g_stCfg.stLocal.usFCHGMinCellU = 2900;
	g_stCfg.stLocal.usBCHGDeltaCellU = 50;
	g_stCfg.stLocal.usChgAdjSVolt = 3450;
	g_stCfg.stLocal.usChgAdjEVolt = 3500;
	g_stCfg.stLocal.usDsgAdjSVolt = 2750;
	g_stCfg.stLocal.usDsgAdjEVolt = 2600;
	g_stCfg.stLocal.uiSleepSec = 259200;
	g_stCfg.stLocal.usCyclePeriod = 500;
	memset(&g_stCfg.stLocal.stTUI, 0, sizeof(g_stCfg.stLocal.stTUI));
	g_stCfg.stLocal.stTUI.asTriggerT[0] = 50;
	g_stCfg.stLocal.stTUI.asRecovery[0] = 45;
	g_stCfg.stLocal.stTUI.asTriggerT[1] = 45;
	g_stCfg.stLocal.stTUI.asRecovery[1] = 40;
	g_stCfg.stLocal.stTUI.asTriggerT[2] = 40;
	g_stCfg.stLocal.stTUI.asRecovery[2] = 10;
	g_stCfg.stLocal.stTUI.asTriggerT[3] = 10;
	g_stCfg.stLocal.stTUI.asRecovery[3] = 12;
	g_stCfg.stLocal.stTUI.asTriggerT[4] = 2;
	g_stCfg.stLocal.stTUI.asRecovery[4] = 5;
	g_stCfg.stLocal.stTUI.asTriggerT[5] = -10;
	g_stCfg.stLocal.stTUI.asRecovery[5] = -8;
	g_stCfg.stLocal.stTUI.asTriggerT[6] = -20;
	g_stCfg.stLocal.stTUI.asRecovery[6] = -18;
	g_stCfg.stLocal.stTUI.ausHV[0] = 5000;
	g_stCfg.stLocal.stTUI.ausLV[0] = 3505;
	g_stCfg.stLocal.stTUI.ausHV[1] = 3495;
	g_stCfg.stLocal.stTUI.ausLV[1] = 3485;
	g_stCfg.stLocal.stTUI.ausHV[2] = 3475;
	g_stCfg.stLocal.stTUI.ausLV[2] = 3460;
	g_stCfg.stLocal.stTUI.ausHV[3] = 3440;
	g_stCfg.stLocal.stTUI.ausLV[3] = 3010;
	g_stCfg.stLocal.stTUI.ausHV[4] = 2990;
	g_stCfg.stLocal.stTUI.ausLV[4] = 2910;
	g_stCfg.stLocal.stTUI.ausHV[5] = 2890;
	g_stCfg.stLocal.stTUI.ausLV[5] = 0;
	CFG_TUI_PARA_S aastTUIPara[CFG_MAX_TUI_ROWS][CFG_MAX_TUI_COLUMNS] = {
		{		/* 高温II */
			{  0, 3510, 5616,  20, 3000, 4640},
			{  5, 3510, 5616,  20, 3000, 4640},
			{  5, 3510, 5616,  10, 3000, 4640},
			{  5, 3510, 5616,  10, 3000, 4640},
			{  5, 3510, 5616,  10, 3000, 4640},
			{  5, 3510, 5616,   0, 3000, 4640}
		},{	/* 高温I */
			{  0, 3510, 5616, 100, 3000, 4640},
			{ 20, 3510, 5616, 100, 3000, 4640},
			{ 50, 3510, 5616, 100, 3000, 4640},
			{ 50, 3510, 5616, 105, 3000, 4640},
			{ 20, 3510, 5616,  10, 3000, 4640},
			{ 20, 3510, 5616,   0, 3000, 4640},
		},{	/* 常温 */
			{ 0,  3510, 5616, 100, 3000, 4640},
			{ 20, 3510, 5616, 100, 3000, 4640},
			{ 50, 3510, 5616, 100, 3000, 4640},
			{100, 3510, 5616, 100, 3000, 4640},
			{ 20, 3510, 5616,  10, 3000, 4640},
			{ 20, 3510, 5616,   0, 3000, 4640},
		},{	/* 低温I */
			{  0, 3510, 5616,  50, 3000, 4640},
			{ 20, 3510, 5616,  50, 3000, 4640},
			{ 50, 3510, 5616,  50, 3000, 4640},
			{ 50, 3510, 5616,  50, 3000, 4640},
			{ 10, 3510, 5616,  10, 3000, 4640},
			{ 10, 3510, 5616,   0, 3000, 4640},
		},{	/* 低温II */
			{  0, 3510, 5616,  20, 3000, 4640},
			{ 10, 3450, 5616,  20, 3000, 4640},
			{ 20, 3450, 5616,  20, 3000, 4640},
			{ 20, 3450, 5616,  20, 3000, 4640},
			{  5, 3510, 5616,  10, 3000, 4640},
			{  5, 3510, 5616,   0, 3000, 4640},
		},{	/* 低温III */
			{  0, 3450, 5520,  10, 3000, 4640},
			{  0, 3450, 5520,  10, 3000, 4640},
			{  0, 3450, 5520,  10, 3000, 4640},
			{  0, 3450, 5520,  10, 3000, 4640},
			{  0, 3450, 5520,  10, 3000, 4640},
			{  0, 3450, 5520,   0, 3000, 4640},
		},{	/* 低温IV */
			{  0, 3450, 5520,   0, 3000, 4640},
			{  0, 3450, 5520,   0, 3000, 4640},
			{  0, 3450, 5520,   0, 3000, 4640},
			{  0, 3450, 5520,   0, 3000, 4640},
			{  0, 3450, 5520,   0, 3000, 4640},
			{  0, 3450, 5520,   0, 3000, 4640},
		}
	};
	memcpy(g_stCfg.stLocal.stTUI.aastTUIPara, aastTUIPara, sizeof(CFG_TUI_PARA_S) * CFG_MAX_TUI_ROWS * CFG_MAX_TUI_COLUMNS);
	g_stCfg.stLocal.ucPDSGDly = 3;
	g_stCfg.stLocal.usBCHGMaxCellU = 3400;
	/* Y. 写入Flash */
	return cfg_save();
}



/*******************************************************************************
* Function Name  : cfg_init
* Description    : 将flash中的配置信息载入到内存中
* Input          : None
* Output         : None
* Return         : result, 1写成功 0写失败
*******************************************************************************/
bool cfg_init(void) {
	if(!cfg_load()) {
		cfg_set_default();
		CFG_RETURN_FALSE;
	}
    //cfg_set_default();
	g_stCfg.stLocal.ucHVer = 1;
	g_stCfg.stLocal.ucHVerR = 1;
	g_stCfg.stLocal.ucHVerV = 0;
	g_stCfg.stLocal.ucSVerVMajor = 1;
	g_stCfg.stLocal.ucSVerVMinor = 0;
	g_stCfg.stLocal.ucSVerRMajor = 1;
	g_stCfg.stLocal.ucSVerRMinor = 5;
	if(g_stCfg.stLocal.ucPDSGDly > 10) {
		g_stCfg.stLocal.ucPDSGDly = 10;
	}
	if(g_stCfg.stOta.ucUpdate != 0 && g_stCfg.stOta.ucUpdate != 0xFF) {
		CFG_DEBUG("OTA配置处于待升级状态, 固件长度: %d, OTA异常", g_stCfg.stOta.uiLen);
	}
	CFG_RETURN_TRUE;
}

/*******************************************************************************
* Function Name  : cfg_proc
* Description    : 配置服务, 每次主循环调用
* Input          : None
* Output         : None
* Return         : result, 1写成功 0写失败
*******************************************************************************/
bool cfg_proc(void) {
//	if(g_stAppArrayRVal.eAppStat != eAppStatRun) {
//		CFG_RETURN_TRUE;
//	}
	
	CFG_RETURN_TRUE;
}
