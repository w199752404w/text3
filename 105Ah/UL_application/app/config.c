#include "config.h"
#include "gd32f10x.h"
#include "bsp_gd25q32.h"
#include "history.h"
#include "local.h"
#include "bsp_rtc.h"
#include "bsp_gpio.h"
#include "bsp_usart.h"
#include <stdio.h>
#include <string.h>
#include "bsp_exti.h"
#include <time.h>

CFG_S g_stCfg = {0};
CFG_BackUp g_stCfg_BackUp;
bool g_SetDefaultFlag = 0;

/*******************************************************************************
* Function Name  : cfg_save
* Description    : Write all configuration-related configuration information to flash
* Input          : None
* Output         : None
* Return         : result, 1 write succeeded, 0 write failed
*******************************************************************************/
bool cfg_save(void) {
	MEM_FlashWrite(eHisCfgPStart * PAGE_SIZE, (uint8_t*)&g_stCfg, sizeof(CFG_S));
	delay_1ms(100);
	CFG_RETURN_TRUE;
}

/*******************************************************************************
* Function Name  : cfg_load
* Description    : Read out all configuration information from Flash
* Input          : None
* Output         : None
* Return         : result, 1 write succeeded, 0 write failed
*******************************************************************************/
bool cfg_load(void) {
	MEM_FlashRead(eHisCfgPStart * PAGE_SIZE, (uint8_t*)&g_stCfg, sizeof(CFG_S));
	cfg_backup_load();
	if((g_stCfg.stCom.uiCanBaud == 250 || g_stCfg.stCom.uiCanBaud == 500) && g_stCfg.stLocal.usCyclePeriod != 0xFFFF && g_stCfg.stLocal.ucSWPulse ==0 && g_stCfg.stLocal.ucSW2Pulse ==1){
	 	CFG_RETURN_TRUE;
	}
  CFG_RETURN_FALSE;
}

/*******************************************************************************
* Function Name  : cfg_set_default
* Description    : Restore the configuration information in memory to its default value
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
bool cfg_set_default(void) {
	g_SetDefaultFlag = 1;
	/* 0. OTA configuration */
	memset(&g_stCfg.stOta, 0, sizeof(CFG_OTA_S));
	/* start after reset */
	g_stCfg.usGoRun = 3;
	/* 2. UART & CAN configuration */
	g_stCfg.stCom.stUart.uiBaud = 115200;//Serial baud rate
	g_stCfg.stCom.stUart.uiWordLen = USART_WL_8BIT;//The word length is in 8-bit data format
	g_stCfg.stCom.stUart.uiStopBit = USART_STB_1BIT;//A stop bit
	g_stCfg.stCom.stUart.uiParity = USART_PM_NONE;//No parity bits
	g_stCfg.stCom.uiCanBaud = 250;		//version A: 250k; version B: 500k
	memset(g_stCfg.stCom.aucReserve, 0, sizeof(g_stCfg.stCom.aucReserve));
	/* 4. Historical service configuration */
	g_stCfg.stHis.ucDataEn = 1;
	g_stCfg.stHis.ucLogEn = 1;
	g_stCfg.stHis.ucWaveEn = 1;
	g_stCfg.stHis.ucReserveBit = 0;
	g_stCfg.stHis.usDataPeriod = 60;
	g_stCfg.stHis.usLogPeriod = 30;
	memset(g_stCfg.stHis.aucReserve, 0, sizeof(g_stCfg.stHis.aucReserve));
	/* 5. AFE configuration */
	/* The value of the ROM here needs to be reversed because of the big and small ends, and the odd and even numbers need to be reversed */
	g_stCfg.stAfe.aucRomByte[1] = 0x40;//40
	g_stCfg.stAfe.aucRomByte[0] = 0x0E;
	g_stCfg.stAfe.aucRomByte[3] = 0x6B;   /* OV trigger 4.4V */
	g_stCfg.stAfe.aucRomByte[2] = 0x70;
	g_stCfg.stAfe.aucRomByte[5] = 0x63;   /* OV recover 4.2V */
	g_stCfg.stAfe.aucRomByte[4] = 0x48;
	g_stCfg.stAfe.aucRomByte[7] = 0x32;   /* UV trigger 1.0V */
	g_stCfg.stAfe.aucRomByte[6] = 0x3C;   /* UV recover 1.2V */
	g_stCfg.stAfe.aucRomByte[9] = 0xA5;//AA
	g_stCfg.stAfe.aucRomByte[8] = 0x7D;
	g_stCfg.stAfe.aucRomByte[11] = 0x64;
	g_stCfg.stAfe.aucRomByte[10] = 0xE1;  /* OV Secondary triggering 4.5V */
	g_stCfg.stAfe.aucRomByte[13] = 0xF2;	/* OCD1,100,150,200,250,300,350,400,450,500,550,600,650,700,800,900,1000A;					50,100,200,400,600,800,1000,2000,4000,6000,8000,10000,15000,20000,30000,40000ms */
	g_stCfg.stAfe.aucRomByte[12] = 0xC2;	/* OCD2,150,200,250,300,350,400,450,500,600,700,800,900,1000,1500,2000,2500A;				10,20,40,60,80,100,200,400,600,800,1000,2000,4000,8000,10000,20000ms */
	g_stCfg.stAfe.aucRomByte[15] = 0xBF;	/* 原1000A 0x5F 现2000A */ /* SC , 250,400,550,700,850,1000,1150,1300,1450,1600,1750,2000,2500,3000,4000,5000A;0,64,128,192,256,320,384,448,512,576,640,704,768,832,896,960us */
	g_stCfg.stAfe.aucRomByte[14] = 0x42;	/* OCC, 100,150,200,250,300,350,400,450,500,550,600,650,700,800,900,1000A;					10,20,40,60,80,100,200,400,600,800,1000,2000,4000,8000,10000,20000ms */
	g_stCfg.stAfe.aucRomByte[17] = 0x00;
	g_stCfg.stAfe.aucRomByte[16] = 0x00;
	g_stCfg.stAfe.aucRomByte[19] = 0x00;
	g_stCfg.stAfe.aucRomByte[18] = 0xEC;	/* Charging low temperature protection threshold -45 UTC */
	g_stCfg.stAfe.aucRomByte[21] = 0xEC;	/* Charging Low Temperature Protection Release Threshold -45 UTCR */
	g_stCfg.stAfe.aucRomByte[20] = 0x00;
	g_stCfg.stAfe.aucRomByte[23] = 0x00;
	g_stCfg.stAfe.aucRomByte[22] = 0xEC;	/* Discharge cryogenic protection threshold -45 UTD */
	g_stCfg.stAfe.aucRomByte[25] = 0xEC;	/* Discharge cryogenic protection release threshold -45 UTCR */
	g_stCfg.stAfe.aucRomByte[24] = 0x00;
	for(uint8_t i=0;i<CFG_TMP_NUM;i++) {
		g_stCfg.stAfe.afTempCali[i] = 0;
	}
	g_stCfg.stAfe.fAvgTempCali = 0.5;
  g_stCfg.stAfe.fCurCaliA = 1;
  g_stCfg.stAfe.fCurCaliB = 0;
//g_stCfg.stAfe.fCDATACaliA = 1.16;
//g_stCfg.stAfe.fCDATACaliB = 0;
	g_stCfg.stAfe.fPackVolCali = 0;
//g_stCfg.stAfe.fCDATACaliA_CHG = 1;
//g_stCfg.stAfe.fCDATACaliA_DSG = 1;
//for(uint8_t i=0;i<CFG_CELL_NUM;i++) {
//	g_stCfg.stAfe.afCellVolCali[i] = 1;
//}
	memset(g_stCfg.stAfe.aucReserve, 0, sizeof(g_stCfg.stAfe.aucReserve));
	/* 6. Native service configuration */
	g_stCfg.stLocal.ucHVer = 3;
	g_stCfg.stLocal.ucHVerV = 3;
	g_stCfg.stLocal.ucHVerR = 1;
//	g_stCfg.stLocal.ucSVerVMajor = 3;
//	g_stCfg.stLocal.ucSVerVMinor = 1;
//	g_stCfg.stLocal.ucSVerRMajor = 4;
//	g_stCfg.stLocal.ucSVerRMinor = 0;
	g_stCfg.stLocal.ucSWPulse = 0;
	g_stCfg.stLocal.ucSW2Pulse = 1;   
	g_stCfg.stLocal.ausHWSN[0] = 0x1234;
	g_stCfg.stLocal.ausHWSN[1] = 0x5678;
	g_stCfg.stLocal.ausHWSN[2] = 0x9ABC;
	g_stCfg.stLocal.ausHWSN[3] = 0xDEF0;
	g_stCfg.stLocal.usCellNum = CFG_CELL_NUM;
	g_stCfg.stLocal.usSerialCellNum = CFG_CELL_NUM;
	g_stCfg.stLocal.fNominalU = 3.2 * CFG_CELL_NUM;
	g_stCfg.stLocal.usDesignAH = 95;
	g_stCfg.stLocal.usCycle = 0;
	g_stCfg.stLocal.iStandbySlpSec = 604800;		//604800s = 1 week = 168h
	g_stCfg.stLocal.iLowPwrSlpSec = 86400;			//86400s = 1 days 518400s = 144h
	g_stCfg.stLocal.iSreenSlpSec = -3600;       // -3600s, -1 hours,Indicates that it is not dormant
	g_stCfg.stLocal.usDbgPara = 250;
	g_stCfg.stLocal.usCyclePeriod = 50;
	g_stCfg.stLocal.usBALActVolt = 3300;
	g_stCfg.stLocal.usBALActDVolt = 10;//2024.8.22
	g_stCfg.stLocal.usVALReactDVolt = 65535;
	g_stCfg.stLocal.sChgCurSnrCoeB = 100;
	g_stCfg.stLocal.sDsgCurSnrCoeB = 100;
	g_stCfg.stLocal.sVoltSnrCoeB = 0;
	g_stCfg.stLocal.usCurLmtMode = 0;
	g_stCfg.stLocal.ausSocFCaliU[0] = 3650;  //100% demarcate
	g_stCfg.stLocal.ausSocFCaliU[1] = 3450;  //100% demarcate
	g_stCfg.stLocal.ausSocECaliU[0] = 2850;  //10%  demarcate
	g_stCfg.stLocal.ausSocECaliU[1] = 2700;  //5%   demarcate
	g_stCfg.stLocal.ausSocECaliU[2] = 2600;  //0%   demarcate
	g_stCfg.stLocal.usSCRTThr = 10000;
	g_stCfg.stLocal.usCellOVTVThr1 = 3700;
	g_stCfg.stLocal.usCellOVTTThr1 = 6000;
	g_stCfg.stLocal.usCellOVRVThr1 = 3550;
	g_stCfg.stLocal.usCellOVRTThr1 = 0;
	g_stCfg.stLocal.usCellOVTVThr2 = 3500;
	g_stCfg.stLocal.usCellOVTTThr2 = 3000;
	g_stCfg.stLocal.usCellOVRVThr2 = 3550;
	g_stCfg.stLocal.usCellOVRTThr2 = 0;
	g_stCfg.stLocal.usCellUVTVThr1 = 2450;
	g_stCfg.stLocal.usCellUVTTThr1 = 3000;
	g_stCfg.stLocal.usCellUVRVThr1 = 2900;
	g_stCfg.stLocal.usCellUVRTThr1 = 0;
	g_stCfg.stLocal.usCellUVTVThr2 = 2550;
	g_stCfg.stLocal.usCellUVTTThr2 = 3000;
	g_stCfg.stLocal.usCellUVRVThr2 = 2900;
	g_stCfg.stLocal.usCellUVRTThr2 = 0;
	g_stCfg.stLocal.usCellUVTVThr3 = 2600;
	g_stCfg.stLocal.usCellUVTTThr3 = 3000;
	g_stCfg.stLocal.usCellUVRVThr3 = 2850;
	g_stCfg.stLocal.usCellUVRTThr3 = 0;
	g_stCfg.stLocal.sCellOCTTVThr1 = 65;
	g_stCfg.stLocal.usCellOCTTTThr1 = 5000;
	g_stCfg.stLocal.sCellOCTRVThr1 = 60;
	g_stCfg.stLocal.usCellOCTRTThr1 = 0;
	g_stCfg.stLocal.sCellODTTVThr1 = 65;
	g_stCfg.stLocal.usCellODTTTThr1 = 5000;
	g_stCfg.stLocal.sCellODTRVThr1 = 60;
	g_stCfg.stLocal.usCellODTRTThr1 = 0;
	g_stCfg.stLocal.sCellUCTTVThr1 = 0;
	g_stCfg.stLocal.usCellUCTTTThr1 = 3000;
	g_stCfg.stLocal.sCellUCTRVThr1 = 1;
	g_stCfg.stLocal.usCellUCTRTThr1 = 30000;
	g_stCfg.stLocal.sCellUDTTVThr1 = -15;
	g_stCfg.stLocal.usCellUDTTTThr1 = 5000;
	g_stCfg.stLocal.sCellUDTRVThr1 = -10;
	g_stCfg.stLocal.usCellUDTRTThr1 = 0;
	g_stCfg.stLocal.sHeaterUTTVThr = 2;
	g_stCfg.stLocal.usHeaterUTTTThr = 5000;
	g_stCfg.stLocal.sHeaterUTRVThr = 8;
	g_stCfg.stLocal.usHeaterUTRTThr = 5000;
 	g_stCfg.stLocal.usPackOVTVThr1 = 584;
	g_stCfg.stLocal.usPackOVTTThr1 = 3000;
	g_stCfg.stLocal.usPackOVRVThr1 = 568;
	g_stCfg.stLocal.usPackOVRTThr1 = 0;
	g_stCfg.stLocal.usPackOVTVThr2 = 560;
	g_stCfg.stLocal.usPackOVTTThr2 = 3000;
	g_stCfg.stLocal.usPackOVRVThr2 = 568;
	g_stCfg.stLocal.usPackOVRTThr2 = 0;
	g_stCfg.stLocal.usPackUVTVThr1 = 400;
	g_stCfg.stLocal.usPackUVTTThr1 = 3000;
	g_stCfg.stLocal.usPackUVRVThr1 = 464;
	g_stCfg.stLocal.usPackUVRTThr1 = 0;
	g_stCfg.stLocal.usPackUVTVThr2 = 416;
	g_stCfg.stLocal.usPackUVTTThr2 = 3000;
	g_stCfg.stLocal.usPackUVRVThr2 = 464;
	g_stCfg.stLocal.usPackUVRTThr2 = 0;
	g_stCfg.stLocal.usPackUVTVThr3 = 424;
	g_stCfg.stLocal.usPackUVTTThr3 = 3000;
	g_stCfg.stLocal.usPackUVRVThr3 = 464;
	g_stCfg.stLocal.usPackUVRTThr3 = 0;
	g_stCfg.stLocal.sMOSOTTVThr1 = 90;
	g_stCfg.stLocal.usMOSOTTTThr1 = 5000;
	g_stCfg.stLocal.sMOSOTRVThr1 = 80;
	g_stCfg.stLocal.usMOSOTRTThr1 = 0;
	g_stCfg.stLocal.sMOSOTTVThr2 = 88;
	g_stCfg.stLocal.usMOSOTTTThr2 = 5000;
	g_stCfg.stLocal.sMOSOTRVThr2 = 85;
	g_stCfg.stLocal.usMOSOTRTThr2 = 0;
	g_stCfg.stLocal.sMOSOTTVThr3 = 70;
	g_stCfg.stLocal.usMOSOTTTThr3 = 5000;
	g_stCfg.stLocal.sMOSOTRVThr3 = 65;
	g_stCfg.stLocal.usMOSOTRTThr3 = 0;
	g_stCfg.stLocal.usOCCTVThr1 = 250;
	g_stCfg.stLocal.usOCCTTThr1 = 3000;
	g_stCfg.stLocal.usOCCRVThr1 = g_stCfg.stLocal.usOCCTVThr1;
	g_stCfg.stLocal.usOCCRTThr1 = 1000;
	g_stCfg.stLocal.usODCTVThr1 = 400;
	g_stCfg.stLocal.usODCTTThr1 = 10000;
	g_stCfg.stLocal.usODCRVThr1 = g_stCfg.stLocal.usODCTVThr1;
	g_stCfg.stLocal.usODCRTThr1 = 1000;
	g_stCfg.stLocal.sBalanceOn = 0;
	g_stCfg.stLocal.usBalanceValue = 3300;
	g_stCfg.stLocal.usBalanceDiff = 10;
	g_stCfg.stLocal.usSocCaliDelay = 1;
	g_stCfg.stLocal.usSocCaliCurVal = 2;
	g_stCfg.stCom.ucCanPtcType = 1;
	{
		HIS_DATA_S stData = {0};
		his_data_read(0, &stData);
		if(stData.stRtv.ucCanPtcType == 1) {
			g_stCfg.stCom.ucCanPtcType = 1;
			g_stCfg.stLocal.ucSVerAgree = 'A';//0x41
		} else if(stData.stRtv.ucCanPtcType == 2) {
			g_stCfg.stCom.ucCanPtcType = 2;
			g_stCfg.stCom.uiCanBaud = 250;		//version A: 250k; version B: 500k
			g_stCfg.stLocal.ucSVerAgree = 'B';
		} else if(stData.stRtv.ucCanPtcType == 3) {
			g_stCfg.stCom.ucCanPtcType = 3;
			g_stCfg.stCom.uiCanBaud = 500;		//version A: 250k; version B: 500k
			g_stCfg.stLocal.ucSVerAgree = 'C';
		} else if(stData.stRtv.ucCanPtcType == 4) {
			g_stCfg.stCom.ucCanPtcType = 4;
			g_stCfg.stCom.uiCanBaud = 500;		//version A: 250k; version B: 500k
			g_stCfg.stLocal.ucSVerAgree = 'D';
		} else if(stData.stRtv.ucCanPtcType == 5) {
			g_stCfg.stCom.ucCanPtcType = 5;
			g_stCfg.stCom.uiCanBaud = 250;		//version A: 250k; version B: 500k
			g_stCfg.stLocal.ucSVerAgree = 'E';
		} else if(stData.stRtv.ucCanPtcType == 6) {
			g_stCfg.stCom.ucCanPtcType = 6;
			g_stCfg.stCom.uiCanBaud = 250;		//version A: 250k; version B: 500k
			g_stCfg.stLocal.ucSVerAgree = 'F';
		} else if(stData.stRtv.ucCanPtcType == 7) {
			g_stCfg.stCom.ucCanPtcType = 7;
			g_stCfg.stCom.uiCanBaud = 250;		//version A: 250k; version B: 500k
			g_stCfg.stLocal.ucSVerAgree = 'G';
		} else {
			g_stCfg.stCom.ucCanPtcType = 1;
			g_stCfg.stLocal.ucSVerAgree = 'A';
		}
	}
	/* Y. Write to Flash */
	return cfg_save();
}

/*******************************************************************************
* Function Name  : cfg_backup_load
* Description    : Load the backup configuration information in the flash into memory
* Input          : None
* Output         : None
* Return         : result, 1 write succeeded, 0 write failed
*******************************************************************************/
bool cfg_backup_load(void) {
	uint8_t Need_Back = 0;
	MEM_FlashRead(eCfgBackUpStart, (uint8_t*)&g_stCfg_BackUp, sizeof(CFG_BackUp));
	/* Initialize the backup configuration */
	if((uint8_t)g_stCfg_BackUp.aucBleName_BackUp[0] == 0xFF && (uint8_t)g_stCfg_BackUp.aucBleName_BackUp[1] == 0xFF && (uint8_t)g_stCfg_BackUp.aucBleName_BackUp[2] == 0xFF && (uint8_t)g_stCfg_BackUp.aucBleName_BackUp[3] == 0xFF){
		if((uint8_t)g_stCfg.stLocal.aucBleName[0] == 0xFF && (uint8_t)g_stCfg.stLocal.aucBleName[1] == 0xFF && (uint8_t)g_stCfg.stLocal.aucBleName[2] == 0xFF && (uint8_t)g_stCfg.stLocal.aucBleName[3] == 0xFF){
			memcpy(g_stCfg_BackUp.aucBleName_BackUp,"WO0123456789",19);			
		}else{
			memcpy(g_stCfg_BackUp.aucBleName_BackUp,g_stCfg.stLocal.aucBleName,19);
		}
		Need_Back =1;
	}
	if((uint8_t)g_stCfg_BackUp.aucBMS_ID_BackUp[0] == 0xFF && (uint8_t)g_stCfg_BackUp.aucBMS_ID_BackUp[1] == 0xFF && (uint8_t)g_stCfg_BackUp.aucBMS_ID_BackUp[2] == 0xFF && (uint8_t)g_stCfg_BackUp.aucBMS_ID_BackUp[3] == 0xFF){
		if((uint8_t)g_stCfg.stLocal.aucBMS_ID[0] == 0xFF && (uint8_t)g_stCfg.stLocal.aucBMS_ID[1] == 0xFF && (uint8_t)g_stCfg.stLocal.aucBMS_ID[2] == 0xFF && (uint8_t)g_stCfg.stLocal.aucBMS_ID[3] == 0xFF){
			memcpy(g_stCfg_BackUp.aucBMS_ID_BackUp,"EB30BTP2102401010001",25);
		}else{
			memcpy(g_stCfg_BackUp.aucBMS_ID_BackUp,g_stCfg.stLocal.aucBMS_ID,25);
		}	
    Need_Back =1;		
	}
	if(g_stCfg.stAfe.fCDATACaliA_CHG != 0xFFFFFFFF && g_stCfg_BackUp.fCDATACaliA_CHG_BackUp == 0xFFFFFFFF){
		g_stCfg_BackUp.fCDATACaliA_CHG_BackUp = g_stCfg.stAfe.fCDATACaliA_CHG;
    Need_Back =1;		
	}
	if(g_stCfg.stAfe.fCDATACaliA_DSG != 0xFFFFFFFF && g_stCfg_BackUp.fCDATACaliA_DSG_BackUp == 0xFFFFFFFF){
		g_stCfg_BackUp.fCDATACaliA_DSG_BackUp = g_stCfg.stAfe.fCDATACaliA_DSG;
		Need_Back =1;
	}
	
	/*Get the backup configuration*/
	if((strcmp(g_stCfg.stLocal.aucBMS_ID,g_stCfg_BackUp.aucBMS_ID_BackUp)) && (uint8_t)g_stCfg_BackUp.aucBMS_ID_BackUp[0] != 0xFF 
		&& (uint8_t)g_stCfg_BackUp.aucBMS_ID_BackUp[1] != 0xFF && (uint8_t)g_stCfg_BackUp.aucBMS_ID_BackUp[2] != 0xFF  
		&& (uint8_t)g_stCfg_BackUp.aucBMS_ID_BackUp[3] != 0xFF){
		memcpy(g_stCfg.stLocal.aucBMS_ID,g_stCfg_BackUp.aucBMS_ID_BackUp,25); 
	}
	if((strcmp(g_stCfg.stLocal.aucBleName,g_stCfg_BackUp.aucBleName_BackUp)) && (uint8_t)g_stCfg_BackUp.aucBleName_BackUp[0] != 0xFF 
		&& (uint8_t)g_stCfg_BackUp.aucBleName_BackUp[1] != 0xFF && (uint8_t)g_stCfg_BackUp.aucBleName_BackUp[2] != 0xFF 
		&& (uint8_t)g_stCfg_BackUp.aucBleName_BackUp[3] != 0xFF){
		memcpy(g_stCfg.stLocal.aucBleName,g_stCfg_BackUp.aucBleName_BackUp,19); 
	}
	if(g_stCfg_BackUp.fCDATACaliA_CHG_BackUp != g_stCfg.stAfe.fCDATACaliA_CHG && g_stCfg_BackUp.fCDATACaliA_CHG_BackUp != 0xFFFFFFFF){
		g_stCfg.stAfe.fCDATACaliA_CHG = g_stCfg_BackUp.fCDATACaliA_CHG_BackUp;
	}
	if(g_stCfg_BackUp.fCDATACaliA_DSG_BackUp != g_stCfg.stAfe.fCDATACaliA_DSG && g_stCfg_BackUp.fCDATACaliA_DSG_BackUp != 0xFFFFFFFF){
		g_stCfg.stAfe.fCDATACaliA_DSG = g_stCfg_BackUp.fCDATACaliA_DSG_BackUp;
	}
  if(Need_Back ==1){
		MEM_FlashWrite(eCfgBackUpStart, (uint8_t*)&g_stCfg_BackUp, sizeof(CFG_BackUp));
	}
  CFG_RETURN_TRUE;
}

/*******************************************************************************
* Function Name  : cfg_init
* Description    : Load the configuration information from the flash into memory
* Input          : None
* Output         : None
* Return         : result, 1 write succeeded, 0 write failed
*******************************************************************************/
bool cfg_init(void) {
	if(!cfg_load()) {
		cfg_set_default();
		CFG_RETURN_FALSE;
	}
//	cfg_set_default();
	if(g_stCfg.stOta.ucUpdate != 0 && g_stCfg.stOta.ucUpdate != 0xFF) {
//		CFG_DEBUG("The OTA configuration is in the pending upgrade state, firmware length: %d, OTA is abnormal", g_stCfg.stOta.uiLen);
	}
	if(guc_FirstFlag == 1 && g_SetDefaultFlag == 1){     //First run
		g_stCfg.stAfe.fCDATACaliA_CHG = 1;
    g_stCfg.stAfe.fCDATACaliA_DSG = 1;
    for(uint8_t i=0;i<CFG_CELL_NUM;i++) {
	    g_stCfg.stAfe.afCellVolCali[i] = 1;
    }
//		for(uint8_t j = 0; j < 25; j++ ){
//			g_stCfg.stLocal.aucBMS_ID[j] = '1';
//		}
		memcpy(g_stCfg.stLocal.aucBMS_ID,"EB30BTP2102401010001",20);
		memset(g_stCfg.stLocal.aucBMS_ID + 20, 0, 5);
		memcpy(g_stCfg.stLocal.aucBleName, "WO0123456789", 12);
    memset(g_stCfg.stLocal.aucBleName + 12, 0, 8);
	}
	// V3.3.7.1->3.3.7.2, h00205922, 2024.03.31
	// 1)modified external can id 0x273 sending data
	// 2)delete unnecessary global varible 'g_HeatFlag'
	// 3)change varible name 'g_g_usCanChgerOnTick' to 'g_usCanChgerOnTick'
	// 4)modified external can id 0x0x1806E5F4 sending data logic
	// 5)modified heating start & stop logic
	// V3.3.7.2->3.3.7.3, h00205922, 2024.04.01
	// 1)add soc calibration history alarm & data save
	// 2)charge parallel bluetooth name to id calculate method, support bluetooth name end with 'A'~'F'
	// 3)debuged parallel can frame deviceId
	// 4)modified heating start & stop logic
	// V3.3.7.3->3.3.7.4, h00205922, 2024.04.02
	// 1)modified heating threashoulds
	// 2)add dog feeding in large flash RW operation case
	// 3)add MOS temp. power limit mapping table line
	// 4)add BMS global state eBStatSOCRecalc
	// 5)move MOS Abnormality trigger logic to the end of protection process
	// 6)change cell 2.3V and 2.5V under voltage protection trigger timeout threshould
	// 7)add canid 0xE013, 0xE014, 0xE015, 0x1E1, 0x1F5, 0x800A6A9, 0x1000A6A9, 0x1C00A6A9 frame
	// 8)fix parallel soc calculation bug
	// 9)support parallel consult in running state
	// V3.3.7.4->V3.3.7.5, h00205922, 2024.04.03
	// 1)fix heating logic bug
	// 2)remove unnecessary soc calibration history alarm & data save
	// 3)change parallel data sending period from 50ms to 500ms
	// 4)fix parallel init section bug, when slave bms self id change from 0 (default) to 1, rtv data lost
	// 5)fix init section bug in main.c, when soc<1% and all sensor should be ok, then do soc calibration
	// V3.3.7.5->V3.3.7.6, h00205922, 2024.04.04
	// 1)change SC threshould from 400A/0us to 1000A/960us
	// V3.3.7.6->V3.3.7.7, h00205922, 2024.04.04
	// 2)reduce -3 of MOS. temp.
	// V3.3.7.7->V3.3.7.8, h00205922, 2024.04.05
	// 1)delay 500ms between his_data_write() and system_reset()
	// 2)release the code that if run first time or read history data error, set realAh, leftAh, soc, cycleCnt to default
	// 3)fix bug : before main loop history data saving probably do not has g_stEcoRtv inited, if then, history data will save uninited g_stEcoRtv
	// 4)add function : in pararell state, master node take slave node sleep together
	// V3.3.7.8->V3.3.7.9, h00205922, 2024.04.08
	// 1)fix heater run & stop logic bug
	// V3.3.7.9->V3.3.8.0, h00205922, 2024.04.08
	// 1)fix bug: parallel mode, slave MOS control logic, every node should judge heating or not by itself
	// 2)limit max. parallel node number to 2
	// 3)modified parallel slave mode data share receive & send
	// V3.3.8.0->V3.3.8.1, h00205922, 2024.04.08
	// 1)delete request charge current modification in can id 0x1806E5F4 frame
	// 2)fix bug : stop heating logic
	// V3.3.8.1->V3.3.8.2, h00205922, 2024.04.11
	// 1)adjust CAN prescaler, seg1, seg2 parameters
	// 2)remove parallel consult in init section
	// V3.3.8.2->V3.3.8.3, h00205922, 2024.04.11
	// 1)recover parallel consult in init section
	// V3.3.8.3->V3.3.8.4, h00205922, 2024.04.12
	// 1)redesigne MOS. control logic & parallel SOP
	// V3.3.8.4->V3.3.8.5, h00205922, 2024.04.16
	// 1)fix state machine bug
	// 2)remove parallel refered
	// V3.3.8.5->V3.3.8.6, h00205922, 2024.04.19
	// 1)fix soc calibration history save bug
	// V3.3.8.6->V3.3.8.7, h00205922, 2024.04.19
	// 1)fix soc empty or full calibration keeping write flash bug
	// V3.3.8.7->V3.3.8.8, h00205922, 2024.04.19
	// 1)keep sending request voltage & current whenever charger is connected or not
	// V3.3.8.8->V3.3.8.9, h00205922, 2024.04.19
	// 1)recover heat status heating wait, heating, heating end
	// 2)change button wake & sleep logic
	// 3)fix some CAN frame bugs
	// V3.3.8.9->V3.3.9.0, h00205922, 2024.04.20
	// 1)fix bug in BSP_WK_Detection()
	// V3.3.9.0->V3.3.9.1, h00205922, 2024.04.21
	// 1)shutdown CMOS & DMOS before reset
	// V3.3.9.1->V3.3.9.2, h00205922, 2024.04.21
	// 1)recover parallel can frame sending & receiving
	// V3.3.9.2->V3.3.9.3, h00205922, 2024.04.22
	// 1)fix can frame reply bug
	// V3.3.9.3->V3.3.9.4, h00205922, 2024.04.22
	// 1)keep all soc sent by can to exteranl device is round interger type to make sure meter & touch pannel shows same soc
	// V3.3.9.4->V3.3.9.5, h00205922, 2024.04.23
	// 1)fix MOS. state machine bug
	// V3.3.9.5->V3.3.9.6, h00205922, 2024.04.23
	// 1)change temperature difference stop heating from 15 to 10
	// 2)change UCTRVThr from 0 to 1
	// 3)change UCTRTThr from 0 to 30000
	// 4)change UCTTTThr from 5000 to 30000
	// 5)change heating flow delay time
	// V3.3.9.6->V3.3.9.7, h00205922, 2024.04.24
	// 1)fix heating state machine bug
	// V3.3.9.8->V3.3.9.9, h00205922, 2024.05.01
	// 1)CAN ID 0xE0015 frame data byte[7] send 0x03, means model GEN3
	g_stCfg.stLocal.ucSVerVMajor = 3;
	g_stCfg.stLocal.ucSVerVMinor = 5;
	g_stCfg.stLocal.ucSVerRMajor = 6;
	g_stCfg.stLocal.ucSVerRMinor = 0;//电池容量型号变化
	g_stCfg.stOta.uiAddr = 0;
	g_stCfg.stOta.uiLen = 0;
	g_stCfg.stOta.ucUpdate = 0;
	cfg_save();
	CFG_DEBUG("Software version : %d.%d.%d\n", g_stCfg.stLocal.ucSVerVMajor, g_stCfg.stLocal.ucSVerVMinor, g_stCfg.stLocal.ucSVerRMinor);
	CFG_RETURN_TRUE;
}
