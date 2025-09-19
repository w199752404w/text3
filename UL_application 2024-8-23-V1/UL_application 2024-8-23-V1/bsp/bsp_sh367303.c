/**
  ******************************************************************************
  * @file    bsp_sh367309.c
  * @author  tangderong
  * @version V1.0
  * @date    2023-05-23
  * @brief   Using I2C to read and write Zhongying SH367309 chip
  ******************************************************************************
  */

#include "bsp_sh367303.h"
#include "bsp_i2c.h"
#include "config.h"
#include "local.h"
#include "bsp_gpio.h"
#include "systick.h"
#include "parallel.h"
#include "main.h"

#include <stdio.h>
#include <string.h>

AFE_S g_stAfe;

uint8_t g_crc8Table[] = {
	0x00,0x07,0x0E,0x09,0x1C,0x1B,0x12,0x15,0x38,0x3F,0x36,0x31,0x24,0x23,0x2A,0x2D,
	0x70,0x77,0x7E,0x79,0x6C,0x6B,0x62,0x65,0x48,0x4F,0x46,0x41,0x54,0x53,0x5A,0x5D,
	0xE0,0xE7,0xEE,0xE9,0xFC,0xFB,0xF2,0xF5,0xD8,0xDF,0xD6,0xD1,0xC4,0xC3,0xCA,0xCD,
	0x90,0x97,0x9E,0x99,0x8C,0x8B,0x82,0x85,0xA8,0xAF,0xA6,0xA1,0xB4,0xB3,0xBA,0xBD,
	0xC7,0xC0,0xC9,0xCE,0xDB,0xDC,0xD5,0xD2,0xFF,0xF8,0xF1,0xF6,0xE3,0xE4,0xED,0xEA,
	0xB7,0xB0,0xB9,0xBE,0xAB,0xAC,0xA5,0xA2,0x8F,0x88,0x81,0x86,0x93,0x94,0x9D,0x9A,
	0x27,0x20,0x29,0x2E,0x3B,0x3C,0x35,0x32,0x1F,0x18,0x11,0x16,0x03,0x04,0x0D,0x0A,
	0x57,0x50,0x59,0x5E,0x4B,0x4C,0x45,0x42,0x6F,0x68,0x61,0x66,0x73,0x74,0x7D,0x7A,
	0x89,0x8E,0x87,0x80,0x95,0x92,0x9B,0x9C,0xB1,0xB6,0xBF,0xB8,0xAD,0xAA,0xA3,0xA4,
	0xF9,0xFE,0xF7,0xF0,0xE5,0xE2,0xEB,0xEC,0xC1,0xC6,0xCF,0xC8,0xDD,0xDA,0xD3,0xD4,
	0x69,0x6E,0x67,0x60,0x75,0x72,0x7B,0x7C,0x51,0x56,0x5F,0x58,0x4D,0x4A,0x43,0x44,
	0x19,0x1E,0x17,0x10,0x05,0x02,0x0B,0x0C,0x21,0x26,0x2F,0x28,0x3D,0x3A,0x33,0x34,
	0x4E,0x49,0x40,0x47,0x52,0x55,0x5C,0x5B,0x76,0x71,0x78,0x7F,0x6A,0x6D,0x64,0x63,
	0x3E,0x39,0x30,0x37,0x22,0x25,0x2C,0x2B,0x06,0x01,0x08,0x0F,0x1A,0x1D,0x14,0x13,
	0xAE,0xA9,0xA0,0xA7,0xB2,0xB5,0xBC,0xBB,0x96,0x91,0x98,0x9F,0x8A,0x8D,0x84,0x83,
	0xDE,0xD9,0xD0,0xD7,0xC2,0xC5,0xCC,0xCB,0xE6,0xE1,0xE8,0xEF,0xFA,0xFD,0xF4,0xF3
};

void afe_delay(uint16_t ms) {
	uint32_t i = 5000 * ms;
	while(i--) {
		__nop();__nop();__nop();__nop();
		__nop();__nop();__nop();
	}
}

/*******************************************************************************
* Function Name  : CRC8Cal
* Description    : CRC8校验
* Input          : pucBuf, 数组首地址
*                  ucLength, 校验长度
* Output         : None
* Return         : crc8校验值
*******************************************************************************/
uint8_t CRC8Cal(const uint8_t* pucBuf, uint8_t ucLength) {    		   //look-up table calculte CRC
	uint8_t ucCrc = 0;
	
	for(; ucLength > 0; ucLength--) {
		ucCrc = g_crc8Table[ucCrc ^ *pucBuf];
		pucBuf++;
	}
	return ucCrc;
}

/*******************************************************************************
* Function Name  : IIC309_ReadData
* Description    : 向从机读取数据
* Input          : ucDevAdd, 从机设备地址
*				   				 ucRegAdd, 内存器地址
*				   				 ucDatLen, 写入数据长度
* Output         : pucData, 写入数据缓存器首地址
* Return         : result, 1写成功 0写失败
*******************************************************************************/
bool IIC303_ReadData(uint8_t ucDevAdd, uint8_t ucRegAdd, uint8_t* const pucData){
	if(0 == pucData) {
		AFE303_RETURN_FALSE;
	}
    I2C_SCL_0();
    I2C_SDA_0();
    delay_1ms(1);
    
	i2c_Start();
	if(!i2c_SendByte(0x36)) {
		i2c_Stop();
		AFE303_RETURN_FALSE;
	}
	if(!i2c_SendByte(ucRegAdd)) {
		i2c_Stop();
		AFE303_RETURN_FALSE;
	}

	i2c_Start();
	if(!i2c_SendByte((ucDevAdd << 1) | 0x01)) {
		i2c_Stop();
		AFE303_RETURN_FALSE;
	}
    uint8_t aucTempBuf[5];	
    aucTempBuf[0] = ucDevAdd << 1;
    aucTempBuf[1] = ucRegAdd;
    aucTempBuf[2] = (ucDevAdd << 1) | 0x01;
	
    for(uint8_t i=0;i<2;i++) {
		aucTempBuf[3+i] = i2c_ReadByte();
		i2c_Ack();
	}
    uint8_t ucCrc = i2c_ReadByte();
    i2c_NoAck();
	i2c_Stop();
    if(ucCrc != CRC8Cal(aucTempBuf, 3 + 2)) {		//CRC verification is performed on the read data
		AFE303_RETURN_FALSE;
	}
    for(uint8_t i=0;i<2;i++) {
		pucData[i] = aucTempBuf[3+i];										//Read the value of IIC
	}
	
	AFE303_RETURN_TRUE;

}


/*******************************************************************************
* Function Name  : IIC303_WriteData
* Description    : 向从机写数据
* Input          : ucDevAdd, 从机设备地址
*				   				 ucRegAdd, 内存器地址
*				   				 ucData, 写入数据
* Output         : None
* Return         : result ：1写成功 0写失败
*******************************************************************************/
bool IIC303_WriteData(uint8_t ucDevAdd, uint8_t ucRegAdd, uint8_t ucData) {	
	i2c_Start();
	if(!i2c_SendByte(ucDevAdd << 1)) {
		i2c_Stop();
		AFE303_RETURN_FALSE;
	}
	if(!i2c_SendByte(ucRegAdd)) {
		i2c_Stop();
		AFE303_RETURN_FALSE;
	}
	if(!i2c_SendByte(ucData)) {							//写入成功后写下一个数据
		i2c_Stop();
		AFE303_RETURN_FALSE;
	}
	uint8_t aucTempBuf1[4];												//用于存储数据进行CRC8校验
	aucTempBuf1[0] = ucDevAdd << 1;
	aucTempBuf1[1] = ucRegAdd;
	aucTempBuf1[2] = ucData;
	aucTempBuf1[3] = CRC8Cal(aucTempBuf1, 3);			//对要发送的数据进行CRC8校验
	if(!i2c_SendByte(aucTempBuf1[3])) {
		i2c_Stop();
		AFE303_RETURN_FALSE;
	}
	i2c_Stop();
    afe_delay(1);				

	AFE303_RETURN_TRUE;
}

/*******************************************************************************
* Function Name  : MTP_Write309
* Description    : 向从机写数据
* Input          : ucDevAdd, 从机设备地址写
*				   ucRegAdd, 内存器地址
*				   pucData, 写入数据缓存器首地址
*				   ucDatLen, 写入数据长度
*				   ucCount, 容错次数
* Output         : None
* Return         : result ：1写成功 0写失败
*******************************************************************************/
bool MTP_Write303(uint8_t ucDevAdd, uint8_t ucRegAdd, uint8_t* const pucData, uint8_t ucDatLen, uint8_t ucCount) {
	for(uint8_t i=0;i<ucDatLen;i++) {
		if(!IIC303_WriteData(ucDevAdd, ucRegAdd, pucData[i])) {			//判断写数据是否成功
			ucCount--;
			if(0 == ucCount) {
				AFE303_RETURN_FALSE;
			}
			afe_delay(1);
			continue;				//增加程序容错率
		}
	}
	AFE303_RETURN_TRUE;
}

/*******************************************************************************
* Function Name  : MTP_Read309
* Description    : 向从机读数据
* Input          : ucDevAdd, 从机设备地址读
*				   				 ucRegAdd, 内存器地址
*				  			     ucDatLen, 写入数据长度
*                                ucCount, 容错次数
* Output         : pucData, 写入数据缓存器首地址
* Return         : result ：1写成功 0写失败
*******************************************************************************/
bool MTP_Read303(uint8_t ucDevAdd, uint8_t ucRegAdd, uint8_t* pucData, uint8_t ucDatLen, uint8_t ucCount) {
//	while(ucCount--) {																													//延时1ms重复写
//		if(IIC303_ReadData(ucDevAdd, ucRegAdd, pucData)) {				//判断写数据是否成功
//			AFE303_RETURN_TRUE;
//		}
//		//HAL_Delay(1);
//		afe_delay(1);
//	}
//    AFE303_RETURN_TRUE;
    
    uint8_t aucData[2];
    for(uint8_t i=0;i<((ucDatLen+1)/2);i++) {
		if(!IIC303_ReadData(ucDevAdd, ucRegAdd+i*2, aucData)) {			//判断读数据是否成功
			ucCount--;
			if(0 == ucCount) {
				AFE303_RETURN_FALSE;
			}
			afe_delay(1);
			continue;				//增加程序容错率
		}else{
            pucData[2*i] = aucData[0]; 
            pucData[2*i+1] = aucData[1]; 
        }
	}
    AFE303_RETURN_TRUE;
//if(ucRegAdd != 0){
////    if(!IIC303_ReadData(ucDevAdd, 0x0E, aucData)){
////        AFE303_RETURN_FALSE;
////    }
////    pucData[0] = aucData[0];
////    pucData[1] = aucData[1];
////    afe_delay(30);
//    
//    //四串采集寄存器10-17
//    aucData[0] = 0;
//    aucData[1] = 0;
//    if(!IIC303_ReadData(ucDevAdd, 0x10, aucData)){
//        AFE303_RETURN_FALSE;
//    }
//    pucData[2] = aucData[0];
//    pucData[3] = aucData[1];
//    delay_1ms(10);
//  //  afe_delay(500);
////    aucData[0] = 0;
////    if(!MTP_Write303(AFE_ID, 0x04, aucData, 1, 5)) {
////        AFE303_RETURN_FALSE;
////    }
////    afe_delay(5);
////         aucData[0] = 3;
////    if(!MTP_Write303(AFE_ID, 0x04, aucData, 1, 5)) {
////        AFE303_RETURN_FALSE;
////    }
////    afe_delay(5);
////    aucData[0] = 4;
////    if(!MTP_Write303(AFE_ID, 0x05, aucData, 1, 5)) {
////        AFE303_RETURN_FALSE;
////    }
////    afe_delay(5);
//    
//    
//    aucData[0] = 0;
//    aucData[1] = 0;
//    if(!IIC303_ReadData(ucDevAdd, 0x12, aucData)){
//        AFE303_RETURN_FALSE;
//    }
//    pucData[2] = aucData[0];
//    pucData[3] = aucData[1];
//    afe_delay(5);
//    aucData[0] = 0;
////    if(!MTP_Write303(AFE_ID, 0x04, aucData, 1, 5)) {
////        AFE303_RETURN_FALSE;
////    }
////    afe_delay(5);
//    if(!IIC303_ReadData(ucDevAdd, 0x14, aucData)){
//        AFE303_RETURN_FALSE;
//    }
//    pucData[4] = aucData[0];
//    pucData[5] = aucData[1];
//    afe_delay(5);
//    if(!IIC303_ReadData(ucDevAdd, 0x16, aucData)){
//        AFE303_RETURN_FALSE;
//    }
//    pucData[6] = aucData[0];
//    pucData[7] = aucData[1];
//    afe_delay(5);
//    //IIC303_ReadData(ucDevAdd,0x12, aucData);
//	AFE303_RETURN_TRUE;
//}else{
//    if(!IIC303_ReadData(ucDevAdd, 0, aucData)){
//        AFE303_RETURN_FALSE;
//    }
//    pucData[0] = aucData[0];
//    pucData[1] = aucData[1];
//AFE303_RETURN_TRUE;
//}

}


/* 热敏电阻表, -50~110℃的温度, 单位kΩ, 不同型号热敏电阻数据不同, 需要根据实际产品手册更新该表, 现在的阻值可能不对 */ /*对应的是3435型号*/
float fNTC103AT[161] = {329.50,	309.70,	291.24,	274.04,	257.99,	247.70,	233.35,	219.95,	207.43,	195.72,	/* -50~-41℃ */
												188.50,	177.96,	168.10,	158.85,	150.19,	144.10,	136.33,	129.04,	122.19,	115.76,	/* -40~-31℃ */
												111.30,	105.50,	100.05,	94.92,	90.09,	86.43,	82.08,	77.98,	74.12,	70.48,	/* -30~-21℃ */
												67.77,	64.47,	61.36,	58.42,	55.64,	53.41,	50.90,	48.52,	46.27,	44.14,	/* -20~-11℃ */
												42.47,	40.53,	38.70,	36.96,	35.32,	33.90,	32.41,	30.99,	29.64,	28.36,	/* -10~-1℃ */
												27.28,	26.11,	25.01,	23.95,	22.95,	22.05,	21.14,	20.27,	19.44,	18.66,	/* 0~9℃ */
												17.96,	17.24,	16.55,	15.90,	15.27,	14.69,	14.12,	13.57,	13.05,	12.56,	/* 10~19℃ */
												12.09,	11.63,	11.20,	10.78,	10.38,	10.00,	9.63,		9.28,		8.94,		8.62,		/* 20~29℃ */
												8.31,		8.01,		7.72,		7.45,		7.19,		6.94,		6.70,		6.46,		6.24,		6.03,		/* 30~39℃ */
												5.83,		5.62,		5.43,		5.25,		5.08,		4.91,		4.74,		4.59,		4.44,		4.30,		/* 40~49℃ */
												4.16,		4.02,		3.89,		3.77,		3.65,		3.54,		3.42,		3.31,		3.21,		3.11,		/* 50~59℃ */
												3.02,		2.92,		2.83,		2.75,		2.67,		2.59,		2.51,		2.43,		2.36,		2.29,		/* 60~69℃ */
												2.23,		2.16,		2.10,		2.04,		1.98,		1.92,		1.86,		1.81,		1.76,		1.71,		/* 70~79℃ */
												1.67,		1.62,		1.57,		1.53,		1.49,		1.45,		1.41,		1.37,		1.33,		1.30,		/* 80~89℃ */
												1.27,		1.23,		1.20,		1.17,		1.14,		1.11,		1.08,		1.05,		1.02,		1.00,		/* 90~99℃ */
												0.97,		0.95,		0.92,		0.90,		0.88,		0.86,		0.83,		0.81,		0.79,		0.78,		/* 100~109℃ */
												0.76};																																					/* 110℃ */

/*******************************************************************************
* Function Name  : afe_ohms2temp
* Description    : 通过查表发将电阻值转换为摄氏度
* Input          : fOhms, 温度对应的电阻值
* Output         : None
* Return         : result ：摄氏度值
*******************************************************************************/
float afe_ohms2temp(float fOhms) {
	if(fOhms >= fNTC103AT[0]) {
		return -50;
	}
	if(fOhms <= fNTC103AT[160]) {
		return 110;
	}
	for(uint8_t i=0;i<160;i++) {
		if(fOhms <= fNTC103AT[i] && fOhms >= fNTC103AT[i + 1]) {
			return i + (fNTC103AT[i] - fOhms ) / (fNTC103AT[i] - fNTC103AT[i+1]) - 50;
		}
	}
	return -273;
}

/*******************************************************************************
* Function Name  : afe_temp2ohms
* Description    : 通过查表发将摄氏度转换为电阻值
* Input          : fTemp, 电阻值对应的温度
* Output         : None
* Return         : result ：电阻值
*******************************************************************************/
float afe_temp2ohms(float fTemp) {
	if(fTemp <= -50) {
		return fNTC103AT[0];
	}
	if(fTemp >= 110) {
		return fNTC103AT[160];
	}
	return fNTC103AT[(int8_t)fTemp + 50] + (fNTC103AT[(int8_t)fTemp + 51] - fNTC103AT[(int8_t)fTemp + 50]) * (fTemp - (int8_t)fTemp);
}

/*******************************************************************************
* Function Name  : afe_calibrate
* Description    : Calibrated according to standard temperature, voltage, and current
* Input          : usMask, masks, where different bits represent different calibration enabled or not
* Output         : None
* Return         : result ：1 write succeeded, 0 write failed
*******************************************************************************/
bool afe_calibrate(uint16_t usMask) {
	if(usMask & 0x0001) {	/* The temperature drift coefficient is calibrated, and the standard temperature is 25°C*/
		for(uint8_t i=0;i<CFG_TMP_NUM;i++) {
			if(g_stAfe.stRamApp.fTS[i] < 10 || g_stAfe.stRamApp.fTS[i] > 40) {
				continue;
			}
			g_stCfg.stAfe.afTempCali[i] += (25 - g_stAfe.stRamApp.fTS[i]);
		}
		g_stCfg.stAfe.fAvgTempCali = 0;
		for(uint8_t i=0;i<CFG_TMP_NUM;i++) {
			g_stCfg.stAfe.fAvgTempCali += g_stCfg.stAfe.afTempCali[i];
		}
		g_stCfg.stAfe.fAvgTempCali /= CFG_TMP_NUM;
	}
	if(usMask & 0x0002) {	/* CUR drift coefficient calibration */
		g_stCfg.stAfe.fCurCaliB -= g_stAfe.stRamApp.fCUR;
	}
	if(usMask & 0x0004) {	/* CUR scale factor calibration, standard current 100000mA */
		if(0 == g_stCfg.stAfe.fCurCaliA) {
			g_stCfg.stAfe.fCurCaliA = 1;
		}
		g_stCfg.stAfe.fCurCaliA /= ((g_stAfe.stRamApp.fCUR - g_stCfg.stAfe.fCurCaliB) / (1 + (g_stLocalPackRVal[g_stPrl.ucSelfId].astPackReport.fMosT - 25) * 0.00005) / 100000);
	}
//	if(usMask & 0x0008) {	/* CDATA drift coefficient calibration */
//		g_stCfg.stAfe.fCDATACaliB -= g_stAfe.stRamApp.fCDATA;
//	}
	if(usMask & 0x0010) {	/* CDATA scale coefficient calibration, standard current 100000mA */
//		if(0 == g_stCfg.stAfe.fCDATACaliA) {
//			g_stCfg.stAfe.fCDATACaliA = 1;
//		}
//		g_stCfg.stAfe.fCDATACaliA /= ((g_stAfe.stRamApp.fCDATA - g_stCfg.stAfe.fCDATACaliB) / (1 + (g_stLocalArrayRVal.astPackRVal[g_stPrl.ucSelfId].fMosT - 25) * 0.00005) / 100000);
	}
	if(usMask & 0x0020) {	/* Cell proportional coefficient calibration, standard voltage 3800mV */
		for(uint8_t i=0;i<CFG_CELL_NUM;i++) {
			if(0 == g_stCfg.stAfe.afCellVolCali[i]) {
				g_stCfg.stAfe.afCellVolCali[i] = 1;
			}
			g_stCfg.stAfe.afCellVolCali[i] /= (g_stAfe.stRamApp.fCELL[i] / 3800);
		}
	}
	if(usMask & 0x0040) {	/* The proportional coefficient of the whole string battery pack is calibrated, and the standard voltage is 3800*CN mV */
		float fVal = 0;
		for(uint8_t i=0;i<CFG_CELL_NUM;i++) {
			fVal += g_stAfe.stRamApp.fCELL[i];
		}
		if(0 == g_stCfg.stAfe.fPackVolCali) {
			g_stCfg.stAfe.fPackVolCali = 1;
		}
		g_stCfg.stAfe.fPackVolCali /= (fVal / 3800 / CFG_CELL_NUM);
	}
	return cfg_save();
}


/*******************************************************************************
* Function Name  : afe_get_rom
* Description    : Overwrite the ROM configuration of the AFE into memory
* Input          : None
* Output         : None
* Return         : result ：1 write succeeded, 0 write failed
*******************************************************************************/
bool afe_get_rom(void) {
    uint8_t aucByte[CFG_AFE_ROM_BLEN];
    if(!MTP_Read303(AFE_ID, eAfeRomByteStart, aucByte, CFG_AFE_ROM_BLEN, TRY_TIMES)) {
		AFE303_RETURN_FALSE;
	}
//    for(uint8_t i=0;i<CFG_AFE_ROM_BLEN/2;i++) {
//		g_stAfe.uRom.aucByte[i * 2] = aucByte[i * 2 + 1];
//		g_stAfe.uRom.aucByte[i * 2 + 1] = aucByte[i * 2];
//	}
    for(uint8_t i=0;i<CFG_AFE_ROM_BLEN;i++) {
    	g_stAfe.uRom.aucByte[i] = aucByte[i];
    }
    /*****显示AFE-ROM参数***************/
	AFE303_DEBUG("\r\n【****************AFE-ROM寄存器参数*********************】\r\n");
    for(uint8_t i = 0;i < 14;i++) {
		AFE303_DEBUG("[%02x]%02x\t",i,aucByte[i]);
	}
    AFE303_DEBUG("\r\n");
    AFE_ROM_CODE_S* pstCode = &g_stAfe.uRom.stCode;
    AFE_ROM_APP_S* pstApp = &g_stAfe.stRomApp;
	//CFG_AFE_S* pstCfg = &g_stCfg.stAfe;

    switch(pstCode->SCT){
        case 0:pstApp->usSCT = 50;break;
		case 1:pstApp->usSCT = 100;break;
		case 2:pstApp->usSCT = 300;break;
		case 3:pstApp->usSCT = 500;break;
		default:pstApp->usSCT = 0xFFFF;break;
    }
    
    switch(pstCode->SCV){
        case 0:pstApp->usSCV = 100;break;
		case 1:pstApp->usSCV = 200;break;
		case 2:pstApp->usSCV = 300;break;
		case 3:pstApp->usSCV = 400;break;
		default:pstApp->usSCV = 0xFFFF;break;
    }

    switch(pstCode->RST){
        case 0:pstApp->usRST = 16;break;
		case 1:pstApp->usRST = 32;break;
		case 2:pstApp->usRST = 128;break;
		case 3:pstApp->usRST = 1000;break;
		default:pstApp->usRST = 0xFFFF;break;
    }
    switch(pstCode->RSNS){
        case 0:pstApp->usRSNS = 400;break;
		case 1:pstApp->usRSNS = 200;break;
		case 2:pstApp->usRSNS = 100;break;
		case 3:pstApp->usRSNS = 50;break;
		default:pstApp->usRSNS = 0xFFFF;break;
    }
    switch(pstCode->WDTT){
        case 0:pstApp->usWDTT = 3000;break;
		case 1:pstApp->usWDTT = 1000;break;
		case 2:pstApp->usWDTT = 2000;break;
		case 3:pstApp->usWDTT = 500;break;
		default:pstApp->usWDTT = 0xFFFF;break;
    }
    switch(pstCode->CHS){
        case 0:pstApp->usCHS = 1400;break;
		case 1:pstApp->usCHS = 3000;break;
		case 2:pstApp->usCHS = 6000;break;
		case 3:pstApp->usCHS = 12000;break;
		default:pstApp->usCHS = 0xFFFF;break;
    }
    switch(pstCode->OVT){
        case 0:pstApp->usOVT = 1;break;
		case 1:pstApp->usOVT = 2;break;
		case 2:pstApp->usOVT = 4;break;
		case 3:pstApp->usOVT = 8;break;
        case 4:pstApp->usOVT = 16;break;
		case 5:pstApp->usOVT = 32;break;
		case 6:pstApp->usOVT = 64;break;
		case 7:pstApp->usOVT = 128;break;
		default:pstApp->usOVT = 0xFFFF;break;
    }
        
    pstApp->usOVD = ((pstCode->OVD1 << 8) + pstCode->OVD2)*5.86;
    pstApp->PIN =  pstCode->PIN;
    
    
    AFE303_RETURN_TRUE;
    
    
}

/*******************************************************************************
* Function Name  : afe_set_rom
* Description    : 将期望的配置与AFE的ROM进行对比, 如有不一致, 将内存中配置覆盖到AFE中
* Input          : None
* Output         : None
* Return         : result ：1写成功 0写失败
*******************************************************************************/
bool afe_set_rom(void) {
 	uint8_t aucByte[CFG_AFE_ROM_BLEN-3];
	if(!MTP_Read303(AFE_ID, eAfeRomByteRWStart, aucByte, CFG_AFE_ROM_BLEN - 3, 5)) {
		AFE303_RETURN_FALSE;
	}
    for(uint8_t i=0;i<(CFG_AFE_ROM_BLEN-3);i++) {
        if(g_stAfe.uRom.aucByte[3+i] != aucByte[i]){
            if(!MTP_Write303(AFE_ID, eAfeRomByteRWStart + i, g_stAfe.uRom.aucByte + 3 + i, 1, 5)) {
				AFE303_RETURN_FALSE;
            }
        }
        
        
        
        
//        if((g_stAfe.uRom.aucByte[3] != aucByte[0]) || (g_stAfe.uRom.aucByte[4] != aucByte[1])) {
//            if(!MTP_Write303(AFE_ID, eAfeRomByteRWStart + i, g_stAfe.uRom.aucByte + 3 + i, 1, 5)) {
//				AFE303_RETURN_FALSE;
//			}
//        }
    }
    AFE303_RETURN_TRUE;   
}

/*******************************************************************************
* Function Name  : afe_get_ram
* Description    : 将AFE的RAM配置覆盖到内存中
* Input          : None
* Output         : None
* Return         : result ：1写成功 0写失败
*******************************************************************************/
bool afe_get_ram(void) {
	uint8_t aucByte[AFE_RAM_BLEN];
	if(!MTP_Read303(AFE_ID, eAfeRamByteStart, aucByte, AFE_RAM_BLEN, TRY_TIMES)) {
		AFE303_RETURN_FALSE;
	}
	for(uint8_t i=0;i<AFE_RAM_BLEN/2;i++) {
		g_stAfe.uRam.aucByte[i * 2] = aucByte[i  * 2 + 1];
		g_stAfe.uRam.aucByte[i * 2 + 1] = aucByte[i * 2];
	}
	/****显示AFE-RAM参数****************/
//	AFE309_DEBUG("\r\n【***************AFE-RAM寄存器参数****************】\r\n");
//	for(uint8_t i = 0;i < AFE_RAM_BLEN;i++) {
//		printf("[%02x]%02x\t",(i+0x40),aucByte[i]);
//	}
//	printf("\r\n");
	AFE_RAM_CODE_S* pstCode = &g_stAfe.uRam.stCode;
	AFE_RAM_APP_S* pstApp = &g_stAfe.stRamApp;
	CFG_AFE_S* pstCfg = &g_stCfg.stAfe;
    for(uint8_t i=0;i<CFG_CELL_NUM;i++) {
		if(g_stCfg.stAfe.afCellVolCali[i] == 0) {
			pstApp->fCELL[i] = (pstCode->CELL[i+1] * 6 * 1000 / 4096);
		} else {
			pstApp->fCELL[i] = (pstCode->CELL[i+1] * 6 *1000 / 4096 * g_stCfg.stAfe.afCellVolCali[i]);
		}
	}
    //外部温度
    for(uint8_t i = 0 ; i < 2 ; i++) {
		pstApp->fTS[i] = afe_ohms2temp(pstCode->TS[i] * 10 / (4096 - pstCode->TS[i]) + pstCfg->afTempCali[i]);
	}
    //内部温度
    pstApp->fTEMP = 0.17 *pstCode->TEMP[0] - 270;
    
    
    //电流计算
    if(((pstCode->CUR >> 12) == 1)){						//放电
       pstCode->CUR = pstCode->CUR | 0xE000; 
    }
    switch(g_stAfe.stRomApp.usRSNS){
        case 50:
				pstApp->fCUR = (1000* (float)pstCode->CUR)/(65535 *LOCAL_CUR_RESIST);
				if(pstApp->fCUR> 0){
				pstApp->fCUR = pstApp->fCUR * g_stCfg.stAfe.fCurCaliA + g_stCfg.stAfe.fCurCaliB;
				
				}else{
				pstApp->fCUR = pstApp->fCUR * g_stCfg.stAfe.fCDATACaliA +g_stCfg.stAfe.fCurCaliB;
				
				}
            break;
        case 100:
            pstApp->fCUR = (1000* (float)pstCode->CUR)/(32768 *LOCAL_CUR_RESIST);
            break;
        case 200:
            pstApp->fCUR = (1000* (float)pstCode->CUR)/(16384 *LOCAL_CUR_RESIST);
            break;
        case 400:
            pstApp->fCUR = (1000* (float)pstCode->CUR)/(8192 *LOCAL_CUR_RESIST);
            break;
        default:
            break;
    }
    AFE303_RETURN_TRUE;
}

/*******************************************************************************
* Function Name  : afe_set_ram
* Description    : 将期望的配置与AFE的RAM进行对比, 如有不一致, 将内存中配置覆盖到AFE中
* Input          : None
* Output         : None
* Return         : result ：1写成功 0写失败
*******************************************************************************/
bool afe_set_ram(void) {
	/* 经过分析,都可以只读 */
	AFE303_RETURN_TRUE;
}


/*******************************************************************************
* Function Name  : afe_get_ai
* Description    : 按枚举读取变量
* Input          : usAddr,	枚举地址
* Output         : pfVal,	读取变量结果
* Return         : result, 1写成功 0写失败
*******************************************************************************/
bool afe_get_ai(uint16_t usAddr, float* pfVal) {
	if(0 == pfVal) {
		AFE303_RETURN_FALSE;
	}
	if(/*usAddr >= eAfeRomByteStart && */usAddr <= eAfeRomByteEnd) {
		*pfVal = g_stAfe.uRom.aucByte[usAddr - eAfeRomByteStart];
		AFE303_RETURN_TRUE;
	}
	if(usAddr >= eAfeRamByteStart && usAddr <= eAfeRamByteEnd) {
		*pfVal = g_stAfe.uRam.aucByte[usAddr - eAfeRamByteStart];
		AFE303_RETURN_TRUE;
	}
    
    if(usAddr >= eAfeRomCodeStart && usAddr <= eAfeRomCodeEnd) {
       switch(usAddr) {
            case eAfeRomCodeTWI:*pfVal = g_stAfe.uRom.stCode.TWI;break;
            case eAfeRomCodeWDT:*pfVal = g_stAfe.uRom.stCode.WDT;break;
            case eAfeRomCodeOV:*pfVal = g_stAfe.uRom.stCode.OV;break;
            case eAfeRomCodeSC:*pfVal = g_stAfe.uRom.stCode.SC;break;
            case eAfeRomCodeCHGR:*pfVal = g_stAfe.uRom.stCode.CHGR;break;
            case eAfeRomCodeLOAD:*pfVal = g_stAfe.uRom.stCode.LOAD;break;
            case eAfeRomCodeCHGING:*pfVal = g_stAfe.uRom.stCode.CHGING;break;
            case eAfeRomCodeDSGING:*pfVal = g_stAfe.uRom.stCode.DSGING;break;
            case eAfeRomCodeCHG:*pfVal = g_stAfe.uRom.stCode.CHG;break;
            case eAfeRomCodeDSG:*pfVal = g_stAfe.uRom.stCode.DSG;break;
            case eAfeRomCodeTWI_INT:*pfVal = g_stAfe.uRom.stCode.TWI_INT;break;
            case eAfeRomCodeWDT_INT:*pfVal = g_stAfe.uRom.stCode.WDT_INT;break;
            case eAfeRomCodeVADC_INT:*pfVal = g_stAfe.uRom.stCode.VADC_INT;break;
            case eAfeRomCodeCADC_INT:*pfVal = g_stAfe.uRom.stCode.SCVADC_INT;break;
            case eAfeRomCodeCD_INT:*pfVal = g_stAfe.uRom.stCode.CD_INT;break;
            case eAfeRomCodeOV_INT:*pfVal = g_stAfe.uRom.stCode.OV_INT;break;
            case eAfeRomCodeSC_INT:*pfVal = g_stAfe.uRom.stCode.SC_INT;break;
            case eAfeRomCodeCHGR_EN:*pfVal = g_stAfe.uRom.stCode.CHGR_EN;break;
            case eAfeRomCodeLOAD_EN:*pfVal = g_stAfe.uRom.stCode.LOAD_EN;break;
            case eAfeRomCodeOV_EN:*pfVal = g_stAfe.uRom.stCode.OV_EN;break;
            case eAfeRomCodeSC_EN:*pfVal = g_stAfe.uRom.stCode.SC_EN;break;
            case eAfeRomCodeWDT_EN:*pfVal = g_stAfe.uRom.stCode.WDT_EN;break;
            case eAfeRomCodePD_EN:*pfVal = g_stAfe.uRom.stCode.PD_EN;break;
            case eAfeRomCodeCTLD_EN:*pfVal = g_stAfe.uRom.stCode.CTLD_EN;break;
            case eAfeRomCodeLTCLR:*pfVal = g_stAfe.uRom.stCode.LTCLR;break;
            case eAfeRomCodeCHG_C:*pfVal = g_stAfe.uRom.stCode.CHG_G;break;
            case eAfeRomCodeDSG_C:*pfVal = g_stAfe.uRom.stCode.DSG_G;break;
            case eAfeRomCodeALARM_C:*pfVal = g_stAfe.uRom.stCode.ALARM_C;break;
            case eAfeRomCodeRESET_PF:*pfVal = g_stAfe.uRom.stCode.RESET_PT;break;
            case eAfeRomCodeSCAN_C:*pfVal = g_stAfe.uRom.stCode.SCAN_C;break;
            case eAfeRomCodeVADC_C:*pfVal = g_stAfe.uRom.stCode.VADC_C;break;
            case eAfeRomCodeVADC_EN:*pfVal = g_stAfe.uRom.stCode.VADC_EN;break;
            case eAfeRomCodeCBIT_C:*pfVal = g_stAfe.uRom.stCode.CBIT_C;break;
            case eAfeRomCodeCADC_M:*pfVal = g_stAfe.uRom.stCode.CADC_M;break;
            case eAfeRomCodeCADC_EN:*pfVal = g_stAfe.uRom.stCode.CADC_EN;break;
            case eAfeRomCodeCB6:*pfVal = g_stAfe.uRom.stCode.CB6;break;
            case eAfeRomCodeCB7:*pfVal = g_stAfe.uRom.stCode.CB7;break;
            case eAfeRomCodeCB8:*pfVal = g_stAfe.uRom.stCode.CB8;break;
            case eAfeRomCodeCB9:*pfVal = g_stAfe.uRom.stCode.CB9;break;
            case eAfeRomCodeCB10:*pfVal = g_stAfe.uRom.stCode.CB10;break;
            case eAfeRomCodeCB1:*pfVal = g_stAfe.uRom.stCode.CB1;break;
            case eAfeRomCodeCB2:*pfVal = g_stAfe.uRom.stCode.CB2;break;
            case eAfeRomCodeCB3:*pfVal = g_stAfe.uRom.stCode.CB3;break;
            case eAfeRomCodeCB4:*pfVal = g_stAfe.uRom.stCode.CB4;break;
            case eAfeRomCodeCB5:*pfVal = g_stAfe.uRom.stCode.CB5;break;
            case eAfeRomCodeSCT:*pfVal = g_stAfe.uRom.stCode.SCT;break;
            case eAfeRomCodeSCV:*pfVal = g_stAfe.uRom.stCode.SCV;break;
            case eAfeRomCodeRST:*pfVal = g_stAfe.uRom.stCode.RST;break;
            case eAfeRomCodeRSNS:*pfVal = g_stAfe.uRom.stCode.RSNS;break;
            case eAfeRomCodeWDTT:*pfVal = g_stAfe.uRom.stCode.WDTT;break;
            case eAfeRomCodeCHS:*pfVal = g_stAfe.uRom.stCode.CHS;break;
            case eAfeRomCodeOVT:*pfVal = g_stAfe.uRom.stCode.OVT;break;
            case eAfeRomCodeOVD1:*pfVal = g_stAfe.uRom.stCode.OVD1;break;
            case eAfeRomCodeOVD2:*pfVal = g_stAfe.uRom.stCode.OVD2;break;
            case eAfeRomCodePIN:*pfVal = g_stAfe.uRom.stCode.PIN;break;
			default:AFE303_RETURN_FALSE;
		}
		AFE303_RETURN_TRUE; 
    }
	if(usAddr >= eAfeRamCodeStart && usAddr <= eAfeRamCodeEnd) {
		switch(usAddr) {
            case eAfeRamCodeCELL1:*pfVal = g_stAfe.uRam.stCode.CELL[0];break;
            /* CELL2 */
            case eAfeRamCodeCELL2:*pfVal = g_stAfe.uRam.stCode.CELL[1];break;
            /* CELL3 */
            case eAfeRamCodeCELL3:*pfVal = g_stAfe.uRam.stCode.CELL[2];break;
            /* CELL4 */
            case eAfeRamCodeCELL4:*pfVal = g_stAfe.uRam.stCode.CELL[3];break;
            /* TS1 */
            case eAfeRamCodeTS1:*pfVal = g_stAfe.uRam.stCode.TS[0];break;
            /* TS2 */
            case eAfeRamCodeTS2:*pfVal = g_stAfe.uRam.stCode.TS[1];break;
            /* TEMP1 */
            case eAfeRamCodeTEMP1:*pfVal = g_stAfe.uRam.stCode.TEMP[0];break;

			default:AFE303_RETURN_FALSE;
		}
		AFE303_RETURN_TRUE;
	}
	if(usAddr >= eAfeRomAppStart && usAddr <= eAfeRomAppEnd) {
		switch(usAddr) {
            
            case eAfeRomAppSCT:*pfVal = g_stAfe.stRomApp.usSCT;break;
            case eAfeRomAppSCV:*pfVal = g_stAfe.stRomApp.usSCV;break;
            case eAfeRomAppRST:*pfVal = g_stAfe.stRomApp.usRST;break;
            case eAfeRomAppRSNS:*pfVal = g_stAfe.stRomApp.usRSNS;break;
            case eAfeRomAppWDTT:*pfVal = g_stAfe.stRomApp.usWDTT;break;
            case eAfeRomAppCHS:*pfVal = g_stAfe.stRomApp.usCHS;break;
            case eAfeRomAppOVT:*pfVal = g_stAfe.stRomApp.usOVT;break;
            case eAfeRomAppOVD:*pfVal = g_stAfe.stRomApp.usOVD;break;
            case eAfeRomAppPIN:*pfVal = g_stAfe.stRomApp.PIN;break;
			default:AFE303_RETURN_FALSE;
		}
		AFE303_RETURN_TRUE;
	}
	if(usAddr >= eAfeRamAppStart && usAddr <= eAfeRamAppEnd) {
		switch(usAddr) {
            case eAfeRamCELL1:*pfVal = g_stAfe.stRamApp.fCELL[0];break;
			case eAfeRamCELL2:*pfVal = g_stAfe.stRamApp.fCELL[1];break;
			case eAfeRamCELL3:*pfVal = g_stAfe.stRamApp.fCELL[2];break;
			case eAfeRamCELL4:*pfVal = g_stAfe.stRamApp.fCELL[3];break;
            case eAfeRamTS1:*pfVal = g_stAfe.stRamApp.fTS[0];break;
			case eAfeRamTS2:*pfVal = g_stAfe.stRamApp.fTS[1];break;
            case eAfeRamTEMP1:*pfVal = g_stAfe.stRamApp.fTEMP;break;
			case eAfeRamCUR:*pfVal = g_stAfe.stRamApp.fCUR;break;
			default:AFE303_RETURN_FALSE;
		}
		AFE303_RETURN_TRUE;
	}
	AFE303_RETURN_FALSE;
}

/*******************************************************************************
* Function Name  : afe_set_ao
* Description    : 按枚举写入变量
* Input          : usAddr,	枚举地址
								 : ucCnt, 需要写入的变量个数
								 : pusVal,	需要写入的变量值
* Output         : None
* Return         : result, 1写成功 0写失败
*******************************************************************************/
bool afe_set_ao(uint16_t usAddr, uint8_t ucCnt, float* pfVal) {
	AFE303_DEBUG("usAddr=%d,ucCnt=%d,*pfVal=%f", usAddr, ucCnt, *pfVal);
	if(0 == pfVal) {
		AFE303_RETURN_FALSE;
	}
	for(uint16_t i=usAddr;i<(usAddr + ucCnt);i++) {
		if(i <= eAfeRomByteEnd - 1) {
			g_stAfe.uRom.aucByte[i - eAfeRomByteStart] = ((uint16_t)pfVal[i - usAddr] & 0xFF);
		} else if(i >= eAfeRamByteStart && i <= eAfeRamByteEnd) {
			g_stAfe.uRam.aucByte[i - eAfeRamByteStart] = ((uint16_t)pfVal[i - usAddr] & 0xFF);
		} else if(i >= eAfeRomCodeStart && i <= eAfeRomCodeEnd) {
            switch(i) {
                case eAfeRomCodeTWI:g_stAfe.uRom.stCode.TWI = (uint16_t)pfVal[i - usAddr];break;
                case eAfeRomCodeWDT:g_stAfe.uRom.stCode.WDT = (uint16_t)pfVal[i - usAddr];break;
                case eAfeRomCodeOV:g_stAfe.uRom.stCode.OV = (uint16_t)pfVal[i - usAddr];break;
                case eAfeRomCodeSC:g_stAfe.uRom.stCode.SC = (uint16_t)pfVal[i - usAddr];break;
                case eAfeRomCodeCHGR:g_stAfe.uRom.stCode.CHGR = (uint16_t)pfVal[i - usAddr];break;
                case eAfeRomCodeLOAD:g_stAfe.uRom.stCode.LOAD = (uint16_t)pfVal[i - usAddr];break;
                case eAfeRomCodeCHGING:g_stAfe.uRom.stCode.CHGING = (uint16_t)pfVal[i - usAddr];break;
                case eAfeRomCodeDSGING:g_stAfe.uRom.stCode.DSGING = (uint16_t)pfVal[i - usAddr];break;
                case eAfeRomCodeCHG:g_stAfe.uRom.stCode.CHG = (uint16_t)pfVal[i - usAddr];break;
                case eAfeRomCodeDSG:g_stAfe.uRom.stCode.DSG = (uint16_t)pfVal[i - usAddr];break;
                case eAfeRomCodeTWI_INT:g_stAfe.uRom.stCode.TWI_INT = (uint16_t)pfVal[i - usAddr];break;
                case eAfeRomCodeWDT_INT:g_stAfe.uRom.stCode.WDT_INT = (uint16_t)pfVal[i - usAddr];break;
                case eAfeRomCodeVADC_INT:g_stAfe.uRom.stCode.VADC_INT = (uint16_t)pfVal[i - usAddr];break;
                case eAfeRomCodeCADC_INT:g_stAfe.uRom.stCode.SCVADC_INT = (uint16_t)pfVal[i - usAddr];break;
                case eAfeRomCodeCD_INT:g_stAfe.uRom.stCode.CD_INT = (uint16_t)pfVal[i - usAddr];break;
                case eAfeRomCodeOV_INT:g_stAfe.uRom.stCode.OV_INT = (uint16_t)pfVal[i - usAddr];break;
                case eAfeRomCodeSC_INT:g_stAfe.uRom.stCode.SC_INT = (uint16_t)pfVal[i - usAddr];break;
                case eAfeRomCodeCHGR_EN:g_stAfe.uRom.stCode.CHGR_EN = (uint16_t)pfVal[i - usAddr];break;
                case eAfeRomCodeLOAD_EN:g_stAfe.uRom.stCode.LOAD_EN = (uint16_t)pfVal[i - usAddr];break;
                case eAfeRomCodeOV_EN:g_stAfe.uRom.stCode.OV_EN = (uint16_t)pfVal[i - usAddr];break;
                case eAfeRomCodeSC_EN:g_stAfe.uRom.stCode.SC_EN = (uint16_t)pfVal[i - usAddr];break;
                case eAfeRomCodeWDT_EN:g_stAfe.uRom.stCode.WDT_EN = (uint16_t)pfVal[i - usAddr];break;
                case eAfeRomCodePD_EN:g_stAfe.uRom.stCode.PD_EN = (uint16_t)pfVal[i - usAddr];break;
                case eAfeRomCodeCTLD_EN:g_stAfe.uRom.stCode.CTLD_EN = (uint16_t)pfVal[i - usAddr];break;
                case eAfeRomCodeLTCLR:g_stAfe.uRom.stCode.LTCLR = (uint16_t)pfVal[i - usAddr];break;
                case eAfeRomCodeCHG_C:g_stAfe.uRom.stCode.CHG_G = (uint16_t)pfVal[i - usAddr];break;
                case eAfeRomCodeDSG_C:g_stAfe.uRom.stCode.DSG_G = (uint16_t)pfVal[i - usAddr];break;
                case eAfeRomCodeALARM_C:g_stAfe.uRom.stCode.ALARM_C = (uint16_t)pfVal[i - usAddr];break;
                case eAfeRomCodeRESET_PF:g_stAfe.uRom.stCode.RESET_PT = (uint16_t)pfVal[i - usAddr];break;
                case eAfeRomCodeSCAN_C:g_stAfe.uRom.stCode.SCAN_C = (uint16_t)pfVal[i - usAddr];break;
                case eAfeRomCodeVADC_C:g_stAfe.uRom.stCode.VADC_C = (uint16_t)pfVal[i - usAddr];break;
                case eAfeRomCodeVADC_EN:g_stAfe.uRom.stCode.VADC_EN = (uint16_t)pfVal[i - usAddr];break;
                case eAfeRomCodeCBIT_C:g_stAfe.uRom.stCode.CBIT_C = (uint16_t)pfVal[i - usAddr];break;
                case eAfeRomCodeCADC_M:g_stAfe.uRom.stCode.CADC_M = (uint16_t)pfVal[i - usAddr];break;
                case eAfeRomCodeCADC_EN:g_stAfe.uRom.stCode.CADC_EN = (uint16_t)pfVal[i - usAddr];break;
                case eAfeRomCodeCB6:g_stAfe.uRom.stCode.CB6 = (uint16_t)pfVal[i - usAddr];break;
                case eAfeRomCodeCB7:g_stAfe.uRom.stCode.CB7 = (uint16_t)pfVal[i - usAddr];break;
                case eAfeRomCodeCB8:g_stAfe.uRom.stCode.CB8 = (uint16_t)pfVal[i - usAddr];break;
                case eAfeRomCodeCB9:g_stAfe.uRom.stCode.CB9 = (uint16_t)pfVal[i - usAddr];break;
                case eAfeRomCodeCB10:g_stAfe.uRom.stCode.CB10 = (uint16_t)pfVal[i - usAddr];break;
                case eAfeRomCodeCB1:g_stAfe.uRom.stCode.CB1 = (uint16_t)pfVal[i - usAddr];break;
                case eAfeRomCodeCB2:g_stAfe.uRom.stCode.CB2 = (uint16_t)pfVal[i - usAddr];break;
                case eAfeRomCodeCB3:g_stAfe.uRom.stCode.CB3 = (uint16_t)pfVal[i - usAddr];break;
                case eAfeRomCodeCB4:g_stAfe.uRom.stCode.CB4 = (uint16_t)pfVal[i - usAddr];break;
                case eAfeRomCodeCB5:g_stAfe.uRom.stCode.CB5 = (uint16_t)pfVal[i - usAddr];break;
                case eAfeRomCodeSCT:g_stAfe.uRom.stCode.SCT = (uint16_t)pfVal[i - usAddr];break;
                case eAfeRomCodeSCV:g_stAfe.uRom.stCode.SCV = (uint16_t)pfVal[i - usAddr];break;
                case eAfeRomCodeRST:g_stAfe.uRom.stCode.RST = (uint16_t)pfVal[i - usAddr];break;
                case eAfeRomCodeRSNS:g_stAfe.uRom.stCode.RSNS = (uint16_t)pfVal[i - usAddr];break;
                case eAfeRomCodeWDTT:g_stAfe.uRom.stCode.WDTT = (uint16_t)pfVal[i - usAddr];break;
                case eAfeRomCodeCHS:g_stAfe.uRom.stCode.CHS = (uint16_t)pfVal[i - usAddr];break;
                case eAfeRomCodeOVT:g_stAfe.uRom.stCode.OVT = (uint16_t)pfVal[i - usAddr];break;
                case eAfeRomCodeOVD1:g_stAfe.uRom.stCode.OVD1 = (uint16_t)pfVal[i - usAddr];break;
                case eAfeRomCodeOVD2:g_stAfe.uRom.stCode.OVD2 = (uint16_t)pfVal[i - usAddr];break;
                case eAfeRomCodePIN:g_stAfe.uRom.stCode.PIN = (uint16_t)pfVal[i - usAddr];break;

            }
        
        } else if(i >= eAfeRomAppStart && i <= eAfeRomAppEnd) {
			switch(i) {
                case eAfeRomAppSCT: 
                    if(50 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.SCT = 0;break;}
                    if(100 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.SCT = 1;break;}
                    if(300 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.SCT = 2;break;}
                    if(500 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.SCT = 3;break;}
                case eAfeRomAppSCV:
                    if(100 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.SCV = 0;break;}
                    if(200 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.SCV = 1;break;}
                    if(300 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.SCV = 2;break;}
                    if(400 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.SCV = 3;break;}
                case eAfeRomAppRST:
                    if(16 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.RST = 0;break;}
                    if(32 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.RST = 1;break;}
                    if(128 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.RST = 2;break;}
                    if(1000 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.RST = 3;break;}  
                case eAfeRomAppRSNS:
                    if(400 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.RSNS = 0;break;}
                    if(200 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.RSNS = 1;break;}
                    if(100 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.RSNS = 2;break;}
                    if(50 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.RSNS = 3;break;}  
                case eAfeRomAppWDTT:
                    if(30000 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.RSNS = 0;break;}
                    if(10000 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.RSNS = 1;break;}
                    if(2000 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.RSNS = 2;break;}
                    if(500 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.RSNS = 3;break;}  
                case eAfeRomAppCHS:
                    if(1400 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.RSNS = 0;break;}
                    if(3000 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.RSNS = 1;break;}
                    if(6000 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.RSNS = 2;break;}
                    if(12000 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.RSNS = 3;break;}  
                case eAfeRomAppOVT:
                    if(1 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.OVT = 0;break;}
                    if(2 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.OVT = 1;break;}
                    if(4 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.OVT = 2;break;}
                    if(8 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.OVT = 3;break;}  
                    if(16 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.OVT = 4;break;}
                    if(32 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.OVT = 5;break;}
                    if(64 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.OVT = 6;break;}
                    if(128 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.OVT = 7;break;}  
                case eAfeRomAppOVD:
                    g_stAfe.uRom.stCode.OVD1 = (uint16_t)(pfVal[i - usAddr])>>12;
                    g_stAfe.uRom.stCode.OVD2 = (uint16_t)(pfVal[i - usAddr])&0xFF;
                case eAfeRomAppPIN:
                    g_stAfe.uRom.stCode.PIN = (uint16_t)(pfVal[i - usAddr]);    
				default:AFE303_RETURN_FALSE;
			}
		} else if(i >= eAfeRamAppStart && i <= eAfeRamAppEnd) {

		}  else {
			AFE303_RETURN_FALSE;
		}
	}
    if(!afe_set_rom()) {
		AFE303_RETURN_FALSE;
	}
	if(!afe_set_ram()) {
		AFE303_RETURN_FALSE;
	}
    
    if(!afe_get_rom()) {
		AFE303_RETURN_FALSE;
	}
    
    
	AFE303_RETURN_TRUE;
}
    
/*******************************************************************************
* Function Name  : afe_get_di
* Description    : 按枚举和bit偏移量读取变量
* Input          : usAddr,	枚举地址 * 8 + bit偏移量
* Output         : pbVal,	读取变量结果
* Return         : result, 1写成功 0写失败
*******************************************************************************/
bool afe_get_di(uint16_t usAddr, bool* pbVal) {
	if(0 == pbVal) {
		AFE303_RETURN_FALSE;
	}
	if(/*usAddr >= eAfeRomByteStart && */usAddr <= eAfeRomByteEnd * 8) {                                              //change
		*pbVal = (bool)(g_stAfe.uRom.aucByte[usAddr / 8 - eAfeRomByteStart + 1 - usAddr / 8 % 2 * 2] >> (usAddr % 8) & 0x01);
		AFE303_RETURN_TRUE;
	}
	if(usAddr >= eAfeRamByteStart * 8 && usAddr <= eAfeRamByteEnd * 8) {                                             //change
		*pbVal = (bool)(g_stAfe.uRam.aucByte[usAddr / 8 - eAfeRamByteStart + 1 - usAddr / 8 % 2 * 2] >> (usAddr % 8) & 0x01);
		AFE303_RETURN_TRUE;
	}
	
	AFE303_RETURN_FALSE;
}

/*******************************************************************************
* Function Name  : afe_set_do
* Description    : 按枚举和bit偏移量写入变量
* Input          : usAddr,	枚举地址 * 8 + bit偏移量
								 : bVal,	需要写入的变量
* Output         : None
* Return         : result, 1写成功 0写失败
*******************************************************************************/
bool afe_set_do(uint16_t usAddr, bool bVal) {
	if(/*usAddr >= eAfeRamByteStart && */usAddr < eAfeRomByteEnd * 8) {
		if(0 == bVal) {
			g_stAfe.uRom.aucByte[usAddr / 8 - eAfeRomByteStart] &= ~(1 << (usAddr % 8));
		} else {
			g_stAfe.uRom.aucByte[usAddr / 8 - eAfeRomByteStart] |= (bVal << (usAddr % 8));
		}
	}
	if(usAddr >= eAfeRamByteStart * 8 && usAddr < eAfeRamByteEnd * 8) {
		if(0 == bVal) {
			g_stAfe.uRam.aucByte[usAddr / 8 - eAfeRamByteStart] &= ~(1 << (usAddr % 8));
		} else {
			g_stAfe.uRam.aucByte[usAddr / 8 - eAfeRamByteStart] |= (bVal << (usAddr % 8));  
		}
	}
	if(!afe_set_rom()) {
		AFE303_RETURN_FALSE;
	}
	if(!afe_set_ram()) {
		AFE303_RETURN_FALSE;
	}
	
	AFE303_RETURN_TRUE;
}




/*******************************************************************************
* Function Name  : afe_init
* Description    : Initialize SH367306 chip Initialize the I2C pin
* Input          : None
* Output         : None
* Return         : result ：1 write succeeded, 0 write failed
*******************************************************************************/
bool afe_init(void){
//    g_stCfg.stAfe.aucRomByte[1] = 0x50;
//	g_stCfg.stAfe.aucRomByte[0] = 0x0E;
//	g_stCfg.stAfe.aucRomByte[3] = 0x6B;   	
//	g_stCfg.stAfe.aucRomByte[2] = 0x00;
//	g_stCfg.stAfe.aucRomByte[5] = 0x00;   	
//	g_stCfg.stAfe.aucRomByte[4] = 0x00;
//	g_stCfg.stAfe.aucRomByte[7] = 0xD8;     //开启电流检测 连续采集 10bit VADC 温度 VADC转换周期50ms	
//	g_stCfg.stAfe.aucRomByte[6] = 0x00;  	
//	g_stCfg.stAfe.aucRomByte[9] = 0x00;
//	g_stCfg.stAfe.aucRomByte[8] = 0xFF;     //CADC范围0~50mv 复位MCU脉冲宽度1S 短路保护电压400mv 短路保护延时500us
//	g_stCfg.stAfe.aucRomByte[11] = 0x01;    //硬件过充保护 1个VADC周期 充电状态检测 1.4mv 看门狗溢出时间10S
//	g_stCfg.stAfe.aucRomByte[10] = 0xE1; 	
//	g_stCfg.stAfe.aucRomByte[12] = 0x42;	
    
    g_stCfg.stAfe.aucRomByte[1] = 0x00;
	g_stCfg.stAfe.aucRomByte[0] = 0x00;
	g_stCfg.stAfe.aucRomByte[2] = 0x00;
    g_stCfg.stAfe.aucRomByte[3] = 0x63;   	
    g_stCfg.stAfe.aucRomByte[4] = 0x00;
	g_stCfg.stAfe.aucRomByte[5] = 0x00;   	
	g_stCfg.stAfe.aucRomByte[6] = 0xD8;     //开启电流检测 连续采集 10bit VADC 温度 VADC转换周期50ms	
	g_stCfg.stAfe.aucRomByte[7] = 0x00;     
    g_stCfg.stAfe.aucRomByte[8] = 0x00;     
	g_stCfg.stAfe.aucRomByte[9] = 0xFF;     //CADC范围0~50mv 复位MCU脉冲宽度1S 短路保护电压400mv 短路保护延时500us
    g_stCfg.stAfe.aucRomByte[10] = 0x01; 
	g_stCfg.stAfe.aucRomByte[11] = 0x03;    //硬件过充保护 1个VADC周期 充电状态检测 1.4mv 看门狗溢出时间10S
	g_stCfg.stAfe.aucRomByte[12] = 0xFF;	
    g_stCfg.stAfe.aucRomByte[13] = 0x00;
	memcpy(g_stAfe.uRom.aucByte, g_stCfg.stAfe.aucRomByte, CFG_AFE_ROM_BLEN);
    if(!afe_set_rom()) {
		AFE303_RETURN_FALSE;
	}




//    uint8_t aucData[2] ;
//    aucData[0] = 0x63;
//    if(!MTP_Write303(AFE_ID, 0x03, aucData, 1, 5)) {
//        AFE303_RETURN_FALSE;
//    }
//    aucData[0] = 0x00;
//    if(!MTP_Write303(AFE_ID, 0x04, aucData, 1, 5)) {
//        AFE303_RETURN_FALSE;
//    }
//    if(!MTP_Write303(AFE_ID, 0x05, aucData, 1, 5)) {
//        AFE303_RETURN_FALSE;
//    }
//    aucData[0] = 0xF8;
//    if(!MTP_Write303(AFE_ID, 0x06, aucData, 1, 5)) {
//        AFE303_RETURN_FALSE;
//    }
//    aucData[0] = 0x00;
//    if(!MTP_Write303(AFE_ID, 0x07, aucData, 1, 5)) {
//        AFE303_RETURN_FALSE;
//    }
//    if(!MTP_Write303(AFE_ID, 0x08, aucData, 1, 5)) {
//        AFE303_RETURN_FALSE;
//    }
//    aucData[0] = 0xFF;
//    if(!MTP_Write303(AFE_ID, 0x09, aucData, 1, 5)) {
//        AFE303_RETURN_FALSE;
//    }
//     aucData[0] = 0x01;
//    if(!MTP_Write303(AFE_ID, 0x0A, aucData, 1, 5)) {
//        AFE303_RETURN_FALSE;
//    }
//    aucData[0] = 0x03;
//    if(!MTP_Write303(AFE_ID, 0x0B, aucData, 1, 5)) {
//        AFE303_RETURN_FALSE;
//    }
//    aucData[0] = 0xFF;
//    if(!MTP_Write303(AFE_ID, 0x0C, aucData, 1, 5)) {
//        AFE303_RETURN_FALSE;
//    }
//    aucData[0] = 0x00;
//    if(!MTP_Write303(AFE_ID, 0x0D, aucData, 1, 5)) {
//        AFE303_RETURN_FALSE;
//    }



	if(!afe_get_rom()) {
		AFE303_RETURN_FALSE;
	}
	if(!afe_get_ram()) {
		AFE303_RETURN_FALSE;
	}
	
	AFE303_RETURN_TRUE;
}

/*******************************************************************************
* Function Name  : afe_proc
* Description    : AFE所有服务处理, 该函数是在主循环中对AFE轮训的唯一接口
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
bool afe_proc(void) {
	static uint32_t uiTick = 0;
	uiTick++;
	if(uiTick == 1)
	{

		g_stCfg.stAfe.fCurCaliA =  0.9324;		//充电
		g_stCfg.stAfe.fCDATACaliA = 1.0330;		//放电
		g_stCfg.stAfe.fCurCaliB = 31891;
		cfg_save();
//		
	}
	if(uiTick * g_stCfg.stLocal.usCyclePeriod < 250) {
		AFE303_RETURN_TRUE;
	}
	uiTick = 0;
	if(!afe_get_ram()) {
		AFE303_RETURN_FALSE;
	}
	
	
	
	
	AFE303_RETURN_TRUE;
}

