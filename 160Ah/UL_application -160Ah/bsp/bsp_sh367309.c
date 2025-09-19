/**
  ******************************************************************************
  * @file    bsp_sh367309.c
  * @author  tangderong
  * @version V1.0
  * @date    2023-05-23
  * @brief   Using I2C to read and write Zhongying SH367309 chip
  ******************************************************************************
  */

#include "bsp_sh367309.h"
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
* Description    : CRC8 checksum
* Input          : pucBuf, The first address of the array
*									 ucLength, Check the length
* Output         : None
* Return         : CRC8 check value
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
* Description    : Read data from the slave
* Input          : ucDevAdd, The address of the slave device
*				   				 ucRegAdd, Memory address
*				   				 ucDatLen, The length of the written data
* Output         : pucData, Write to the first address of the data buffer
* Return         : result, 1 write succeeded, 0 write failed
*******************************************************************************/
bool IIC309_ReadData(uint8_t ucDevAdd, uint8_t ucRegAdd, uint8_t* const pucData, uint8_t ucDatLen) {
	if(0 == pucData) {
		AFE309_RETURN_FALSE;
	}
	if(ucDatLen > 51) {
		AFE309_RETURN_FALSE;
	}
	if(ucDatLen > 0) {
		i2c_Start();
		if(!i2c_SendByte(ucDevAdd << 1)) {
			i2c_Stop();
			AFE309_RETURN_FALSE;
		}
		if(!i2c_SendByte(ucRegAdd)) {
			i2c_Stop();
			AFE309_RETURN_FALSE;
		}
		if(!i2c_SendByte(ucDatLen)) {
			i2c_Stop();
			AFE309_RETURN_FALSE;
		}
		i2c_Start();
		if(!i2c_SendByte((ucDevAdd << 1) | 0x01)) {
			i2c_Stop();
			AFE309_RETURN_FALSE;
		}
		uint8_t aucTempBuf[51 + 4];														//There are only 51 RAM and 26 EEPROM in total, with a total of one buffer
		aucTempBuf[0] = ucDevAdd << 1;
		aucTempBuf[1] = ucRegAdd;
		aucTempBuf[2] = ucDatLen;
		aucTempBuf[3] = (ucDevAdd << 1) | 0x01;
		for(uint8_t i=0;i<ucDatLen;i++) {
			aucTempBuf[4+i] = i2c_ReadByte();
			i2c_Ack();
		}
		uint8_t ucCrc = i2c_ReadByte();
		i2c_NoAck();   //2024.7.30
		i2c_Stop();
		if(ucCrc != CRC8Cal(aucTempBuf, 4 + ucDatLen)) {		//CRC verification is performed on the read data
			AFE309_RETURN_FALSE;
		}
		for(uint8_t i=0;i<ucDatLen;i++) {
			pucData[i] = aucTempBuf[4+i];										//Read the value of IIC
		}
	}
	
	AFE309_RETURN_TRUE;
}

/*******************************************************************************
* Function Name  : IIC309_WriteData
* Description    : Write data to the slave
* Input          : ucDevAdd, The address of the slave device
*				   				 ucRegAdd, Memory address
*				   				 ucData, Write data
* Output         : None
* Return         : result £∫1 write succeeded, 0 write failed
*******************************************************************************/
bool IIC309_WriteData(uint8_t ucDevAdd, uint8_t ucRegAdd, uint8_t ucData) {	
	i2c_Start();
	if(!i2c_SendByte(ucDevAdd << 1)) {
		i2c_Stop();
		AFE309_RETURN_FALSE;
	}
	if(!i2c_SendByte(ucRegAdd)) {
		i2c_Stop();
		AFE309_RETURN_FALSE;
	}
	if(!i2c_SendByte(ucData)) {							//After the write is successful, write down the next data
		i2c_Stop();
		AFE309_RETURN_FALSE;
	}
	uint8_t aucTempBuf[4];												//It is used to store data for CRC8 verification
	aucTempBuf[0] = ucDevAdd << 1;
	aucTempBuf[1] = ucRegAdd;
	aucTempBuf[2] = ucData;
	aucTempBuf[3] = CRC8Cal(aucTempBuf, 3);			//Perform CRC8 validation on the data to be sent
	if(!i2c_SendByte(aucTempBuf[3])) {
		i2c_Stop();
		AFE309_RETURN_FALSE;
	}
	i2c_Stop();
	if(ucRegAdd > 0x18) {
		afe_delay(1);				//Determine whether to write a program to EEPROM or RAM, with a delay of 1ms,
	} else {
		afe_delay(35);				//EEPROM writes have a delay of 35ms and a RAM write delay of 1ms
	}

	AFE309_RETURN_TRUE;
}

/*******************************************************************************
* Function Name  : MTP_Write309
* Description    : Write data to the slave
* Input          : ucDevAdd, Write the address of the slave device
*				   				 ucRegAdd, Memory address
*				   				 pucData, Write to the first address of the data buffer
*				  			   ucDatLen, The length of the written data
*									 ucCount, Number of fault tolerances
* Output         : None
* Return         : result £∫1 write succeeded, 0 write failed
*******************************************************************************/
bool MTP_Write309(uint8_t ucDevAdd, uint8_t ucRegAdd, uint8_t* const pucData, uint8_t ucDatLen, uint8_t ucCount) {
	for(uint8_t i=0;i<ucDatLen;i++) {
		if(!IIC309_WriteData(ucDevAdd, ucRegAdd, pucData[i])) {			//Determine whether the data is successfully written
			ucCount--;
			if(0 == ucCount) {
				AFE309_RETURN_FALSE;
			}
			afe_delay(1);
			continue;				//Increase program fault tolerance
		}
	}
	AFE309_RETURN_TRUE;
}

/*******************************************************************************
* Function Name  : MTP_Read309
* Description    : Read data to the slave
* Input          : ucDevAdd, Write the address of the slave device
*				   				 ucRegAdd, Memory address
*				  			   ucDatLen, The length of the written data
*									 ucCount, Number of fault tolerances
* Output         : pucData, Write to the first address of the data buffer
* Return         : result £∫1 write succeeded, 0 write failed
*******************************************************************************/
bool MTP_Read309(uint8_t ucDevAdd, uint8_t ucRegAdd, uint8_t* const pucData, uint8_t ucDatLen, uint8_t ucCount) {
	while(ucCount--) {																													//Write repeatedly with a delay of 1 ms
		if(IIC309_ReadData(ucDevAdd, ucRegAdd, pucData, ucDatLen)) {				//Determine whether the data is successfully written
			AFE309_RETURN_TRUE;
		}
		//delay_1ms(1);
		afe_delay(1);
	}
	
	AFE309_RETURN_FALSE;
}

/*******************************************************************************
* Function Name  : MTP_SetVPRO
* Description    : Read data to the slave
* Input          : bVal, 1- Enter the flashing mode; 0 - Exit the flashing mode
* Output         : not
* Return         : result £∫1 write succeeded, 0 write failed
*******************************************************************************/
bool MTP_SetVPRO(bool bVal) {
	if(0 == bVal) {
		AFE_VPRO_L;
		uint8_t ucVal = 0xC0;
		afe_delay(250);
		MTP_Write309(AFE_ID, 0xEA, &ucVal, 1, 5);
		afe_delay(250);
		AFE309_RETURN_TRUE;
	}
	AFE_VPRO_H;
	afe_delay(15);
	
	AFE309_RETURN_TRUE;
}

/* Thermistor table, -50~110°„C temperature, unit k¶∏, different types of thermistor data are different, need to update the table according to the actual product manual, the current resistance value may be wrong */
/*The corresponding is the 3435 model*/
//float fNTC103AT[161] = {329.50,	309.70,	291.24,	274.04,	257.99,	247.70,	233.35,	219.95,	207.43,	195.72,	/* -50~-41°Ê */
//												188.50,	177.96,	168.10,	158.85,	150.19,	144.10,	136.33,	129.04,	122.19,	115.76,	/* -40~-31°Ê */
//												111.30,	105.50,	100.05,	94.92,	90.09,	86.43,	82.08,	77.98,	74.12,	70.48,	/* -30~-21°Ê */
//												67.77,	64.47,	61.36,	58.42,	55.64,	53.41,	50.90,	48.52,	46.27,	44.14,	/* -20~-11°Ê */
//												42.47,	40.53,	38.70,	36.96,	35.32,	33.90,	32.41,	30.99,	29.64,	28.36,	/* -10~-1°Ê */
//												27.28,	26.11,	25.01,	23.95,	22.95,	22.05,	21.14,	20.27,	19.44,	18.66,	/* 0~9°Ê */
//												17.96,	17.24,	16.55,	15.90,	15.27,	14.69,	14.12,	13.57,	13.05,	12.56,	/* 10~19°Ê */
//												12.09,	11.63,	11.20,	10.78,	10.38,	10.00,	9.63,		9.28,		8.94,		8.62,		/* 20~29°Ê */
//												8.31,		8.01,		7.72,		7.45,		7.19,		6.94,		6.70,		6.46,		6.24,		6.03,		/* 30~39°Ê */
//												5.83,		5.62,		5.43,		5.25,		5.08,		4.91,		4.74,		4.59,		4.44,		4.30,		/* 40~49°Ê */
//												4.16,		4.02,		3.89,		3.77,		3.65,		3.54,		3.42,		3.31,		3.21,		3.11,		/* 50~59°Ê */
//												3.02,		2.92,		2.83,		2.75,		2.67,		2.59,		2.51,		2.43,		2.36,		2.29,		/* 60~69°Ê */
//												2.23,		2.16,		2.10,		2.04,		1.98,		1.92,		1.86,		1.81,		1.76,		1.71,		/* 70~79°Ê */
//												1.67,		1.62,		1.57,		1.53,		1.49,		1.45,		1.41,		1.37,		1.33,		1.30,		/* 80~89°Ê */
//												1.27,		1.23,		1.20,		1.17,		1.14,		1.11,		1.08,		1.05,		1.02,		1.00,		/* 90~99°Ê */
//												0.97,		0.95,		0.92,		0.90,		0.88,		0.86,		0.83,		0.81,		0.79,		0.78,		/* 100~109°Ê */
//												0.76};	
float fNTC103AT[161] = {369.42,	344.52,	344.52,	344.52,	344.52,	344.52,	344.52,	344.52,	344.52,	344.52,	/* -50~-41°Ê */
												344.52,	344.52,	321.48,	300.14,	280.38,	262.06,	245.07,	229.30,	214.66,	201.06,	/* -40~-31°Ê */
												188.41,	176.65,	165.71,	155.52,	146.03,	137.18,	128.93,	121.23,	114.05,	107.34,	/* -30~-21°Ê */
												101.07,	95.21,	89.73,	84.60,	79.80,	75.31,	71.10,	67.14,	63.44,	59.97,	/* -20~-11°Ê */
												56.70,	53.64,	50.76,	48.06,	45.52,	43.12,	40.87,	38.75,	36.76,	34.88,	/* -10~-1°Ê */
												33.10,	31.43,	29.86,	28.37,	26.97,	25.64,	24.39,	23.20,	22.09,	21.03,	/* 0~9°Ê */
												20.03,	19.08,	18.18,	17.33,	16.53,	15.77,	15.05,	14.36,	13.71,	13.09,	/* 10~19°Ê */
												12.51,	11.96,	11.43,	10.93,	10.45,	10.00,	9.56,		9.15,		8.77,		8.40,		/* 20~29°Ê */
												8.04,		7.71,		7.39,		7.08,		6.79,		6.51,		6.25,		6.00,		5.75,		5.53,		/* 30~39°Ê */
												5.31,		5.10,		4.90,		4.71,		4.52,		4.35,		4.18,		4.02,		3.87,		3.72,		/* 40~49°Ê */
												3.59,		3.45,		3.32,		3.20,		3.08,		2.97,		2.86,		2.76,		2.66,		2.56,		/* 50~59°Ê */
												2.47,		2.39,		2.30,		2.22,		2.14,		2.07,		2.00,		1.93,		1.86,		1.80,		/* 60~69°Ê */
												1.74,		1.68,		1.63,		1.57,		1.52,		1.47,		1.42,		1.38,		1.33,		1.29,		/* 70~79°Ê */
												1.25,		1.21,		1.17,		1.13,		1.10,		1.06,		1.03,		1.00,		0.97,		0.94,		/* 80~89°Ê */
												0.91,		0.88,		0.85,		0.83,		0.80,		0.78,		0.76,		0.73,		0.71,		0.69,		/* 90~99°Ê */
												0.67,		0.65,		0.63,		0.61,		0.60,		0.58,		0.58,		0.58,		0.58,		0.58,		/* 100~109°Ê */
												0.58};/* 110°Ê */

/*******************************************************************************
* Function Name  : afe_ohms2temp
* Description    : The resistance value is converted to degrees Celsius by looking up the meter
* Input          : fOhms, The resistance value corresponding to the temperature
* Output         : None
* Return         : result : Celsius value
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
* Description    : Convert degrees Celsius to resistance values by looking up the meter
* Input          : fTemp, The temperature at which the resistance value corresponds
* Output         : None
* Return         : result : Resistance value
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
* Return         : result £∫1 write succeeded, 0 write failed
*******************************************************************************/
bool afe_calibrate(uint16_t usMask) {
	if(usMask & 0x0001) {	/* The temperature drift coefficient is calibrated, and the standard temperature is 25°„C*/
		for(uint8_t i=0;i<CFG_TMP_NUM;i++) {
			if(g_stAfe.stRamApp.fTEMP[i] < 10 || g_stAfe.stRamApp.fTEMP[i] > 40) {
				continue;
			}
			g_stCfg.stAfe.afTempCali[i] += (25 - g_stAfe.stRamApp.fTEMP[i]);
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
		g_stCfg.stAfe.fCurCaliA /= ((g_stAfe.stRamApp.fCUR - g_stCfg.stAfe.fCurCaliB) / (1 + (g_stLocalArrayRVal.astPackRVal[g_stPrl.ucSelfId].fMosT - 25) * 0.00005) / 100000);
	}
	if(usMask & 0x0008) {	/* CDATA drift coefficient calibration */
		g_stCfg.stAfe.fCDATACaliB -= g_stAfe.stRamApp.fCDATA;
	}
	if(usMask & 0x0010) {	/* CDATA scale coefficient calibration, standard current 100000mA */
//		if(0 == g_stCfg.stAfe.fCDATACaliA) {
//			g_stCfg.stAfe.fCDATACaliA = 1;
//		}
//		g_stCfg.stAfe.fCDATACaliA /= ((g_stAfe.stRamApp.fCDATA - g_stCfg.stAfe.fCDATACaliB) / (1 + (g_stLocalArrayRVal.astPackRVal[g_stPrl.ucSelfId].fMosT - 25) * 0.00005) / 100000);
	}
	if(usMask & 0x0020) {	/* Cell proportional coefficient calibration, standard voltage 3800mV */
		for(uint8_t i=0;i<g_stAfe.stRomApp.usCN;i++) {
			if(0 == g_stCfg.stAfe.afCellVolCali[i]) {
				g_stCfg.stAfe.afCellVolCali[i] = 1;
			}
			g_stCfg.stAfe.afCellVolCali[i] /= (g_stAfe.stRamApp.fCELLVol[i] / 3800);
		}
	}
	if(usMask & 0x0040) {	/* The proportional coefficient of the whole string battery pack is calibrated, and the standard voltage is 3800*CN mV */
		float fVal = 0;
		for(uint8_t i=0;i<g_stAfe.stRomApp.usCN;i++) {
			fVal += g_stAfe.stRamApp.fCELLVol[i];
		}
		if(0 == g_stCfg.stAfe.fPackVolCali) {
			g_stCfg.stAfe.fPackVolCali = 1;
		}
		g_stCfg.stAfe.fPackVolCali /= (fVal / 3800 / g_stAfe.stRomApp.usCN);
	}
	return cfg_save();
}

/*******************************************************************************
* Function Name  : afe_get_rom
* Description    : Overwrite the ROM configuration of the AFE into memory
* Input          : None
* Output         : None
* Return         : result £∫1 write succeeded, 0 write failed
*******************************************************************************/
bool afe_get_rom(void) {
	uint8_t aucByte[CFG_AFE_ROM_BLEN];
	if(!MTP_Read309(AFE_ID, eAfeRomByteStart, aucByte, CFG_AFE_ROM_BLEN, TRY_TIMES)) {
		AFE309_RETURN_FALSE;
	}
	for(uint8_t i=0;i<CFG_AFE_ROM_BLEN/2;i++) {
		g_stAfe.uRom.aucByte[i * 2] = aucByte[i * 2 + 1];
		g_stAfe.uRom.aucByte[i * 2 + 1] = aucByte[i * 2];
	}
	
	/*****AFE-ROM parameters are displayed***************/
	AFE309_DEBUG("\r\n°æ****************AFE-ROM register parameters*********************°ø\r\n");
	AFE309_DEBUG("		%02x\t%02x\t%02x\t%02x\t%02x\t%02x\t%02x\t%02x\n \
		%02x\t%02x\t%02x\t%02x\t%02x\t%02x\t%02x\t%02x\n \
		%02x\t%02x\t%02x\t%02x\t%02x\t%02x\t%02x\t%02x\n \
		%02x\t%02x\n",
		aucByte[0], aucByte[1], aucByte[2], aucByte[3],
		aucByte[4], aucByte[5], aucByte[6], aucByte[7],
		aucByte[8], aucByte[9], aucByte[10], aucByte[11],
		aucByte[12], aucByte[13], aucByte[14], aucByte[15],
		aucByte[16], aucByte[17], aucByte[18], aucByte[19],
		aucByte[20], aucByte[21], aucByte[22], aucByte[23],
		aucByte[24], aucByte[25]);
	AFE_ROM_CODE_S* pstCode = &g_stAfe.uRom.stCode;
	AFE_ROM_APP_S* pstApp = &g_stAfe.stRomApp;
	CFG_AFE_S* pstCfg = &g_stCfg.stAfe;
	pstApp->usCTLC = pstCode->CTLC;
	if(pstCode->CN < 5) {
		pstApp->usCN = CFG_CELL_NUM;
	} else {
		pstApp->usCN = pstCode->CN;
	}
	switch(pstCode->LDRT) {
		case 0:pstApp->usLDRT = 100;break;
		case 1:pstApp->usLDRT = 500;break;
		case 2:pstApp->usLDRT = 1000;break;
		case 3:pstApp->usLDRT = 2000;break;
		default:pstApp->usLDRT = 0xFFFF;break;
	}
	pstApp->usOV = pstCode->OV * 5 + pstCfg->fPackVolCali / pstApp->usCN;
	switch(pstCode->OVT){
		case 0:pstApp->usOVT = 100;break;
		case 1:pstApp->usOVT = 200;break;
		case 2:pstApp->usOVT = 300;break;
		case 3:pstApp->usOVT = 400;break;
		case 4:pstApp->usOVT = 600;break;
		case 5:pstApp->usOVT = 800;break;
		case 6:pstApp->usOVT = 1000;break;
		case 7:pstApp->usOVT = 2000;break;
		case 8:pstApp->usOVT = 3000;break;
		case 9:pstApp->usOVT = 4000;break;
		case 10:pstApp->usOVT = 6000;break;
		case 11:pstApp->usOVT = 8000;break;
		case 12:pstApp->usOVT = 10000;break;
		case 13:pstApp->usOVT = 20000;break;
		case 14:pstApp->usOVT = 30000;break;
		case 15:pstApp->usOVT = 40000;break;
		default:pstApp->usOVT = 0xFFFF;break;
	}
	pstApp->usOVR = pstCode->OVR * 5 + pstCfg->fPackVolCali / pstApp->usCN;
	pstApp->usUV = pstCode->UV * 20 + pstCfg->fPackVolCali / pstApp->usCN;
	switch(pstCode->UVT){
		case 0:pstApp->usUVT = 100;break;
		case 1:pstApp->usUVT = 200;break;
		case 2:pstApp->usUVT = 300;break;
		case 3:pstApp->usUVT = 400;break;
		case 4:pstApp->usUVT = 600;break;
		case 5:pstApp->usUVT = 800;break;
		case 6:pstApp->usUVT = 1000;break;
		case 7:pstApp->usUVT = 2000;break;
		case 8:pstApp->usUVT = 3000;break;
		case 9:pstApp->usUVT = 4000;break;
		case 10:pstApp->usUVT = 6000;break;
		case 11:pstApp->usUVT = 8000;break;
		case 12:pstApp->usUVT = 10000;break;
		case 13:pstApp->usUVT = 20000;break;
		case 14:pstApp->usUVT = 30000;break;
		case 15:pstApp->usUVT = 40000;break;
		default:pstApp->usUVT = 0xFFFF;break;
	}
	pstApp->usUVR = pstCode->UVR * 20 + pstCfg->fPackVolCali / pstApp->usCN;
	pstApp->usBALV = pstCode->BALV * 20;
	pstApp->usPREV = pstCode->PREV * 20;
	pstApp->usL0V = pstCode->L0V * 20 + pstCfg->fPackVolCali / pstApp->usCN;
	pstApp->usPFV = pstCode->PFV * 20 + pstCfg->fPackVolCali / pstApp->usCN;
	switch(pstCode->OCD1T){
		case 0:pstApp->usOCD1T = 50;break;
		case 1:pstApp->usOCD1T = 100;break;
		case 2:pstApp->usOCD1T = 200;break;
		case 3:pstApp->usOCD1T = 400;break;
		case 4:pstApp->usOCD1T = 600;break;
		case 5:pstApp->usOCD1T = 800;break;
		case 6:pstApp->usOCD1T = 1000;break;
		case 7:pstApp->usOCD1T = 2000;break;
		case 8:pstApp->usOCD1T = 4000;break;
		case 9:pstApp->usOCD1T = 6000;break;
		case 10:pstApp->usOCD1T = 8000;break;
		case 11:pstApp->usOCD1T = 10000;break;
		case 12:pstApp->usOCD1T = 15000;break;
		case 13:pstApp->usOCD1T = 20000;break;
		case 14:pstApp->usOCD1T = 30000;break;
		case 15:pstApp->usOCD1T = 40000;break;
		default:pstApp->usOCD1T = 0xFFFF;break;
	}
	switch(pstCode->OCD1V){
		case 0:pstApp->usOCD1V = 20;break;
		case 1:pstApp->usOCD1V = 30;break;
		case 2:pstApp->usOCD1V = 40;break;
		case 3:pstApp->usOCD1V = 50;break;
		case 4:pstApp->usOCD1V = 60;break;
		case 5:pstApp->usOCD1V = 70;break;
		case 6:pstApp->usOCD1V = 80;break;
		case 7:pstApp->usOCD1V = 90;break;
		case 8:pstApp->usOCD1V = 100;break;
		case 9:pstApp->usOCD1V = 110;break;
		case 10:pstApp->usOCD1V = 120;break;
		case 11:pstApp->usOCD1V = 130;break;
		case 12:pstApp->usOCD1V = 140;break;
		case 13:pstApp->usOCD1V = 160;break;
		case 14:pstApp->usOCD1V = 180;break;
		case 15:pstApp->usOCD1V = 200;break;
		default:pstApp->usOCD1V = 0xFFFF;break;
	}
	switch(pstCode->OCD2T){
		case 0:pstApp->usOCD2T = 10;break;
		case 1:pstApp->usOCD2T = 20;break;
		case 2:pstApp->usOCD2T = 40;break;
		case 3:pstApp->usOCD2T = 60;break;
		case 4:pstApp->usOCD2T = 80;break;
		case 5:pstApp->usOCD2T = 100;break;
		case 6:pstApp->usOCD2T = 200;break;
		case 7:pstApp->usOCD2T = 400;break;
		case 8:pstApp->usOCD2T = 600;break;
		case 9:pstApp->usOCD2T = 800;break;
		case 10:pstApp->usOCD2T = 1000;break;
		case 11:pstApp->usOCD2T = 2000;break;
		case 12:pstApp->usOCD2T = 4000;break;
		case 13:pstApp->usOCD2T = 8000;break;
		case 14:pstApp->usOCD2T = 10000;break;
		case 15:pstApp->usOCD2T = 20000;break;
		default:pstApp->usOCD2T = 0xFFFF;break;
	}
	switch(pstCode->OCD2V){
		case 0:pstApp->usOCD2V = 30;break;
		case 1:pstApp->usOCD2V = 40;break;
		case 2:pstApp->usOCD2V = 50;break;
		case 3:pstApp->usOCD2V = 60;break;
		case 4:pstApp->usOCD2V = 70;break;
		case 5:pstApp->usOCD2V = 80;break;
		case 6:pstApp->usOCD2V = 90;break;
		case 7:pstApp->usOCD2V = 100;break;
		case 8:pstApp->usOCD2V = 120;break;
		case 9:pstApp->usOCD2V = 140;break;
		case 10:pstApp->usOCD2V = 160;break;
		case 11:pstApp->usOCD2V = 180;break;
		case 12:pstApp->usOCD2V = 200;break;
		case 13:pstApp->usOCD2V = 300;break;
		case 14:pstApp->usOCD2V = 400;break;
		case 15:pstApp->usOCD2V = 500;break;
		default:pstApp->usOCD2V = 0xFFFF;break;
	}
	switch(pstCode->SCT){
		case 0:pstApp->usSCT = 0;break;
		case 1:pstApp->usSCT = 64;break;
		case 2:pstApp->usSCT = 128;break;
		case 3:pstApp->usSCT = 192;break;
		case 4:pstApp->usSCT = 256;break;
		case 5:pstApp->usSCT = 320;break;
		case 6:pstApp->usSCT = 384;break;
		case 7:pstApp->usSCT = 448;break;
		case 8:pstApp->usSCT = 512;break;
		case 9:pstApp->usSCT = 576;break;
		case 10:pstApp->usSCT = 640;break;
		case 11:pstApp->usSCT = 704;break;
		case 12:pstApp->usSCT = 768;break;
		case 13:pstApp->usSCT = 832;break;
		case 14:pstApp->usSCT = 896;break;
		case 15:pstApp->usSCT = 960;break;
		default:pstApp->usSCT = 0xFFFF;break;
	}
	switch(pstCode->SCV){
		case 0:pstApp->usSCV = 50;break;
		case 1:pstApp->usSCV = 80;break;
		case 2:pstApp->usSCV = 110;break;
		case 3:pstApp->usSCV = 140;break;
		case 4:pstApp->usSCV = 170;break;
		case 5:pstApp->usSCV = 200;break;
		case 6:pstApp->usSCV = 230;break;
		case 7:pstApp->usSCV = 260;break;
		case 8:pstApp->usSCV = 290;break;
		case 9:pstApp->usSCV = 320;break;
		case 10:pstApp->usSCV = 350;break;
		case 11:pstApp->usSCV = 400;break;
		case 12:pstApp->usSCV = 500;break;
		case 13:pstApp->usSCV = 600;break;
		case 14:pstApp->usSCV = 800;break;
		case 15:pstApp->usSCV = 1000;break;
		default:pstApp->usSCV = 0xFFFF;break;
	}
	switch(pstCode->OCCT){
		case 0:pstApp->usOCCT = 10;break;
		case 1:pstApp->usOCCT = 20;break;
		case 2:pstApp->usOCCT = 40;break;
		case 3:pstApp->usOCCT = 60;break;
		case 4:pstApp->usOCCT = 80;break;
		case 5:pstApp->usOCCT = 100;break;
		case 6:pstApp->usOCCT = 200;break;
		case 7:pstApp->usOCCT = 400;break;
		case 8:pstApp->usOCCT = 600;break;
		case 9:pstApp->usOCCT = 800;break;
		case 10:pstApp->usOCCT = 1000;break;
		case 11:pstApp->usOCCT = 2000;break;
		case 12:pstApp->usOCCT = 4000;break;
		case 13:pstApp->usOCCT = 8000;break;
		case 14:pstApp->usOCCT = 10000;break;
		case 15:pstApp->usOCCT = 20000;break;
		default:pstApp->usOCCT = 0xFFFF;break;
	}
	switch(pstCode->OCCV){
		case 0:pstApp->usOCCV = 20;break;
		case 1:pstApp->usOCCV = 30;break;
		case 2:pstApp->usOCCV = 40;break;
		case 3:pstApp->usOCCV = 50;break;
		case 4:pstApp->usOCCV = 60;break;
		case 5:pstApp->usOCCV = 70;break;
		case 6:pstApp->usOCCV = 80;break;
		case 7:pstApp->usOCCV = 90;break;
		case 8:pstApp->usOCCV = 100;break;
		case 9:pstApp->usOCCV = 110;break;
		case 10:pstApp->usOCCV = 120;break;
		case 11:pstApp->usOCCV = 130;break;
		case 12:pstApp->usOCCV = 140;break;
		case 13:pstApp->usOCCV = 160;break;
		case 14:pstApp->usOCCV = 180;break;
		case 15:pstApp->usOCCV = 200;break;
		default:pstApp->usOCCV = 0xFFFF;break;
	}
	switch(pstCode->PFT){
		case 0:pstApp->usPFT = 8;break;
		case 1:pstApp->usPFT = 16;break;
		case 2:pstApp->usPFT = 32;break;
		case 3:pstApp->usPFT = 64;break;
		default:pstApp->usPFT = 0xFFFF;break;
	}
	switch(pstCode->OCRT){
		case 0:pstApp->usOCRT = 8;break;
		case 1:pstApp->usOCRT = 16;break;
		case 2:pstApp->usOCRT = 32;break;
		case 3:pstApp->usOCRT = 64;break;
		default:pstApp->usOCRT = 0xFFFF;break;
	}
	switch(pstCode->MOST){
		case 0:pstApp->usMOST = 64;break;
		case 1:pstApp->usMOST = 128;break;
		case 2:pstApp->usMOST = 256;break;
		case 3:pstApp->usMOST = 512;break;
		default:pstApp->usMOST = 0xFFFF;break;
	}
	switch(pstCode->CHS){
		case 0:pstApp->usCHS = 200;break;
		case 1:pstApp->usCHS = 500;break;
		case 2:pstApp->usCHS = 1000;break;
		case 3:pstApp->usCHS = 2000;break;
		default:pstApp->usCHS = 0xFFFF;break;
	}
	pstApp->fTR = 6.8 + 0.05 * pstCode->TR;		/* The unit is k¶∏ */
	pstApp->fOTC = afe_ohms2temp(pstCode->OTC * pstApp->fTR / (512 - pstCode->OTC)) + pstCfg->fAvgTempCali;						/* Unit: °„C */
	pstApp->fOTCR = afe_ohms2temp(pstCode->OTCR * pstApp->fTR / (512 - pstCode->OTCR)) + pstCfg->fAvgTempCali;				/* Unit: °„C */
	pstApp->fUTC = afe_ohms2temp((pstCode->UTC + 256) * pstApp->fTR / (256 - pstCode->UTC)) + pstCfg->fAvgTempCali;		/* Unit: °„C */
	pstApp->fUTCR = afe_ohms2temp((pstCode->UTCR + 256) * pstApp->fTR / (256 - pstCode->UTCR)) + pstCfg->fAvgTempCali;/* Unit: °„C */
	pstApp->fOTD = afe_ohms2temp(pstCode->OTD * pstApp->fTR / (512 - pstCode->OTD)) + pstCfg->fAvgTempCali;						/* Unit: °„C */
	pstApp->fOTDR = afe_ohms2temp(pstCode->OTDR * pstApp->fTR / (512 - pstCode->OTDR)) + pstCfg->fAvgTempCali;				/* Unit: °„C */
	pstApp->fUTD = afe_ohms2temp((pstCode->UTD + 256) * pstApp->fTR / (256 - pstCode->UTD)) + pstCfg->fAvgTempCali;		/* Unit: °„C */
	pstApp->fUTDR = afe_ohms2temp((pstCode->UTDR + 256) * pstApp->fTR / (256 - pstCode->UTDR)) + pstCfg->fAvgTempCali;/* Unit: °„C */
	
	AFE309_RETURN_TRUE;
}

/*******************************************************************************
* Function Name  : afe_set_rom
* Description    : Compare the desired configuration with the ROM's ROM, and if there is any discrepancy, overwrite the in-memory configuration into the AFE
* Input          : None
* Output         : None
* Return         : result £∫1 write succeeded, 0 write failed
*******************************************************************************/
bool afe_set_rom(void) {
	uint8_t aucByte[CFG_AFE_ROM_BLEN];
	if(!MTP_Read309(AFE_ID, eAfeRomByteStart, aucByte, CFG_AFE_ROM_BLEN, TRY_TIMES)) {
		//MTP_SetVPRO(false);
		AFE309_RETURN_FALSE;
	}
	uint8_t ucVProSet = false;
	for(uint8_t i=0;i<CFG_AFE_ROM_BLEN / 2;i++) {
		if(g_stAfe.uRom.aucByte[i * 2 + 1] != aucByte[i * 2]) {
			if(!ucVProSet) {
				if(!MTP_SetVPRO(true)) {
					AFE309_RETURN_FALSE;
				}
				ucVProSet = true;
			}
			if(!MTP_Write309(AFE_ID, eAfeRomByteStart + i * 2, g_stAfe.uRom.aucByte + i * 2 + 1, 1, 5)) {
				MTP_SetVPRO(false);
				AFE309_RETURN_FALSE;
			}
		}
		if(g_stAfe.uRom.aucByte[i * 2] != aucByte[i * 2 + 1] && (i * 2 + 1) != 0x19) {
			if(!ucVProSet) {
				if(!MTP_SetVPRO(true)) {
					AFE309_RETURN_FALSE;
				}
				ucVProSet = true;
			}
			if(!MTP_Write309(AFE_ID, eAfeRomByteStart + i * 2 + 1, g_stAfe.uRom.aucByte + i * 2, 1, 5)) {
				MTP_SetVPRO(false);
				AFE309_RETURN_FALSE;
			}
		}
	}
	if(ucVProSet) {
		MTP_SetVPRO(false);
	}
	
	AFE309_RETURN_TRUE;
}

/*******************************************************************************
* Function Name  : afe_get_ram
* Description    : Overwrite the RAM configuration of the AFE into memory
* Input          : None
* Output         : None
* Return         : result £∫1 write succeeded, 0 write failed
*******************************************************************************/
bool afe_get_ram(void) {
	uint8_t aucByte[AFE_RAM_BLEN];
	if(!MTP_Read309(AFE_ID, eAfeRamByteStart, aucByte, AFE_RAM_BLEN, TRY_TIMES)) {
		AFE309_RETURN_FALSE;
	}
	for(uint8_t i=0;i<AFE_RAM_BLEN/2;i++) {
		g_stAfe.uRam.aucByte[i * 2] = aucByte[i  * 2 + 1];
		g_stAfe.uRam.aucByte[i * 2 + 1] = aucByte[i * 2];
	}
	
	/****The AFE-RAM parameters are displayed****************/
//	AFE309_DEBUG("\r\n°æ***************AFE-RAM register parameters****************°ø\r\n");
//	for(uint8_t i = 0;i < AFE_RAM_BLEN;i++) {
//	AFE309_DEBUG("[%02x]%02x\t",(i+0x40),aucByte[i]);
//	}
//	AFE309_DEBUG("\r\n");
	AFE_RAM_CODE_S* pstCode = &g_stAfe.uRam.stCode;
	AFE_RAM_APP_S* pstApp = &g_stAfe.stRamApp;
	CFG_AFE_S* pstCfg = &g_stCfg.stAfe;
	for(uint8_t i=0;i<CFG_TMP_NUM;i++) {
		pstApp->fTEMP[i] = (int16_t)(afe_ohms2temp(pstCode->TEMP[i] * g_stAfe.stRomApp.fTR / (32767 - pstCode->TEMP[i]) + pstCfg->afTempCali[i]));
	}
 	pstApp->fCUR = (float)pstCode->CUR * 200 / 26837 / LOCAL_CUR_RESIST * pstCfg->fCurCaliA / (1 + (g_stLocalArrayRVal.astPackRVal[g_stPrl.ucSelfId].fMosT - 25) * 0.00005) + pstCfg->fCurCaliB;
	for(uint8_t i=0;i<CFG_CELL_NUM;i++) {
		if(g_stCfg.stAfe.afCellVolCali[i] < 0.01 || g_stCfg.stAfe.afCellVolCali[i] == 0XFFFF || g_stCfg.stAfe.afCellVolCali[i] > 255) {
			g_stCfg.stAfe.afCellVolCali[i] = 1;
			pstApp->fCELLVol[i] = pstCode->CELL[i] * 5 / 32;
		} else {
			pstApp->fCELLVol[i] = pstCode->CELL[i] * 5 / 32 * g_stCfg.stAfe.afCellVolCali[i];
		}
	}
//	if(pstCode->CDATA > 0) {
//		pstApp->fCDATA = (float)pstCode->CDATA * 200 / (LOCAL_CUR_RESIST + g_stCfg.stLocal.sChgCurSnrCoeB * 0.000001)
//			/ 21470 * pstCfg->fCDATACaliA / (1 + (g_stLocalArrayRVal.astPackRVal[g_stPrl.ucSelfId].fMosT - 25) * 0.00005)
//			+ pstCfg->fCDATACaliB;
//	} else {
//		pstApp->fCDATA = (float)pstCode->CDATA * 200 / (LOCAL_CUR_RESIST + g_stCfg.stLocal.sDsgCurSnrCoeB * 0.000001)
//			/ 21470 * pstCfg->fCDATACaliA / (1 + (g_stLocalArrayRVal.astPackRVal[g_stPrl.ucSelfId].fMosT - 25) * 0.00005)
//			+ pstCfg->fCDATACaliB;
//	}
	  if(g_stCfg.stAfe.fCDATACaliA_CHG < 0.01 || g_stCfg.stAfe.fCDATACaliA_CHG == 0XFFFF || g_stCfg.stAfe.fCDATACaliA_CHG > 255){
		  g_stCfg.stAfe.fCDATACaliA_CHG = 1;
	  }
		if(g_stCfg.stAfe.fCDATACaliA_DSG < 0.01 || g_stCfg.stAfe.fCDATACaliA_DSG == 0XFFFF || g_stCfg.stAfe.fCDATACaliA_CHG > 255){
			g_stCfg.stAfe.fCDATACaliA_DSG = 1;
		}
		if(g_stCfg.stAfe.fCDATACaliA_DSG ==1){
			g_stCfg.stAfe.fCDATACaliA_DSG = g_stCfg.stAfe.fCDATACaliA_CHG;
		}
		if(g_stCfg.stAfe.fCDATACaliB > 300000 || g_stCfg.stAfe.fCDATACaliB < -300000){
			g_stCfg.stAfe.fCDATACaliB = 0;
		}
		if(pstCode->CDATA > 0) {
		pstApp->fCDATA = (float)pstCode->CDATA * 200 * g_stCfg.stAfe.fCDATACaliA_CHG / (LOCAL_CUR_RESIST)
			/ 21470 * g_stCfg.stLocal.sChgCurSnrCoeB * 0.01 / (1 + (g_stLocalArrayRVal.astPackRVal[g_stPrl.ucSelfId].fMosT - 25) * 0.00005)
			+ g_stCfg.stAfe.fCDATACaliB;
	  } else {
		pstApp->fCDATA = (float)pstCode->CDATA * 200 * g_stCfg.stAfe.fCDATACaliA_DSG / (LOCAL_CUR_RESIST)
			/ 21470 * g_stCfg.stLocal.sDsgCurSnrCoeB * 0.01 / (1 + (g_stLocalArrayRVal.astPackRVal[g_stPrl.ucSelfId].fMosT - 25) * 0.00005)
			+ g_stCfg.stAfe.fCDATACaliB;
	  }
	
	/******Print cell voltage********/
//	AFE309_DEBUG("\r\n°æ**********AFE - Voltage - Temperature - Current*************°ø\r\n");
//	float Vol_all=0;
//	for(int i=0;i<CFG_CELL_NUM;i++){
//		AFE309_DEBUG("[%d]%.2f mV\t",i,pstApp->fCELLVol[i]);
//		Vol_all += pstApp->fCELLVol[i];
//	}
//	AFE309_DEBUG("\r\n");
//	AFE309_DEBUG("AFE battery voltage:%.2f mV\r\n",Vol_all);
//	
//	/******Print the temperature value********/
//	for(int i=0;i<CFG_TMP_NUM;i++) {
//		AFE309_DEBUG("[%d]%.2f °Ê\t",i,pstApp->fTEMP[i]);
//	}
//	AFE309_DEBUG("\r\n");
//	/******Print the battery current********/
//	AFE309_DEBUG("AFE current:%.2f mA\r\n",pstApp->fCDATA);
	
	AFE309_RETURN_TRUE;
}

/*******************************************************************************
* Function Name  : afe_set_ram
* Description    : Compare the desired configuration with the RAM of the AFE, and if there is any discrepancy, overwrite the in-memory configuration to the AFE
* Input          : None
* Output         : None
* Return         : result £∫1 write succeeded, 0 write failed
*******************************************************************************/
bool afe_set_ram(void) {
	uint8_t ucByte;
	if(!MTP_Read309(AFE_ID, eAfeRamByteStart, &ucByte, 1, TRY_TIMES)) {
		AFE309_RETURN_FALSE;
	}
	/* After analysis, only 0x40 registers need to be rewritten, and the others can be read-only */
	if(g_stAfe.uRam.aucByte[1] != ucByte) {
		if(!MTP_Write309(AFE_ID, eAfeRamByteStart, g_stAfe.uRam.aucByte + 1, 1, 5)) {
				AFE309_RETURN_FALSE;
		}
	}
//	for(uint8_t i=0;i<AFE_RAM_BLEN / 2;i++) {
//		/* There are two bits in BFLAG2 with address 0x71 that cannot be written, but the AFE operation is written in bytes, and I don't know how to do it */
//		if(g_stAfe.uRam.aucByte[i * 2 + 1] != aucByte[i * 2] && (i * 2 + 1) != (0x43-0x40) && (i * 2 + 1) != (0x45-0x40)) {   //change
//			if(!MTP_Write309(AFE_ID, eAfeRamByteStart + i * 2, g_stAfe.uRam.aucByte +(i * 2 + 1), 1, 5)) {
//				AFE309_RETURN_FALSE;
//			}
//		}
//		if(g_stAfe.uRam.aucByte[i * 2] != aucByte[i * 2 + 1] && (i * 2) != (0x44-0x40) && (i * 2) != (0x72-0x40)) {           //change
//			if(!MTP_Write309(AFE_ID, eAfeRamByteStart + i * 2 + 1, g_stAfe.uRam.aucByte + (i * 2), 1, 5)) {
//				AFE309_RETURN_FALSE;
//			}
//		}
//	}

	AFE309_RETURN_TRUE;
}

/*******************************************************************************
* Function Name  : afe_get_ai
* Description    : Read variables by enumeration
* Input          : usAddr,	Enumerate the address
* Output         : pfVal,	Read the variable results
* Return         : result, 1 write succeeded, 0 write failed
*******************************************************************************/
bool afe_get_ai(uint16_t usAddr, float* pfVal) {
	if(0 == pfVal) {
		AFE309_RETURN_FALSE;
	}
	if(/*usAddr >= eAfeRomByteStart && */usAddr <= eAfeRomByteEnd) {
		*pfVal = g_stAfe.uRom.aucByte[usAddr - eAfeRomByteStart + 1 - usAddr % 2 * 2];
		AFE309_RETURN_TRUE;
	}
	if(usAddr >= eAfeRamByteStart && usAddr <= eAfeRamByteEnd) {
		*pfVal = g_stAfe.uRam.aucByte[usAddr - eAfeRamByteStart + 1 - usAddr % 2 * 2];
		AFE309_RETURN_TRUE;
	}
	if(usAddr >= eAfeRomCodeStart && usAddr <= eAfeRomCodeEnd) {
		switch(usAddr) {
			case eAfeRomCodeCN:*pfVal = g_stAfe.uRom.stCode.CN;break;
			case eAfeRomCodeBAL:*pfVal = g_stAfe.uRom.stCode.BAL;break;
			case eAfeRomCodeOCPM:*pfVal = g_stAfe.uRom.stCode.OCPM;break;
			case eAfeRomCodeENMOS:*pfVal = g_stAfe.uRom.stCode.ENMOS;break;
			case eAfeRomCodeENPCH:*pfVal = g_stAfe.uRom.stCode.ENPCH;break;
			case eAfeRomCodeEUVR:*pfVal = g_stAfe.uRom.stCode.EUVR;break;
			case eAfeRomCodeOCRA:*pfVal = g_stAfe.uRom.stCode.OCRA;break;
			case eAfeRomCodeCTLC:*pfVal = g_stAfe.uRom.stCode.CTLC;break;
			case eAfeRomCodeDIS_PF:*pfVal = g_stAfe.uRom.stCode.DIS_PF;break;
			case eAfeRomCodeUV_OP:*pfVal = g_stAfe.uRom.stCode.UV_OP;break;
			case eAfeRomCodeE0VB:*pfVal = g_stAfe.uRom.stCode.E0VB;break;
			case eAfeRomCodeOV:*pfVal = g_stAfe.uRom.stCode.OV;break;
			case eAfeRomCodeLDRT:*pfVal = g_stAfe.uRom.stCode.LDRT;break;
			case eAfeRomCodeOVT:*pfVal = g_stAfe.uRom.stCode.OVT;break;
			case eAfeRomCodeOVR:*pfVal = g_stAfe.uRom.stCode.OVR;break;
			case eAfeRomCodeUVT:*pfVal = g_stAfe.uRom.stCode.UVT;break;
			case eAfeRomCodeUV:*pfVal = g_stAfe.uRom.stCode.UV;break;
			case eAfeRomCodeUVR:*pfVal = g_stAfe.uRom.stCode.UVR;break;
			case eAfeRomCodeBALV:*pfVal = g_stAfe.uRom.stCode.BALV;break;
			case eAfeRomCodePREV:*pfVal = g_stAfe.uRom.stCode.PREV;break;
			case eAfeRomCodeL0V:*pfVal = g_stAfe.uRom.stCode.L0V;break;
			case eAfeRomCodePFV:*pfVal = g_stAfe.uRom.stCode.PFV;break;
			case eAfeRomCodeOCD1T:*pfVal = g_stAfe.uRom.stCode.OCD1T;break;
			case eAfeRomCodeOCD1V:*pfVal = g_stAfe.uRom.stCode.OCD1V;break;
			case eAfeRomCodeOCD2T:*pfVal = g_stAfe.uRom.stCode.OCD2T;break;
			case eAfeRomCodeOCD2V:*pfVal = g_stAfe.uRom.stCode.OCD2V;break;
			case eAfeRomCodeSCT:*pfVal = g_stAfe.uRom.stCode.SCT;break;
			case eAfeRomCodeSCV:*pfVal = g_stAfe.uRom.stCode.SCV;break;
			case eAfeRomCodeOCCT:*pfVal = g_stAfe.uRom.stCode.OCCT;break;
			case eAfeRomCodeOCCV:*pfVal = g_stAfe.uRom.stCode.OCCV;break;
			case eAfeRomCodePFT:*pfVal = g_stAfe.uRom.stCode.PFT;break;
			case eAfeRomCodeOCRT:*pfVal = g_stAfe.uRom.stCode.OCRT;break;
			case eAfeRomCodeMOST:*pfVal = g_stAfe.uRom.stCode.MOST;break;
			case eAfeRomCodeCHS:*pfVal = g_stAfe.uRom.stCode.CHS;break;
			case eAfeRomCodeOTC:*pfVal = g_stAfe.uRom.stCode.OTC;break;
			case eAfeRomCodeOTCR:*pfVal = g_stAfe.uRom.stCode.OTCR;break;
			case eAfeRomCodeUTC:*pfVal = g_stAfe.uRom.stCode.UTC;break;
			case eAfeRomCodeUTCR:*pfVal = g_stAfe.uRom.stCode.UTCR;break;
			case eAfeRomCodeOTD:*pfVal = g_stAfe.uRom.stCode.OTD;break;
			case eAfeRomCodeOTDR:*pfVal = g_stAfe.uRom.stCode.OTDR;break;
			case eAfeRomCodeUTD:*pfVal = g_stAfe.uRom.stCode.UTD;break;
			case eAfeRomCodeUTDR:*pfVal = g_stAfe.uRom.stCode.UTDR;break;
			case eAfeRomCodeTR:*pfVal = g_stAfe.uRom.stCode.TR;break;
			default:AFE309_RETURN_FALSE;
		}
		AFE309_RETURN_TRUE;
	}
	if(usAddr >= eAfeRamCodeStart && usAddr <= eAfeRamCodeEnd) {
		switch(usAddr) {
			case eAfeRamCodeIDLE:*pfVal = g_stAfe.uRam.stCode.IDLE;break;
			case eAfeRamCodeSLEEP:*pfVal = g_stAfe.uRam.stCode.SLEEP;break;
			case eAfeRamCodeENWDT:*pfVal = g_stAfe.uRam.stCode.ENWDT;break;
			case eAfeRamCodeCADCON:*pfVal = g_stAfe.uRam.stCode.CADCON;break;
			case eAfeRamCodeCHGMOS:*pfVal = g_stAfe.uRam.stCode.CHGMOS;break;
			case eAfeRamCodeDSGMOS:*pfVal = g_stAfe.uRam.stCode.DSGMOS;break;
			case eAfeRamCodePCHMOS:*pfVal = g_stAfe.uRam.stCode.PCHMOS;break;
			case eAfeRamCodeOCRC:*pfVal = g_stAfe.uRam.stCode.OCRC;break;
			case eAfeRamCodeCB:*pfVal = g_stAfe.uRam.stCode.BALANCEH * 0x100 + g_stAfe.uRam.stCode.BALANCEL;break;
			case eAfeRamCodeOV:*pfVal = g_stAfe.uRam.stCode.OV;break;
			case eAfeRamCodeUV:*pfVal = g_stAfe.uRam.stCode.UV;break;
			case eAfeRamCodeOCD1:*pfVal = g_stAfe.uRam.stCode.OCD1;break;
			case eAfeRamCodeOCD2:*pfVal = g_stAfe.uRam.stCode.OCD2;break;
			case eAfeRamCodeOCC:*pfVal = g_stAfe.uRam.stCode.OCC;break;
			case eAfeRamCodeSC:*pfVal = g_stAfe.uRam.stCode.SC;break;
			case eAfeRamCodePF:*pfVal = g_stAfe.uRam.stCode.PF;break;
			case eAfeRamCodeWDT:*pfVal = g_stAfe.uRam.stCode.WDT;break;
			case eAfeRamCodeUTC:*pfVal = g_stAfe.uRam.stCode.UTC;break;
			case eAfeRamCodeOTC:*pfVal = g_stAfe.uRam.stCode.OTC;break;
			case eAfeRamCodeUTD:*pfVal = g_stAfe.uRam.stCode.UTD;break;
			case eAfeRamCodeOTD:*pfVal = g_stAfe.uRam.stCode.OTD;break;
			case eAfeRamCodeDSG_FET:*pfVal = g_stAfe.uRam.stCode.DSG_FET;break;
			case eAfeRamCodeCHG_FET:*pfVal = g_stAfe.uRam.stCode.PCHG_FET;break;
			case eAfeRamCodePCHG_FET:*pfVal = g_stAfe.uRam.stCode.PCHG_FET;break;
			case eAfeRamCodeL0V:*pfVal = g_stAfe.uRam.stCode.L0V;break;
			case eAfeRamCodeEEPR_WR:*pfVal = g_stAfe.uRam.stCode.EEPR_WR;break;
			case eAfeRamCodeDSGING:*pfVal = g_stAfe.uRam.stCode.DSGING;break;
			case eAfeRamCodeCHGING:*pfVal = g_stAfe.uRam.stCode.CHGING;break;
			case eAfeRamCodeTEMP1:*pfVal = g_stAfe.uRam.stCode.TEMP[0];break;
			case eAfeRamCodeTEMP2:*pfVal = g_stAfe.uRam.stCode.TEMP[1];break;
			case eAfeRamCodeTEMP3:*pfVal = g_stAfe.uRam.stCode.TEMP[2];break;
			case eAfeRamCodeCUR:*pfVal = g_stAfe.uRam.stCode.CUR;break;
			case eAfeRamCodeCELL1:*pfVal = g_stAfe.uRam.stCode.CELL[0];break;
			case eAfeRamCodeCELL2:*pfVal = g_stAfe.uRam.stCode.CELL[1];break;
			case eAfeRamCodeCELL3:*pfVal = g_stAfe.uRam.stCode.CELL[2];break;
			case eAfeRamCodeCELL4:*pfVal = g_stAfe.uRam.stCode.CELL[3];break;
			case eAfeRamCodeCELL5:*pfVal = g_stAfe.uRam.stCode.CELL[4];break;
			case eAfeRamCodeCELL6:*pfVal = g_stAfe.uRam.stCode.CELL[5];break;
			case eAfeRamCodeCELL7:*pfVal = g_stAfe.uRam.stCode.CELL[6];break;
			case eAfeRamCodeCELL8:*pfVal = g_stAfe.uRam.stCode.CELL[7];break;
			case eAfeRamCodeCELL9:*pfVal = g_stAfe.uRam.stCode.CELL[8];break;
			case eAfeRamCodeCELL10:*pfVal = g_stAfe.uRam.stCode.CELL[9];break;
			case eAfeRamCodeCELL11:*pfVal = g_stAfe.uRam.stCode.CELL[10];break;
			case eAfeRamCodeCELL12:*pfVal = g_stAfe.uRam.stCode.CELL[11];break;
			case eAfeRamCodeCELL13:*pfVal = g_stAfe.uRam.stCode.CELL[12];break;
			case eAfeRamCodeCELL14:*pfVal = g_stAfe.uRam.stCode.CELL[13];break;
			case eAfeRamCodeCELL15:*pfVal = g_stAfe.uRam.stCode.CELL[14];break;
			case eAfeRamCodeCELL16:*pfVal = g_stAfe.uRam.stCode.CELL[15];break;
			case eAfeRamCodeCADCD:*pfVal = g_stAfe.uRam.stCode.CDATA;break;
			case eAfeRamCodeOV_FLG:*pfVal = g_stAfe.uRam.stCode.UTC_FLG;break;
			case eAfeRamCodeUV_FLG:*pfVal = g_stAfe.uRam.stCode.OTC_FLG;break;
			case eAfeRamCodeOCD_FLG:*pfVal = g_stAfe.uRam.stCode.UTD_FLG;break;
			case eAfeRamCodeLOAD_FLG:*pfVal = g_stAfe.uRam.stCode.OTD_FLG;break;
			case eAfeRamCodeOCC_FLG:*pfVal = g_stAfe.uRam.stCode.VADC_FLG;break;
			case eAfeRamCodeSC_FLG:*pfVal = g_stAfe.uRam.stCode.CADC_FLG;break;
			case eAfeRamCodePF_FLG:*pfVal = g_stAfe.uRam.stCode.WAKE_FLG;break;
			case eAfeRamCodeWDT_FLG:*pfVal = g_stAfe.uRam.stCode.RST_FLG;break;
			case eAfeRamCodeUTC_FLG:*pfVal = g_stAfe.uRam.stCode.OV_FLG;break;
			case eAfeRamCodeOTC_FLG:*pfVal = g_stAfe.uRam.stCode.UV_FLG;break;
			case eAfeRamCodeUTD_FLG:*pfVal = g_stAfe.uRam.stCode.OCD_FLG;break;
			case eAfeRamCodeOTD_FLG:*pfVal = g_stAfe.uRam.stCode.LOAD_FLG;break;
			case eAfeRamCodeVADC_FLG:*pfVal = g_stAfe.uRam.stCode.OCC_FLG;break;
			case eAfeRamCodeCADC_FLG:*pfVal = g_stAfe.uRam.stCode.SC_FLG;break;
			case eAfeRamCodeWAKE_FLG:*pfVal = g_stAfe.uRam.stCode.PF_FLG;break;
			case eAfeRamCodeRST_FLG:*pfVal = g_stAfe.uRam.stCode.WDT_FLG;break;
			case eAfeRamCodeWDT_CNT:*pfVal = g_stAfe.uRam.stCode.RSTSTAT;break;
			default:AFE309_RETURN_FALSE;
		}
		AFE309_RETURN_TRUE;
	}
	if(usAddr >= eAfeRomAppStart && usAddr <= eAfeRomAppEnd) {
		switch(usAddr) {
			case eAfeRomAppCTLC:*pfVal = g_stAfe.stRomApp.usCTLC;break;
			case eAfeRomAppCN:*pfVal = g_stAfe.stRomApp.usCN;break;
			case eAfeRomAppLDRT:*pfVal = g_stAfe.stRomApp.usLDRT;break;
			case eAfeRomAppOV:*pfVal = g_stAfe.stRomApp.usOV;break;
			case eAfeRomAppOVT:*pfVal = g_stAfe.stRomApp.usOVT;break;
			case eAfeRomAppOVR:*pfVal = g_stAfe.stRomApp.usOVR;break;
			case eAfeRomAppUV:*pfVal = g_stAfe.stRomApp.usUV;break;
			case eAfeRomAppUVT:*pfVal = g_stAfe.stRomApp.usUVT;break;
			case eAfeRomAppUVR:*pfVal = g_stAfe.stRomApp.usUVR;break;
			case eAfeRomAppBALV:*pfVal = g_stAfe.stRomApp.usBALV;break;
			case eAfeRomAppPREV:*pfVal = g_stAfe.stRomApp.usPREV;break;
			case eAfeRomAppL0V:*pfVal = g_stAfe.stRomApp.usL0V;break;
			case eAfeRomAppPFV:*pfVal = g_stAfe.stRomApp.usPFT;break;
			case eAfeRomAppOCD1T:*pfVal = g_stAfe.stRomApp.usOCD1T;break;
			case eAfeRomAppOCD1V:*pfVal = g_stAfe.stRomApp.usOCD1V;break;
			case eAfeRomAppOCD2T:*pfVal = g_stAfe.stRomApp.usOCD2T;break;
			case eAfeRomAppOCD2V:*pfVal = g_stAfe.stRomApp.usOCD2V;break;
			case eAfeRomAppSCT:*pfVal = g_stAfe.stRomApp.usSCT;break;
			case eAfeRomAppSCV:*pfVal = g_stAfe.stRomApp.usSCV;break;
			case eAfeRomAppOCCT:*pfVal = g_stAfe.stRomApp.usOCCT;break;
			case eAfeRomAppOCCV:*pfVal = g_stAfe.stRomApp.usOCCV;break;
			case eAfeRomAppPFT:*pfVal = g_stAfe.stRomApp.usPFT;break;
			case eAfeRomAppOCRT:*pfVal = g_stAfe.stRomApp.usOCRT;break;
			case eAfeRomAppMOST:*pfVal = g_stAfe.stRomApp.usMOST;break;
			case eAfeRomAppCHS:*pfVal = g_stAfe.stRomApp.usCHS;break;
			case eAfeRomAppOTC:*pfVal = g_stAfe.stRomApp.fOTC;break;
			case eAfeRomAppOTCR:*pfVal = g_stAfe.stRomApp.fOTCR;break;
			case eAfeRomAppUTC:*pfVal = g_stAfe.stRomApp.fUTC;break;
			case eAfeRomAppUTCR:*pfVal = g_stAfe.stRomApp.fUTCR;break;
			case eAfeRomAppOTD:*pfVal = g_stAfe.stRomApp.fOTD;break;
			case eAfeRomAppOTDR:*pfVal = g_stAfe.stRomApp.fOTDR;break;
			case eAfeRomAppUTD:*pfVal = g_stAfe.stRomApp.fUTD;break;
			case eAfeRomAppUTDR:*pfVal = g_stAfe.stRomApp.fUTDR;break;
			case eAfeRomAppTR:*pfVal = g_stAfe.stRomApp.fTR;break;
			default:AFE309_RETURN_FALSE;
		}
		AFE309_RETURN_TRUE;
	}
	if(usAddr >= eAfeRamAppStart && usAddr <= eAfeRamAppEnd) {
		switch(usAddr) {
			case eAfeRamTEMP1:*pfVal = g_stAfe.stRamApp.fTEMP[0];break;
			case eAfeRamTEMP2:*pfVal = g_stAfe.stRamApp.fTEMP[1];break;
			case eAfeRamTEMP3:*pfVal = g_stAfe.stRamApp.fTEMP[2];break;
			case eAfeRamCUR:*pfVal = g_stAfe.stRamApp.fCUR;break;
			case eAfeRamCELL1:*pfVal = g_stAfe.stRamApp.fCELLVol[0];break;
			case eAfeRamCELL2:*pfVal = g_stAfe.stRamApp.fCELLVol[1];break;
			case eAfeRamCELL3:*pfVal = g_stAfe.stRamApp.fCELLVol[2];break;
			case eAfeRamCELL4:*pfVal = g_stAfe.stRamApp.fCELLVol[3];break;
			case eAfeRamCELL5:*pfVal = g_stAfe.stRamApp.fCELLVol[4];break;
			case eAfeRamCELL6:*pfVal = g_stAfe.stRamApp.fCELLVol[5];break;
			case eAfeRamCELL7:*pfVal = g_stAfe.stRamApp.fCELLVol[6];break;
			case eAfeRamCELL8:*pfVal = g_stAfe.stRamApp.fCELLVol[7];break;
			case eAfeRamCELL9:*pfVal = g_stAfe.stRamApp.fCELLVol[8];break;
			case eAfeRamCELL10:*pfVal = g_stAfe.stRamApp.fCELLVol[9];break;
			case eAfeRamCELL11:*pfVal = g_stAfe.stRamApp.fCELLVol[10];break;
			case eAfeRamCELL12:*pfVal = g_stAfe.stRamApp.fCELLVol[11];break;
			case eAfeRamCELL13:*pfVal = g_stAfe.stRamApp.fCELLVol[12];break;
			case eAfeRamCELL14:*pfVal = g_stAfe.stRamApp.fCELLVol[13];break;
			case eAfeRamCELL15:*pfVal = g_stAfe.stRamApp.fCELLVol[14];break;
			case eAfeRamCELL16:*pfVal = g_stAfe.stRamApp.fCELLVol[15];break;
			case eAfeRamCDATA:*pfVal = g_stAfe.stRamApp.fCDATA;break;
			case eAfeRamWDT:*pfVal = g_stAfe.stRamApp.usWDT;break;
			default:AFE309_RETURN_FALSE;
		}
		AFE309_RETURN_TRUE;
	}
	
	AFE309_RETURN_FALSE;
}

/*******************************************************************************
* Function Name  : afe_set_ao
* Description    : Write variables by enumeration
* Input          : usAddr,	Enumerate the address
								 : ucCnt, The number of variables that need to be written
								 : pusVal,	The value of the variable that needs to be written
* Output         : None
* Return         : result, 1 write succeeded, 0 write failed
*******************************************************************************/
bool afe_set_ao(uint16_t usAddr, uint8_t ucCnt, float* pfVal) {
//AFE309_DEBUG("Addr=%d,Cnt=%d,Val=%f", usAddr, ucCnt, *pfVal);
	if(0 == pfVal) {
		AFE309_RETURN_FALSE;
	}
	for(uint16_t i=usAddr;i<(usAddr + ucCnt);i++) {
		if(/*i >= eAfeRomByteStart && */i <= eAfeRomByteEnd - 1) {
			g_stAfe.uRom.aucByte[i - eAfeRomByteStart + 1 - i % 2 * 2] = ((uint16_t)pfVal[i - usAddr] & 0xFF);
		} else if(i >= eAfeRamByteStart && i <= eAfeRamByteEnd) {
			g_stAfe.uRam.aucByte[i - eAfeRamByteStart + 1 - i % 2 * 2] = ((uint16_t)pfVal[i - usAddr] & 0xFF);
		} else if(i >= eAfeRomCodeStart && i <= eAfeRomCodeEnd) {
			switch(i) {
				case eAfeRomCodeCN:g_stAfe.uRom.stCode.CN = (uint16_t)pfVal[i - usAddr];break;
				case eAfeRomCodeBAL:g_stAfe.uRom.stCode.BAL = (uint16_t)pfVal[i - usAddr];break;
				case eAfeRomCodeOCPM:g_stAfe.uRom.stCode.BAL = (uint16_t)pfVal[i - usAddr];break;
				case eAfeRomCodeENMOS:g_stAfe.uRom.stCode.BAL = (uint16_t)pfVal[i - usAddr];break;
				case eAfeRomCodeENPCH:g_stAfe.uRom.stCode.BAL = (uint16_t)pfVal[i - usAddr];break;
				case eAfeRomCodeEUVR:g_stAfe.uRom.stCode.BAL = (uint16_t)pfVal[i - usAddr];break;
				case eAfeRomCodeOCRA:g_stAfe.uRom.stCode.BAL = (uint16_t)pfVal[i - usAddr];break;
				case eAfeRomCodeCTLC:g_stAfe.uRom.stCode.BAL = (uint16_t)pfVal[i - usAddr];break;
				case eAfeRomCodeDIS_PF:g_stAfe.uRom.stCode.BAL = (uint16_t)pfVal[i - usAddr];break;
				case eAfeRomCodeUV_OP:g_stAfe.uRom.stCode.BAL = (uint16_t)pfVal[i - usAddr];break;
				case eAfeRomCodeE0VB:g_stAfe.uRom.stCode.BAL = (uint16_t)pfVal[i - usAddr];break;
				case eAfeRomCodeOV:g_stAfe.uRom.stCode.BAL = (uint16_t)pfVal[i - usAddr];break;
				case eAfeRomCodeLDRT:g_stAfe.uRom.stCode.BAL = (uint16_t)pfVal[i - usAddr];break;
				case eAfeRomCodeOVT:g_stAfe.uRom.stCode.BAL = (uint16_t)pfVal[i - usAddr];break;
				case eAfeRomCodeOVR:g_stAfe.uRom.stCode.BAL = (uint16_t)pfVal[i - usAddr];break;
				case eAfeRomCodeUVT:g_stAfe.uRom.stCode.BAL = (uint16_t)pfVal[i - usAddr];break;
				case eAfeRomCodeUV:g_stAfe.uRom.stCode.BAL = (uint16_t)pfVal[i - usAddr];break;
				case eAfeRomCodeUVR:g_stAfe.uRom.stCode.BAL = (uint16_t)pfVal[i - usAddr];break;
				case eAfeRomCodeBALV:g_stAfe.uRom.stCode.BAL = (uint16_t)pfVal[i - usAddr];break;
				case eAfeRomCodePREV:g_stAfe.uRom.stCode.BAL = (uint16_t)pfVal[i - usAddr];break;
				case eAfeRomCodeL0V:g_stAfe.uRom.stCode.BAL = (uint16_t)pfVal[i - usAddr];break;
				case eAfeRomCodePFV:g_stAfe.uRom.stCode.BAL = (uint16_t)pfVal[i - usAddr];break;
				case eAfeRomCodeOCD1T:g_stAfe.uRom.stCode.BAL = (uint16_t)pfVal[i - usAddr];break;
				case eAfeRomCodeOCD1V:g_stAfe.uRom.stCode.BAL = (uint16_t)pfVal[i - usAddr];break;
				case eAfeRomCodeOCD2T:g_stAfe.uRom.stCode.BAL = (uint16_t)pfVal[i - usAddr];break;
				case eAfeRomCodeOCD2V:g_stAfe.uRom.stCode.BAL = (uint16_t)pfVal[i - usAddr];break;
				case eAfeRomCodeSCT:g_stAfe.uRom.stCode.BAL = (uint16_t)pfVal[i - usAddr];break;
				case eAfeRomCodeSCV:g_stAfe.uRom.stCode.BAL = (uint16_t)pfVal[i - usAddr];break;
				case eAfeRomCodeOCCT:g_stAfe.uRom.stCode.BAL = (uint16_t)pfVal[i - usAddr];break;
				case eAfeRomCodeOCCV:g_stAfe.uRom.stCode.BAL = (uint16_t)pfVal[i - usAddr];break;
				case eAfeRomCodePFT:g_stAfe.uRom.stCode.BAL = (uint16_t)pfVal[i - usAddr];break;
				case eAfeRomCodeOCRT:g_stAfe.uRom.stCode.BAL = (uint16_t)pfVal[i - usAddr];break;
				case eAfeRomCodeMOST:g_stAfe.uRom.stCode.BAL = (uint16_t)pfVal[i - usAddr];break;
				case eAfeRomCodeCHS:g_stAfe.uRom.stCode.BAL = (uint16_t)pfVal[i - usAddr];break;
				case eAfeRomCodeOTC:g_stAfe.uRom.stCode.BAL = (uint16_t)pfVal[i - usAddr];break;
				case eAfeRomCodeOTCR:g_stAfe.uRom.stCode.BAL = (uint16_t)pfVal[i - usAddr];break;
				case eAfeRomCodeUTC:g_stAfe.uRom.stCode.BAL = (uint16_t)pfVal[i - usAddr];break;
				case eAfeRomCodeUTCR:g_stAfe.uRom.stCode.BAL = (uint16_t)pfVal[i - usAddr];break;
				case eAfeRomCodeOTD:g_stAfe.uRom.stCode.BAL = (uint16_t)pfVal[i - usAddr];break;
				case eAfeRomCodeOTDR:g_stAfe.uRom.stCode.BAL = (uint16_t)pfVal[i - usAddr];break;
				case eAfeRomCodeUTD:g_stAfe.uRom.stCode.BAL = (uint16_t)pfVal[i - usAddr];break;
				case eAfeRomCodeUTDR:g_stAfe.uRom.stCode.BAL = (uint16_t)pfVal[i - usAddr];break;
				default:AFE309_RETURN_FALSE;
			}
		} else if(i >= eAfeRamCodeStart && i <= eAfeRamCodeEnd) {
			switch(i) {
				case eAfeRamCodeIDLE:g_stAfe.uRam.stCode.IDLE = (uint16_t)pfVal[i - usAddr];break;
				case eAfeRamCodeSLEEP:g_stAfe.uRam.stCode.SLEEP = (uint16_t)pfVal[i - usAddr];break;
				case eAfeRamCodeENWDT:g_stAfe.uRam.stCode.ENWDT = (uint16_t)pfVal[i - usAddr];break;
				case eAfeRamCodeCADCON:g_stAfe.uRam.stCode.CADCON = (uint16_t)pfVal[i - usAddr];break;
				case eAfeRamCodeCHGMOS:g_stAfe.uRam.stCode.CHGMOS = (uint16_t)pfVal[i - usAddr];break;
				case eAfeRamCodeDSGMOS:g_stAfe.uRam.stCode.DSGMOS = (uint16_t)pfVal[i - usAddr];break;
				case eAfeRamCodePCHMOS:g_stAfe.uRam.stCode.PCHMOS = (uint16_t)pfVal[i - usAddr];break;
				case eAfeRamCodeOCRC:g_stAfe.uRam.stCode.OCRC = (uint16_t)pfVal[i - usAddr];break;
				case eAfeRamCodeCB:
					g_stAfe.uRam.stCode.BALANCEH = (uint16_t)pfVal[i - usAddr] >> 8;
					g_stAfe.uRam.stCode.BALANCEL = (uint16_t)pfVal[i - usAddr] & 0xFF;
					break;
				//case eAfeRamCodeOV:g_stAfe.uRam.stCode.OV = (uint16_t)pfVal[i - usAddr];break;
				//case eAfeRamCodeUV:g_stAfe.uRam.stCode.UV = (uint16_t)pfVal[i - usAddr];break;
				//case eAfeRamCodeOCD1:g_stAfe.uRam.stCode.OCD1 = (uint16_t)pfVal[i - usAddr];break;
				//case eAfeRamCodeOCD2:g_stAfe.uRam.stCode.OCD2 = (uint16_t)pfVal[i - usAddr];break;
				//case eAfeRamCodeOCC:g_stAfe.uRam.stCode.OCC = (uint16_t)pfVal[i - usAddr];break;
				//case eAfeRamCodeSC:g_stAfe.uRam.stCode.SC = (uint16_t)pfVal[i - usAddr];break;
				//case eAfeRamCodePF:g_stAfe.uRam.stCode.PF = (uint16_t)pfVal[i - usAddr];break;
				//case eAfeRamCodeWDT:g_stAfe.uRam.stCode.WDT = (uint16_t)pfVal[i - usAddr];break;
				//case eAfeRamCodeUTC:g_stAfe.uRam.stCode.UTC = (uint16_t)pfVal[i - usAddr];break;
				//case eAfeRamCodeOTC:g_stAfe.uRam.stCode.OTC = (uint16_t)pfVal[i - usAddr];break;
				//case eAfeRamCodeUTD:g_stAfe.uRam.stCode.UTD = (uint16_t)pfVal[i - usAddr];break;
				//case eAfeRamCodeOTD:g_stAfe.uRam.stCode.OTD = (uint16_t)pfVal[i - usAddr];break;
				//case eAfeRamCodeDSG_FET:g_stAfe.uRam.stCode.DSG_FET = (uint16_t)pfVal[i - usAddr];break;
				//case eAfeRamCodeCHG_FET:g_stAfe.uRam.stCode.CHG_FET = (uint16_t)pfVal[i - usAddr];break;
				//case eAfeRamCodePCHG_FET:g_stAfe.uRam.stCode.PCHG_FET = (uint16_t)pfVal[i - usAddr];break;
				//case eAfeRamCodeL0V:g_stAfe.uRam.stCode.L0V = (uint16_t)pfVal[i - usAddr];break;
				//case eAfeRamCodeEEPR_WR:g_stAfe.uRam.stCode.EEPR_WR = (uint16_t)pfVal[i - usAddr];break;
				//case eAfeRamCodeDSGING:g_stAfe.uRam.stCode.DSGING = (uint16_t)pfVal[i - usAddr];break;
				//case eAfeRamCodeCHGING:g_stAfe.uRam.stCode.CHGING = (uint16_t)pfVal[i - usAddr];break;
				case eAfeRamCodeTEMP1:g_stAfe.uRam.stCode.TEMP[0] = (uint16_t)pfVal[i - usAddr];break;
				case eAfeRamCodeTEMP2:g_stAfe.uRam.stCode.TEMP[1] = (uint16_t)pfVal[i - usAddr];break;
				case eAfeRamCodeTEMP3:g_stAfe.uRam.stCode.TEMP[2] = (uint16_t)pfVal[i - usAddr];break;
				case eAfeRamCodeCUR:g_stAfe.uRam.stCode.CUR = (uint16_t)pfVal[i - usAddr];break;
				case eAfeRamCodeCELL1:g_stAfe.uRam.stCode.CELL[0]= (int16_t)pfVal[i - usAddr];break;
				case eAfeRamCodeCELL2:g_stAfe.uRam.stCode.CELL[1] = (int16_t)pfVal[i - usAddr];break;
				case eAfeRamCodeCELL3:g_stAfe.uRam.stCode.CELL[2] = (int16_t)pfVal[i - usAddr];break;
				case eAfeRamCodeCELL4:g_stAfe.uRam.stCode.CELL[3] = (int16_t)pfVal[i - usAddr];break;
				case eAfeRamCodeCELL5:g_stAfe.uRam.stCode.CELL[4] = (int16_t)pfVal[i - usAddr];break;
				case eAfeRamCodeCELL6:g_stAfe.uRam.stCode.CELL[5] = (int16_t)pfVal[i - usAddr];break;
				case eAfeRamCodeCELL7:g_stAfe.uRam.stCode.CELL[6] = (int16_t)pfVal[i - usAddr];break;
				case eAfeRamCodeCELL8:g_stAfe.uRam.stCode.CELL[7] = (int16_t)pfVal[i - usAddr];break;
				case eAfeRamCodeCELL9:g_stAfe.uRam.stCode.CELL[8] = (int16_t)pfVal[i - usAddr];break;
				case eAfeRamCodeCELL10:g_stAfe.uRam.stCode.CELL[9] = (int16_t)pfVal[i - usAddr];break;
				case eAfeRamCodeCELL11:g_stAfe.uRam.stCode.CELL[10] = (int16_t)pfVal[i - usAddr];break;
				case eAfeRamCodeCELL12:g_stAfe.uRam.stCode.CELL[11] = (int16_t)pfVal[i - usAddr];break;
				case eAfeRamCodeCELL13:g_stAfe.uRam.stCode.CELL[12] = (int16_t)pfVal[i - usAddr];break;
				case eAfeRamCodeCELL14:g_stAfe.uRam.stCode.CELL[13] = (int16_t)pfVal[i - usAddr];break;
				case eAfeRamCodeCELL15:g_stAfe.uRam.stCode.CELL[14] = (int16_t)pfVal[i - usAddr];break;
				case eAfeRamCodeCELL16:g_stAfe.uRam.stCode.CELL[15] = (int16_t)pfVal[i - usAddr];break;
				case eAfeRamCodeCADCD:g_stAfe.uRam.stCode.CDATA = (int16_t)pfVal[i - usAddr];break;
				case eAfeRamCodeOV_FLG:g_stAfe.uRam.stCode.OV_FLG = (uint16_t)pfVal[i - usAddr];break;
				case eAfeRamCodeUV_FLG:g_stAfe.uRam.stCode.UV_FLG = (uint16_t)pfVal[i - usAddr];break;
				case eAfeRamCodeOCD_FLG:g_stAfe.uRam.stCode.OCD_FLG = (uint16_t)pfVal[i - usAddr];break;
				case eAfeRamCodeLOAD_FLG:g_stAfe.uRam.stCode.LOAD_FLG = (uint16_t)pfVal[i - usAddr];break;
				case eAfeRamCodeOCC_FLG:g_stAfe.uRam.stCode.OCC_FLG = (uint16_t)pfVal[i - usAddr];break;
				case eAfeRamCodeSC_FLG:g_stAfe.uRam.stCode.SC_FLG = (uint16_t)pfVal[i - usAddr];break;
				case eAfeRamCodePF_FLG:g_stAfe.uRam.stCode.PF_FLG = (uint16_t)pfVal[i - usAddr];break;
				case eAfeRamCodeWDT_FLG:g_stAfe.uRam.stCode.WDT_FLG = (uint16_t)pfVal[i - usAddr];break;
				case eAfeRamCodeUTC_FLG:g_stAfe.uRam.stCode.UTC_FLG = (uint16_t)pfVal[i - usAddr];break;
				case eAfeRamCodeOTC_FLG:g_stAfe.uRam.stCode.OTC_FLG = (uint16_t)pfVal[i - usAddr];break;
				case eAfeRamCodeUTD_FLG:g_stAfe.uRam.stCode.UTD_FLG = (uint16_t)pfVal[i - usAddr];break;
				case eAfeRamCodeOTD_FLG:g_stAfe.uRam.stCode.OTD_FLG = (uint16_t)pfVal[i - usAddr];break;
				//case eAfeRamCodeVADC_FLG:g_stAfe.uRam.stCode.VADC_FLG = (uint16_t)pfVal[i - usAddr];break;
				//case eAfeRamCodeCADC_FLG:g_stAfe.uRam.stCode.CADC_FLG = (uint16_t)pfVal[i - usAddr];break;
				case eAfeRamCodeWAKE_FLG:g_stAfe.uRam.stCode.WAKE_FLG = (uint16_t)pfVal[i - usAddr];break;
				case eAfeRamCodeRST_FLG:g_stAfe.uRam.stCode.RST_FLG = (uint16_t)pfVal[i - usAddr];break;
				//case eAfeRamCodeWDT_CNT:g_stAfe.uRam.stCode.RSTSTAT = (uint16_t)pfVal[i - usAddr];break;
				default:AFE309_RETURN_FALSE;
			}
		} else if(i >= eAfeRomAppStart && i <= eAfeRomAppEnd) {
			switch(i) {
				case eAfeRomAppCTLC:g_stAfe.uRom.stCode.CTLC = (uint16_t)pfVal[i - usAddr];break;
				case eAfeRomAppCN:g_stAfe.uRom.stCode.CN = (uint16_t)pfVal[i - usAddr];break;
				case eAfeRomAppLDRT:
					if(100 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.LDRT = 0;break;}
					if(500 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.LDRT = 1;break;}
					if(1000 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.LDRT = 2;break;}
					if(2000 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.LDRT = 3;break;}
					AFE309_RETURN_FALSE;
				case eAfeRomAppOV:g_stAfe.uRom.stCode.OV = (uint16_t)(pfVal[i - usAddr] - g_stCfg.stAfe.fPackVolCali / g_stAfe.stRomApp.usCN) / 5;break;
				case eAfeRomAppOVT:
					if(100 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.OVT = 0;break;}
					if(200 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.OVT = 1;break;}
					if(300 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.OVT = 2;break;}
					if(400 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.OVT = 3;break;}
					if(600 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.OVT = 4;break;}
					if(800 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.OVT = 5;break;}
					if(1000 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.OVT = 6;break;}
					if(2000 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.OVT = 7;break;}
					if(3000 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.OVT = 8;break;}
					if(4000 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.OVT = 9;break;}
					if(6000 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.OVT = 10;break;}
					if(8000 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.OVT = 11;break;}
					if(10000 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.OVT = 12;break;}
					if(20000 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.OVT = 13;break;}
					if(30000 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.OVT = 14;break;}
					if(40000 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.OVT = 15;break;}
					AFE309_RETURN_FALSE;
				case eAfeRomAppOVR:g_stAfe.uRom.stCode.OVR = (uint16_t)(pfVal[i - usAddr] - g_stCfg.stAfe.fPackVolCali / g_stAfe.stRomApp.usCN) / 5;break;
				case eAfeRomAppUV:g_stAfe.uRom.stCode.UV = (uint16_t)(pfVal[i - usAddr] - g_stCfg.stAfe.fPackVolCali / g_stAfe.stRomApp.usCN) / 20;break;
				case eAfeRomAppUVT:
					if(100 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.UVT = 0;break;}
					if(200 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.UVT = 1;break;}
					if(300 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.UVT = 2;break;}
					if(400 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.UVT = 3;break;}
					if(600 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.UVT = 4;break;}
					if(800 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.UVT = 5;break;}
					if(1000 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.UVT = 6;break;}
					if(2000 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.UVT = 7;break;}
					if(3000 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.UVT = 8;break;}
					if(4000 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.UVT = 9;break;}
					if(6000 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.UVT = 10;break;}
					if(8000 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.UVT = 11;break;}
					if(10000 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.UVT = 12;break;}
					if(20000 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.UVT = 13;break;}
					if(30000 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.UVT = 14;break;}
					if(40000 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.UVT = 15;break;}
					AFE309_RETURN_FALSE;
				case eAfeRomAppUVR:g_stAfe.uRom.stCode.UVR = (uint16_t)(pfVal[i - usAddr] - g_stCfg.stAfe.fPackVolCali / g_stAfe.stRomApp.usCN) / 20;break;
				case eAfeRomAppBALV:g_stAfe.uRom.stCode.BALV = (uint16_t)pfVal[i - usAddr] / 20;break;
				case eAfeRomAppPREV:g_stAfe.uRom.stCode.PREV = (uint16_t)pfVal[i - usAddr] / 20;break;
				case eAfeRomAppL0V:g_stAfe.uRom.stCode.L0V = (uint16_t)(pfVal[i - usAddr] - g_stCfg.stAfe.fPackVolCali / g_stAfe.stRomApp.usCN) / 20;break;
				case eAfeRomAppPFV:g_stAfe.uRom.stCode.PFV = (uint16_t)(pfVal[i - usAddr] - g_stCfg.stAfe.fPackVolCali / g_stAfe.stRomApp.usCN) / 20;break;
				case eAfeRomAppOCD1T:
					if(50 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.OCD1T = 0;break;}
					if(100 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.OCD1T = 1;break;}
					if(200 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.OCD1T = 2;break;}
					if(400 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.OCD1T = 3;break;}
					if(600 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.OCD1T = 4;break;}
					if(800 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.OCD1T = 5;break;}
					if(1000 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.OCD1T = 6;break;}
					if(2000 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.OCD1T = 7;break;}
					if(4000 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.OCD1T = 8;break;}
					if(6000 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.OCD1T = 9;break;}
					if(8000 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.OCD1T = 10;break;}
					if(10000 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.OCD1T = 11;break;}
					if(15000 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.OCD1T = 12;break;}
					if(20000 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.OCD1T = 13;break;}
					if(30000 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.OCD1T = 14;break;}
					if(40000 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.OCD1T = 15;break;}
					AFE309_RETURN_FALSE;
				case eAfeRomAppOCD1V:
					if(20 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.OCD1V = 0;break;}
					if(30 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.OCD1V = 1;break;}
					if(40 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.OCD1V = 2;break;}
					if(50 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.OCD1V = 3;break;}
					if(60 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.OCD1V = 4;break;}
					if(70 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.OCD1V = 5;break;}
					if(80 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.OCD1V = 6;break;}
					if(90 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.OCD1V = 7;break;}
					if(100 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.OCD1V = 8;break;}
					if(110 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.OCD1V = 9;break;}
					if(120 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.OCD1V = 10;break;}
					if(130 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.OCD1V = 11;break;}
					if(140 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.OCD1V = 12;break;}
					if(160 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.OCD1V = 13;break;}
					if(180 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.OCD1V = 14;break;}
					if(200 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.OCD1V = 15;break;}
					AFE309_RETURN_FALSE;
				case eAfeRomAppOCD2T:
					if(10 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.OCD2T = 0;break;}
					if(20 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.OCD2T = 1;break;}
					if(40 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.OCD2T = 2;break;}
					if(60 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.OCD2T = 3;break;}
					if(80 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.OCD2T = 4;break;}
					if(100 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.OCD2T = 5;break;}
					if(200 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.OCD2T = 6;break;}
					if(400 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.OCD2T = 7;break;}
					if(600 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.OCD2T = 8;break;}
					if(800 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.OCD2T = 9;break;}
					if(1000 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.OCD2T = 10;break;}
					if(2000 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.OCD2T = 11;break;}
					if(4000 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.OCD2T = 12;break;}
					if(8000 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.OCD2T = 13;break;}
					if(10000 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.OCD2T = 14;break;}
					if(20000 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.OCD2T = 15;break;}
					AFE309_RETURN_FALSE;
				case eAfeRomAppOCD2V:
					if(30 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.OCD2V = 0;break;}
					if(40 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.OCD2V = 1;break;}
					if(50 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.OCD2V = 2;break;}
					if(60 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.OCD2V = 3;break;}
					if(70 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.OCD2V = 4;break;}
					if(80 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.OCD2V = 5;break;}
					if(90 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.OCD2V = 6;break;}
					if(100 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.OCD2V = 7;break;}
					if(120 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.OCD2V = 8;break;}
					if(140 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.OCD2V = 9;break;}
					if(160 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.OCD2V = 10;break;}
					if(180 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.OCD2V = 11;break;}
					if(200 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.OCD2V = 12;break;}
					if(300 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.OCD2V = 13;break;}
					if(400 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.OCD2V = 14;break;}
					if(500 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.OCD2V = 15;break;}
					AFE309_RETURN_FALSE;
				case eAfeRomAppSCT:
					if(0 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.SCT = 0;break;}
					if(64 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.SCT = 1;break;}
					if(128 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.SCT = 2;break;}
					if(192 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.SCT = 3;break;}
					if(256 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.SCT = 4;break;}
					if(320 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.SCT = 5;break;}
					if(384 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.SCT = 6;break;}
					if(448 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.SCT = 7;break;}
					if(512 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.SCT = 8;break;}
					if(576 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.SCT = 9;break;}
					if(640 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.SCT = 10;break;}
					if(704 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.SCT = 11;break;}
					if(768 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.SCT = 12;break;}
					if(832 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.SCT = 13;break;}
					if(896 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.SCT = 14;break;}
					if(960 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.SCT = 15;break;}
					AFE309_RETURN_FALSE;
				case eAfeRomAppSCV:
					if(50 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.SCV = 0;break;}
					if(80 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.SCV = 1;break;}
					if(110 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.SCV = 2;break;}
					if(140 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.SCV = 3;break;}
					if(170 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.SCV = 4;break;}
					if(200 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.SCV = 5;break;}
					if(230 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.SCV = 6;break;}
					if(260 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.SCV = 7;break;}
					if(290 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.SCV = 8;break;}
					if(320 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.SCV = 9;break;}
					if(350 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.SCV = 10;break;}
					if(400 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.SCV = 11;break;}
					if(500 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.SCV = 12;break;}
					if(600 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.SCV = 13;break;}
					if(800 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.SCV = 14;break;}
					if(1000 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.SCV = 15;break;}
					AFE309_RETURN_FALSE;
				case eAfeRomAppOCCT:
					if(10 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.OCCT = 0;break;}
					if(20 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.OCCT = 1;break;}
					if(40 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.OCCT = 2;break;}
					if(60 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.OCCT = 3;break;}
					if(80 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.OCCT = 4;break;}
					if(100 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.OCCT = 5;break;}
					if(200 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.OCCT = 6;break;}
					if(400 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.OCCT = 7;break;}
					if(600 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.OCCT = 8;break;}
					if(800 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.OCCT = 9;break;}
					if(1000 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.OCCT = 10;break;}
					if(2000 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.OCCT = 11;break;}
					if(4000 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.OCCT = 12;break;}
					if(8000 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.OCCT = 13;break;}
					if(10000 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.OCCT = 14;break;}
					if(20000 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.OCCT = 15;break;}
					AFE309_RETURN_FALSE;
				case eAfeRomAppOCCV:
					if(20 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.OCCV = 0;break;}
					if(30 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.OCCV = 1;break;}
					if(40 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.OCCV = 2;break;}
					if(50 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.OCCV = 3;break;}
					if(60 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.OCCV = 4;break;}
					if(70 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.OCCV = 5;break;}
					if(80 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.OCCV = 6;break;}
					if(90 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.OCCV = 7;break;}
					if(100 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.OCCV = 8;break;}
					if(110 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.OCCV = 9;break;}
					if(120 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.OCCV = 10;break;}
					if(130 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.OCCV = 11;break;}
					if(140 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.OCCV = 12;break;}
					if(160 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.OCCV = 13;break;}
					if(180 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.OCCV = 14;break;}
					if(200 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.OCCV = 15;break;}
					AFE309_RETURN_FALSE;
				case eAfeRomAppPFT:
					if(8 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.PFT = 0;break;}
					if(16 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.PFT = 1;break;}
					if(32 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.PFT = 2;break;}
					if(64 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.PFT = 3;break;}
					AFE309_RETURN_FALSE;
				case eAfeRomAppOCRT:
					if(8 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.OCRT = 0;break;}
					if(16 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.OCRT = 1;break;}
					if(32 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.OCRT = 2;break;}
					if(64 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.OCRT = 3;break;}
					AFE309_RETURN_FALSE;
				case eAfeRomAppMOST:
					if(64 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.MOST = 0;break;}
					if(128 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.MOST = 1;break;}
					if(256 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.MOST = 2;break;}
					if(512 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.MOST = 3;break;}
					AFE309_RETURN_FALSE;
				case eAfeRomAppCHS:
					if(200 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.CHS = 0;break;}
					if(500 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.CHS = 1;break;}
					if(1000 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.CHS = 2;break;}
					if(2000 == (uint16_t)pfVal[i - usAddr]) {g_stAfe.uRom.stCode.CHS = 3;break;}
					AFE309_RETURN_FALSE;
				case eAfeRomAppOTC:g_stAfe.uRom.stCode.OTC = afe_temp2ohms(pfVal[i - usAddr] - g_stCfg.stAfe.fAvgTempCali) / (g_stAfe.stRomApp.fTR + afe_temp2ohms(pfVal[i - usAddr] - g_stCfg.stAfe.fAvgTempCali)) * 512;break;
				case eAfeRomAppOTCR:g_stAfe.uRom.stCode.OTCR = afe_temp2ohms(pfVal[i - usAddr] - g_stCfg.stAfe.fAvgTempCali) / (g_stAfe.stRomApp.fTR + afe_temp2ohms(pfVal[i - usAddr] - g_stCfg.stAfe.fAvgTempCali)) * 512;break;
				case eAfeRomAppUTC:g_stAfe.uRom.stCode.UTC = (afe_temp2ohms(pfVal[i - usAddr] - g_stCfg.stAfe.fAvgTempCali) / (g_stAfe.stRomApp.fTR + afe_temp2ohms(pfVal[i - usAddr] - g_stCfg.stAfe.fAvgTempCali)) - 0.5) * 512;break;
				case eAfeRomAppUTCR:g_stAfe.uRom.stCode.UTCR = (afe_temp2ohms(pfVal[i - usAddr] - g_stCfg.stAfe.fAvgTempCali) / (g_stAfe.stRomApp.fTR + afe_temp2ohms(pfVal[i - usAddr] - g_stCfg.stAfe.fAvgTempCali)) - 0.5) * 512;break;
				case eAfeRomAppOTD:g_stAfe.uRom.stCode.OTD = afe_temp2ohms(pfVal[i - usAddr] - g_stCfg.stAfe.fAvgTempCali) / (g_stAfe.stRomApp.fTR + afe_temp2ohms(pfVal[i - usAddr] - g_stCfg.stAfe.fAvgTempCali)) * 512;break;
				case eAfeRomAppOTDR:g_stAfe.uRom.stCode.OTDR = afe_temp2ohms(pfVal[i - usAddr] - g_stCfg.stAfe.fAvgTempCali) / (g_stAfe.stRomApp.fTR + afe_temp2ohms(pfVal[i - usAddr] - g_stCfg.stAfe.fAvgTempCali)) * 512;break;
				case eAfeRomAppUTD:g_stAfe.uRom.stCode.UTD = (afe_temp2ohms(pfVal[i - usAddr] - g_stCfg.stAfe.fAvgTempCali) / (g_stAfe.stRomApp.fTR + afe_temp2ohms(pfVal[i - usAddr] - g_stCfg.stAfe.fAvgTempCali)) - 0.5) * 512;break;
				case eAfeRomAppUTDR:g_stAfe.uRom.stCode.UTDR = (afe_temp2ohms(pfVal[i - usAddr] - g_stCfg.stAfe.fAvgTempCali) / (g_stAfe.stRomApp.fTR + afe_temp2ohms(pfVal[i - usAddr] - g_stCfg.stAfe.fAvgTempCali)) - 0.5) * 512;break;
				//case eAfeRomAppTR:g_stAfe.uRom.stCode.TR = (uint16_t)pfVal[i - usAddr];break;
				default:AFE309_RETURN_FALSE;
			}
		} else if(i >= eAfeRamAppStart && i <= eAfeRamAppEnd) {
			switch(i) {
				//case eAfeRamTEMP1:
				//case eAfeRamTEMP2:
				//case eAfeRamTEMP3:
				//case eAfeRamCUR:
				//case eAfeRamCELL1:
				//case eAfeRamCELL2:
				//case eAfeRamCELL3:
				//case eAfeRamCELL4:
				//case eAfeRamCELL5:
				//case eAfeRamCELL6:
				//case eAfeRamCELL7:
				//case eAfeRamCELL8:
				//case eAfeRamCELL9:
				//case eAfeRamCELL10:
				//case eAfeRamCELL11:
				//case eAfeRamCELL12:
				//case eAfeRamCELL13:
				//case eAfeRamCELL14:
				//case eAfeRamCELL15:
				//case eAfeRamCELL16:
				//case eAfeRamCDATA:
				//case eAfeRamWDT:
				default:
					AFE309_RETURN_FALSE;
			}
		} else if(i >= eAfeCmdStart && i <= eAfeCmdEnd) {
			switch(i) {
				case eAfeCmdCali:if(!afe_calibrate(pfVal[i - usAddr])) {AFE309_RETURN_FALSE;}break;
				default:AFE309_RETURN_FALSE;
			}
		} else {
			AFE309_RETURN_FALSE;
		}
	}
	if(!afe_set_rom()) {
		AFE309_RETURN_FALSE;
	}
	if(!afe_set_ram()) {
		AFE309_RETURN_FALSE;
	}
	
	AFE309_RETURN_TRUE;
}

/*******************************************************************************
* Function Name  : afe_get_di
* Description    : Read variables by enumeration and bit offset
* Input          : usAddr,	Enumeration address * 8 + bit offset
* Output         : pbVal,	Read the variable results
* Return         : result, 1 write succeeded, 0 write failed
*******************************************************************************/
bool afe_get_di(uint16_t usAddr, bool* pbVal) {
	if(0 == pbVal) {
		AFE309_RETURN_FALSE;
	}
	if(/*usAddr >= eAfeRomByteStart && */usAddr <= eAfeRomByteEnd * 8) {                                              //change
		*pbVal = (bool)(g_stAfe.uRom.aucByte[usAddr / 8 - eAfeRomByteStart + 1 - usAddr / 8 % 2 * 2] >> (usAddr % 8) & 0x01);
		AFE309_RETURN_TRUE;
	}
	if(usAddr >= eAfeRamByteStart * 8 && usAddr <= eAfeRamByteEnd * 8) {                                             //change
		*pbVal = (bool)(g_stAfe.uRam.aucByte[usAddr / 8 - eAfeRamByteStart + 1 - usAddr / 8 % 2 * 2] >> (usAddr % 8) & 0x01);
		AFE309_RETURN_TRUE;
	}
	
	AFE309_RETURN_FALSE;
}

/*******************************************************************************
* Function Name  : afe_set_do
* Description    : Write variables by enumeration and bit offset
* Input          : usAddr,	Enumeration address * 8 + bit offset
								 : bVal,	Variables that need to be written
* Output         : None
* Return         : result, 1 write succeeded, 0 write failed
*******************************************************************************/
bool afe_set_do(uint16_t usAddr, bool bVal) {
	if(/*usAddr >= eAfeRamByteStart && */usAddr < eAfeRomByteEnd * 8) {
		if(0 == bVal) {
			g_stAfe.uRom.aucByte[usAddr / 8 - eAfeRomByteStart + 1 - usAddr / 8 % 2 * 2] &= ~(1 << (usAddr % 8));
		} else {
			g_stAfe.uRom.aucByte[usAddr / 8 - eAfeRomByteStart + 1 - usAddr / 8 % 2 * 2] |= (bVal << (usAddr % 8));
		}
	}
	if(usAddr >= eAfeRamByteStart * 8 && usAddr < eAfeRamByteEnd * 8) {
		if(0 == bVal) {
			g_stAfe.uRam.aucByte[usAddr / 8 - eAfeRamByteStart + 1 - usAddr / 8 % 2 * 2] &= ~(1 << (usAddr % 8));
		} else {
			g_stAfe.uRam.aucByte[usAddr / 8 - eAfeRamByteStart + 1 - usAddr / 8 % 2 * 2] |= (bVal << (usAddr % 8));  
		}
	}
	if(!afe_set_rom()) {
		AFE309_RETURN_FALSE;
	}
	if(!afe_set_ram()) {
		AFE309_RETURN_FALSE;
	}
	
	AFE309_RETURN_TRUE;
}

/*******************************************************************************
* Function Name  : afe_init
* Description    : Initialize SH367309 chip Initialize the I2C pin
* Input          : None
* Output         : None
* Return         : result £∫1 write succeeded, 0 write failed
*******************************************************************************/
bool afe_init(void) {
	g_stCfg.stAfe.aucRomByte[1] = 0x40;
	g_stCfg.stAfe.aucRomByte[0] = 0x0E;
	g_stCfg.stAfe.aucRomByte[3] = 0x6B;   	/* OV trigger 4.4V */
	g_stCfg.stAfe.aucRomByte[2] = 0x70;
	g_stCfg.stAfe.aucRomByte[5] = 0x63;   	/* OV recover 4.2V */
	g_stCfg.stAfe.aucRomByte[4] = 0x48;
	g_stCfg.stAfe.aucRomByte[7] = 0x32;   	/* UV trigger 1.0V */
	g_stCfg.stAfe.aucRomByte[6] = 0x3C;  	  /* UV recover 1.2V */
	g_stCfg.stAfe.aucRomByte[9] = 0xA5;
	g_stCfg.stAfe.aucRomByte[8] = 0x7D;
	g_stCfg.stAfe.aucRomByte[11] = 0x64;
	g_stCfg.stAfe.aucRomByte[10] = 0xE1; 	/* OV Secondary triggering 4.5V */
	g_stCfg.stAfe.aucRomByte[13] = 0xF2;	/* OCD1,100,150,200,250,300,350,400,450,500,550,600,650,700,800,900,1000A;					50,100,200,400,600,800,1000,2000,4000,6000,8000,10000,15000,20000,30000,40000ms */
	g_stCfg.stAfe.aucRomByte[12] = 0xC2;	/* OCD2,150,200,250,300,350,400,450,500,600,700,800,900,1000,1500,2000,2500A;				10,20,40,60,80,100,200,400,600,800,1000,2000,4000,8000,10000,20000ms */
	g_stCfg.stAfe.aucRomByte[15] = 0xBF;	/* ‘≠1000A 0x5F œ÷2000A*/ /* SC , 250,400,550,700,850,1000,1150,1300,1450,1600,1750,2000,2500,3000,4000,5000A;0,64,128,192,256,320,384,448,512,576,640,704,768,832,896,960us */
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
	memcpy(g_stAfe.uRom.aucByte, g_stCfg.stAfe.aucRomByte, CFG_AFE_ROM_BLEN);
	if(!afe_set_rom()) {
		AFE309_RETURN_FALSE;
	}
	float fVal=1;
	afe_set_ao(eAfeRamCodeCADCON, 1, &fVal);  /* Turn on the current detection switch */
	delay_1ms(250);
	fVal = 0;
	afe_set_ao(eAfeRamCodePCHMOS, 1, &fVal);	/* Turn off the AFE pre-charge loop */
	afe_set_ao(eAfeRamCodeCHGMOS, 1, &fVal);	/* Turn off the AFE charging loop */
	afe_set_ao(eAfeRamCodeDSGMOS, 1, &fVal);	/* Close the AFE discharge circuit */
	if(!afe_get_rom()) {
		AFE309_RETURN_FALSE;
	}
	if(!afe_get_ram()) {
		AFE309_RETURN_FALSE;
	}
	ResetAFE();
	cfg_save();	
	AFE309_RETURN_TRUE;
}

/*******************************************************************************
* Function Name  : afe_proc
* Description    : This function is the only interface for AFE rotation in the main loop
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
bool afe_proc(void) {
	static uint32_t uiTick = 0;
	uiTick++;
	if(uiTick * g_stCfg.stLocal.usCyclePeriod < 250) {
		AFE309_RETURN_TRUE;
	}
	uiTick = 0;
	if(!afe_get_ram()) {
		AFE309_RETURN_FALSE;
	}
	
	AFE309_RETURN_TRUE;
}

/*------------------------------------/   
	* @name   Bat_Balance(unsigned char nChannel)
	* @brief  Note that adjacent battery strings cannot simultaneously turn on equalization.
            Balancing needs to be activated based on the actual state of the project, and users need to activate it according to their own needs
	* @param  None
	* @retval None      
/------------------------------------*/   


void Bat_Balance(unsigned char nChannel)
{
		switch(nChannel)
		{
			case 0:  afe_set_do(528,1);break;
			case 1:  afe_set_do(529,1);break;
			case 2:  afe_set_do(530,1);break;
			case 3:  afe_set_do(531,1);break;
			case 4:  afe_set_do(532,1);break;
			case 5:  afe_set_do(533,1);break;
			case 6:  afe_set_do(534,1);break;
			case 7:  afe_set_do(535,1);break;
			case 8:  afe_set_do(520,1);break;
			case 9:  afe_set_do(521,1);break;
			case 10: afe_set_do(522,1);break;
			case 11: afe_set_do(523,1);break;
			case 12: afe_set_do(524,1);break;
			case 13: afe_set_do(525,1);break;
			case 14: afe_set_do(526,1);break;
			case 15: afe_set_do(527,1);break;
			default:
				break ;
		}
}

/*******************************************************************************
Function:ResetAFE() 
Description:  Reset SH367309 IC, Send Data:0xEA, 0xC0, CRC
Input:	 NULL
Output: NULL
Others:
*******************************************************************************/
void ResetAFE(void)
{
	uint8_t ucVal = 0xC0;
	if(g_stAfe.stRamApp.fTEMP[0] == 110 || g_stAfe.stRamApp.fTEMP[1] == 110 || g_stAfe.stRamApp.fTEMP[2] == 110){
	  MTP_Write309(AFE_ID, 0xEA, &ucVal, 1, 5);
	  __set_PRIMASK(1);   //Turn off all interruptions
	  NVIC_SystemReset(); //Perform a software reset
	}
	for(uint8_t i=0;i<10;i++){
		if(g_stAfe.stRamApp.fCDATA > 100000 || g_stAfe.stRamApp.fCDATA < -100000) {
			MTP_Write309(AFE_ID, 0xEA, &ucVal, 1, 5);
		  delay_1ms(250);
		  afe_get_ram();
		}else{
			g_stCfg.stAfe.fCDATACaliB -= g_stAfe.stRamApp.fCDATA;
 			break;
		}
	}
}

/*******************************************************************************
Function:ResetAFE1() 
Description:  Reset SH367309 IC, Send Data:0xEA, 0xC0, CRC
Input:	 NULL
Output: NULL
Others:
*******************************************************************************/
void ResetAFE1(void)
{
	uint8_t ucVal = 0xC0;
	if(g_stAfe.stRamApp.fTEMP[0] == 110 || g_stAfe.stRamApp.fTEMP[1] == 110 || g_stAfe.stRamApp.fTEMP[2] == 110){
	  MTP_Write309(AFE_ID, 0xEA, &ucVal, 1, 5);
	  __set_PRIMASK(1);   //Turn off all interruptions
	  NVIC_SystemReset(); //Perform a software reset
	}
}
