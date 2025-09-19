#ifndef _BSP_SH367303_H
#define	_BSP_SH367303_H

#include <stdbool.h>
#include <stdint.h>
#include "config.h"
#include "main.h"

#ifdef USER_DEBUG
#define AFE303_DEBUG_EN	1		/* 0: non-DEBUG state, 1: DEBUG status */
#else
#define AFE303_DEBUG_EN 0
#endif
#define AFE303_DEBUG(fmt,arg...)	do{if(AFE303_DEBUG_EN){printf(fmt,##arg);printf("[%s][%d]\r\n",__func__,__LINE__);}}while(0)
#define AFE303_RETURN_FALSE	do{AFE303_DEBUG("Return failed");return false;}while(0)
//#define AFE303_RETURN_TRUE do{AFE303_DEBUG("Return success");return true;}while(0)
#define AFE303_RETURN_TRUE do{return true;}while(0)

#define TRY_TIMES 3				//Number of errors
#define AFE_ID	0x1B      //Slave SH367303 address

//SH367303 system flag status control registers
#pragma pack(1)

#define AFE_RAM_BLEN	(0x002C - 0x000D)

typedef enum {
	/* 0.1.ROM BYTE */
	eAfeRomByteStart = 0,
    eAfeRomByteRWStart = 0x03,
	eAfeRomByteEnd = eAfeRomByteStart + CFG_AFE_ROM_BLEN - 1,
	/* 0.2.RAM BYTE */
	eAfeRamByteStart = 0x000E,
    //eAfeRamByteStart = ;
	eAfeRamByteEnd = eAfeRamByteStart + AFE_RAM_BLEN - 1,
	/* 1.1.ROM CODE,  54in total */
	eAfeRomCodeStart = 0x0030,

    eAfeRomCodeTWI = eAfeRomCodeStart,
    eAfeRomCodeWDT,
    eAfeRomCodeOV,
    eAfeRomCodeSC,
    
    eAfeRomCodeCHGR,
    eAfeRomCodeLOAD,
    eAfeRomCodeCHGING,
    eAfeRomCodeDSGING,
    eAfeRomCodeCHG,
    eAfeRomCodeDSG,

    eAfeRomCodeTWI_INT,
    eAfeRomCodeWDT_INT,
    eAfeRomCodeVADC_INT,
    eAfeRomCodeCADC_INT,
    eAfeRomCodeCD_INT,
    eAfeRomCodeOV_INT,
    eAfeRomCodeSC_INT,
    
    eAfeRomCodeCHGR_EN,
    eAfeRomCodeLOAD_EN,
    eAfeRomCodeOV_EN,
    eAfeRomCodeSC_EN,
    eAfeRomCodeWDT_EN,
    eAfeRomCodePD_EN,
    eAfeRomCodeCTLD_EN,
    eAfeRomCodeLTCLR,
    eAfeRomCodeCHG_C,
    eAfeRomCodeDSG_C,
    eAfeRomCodeALARM_C,
    eAfeRomCodeRESET_PF,
    
    eAfeRomCodeSCAN_C,
    eAfeRomCodeVADC_C,
    eAfeRomCodeVADC_EN,
    eAfeRomCodeCBIT_C,
    eAfeRomCodeCADC_M,
    eAfeRomCodeCADC_EN,
    
    eAfeRomCodeCB6,
    eAfeRomCodeCB7,
    eAfeRomCodeCB8,
    eAfeRomCodeCB9,
    eAfeRomCodeCB10,
    
    eAfeRomCodeCB1,
    eAfeRomCodeCB2,
    eAfeRomCodeCB3,
    eAfeRomCodeCB4,
    eAfeRomCodeCB5,
    
    eAfeRomCodeSCT,
    eAfeRomCodeSCV,
    eAfeRomCodeRST,
    eAfeRomCodeRSNS,
    
    eAfeRomCodeWDTT,
    eAfeRomCodeCHS,
    eAfeRomCodeOVT,
    
    eAfeRomCodeOVD1,
    eAfeRomCodeOVD2,
    eAfeRomCodePIN,
    

    /* Temperature internal reference resistivity */
	eAfeRomCodeEnd = eAfeRomCodePIN,
	/* 1.2.RAM CODE,  in total */
	eAfeRamCodeStart = 0x0070,
	/* CELL1 */
	eAfeRamCodeCELL1 = eAfeRamCodeStart,
	/* CELL2 */
	eAfeRamCodeCELL2,
	/* CELL3 */
	eAfeRamCodeCELL3,
	/* CELL4 */
	eAfeRamCodeCELL4,
    /* TS1 */
	eAfeRamCodeTS1,
	/* TS2 */
	eAfeRamCodeTS2,
    
	/* TEMP1 */
	eAfeRamCodeTEMP1,

	/* CUR */
	eAfeRamCodeCUR,
	
	eAfeRamCodeEnd = eAfeRamCodeCUR,
	/* 2.1.ROM APP,  in total */
	eAfeRomAppStart = 0x0080,
    
    
    eAfeRomAppSCT = eAfeRomAppStart,
    eAfeRomAppSCV,
    eAfeRomAppRST,
    eAfeRomAppRSNS,
    eAfeRomAppWDTT,
    eAfeRomAppCHS,
    eAfeRomAppOVT,
    eAfeRomAppOVD,
    eAfeRomAppPIN,

	eAfeRomAppEnd = eAfeRomAppPIN,
	eAfeRomAppLen = eAfeRomAppEnd - eAfeRomAppStart + 1,
	/* 2.2.RAM APP, 22 in total */
	eAfeRamAppStart = 0x090,
    eAfeRamCELL1 = eAfeRamAppStart,
	eAfeRamCELL2,
	eAfeRamCELL3,
	eAfeRamCELL4,
    
    eAfeRamTS1,
    eAfeRamTS2,
    eAfeRamTEMP1,
    eAfeRamCUR,
    
	eAfeRamAppEnd = eAfeRamCUR,
	eAfeRamLen = eAfeRamAppEnd - eAfeRamAppStart + 1,
	/* 3.1.Control commands, 1 in total */
	eAfeCmdStart = 0x01C0,
	eAfeCmdCali,
	eAfeCmdReserve = eAfeCmdCali + 0x0100,
	eAfeCmdEnd,
	eAfeEnd = eAfeCmdEnd
} AFE_E;

typedef struct {
    //FLAG1
    uint16_t TWI:1;
    uint16_t WDT:1;
    uint16_t OV:1;
    uint16_t SC:1;
    uint16_t Reserved2:4;
    //FLAG2
    uint16_t VADC:1;                    /* VADC中断标志位 */ 
                                        /* 0：未发生过中断  */
                                        /* 1：发生过VADC中断，该bit被读取之后，硬件会自动清零*/
    uint16_t CADC:1;
    uint16_t RSTFLAG:1;
    uint16_t Reserved:5;

    //BSTATUS
    uint16_t CHGR:1;
    uint16_t LOAD:1;
    uint16_t CHGING:1;
    uint16_t DSGING:1;
    uint16_t CHG:1;
    uint16_t DSG:1;
    uint16_t Reserved4:2;
    //INT_EN
    uint16_t TWI_INT:1;
    uint16_t WDT_INT:1;
    uint16_t VADC_INT:1;
    uint16_t SCVADC_INT:1;
    uint16_t CD_INT:1;
    uint16_t OV_INT:1;
    uint16_t SC_INT:1;
    uint16_t Reserved3:1;

    //SCONF1
    uint16_t CHGR_EN:1;
    uint16_t LOAD_EN:1;
    uint16_t OV_EN:1;
    uint16_t SC_EN:1;
    uint16_t WDT_EN:1;
    uint16_t PD_EN:1;
    uint16_t CTLD_EN:1;
    uint16_t LTCLR:1;
    //SCONF2
    uint16_t CHG_G:1;
    uint16_t DSG_G:1;
    uint16_t ALARM_C:1;
    uint16_t RESET_PT:1;
    uint16_t Reserved5:4;

    //SCONF3
    uint16_t SCAN_C:3;
    uint16_t VADC_C:1;
    uint16_t VADC_EN:1;
    uint16_t CBIT_C:1;
    uint16_t CADC_M:1;
    uint16_t CADC_EN:1;
    //SCONF4
    uint16_t CB6:1;
    uint16_t CB7:1;
    uint16_t CB8:1;
    uint16_t CB9:1;
    uint16_t CB10:1;
    uint16_t Reserved6:3;

    //SCONF5
    uint16_t CB1:1;
    uint16_t CB2:1;
    uint16_t CB3:1;
    uint16_t CB4:1;
    uint16_t CB5:1;
    uint16_t Reserved7:3;
    //SCONF6
    uint16_t SCT:2;
    uint16_t SCV:2;
    uint16_t RST:2;   
    uint16_t RSNS:2;

    //SCONF7
    uint16_t WDTT:2;
    uint16_t CHS:2;
    uint16_t OVT:3;   
    uint16_t Reserved9:1;
//    //SCONF8+SCONF9
//    uint16_t OVD:10;
//    uint16_t Reserved8:6;
    //SCONF8+SCONF9
    uint16_t OVD1:2;
    uint16_t Reserved8:6;
    uint16_t OVD2:8;
    //SCONF10
    uint16_t PIN:8;

    
} AFE_ROM_CODE_S;

typedef struct {
    int16_t  CELL[10];	    /* Cell XX measurements,  this value is a signed 16-bit */
    uint16_t TS[2];			/*外部温度检测值 */
    uint16_t TEMP[2];         /*内部温度检测值*/
	int16_t  CUR;	
} AFE_RAM_CODE_S;//RAM

typedef union {
	AFE_ROM_CODE_S stCode;
	uint8_t aucByte[CFG_AFE_ROM_BLEN];	/* Due to the size of the side, the byte order stored here is reversed from the byte order in the chip, and the internal high and low bytes are interchanged in 2-byte bits */
} AFE_ROM_U;

typedef union {
	AFE_RAM_CODE_S stCode;
	uint8_t aucByte[AFE_RAM_BLEN];	/* Due to the size of the side, the byte order stored here is reversed from the byte order in the chip, and the internal high and low bytes are interchanged in 2-byte bits */
} AFE_RAM_U;

typedef struct {
    uint16_t usSCT;
    uint16_t usSCV;
    uint16_t usRST;
    uint16_t usRSNS;
    uint16_t usWDTT;
    uint16_t usCHS;
    uint16_t usOVT;
    float    usOVD;        /* 单位: mV */ 
    uint16_t PIN;
    
} AFE_ROM_APP_S;

typedef struct {
    float  fCELL[4];	    /* Cell XX measurements,  this value is a signed 16-bit */
    float  fTS[2];			/*外部温度检测值 */
    float  fTEMP;         /*内部温度检测值*/
	float  fCUR;	
} AFE_RAM_APP_S;

typedef struct {
	AFE_ROM_U uRom;
	AFE_RAM_U uRam;
	AFE_ROM_APP_S stRomApp;
	AFE_RAM_APP_S stRamApp;
} AFE_S;

extern AFE_S g_stAfe;
static void afe_delay(uint16_t ms);
static uint8_t CRC8Cal(const uint8_t* pucBuf, uint8_t ucLength);
static bool IIC303_ReadData(uint8_t ucDevAdd, uint8_t ucRegAdd, uint8_t* const pucData);
static bool IIC303_WriteData(uint8_t ucDevAdd, uint8_t ucRegAdd, uint8_t ucData);
static bool MTP_Write303(uint8_t ucDevAdd, uint8_t ucRegAdd, uint8_t* const pucData, uint8_t ucDatLen, uint8_t ucCount);
static bool MTP_Read303(uint8_t ucDevAdd, uint8_t ucRegAdd, uint8_t* const pucData, uint8_t ucDatLen, uint8_t ucCount);
static bool MTP_SetVPRO(bool bVal);
static float afe_ohms2temp(float fOhms);
static float afe_temp2ohms(float fTemp);
static bool afe_calibrate(uint16_t usMask);
static bool afe_get_rom(void);
extern bool afe_set_rom(void);
extern bool afe_get_ram(void);
static bool afe_set_ram(void);
extern bool afe_get_ai(uint16_t usAddr, float* pfVal);
extern bool afe_set_ao(uint16_t usAddr, uint8_t ucCnt, float* pfVal);
extern bool afe_get_di(uint16_t usAddr, bool* pbVal);
extern bool afe_set_do(uint16_t usAddr, bool bVal);
extern bool afe_init(void);
extern bool afe_proc(void);
extern void afe_pch2chg(void);
extern void Bat_Balance(unsigned char nChannel);
static void ResetAFE(void);

#pragma pack()

#endif /* _BSP__SH367303_H */
