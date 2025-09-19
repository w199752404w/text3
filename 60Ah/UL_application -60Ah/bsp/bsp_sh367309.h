#ifndef _BSP_SH367309_H
#define	_BSP_SH367309_H

#include <stdbool.h>
#include <stdint.h>
#include "config.h"
#include "main.h"

#ifdef USER_DEBUG
#define AFE309_DEBUG_EN	0		/* 0: non-DEBUG state, 1: DEBUG status */
#else
#define AFE309_DEBUG_EN 0
#endif
#define AFE309_DEBUG(fmt,arg...)	do{if(AFE309_DEBUG_EN){printf(fmt,##arg);printf("[%s][%d]\r\n",__func__,__LINE__);}}while(0)
#define AFE309_RETURN_FALSE	do{AFE309_DEBUG("Return failed");return false;}while(0)
//#define AFE309_RETURN_TRUE do{AFE309_DEBUG("Return success");return true;}while(0)
#define AFE309_RETURN_TRUE do{return true;}while(0)

#define TRY_TIMES 3				//Number of errors
#define AFE_ID	0x1A      //Slave SH367309 address

//SH367309 system flag status control registers
#pragma pack(1)

#define AFE_RAM_BLEN	(0x0073 - 0x0040)

typedef enum {
	/* 0.1.ROM BYTE */
	eAfeRomByteStart = 0,
	eAfeRomByteEnd = eAfeRomByteStart + CFG_AFE_ROM_BLEN - 1,
	/* 0.2.RAM BYTE */
	eAfeRamByteStart = 0x0040,
	eAfeRamByteEnd = eAfeRamByteStart + AFE_RAM_BLEN - 1,
	/* 1.1.ROM CODE, 43 in total */
	eAfeRomCodeStart = 0x0080,
	/* SCONF1 */
	eAfeRomCodeCN = eAfeRomCodeStart,	/* The number of strings is configured with control bits. Register values of 5~15 correspond to 5~15 strings, and other values correspond to 16 strings */
	eAfeRomCodeBAL,										/* The balance function module enables the control bit.
																			0: The balance function is turned on and controlled by the AFE's internal logic; 1: The balance on is controlled by the external MCU, and the balance timing is controlled by the AFE's internal logic */
	eAfeRomCodeOCPM,									/* Charge/discharge overcurrent MOS control bit.
																			0: Charge overcurrent only turn off the charging MOS, discharge overcurrent only turn off the discharge MOS; 1: Charge and discharge overcurrent to turn off charge and discharge MOS */
	eAfeRomCodeENMOS,									/* Charge MOS to restore the control bit.
																			0: Disable charging MOS to restore the control bit; 1: Enable the charging MOS recovery control bit, when the overcurrent charging/temperature protection turns off the charging MOS,
																			If a charge overcurrent 1 or discharge state is detected, the charge MOS is turned on */
	eAfeRomCodeENPCH,									/* Pre-charge module control bit. 0: Disable the pre-charge function; 1: Enable the pre-charge function */
	/* SCONF2 */
	eAfeRomCodeEUVR,									/* Over-discharge recovery sets the control bit. 0: The release of the over-discharge protection state has nothing to do with the load release; 1: The release of the over-discharge protection state also needs to release the load */
	eAfeRomCodeOCRA,									/* The current recovery sets the control bits. 0: Current protection timing recovery is not allowed; 1: Allow current protection timing to resume */
	eAfeRomCodeCTLC,									/* The CTL pin function sets the control bit. 
																			00: The charge-discharge and pre-charge MOS are controlled by internal logic, and the CTL pin input is invalid; 
																			01: Control charging and pre-charging MOS. The CTL input VL-CTL is forced to turn off the charging and precharging MOS at the level
																				The charging and pre-charging MOS at the CTL input VH-CTL level are controlled by internal logic; 
																			10: Control the discharge MOS. The CTL input VL-CTL level forcibly shuts down the discharge MOS
																				CTL input VH-CTL level, discharge MOS controlled by internal logic; 
																			11: Control charge, discharge and precharge MOS. The CTL input VL-CTL level forcibly turns off the charge-discharge and pre-charge MOS
																				When the CTL input VH-CTL level, the charge/discharge and pre-charge MOS are controlled by internal logic */
	eAfeRomCodeDIS_PF,								/* The secondary overcharge module enables the control bit. 0: Enable secondary overcharge protection; 1: No secondary overcharge protection */
	eAfeRomCodeUV_OP,									/* MOS control bit during overdischarge.
																			0: Over-discharge only turns off the discharge MOS; 1: Over-discharge prayer to turn off the charge and discharge MOS */
	eAfeRomCodeE0VB,									/* It is forbidden to set the control position of the charging function of the low-voltage cell. 0: Disable the "Disable low-voltage cell charging" function; 1: Turn on the "Disable low-voltage cell charging" function */
	/* OV/LDRT/OVT */
	eAfeRomCodeOV,										/* Overcharge protection voltage, calculated as register value X 5mV */
	eAfeRomCodeLDRT,									/* The load release delay is set to the control bit.
																			00: Load release delay = 100ms; 01: Load release delay = 500ms; 10: Load release delay = 1000ms; 11: Load release delay = 2000 ms */
	eAfeRomCodeOVT,										/* Overcharge protection delay set control bit.
																			0000: Overcharge protection delay = 100ms; 0001: Overcharge protection delay = 200ms;
																			0010: Overcharge protection delay = 300ms; 0011: Overcharge protection delay = 400ms;
																			0100: Overcharge protection delay = 600ms; 0101: Overcharge protection delay = 800ms;
																			0110: Overcharge protection delay = 1000ms; 0111: Overcharge protection delay = 2000ms;
																			1000: Overcharge protection delay = 3000ms; 1001: Overcharge protection delay = 4000ms;
																			1010: Overcharge protection delay = 6000ms; 1011: Overcharge protection delay = 8000ms;
																			1100: Overcharge protection delay = 10000ms; 1101: Overcharge protection delay = 20000ms;
																			1110: Overcharge protection delay = 30000ms; 1111: Overcharge protection delay = 40000ms */
	/* OVR/UVT */
	eAfeRomCodeOVR,										/* Overcharge recovery voltage, calculated as register value X 5mV */
	eAfeRomCodeUVT,										/* Over-discharge protection delay set control bit. 
																			0000: Over-discharge protection delay = 100ms; 0001: Over-discharge protection delay = 200ms;
																			0010: Over-discharge protection delay = 300ms; 0011: Over-discharge protection delay = 400ms;
																			0100: Over-discharge protection delay = 600ms; 0101: Over-discharge protection delay = 800ms;
																			0110: Over-discharge protection delay = 1000ms; 0111: Over-discharge protection delay = 2000ms;
																			1000: Over-discharge protection delay = 3000ms; 1001: Over-discharge protection delay = 4000ms;
																			1010: Over-discharge protection delay = 6000ms; 1011: Over-discharge protection delay = 8000ms;
																			1100: Over-discharge protection delay = 10000ms; 1101: Over-discharge protection delay = 20000ms;
																			1110: Over-discharge protection delay = 30000ms; 1111: Over-discharge protection delay = 40000ms */
	/* UV */
	eAfeRomCodeUV,										/* Over-discharge protection voltage, calculated by registering value X 20mV */
	/* UVR */
	eAfeRomCodeUVR,										/* Over-discharge recovery voltage, calculated as register value X 20mV */
	/* BALV */
	eAfeRomCodeBALV,									/* Balanced turn-on voltage, calculated as register value X 20mV */
	/* PREV */
	eAfeRomCodePREV,									/* Pre-charge voltage, calculated by registering value X 20mV */
	/* L0V */
	eAfeRomCodeL0V,										/* Low voltage prohibition charge voltage, calculated as: register value X 20mV */
	/* PFV */
	eAfeRomCodePFV,										/* Secondary overcharge protection voltage, calculated as register value X 20mV */
	/* OCD1V/OCD1T */
	eAfeRomCodeOCD1T,									/* The control position is set for the protection delay of the discharge overcurrent 1.
																			0000: Discharge overcurrent 1 protection delay = 50ms; 0001: Discharge overcurrent 1 protection delay = 100ms;
																			0010: Discharge overcurrent 1 protection delay = 200ms; 0011: Discharge overcurrent 1 protection delay = 400ms;
																			0100: Discharge overcurrent 1 protection delay = 600ms; 0101: Discharge overcurrent 1 protection delay = 800ms;
																			0110: Discharge overcurrent 1 protection delay = 1000ms; 0111: Discharge overcurrent 1 protection delay = 2000ms;
																			1000: Discharge overcurrent 1 protection delay = 4000ms; 1001: Discharge overcurrent 1 protection delay = 6000ms;
																			1010: Discharge overcurrent 1 protection delay = 8000ms; 1011: Discharge overcurrent 1 protection delay = 10000ms;
																			1100: Discharge overcurrent 1 protection delay = 15000ms; 1101: Discharge overcurrent 1 protection delay = 20000ms;
																			1110: Discharge overcurrent 1 protection delay = 30000ms; 1111: Discharge overcurrent 1 protection delay = 40000ms */
	eAfeRomCodeOCD1V,									/* Discharge overcurrent 1 protection voltage set control bit.
																			0000: Discharge overcurrent 1 protection voltage = 20mV; 0001: Discharge overcurrent 1 protection voltage = 30mV;
																			0010: Discharge overcurrent 1 protection voltage = 40mV; 0011: Discharge overcurrent 1 protection voltage = 50mV;
																			0100: Discharge overcurrent 1 protection voltage = 60mV; 0101: Discharge overcurrent 1 protection voltage = 70mV;
																			0110: Discharge overcurrent 1 protection voltage = 80mV; 0111: Discharge overcurrent 1 protection voltage = 90mV;
																			1000: Discharge overcurrent 1 protection voltage = 100mV; 1001: Discharge overcurrent 1 protection voltage = 110mV;
																			1010: Discharge overcurrent 1 protection voltage = 120mV; 1011: Discharge overcurrent 1 protection voltage = 130mV;
																			1100: Discharge overcurrent 1 protection voltage = 140mV; 1101: Discharge overcurrent 1 protection voltage = 160mV;
																			1110: Discharge overcurrent 1 protection voltage = 180mV; 1111: Discharge overcurrent 1 protection voltage = 200mV */
	/* OCD2V/OCD2T */
	eAfeRomCodeOCD2T,									/* Discharge overcurrent 2 protection delay set control bit.
																			0000: Discharge overcurrent 2 protection delay = 10ms; 0001: Discharge overcurrent 2 protection delay = 20ms;
																			0010: Discharge overcurrent 2 protection delay = 40ms; 0011: Discharge overcurrent 2 protection delay = 60ms;
																			0100: Discharge overcurrent 2 protection delay = 80ms; 0101: Discharge overcurrent 2 protection delay = 100ms;
																			0110: Discharge overcurrent 2 protection delay = 200ms; 0111: Discharge overcurrent 2 protection delay = 400ms;
																			1000: Discharge overcurrent 2 protection delay = 600ms; 1001: Discharge overcurrent 2 protection delay = 800ms;
																			1010: Discharge overcurrent 2 protection delay = 1000ms; 1011: Discharge overcurrent 2 protection delay = 2000ms;
																			1100: Discharge overcurrent 2 protection delay = 4000ms; 1101: Discharge overcurrent 2 protection delay = 8000ms;
																			1110: Discharge overcurrent 2 protection delay = 10000ms; 1111: Discharge overcurrent 2 protection delay = 20000ms */
	eAfeRomCodeOCD2V,									/* Discharge overcurrent 2 protection voltage set control bit.
																			0000: Discharge overcurrent 2 protection voltage = 30mV; 0001: Discharge overcurrent 2 protection voltage = 40mV;
																			0010: Discharge overcurrent 2 protection voltage = 50mV; 0011: Discharge Overcurrent 2 Protection Voltage = 60mV;
																			0100: Discharge overcurrent 2 protection voltage = 70mV; 0101: Discharge overcurrent 2 protection voltage = 80mV;
																			0110: Discharge overcurrent 2 protection voltage = 90mV; 0111: Discharge overcurrent 2 protection voltage = 100mV;
																			1000: Discharge overcurrent 2 protection voltage = 120mV; 1001: Discharge overcurrent 2 protection voltage = 140mV;
																			1010: Discharge overcurrent 2 protection voltage = 160mV; 1011: Discharge overcurrent 2 protection voltage = 180mV;
																			1100: Discharge overcurrent 2 protection voltage = 200mV; 1101: Discharge overcurrent 2 protection voltage = 300mV;
																			1110: Discharge overcurrent 2 protection voltage = 400mV; 1111: Discharge overcurrent 2 protection voltage = 500mV */
	/* SCT/SCV */
	eAfeRomCodeSCT,										/* The short-circuit protection delay is set to the control bit.
																			0000: short-circuit protection delay = 0us; 0001: Short Circuit Protection Delay = 64us;
																			0010: Short circuit protection delay = 128us; 0011: Short Circuit Protection Delay = 192us;
																			0100: Short circuit protection delay = 256us; 0101: Short Circuit Protection Delay = 320us;
																			0110: Short Circuit Protection Delay = 384us; 0111: Short Circuit Protection Delay = 448us;
																			1000: short-circuit protection delay = 512us; 1001: Short Circuit Protection Delay = 576us;
																			1010: short-circuit protection delay = 640us; 1011: Short Circuit Protection Delay = 704us;
																			1100: Short Circuit Protection Delay = 768us; 1101: Short Circuit Protection Delay = 832us;
																			1110: Short Circuit Protection Delay = 896us; 1111: Short-circuit protection delay = 960us */
	eAfeRomCodeSCV,										/* The short-circuit protection voltage is set to the control bit.
																			0000: short-circuit protection voltage = 50mV; 0001: Short Circuit Protection Voltage = 80mV;
																			0010: Short Circuit Protection Voltage = 110mV; 0011: Short Circuit Protection Voltage = 140mV;
																			0100: Short Circuit Protection Voltage = 170mV; 0101: Short Circuit Protection Voltage = 200mV;
																			0110: Short Circuit Protection Voltage = 230mV; 0111: Short Circuit Protection Voltage = 260mV;
																			1000: short-circuit protection voltage = 290mV; 1001: short-circuit protection voltage = 320mV;
																			1010: short-circuit protection voltage = 350mV; 1011: Short Circuit Protection Voltage = 400mV;
																			1100: short-circuit protection voltage = 500mV; 1101: Short Circuit Protection Voltage = 600mV;
																			1110: Short Circuit Protection Voltage = 800mV; 1111: Short-circuit protection voltage = 1000mV */
	/* OCCT/OCCV */
	eAfeRomCodeOCCT,									/* The charging overcurrent protection delay is set to the control bit.
																			0000: Charging overcurrent protection delay = 10ms; 0001: Charging overcurrent protection delay = 20ms;
																			0010: Charging overcurrent protection delay = 40ms; 0011: Charging overcurrent protection delay = 60ms;
																			0100: Charging overcurrent protection delay = 80ms; 0101: Charging overcurrent protection delay = 100ms;
																			0110: Charging overcurrent protection delay = 200ms; 0111: Charging overcurrent protection delay = 400ms;
																			1000: Charging overcurrent protection delay = 600ms; 1001: Charging overcurrent protection delay = 800ms;
																			1010: Charging overcurrent protection delay = 1000ms; 1011: Charging overcurrent protection delay = 2000ms;
																			1100: Charging overcurrent protection delay = 4000ms; 1101: Charging overcurrent protection delay = 8000ms;
																			1110: Charging over-current protection delay = 10000ms; 1111: Charging overcurrent protection delay = 20000ms */
	eAfeRomCodeOCCV,									/* The charging overcurrent protection voltage is set to the control bit.
																			0000: Charging overcurrent protection voltage = 20mV; 0001: Charge overcurrent protection voltage = 30mV;
																			0010: Charge overcurrent protection voltage = 40mV; 0011: Charge overcurrent protection voltage = 50mV;
																			0100: Charge overcurrent protection voltage = 60mV; 0101: Charge overcurrent protection voltage = 70mV;
																			0110: Charge overcurrent protection voltage = 80mV; 0111: Charge overcurrent protection voltage = 90mV;
																			1000: Charging overcurrent protection voltage = 100mV; 1001: Charge overcurrent protection voltage = 110mV;
																			1010: Charge overcurrent protection voltage = 120mV; 1011: Charge overcurrent protection voltage = 130mV;
																			1100: Charging overcurrent protection voltage = 140mV; 1101: Charge overcurrent protection voltage = 160mV;
																			1110: Charge overcurrent protection voltage = 180mV; 1111: Charge overcurrent protection voltage = 200mV */
	/* PFT/OCRT/MOST/CHS */
	eAfeRomCodePFT,										/* The delay setting of the secondary overcharge protection.
																			00: Secondary overcharge protection delay = 8s; 01: Secondary overcharge protection delay = 16s;
																			10: Secondary overcharge protection delay = 32s; 11: Secondary overcharge protection delay = 64s */
	eAfeRomCodeOCRT,									/* Charge-discharge over-current self-recovery delay setting.
																			00: Charge/discharge overcurrent auto-recovery delay = 8s; 01: Charge-discharge over-current auto-recovery delay = 16s;
																			10: Charge-discharge over-current auto-recovery delay = 32s; 11: Charge-discharge over-current auto-recovery delay = 64s */
	eAfeRomCodeMOST,									/* Charge and discharge MOS enable delay setting.
																			00: Charge-discharge MOS opening delay = 64us; 01: Charge/discharge MOSFET turn-on delay = 128us;
																			10: Charge/discharge MOS opening delay = 256us; 11: Charge-discharge MOS turn-on delay = 512us */
	eAfeRomCodeCHS,										/* Charge and discharge status detection voltage setting.
																			00: Charge and discharge state detection voltage = 200uV; 01: Charge and discharge state detection voltage=500uV;
																			10: Charge-discharge state detection voltage=1000uV; 11: Charge-discharge state detection voltage = 2000uV */
	/* OTC */
	eAfeRomCodeOTC,										/* Charging high temperature protection threshold */
	/* OTCR */
	eAfeRomCodeOTCR,									/* Charging high temperature protection release threshold */
	/* UTC */
	eAfeRomCodeUTC,										/* Charging low temperature protection threshold */
	/* UTCR */
	eAfeRomCodeUTCR,									/* Charging Low Temperature Protection Release Threshold */
	/* OTD */
	eAfeRomCodeOTD,										/* Discharge high temperature protection threshold */
	/* OTDR */
	eAfeRomCodeOTDR,									/* Discharge high temperature protection release threshold */
	/* UTD */
	eAfeRomCodeUTD,										/* Discharge cryogenic protection threshold */
	/* UTDR */
	eAfeRomCodeUTDR,									/* Discharge cryogenic protection release threshold */
	/* TR */
	eAfeRomCodeTR,										/* Temperature internal reference resistivity */
	eAfeRomCodeEnd = eAfeRomCodeTR,
	/* 1.2.RAM CODE, 66 in total */
	eAfeRamCodeStart = 0x00C0,
	/* CONF */
	eAfeRamCodeIDLE = eAfeRamCodeStart,
	eAfeRamCodeSLEEP,
	eAfeRamCodeENWDT,
	eAfeRamCodeCADCON,
	eAfeRamCodeCHGMOS,
	eAfeRamCodeDSGMOS,
	eAfeRamCodePCHMOS,
	eAfeRamCodeOCRC,
	/* BALANCE */
	eAfeRamCodeCB,
	/* BSTATUS1 */
	eAfeRamCodeOV,
	eAfeRamCodeUV,
	eAfeRamCodeOCD1,
	eAfeRamCodeOCD2,
	eAfeRamCodeOCC,
	eAfeRamCodeSC,
	eAfeRamCodePF,
	eAfeRamCodeWDT,
	/* BSTATUS2 */
	eAfeRamCodeUTC,
	eAfeRamCodeOTC,
	eAfeRamCodeUTD,
	eAfeRamCodeOTD,
	/* BSTATUS3 */
	eAfeRamCodeDSG_FET,
	eAfeRamCodeCHG_FET,
	eAfeRamCodePCHG_FET,
	eAfeRamCodeL0V,
	eAfeRamCodeEEPR_WR,
	eAfeRamCodeDSGING,
	eAfeRamCodeCHGING,
	/* TEMP1 */
	eAfeRamCodeTEMP1,
	/* TEMP2 */
	eAfeRamCodeTEMP2,
	/* TEMP3 */
	eAfeRamCodeTEMP3,
	/* CUR */
	eAfeRamCodeCUR,
	/* CELL1 */
	eAfeRamCodeCELL1,
	/* CELL2 */
	eAfeRamCodeCELL2,
	/* CELL3 */
	eAfeRamCodeCELL3,
	/* CELL4 */
	eAfeRamCodeCELL4,
	/* CELL5 */
	eAfeRamCodeCELL5,
	/* CELL6 */
	eAfeRamCodeCELL6,
	/* CELL7 */
	eAfeRamCodeCELL7,
	/* CELL8 */
	eAfeRamCodeCELL8,
	/* CELL9 */
	eAfeRamCodeCELL9,
	/* CELL10 */
	eAfeRamCodeCELL10,
	/* CELL11 */
	eAfeRamCodeCELL11,
	/* CELL12 */
	eAfeRamCodeCELL12,
	/* CELL13 */
	eAfeRamCodeCELL13,
	/* CELL14 */
	eAfeRamCodeCELL14,
	/* CELL15 */
	eAfeRamCodeCELL15,
	/* CELL16 */
	eAfeRamCodeCELL16,
	/* CADCD */
	eAfeRamCodeCADCD,
	/* BFLAG1 */
	eAfeRamCodeOV_FLG,
	eAfeRamCodeUV_FLG,
	eAfeRamCodeOCD_FLG,
	eAfeRamCodeLOAD_FLG,
	eAfeRamCodeOCC_FLG,
	eAfeRamCodeSC_FLG,
	eAfeRamCodePF_FLG,
	eAfeRamCodeWDT_FLG,
	/* BFLAG2 */
	eAfeRamCodeUTC_FLG,
	eAfeRamCodeOTC_FLG,
	eAfeRamCodeUTD_FLG,
	eAfeRamCodeOTD_FLG,
	eAfeRamCodeVADC_FLG,
	eAfeRamCodeCADC_FLG,
	eAfeRamCodeWAKE_FLG,
	eAfeRamCodeRST_FLG,
	eAfeRamCodeWDT_CNT,
	eAfeRamCodeEnd = eAfeRamCodeWDT_CNT,
	/* 2.1.ROM APP, 34 in total */
	eAfeRomAppStart = 0x0140,
	eAfeRomAppCTLC = eAfeRomAppStart,
	eAfeRomAppCN,
	eAfeRomAppLDRT,
	eAfeRomAppOV,
	eAfeRomAppOVT,
	eAfeRomAppOVR,
	eAfeRomAppUV,
	eAfeRomAppUVT,
	eAfeRomAppUVR,
	eAfeRomAppBALV,
	eAfeRomAppPREV,
	eAfeRomAppL0V,
	eAfeRomAppPFV,
	eAfeRomAppOCD1T,
	eAfeRomAppOCD1V,
	eAfeRomAppOCD2T,
	eAfeRomAppOCD2V,
	eAfeRomAppSCT,
	eAfeRomAppSCV,
	eAfeRomAppOCCT,
	eAfeRomAppOCCV,
	eAfeRomAppPFT,
	eAfeRomAppOCRT,
	eAfeRomAppMOST,
	eAfeRomAppCHS,
	eAfeRomAppOTC,
	eAfeRomAppOTCR,
	eAfeRomAppUTC,
	eAfeRomAppUTCR,
	eAfeRomAppOTD,
	eAfeRomAppOTDR,
	eAfeRomAppUTD,
	eAfeRomAppUTDR,
	eAfeRomAppTR,
	eAfeRomAppEnd = eAfeRomAppTR,
	eAfeRomAppLen = eAfeRomAppEnd - eAfeRomAppStart + 1,
	/* 2.2.RAM APP, 22 in total */
	eAfeRamAppStart = 0x0180,
	eAfeRamTEMP1 = eAfeRamAppStart,
	eAfeRamTEMP2,
	eAfeRamTEMP3,
	eAfeRamCUR,
	eAfeRamCELL1,
	eAfeRamCELL2,
	eAfeRamCELL3,
	eAfeRamCELL4,
	eAfeRamCELL5,
	eAfeRamCELL6,
	eAfeRamCELL7,
	eAfeRamCELL8,
	eAfeRamCELL9,
	eAfeRamCELL10,
	eAfeRamCELL11,
	eAfeRamCELL12,
	eAfeRamCELL13,
	eAfeRamCELL14,
	eAfeRamCELL15,
	eAfeRamCELL16,
	eAfeRamCDATA,
	eAfeRamWDT,
	eAfeRamAppEnd = eAfeRamWDT,
	eAfeRamLen = eAfeRamAppEnd - eAfeRamAppStart + 1,
	/* 3.1.Control commands, 1 in total */
	eAfeCmdStart = 0x01C0,
	eAfeCmdCali,
	eAfeCmdReserve = eAfeCmdCali + 0x0100,
	eAfeCmdEnd,
	eAfeEnd = eAfeCmdEnd
} AFE_E;

typedef struct {
	uint16_t EUVR:1;			/* The over-discharge recovery sets the control bit, 0: the over-discharge protection state release has nothing to do with the load release, 1: the over-discharge protection state release also needs the load release */
	uint16_t OCRA:1;			/* The current recovery is set to the control bit, 0: the current protection is not allowed to be restored at a time, and 1: the current protection is allowed to be recovered at a timed basis */
	uint16_t CTLC:2;			/* The CTL pin function sets the control bit */
												/* 00: The charge-discharge and pre-charge MOSFETs are controlled by internal logic, and the CTL pin input is invalid */
												/* 01: Control charge and precharge MOSFETs. The charge and precharge MOSFETs are forcibly turned off at the CTL input VL-CTL level, and the charge and precharge MOSFETs are controlled by internal logic at the CTL input VH-CTL level */
												/* 10: Control discharge MOSFET. The CTL input VL-CTL level forcibly closes the discharge MOSFET, and the CTL input VH-CTL level, and the discharge MOSFET is controlled by internal logic */
												/* 11: Control charge, discharge and precharge MOSFETs. The charge/discharge and pre-charge MOSGET are forcibly turned off at the CTL input VL-CTL level, and the charge/discharge and pre-charge MOSFETs are controlled by internal logic at the CTL input VH-CTL level */
	uint16_t DIS_PF:1;		/* The secondary overcharge module enables the control bit */
												/* 0: Enables secondary overcharge protection */
												/* 1: It is forbidden to protect the secondary overcharge */
	uint16_t UV_OP:1;			/* MOSFET control bit during overdischarge */
												/* 0: Over-discharge only turns off the discharge MOSFET */
												/* 1: Over-discharge turns off the charge-discharge MOSFET */
	uint16_t Reserved1:1;	/* spare */
	uint16_t E0VB:1;			/* It is forbidden to set the control position of the charging function of the low-voltage cell */
												/* 0: Disable the "Disable low-voltage cell charging" function */
												/* 1: Turn on the function of "Disable low-voltage cell charging". */
	uint16_t CN:4;				/* The string configuration controls the control bits */
	uint16_t BAL:1;				/* The balance function module enables the control bit */
												/* 0: The balance is enabled and controlled by the internal logic of the SH367309 */
												/* 1: The balance is controlled by the external MCU, and the balance timing is still controlled by the SH367309's internal logic */
	uint16_t OCPM:1;			/* Charge/discharge overcurrent MOSFET control bit */
												/* 0: Charge overcurrent only turns off the charging MOSFET, and discharge overcurrent only turns off the discharge MOSFET */
												/* 1: Charge and discharge overcurrent to turn off the charge and discharge MOSFET */
	uint16_t ENMOS:1;			/* The charging MOSFET restores the control bit */
												/* 0: Disable the charging MOSFET to restore the control bit */
												/* 1: Start the charging MOSFET to restore the control bit, when the overcharge/temperature protection turns off the charging MOSFET, if the discharge overcurrent or discharge state is detected, the charging MOSFET will be turned on */
	uint16_t ENPCH:1;			/* Pre-charge module control bit */
												/* 0: The pre-charging function is disabled */
												/* 1: Enable the pre-charge function */
	uint16_t OV:10;				/* Overcharge protection voltage, calculated as register value X 5mV */
	uint16_t LDRT:2;			/* The load release delay is set to the control bit */
												/* 00: Load release delay = 100 ms */
												/* 01: Load release delay = 500 ms */
												/* 10: Load release delay = 1000 ms */
												/* 11: Load release delay = 2000 ms */
	uint16_t OVT:4;				/* Overcharge protection delay set control bit */
												/* 0000: Overcharge protection delay = 100ms */
												/* 0001: Overcharge protection delay = 200ms */
												/* 0010: Overcharge protection delay = 300ms */
												/* 0011: Overcharge protection delay = 400ms */
												/* 0100: Overcharge protection delay = 600ms */
												/* 0101: Overcharge protection delay = 800ms */
												/* 0110: Overcharge protection delay = 1000ms */
												/* 0111: Overcharge protection delay = 2000ms */
												/* 1000: Overcharge protection delay = 3000ms */
												/* 1001: Overcharge protection delay = 4000ms */
												/* 1010: Overcharge protection delay = 6000ms */
												/* 1011: Overcharge protection delay = 8000ms */
												/* 1100: Overcharge protection delay = 10000ms */
												/* 1101: Overcharge protection delay = 20000ms */
												/* 1110: Overcharge protection delay = 30000ms */
												/* 1111: Overcharge protection delay = 40000ms */
	uint16_t OVR:10;			/* Overcharge recovery voltage, calculated as register value X 5mV */
	uint16_t Reserved2:2;	/* spare */
	uint16_t UVT:4;				/* Over-discharge protection delay set control bit */
												/* 0000: Over-discharge protection delay = 100ms */
												/* 0001: Over-discharge protection delay = 200ms */
												/* 0010: Over-discharge protection delay = 300ms */
												/* 0011: Over-discharge protection delay = 400ms */
												/* 0100: Over-discharge protection delay = 600ms */
												/* 0101: Over-discharge protection delay = 800ms */
												/* 0110: Over-discharge protection delay = 1000ms */
												/* 0111: Over-discharge protection delay = 2000ms */
												/* 1000: Over-discharge protection delay = 3000ms */
												/* 1001: Over-discharge protection delay = 4000ms */
												/* 1010: Over-discharge protection delay = 6000ms */
												/* 1011: Over-discharge protection delay = 8000ms */
												/* 1100: Over-discharge protection delay = 10000ms */
												/* 1101: Over-discharge protection delay = 20000ms */
												/* 1110: Over-discharge protection delay = 30000ms */
												/* 1111: Over-discharge protection delay = 40000ms */
	uint16_t UVR:8;				/* Over-discharge recovery voltage, calculated as register value X 20mV */
	uint16_t UV:8;				/* Over-discharge protection voltage, calculated by registering value X 20mV */
	uint16_t PREV:8;			/* Pre-charge voltage, calculated by registering value X 20mV */
	uint16_t BALV:8;			/* Balanced turn-on voltage, calculated as register value X 20mV */
	uint16_t PFV:8;				/* Secondary overcharge protection voltage, calculated as register value X 20mV */
	uint16_t L0V:7;				/* Low voltage prohibition charge voltage, calculated as: register value X 20mV */
	uint16_t Reserved3:1;	/* spare */
	uint16_t OCD2T:4;			/* Discharge overcurrent 2 protection delay set control bit */
												/* 0000: Discharge overcurrent 2 protection delay = 10ms */
												/* 0001: Discharge overcurrent 2 protection delay = 20ms */
												/* 0010: Discharge overcurrent 2 protection delay = 40ms */
												/* 0011: Discharge overcurrent 2 protection delay = 60ms */
												/* 0100: Discharge overcurrent 2 protection delay = 80ms */
												/* 0101: Discharge overcurrent 2 protection delay = 100ms */
												/* 0110: Discharge overcurrent 2 protection delay = 200ms */
												/* 0111: Discharge overcurrent 2 protection delay = 400ms */
												/* 1000: Discharge overcurrent 2 protection delay = 600ms */
												/* 1001: Discharge overcurrent 2 protection delay = 800ms */
												/* 1010: Discharge overcurrent 2 protection delay = 1000ms */
												/* 1011: Discharge overcurrent 2 protection delay = 2000ms */
												/* 1100: Discharge overcurrent 2 protection delay = 4000ms */
												/* 1101: Discharge overcurrent 2 protection delay = 8000ms */
												/* 1110: Discharge overcurrent 2 protection delay = 10000ms */
												/* 1111: Discharge overcurrent 2 protection delay = 20000ms */
	uint16_t OCD2V:4;			/* Discharge overcurrent 2 protection voltage set control bit */
												/* 0000: Discharge overcurrent 2 protection voltage = 30mV */
												/* 0001: Discharge overcurrent 2 protection voltage = 40mV */
												/* 0010: Discharge overcurrent 2 protection voltage = 50mV */
												/* 0011: Discharge overcurrent 2 protection voltage = 60mV */
												/* 0100: Discharge overcurrent 2 protection voltage = 70mV */
												/* 0101: Discharge overcurrent 2 protection voltage = 80mV */
												/* 0110: Discharge overcurrent 2 protection voltage = 90mV */
												/* 0111: Discharge overcurrent 2 protection voltage = 100mV */
												/* 1000: Discharge overcurrent 2 protection voltage = 120mV */
												/* 1001: Discharge overcurrent 2 protection voltage = 140mV */
												/* 1010: Discharge overcurrent 2 protection voltage = 160mV */
												/* 1011: Discharge overcurrent 2 protection voltage = 180mV */
												/* 1100: Discharge overcurrent 2 protection voltage = 200mV */
												/* 1101: Discharge overcurrent 2 protection voltage = 300mV */
												/* 1110: Discharge overcurrent 2 protection voltage = 400mV */
												/* 1111: Discharge overcurrent 2 protection voltage = 500mV */
	uint16_t OCD1T:4;			/* The control position is set for the protection delay of the discharge overcurrent 1 */
												/* 0000: Discharge overcurrent 1 protection delay = 50ms */
												/* 0001: Discharge overcurrent 1 protection delay = 100ms */
												/* 0010: Discharge overcurrent 1 protection delay = 200ms */
												/* 0011: Discharge overcurrent 1 protection delay = 400ms */
												/* 0100: Discharge overcurrent 1 protection delay = 600ms */
												/* 0101: Discharge overcurrent 1 protection delay = 800ms */
												/* 0110: Discharge overcurrent 1 protection delay = 1000ms */
												/* 0111: Discharge overcurrent 1 protection delay = 2000ms */
												/* 1000: Discharge overcurrent 1 protection delay = 4000ms */
												/* 1001: Discharge overcurrent 1 protection delay = 6000ms */
												/* 1010: Discharge overcurrent 1 protection delay = 8000ms */
												/* 1011: Discharge overcurrent 1 protection delay = 10000ms */
												/* 1100: Discharge overcurrent 1 protection delay = 15000ms */
												/* 1101: Discharge overcurrent 1 protection delay = 20000ms */
												/* 1110: Discharge overcurrent 1 protection delay = 30000ms */
												/* 1111: Discharge overcurrent 1 protection delay = 40000ms */
	uint16_t OCD1V:4;			/* Discharge overcurrent 1 protection voltage set control bit */
												/* 0000: Discharge overcurrent 1 protection voltage = 20mV */
												/* 0001: Discharge overcurrent 1 protection voltage = 30mV */
												/* 0010: Discharge overcurrent 1 protection voltage = 40mV */
												/* 0011: Discharge overcurrent 1 protection voltage = 50mV */
												/* 0100: Discharge overcurrent 1 protection voltage = 60mV */
												/* 0101: Discharge overcurrent 1 protection voltage = 70mV */
												/* 0110: Discharge overcurrent 1 protection voltage = 80mV */
												/* 0111: Discharge overcurrent 1 protection voltage = 90mV */
												/* 1000: Discharge overcurrent 1 protection voltage = 100mV */
												/* 1001: Discharge overcurrent 1 protection voltage = 110mV */
												/* 1010: Discharge overcurrent 1 protection voltage = 120mV */
												/* 1011: Discharge overcurrent 1 protection voltage = 130mV */
												/* 1100: Discharge overcurrent 1 protection voltage = 140mV */
												/* 1101: Discharge overcurrent 1 protection voltage = 160mV */
												/* 1110: Discharge overcurrent 1 protection voltage = 180mV */
												/* 1111: Discharge overcurrent 1 protection voltage = 200mV */
	uint16_t OCCT:4;			/* The charging overcurrent protection delay is set to the control bit */
												/* 0000: Charging overcurrent protection delay = 10ms */
												/* 0001: Charging overcurrent protection delay = 20ms */
												/* 0010: Charging overcurrent protection delay = 40ms */
												/* 0011: Charging overcurrent protection delay = 60ms */
												/* 0100: Charging overcurrent protection delay = 80ms */
												/* 0101: Charging overcurrent protection delay = 100ms */
												/* 0110: Charging overcurrent protection delay = 200ms */
												/* 0111: Charging overcurrent protection delay = 400ms */
												/* 1000: Charging overcurrent protection delay = 600ms */
												/* 1001: Charging overcurrent protection delay = 800ms */
												/* 1010: Charging overcurrent protection delay = 1000ms */
												/* 1011: Charging overcurrent protection delay = 2000ms */
												/* 1100: Charging overcurrent protection delay = 4000ms */
												/* 1101: Charging overcurrent protection delay = 8000ms */
												/* 1110: Charging overcurrent protection delay = 10000ms */
												/* 1111: Charging overcurrent protection delay = 20000ms */
	uint16_t OCCV:4;			/* The charging overcurrent protection voltage is set to the control bit */
												/* 0000: Charging overcurrent protection voltage = 20mV */
												/* 0001: Charging overcurrent protection voltage = 30mV */
												/* 0010: Charging overcurrent protection voltage = 40mV */
												/* 0011: Charging overcurrent protection voltage = 50mV */
												/* 0100: Charging overcurrent protection voltage = 60mV */
												/* 0101: Charging overcurrent protection voltage = 70mV */
												/* 0110: Charging overcurrent protection voltage = 80mV */
												/* 0111: Charging overcurrent protection voltage = 90mV */
												/* 1000: Charging overcurrent protection voltage = 100mV */
												/* 1001: Charging overcurrent protection voltage = 110mV */
												/* 1010: Charging overcurrent protection voltage = 120mV */
												/* 1011: Charging overcurrent protection voltage = 130mV */
												/* 1100: Charging overcurrent protection voltage = 140mV */
												/* 1101: Charging overcurrent protection voltage = 160mV */
												/* 1110: Charging overcurrent protection voltage = 180mV */
												/* 1111: Charging overcurrent protection voltage = 200mV */
	uint16_t SCT:4;				/* The short-circuit protection delay is set to the control bit */
												/* 0000: short-circuit protection delay = 0us */
												/* 0001: short-circuit protection delay = 64us */
												/* 0010: short-circuit protection delay = 128us */
												/* 0011: short-circuit protection delay = 192us */
												/* 0100: short-circuit protection delay = 256us */
												/* 0101: short-circuit protection delay = 320us */
												/* 0110: short-circuit protection delay = 384us */
												/* 0111: short-circuit protection delay = 448uS */
												/* 1000: short-circuit protection delay = 512us */
												/* 1001: short-circuit protection delay = 576us */
												/* 1010: short-circuit protection delay = 640us */
												/* 1011: short-circuit protection delay = 704us */
												/* 1100: short-circuit protection delay = 768us */
												/* 1101: short-circuit protection delay = 832us */
												/* 1110: short-circuit protection delay = 896us */
												/* 1111: short-circuit protection delay = 960us */
	uint16_t SCV:4;				/* The short-circuit protection voltage is set to the control bit */
												/* 0000: short-circuit protection voltage = 50mV */
												/* 0001: short-circuit protection voltage = 80mV */
												/* 0010: short-circuit protection voltage = 110mV */
												/* 0011: short-circuit protection voltage = 140mV */
												/* 0100: short-circuit protection voltage = 170mV */
												/* 0101: short-circuit protection voltage = 200mV */
												/* 0110: short-circuit protection voltage = 230mV */
												/* 0111: short-circuit protection voltage = 260mV */
												/* 1000: short-circuit protection voltage = 290mV */
												/* 1001: short-circuit protection voltage = 320mV */
												/* 1010: short-circuit protection voltage = 350mV */
												/* 1011: short-circuit protection voltage = 400mV */
												/* 1100: short-circuit protection voltage = 500mV */
												/* 1101: short-circuit protection voltage = 600mV */
												/* 1110: short-circuit protection voltage = 800mV */
												/* 1111: short-circuit protection voltage = 1000mV */
	uint16_t OTC:8;				/* Charging high temperature protection threshold */
	uint16_t PFT:2;				/* The delay setting of the secondary overcharge protection */
												/* 00: Secondary overcharge protection delay = 8s */
												/* 01: Secondary overcharge protection delay = 16s */
												/* 10: Secondary overcharge protection delay = 32s */
												/* 11: Secondary overcharge protection delay = 64s */
	uint16_t OCRT:2;			/* Charge-discharge over-current self-recovery delay setting */
												/* 00: Charge-discharge overcurrent auto-recovery delay = 8s */
												/* 01: Charge-discharge overcurrent auto-recovery delay = 16s */
												/* 10: Charge-discharge overcurrent auto-recovery delay = 32s */
												/* 11: Charge-discharge overcurrent auto-recovery delay = 64s */
	uint16_t MOST:2;			/* The charge/discharge MOSFET enables the delay setting */
												/* 00: Charge/discharge MOSFET turn-on delay = 64us */
												/* 01: Charge/discharge MOSFET turn-on delay = 128us */
												/* 10: Charge/discharge MOSFET turn-on delay = 256us */
												/* 11: Charge/discharge MOSFET turn-on delay = 512us */
	uint16_t CHS:2;				/* Charge and discharge status detection voltage setting */
												/* 00: Charge and discharge state detection voltage = 200uV */
												/* 01: Charge and discharge state detection voltage = 500uV */
												/* 10: Charge and discharge state detection voltage = 1000uV */
												/* 11: Charge and discharge state detection voltage = 2000uV */
	uint16_t UTC:8;				/* Charging low temperature protection threshold */
	uint16_t OTCR:8;			/* Charging high temperature protection release threshold */
	uint16_t OTD:8;				/* Discharge high temperature protection threshold */
	uint16_t UTCR:8;			/* Charging Low Temperature Protection Release Threshold */
	uint16_t UTD:8;				/* Discharge cryogenic protection threshold */
	uint16_t OTDR:8;			/* Discharge high temperature protection release threshold */
	uint16_t TR:7;				/* Temperature internal reference resistivity */
	uint16_t Reserved4:1;	/* spare */
	uint16_t UTDR:8;			/* Discharge cryogenic protection release threshold */
} AFE_ROM_CODE_S;

typedef struct {
	uint16_t BALANCEH:8;	/* Passive equilibrium */
	uint16_t IDLE:1;			/* IDLE sets the control bits */
												/* 0: SH367309 does not enter the IDLE state */
												/* 1:SH367309 will enter the IDLE state, and the hardware will be automatically cleared after waking up */
												/* Note: When set to "1", if any protection occurs in the SH367309, it will not enter the IDLE state, and the hardware will be automatically cleared */
	uint16_t SLEEP:1;			/* SLEEP sets the control bit */
												/* 0:SH367309 does not enter the SLEEP state */
												/* 1:SH367309 will enter the SLEEP state, and the hardware will be automatically cleared after waking up */
												/* Note: When set to "1", if the charger is connected SH367309, it will not enter the SLEEP state, and the hardware will automatically clear to zero */
	uint16_t ENWDT:1;			/* The watchdog sets the control bit */
												/* 0:SH367309 close the watchdog module */
												/* 1:SH367309 enable the watchdog module */
	uint16_t CADCON:1;		/* CADC sets the control bits */
												/* 0:SH367309 close CADC */
												/* 1:SH367309 Turn on CADC for current collection */
	uint16_t CHGMOS:1;		/* Charging MOSFET control bit */
												/* 0: The charging MOSFET is turned off */
												/* 1: The charging MOSFET is determined by the hardware protection module */
	uint16_t DSGMOS:1;		/* Discharge MOSFET control bit */
												/* 0: The discharge MOSFET is turned off */
												/* 1: The discharge MOSFET is determined by the hardware protection module */
	uint16_t PCHMOS:1;		/* Pre-charge MOSFET control bit */
												/* 0: The pre-charged MOSFET is turned off */
												/* 1: The pre-charge MOSFET is determined by the hardware protection module */
	uint16_t OCRC:1;			/* Overcurrent protection control bit, overcurrent protection status clearing needs to be written in OCRC bit: 0-1-0 continuously */
	uint16_t OV:1;				/* Overvoltage protection status bits */
												/* 0: No overvoltage protection */
												/* 1: Overvoltage protection occurs */
	uint16_t UV:1;				/* Undervoltage protection status bits */
												/* 0: No undervoltage protection occurs */
												/* 1: Underprotection has occurred */
	uint16_t OCD1:1;			/* Discharge overcurrent 1 protection status bit */
												/* 0: No overcurrent 1 protection occurs */
												/* 1: Overcurrent 1 protection occurs */
	uint16_t OCD2:1;			/* Discharge overcurrent 2 protection status bits */
												/* 0: No overcurrent 2 protection occurred */
												/* 1: Overcurrent 2 protection occurs */
	uint16_t OCC:1;				/* Charge overcurrent protection status bit */
												/* 0: No overcurrent protection occurs */
												/* 1: Overcurrent protection occurs */
	uint16_t SC:1;				/* Short-circuit protection status bits */
												/* 0: No short-circuit protection occurred */
												/* 1: Short circuit protection occurs */
	uint16_t PF:1;				/* Secondary overcharge protection status bits */
												/* 0: No overcharge protection occurred */
												/* 1: Overcharge protection occurs */
	uint16_t WDT:1;				/* Watchdog status bits */
												/* 0: Watchdog overflowing */
												/* 1: The watchdog is normal */
	uint16_t BALANCEL:8;	/* It's not written in the information, it should be missing, so I don't know what to do */
	uint16_t DSG_FET:1;		/* Discharge MOSFET switching status bits */
												/* 0: The discharge MOSFET is turned off */
												/* 1: The discharge MOSFET is turned on */
	uint16_t CHG_FET:1;		/* Charging MOSFET switching status bits */
												/* 0: The charging MOSFET is turned off */
												/* 1: The charging MOSFET is turned on */
	uint16_t PCHG_FET:1;	/* Pre-charge MOSFET switching status bits */
												/* 0: The pre-charged MOSFET is turned off */
												/* 1: The pre-charged MOSFET is turned on */
	uint16_t L0V:1;				/* Low voltage prohibits the state of charge bit */
												/* 0: No low voltage is prohibited */
												/* 1: Charging is prohibited in the event of low voltage */
	uint16_t EEPR_WR:1;		/* EEPROM write status bits */
												/* 0: The EEPROM write operation is correct */
												/* 1: EEPROM write error */
	uint16_t Reserved2:1;	/* spare */
	uint16_t DSGING:1;		/* Discharge status bits */
												/* 0: non-discharged state */
												/* 1: Discharge state */
	uint16_t CHGING:1;		/* State of charge bits */
												/* 0: non-charging state */
												/* 1: Charging status */
	uint16_t UTC:1;				/* Charging low temperature protection status bit */
												/* 0: Charging low temperature protection does not occur */
												/* 1: Charging low temperature protection occurs */
	uint16_t OTC:1;				/* Charging high temperature protection status bits */
												/* 0: Charging high temperature protection does not occur */
												/* 1: Charging high temperature protection occurs */
	uint16_t UTD:1;				/* Discharge cryogenic protection status bit */
												/* 0: No discharge cryogenic protection */
												/* 1: Low temperature protection in the event of discharge */
	uint16_t OTD:1;				/* Discharge high temperature protection status bits */
												/* 0: No discharge high temperature protection occurs */
												/* 1: High temperature protection in the event of discharge */
	uint16_t Reserved1:4;	/* spare */
	uint16_t TEMP[CFG_TMP_NUM];			/* 3 temperature measurements */
	int16_t CUR;					/* Current measurements */
	int16_t CELL[CFG_CELL_NUM];		/* Cell XX measurements, according to chip manual 12.1.2, this value is a signed 16-bit */
	int16_t CDATA;				/* CADCD some kind of measurement */
	uint16_t UTC_FLG:1;		/* Charging low temperature protection marker */
												/* 0: No overcharge low temperature protection has occurred */
												/* 1: Overcharge low temperature protection occurs */
	uint16_t OTC_FLG:1;		/* Charging high temperature protection marker */
												/* 0: No overcharging high temperature protection has occurred */
												/* 1: Overcharge high temperature protection occurs */
	uint16_t UTD_FLG:1;		/* Discharge cryogenic protection marker */
												/* 0: No over-discharge cryogenic protection */
												/* 1: Over-discharge low-temperature protection */
	uint16_t OTD_FLG:1;		/* Discharge high temperature protection marker */
												/* 0: No over-discharge high temperature protection */
												/* 1: Over-discharge high temperature protection */
	uint16_t VADC_FLG:1;	/* VADC Interrupt flag */
												/* 0: No VADC interruption has occurred */
												/* 1: No VADC interruption has occurred */
	uint16_t CADC_FLG:1;	/* CADC interrupt flag */
												/* 0: No CADC interrupt has occurred */
												/* 1: A CADC interrupt has occurred */
	uint16_t WAKE_FLG:1;	/* Wake up the interrupt flag */
												/* 0: Not awakened */
												/* 1: Wake up from IDLE status (charge and discharge current detected) or SLEEP state (charger connection). */
	uint16_t RST_FLG:1;		/* Reset the flag */
												/* 0: Not awakened */
												/* 1: After the system is reset, it will automatically set 1 to zero, and the MCU needs to clear it */
	uint16_t OV_FLG:1;		/* Overvoltage protection flags */
												/* 0: No overvoltage protection occurs */
												/* 1: Overvoltage protection occurs */
	uint16_t UV_FLG:1;		/* Under-voltage protection flag */
												/* 0: No undervoltage protection occurs */
												/* 1: Over-voltage protection occurs */
	uint16_t OCD_FLG:1;		/* Discharge overcurrent protection flag */
												/* 0: No over-discharge and over-current protection occurs */
												/* 1: Over-discharge and over-current protection occurs */
	uint16_t LOAD_FLG:1;	/* LDO3 overcurrent flag */
												/* 0: No overcurrent has occurred */
												/* 1: Overcurrent has occurred */
	uint16_t OCC_FLG:1;		/* Charging overcurrent protection flag */
												/* 0: No overcharge overcurrent protection occurs */
												/* 1: Overcharge and overcurrent protection occurs */
	uint16_t SC_FLG:1;		/* Short circuit protection markers */
												/* 0: No short-circuit protection has occurred */
												/* 1: Short-circuit protection has occurred */
	uint16_t PF_FLG:1;		/* Secondary overcharge protection marker */
												/* 0: No secondary overcharge protection has occurred */
												/* 1: There has been a second overcharge protection */
	uint16_t WDT_FLG:1;		/* Watchdog sign */
												/* 0: No watchdog overflow has occurred */
												/* 1: A watchdog overflow has occurred */
	uint8_t RSTSTAT:2;		/* Watchdog counting */
	uint8_t Reserved3:6;	/* spare */
} AFE_RAM_CODE_S;

typedef union {
	AFE_ROM_CODE_S stCode;
	uint8_t aucByte[CFG_AFE_ROM_BLEN];	/* Due to the size of the side, the byte order stored here is reversed from the byte order in the chip, and the internal high and low bytes are interchanged in 2-byte bits */
} AFE_ROM_U;

typedef union {
	AFE_RAM_CODE_S stCode;
	uint8_t aucByte[AFE_RAM_BLEN];	/* Due to the size of the side, the byte order stored here is reversed from the byte order in the chip, and the internal high and low bytes are interchanged in 2-byte bits */
} AFE_RAM_U;

typedef struct {
	uint16_t usCTLC;
	uint16_t usCN;
	uint16_t usLDRT;	/* Unit: ms */
	uint16_t usOV;		/* Unit: mV */
	uint16_t usOVT;		/* Unit: ms */
	uint16_t usOVR;		/* Unit: mV */
	uint16_t usUV;		/* Unit: mV */
	uint16_t usUVT;		/* Unit: ms */
	uint16_t usUVR;		/* Unit: mV */
	uint16_t usBALV;	/* Unit: mV */
	uint16_t usPREV;	/* Unit: mV */
	uint16_t usL0V;		/* Unit: mV */
	uint16_t usPFV;		/* Unit: mV */
	uint16_t usOCD1T;	/* Unit: ms */
	uint16_t usOCD1V;	/* Unit: mV */
	uint16_t usOCD2T;	/* Unit: ms */
	uint16_t usOCD2V;	/* Unit: mV */
	uint16_t usSCT;		/* Unit: us */
	uint16_t usSCV;		/* Unit: mV */
	uint16_t usOCCT;	/* Unit: ms */
	uint16_t usOCCV;	/* Unit: mV */
	uint16_t usPFT;		/* Unit: s */
	uint16_t usOCRT;	/* Unit: s */
	uint16_t usMOST;	/* Unit: us */
	uint16_t usCHS;		/* Unit: uV */
	float fOTC;		/* Unit: ¡æ */
	float fOTCR;	/* Unit: ¡æ */
	float fUTC;		/* Unit: ¡æ */
	float fUTCR;	/* Unit: ¡æ */
	float fOTD;		/* Unit: ¡æ */
	float fOTDR;	/* Unit: ¡æ */
	float fUTD;		/* Unit: ¡æ */
	float fUTDR;	/* Unit: ¡æ */
	float fTR;		/* Unit: k¦¸ */
} AFE_ROM_APP_S;

typedef struct {
	float fTEMP[CFG_TMP_NUM];			/* Unit: ¡æ */
	float fCUR;					/* Unit: mA */
	float fCELLVol[CFG_CELL_NUM];	/* Unit: mV */
	float fCDATA;				/* Unit: mA */
	uint16_t usWDT;
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
static bool IIC309_ReadData(uint8_t ucDevAdd, uint8_t ucRegAdd, uint8_t* const pucData, uint8_t ucDatLen);
static bool IIC309_WriteData(uint8_t ucDevAdd, uint8_t ucRegAdd, uint8_t ucData);
static bool MTP_Write309(uint8_t ucDevAdd, uint8_t ucRegAdd, uint8_t* const pucData, uint8_t ucDatLen, uint8_t ucCount);
static bool MTP_Read309(uint8_t ucDevAdd, uint8_t ucRegAdd, uint8_t* const pucData, uint8_t ucDatLen, uint8_t ucCount);
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
//extern void ResetAFE(void);
extern void ResetAFE1(void);

#pragma pack()

#endif /* _BSP__SH367309_H */
