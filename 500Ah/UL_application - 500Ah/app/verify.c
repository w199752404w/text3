#include "bsp_gpio.h"
#include "bsp_usart.h"
#include "bsp_can.h"
#include "verify.h"
#include "bsp_rtc.h"
#include <stdio.h>
#include "systick.h"
#include "bsp_sh367309.h"
#include "bsp_gd25q32.h"
#include "bsp_adc.h"
#include "bsp.h"
#include "local.h"

void verify(void) {
  VERIFY_DEBUG("¡¾**** Welcome to GD32-BMS ****¡¿\n");
	
/*1.*******Test the RTC---------------------start*********/
	RTC_Time_Verify(); 
/**********Test the RTC---------------------end***********/
	
/*2.*******Test CAN---------------------start*********/		
//	uint8_t a[8]={0,0,0,0,0,0,0,0}; 
//	switch(g_eWkType){
//		case eBspWkInvalid: a[0]=0;break;
//	  case eBspWkKey1   : a[0]=1;break;
//		case eBspWkKey2   : a[0]=2;break;
//		case eBspWkCan    : a[0]=6;break;
//	  case eBspWkChg    : a[0]=7;break;	
//		default:
//		  break;	
//  }
//	CAN0_SendMsg(0x123,a,8);
/**********Test CAN---------------------end***********/
	
/*3.*******Test relays------------------start*********/
	HEAT_RELAY_H;
	delay_1ms(250);
	HEAT_RELAY_L;
/**********Test relays-----------------end************/
	
/*4.*******Test the LEDs----------start********************/
	LED_Test_ON;	
/**********Test the LEDs-----------end*********************/
	
/*5.*******Test GD25Q32 FLASH---------start***********/
	uint8_t ID[4];
	BSP_W25Qx_Read_ID(ID);                        
	VERIFY_DEBUG("GD25Q32 ID : 0x%02X%02X", ID[0], ID[1]);
	if(ID[0] == 0xC8 || ID[0] == 0x15 || ID[1] == 0xC8 || ID[1] == 0x15) {
		VERIFY_DEBUG("GD25Q32 FLASH Test Success");
	}else {
		VERIFY_DEBUG("GD25Q32 FLASH Test Error");
	}
/**********Test GD25Q32 FLASH----------end************/

/*6.*******Test AFE SH367309-----------start**********/
//delay_1ms(1000);
 	afe_get_ram();
	//g_stCfg.stAfe.fCDATACaliB -= g_stAfe.stRamApp.fCDATA;
	//cfg_save();
//	float fVal=1,ff=0;
//	afe_set_ao(eAfeRamCodeCHGMOS, 1, &ff);	/* Turn off the AFE charging loop */
//	afe_set_ao(eAfeRamCodeDSGMOS, 1, &ff);	/* Close the AFE discharge circuit */
/**********Test AFE SH367309-----------end************/
	
/*7.*******Collect the MOS tube temperature--------------start**********/
//  VERIFY_DEBUG("MOSFET Temperature: %.2f ¡æ\r\n",ADC_Get_Temp(g_auiAdcBuf[1]));	
/**********Collect the MOS tube temperature--------------end************/
	
/*8.*******Collects 5V-1A-3A current--------------start**********/
	 float  Current_5V;
	 Current_5V=ADC1_Get_Current(g_auiAdcBuf[0]); 
   VERIFY_DEBUG("5V-1A-3A Current: %.2f A\r\n",Current_5V);	
/**********Collects 5V-1A-3A current--------------end************/
}
