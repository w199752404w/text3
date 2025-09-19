#include "stdio.h"
#include "systick.h"
#include "bsp.h"
#include "bsp_usart.h"
#include "bsp_can.h"
#include "bsp_gpio.h"
#include "bsp_tim.h"
#include "bsp_exti.h"
#include "bsp_adc.h"
#include "bsp_wdg.h"
#include "bsp_rtc.h"
#include "bsp_i2c.h"

#include "bsp_gd25q32.h"
#include "bsp_exti.h"
#include "history.h"

BSP_WK_TYPE_E g_eWkType = eBspWkInvalid;

/*******************************************************************************
* Function Name  : BSP_Init
* Description    : BSP Initialize Collection
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void BSP_Init(void) {	 
	rcu_ahb_clock_config(RCU_AHB_CKSYS_DIV1);                 //Set the AHB main frequency to 1 division, that is, the main frequency is 108M
	systick_config();                                         //Initialize the systick timer and use the delay function
	MX_GPIO_Init();                                           //Initialize GPIO port
	UART_Init(e485Port1,115200, USART_WL_8BIT, USART_STB_1BIT, USART_PM_NONE);
    //UART_Init1(115200, USART_WL_8BIT, USART_STB_1BIT, USART_PM_NONE);
    UART_Init_Dma_Receive(); 
	CAN0_Init(250);
	ADC_Init();                                               //Initialize ADC 	
	TIM_Init();                                               //Initialize TIM
	RTC_Init();                                               //Initialize RTC 
	IIC_Init();                                               //Initialize IIC of SH367309 chip 
	for(uint16_t i = 0; i < 5000; i++) {
		if(BSP_W25Qx_Init() == W25Qx_OK) {
			break;
		}
	}
	BSP_Get_ADC1_Temp();                                      //Obtaining ADC1_Temp value 
	timer_enable(TIMER1);
	RTC_Time_Verify();
//exti_deinit();                                            //close the exti
}

void BSP_WK_Detection(void) {
	//MEM_FlashRead(eHisCfgPStart * PAGE_SIZE, (uint8_t*)&g_stCfg, sizeof(CFG_S));
//	if(g_stCfg.usGoRun == 0 || g_stCfg.usGoRun == 2) {
//	  g_stCfg.usGoRun = 3;
//	  cfg_save();
//		BSP_DEBUG("shutdown\r\n");
//		delay_1ms(1000);
//		System_Sleep();
//		delay_1ms(1000);
//		return;
//	}
//	if(g_stCfg.usGoRun == 1) {
//	  g_stCfg.usGoRun = 3;
//	  cfg_save();
//		SW_EN_LED_H;
//		return;
//	}
	SW_EN_LED_H;
	uint8_t ucCnt = 0;
	while(SW_WK_READ == 0) {
        SW_EN_LED_H;
		ucCnt++;
		if(ucCnt > 99) {
			g_eWkType = eBspWkKey1;
			//g_stCfg.usGoRun = 3;
			//break;
		}
	}
	ucCnt = 0;
	while(CHG_WK_12V_READ == 1) {
		ucCnt++;
		if(ucCnt > 99) {
			g_eWkType = eBspWkExPwr;
			//g_stCfg.usGoRun = 3;
			break;
		}
	}
	ucCnt = 0;
	while(CHG_WK_50V_READ == 1) {
		ucCnt++;
		if(ucCnt > 99) {
			g_eWkType = eBspWkChg;
			//g_stCfg.usGoRun = 3;
			break;
		}
	}
	ucCnt = 0;
	while(REST_READ == 1) {
		ucCnt++;
		if(ucCnt > 99) {
			g_eWkType = eBspWkCan;
			//g_stCfg.usGoRun = 3;
			break;
		}
	}
}

void BSP_5V_1A_3A_Detection(void) {
//	uint8_t i = 0;
////	static uint32_t s_ui4AOverload = 0;
//	for(i = 0; i < 100; i++) {
////		if(MCU_OV_4A_READ != 0) {
//			break;
//		}
//		Delay_us(10);
//	}
//	if(i >= 100) {
//		if(s_ui4AOverload < 10000) {
//			s_ui4AOverload += 1000;
//		}
////		MCU_3A_OFF;
////		MCU_3A_OFF_old;
//		BSP_DEBUG("5V_4A circuit is overworking\r\n"); 
//	}
//	if(s_ui4AOverload <= 9000) {
//		static uint8_t s_ucTick = 0;
////		if(MCU_OV_4A_READ == 0) {
//			if(s_ucTick < 100) {
//				s_ucTick++;
//			}
//		} else {
//			s_ucTick = 0;
//		}
//		if(s_ucTick >= 100) {
////			MCU_3A_ON;
////			MCU_3A_ON_old;
//			BSP_DEBUG("5V_4A circuit is opened\r\n");
//		}
//	} else {
//		s_ui4AOverload -= 1;
//	}
}

void BSP_Get_ADC1_Temp(void) {
 	ADC1_Temp = g_auiAdcBuf[0]; 
//	MCU_3A_ON;           //open the switch of 5V_3A 
//	MCU_3A_ON_old;       //open the switch of 5V_3A
}

void BSP_Stop(void) {
//timer_disable(TIMER1);
//	AFE_VPRO_L;
	adc_deinit(ADC0);
	dma_deinit(DMA0, DMA_CH0);
}
