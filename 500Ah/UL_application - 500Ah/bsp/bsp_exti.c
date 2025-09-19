/**
  ******************************************************************************
  * @file    bsp_exti.c
  * @author  tang
  * @version V1.0
  * @date    2023-xx-xx
  ******************************************************************************
  */
#include "bsp_exti.h"
#include "bsp.h"
#include "bsp_gpio.h"
#include "stdio.h"
#include "bsp_wdg.h"
#include "bsp_usart.h"
#include "systick.h"
#include "config.h"
#include "bsp_can.h"
#include "bsp_sh367309.h"

//#include "bsp_bq76940.h"
uint8_t g_ucSWTick = 0,g_ucSW2Tick = 0;

/******************************************************************************
*Functions Name£ºvoid Delay_us(void)
*function£ºus-level latency
*Parameters£ºi
*Return value: None
*******************************************************************************/	
void Delay_us(unsigned long i)
{
	unsigned long j;
	for(;i>0;i--)
	{
			for(j=26;j>0;j--);
	}
}

void EXTI0_REST_Init(void) {
	REST_GPIO_CLK_ENABLE(); 
	gpio_init(REST_INT_GPIO_PORT, GPIO_MODE_IPD, GPIO_OSPEED_50MHZ, REST_INT_GPIO_PIN);
	nvic_irq_enable(EXTI0_IRQn, 0U, 0U);                               //enable and set key EXTI interrupt to the lowest priority 
  gpio_exti_source_select(GPIO_PORT_SOURCE_GPIOA, GPIO_PIN_SOURCE_0);//connect key EXTI line to key GPIO pin
	exti_init(EXTI_0, EXTI_INTERRUPT, EXTI_TRIG_RISING);               //The rising edge is interrupted
	exti_interrupt_flag_clear(EXTI_0);                                 //Clear the break sign 
}

void EXTI2_SW_Init(void) {
	REST2_GPIO_CLK_ENABLE(); 
	gpio_init(REST2_INT_GPIO_PORT, GPIO_MODE_IPU, GPIO_OSPEED_50MHZ, REST2_INT_GPIO_PIN);
	nvic_irq_enable(EXTI2_IRQn, 0U, 0U);                               //enable and set key EXTI interrupt to the lowest priority 
  gpio_exti_source_select(GPIO_PORT_SOURCE_GPIOC, GPIO_PIN_SOURCE_2);//connect key EXTI line to key GPIO pin
	exti_init(EXTI_2, EXTI_INTERRUPT, EXTI_TRIG_BOTH);              	 //Falling edge breaks
	exti_interrupt_flag_clear(EXTI_2);                                  //Clear the break sign 
}

void EXTI5_SW2_Init(void) {
	REST5_GPIO_CLK_ENABLE(); 
	gpio_init(REST5_INT_GPIO_PORT, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, REST5_INT_GPIO_PIN); 
	nvic_irq_enable(EXTI5_9_IRQn, 0U, 0U);                               //enable and set key EXTI interrupt to the lowest priority 
  gpio_exti_source_select(GPIO_PORT_SOURCE_GPIOC, GPIO_PIN_SOURCE_5);//connect key EXTI line to key GPIO pin
	exti_init(EXTI_5, EXTI_INTERRUPT, EXTI_TRIG_FALLING);              //Falling edge breaks
	exti_interrupt_flag_clear(EXTI_5);                                 //Clear the break sign 
}

void EXTI15_CHG_Init(void) {
	rcu_periph_clock_enable(RCU_GPIOA); 
	gpio_init(GPIOA, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_15); 
	nvic_irq_enable(EXTI10_15_IRQn, 0U, 0U);                               //enable and set key EXTI interrupt to the lowest priority 
  gpio_exti_source_select(GPIO_PORT_SOURCE_GPIOA, GPIO_PIN_SOURCE_15);//connect key EXTI line to key GPIO pin
	exti_init(EXTI_15, EXTI_INTERRUPT, EXTI_TRIG_BOTH);              //Falling edge breaks
	exti_interrupt_flag_clear(EXTI_15);                                 //Clear the break sign 
}

void EXTI0_IRQHandler(void)
{
	uint16_t i,count=0;
	/* proceess code here - START*/
	for(i=0; i<100; i++){
		if(REST_READ == 1){
			count++;
//			delay_1ms(30);
			if(count > 2){//2
				count=0;
     // if(g_stCanRCA.astRBuf[g_stCanRCA.usCIdx].uiId == 0x00000001){								
					g_eWkType = eBspWkCan;
					g_stCfg.usGoRun = 1;
		 //	}					
			}			
		}
	}
		/*proceess code here - END  */ 
	exti_interrupt_flag_clear(EXTI_0);
}

void EXTI2_IRQHandler(void)
{
	uint16_t i,count=0;
	/* proceess code here - START*/
		for(i=0; i<100; i++){
			if(SW_WK_READ  == 0){
				count++;
				if(count > 2){//2
					 count=0;			 
					 g_eWkType = eBspWkKey1;
					 g_stCfg.usGoRun = 1;
				}		
			}
		}	
		/*proceess code here - END  */ 
	exti_interrupt_flag_clear(EXTI_2);
}

void EXTI10_15_IRQHandler(void)
{
	uint8_t aucSBuf[8] = {0};
	aucSBuf[0] = 1;
	CAN0_SendMsg(0x103 , aucSBuf, 1);
	uint16_t i,count=0;
	/* proceess code here - START*/
		for(i=0; i<1; i++){
			if(CHG_WK_50V_READ  == 1){
				count++;
				if(count > 2){//2
					 count=0;			 
					 g_eWkType = eBspWkChg;
					 g_stCfg.usGoRun = 3;
				}		
			}
		}	
		/*proceess code here - END  */ 
	exti_interrupt_flag_clear(EXTI_15);
}

void EXTI5_9_IRQHandler(void)
{
//	uint16_t i,count=0;
//	/* proceess code here - START*/
//	for(i=0; i<1000; i++) {
//		if(SW2_WK_READ  == 0) {
//			count++;
//			if(count > 999) {
//				 count=0;
//         g_ucSW2Tick++;
//				 EXTI_DEBUG("SW2:g_ucSW2Tick=%d\r\n",g_ucSW2Tick);
//				if(g_ucSW2Tick > 3)
//					g_ucSW2Tick=0;
//			}
//		}
//	}
	/*proceess code here - END  */ 
	exti_interrupt_flag_clear(EXTI_10);
}

//Software reset function
void System_Reset(void) {
	float fVal = 0;
	PRE_DSG_L;
	afe_set_ao(eAfeRamCodeCHGMOS, 1, &fVal);
	afe_set_ao(eAfeRamCodeDSGMOS, 1, &fVal);
	uint8_t aucUartSBuf[6]={0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f};
	UART_Send(aucUartSBuf, 6);
	delay_1ms(500);
	__set_PRIMASK(1);   //Turn off all interruptions
	NVIC_SystemReset(); //Perform a software reset
}

void System_Sleep(void) {
	POWER_ON_12V_VDD_L;
	Delay_us(100000);
//	delay_1ms(100);
	Into_Standby_Mode();
}

/*********************************************END OF FILE**********************/
