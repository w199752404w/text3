/**
  ******************************************************************************
  * @file    bsp_exti.c
  * @author  tang
  * @version V1.0
  * @date    2023-xx-xx
  ******************************************************************************
  */
#include "bsp_exti.h"
#include "bsp_gpio.h"
#include "systick.h"
#include "stdio.h"

uint8_t g_ucSWTick = 0,g_ucSW2Tick = 0;

/******************************************************************************
*函  数：void Delay_us(void)
*功　能：us级别延时
*参  数：i
*返回值：无
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
	exti_init(EXTI_0, EXTI_INTERRUPT, EXTI_TRIG_RISING);               //上升沿中断
	exti_interrupt_flag_clear(EXTI_0);                                 //清中断标志 
}

void EXTI2_SW_Init(void) {
	REST2_GPIO_CLK_ENABLE(); 
	gpio_init(REST2_INT_GPIO_PORT, GPIO_MODE_IPU, GPIO_OSPEED_50MHZ, REST2_INT_GPIO_PIN);
	nvic_irq_enable(EXTI2_IRQn, 0U, 0U);                               //enable and set key EXTI interrupt to the lowest priority 
  gpio_exti_source_select(GPIO_PORT_SOURCE_GPIOC, GPIO_PIN_SOURCE_2);//connect key EXTI line to key GPIO pin
	exti_init(EXTI_2, EXTI_INTERRUPT, EXTI_TRIG_FALLING);               //下降沿中断
	exti_interrupt_flag_clear(EXTI_2);                                  //清中断标志 
}

void EXTI5_SW2_Init(void) {
	REST5_GPIO_CLK_ENABLE(); 
	gpio_init(REST5_INT_GPIO_PORT, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, REST5_INT_GPIO_PIN); 
	nvic_irq_enable(EXTI5_9_IRQn, 0U, 0U);                               //enable and set key EXTI interrupt to the lowest priority 
  gpio_exti_source_select(GPIO_PORT_SOURCE_GPIOC, GPIO_PIN_SOURCE_5);//connect key EXTI line to key GPIO pin
	exti_init(EXTI_5, EXTI_INTERRUPT, EXTI_TRIG_FALLING);              //下降沿中断
	exti_interrupt_flag_clear(EXTI_5);                                 //清中断标志 
}

void EXTI0_IRQHandler(void)
{
	uint16_t i,count=0;
	/* proceess code here - START*/
	for(i=0; i<100; i++){
		if(REST_READ == 1){
			count++;
			if(count > 2){
				count=0;
				g_ucSWTick = 1;
     // if(g_stCanRCA.astRBuf[g_stCanRCA.usCIdx].uiId == 0x00000001){								
     //			g_eWkType = eBspWkCan;
     //			g_stCfg.usGoRun = 1;
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
				if(count > 2){
					 count=0;			 
//					 g_eWkType = eBspWkKey1;
//					 g_stCfg.usGoRun = 1;

				}		
			}
		}	
		/*proceess code here - END  */ 
	exti_interrupt_flag_clear(EXTI_2);
}

void EXTI5_9_IRQHandler(void)
{
	uint16_t i,count=0;
	/* proceess code here - START*/
	for(i=0; i<1000; i++){
		if(SW2_WK_READ  == 0){
			count++;	
			if(count > 999){
				 count=0;			 
         g_ucSW2Tick++;
				 EXTI_DEBUG("SW2:g_ucSW2Tick=%d\r\n",g_ucSW2Tick);
				if(g_ucSW2Tick > 3)
					g_ucSW2Tick=0;
			}		
		}
	}
		/*proceess code here - END  */ 
	exti_interrupt_flag_clear(EXTI_10);
}





/*********************************************END OF FILE**********************/
