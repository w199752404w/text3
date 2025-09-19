/**
  ******************************************************************************
  * @file    bsp_tim.c
  * @author  tang
  * @version V1.0
  * @date    2023-xx-xx
  * @brief   Basic timer timing
  ******************************************************************************
  */
  
#include "bsp_tim.h"
#include "bsp_gpio.h"
#include "bsp_wdg.h"
#include "bsp_usart.h"
#include "stdio.h"
#include "config.h"
#include "protocol.h"
#include "local.h"
uint8_t g_ucTm = 0;
uint16_t g_uiUg_Tick[3] = {0};
	
void TIM_Init(void)
{
    /* -----------------------------------------------------------------------
	   The main frequency of the system108MHZ,timer_initpara.prescaler is 119£¬timer_initpara.period is 49999£¬That's 50ms,frequency£ºp=108M/(prescaler+1)£¬The time of the execution is 1/p, and the timing time is T=(period+1)/p, unit: seconds
    ----------------------------------------------------------------------- */
    timer_parameter_struct timer_initpara;
    rcu_periph_clock_enable(RCU_TIMER1);
    timer_deinit(TIMER1);
    /* TIMER1 configuration */
    timer_initpara.prescaler         = 107;
    timer_initpara.alignedmode       = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
    timer_initpara.period            = 49999;
    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
    timer_initpara.repetitioncounter = 0;
    timer_init(TIMER1,&timer_initpara);
	  nvic_priority_group_set(NVIC_PRIGROUP_PRE1_SUB3);
		nvic_irq_enable(TIMER1_IRQn, 0, 1);
		timer_interrupt_enable(TIMER1, TIMER_INT_UP);
    /* auto-reload preload enable */
    timer_auto_reload_shadow_enable(TIMER1);   
}


/*******************************************************************************
* Function Name  : HAL_TIM_PeriodElapsedCallback
* Description    : 50ms timer callback function
* Input          : htim,	Timer handle
* Output         : None
* Return         : None
*******************************************************************************/
void TIMER1_IRQHandler(void)
{
timer_flag_clear(TIMER1,TIMER_FLAG_UP);	
	g_ucTm = 1;
	if(g_stLocalArrayRVal.eLocalStat == eLocalStatUartBootMode) {
		static uint16_t s_last_usLen = 0;
		static uint16_t s_new_usLen = 0;
		s_last_usLen = s_new_usLen;
	  s_new_usLen = sizeof(g_stUsart.aucRBuf) - dma_transfer_number_get(DMA0, DMA_CH2);
		if(s_last_usLen == s_new_usLen && s_new_usLen > 0){
			g_uiUg_Tick[0] ++;
		  if(g_uiUg_Tick[0] > 100){
			  g_uiUg_Tick[0] = 0;
        USART2_RxFlag = 1;
		  }
		}else{
			g_uiUg_Tick[0] = 0;			
		}
		
		if(s_new_usLen < 170){
		  g_uiUg_Tick[1] ++;
		  if(g_uiUg_Tick[1] > 200){
			  g_uiUg_Tick[1] = 0;
				g_uiUg_Tick[2] = 0;
			  g_stLocalArrayRVal.eLocalStat = eLocalStatRun;
	      USART2_UgFlag = 0;
				USART2_RxFlag = 0;
				dma_channel_disable(DMA0, DMA_CH2);
        dma_flag_clear(DMA0, DMA_CH2, DMA_INTF_FTFIF);  /* Clear the DMA flag */ 
        dma_transfer_number_config(DMA0, DMA_CH2, sizeof(g_stUsart.aucRBuf)); //	* Reassign the count value */
		    dma_channel_enable(DMA0, DMA_CH2);
		  }
	  }else{
			g_uiUg_Tick[1] = 0;
		}
		
		g_uiUg_Tick[2]++;	
		if(g_uiUg_Tick[2] > 6000){//2400=2min
				g_uiUg_Tick[2] = 0;
				g_stLocalArrayRVal.eLocalStat = eLocalStatRun;
				USART2_UgFlag = 0;
				USART2_RxFlag = 0; 
				dma_channel_disable(DMA0, DMA_CH2);
				dma_flag_clear(DMA0, DMA_CH2, DMA_INTF_FTFIF);  /* Clear the DMA flag */
				dma_transfer_number_config(DMA0, DMA_CH2, sizeof(g_stUsart.aucRBuf)); //	* Reassign the count value */
				dma_channel_enable(DMA0, DMA_CH2);
		}
  }
#ifndef USER_DEBUG
	if(g_stCfg.usGoRun != 0){
		gd32_wdgt_feed_dog(EWdgType_fwdg);	//feed dog
	}
#endif
}
/*********************************************END OF FILE**********************/
