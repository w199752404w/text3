/**
  ******************************************************************************
  * @file    bsp_wdg.c
  * @author  tang
  * @version V1.0
  * @date    2023-xx-xx
  * @brief   Watchdog drive
  ******************************************************************************
  */

#include "bsp_wdg.h" 
/*
Initialization of Watchdog
  Independent watchdog - default setting for 26.2s timeout
  Window watchdog - default setting of 15ms timeout
*/
void gd32_wdgt_init(EWdgType type)
{
	if(type == EWdgType_fwdg) //Independent watchdog
	{
		/* enable IRC40K */
		rcu_osci_on(RCU_IRC40K);   
		/* wait till IRC40K is ready */
		while(SUCCESS != rcu_osci_stab_wait(RCU_IRC40K)){
		}
		/* 40K / 256 = 0.15625 KHz      t = 4095 /0.15625  = 26208 ms */
		fwdgt_write_enable();
		fwdgt_config(0x0FFF, FWDGT_PSC_DIV256); //40S timeout
		fwdgt_write_disable();
		fwdgt_enable();
	}
	else if(type == EWdgType_wwdg)//Window Watchdog
	{
		wwdgt_deinit();
		rcu_periph_clock_enable(RCU_WWDGT);
		/* 60M / 4096 / 4 = 3.662109375 KHz  t = 1/3.6621 *48 = 13ms    
		The dog needs to be fed around 4-17ms after being turned on, otherwise it will time out*/
		wwdgt_config(0x7F, 0x6F, WWDGT_CFG_PSC_DIV4);
		wwdgt_counter_update(0x7F);
		wwdgt_enable();
	}
}

/*
Watchdog feeding dog
*/
void gd32_wdgt_feed_dog(EWdgType type)
{
	if(type == EWdgType_fwdg)//Independent watchdog
	{
		fwdgt_write_enable();
		fwdgt_counter_reload();
		fwdgt_write_disable();
	}
	else if(type == EWdgType_wwdg)//Window Watchdog
	{
		wwdgt_counter_update(0x7F);
	}
}

void Into_Standby_Mode(void)
{	
	 rcu_periph_clock_enable(RCU_GPIOA);
	 rcu_periph_clock_enable(RCU_GPIOB);
	 rcu_periph_clock_enable(RCU_GPIOC);
	 rcu_periph_clock_enable(RCU_GPIOD);
	
	 gpio_init(GPIOA, GPIO_MODE_AIN, GPIO_OSPEED_50MHZ, GPIO_PIN_ALL);
	 gpio_init(GPIOB, GPIO_MODE_AIN, GPIO_OSPEED_50MHZ, GPIO_PIN_ALL);
	 gpio_init(GPIOC, GPIO_MODE_AIN, GPIO_OSPEED_50MHZ, GPIO_PIN_ALL);
	 gpio_init(GPIOD, GPIO_MODE_AIN, GPIO_OSPEED_50MHZ, GPIO_PIN_ALL);
	
   rcu_periph_clock_enable(RCU_PMU);  //Enable power management and enable peripheral PMU clocks
   pmu_wakeup_pin_enable();           //Enable the wake pin
	 rcu_system_clock_source_config(RCU_CKSYSSRC_IRC8M); //2024/12/20 dgx
   pmu_to_standbymode();              //Enter standby mode, wait for PA0 to appear on the rising edge, and the program will be executed from MAIN().
}

/*********************************************END OF FILE**********************/
