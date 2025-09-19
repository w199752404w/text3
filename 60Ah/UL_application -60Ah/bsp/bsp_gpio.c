#include "bsp_gpio.h"
#include "bsp_gd25q32.h"
#include "history.h"
#include "systick.h"

void MX_GPIO_Init(void) {	
    rcu_periph_clock_enable(RCU_GPIOA);
	  rcu_periph_clock_enable(RCU_GPIOB);
	  rcu_periph_clock_enable(RCU_GPIOC);
	  rcu_periph_clock_enable(RCU_GPIOD);
	
	  rcu_periph_clock_enable(RCU_AF);                             //AF clock enable 
		gpio_pin_remap_config(GPIO_SWJ_SWDPENABLE_REMAP, ENABLE);    //Turn off the JTAG-DP enable and keep the SW-DP enable in order for PB3 and PB4 and PA15 to function properly
	
	  gpio_init(GPIOA, GPIO_MODE_IPD , GPIO_OSPEED_50MHZ, GPIO_PIN_0 | GPIO_PIN_15);
//		gpio_init(GPIOA, GPIO_MODE_AIN , GPIO_OSPEED_50MHZ, GPIO_PIN_15);
	  gpio_init(GPIOA, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10);
	  gpio_init(GPIOB, GPIO_MODE_IPU , GPIO_OSPEED_50MHZ, GPIO_PIN_4);
	  gpio_init(GPIOB, GPIO_MODE_IPD , GPIO_OSPEED_50MHZ, GPIO_PIN_12);
	  gpio_init(GPIOB, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_3 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_13 |GPIO_PIN_15);	
	  gpio_init(GPIOC, GPIO_MODE_IPU, GPIO_OSPEED_50MHZ, GPIO_PIN_2 | GPIO_PIN_5 | GPIO_PIN_8);
	  gpio_init(GPIOC, GPIO_MODE_IPD, GPIO_OSPEED_50MHZ, GPIO_PIN_12);
	  gpio_init(GPIOC, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_3 | GPIO_PIN_11 | GPIO_PIN_13 );	
	  gpio_init(GPIOD, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_2);

//		WDG_DONE_H;
//		delay_1ms(20);
//	  WDG_DONE_L;
	
	  POWER_ON_12V_VDD_H;   //open V12 serial isolate chip power supply
	  AFE_SHIP_VDD_H;       //Enable SHIP to enter acquisition mode and open V3.3 power supply
		AFE_VPRO_L;
		HEAT_RELAY_L;         //Close heater relay
	  TURN_DETN_LOAD_H;     //Turn on the switch for detecting charge and discharge
		WDG_IO_H;
//		CAN_RES_ON;
		CAN_RES_OFF;
	  MCU_DSG_OFF;
	  MCU_CHG_OFF;
		MCU_1A_OFF;       //close the switch of 5V_1A 
	  MCU_3A_OFF;       //close the switch of 5V_3A
		MCU_3A_OFF_old;
		MEM_FlashRead(eHisCfgPStart * PAGE_SIZE, (uint8_t*)&g_stCfg, sizeof(CFG_S));
		if(g_stCfg.usGoRun == 0 || g_stCfg.usGoRun == 2) {
			DECOUPLER_H;
		} else {
			DECOUPLER_L;
		}				
}


/* Function Name  :   gpio_bit_toggle(uint32_t gpio_periph,uint32_t pin)
*  Input          :   gpio_periph GPIO pin 
*  Output         :   None
*  Description    :   Output pin level flipping: If it is high, it becomes low, and if it is low, it becomes high
*  Usage examples : gpio_bit_toggle(GPIOB, GPIO_PIN_2)
*/
void gpio_bit_toggle(uint32_t gpio_periph,uint32_t pin)
{
		if(gpio_output_bit_get(gpio_periph, pin)==SET)            
			gpio_bit_reset(gpio_periph, pin);						              
		else														                           
			gpio_bit_set(gpio_periph, pin);							             
}
