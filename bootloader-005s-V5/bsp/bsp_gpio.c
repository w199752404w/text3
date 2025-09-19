#include "bsp_gpio.h" 
#include "systick.h"

void MX_GPIO_Init(void) {	 	
    rcu_periph_clock_enable(RCU_GPIOA);
	  rcu_periph_clock_enable(RCU_GPIOB);
	  rcu_periph_clock_enable(RCU_GPIOC);
	  rcu_periph_clock_enable(RCU_GPIOD);
	
	  rcu_periph_clock_enable(RCU_AF);                             //AF clock enable 
	  gpio_pin_remap_config(GPIO_SWJ_SWDPENABLE_REMAP, ENABLE);    //Turn off the JTAG-DP enable and keep the SW-DP enable in order for PB3 and PB4 and PA15 to function properly

	  gpio_init(GPIOA, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_8 | GPIO_PIN_9);
	  gpio_init(GPIOB, GPIO_MODE_IPU , GPIO_OSPEED_50MHZ, GPIO_PIN_4);
	  gpio_init(GPIOB, GPIO_MODE_IPD , GPIO_OSPEED_50MHZ, GPIO_PIN_12);
	  gpio_init(GPIOB, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_13 |GPIO_PIN_15);	
	  gpio_init(GPIOC, GPIO_MODE_IPU, GPIO_OSPEED_50MHZ, GPIO_PIN_2 | GPIO_PIN_5 | GPIO_PIN_8);
		WDG_DONE_H;
		delay_1ms(20);
	  WDG_DONE_L;
		DECOUPLER_H;
	  POWER_ON_12V_VDD_H;   //open V12 serial isolate chip power supply
	  AFE_SHIP_VDD_H;       //Enable SHIP to enter acquisition mode and open V3.3 power supply
		HEAT_RELAY_L;         //Close relay
		CAN_RES_OFF;
	  MCU_DSG_OFF;
	  MCU_CHG_OFF;
//		WDG_DONE_H;
//		WDG_DONE_L; 
}


//���ŷ�ת����
/* ������   gpio_bit_toggle(uint32_t gpio_periph,uint32_t pin)
*  ����ֵ   gpio_periph GPIO�˿� pin GPIO����
*  ����ֵ   ��
*  ����     ������ŵ�ƽ��ת ����Ǹ߱�ɵͣ�����ǵͱ�ɸ�
*  ʹ�þ��� gpio_bit_toggle(GPIOB, GPIO_PIN_2);//��תPB2 �����ƽ
*/
void gpio_bit_toggle(uint32_t gpio_periph,uint32_t pin)
{
		if(gpio_output_bit_get(gpio_periph, pin)==SET)              //�����ǰ����Ǹߵ�ƽ
			gpio_bit_reset(gpio_periph, pin);						              //��תΪ�͵�ƽ
		else														                            //�����ǰ����ǵ͵�ƽ
			gpio_bit_set(gpio_periph, pin);							              //��תΪ�ߵ�ƽ
}
