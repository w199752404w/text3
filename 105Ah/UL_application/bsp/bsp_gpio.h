/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _BSP_GPIO_H_
#define _BSP_GPIO_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "gd32f10x.h"

extern void MX_GPIO_Init(void);
extern void gpio_bit_toggle(uint32_t gpio_periph,uint32_t pin);

//IO operation definition
#define CHG_WK_12V_READ         gpio_input_bit_get(GPIOC,GPIO_PIN_12)	      //charge 12V connected, signal on, useless
#define CHG_WK_50V_READ         gpio_input_bit_get(GPIOA,GPIO_PIN_15)	      //charge 50V connected, signal on
#define CAN_WK_READ             gpio_input_bit_get(GPIOA,GPIO_PIN_0)
#define SW_WK_READ              gpio_input_bit_get(GPIOC,GPIO_PIN_2)	      //pack button pressed, signal off
#define SW2_WK_READ             gpio_input_bit_get(GPIOC,GPIO_PIN_5)        //extern button pressed, signal off
#define MCU_OV_4A_READ          gpio_input_bit_get(GPIOC,GPIO_PIN_8)
#define PRE_DSG_READ            gpio_input_bit_get(GPIOB,GPIO_PIN_12)
#define CHG_DSG_DETN_READ       gpio_input_bit_get(GPIOB,GPIO_PIN_4)        //Load detection, low level with load

#define POWER_ON_12V_VDD_H      gpio_bit_set(GPIOC,GPIO_PIN_11)
#define POWER_ON_12V_VDD_L      gpio_bit_reset(GPIOC,GPIO_PIN_11)
#define AFE_VPRO_H    	        gpio_bit_set(GPIOB,GPIO_PIN_6) 
#define AFE_VPRO_L    	        gpio_bit_reset(GPIOB,GPIO_PIN_6)
#define AFE_SHIP_VDD_H          gpio_bit_set(GPIOB,GPIO_PIN_7) 
#define AFE_SHIP_VDD_L          gpio_bit_reset(GPIOB,GPIO_PIN_7)
#define CAN_RES_ON              gpio_bit_reset(GPIOA,GPIO_PIN_8)
#define CAN_RES_OFF             gpio_bit_set(GPIOA,GPIO_PIN_8)
#define MCU_1A_ON               gpio_bit_reset(GPIOC,GPIO_PIN_3)            //Low level enable
#define MCU_1A_OFF              gpio_bit_set(GPIOC,GPIO_PIN_3) 
#define MCU_3A_ON               gpio_bit_reset(GPIOA,GPIO_PIN_10)	          //Low level enable
#define MCU_3A_OFF              gpio_bit_set(GPIOA,GPIO_PIN_10) 
#define MCU_3A_ON_old           gpio_bit_reset(GPIOB,GPIO_PIN_3)            //Low level enable
#define MCU_3A_OFF_old          gpio_bit_set(GPIOB,GPIO_PIN_3)
#define HEAT_RELAY_H  		      gpio_bit_set(GPIOB,GPIO_PIN_15)							//set heater on
#define HEAT_RELAY_L  		      gpio_bit_reset(GPIOB,GPIO_PIN_15)						//set heater off
#define DECOUPLER_H     	      gpio_bit_set(GPIOB,GPIO_PIN_1) 							//Decoupler off
#define DECOUPLER_L      		    gpio_bit_reset(GPIOB,GPIO_PIN_1)						//Decoupler on
#define SW_EN_LED_H             gpio_bit_set(GPIOC,GPIO_PIN_13)             //Open button LED
#define SW_EN_LED_L             gpio_bit_reset(GPIOC,GPIO_PIN_13)           //Close button LED
#define MCU_DSG_ON   	          gpio_bit_set(GPIOB,GPIO_PIN_0)
#define MCU_DSG_OFF  	          gpio_bit_reset(GPIOB,GPIO_PIN_0)
#define MCU_CHG_ON    	        gpio_bit_set(GPIOB,GPIO_PIN_13)
#define MCU_CHG_OFF  	          gpio_bit_reset(GPIOB,GPIO_PIN_13)
#define PRE_DSG_H               gpio_bit_set(GPIOC,GPIO_PIN_1)
#define PRE_DSG_L               gpio_bit_reset(GPIOC,GPIO_PIN_1)
#define LED_Test_OFF            gpio_bit_set(GPIOD,GPIO_PIN_2)
#define LED_Test_ON             gpio_bit_reset(GPIOD,GPIO_PIN_2)
#define WDG_IO_H                gpio_bit_set(GPIOC,GPIO_PIN_0)
#define WDG_IO_L                gpio_bit_reset(GPIOC,GPIO_PIN_0)
#define WDG_TOGGLE              gpio_bit_toggle(GPIOC,GPIO_PIN_0)  
#define WDG_DONE_H              gpio_bit_set(GPIOA,GPIO_PIN_9)
#define WDG_DONE_L              gpio_bit_reset(GPIOA,GPIO_PIN_9)

#define TURN_DETN_LOAD_H        gpio_bit_set(GPIOB,GPIO_PIN_5)            //Turn on the load detection switch
#define TURN_DETN_LOAD_L        gpio_bit_reset(GPIOB,GPIO_PIN_5)

#ifdef __cplusplus
}
#endif 
#endif
