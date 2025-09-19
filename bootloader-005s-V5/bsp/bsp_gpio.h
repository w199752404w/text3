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
#define CHG_WK_READ             gpio_input_bit_get(GPIOD,GPIO_PIN_2)
#define CAN_WK_READ             gpio_input_bit_get(GPIOA,GPIO_PIN_0)
#define SW_WK_READ              gpio_input_bit_get(GPIOC,GPIO_PIN_2)
#define SW2_WK_READ             gpio_input_bit_get(GPIOC,GPIO_PIN_5)
#define SW3_WK_READ             gpio_input_bit_get(GPIOA,GPIO_PIN_10)
#define MCU_OV_3A_READ          gpio_input_bit_get(GPIOC,GPIO_PIN_8)
#define PRE_DSG_READ            gpio_input_bit_get(GPIOB,GPIO_PIN_12)
#define CHG_DSG_DETN_READ       gpio_input_bit_get(GPIOB,GPIO_PIN_4)     //负载检测，低电平有负载

#define POWER_ON_12V_VDD_H      gpio_bit_set(GPIOC,GPIO_PIN_11)
#define POWER_ON_12V_VDD_L      gpio_bit_reset(GPIOC,GPIO_PIN_11)
#define AFE_VPRO_H    	        gpio_bit_set(GPIOB,GPIO_PIN_6) 
#define AFE_VPRO_L    	        gpio_bit_reset(GPIOB,GPIO_PIN_6)
#define AFE_SHIP_VDD_H          gpio_bit_set(GPIOB,GPIO_PIN_7) 
#define AFE_SHIP_VDD_L          gpio_bit_reset(GPIOB,GPIO_PIN_7)
#define CAN_RES_ON              gpio_bit_reset(GPIOA,GPIO_PIN_8)
#define CAN_RES_OFF             gpio_bit_set(GPIOA,GPIO_PIN_8)
#define MCU_1A_OFF_L            gpio_bit_reset(GPIOC,GPIO_PIN_3)
#define MCU_1A_OFF_H            gpio_bit_set(GPIOC,GPIO_PIN_3)
#define MCU_3A_OFF_L            gpio_bit_reset(GPIOB,GPIO_PIN_3) 
#define MCU_3A_OFF_H            gpio_bit_set(GPIOB,GPIO_PIN_3) 
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
#define LED_Test_H              gpio_bit_set(GPIOC,GPIO_PIN_12)
#define LED_Test_L              gpio_bit_reset(GPIOC,GPIO_PIN_12)
#define WDG_IO_H                gpio_bit_set(GPIOC,GPIO_PIN_0)
#define WDG_IO_L                gpio_bit_reset(GPIOC,GPIO_PIN_0)
#define WDG_TOGGLE              gpio_bit_toggle(GPIOC,GPIO_PIN_0)  
#define WDG_DONE_H              gpio_bit_set(GPIOA,GPIO_PIN_9)
#define WDG_DONE_L              gpio_bit_reset(GPIOA,GPIO_PIN_9)

#define TURN_DETN_LOAD_H        gpio_bit_set(GPIOB,GPIO_PIN_5)           //打开负载检测开关
#define TURN_DETN_LOAD_L        gpio_bit_reset(GPIOB,GPIO_PIN_5)          



#ifdef __cplusplus
}
#endif 
#endif
