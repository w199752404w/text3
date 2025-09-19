#ifndef _BSP_EXTI_H
#define	_BSP_EXTI_H

#include "gd32f10x.h"

#ifdef USER_DEBUG
#define EXTI_DEBUG_EN	1		/* 0:��DEBUG״̬, 1:DEBUG״̬ */
#else
#define EXTI_DEBUG_EN 0
#endif
#define EXTI_DEBUG(fmt,arg...)	do{if(EXTI_DEBUG_EN){printf(fmt,##arg);printf("[%s][%d]\r\n",__func__,__LINE__);}}while(0)
#define EXTI_RETURN_FALSE	do{EXTI_DEBUG("Return failed");return false;}while(0)
//#define EXTI_RETURN_TRUE do{EXTI_DEBUG("Return success");return true;}while(0)
#define EXTI_RETURN_TRUE do{return true;}while(0)

#define REST_INT_GPIO_PORT                GPIOA
#define REST_GPIO_CLK_ENABLE()            rcu_periph_clock_enable(RCU_GPIOA);
#define REST_INT_GPIO_PIN                 GPIO_PIN_0                
#define REST_READ  	  	                  gpio_input_bit_get(GPIOA,GPIO_PIN_0)

#define REST2_INT_GPIO_PORT               GPIOC
#define REST2_GPIO_CLK_ENABLE()           rcu_periph_clock_enable(RCU_GPIOC);
#define REST2_INT_GPIO_PIN                GPIO_PIN_2                
#define REST2_READ  	  	                gpio_input_bit_get(GPIOC,GPIO_PIN_2)

#define REST5_INT_GPIO_PORT               GPIOC
#define REST5_GPIO_CLK_ENABLE()           rcu_periph_clock_enable(RCU_GPIOC);
#define REST5_INT_GPIO_PIN                GPIO_PIN_5                
#define SW2_WK_READ   	  	              gpio_input_bit_get(GPIOC,GPIO_PIN_5)


extern uint8_t g_ucSWTick ,g_ucSW2Tick ;

extern void Delay_us(unsigned long i);
extern void EXTI0_REST_Init(void);
extern void EXTI2_SW_Init(void);
extern void EXTI5_SW2_Init(void);
extern void System_Reset(void);
extern void System_Sleep(void);

#endif /* _BSP_EXTI_H */
