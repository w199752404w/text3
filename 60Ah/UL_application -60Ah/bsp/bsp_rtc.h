#ifndef __BSP_RTC_H
#define __BSP_RTC_H

#include "gd32f10x.h"
#include "time.h"

#include "main.h"

#ifdef USER_DEBUG
#define RTC_DEBUG_EN	1		/* 0: non-DEBUG state, 1: DEBUG status */
#else
#define RTC_DEBUG_EN 0
#endif
#define RTC_DEBUG(fmt,arg...)	do{if(RTC_DEBUG_EN){printf(fmt,##arg);printf("[%s][%d]\r\n",__func__,__LINE__);}}while(0)
#define RTC_RETURN_FALSE	do{RTC_DEBUG("Return failed");return false;}while(0)
//#define RTC_RETURN_TRUE do{RTC_DEBUG("Return success");return true;}while(0)
#define RTC_RETURN_TRUE do{return true;}while(0)


//extern __IO uint32_t timedisplay;

//void nvic_configuration(void);
//void RTC_Config(void);
//void RTC_Init(void);
//uint32_t time_regulate(void);
//void time_adjust(void);
//void time_display(uint32_t timevar);
//void time_show(void);
//uint8_t usart_scanf(uint32_t value);
typedef struct{
	uint8_t hour;
	uint8_t min;
	uint8_t sec;
	uint16_t w_year;
	uint8_t w_month;
	uint8_t w_day;
}_calender_obj;

extern _calender_obj  calender;
extern uint8_t const  month_table[12];
extern uint8_t guc_FirstFlag;

static void RTC_Config(void);
static void RTC_NVIC_Config(void);
static uint8_t Is_Leap_Year(uint16_t year);//Determine if it is a leap year
extern uint8_t RTC_Init(void);
extern uint8_t RTC_Set(uint16_t syear, uint8_t smonth, uint8_t sday, uint8_t shour, uint8_t sminute, uint8_t ssec);
extern time_t DS1302_gtime(void);
extern void RTC_Get(void);              //Obtain RTC_ Convert the value of CNT to date time
extern void RTC_Time_Verify(void);         //Check if the time has been set

#endif
