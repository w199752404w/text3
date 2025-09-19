#include "bsp_rtc.h"
#include <stdio.h>

_calender_obj  calender;
uint8_t const  month_table[12]={31,28,31,30,31,30,31,31,30,31,30,31}; //Number of months and days in ordinary years
uint32_t  timecount=0;
uint8_t guc_FirstFlag = 0;

void RTC_Config(void){
	rcu_periph_clock_enable(RCU_BKPI);      /* enable BKPI clocks */
	rcu_periph_clock_enable(RCU_PMU);       /* enable PMU clocks */
	pmu_backup_write_enable();              /* allow access to BKP domain */
	if(bkp_data_read(BKP_DATA_0) != 0xA5A5){
	  bkp_deinit();                         /* reset backup domain */
	}
	rcu_osci_on(RCU_LXTAL);                 /* enable LXTAL */
	rcu_osci_stab_wait(RCU_LXTAL);          /* wait till LXTAL is ready */
	rcu_rtc_clock_config(RCU_RTCSRC_LXTAL); /* select RCU_LXTAL as RTC clock source */
  rcu_periph_clock_enable(RCU_RTC);       /* enable RTC Clock */
  rtc_register_sync_wait();               /* wait for RTC registers synchronization */
  rtc_lwoff_wait();                       /* wait until last write operation on RTC registers has finished */
  rtc_interrupt_enable(RTC_INT_SECOND);   /* enable the RTC second interrupt*/
  rtc_lwoff_wait();                       /* wait until last write operation on RTC registers has finished */
  rtc_prescaler_set(32767);               /* set RTC prescaler: set RTC period to 1s */
  rtc_lwoff_wait();                       /* wait until last write operation on RTC registers has finished */	
}

void RTC_NVIC_Config(void){
    nvic_irq_enable(RTC_IRQn,1,0);
}

uint8_t RTC_Init(void){
	RTC_Config();
	RTC_NVIC_Config();
	return 0;
}


uint8_t RTC_Set(uint16_t syear, uint8_t smonth, uint8_t sday, uint8_t shour, uint8_t smin, uint8_t ssec){
	uint32_t seccounts = 0;
	uint16_t temp_year = 1970;
	uint8_t temp_month;
	if(syear<1970 || syear>2099){  //The time set is unreasonable
		return 1;
	}
	
	//Seconds throughout the year
	while(temp_year < syear){
		if(Is_Leap_Year(temp_year))seccounts += 31622400; //Leap year, the number of seconds in a year
		else seccounts += 31536000;                       //Secs per year
		temp_year++;
	}

	//Seconds of the entire month
	smonth--;
	for(temp_month = 0; temp_month<smonth; temp_month++){
		seccounts += (uint32_t)month_table[temp_month]*86400;
		if(Is_Leap_Year(syear)&&temp_month==1)seccounts += 86400; //If the year set is a leap year, an additional day will be added in February
	}
	
	//Processing of day, hour, minute, and second
	seccounts += (uint32_t)(sday-1)*86400; //Seconds per day
	seccounts += (uint32_t)shour*3600;     //hour
	seccounts += (uint32_t)smin*60;        //minute
	seccounts += ssec;                     //second
	
	rtc_lwoff_wait();
	rtc_counter_set(seccounts);
	return 0;
}

void RTC_Get(void){
  timecount = rtc_counter_get();   /* Obtain RTC_ Convert the value of CNT to timecount */
	/* Convert timecount to date time and assign it to calender */
	uint32_t temp_days = timecount/86400;
	uint16_t temp_year = 1970;
	uint16_t temp_month;
	
	//Whole year in processing days
	if(temp_days>0){
		while(temp_days>=365){
			if(Is_Leap_Year(temp_year)){//If it's a leap year
				if(temp_days>365){
					temp_days -= 366;
				}
				else{
					break;
				}
			}else{
				temp_days -= 365;
			}
			temp_year++;
		}
		calender.w_year = temp_year;
		
		//If there is less than a year left, handle it for the entire month
		temp_month = 1;           //Used for temporary storage of months
		while(temp_days >= 28){   //Over a month
			if(Is_Leap_Year(calender.w_year) && temp_month == 2){
				if(temp_days>=29){    //February in leap years is 29 days
					temp_days -= 29;
				}else{
					break;
				}
			}else{
				if(temp_days >= month_table[temp_month-1]){//Is the remaining days greater than temp_ Month is the total number of days in this month
					temp_days -= month_table[temp_month-1];
				}else{
					break;
				}
			}
			temp_month++;
		}
	}

	calender.w_month = temp_month;
	calender.w_day = temp_days+1;
	
	//Process the remaining seconds of less than one day, hours: minutes: seconds
	uint32_t temp_seconds = timecount%86400; //Seconds in less than a day
	calender.hour = temp_seconds/3600;
	calender.min = (temp_seconds%3600)/60;
	calender.sec = temp_seconds%60;
}

uint8_t Is_Leap_Year(uint16_t year){ //Determine whether year is a leap year
	if(year%4 == 0){
		if(year%100 == 0){
			if(year%400 == 0)
				return 1;
			else
				return 0;
		}else{
			return 1;
		}
	}else{
		return 0;
	}
}

time_t DS1302_gtime(void) {
	struct tm stDt;
	if(calender.w_year < 1900){
		calender.w_year = 2020;
	}
	stDt.tm_year = calender.w_year -1900;
	stDt.tm_mon = calender.w_month - 1;
	stDt.tm_mday = calender.w_day;
	stDt.tm_hour = calender.hour;
	stDt.tm_min = calender.min;
	stDt.tm_sec = calender.sec;
	return mktime(&stDt) - 8 * 3600;
}

void RTC_Time_Verify(void){   //Check if the time has been set
	if(bkp_data_read(BKP_DATA_0) != 0xA5A5){
		 guc_FirstFlag = 1;
		 RTC_Set(2023, 1, 1, 1, 1, 1);
     pmu_backup_write_enable();
     bkp_data_write(BKP_DATA_0, 0xA5A5);
     pmu_backup_write_disable();
		 RTC_DEBUG("\r\n Time initialization completed \r\n");
	}
}

void RTC_IRQHandler(void){
  if (rtc_flag_get(RTC_FLAG_SECOND) != RESET){
     rtc_flag_clear(RTC_FLAG_SECOND);   //clear the RTC second interrupt flag
	   RTC_Get();
 //  RTC_DEBUG("Now time is: %d-%02d-%02d %02d:%02d:%02d\r\n",calender.w_year,calender.w_month,calender.w_day,calender.hour,calender.min,calender.sec);
  }
}
