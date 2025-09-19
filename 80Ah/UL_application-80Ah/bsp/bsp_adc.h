/**
******************************************************************************
* @file    tsensor.h
* @author  AE Team
* @version V1.1.0
* @date    28/08/2019
* @brief   This file contains all the tsensor inc file for the library.
******************************************************************************
* @copy
*
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
* TIME. AS A RESULT, MindMotion SHALL NOT BE HELD LIABLE FOR ANY
* DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
* FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
* CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*
* <h2><center>&copy; COPYRIGHT 2019 MindMotion</center></h2>
*/
#ifndef __BSP_ADC_H
#define __BSP_ADC_H
#include "stdint.h"
//////////////////////////////////////////////////////////////////////////////////
//Development boards
//ADC Driver code
//////////////////////////////////////////////////////////////////////////////////

#define ADC_DMA_BUF_SIZE 3
extern uint32_t ADCRawValue[11][ADC_DMA_BUF_SIZE];  // ADC raw data buffer 3 channels 11x sampling
extern uint32_t g_auiAdcBuf[ADC_DMA_BUF_SIZE];
extern uint32_t ADC1_Temp,ADC2_Temp;

extern void ADC_Init(void);        
extern float ADC_Get_Temp(uint32_t usData);       //Get the temperature value
extern float ADC1_Get_Current(uint32_t value);    //Obtain the current value
extern uint32_t filter(uint32_t *value_buf);      //Filter, calculate the median

#endif
/*-------------------------(C) COPYRIGHT 2019 MindMotion ----------------------*/
