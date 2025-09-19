/**
  ******************************************************************************
  * @file    bsp_adc.c
  * @author  tangderong
  * @version V1.0
  * @date    2023-05-19
  * @brief   Using adc to read NTC thermistors
  ******************************************************************************
  * @attention
  ******************************************************************************
  */

#include "gd32f10x.h"
#include "bsp_adc.h"
#include "systick.h"
#include "math.h"


uint32_t ADCRawValue[11][ADC_DMA_BUF_SIZE];  // ADC raw data buffer 3 channels 11x sampling
uint32_t g_auiAdcBuf[ADC_DMA_BUF_SIZE]; 
uint32_t ADC1_Temp=0,ADC2_Temp=0;
uint16_t res=10;  //Acquisition resistance 10 mOhm
float CaliA=8;    //Compensation factor

void MX_ADC_Config(void) {
    /* reset ADC */
    adc_deinit(ADC0);
    /* ADC mode config */
    adc_mode_config(ADC_MODE_FREE);
    /* ADC contineous function enable */
    adc_special_function_config(ADC0, ADC_CONTINUOUS_MODE, ENABLE);
    /* ADC scan mode disable */
    adc_special_function_config(ADC0, ADC_SCAN_MODE, ENABLE);
    /* ADC data alignment config */
    adc_data_alignment_config(ADC0, ADC_DATAALIGN_RIGHT);
    /* ADC channel length config */
    adc_channel_length_config(ADC0, ADC_REGULAR_CHANNEL, 4);
 
    /* ADC regular channel config */
    adc_regular_channel_config(ADC0, 0, ADC_CHANNEL_1, ADC_SAMPLETIME_239POINT5);
    adc_regular_channel_config(ADC0, 1, ADC_CHANNEL_2, ADC_SAMPLETIME_239POINT5);
    adc_regular_channel_config(ADC0, 2, ADC_CHANNEL_3, ADC_SAMPLETIME_239POINT5);
    adc_regular_channel_config(ADC0, 3, ADC_CHANNEL_13, ADC_SAMPLETIME_239POINT5);
    
    
    
    
    /* ADC trigger config */
    adc_external_trigger_source_config(ADC0, ADC_REGULAR_CHANNEL, ADC0_1_2_EXTTRIG_REGULAR_NONE);
    adc_external_trigger_config(ADC0, ADC_REGULAR_CHANNEL, ENABLE);
    
    /* enable ADC interface */
    adc_enable(ADC0);
    delay_1ms(1);
    /* ADC calibration and reset calibration */
    adc_calibration_enable(ADC0);

    /* ADC DMA function enable */
    adc_dma_mode_enable(ADC0);
    /* ADC software trigger enable */
    adc_software_trigger_enable(ADC0, ADC_REGULAR_CHANNEL);
}

void MX_DMA_Config(void) {
    /* ADC_DMA_channel configuration */
    dma_parameter_struct dma_data_parameter;
    
    /* ADC DMA_channel configuration */
    dma_deinit(DMA0, DMA_CH0);
    
    /* initialize DMA single data mode */
    dma_data_parameter.periph_addr  = (uint32_t)(&ADC_RDATA(ADC0));
    dma_data_parameter.periph_inc   = DMA_PERIPH_INCREASE_DISABLE;
    dma_data_parameter.memory_addr  = (uint32_t)(&ADCRawValue);
    dma_data_parameter.memory_inc   = DMA_MEMORY_INCREASE_ENABLE;
    dma_data_parameter.periph_width = DMA_PERIPHERAL_WIDTH_32BIT;
    dma_data_parameter.memory_width = DMA_MEMORY_WIDTH_32BIT;  
    dma_data_parameter.direction    = DMA_PERIPHERAL_TO_MEMORY;
    dma_data_parameter.number       = 44;
    dma_data_parameter.priority     = DMA_PRIORITY_HIGH;
    dma_init(DMA0, DMA_CH0, &dma_data_parameter);
    dma_circulation_enable(DMA0, DMA_CH0);
  
    /* enable DMA channel */
    dma_channel_enable(DMA0, DMA_CH0);
		
  
	  dma_interrupt_enable(DMA0,DMA_CH0,DMA_INT_FTF);
		nvic_irq_enable(DMA0_Channel0_IRQn,1,1);

}

void MX_ADC_GPIO_Config(void) {
	rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_GPIOC);
	gpio_init(GPIOA, GPIO_MODE_AIN, GPIO_OSPEED_50MHZ, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);
    gpio_init(GPIOC, GPIO_MODE_AIN, GPIO_OSPEED_50MHZ, GPIO_PIN_3);
	/* enable ADC1 clock */
	rcu_periph_clock_enable(RCU_ADC0);
	/* enable DMA0 clock */
	rcu_periph_clock_enable(RCU_DMA0);
	/* config ADC clock */
	rcu_adc_clock_config(RCU_CKADC_CKAPB2_DIV8);
}

void ADC_Init(void) {
	MX_ADC_GPIO_Config();
	MX_DMA_Config();
	MX_ADC_Config();
}

float ADC_Get_Temp(uint32_t usData) {
	float Rp = 10000;                                     //Series resistance value, 10K
	float T2 = 273.15 + 25;                               //25 degrees Celsius
	float Bx = 3435;                                      //B value
	float Ka = 273.15;                                    //Absolute zero
	float TempRes, Temp;
	float volte;
	usData = usData & 0x0fff;
	volte = (float)usData / 4096 * 3.3;
	TempRes = volte / ((3.3-volte) / Rp);
	Temp = 1 / (1 / T2 + (log(TempRes / Rp)) / Bx) - Ka - 0.5;        //Calculation formula for NTC thermistor
	return Temp;
}

float ADC1_Get_Current(uint32_t value) {
	float Votage,Current;
	Votage= ((float)(abs(value-ADC1_Temp)))/4096*3.3*1000;      // The measured voltage is calculated, Uint£º mV
	Current=Votage/51/res*CaliA;

	return Current;
}

#define M  11
uint32_t filter(uint32_t *value_buf) 
{
	uint8_t i, j;
  uint32_t  temp;  
	for( j = 0; j < M - 1; j++ )
	{
	  for( i = 0; i < M - j - 1; i++ )
		{
			if( value_buf[i] > value_buf[i + 1] )
			{
				temp = value_buf[i];
				value_buf[i] = value_buf[i + 1];
				value_buf[i + 1] = temp;
			}
		}
	}
	return value_buf[( M - 1 ) / 2];
}


void DMA0_Channel0_IRQHandler(void)
{
	if(dma_interrupt_flag_get(DMA0,DMA_CH0,DMA_INT_FLAG_FTF) != RESET)
	{
		uint32_t raw[4][11]={0};
		for(uint8_t i=0;i<11;i++)
		{
			raw[0][i] = ADCRawValue[i][0];
			raw[1][i] = ADCRawValue[i][1];
			raw[2][i] = ADCRawValue[i][2];
            raw[3][i] = ADCRawValue[i][3];
		}
		for(uint8_t i=0;i<4;i++)
		{
			g_auiAdcBuf[i] = filter(raw[i]);
		}
		
		// Clear the Transition Complete Interrupt flag
		dma_interrupt_flag_clear(DMA0,DMA_CH0,DMA_INT_FLAG_FTF);
	}
}

