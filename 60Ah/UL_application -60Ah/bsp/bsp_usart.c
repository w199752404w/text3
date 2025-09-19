#include "bsp_usart.h"
#include "stdio.h"

USART_S g_stUsart = {0};

void UART_Init(uint32_t uiBaud, uint32_t uiWordLen, uint32_t uiStopBit, uint32_t uiParity) {
	usart_deinit(USART2);
	rcu_periph_clock_enable(RCU_GPIOB);
  gpio_init(GPIOB, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_10);
  gpio_init(GPIOB, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_11); 
	rcu_periph_clock_enable(RCU_USART2);
	nvic_irq_enable(USART2_IRQn, 0, 0);                     // Enable UART2 interrupts
	usart_interrupt_enable(USART2, USART_INT_RBNE);         /* Enable the non-null interrupt in the USART2 read zone */  
  usart_interrupt_enable(USART2, USART_INT_IDLE);         /* Enable USART2 idle interrupt */
	usart_baudrate_set(USART2, uiBaud);        // baud rate
	usart_parity_config(USART2, uiParity);     // There is no calibration
	usart_word_length_set(USART2, uiWordLen);  // 8 bits of data
	usart_stop_bit_set(USART2, uiStopBit);     // 1 stop bit
  usart_transmit_config(USART2, USART_TRANSMIT_ENABLE);  // Serial port transmission is enabled
	usart_receive_config(USART2, USART_RECEIVE_ENABLE);  // Serial port reception is enabled
	usart_enable(USART2);  // Serial ports are enabled
}

void UART_Init_Dma_Receive(void)
{
	  dma_parameter_struct dma_init_struct;
    /* enable DMA0 clock */
    rcu_periph_clock_enable(RCU_DMA0);
    /*configure DMA0 interrupt*/
    // nvic_irq_enable(DMA0_Channel4_IRQn, 0, 1);
    /* deinitialize DMA channel4 (USART0 rx) */
    dma_deinit(DMA0, DMA_CH2);
    dma_struct_para_init(&dma_init_struct);

    dma_init_struct.direction = DMA_PERIPHERAL_TO_MEMORY;
    dma_init_struct.memory_addr = (uint32_t)&(g_stUsart.aucRBuf);
    dma_init_struct.memory_inc = DMA_MEMORY_INCREASE_ENABLE;
    dma_init_struct.memory_width = DMA_MEMORY_WIDTH_8BIT;
    dma_init_struct.number = sizeof(g_stUsart.aucRBuf);
    dma_init_struct.periph_addr = (uint32_t)&(USART_DATA(USART2));
    dma_init_struct.periph_inc = DMA_PERIPH_INCREASE_DISABLE;
    dma_init_struct.memory_width = DMA_PERIPHERAL_WIDTH_8BIT;
    dma_init_struct.priority = DMA_PRIORITY_ULTRA_HIGH;

    dma_init(DMA0, DMA_CH2, &dma_init_struct);
    dma_circulation_disable(DMA0, DMA_CH2);
    dma_memory_to_memory_disable(DMA0, DMA_CH2);
    usart_dma_receive_config(USART2, USART_RECEIVE_DMA_ENABLE);  // Enable DMA reception
//		nvic_irq_enable(DMA0_Channel2_IRQn, 2, 1);
//	  dma_interrupt_enable(DMA0, DMA_CH2, DMA_INT_FTF|DMA_INT_ERR);
    /* enable DMA0 channel4 transfer complete interrupt */
    // dma_interrupt_enable(DMA0, DMA_CH4, DMA_INT_FTF);
    /* enable DMA0 channel4 */
    dma_channel_enable(DMA0, DMA_CH2);
}


/*****************  UART sends characters **********************/
void UART_SendBuf(char ch) {
	usart_data_transmit(USART2, (uint8_t)ch);
	while(RESET == usart_flag_get(USART2, USART_FLAG_TBE));
}
 
/*****************  UART sends strings **********************/
void UART_SendString(char* fmt) {
	uint16_t i = 0;
	while(fmt[i] != '\0') {
		UART_SendBuf(fmt[i]);
		i++;
	}
}

void UART_Send(uint8_t* pucData, uint16_t usLen) {
	for(uint16_t i=0;i<usLen;i++) {
		UART_SendBuf(*(pucData + i));
	}
}

//Redirect the C library function printf to the serial port, and use the printf function after redirection
int fputc(int ch, FILE *f) {
	/* Send one byte of data to the serial port */
	usart_data_transmit(USART2, (uint8_t)ch);
	/* Wait for the delivery to complete */
	while (usart_flag_get(USART2, USART_FLAG_TBE) == RESET);
	return (ch);
}

//Redirect the C library function scanf to the serial port, and use functions such as scanf and getchar after rewriting
int fgetc(FILE *f) {
	/* Wait for the serial port to input data */
	while (usart_flag_get(USART2, USART_FLAG_RBNE) == RESET);
	return (int)usart_data_receive(USART2);
}

/*********************************************END OF FILE**********************/
