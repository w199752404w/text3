#include "bsp_usart.h"
#include "stdio.h"

#include "bsp_gpio.h"

USART_S g_stUsart = {0};

void UART_Init(ePortId port, uint32_t uiBaud, uint32_t uiWordLen, uint32_t uiStopBit, uint32_t uiParity) {
    switch(port){
        case e485Port1:
            usart_deinit(UART3);
            g_stUsart.hUart[port]= UART3;
            //gpio_pin_remap_config(GPIO_USART2_PARTIAL_REMAP, ENABLE);
            /* 使能GPI0C时钟，用PC10、PC11为串口 */
            rcu_periph_clock_enable(RCU_GPIOC);
            rcu_periph_clock_enable(RCU_UART3);
            /*配置USARTx_Tx为复用推挽输出*/
            gpio_init(GPIOC, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_10);
            /*配置USARTx_RxPA9)为浮空输入 */
            gpio_init(GPIOC, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_11);
            /* USART 配置 */
            nvic_irq_enable(UART3_IRQn, 0, 0);                      // Enable UART3 interrupts
            usart_interrupt_enable(UART3, USART_INT_RBNE);          /* Enable the non-null interrupt in the UART3 read zone */  
            usart_interrupt_enable(UART3, USART_INT_IDLE);          /* Enable UART3 idle interrupt */
            break;
        case ebluetooth:
            usart_deinit(USART2);
            g_stUsart.hUart[port]= USART2;
            rcu_periph_clock_enable(RCU_GPIOB);
            rcu_periph_clock_enable(RCU_USART2);
            gpio_init(GPIOB, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_10);
            gpio_init(GPIOB, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_11); 
        	nvic_irq_enable(USART2_IRQn, 0, 0);                     // Enable UART2 interrupts
            usart_interrupt_enable(USART2, USART_INT_RBNE);         /* Enable the non-null interrupt in the USART2 read zone */  
            usart_interrupt_enable(USART2, USART_INT_IDLE);         /* Enable USART2 idle interrupt */
            break;
        default:
			return;
    }
    usart_baudrate_set(g_stUsart.hUart[port], uiBaud);                      //设置串口0的波特率为115200
    usart_parity_config(g_stUsart.hUart[port], uiParity);       	        // 无奇偶校验位
    usart_word_length_set(g_stUsart.hUart[port], uiWordLen);      	        // 帧数据字长
    usart_stop_bit_set(g_stUsart.hUart[port], uiStopBit);      	            // 停止位1位
    usart_transmit_config(g_stUsart.hUart[port], USART_TRANSMIT_ENABLE);    //使能发送器
    usart_receive_config(g_stUsart.hUart[port], USART_RECEIVE_ENABLE);      //使能接收器
    usart_enable(g_stUsart.hUart[port]);                                    //使能USART


  



}

void UART_Init_Dma_Receive(void)
{
    dma_parameter_struct dma_init_struct;
    /* enable DMA0 clock */
    rcu_periph_clock_enable(RCU_DMA0);
    /*configure DMA0 interrupt*/
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

    dma_channel_enable(DMA0, DMA_CH2);
}


/*****************  UART sends characters **********************/
void UART_SendByte(ePortId ePort, uint8_t ch) {
	usart_data_transmit(g_stUsart.hUart[ePort], ch);
	while(RESET == usart_flag_get(g_stUsart.hUart[ePort], USART_FLAG_TBE));
}
 
/*****************  UART sends strings **********************/
void UART_SendString(ePortId ePort, uint8_t* pucStr) {
	uint16_t i = 0;
    if(ePort == e485Port1){
        E485_CTL_1_H;
    }
	while(pucStr[i] != '\0') {
		UART_SendByte(g_stUsart.hUart[ePort],pucStr[i]);
		i++;
	}
    if(ePort == e485Port1){
        E485_CTL_1_L;
    }
}

void UART_SendBuf(ePortId ePort,uint8_t* pucData, uint16_t usLen) {
    if(NULL == pucData) {
		return;
	}
	if(0 == usLen) {
		return;
	}
    if(ePort == e485Port1){
        E485_CTL_1_H;
    }
	for(uint16_t i=0;i<usLen;i++) {
		UART_SendByte(g_stUsart.hUart[ePort],(*(pucData + i)));
	}
    if(ePort == e485Port1){
        E485_CTL_1_L;
    }
}


void UART485_SendBuf(uint8_t* pucBuf, uint16_t usLen) {
	if(NULL == pucBuf) {
		return;
	}
	if(0 == usLen) {
		return;
	}
	E485_CTL_1_H;
	for(uint16_t i=0;i<usLen;i++) {
		UART_SendByte(g_stUsart.hUart[e485Port1], *pucBuf + i);
	}
	E485_CTL_1_L;
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

