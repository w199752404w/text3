#include "bsp_usart.h"
#include "stdio.h"

USART_S g_stUsart;

void USART_Init(ePortId port, uint32_t uiBaud, uint32_t uiWordLen, uint32_t uiStopBit, uint32_t uiParity)  
{
		switch(port) {
		case eBLEPort:
			g_stUsart.hUart[port]= USART2;
		  rcu_periph_clock_enable(RCU_GPIOB);
      gpio_init(GPIOB, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_10);
      gpio_init(GPIOB, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_11); 
		  rcu_periph_clock_enable(RCU_USART2);
		  nvic_irq_enable(USART2_IRQn, 3, 0);                     //使能UART2中断
	    usart_interrupt_enable(USART2, USART_INT_RBNE);         /* 使能USART2读区非空中断 */  
      usart_interrupt_enable(USART2, USART_INT_IDLE);         /* 使能USART2空闲中断 */
		break;
		case eLEDPort:
			g_stUsart.hUart[port]= USART1;
		  rcu_periph_clock_enable(RCU_GPIOA);
      gpio_init(GPIOA, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_2);
      gpio_init(GPIOA, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_3);
		  rcu_periph_clock_enable(RCU_USART1);
			nvic_irq_enable(USART1_IRQn, 3, 1);                     //使能USART1中断
	    usart_interrupt_enable(USART1, USART_INT_RBNE);         /* 使能USART1读区非空中断 */  
      usart_interrupt_enable(USART1, USART_INT_IDLE);         /* 使能USART1空闲中断 */
		break;
		case e485Port:
			g_stUsart.hUart[port]= USART0;
		  gpio_init(GPIOA, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_9);
      gpio_init(GPIOA, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_10);
		  rcu_periph_clock_enable(RCU_USART0);
			nvic_irq_enable(USART0_IRQn, 3, 2);                     //使能USART0中断
		  usart_interrupt_enable(USART0, USART_INT_RBNE);         /* 使能USART0读区非空中断 */  
   // usart_interrupt_enable(USART0, USART_INT_IDLE);         /* 使能USART0空闲中断 */
		break;
		default:
			return;
	}

       
    usart_baudrate_set(g_stUsart.hUart[port], uiBaud);        // 波特率
    usart_parity_config(g_stUsart.hUart[port], uiParity);     // 无校检
    usart_word_length_set(g_stUsart.hUart[port], uiWordLen);  // 8位数据位
    usart_stop_bit_set(g_stUsart.hUart[port], uiStopBit);     // 1位停止位
    usart_transmit_config(g_stUsart.hUart[port], USART_TRANSMIT_ENABLE);  // 使能串口发送
 // usart_receive_config(g_stUsart.hUart[port], USART_RECEIVE_ENABLE);  // 使能串口接收
    usart_enable(g_stUsart.hUart[port]);  // 使能串口
	  usart_receive_config(g_stUsart.hUart[port], USART_RECEIVE_DISABLE);  // 静止能串口接收
    usart_interrupt_disable(USART0, USART_INT_IDLE);                     //* 禁止USART0空闲中断
}




/*****************  UART发送字符 **********************/
void UART_SendBuf(ePortId ePort, char ch)
{
	usart_data_transmit(g_stUsart.hUart[ePort], (uint8_t)ch);
	while(RESET == usart_flag_get(g_stUsart.hUart[ePort], USART_FLAG_TBE));
}
 
/*****************  UART发送字符串 **********************/
void UART_SendString(ePortId ePort, char* fmt)
{
	uint16_t i = 0;
	while(fmt[i] != '\0')
	{
		UART_SendBuf(ePort, fmt[i]);
		i++;
	}
}




///重定向c库函数printf到串口，重定向后可使用printf函数
int fputc(int ch, FILE *f)
{
		/* 发送一个字节数据到串口 */
		usart_data_transmit(USART2, (uint8_t)ch);
		
		/* 等待发送完毕 */
		while (usart_flag_get(USART2, USART_FLAG_TBE) == RESET);		
	
		return (ch);
}

///重定向c库函数scanf到串口，重写向后可使用scanf、getchar等函数
int fgetc(FILE *f)
{
		/* 等待串口输入数据 */
		while (usart_flag_get(USART2, USART_FLAG_RBNE) == RESET);

		return (int)usart_data_receive(USART2);
}


////查询法发送字符串
//void USART0_sendStr(const char*msg)
//{
//	uint32_t i;
//	usart_flag_clear(USART0,USART_FLAG_TC);  //为了使用TC，在发送前要先清除TC
//	for(i=0;msg[i];i++)
//	{
//		while(!usart_flag_get(USART0,USART_FLAG_TBE)){}  //等待TBE硬件置1
//		usart_data_transmit(USART0,msg[i]);    //向TDATA寄存器写入字节数据
//	}
//	while(!usart_flag_get(USART0,USART_FLAG_TC)){}  //等待TC硬件置1，代表发送完成
//}

//void USART0_IRQHandler(void)
//{
//    if(RESET != usart_interrupt_flag_get(USART0, USART_INT_FLAG_RBNE)){
//        /* receive data */
//        receiver_buffer[rxcount++] = usart_data_receive(USART0);
//        if(rxcount == receivesize){
//            usart_interrupt_disable(USART0, USART_INT_RBNE);
//        }
//    }

//    if(RESET != usart_interrupt_flag_get(USART0, USART_INT_FLAG_TBE)){
//        /* transmit data */
//        usart_data_transmit(USART0, transmitter_buffer[txcount++]);
//        if(txcount == transfersize){
//            usart_interrupt_disable(USART0, USART_INT_TBE);
//        }
//    }
//}
void USART2_IRQHandler(void) {
	/******receive IRQ*******/
	if(RESET != usart_interrupt_flag_get(USART2, USART_INT_FLAG_RBNE)){
		/* receive data */
     usart_data_receive(USART2);
		 usart_interrupt_flag_clear(USART2, USART_INT_FLAG_RBNE); //清中断标志
	}
	/******Idle IRQ*******/
	if(RESET != usart_interrupt_flag_get(USART2, USART_INT_FLAG_IDLE)) {
		usart_interrupt_flag_clear(USART2,USART_INT_FLAG_IDLE); /* 清除空闲中断标志位 */
		usart_data_receive(USART2);                             /* 清除接收完成标志位 */
	}
	if(RESET != usart_interrupt_flag_get(USART2, USART_INT_FLAG_RBNE_ORERR)){
		usart_interrupt_flag_clear(USART2,USART_INT_FLAG_RBNE_ORERR); /* 清除溢出中断标志位 */
		usart_data_receive(USART2);                              /* 清除溢出完成标志位 */
	}
//	if(usart_interrupt_flag_get(USART2,USART_INT_FLAG_ERR_ORERR) != RESET
//	||usart_interrupt_flag_get(USART2,USART_INT_FLAG_ERR_NERR) != RESET
//	||usart_interrupt_flag_get(USART2,USART_INT_FLAG_ERR_FERR) != RESET)
//	{
//		usart_interrupt_flag_clear(USART2,USART_INT_FLAG_ERR_ORERR);
//		usart_interrupt_flag_clear(USART2,USART_INT_FLAG_ERR_NERR);
//		usart_interrupt_flag_clear(USART2,USART_INT_FLAG_ERR_FERR);
//		return;
//	}
}

/*********************************************END OF FILE**********************/
