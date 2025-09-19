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
		  nvic_irq_enable(USART2_IRQn, 3, 0);                     //ʹ��UART2�ж�
	    usart_interrupt_enable(USART2, USART_INT_RBNE);         /* ʹ��USART2�����ǿ��ж� */  
      usart_interrupt_enable(USART2, USART_INT_IDLE);         /* ʹ��USART2�����ж� */
		break;
		case eLEDPort:
			g_stUsart.hUart[port]= USART1;
		  rcu_periph_clock_enable(RCU_GPIOA);
      gpio_init(GPIOA, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_2);
      gpio_init(GPIOA, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_3);
		  rcu_periph_clock_enable(RCU_USART1);
			nvic_irq_enable(USART1_IRQn, 3, 1);                     //ʹ��USART1�ж�
	    usart_interrupt_enable(USART1, USART_INT_RBNE);         /* ʹ��USART1�����ǿ��ж� */  
      usart_interrupt_enable(USART1, USART_INT_IDLE);         /* ʹ��USART1�����ж� */
		break;
		case e485Port:
			g_stUsart.hUart[port]= USART0;
		  gpio_init(GPIOA, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_9);
      gpio_init(GPIOA, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_10);
		  rcu_periph_clock_enable(RCU_USART0);
			nvic_irq_enable(USART0_IRQn, 3, 2);                     //ʹ��USART0�ж�
		  usart_interrupt_enable(USART0, USART_INT_RBNE);         /* ʹ��USART0�����ǿ��ж� */  
   // usart_interrupt_enable(USART0, USART_INT_IDLE);         /* ʹ��USART0�����ж� */
		break;
		default:
			return;
	}

       
    usart_baudrate_set(g_stUsart.hUart[port], uiBaud);        // ������
    usart_parity_config(g_stUsart.hUart[port], uiParity);     // ��У��
    usart_word_length_set(g_stUsart.hUart[port], uiWordLen);  // 8λ����λ
    usart_stop_bit_set(g_stUsart.hUart[port], uiStopBit);     // 1λֹͣλ
    usart_transmit_config(g_stUsart.hUart[port], USART_TRANSMIT_ENABLE);  // ʹ�ܴ��ڷ���
 // usart_receive_config(g_stUsart.hUart[port], USART_RECEIVE_ENABLE);  // ʹ�ܴ��ڽ���
    usart_enable(g_stUsart.hUart[port]);  // ʹ�ܴ���
	  usart_receive_config(g_stUsart.hUart[port], USART_RECEIVE_DISABLE);  // ��ֹ�ܴ��ڽ���
    usart_interrupt_disable(USART0, USART_INT_IDLE);                     //* ��ֹUSART0�����ж�
}




/*****************  UART�����ַ� **********************/
void UART_SendBuf(ePortId ePort, char ch)
{
	usart_data_transmit(g_stUsart.hUart[ePort], (uint8_t)ch);
	while(RESET == usart_flag_get(g_stUsart.hUart[ePort], USART_FLAG_TBE));
}
 
/*****************  UART�����ַ��� **********************/
void UART_SendString(ePortId ePort, char* fmt)
{
	uint16_t i = 0;
	while(fmt[i] != '\0')
	{
		UART_SendBuf(ePort, fmt[i]);
		i++;
	}
}




///�ض���c�⺯��printf�����ڣ��ض�����ʹ��printf����
int fputc(int ch, FILE *f)
{
		/* ����һ���ֽ����ݵ����� */
		usart_data_transmit(USART2, (uint8_t)ch);
		
		/* �ȴ�������� */
		while (usart_flag_get(USART2, USART_FLAG_TBE) == RESET);		
	
		return (ch);
}

///�ض���c�⺯��scanf�����ڣ���д����ʹ��scanf��getchar�Ⱥ���
int fgetc(FILE *f)
{
		/* �ȴ������������� */
		while (usart_flag_get(USART2, USART_FLAG_RBNE) == RESET);

		return (int)usart_data_receive(USART2);
}


////��ѯ�������ַ���
//void USART0_sendStr(const char*msg)
//{
//	uint32_t i;
//	usart_flag_clear(USART0,USART_FLAG_TC);  //Ϊ��ʹ��TC���ڷ���ǰҪ�����TC
//	for(i=0;msg[i];i++)
//	{
//		while(!usart_flag_get(USART0,USART_FLAG_TBE)){}  //�ȴ�TBEӲ����1
//		usart_data_transmit(USART0,msg[i]);    //��TDATA�Ĵ���д���ֽ�����
//	}
//	while(!usart_flag_get(USART0,USART_FLAG_TC)){}  //�ȴ�TCӲ����1�����������
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
		 usart_interrupt_flag_clear(USART2, USART_INT_FLAG_RBNE); //���жϱ�־
	}
	/******Idle IRQ*******/
	if(RESET != usart_interrupt_flag_get(USART2, USART_INT_FLAG_IDLE)) {
		usart_interrupt_flag_clear(USART2,USART_INT_FLAG_IDLE); /* ��������жϱ�־λ */
		usart_data_receive(USART2);                             /* ���������ɱ�־λ */
	}
	if(RESET != usart_interrupt_flag_get(USART2, USART_INT_FLAG_RBNE_ORERR)){
		usart_interrupt_flag_clear(USART2,USART_INT_FLAG_RBNE_ORERR); /* �������жϱ�־λ */
		usart_data_receive(USART2);                              /* ��������ɱ�־λ */
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
