/**
  ******************************************************************************
  * @file    bsp_can.c
  * @author  tang
  * @version V1.0
  * @date    2023-xx-xx
  * @brief   can driver
  *******************************************************************************/ 
 
#include <string.h>

#include "bsp_can.h"


CAN_RCA_S g_stCanRCA;

can_trasnmit_message_struct transmit_message;//Sending data structures
/*
* Function Name	: CAN_GPIO_Config
* Description		: CANµÄGPIO configuration
* Input					: None
 * Output				: None
 */
static void CAN0_GPIO_Config(void) {
   /* enable CAN clock */
   rcu_periph_clock_enable(RCU_CAN0);
   rcu_periph_clock_enable(RCU_GPIOA);
   rcu_periph_clock_enable(RCU_AF);
   // gpio_pin_remap_config(GPIO_CAN_PARTIAL_REMAP,ENABLE);//PB8 and PB9 require remapping,PA11 and PA12 is not required
   /* configure CAN0 GPIO */
   gpio_init(CAN_RX_GPIO_PORT,GPIO_MODE_IPU,GPIO_OSPEED_50MHZ,CAN_RX_PIN);
   gpio_init(CAN_TX_GPIO_PORT,GPIO_MODE_AF_PP,GPIO_OSPEED_50MHZ,CAN_TX_PIN);  

}

/*
* Function Name	: CAN_NVIC_Config
* Description		: CAN NVIC config
* Input					: None
* Output				: None
 */
static void CAN0_NVIC_Config(void) {
  nvic_irq_enable(CAN0_RX1_IRQn,0,0);
}

/*
* Function Name	: CAN_Mode_Config
* Description		: CAN mode config
* The PCLK clock is 54MHz
* baud rate=54000/((1+4+4)*prescaler) 
*******************************************/
static void CAN0_Mode_Config(uint32_t uiBaud) {

    can_parameter_struct            can_parameter;
    can_filter_parameter_struct     can_filter;
    
    can_struct_para_init(CAN_INIT_STRUCT, &can_parameter);
    can_struct_para_init(CAN_FILTER_STRUCT, &can_filter);
    /* initialize CAN register */
    can_deinit(CAN0);
    /* initialize CAN */
    can_parameter.time_triggered = DISABLE;
    can_parameter.auto_bus_off_recovery = ENABLE;
    can_parameter.auto_wake_up = DISABLE;
    can_parameter.auto_retrans = ENABLE;
    can_parameter.rec_fifo_overwrite = DISABLE;
    can_parameter.trans_fifo_order = DISABLE;
    can_parameter.working_mode = CAN_NORMAL_MODE;
    can_parameter.resync_jump_width = CAN_BT_SJW_1TQ;
		//87.5MHz
    //can_parameter.time_segment_1 = CAN_BT_BS1_4TQ;
    //can_parameter.time_segment_2 = CAN_BT_BS2_4TQ;
    can_parameter.time_segment_1 = CAN_BT_BS1_6TQ;
    can_parameter.time_segment_2 = CAN_BT_BS2_1TQ;
    
    /* baudrate 250kbps */
	if(125 == uiBaud) {
	  can_parameter.prescaler = 48;
	} else if(250 == uiBaud) {
	  //can_parameter.prescaler = 24;
	  can_parameter.prescaler = 27;
	} else if(500 == uiBaud) {
		can_parameter.time_segment_1 = CAN_BT_BS1_7TQ;
    can_parameter.time_segment_2 = CAN_BT_BS2_1TQ;
	  can_parameter.prescaler = 12;
	} else if(1000 ==uiBaud) {
	  can_parameter.prescaler = 6;	
	}
    can_init(CAN0, &can_parameter);

    /* initialize filter */
    //The following two lines of code, CAN0's numbering range is 0-13, and CAN1's numbering range is 14-25; If the configuration is incorrect, it may result in the inability to enter the receive interrupt.
#ifdef  CAN0_USED
    /* CAN0 filter number */
    can_filter.filter_number = 0;
#else
    /* CAN1 filter number */
    can_filter.filter_number = 15;
#endif
    /* initialize filter */    
    can_filter.filter_mode = CAN_FILTERMODE_MASK;
    can_filter.filter_bits = CAN_FILTERBITS_32BIT;
    can_filter.filter_list_high = 0x0000;
    can_filter.filter_list_low = 0x0000;
    can_filter.filter_mask_high = 0x0000;
    can_filter.filter_mask_low = 0x0000;  
    can_filter.filter_fifo_number = CAN_FIFO1;
    can_filter.filter_enable = ENABLE;
    can_filter_init(&can_filter);
}



/*
 * Function Name	: CAN_Config
 * Description		: CAN config
 * Input					: None
 * Output					: None
 */
void CAN0_Init(uint32_t uiBaud) {
  CAN0_GPIO_Config();
  CAN0_NVIC_Config();
  CAN0_Mode_Config(uiBaud);	
	can_interrupt_enable(CAN0, CAN_INT_RFNE1);  /* enable CAN receive FIFO1 not empty interrupt */
}



/*
 * Function Name	£ºCAN_SendMsg
 * Description		£ºCAN message send
 * Input					£ºuiId, CAN ID
										pucData, data to be send
										eLength, length of data to be send
 * Output					: None
 */
bool CAN0_SendMsg(uint32_t uiId, uint8_t *pucData, uint8_t ucLength) {
	uint8_t mbox;
	uint16_t i=0;
	transmit_message.tx_ft = CAN_FT_DATA;
	if(uiId < 0x800) {
	 transmit_message.tx_ff = CAN_FF_STANDARD;//Standard Frame
	 transmit_message.tx_sfid = uiId;
	}
	else{
	 transmit_message.tx_ff = CAN_FF_EXTENDED;//Extended Frame
	 transmit_message.tx_efid = uiId;
	}
	transmit_message.tx_dlen = ucLength;
	
	for(i=0;i<ucLength; i++)
	{
		transmit_message.tx_data[i] = pucData[i];
	}
	
	mbox = can_message_transmit(CAN0, &transmit_message);
	i= 0;
	while((can_transmit_states(CAN0, mbox)==CAN_TRANSMIT_FAILED)&&(i<0XFFF))i++;	//Waiting for sending to end
	if(i == 0x0FFF) {
		CAN_RETURN_FALSE;
	}
  CAN_RETURN_TRUE; 
}

/**************************END OF FILE************************************/
