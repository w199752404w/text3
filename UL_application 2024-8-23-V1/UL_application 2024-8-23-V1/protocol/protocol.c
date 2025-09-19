#include "protocol.h"
#include "config.h"
#include "bsp_usart.h"
#include "bsp.h"
#include "bsp_gd25q32.h"
#include "history.h"

#include "parallel.h"
#include "local.h"
#include "modbus_rtu.h"


#include <string.h>
uint8_t USART2_RxFlag = 0;
uint8_t UART3_RxFlag = 0;
uint8_t USART2_UgFlag = 0;
uint8_t UART3_UgFlag = 0;
uint8_t TimeFlag = 0;
uint8_t Timeout = 0;
//uint32_t g_aullIEMI_IDss[PRL_MAX_NODE_NUM] = {0};
uint16_t USART_RX_LEN[ePortNum] = {0};


/************ RS485 receive IRQ callback ***************/
void USART2_IRQHandler(void) {
	/******receive IRQ*******/
	if(RESET != usart_interrupt_flag_get(USART2, USART_INT_FLAG_RBNE)){
		 usart_interrupt_flag_clear(USART2, USART_INT_FLAG_RBNE); //Clear the break sign
	}
	/******Idle IRQ*******/
	if(RESET != usart_interrupt_flag_get(USART2, USART_INT_FLAG_IDLE)) {
		usart_interrupt_flag_clear(USART2,USART_INT_FLAG_IDLE); /* Clear the idle interrupt flag */
		usart_data_receive(USART2);                             /* Clear the Receive Complete flag */
		if(USART2_UgFlag == 0){
		  USART2_RxFlag = 1;
		}else{
			USART2_RxFlag = 0;
			g_stUsart.usRLen[ebluetooth] = sizeof(g_stUsart.aucRBuf) - dma_transfer_number_get(DMA0, DMA_CH2);
			if(g_stUsart.usRLen[ebluetooth] > 1020 && g_stUsart.usRLen[ebluetooth] < 1200){
				USART2_RxFlag = 1;
			}
			if(g_stUsart.aucRBuf[ebluetooth][0] == 0x04 && g_stUsart.aucRBuf[ebluetooth][1] == 0x01 && g_stUsart.aucRBuf[ebluetooth][2]==0x00 && g_stUsart.aucRBuf[ebluetooth][3] == 0xFF){
			  USART2_RxFlag = 1;
		  }
		}
	}
	if(RESET != usart_interrupt_flag_get(USART2, USART_INT_FLAG_RBNE_ORERR)){
		usart_interrupt_flag_clear(USART2,USART_INT_FLAG_RBNE_ORERR); /* Clear the overflow break flag */
		usart_data_receive(USART2);                              /* Clear the overflow completion flag */
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

/************ RS485 receive IRQ callback ***************/
void UART3_IRQHandler(void) {
    	/******receive IRQ*******/
    if(RESET != usart_interrupt_flag_get(UART3, USART_INT_FLAG_RBNE)){
        /* receive data */
        g_stUsart.aucRBuf[e485Port1][USART_RX_LEN[e485Port1]] = usart_data_receive(UART3);
        USART_RX_LEN[e485Port1]++;
        g_stUsart.usRLen[e485Port1] = 0;
        if(USART_RX_LEN[e485Port1] >= USART_MLEN) {
            USART_RX_LEN[e485Port1] = 0;
        }     
    }
    /******Idle IRQ*******/
    if(RESET != usart_interrupt_flag_get(UART3, USART_INT_FLAG_IDLE)){
        usart_interrupt_flag_clear(UART3,USART_INT_FLAG_IDLE); /* 清除空闲中断标志位 */
        usart_data_receive(UART3);                             /* 清除接收完成标志位 */
        /* transmit data */
        g_stUsart.usRLen[e485Port1] = USART_RX_LEN[e485Port1];
        USART_RX_LEN[e485Port1] = 0;
        if(mbs_recv(g_stUsart.aucRBuf[e485Port1], g_stUsart.usRLen[e485Port1], g_stUsart.aaucSBuf[e485Port1], &g_stUsart.usSLen[e485Port1])) {
            UART_SendBuf(e485Port1, g_stUsart.aaucSBuf[e485Port1], g_stUsart.usSLen[e485Port1]);
        }
        memset(g_stUsart.aucRBuf[e485Port1],0,sizeof(g_stUsart.aucRBuf[e485Port1]));
        USART_RX_LEN[e485Port1]=0;
    }
}


/********外部CAN中断接收函数***************/
/*
*********************************************************************************************************
*	The name of the function: CAN0_RX0_IRQHandler
*	Feature description: CAN0 receives an interrupt and receives the data and sends it back
*	Parameters: No
*	Return value: No
*	Created by: Mxg
*	Creation time: 20230214
*********************************************************************************************************
*/
//void CAN0_RX1_IRQHandler(void) {
////	can_receive_message_struct can0_rxmes; 
////	memset(&can0_rxmes,0,sizeof(can0_rxmes));
////	can_message_receive(CAN0, CAN_FIFO1, &can0_rxmes);
////	CAN_RBUF_S canRBuf;
////	canRBuf.uiId = can0_rxmes.rx_efid;
////	memcpy(canRBuf.aucData, can0_rxmes.rx_data, canRBuf.ucDLC);
////	g_stCanRCA.astRBuf[g_stCanRCA.usRIdx] = canRBuf;
////			g_stCanRCA.usRIdx++;
////		if(g_stCanRCA.usRIdx >= FDCAN_MAX_DATA_CACHE) {
////			g_stCanRCA.usRIdx = 0;
////		}
////	return;
//////	if(can_interrupt_flag_get(CAN0,CAN_INT_FLAG_RFL1)){
//////		can_receive_message_struct can0_rxmes; 
//////		memset(&can0_rxmes,0,sizeof(can0_rxmes));
//////		can_message_receive(CAN0, CAN_FIFO1, &can0_rxmes);
//////		CAN_RBUF_S canRBuf;
//////		canRBuf.uiId = can0_rxmes.rx_efid;
//////		memcpy(canRBuf.aucData, can0_rxmes.rx_data, canRBuf.ucDLC);
//////		
//////		g_stCanRCA.astRBuf[g_stCanRCA.usRIdx] = canRBuf;
//////		g_stCanRCA.usRIdx++;
//////		if(g_stCanRCA.usRIdx >= FDCAN_MAX_DATA_CACHE) {
//////			g_stCanRCA.usRIdx = 0;
//////		}
//////		
//////	}
//	can_receive_message_struct can0_rxmes;  //Receive data structure
//	can_struct_para_init(CAN_RX_MESSAGE_STRUCT, &can0_rxmes);//汪
//	memset(&can0_rxmes,0,sizeof(can0_rxmes));
//	can_message_receive(CAN0, CAN_FIFO1, &can0_rxmes);
//	CAN_RBUF_S canRBuf;
//	if(can0_rxmes.rx_efid == 0) {
//		canRBuf.uiId = can0_rxmes.rx_sfid;
//	} else {
//		canRBuf.uiId = can0_rxmes.rx_efid;
//	}
//	canRBuf.ucDLC = can0_rxmes.rx_dlen;
//	memcpy(canRBuf.aucData, can0_rxmes.rx_data, canRBuf.ucDLC);
//	/* The ID number of each master and slave that is required for receiving negotiations */
//	if((canRBuf.uiId & 0x1FFFFF) != 0x1F0E00) {
//		g_stCanRCA.astRBuf[g_stCanRCA.usRIdx] = canRBuf;
//		g_stCanRCA.usRIdx++;
//		if(g_stCanRCA.usRIdx >= FDCAN_MAX_DATA_CACHE) {
//			g_stCanRCA.usRIdx = 0;
//		}
//		return;
//	}
//#ifndef PARALLEL_EN
//	return;
//#endif
//	for(uint8_t i = 1; i < 8;i++) {
//		if(canRBuf.aucData[i] >= '0' && canRBuf.aucData[i] <= '9') {
//			canRBuf.aucData[i] = canRBuf.aucData[i] - '0';
//		} else if(canRBuf.aucData[i] >= 'a' && canRBuf.aucData[i] <= 'f') {
//			canRBuf.aucData[i] = canRBuf.aucData[i] - 'a' + 10;
//		} else if(canRBuf.aucData[i] >= 'A' && canRBuf.aucData[i] <= 'F') {
//			canRBuf.aucData[i] = canRBuf.aucData[i] - 'A' + 10;
//		} else {
//			//illegal bluetooth name
//		}
//	}
//	uint32_t uiID = canRBuf.aucData[1] * 0x1000000;
//	uiID += canRBuf.aucData[2] * 0x100000;
//	uiID += canRBuf.aucData[3] * 0x10000;
//	uiID += canRBuf.aucData[4] * 0x1000;
//	uiID += canRBuf.aucData[5] * 0x100;
//	uiID += canRBuf.aucData[6] * 0x10;
//	uiID += canRBuf.aucData[7];

//	uint8_t ucSelfId = 0;
//	uint8_t ucDevNum = 1;


//	if(ucDevNum > 2) {
//		g_stLocalStat = eLocalStatInitFail;
//		return;
//	}
//	g_stPrl.ucDevNum = ucDevNum;
//}
void CAN0_RX1_IRQHandler(void) {
	can_receive_message_struct receive_message;  //Receive data structure
	memset(&receive_message,0,sizeof(receive_message));
	can_message_receive(CAN0, CAN_FIFO1, &receive_message);
	CAN_RBUF_S canRBuf;
	if(receive_message.rx_efid == 0) {
		canRBuf.uiId = receive_message.rx_sfid;
	} else {
		canRBuf.uiId = receive_message.rx_efid;
	}
	canRBuf.ucDLC = receive_message.rx_dlen;
	memcpy(canRBuf.aucData, receive_message.rx_data, canRBuf.ucDLC);
	/* The ID number of each master and slave that is required for receiving negotiations */
	if((canRBuf.uiId & 0x1FFFFF) != 0x1F0E00) {
		g_stCanRCA.astRBuf[g_stCanRCA.usRIdx] = canRBuf;
		g_stCanRCA.usRIdx++;
		if(g_stCanRCA.usRIdx >= FDCAN_MAX_DATA_CACHE) {
			g_stCanRCA.usRIdx = 0;
		}
		return;
	}
#ifndef PARALLEL_EN
	return;
#endif
	for(uint8_t i = 1; i < 8;i++) {
		if(canRBuf.aucData[i] >= '0' && canRBuf.aucData[i] <= '9') {
			canRBuf.aucData[i] = canRBuf.aucData[i] - '0';
		} else if(canRBuf.aucData[i] >= 'a' && canRBuf.aucData[i] <= 'f') {
			canRBuf.aucData[i] = canRBuf.aucData[i] - 'a' + 10;
		} else if(canRBuf.aucData[i] >= 'A' && canRBuf.aucData[i] <= 'F') {
			canRBuf.aucData[i] = canRBuf.aucData[i] - 'A' + 10;
		} else {
			//illegal bluetooth name
		}
	}
	uint32_t uiID = canRBuf.aucData[1] * 0x1000000;
	uiID += canRBuf.aucData[2] * 0x100000;
	uiID += canRBuf.aucData[3] * 0x10000;
	uiID += canRBuf.aucData[4] * 0x1000;
	uiID += canRBuf.aucData[5] * 0x100;
	uiID += canRBuf.aucData[6] * 0x10;
	uiID += canRBuf.aucData[7];
	static uint8_t s_uci = 0;
	for(uint8_t i = 0; i < 4; i++) {

	}
	uint8_t ucSelfId = 0;
	uint8_t ucDevNum = 1;
	
//			if(g_auiIEMI_IDs[i] == 0) {
//				break;
//			}	else {
//				if(g_stPrl.uiSelfCode > g_auiIEMI_IDs[i]) {
//					ucSelfId++;
//				}
		
	


}
