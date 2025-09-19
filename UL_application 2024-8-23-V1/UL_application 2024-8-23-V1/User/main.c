#include "gd32f10x.h"
#include "bsp.h"
#include "bsp_gpio.h"
#include "bsp_wdg.h"
#include "bsp_can.h"
#include "bsp_usart.h"
#include "bsp_tim.h"
#include "bsp_sh367303.h"
#include "bsp_exti.h"
#include "systick.h"
#include "verify.h"
#include "history.h"
#include "local.h"
#include "lvcan.h"
#include "modbus_rtu.h"
#include "parallel.h"
#include "config.h"
#include "protocol.h"
#include "string.h"
#include "main.h"

int main(void) {
#ifdef USER_DEBUG
	/*Turn on global interrupt*/
	__set_PRIMASK(0);
	/*Set Interrupt Vector Table*/
	SCB->VTOR = FLASH_BASE | 0x4000;
#endif
 	BSP_Init();
	//BSP_WK_Detection();
	//g_stLocalArrayRVal.eLocalStat = eLocalStatInit;
    g_stLocalStat = eLocalStatInit;
	/* app init */
	for(uint8_t i=0;i<10;i++) {
		if(!cfg_init()) {
			g_stLocalStat = eLocalStatInitFail;
		} else {
 			break;
		}
	}
//	UART_Init(g_stCfg.stCom.stUart.uiBaud, g_stCfg.stCom.stUart.uiWordLen, g_stCfg.stCom.stUart.uiStopBit, g_stCfg.stCom.stUart.uiParity );//Initialize Bluetooth serial port
	delay_1ms(500);
	afe_init();
#ifdef USER_DEBUG
// verify();
#endif
	if(!his_init()) {
		g_stLocalStat = eLocalStatInitFail;
		while(1);
	}
	if(!local_init()) {
		g_stLocalStat = eLocalStatInitFail;
		while(1);
	}
	//CAN0_Init(g_stCfg.stCom.auiCanBaud);	/* Initialize CAN 250K baud rate */
	CAN0_Init(250);
	if(!prl_init()) {
		g_stLocalStat = eLocalStatInitFail;
		while(1);
	}
//eco_init();
	/* wake record*/
	{
		HIS_DATA_S stData = {0};
		local_proc();
		LOCAL_PACK_RVAL_S* pstPackRVal = &g_stLocalPackRVal[g_stPrl.ucSelfId];
		if(pstPackRVal->astPackReport.fPackSoc <= 1 && pstPackRVal->astPackReport.uErrCode.stErrCode.bVoltSensor == 0) {
			//local_standby_leftAH_cali();
		}
		//eco_proc();
		his_data_read(0, &stData);

		his_data_write();
	}
	/* main loop */
	g_stLocalStat = eLocalStatRun;

	while(1) {
		/* CAN port process */
		while(g_stCanRCA.usCIdx != g_stCanRCA.usRIdx) {

			LVCAN_recv(g_stCanRCA.astRBuf[g_stCanRCA.usCIdx]);
			g_stCanRCA.usCIdx++;
			if(g_stCanRCA.usCIdx >= FDCAN_MAX_DATA_CACHE) {
				g_stCanRCA.usCIdx = 0;
			}
		}
		/* Serial port process */
		if(USART2_RxFlag == 1) {
			dma_channel_disable(DMA0, DMA_CH2);
			dma_flag_clear(DMA0, DMA_CH2, DMA_INTF_FTFIF);  /* Clear the DMA flag */
			//g_stUsart.usRLen = sizeof(g_stUsart.aucRBuf) - dma_transfer_number_get(DMA0, DMA_CH2);
			if(g_stLocalStat == eLocalStatRun || g_stLocalStat == eLocalStatUartUpgrade || g_stLocalStat == eLocalStatUartBootMode) {
				//eco_uart_recv_proc(g_stUsart.aucRBuf, g_stUsart.usRLen);
			}
			//g_stUsart.usRLen = 0;
			USART2_RxFlag = 0; 
			dma_transfer_number_config(DMA0, DMA_CH2, sizeof(g_stUsart.aucRBuf)); //	* Reassign the count value */
			dma_channel_enable(DMA0, DMA_CH2);
		}
		/* app process */
//		if(g_stLocalArrayRVal.eLocalStat == eLocalStatRun) {
			if(g_ucTm != 0) {
				static uint16_t s_usTestTick = 0;
				if(s_usTestTick > 100) {
					s_usTestTick = 0;
				} else {
					s_usTestTick++;
				}
				g_ucTm = 0;
				afe_proc();
				local_proc();
				//eco_proc();
				dcc_proc();
				his_proc();
				prl_proc();
				LVCAN_prol();
			}
//		}
	}
}
