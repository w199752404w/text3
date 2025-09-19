#include "gd32f10x.h"
#include "bsp.h"
#include "bsp_gpio.h"
#include "bsp_wdg.h"
#include "bsp_can.h"
#include "bsp_usart.h"
#include "bsp_tim.h"
#include "bsp_sh367309.h"
#include "bsp_exti.h"
#include "systick.h"
#include "verify.h"
#include "history.h"
#include "bsp_gd25q32.h"
#include "local.h"
#include "eco.h"
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
	MX_GPIO_Init();                                           //Initialize GPIO port
 	BSP_Init();
	BSP_WK_Detection();
	g_stLocalArrayRVal.eLocalStat = eLocalStatInit;
	/* app init */
	for(uint8_t i=0;i<10;i++) {
		if(!cfg_init()) {
			g_stLocalArrayRVal.eLocalStat = eLocalStatInitFail;
		} else {
 			break;
		}
	}
//	CAN0_Init(g_stCfg.stCom.uiCanBaud);
//	UART_Init(g_stCfg.stCom.stUart.uiBaud, g_stCfg.stCom.stUart.uiWordLen, g_stCfg.stCom.stUart.uiStopBit, g_stCfg.stCom.stUart.uiParity );//Initialize Bluetooth serial port
	delay_1ms(500);
	afe_init();
#ifdef USER_DEBUG
// verify();
#endif
	if(!his_init()) {
		g_stLocalArrayRVal.eLocalStat = eLocalStatInitFail;
		while(1);
	}
	if(!local_init()) {
		g_stLocalArrayRVal.eLocalStat = eLocalStatInitFail;
		while(1);
	}
//	CAN0_Init(g_stCfg.stCom.uiCanBaud);	/* Initialize CAN 250K baud rate */
	if(!prl_init()) {
		g_stLocalArrayRVal.eLocalStat = eLocalStatInitFail;
		while(1);
	}
	eco_init();
	/* wake record*/
	{
		HIS_DATA_S stData = {0};
		HIS_DATA_S stData1 = {0};
		local_proc();
		LOCAL_PACK_RVAL_S* pstPackRVal = g_stLocalArrayRVal.astPackRVal + g_stPrl.ucSelfId;
		if(pstPackRVal->fPackSoc <= 1 && pstPackRVal->uErrCode.stErrCode.bVoltSensor == 0) {
			local_standby_leftAH_cali();
		}
		eco_proc();
		his_data_read(0, &stData);
		his_data_read(1, &stData1);
		if(stData1.state == eBStatDownload) {
			cfg_set_default(); // 2024.7.12
		}
		if(g_stCfg.stLocal.ucSVerAgree == 'A') {//2024.12.14 dgx
		} else if(g_stCfg.stLocal.ucSVerAgree == 'B') {
		} else if(g_stCfg.stLocal.ucSVerAgree == 'C') {
		} else if(g_stCfg.stLocal.ucSVerAgree == 'D') {
		} else if(g_stCfg.stLocal.ucSVerAgree == 'E') {
		} else {
			cfg_set_default(); // 2024.12.19 dgx
//			g_stCfg.stCom.ucCanPtcType = 1;
//			g_stCfg.stLocal.ucSVerAgree = 'A';
//			cfg_save();
		}
		if(stData.state == eBStatSleep_01 || stData.state == eBStatSleep_02 || stData.state == eBStatReset_51 || stData.state == eBStatReset_52){
			g_eBaseStat = eBStatWake;
		} else {
		  g_eBaseStat = eBStatReset_53;
		}
		his_data_write();
	}
	CAN0_Init(g_stCfg.stCom.uiCanBaud);
	/* main loop */
	g_stLocalArrayRVal.eLocalStat = eLocalStatRun;
	while(1) {
	  WDG_DONE_H;
		delay_1ms(10);
	  WDG_DONE_L;
		/* CAN port process */
		while(g_stCanRCA.usCIdx != g_stCanRCA.usRIdx) {
//			if(g_stCanRCA.astRBuf[g_stCanRCA.usCIdx].uiId == 0x23F016F) {
//				eco_can_recv_proc(g_stCanRCA.astRBuf[g_stCanRCA.usCIdx]);
//			}
			if(g_stLocalArrayRVal.eLocalStat == eLocalStatRun || g_stLocalArrayRVal.eLocalStat == eLocalStatCanUpgrade) {
				eco_can_recv_proc(g_stCanRCA.astRBuf[g_stCanRCA.usCIdx]);
			}
			g_stCanRCA.usCIdx++;
			if(g_stCanRCA.usCIdx >= FDCAN_MAX_DATA_CACHE) {
				g_stCanRCA.usCIdx = 0;
			}
		}
		/* Serial port process */
		if(USART2_RxFlag == 1) {
			dma_channel_disable(DMA0, DMA_CH2);
			dma_flag_clear(DMA0, DMA_CH2, DMA_INTF_FTFIF);  /* Clear the DMA flag */
			g_stUsart.usRLen = sizeof(g_stUsart.aucRBuf) - dma_transfer_number_get(DMA0, DMA_CH2);
			if(g_stLocalArrayRVal.eLocalStat == eLocalStatRun || g_stLocalArrayRVal.eLocalStat == eLocalStatUartUpgrade || g_stLocalArrayRVal.eLocalStat == eLocalStatUartBootMode) {
				eco_uart_recv_proc(g_stUsart.aucRBuf, g_stUsart.usRLen);
			}
			g_stUsart.usRLen = 0;
			USART2_RxFlag = 0; 
			dma_transfer_number_config(DMA0, DMA_CH2, sizeof(g_stUsart.aucRBuf)); //	* Reassign the count value */
			dma_channel_enable(DMA0, DMA_CH2);
		}
		/* app process */
		if(g_stLocalArrayRVal.eLocalStat == eLocalStatRun) {
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
				eco_proc();
				his_proc();
				prl_proc();
				ResetAFE1();
			}
		}
	}
}
