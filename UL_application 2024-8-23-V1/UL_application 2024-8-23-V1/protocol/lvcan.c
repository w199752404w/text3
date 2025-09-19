#include "lvcan.h"
#include "local.h"
#include "bsp_sh367303.h"

#include <string.h>

bool g_bcanSendFlag = 0;


void LVCAN_send_BMSPack(){
		uint8_t aucData[8] = {0};
		uint16_t avg_V = 0;
		avg_V = (g_stLocalPackRVal->astPackReport.afCellU[0]+g_stLocalPackRVal->astPackReport.afCellU[1]+g_stLocalPackRVal->astPackReport.afCellU[2]+g_stLocalPackRVal->astPackReport.afCellU[3])/4;
		aucData[2] = avg_V >> 8;
		aucData[3] = avg_V & 0xFF;
		aucData[4] = (g_stLocalPackRVal->astPackReport.fPackU /10) >> 8;
		aucData[5] = (g_stLocalPackRVal->astPackReport.fPackU /10) & 0xFF;
		aucData[6] = g_stLocalPackRVal->astPackReport.fPackCur >> 8;
		aucData[7] = g_stLocalPackRVal->astPackReport.fPackCur  & 0xFF;

		LVCAN_send(eLVCANBMSINFO,aucData,LVCAN_BLK_SIZE);

}

void LVCAN_send_SOX(){
		uint8_t aucData[8] = {0};
		aucData[0] = g_stLocalPackRVal->astPackReport.fPackSoc & 0xFF;
		aucData[1] = g_stLocalPackRVal->astPackReport.fPackSoh & 0xFF;
		aucData[2] = (g_stLocalPackRVal->astPackReport.fPackRealAH *10) >> 8;
		aucData[3] = (g_stLocalPackRVal->astPackReport.fPackRealAH *10) & 0xFF;
		aucData[4] = (g_stCfg.stLocal.usDesignAH *10)>> 8;
		aucData[5] = (g_stCfg.stLocal.usDesignAH *10)& 0xFF;
		aucData[6] = (g_stLocalPackRVal->astPackReport.fPackRealAH *10) >> 8;
		aucData[7] = (g_stLocalPackRVal->astPackReport.fPackRealAH *10) & 0xFF;

		LVCAN_send(eLVCANSOC,aucData,LVCAN_BLK_SIZE);

}
void LVCAN_send_BMSStat(){
		uint8_t aucData[8] = {0};
		aucData[0] = 0;
		aucData[1] = g_stAfe.uRom.stCode.CHG;
		aucData[2] = g_stAfe.uRom.stCode.DSG;
		aucData[3] = 0;
		aucData[4] = 0;
		aucData[5] = ((g_stLocalPackRVal->astPackReport.ucChgEn - 0x55)>0)?0:1;
		aucData[6] = ((g_stLocalPackRVal->astPackReport.ucDsgEn - 0x55)>0)?0:1;
		aucData[7] = 0;
		LVCAN_send(eLVCANBMSSTAT,aucData,LVCAN_BLK_SIZE);

}
void LVCAN_send_Temp(){
		uint8_t aucData[8] = {0};
		aucData[0] = (g_stLocalPackRVal->astPackReport.fMosT *10) >> 8;
		aucData[1] = (g_stLocalPackRVal->astPackReport.fMosT *10) & 0xFF;
		aucData[2] = (g_stLocalPackRVal->astPackReport.fEnvT *10) >> 8;
		aucData[3] = (g_stLocalPackRVal->astPackReport.fEnvT *10) & 0xFF;

		LVCAN_send(eLVCANT,aucData,LVCAN_BLK_SIZE);

}
void LVCAN_send_CellTM(){
		uint8_t aucData[8] = {0};
		int16_t DiffTemp = 0;
		DiffTemp = g_stLocalPackRVal->astPackReport.fCellTMax - g_stLocalPackRVal->astPackReport.fCellTMin;
		aucData[0] = (g_stLocalPackRVal->astPackReport.fCellTMax *10) >> 8;
		aucData[1] = (g_stLocalPackRVal->astPackReport.fCellTMax *10) & 0xFF;
		aucData[2] = (g_stLocalPackRVal->astPackReport.fCellTMin *10) >> 8;
		aucData[3] = (g_stLocalPackRVal->astPackReport.fCellTMin *10) & 0xFF;
		aucData[4] = g_stLocalPackRVal->astPackReport.ucCellTMaxId;
		aucData[5] = g_stLocalPackRVal->astPackReport.ucCellTMinId;
		aucData[6] = (DiffTemp *10) >> 8;
		aucData[7] = (DiffTemp *10) & 0xFF;

		LVCAN_send(eLVCANCellTM,aucData,LVCAN_BLK_SIZE);

}
void LVCAN_send_CellT(){
		uint8_t aucData[8] = {0};
		aucData[0] =( g_stLocalPackRVal->astPackReport.afCellT[0] *10) >> 8;
		aucData[1] =( g_stLocalPackRVal->astPackReport.afCellT[0] *10) & 0xFF;
		aucData[2] =( g_stLocalPackRVal->astPackReport.afCellT[1] *10) >> 8;
		aucData[3] =( g_stLocalPackRVal->astPackReport.afCellT[1] *10) & 0xFF;
		aucData[4] =( g_stLocalPackRVal->astPackReport.afCellT[2] *10) >> 8;;
		aucData[5] =( g_stLocalPackRVal->astPackReport.afCellT[2] *10) & 0xFF;
		aucData[6] =( g_stLocalPackRVal->astPackReport.afCellT[3] *10) >> 8;;
		aucData[7] =( g_stLocalPackRVal->astPackReport.afCellT[3] *10) & 0xFF;

		LVCAN_send(eLVCANCellT,aucData,LVCAN_BLK_SIZE);

}
void LVCAN_send_CellVM(){
		uint8_t aucData[8] = {0};
		int16_t DiffV = 0;
		DiffV = g_stLocalPackRVal->astPackReport.fCellUMax - g_stLocalPackRVal->astPackReport.fCellUMin;
		aucData[0] = g_stLocalPackRVal->astPackReport.fCellUMax >> 8;
		aucData[1] = g_stLocalPackRVal->astPackReport.fCellUMax & 0xFF;
		aucData[2] = g_stLocalPackRVal->astPackReport.ucCellUMaxId;
		aucData[3] = g_stLocalPackRVal->astPackReport.fCellUMin  >> 8;
		aucData[4] = g_stLocalPackRVal->astPackReport.fCellUMin  & 0xFF;
		aucData[5] = g_stLocalPackRVal->astPackReport.ucCellUMinId;
		aucData[6] = DiffV >> 8;
		aucData[7] = DiffV & 0xFF;

		LVCAN_send(eLVCANVBMS,aucData,LVCAN_BLK_SIZE);

}
void LVCAN_send_CellV(){
		uint8_t aucData[8] = {0};
		aucData[0] = g_stLocalPackRVal->astPackReport.afCellU[0] >> 8;
		aucData[1] = g_stLocalPackRVal->astPackReport.afCellU[0] & 0xFF;
		aucData[2] = g_stLocalPackRVal->astPackReport.afCellU[1] >> 8;
		aucData[3] = g_stLocalPackRVal->astPackReport.afCellU[1] & 0xFF;
		aucData[4] = g_stLocalPackRVal->astPackReport.afCellU[2] >> 8;
		aucData[5] = g_stLocalPackRVal->astPackReport.afCellU[2] & 0xFF;
		aucData[6] = g_stLocalPackRVal->astPackReport.afCellU[3] >> 8;
		aucData[7] = g_stLocalPackRVal->astPackReport.afCellU[3] & 0xFF;

		LVCAN_send(eLVCANCellV1,aucData,LVCAN_BLK_SIZE);

}
void LVCAN_send_Err(){
		uint8_t aucData[8] = {0};
//		aucData[0] += g_stLocalPackRVal->astPackReport.uAlmCode.stAlmCode.bCellOV;
//		aucData[0] += g_stLocalPackRVal->astPackReport.uPtctCode.stPtctCode.bCellOV;
//		aucData[0] += (g_stLocalPackRVal->astPackReport.uAlmCode.stAlmCode.bCellUV << 2);
//		aucData[0] += (g_stLocalPackRVal->astPackReport.uPtctCode.stPtctCode.bCellUV << 2);
//		aucData[0] += (g_stLocalPackRVal->astPackReport.uAlmCode.stAlmCode.bArrayOV << 4);
//		aucData[0] += (g_stLocalPackRVal->astPackReport.uPtctCode.stPtctCode.bArrayOV << 4);
//		aucData[0] += (g_stLocalPackRVal->astPackReport.uAlmCode.stAlmCode.bArrayUV << 6);
//		aucData[0] += (g_stLocalPackRVal->astPackReport.uPtctCode.stPtctCode.bArrayUV << 6);
//		
//		aucData[1] += g_stLocalPackRVal->astPackReport.uAlmCode.stAlmCode.bChgOCC;
//		aucData[1] += g_stLocalPackRVal->astPackReport.uPtctCode.stPtctCode.bChgOCC;
//		aucData[1] += (g_stLocalPackRVal->astPackReport.uAlmCode.stAlmCode.bDsgODC << 2);
//		aucData[1] += (g_stLocalPackRVal->astPackReport.uPtctCode.stPtctCode.bDsgODC << 2);
//		aucData[1] += (g_stLocalPackRVal->astPackReport.uAlmCode.stAlmCode.bChgOT << 4);
//		aucData[1] += (g_stLocalPackRVal->astPackReport.uPtctCode.stPtctCode.bChgOT << 4);
//		aucData[1] += (g_stLocalPackRVal->astPackReport.uAlmCode.stAlmCode.bDsgOT << 6);
//		aucData[1] += (g_stLocalPackRVal->astPackReport.uPtctCode.stPtctCode.bDsgOT << 6);
//		
//		aucData[2] += g_stLocalPackRVal->astPackReport.uAlmCode.stAlmCode.bChgUT;
//		aucData[2] += g_stLocalPackRVal->astPackReport.uPtctCode.stPtctCode.bChgUT;
//		aucData[2] += (g_stLocalPackRVal->astPackReport.uAlmCode.stAlmCode.bDsgUT << 2);
//		aucData[2] += (g_stLocalPackRVal->astPackReport.uPtctCode.stPtctCode.bDsgUT << 2);
//		aucData[2] += (g_stLocalPackRVal->astPackReport.uAlmCode.stAlmCode.bEnvOT << 4);
//		aucData[2] += (g_stLocalPackRVal->astPackReport.uPtctCode.stPtctCode.bEnvOT << 4);
//		aucData[2] += (g_stLocalPackRVal->astPackReport.uAlmCode.stAlmCode.bEnvUT << 6);
//		aucData[2] += (g_stLocalPackRVal->astPackReport.uPtctCode.stPtctCode.bEnvUT << 6);
//		
//		
//		aucData[3] += g_stLocalPackRVal->astPackReport.uAlmCode.stAlmCode.bEnvOT;
//		aucData[3] += g_stLocalPackRVal->astPackReport.uPtctCode.stPtctCode.bEnvOT;
//		aucData[3] += 0;
//		aucData[3] += 0;
//		aucData[3] += 0;
//		aucData[3] += 0;
//		aucData[3] += 0;
//		aucData[3] += 0;
//		
//		aucData[4] += 0;
//		if(g_stLocalPackRVal->astPackReport.uPtctCode.stPtctCode.bSC == 1){
//			aucData[4] += g_stLocalPackRVal->astPackReport.uPtctCode.stPtctCode.bSC + 1;
//		}else{
//			aucData[2] += g_stLocalPackRVal->astPackReport.uPtctCode.stPtctCode.bSC;
//		}
//		
//		aucData[4] += 0;
//		aucData[4] += (g_stLocalPackRVal->astPackReport.uErrCode.stErrCode.bVoltSensor << 4);
//		aucData[4] += (g_stLocalPackRVal->astPackReport.uErrCode.stErrCode.bTempSensor << 6);
//		
//		aucData[5] = 0;
		
		LVCAN_send(0x13400000,aucData,LVCAN_BLK_SIZE);

}

void LVCAN_prol(void)
{
//		if(g_bcanSendFlag == 1){
//			LVCAN_send_BMSPack();
//			delay_1ms(100);
//			LVCAN_send_SOX();
//			delay_1ms(100);
//			LVCAN_send_BMSStat();
//			delay_1ms(100);
//			LVCAN_send_Temp();
//			delay_1ms(100);
//			LVCAN_send_CellTM();
//			delay_1ms(100);
//			LVCAN_send_CellT();
//			delay_1ms(100);
//			LVCAN_send_CellVM();
//			delay_1ms(100);
//			LVCAN_send_CellV();
//			delay_1ms(100);
//			LVCAN_send_Err();
//			delay_1ms(10);
//		/* 每500毫秒调用一次本函数以下部分 */
//    static uint32_t uiTick = 0;
//			
//		can_receive_message_struct can0_rxmes;  //Receive data structure
//		memset(&can0_rxmes,0,sizeof(can0_rxmes));
//		can_message_receive(CAN0, CAN_FIFO1, &can0_rxmes);
//		CAN_RBUF_S canRBuf;
//		if(can0_rxmes.rx_efid == 0) {
//			canRBuf.uiId = can0_rxmes.rx_sfid;
//		} else {
//			canRBuf.uiId = can0_rxmes.rx_efid;
//		}
//		
//		if(canRBuf.uiId != eLVCANHeart){
//    uiTick++;
//    if(uiTick > 60 ) {
//			g_bcanSendFlag = 0;
//			uiTick = 0;
//		}
//	 }
//   }
//		LVCAN_send_BMSPack();

}



bool LVCAN_recv(CAN_RBUF_S stCanRBuf){
		if(stCanRBuf.uiId == eLVCANHeart){
			LVCAN_send_BMSPack();
			delay_1ms(2);
			uint8_t aucData[8] = {1};
			
			
			CAN0_SendMsg(0x13400000,aucData,8);
			//LVCAN_send_SOX();
//			delay_1ms(50);
//			LVCAN_send_BMSStat();
//			delay_1ms(50);
//			LVCAN_send_Temp();
//			delay_1ms(50);
//			LVCAN_send_CellTM();
//			delay_1ms(50);
//			LVCAN_send_CellT();
//			delay_1ms(50);
//			LVCAN_send_CellVM();
//			delay_1ms(50);
//			LVCAN_send_CellV();
//			delay_1ms(50);
//			LVCAN_send_Err();
			delay_1ms(50);
			return 1;
		}
		return 0;
 }


void LVCAN_send(uint32_t uiId, uint8_t *pucData, uint8_t ucLength){
		CAN0_SendMsg(uiId,pucData,ucLength);
}
