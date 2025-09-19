#include "parallel.h"
#include "bsp_gpio.h"
#include "bsp_exti.h"
#include "local.h"
#include "bsp_can.h"
#include <stdio.h> 



PRL_S g_stPrl = {0};
uint16_t g_ausPrlComTick[CFG_MAX_PRL_NUM];
uint8_t  g_ucReqFlag = 0;


/*******************************************************************************
* Function Name  : prl_host
* Description    : 判断是否为主机
* Input          : None
* Output         : None
* Return         : result, 1本机为主机 0本机非主机
*******************************************************************************/
bool prl_host() {
	if(0 == g_stPrl.ucSelfId && g_stCfg.stPrl.ucPrlPackNum > 1) {
		return true;
	} else {
		return false;
	}
}

/*******************************************************************************
* Function Name  : prl_client
* Description    : 判断是否为从机
* Input          : None
* Output         : None
* Return         : result, 1本机为从机 0本机非从机
*******************************************************************************/
bool prl_client() {
	if(g_stPrl.ucSelfId > 0 && g_stPrl.ucSelfId <= ePrlPackId7) {
		return true;
	} else {
		return false;
	}
}

/*******************************************************************************
* Function Name  : prl_single
* Description    : 判断是否为单机
* Input          : None
* Output         : None
* Return         : result, 1本机为单机 0本机非单机
*******************************************************************************/
bool prl_single() {
	if(0 == g_stPrl.ucSelfId && 1 == g_stCfg.stPrl.ucPrlPackNum) {
		return true;
	} else {
		return false;
	}
}





bool prl_init(void){
    if(SW2_WK_READ == SET){             //是否是主机         切换至并机协商主机状态
		g_stLocalStat = eAppStatConsult;
		g_stPrl.ucSelfId = ePrlPackId0;
		g_stPrl.ucDevNum = 1;
		/*新增*/
		CODE_OUT_H;
		PRL_RETURN_TRUE;
    }
    
    if(prl_host()) {
		g_stPrl.ucDevNum = g_stCfg.stPrl.ucPrlPackNum;
		g_stPrl.ucSelfId = ePrlPackId0;
	} else if(prl_client()) {
		g_stPrl.ucDevNum = g_stCfg.stPrl.ucPrlPackNum;
		g_stPrl.ucSelfId = g_stCfg.stPrl.ePrlSelfIdx;
	} else if(prl_single()) {
		g_stPrl.ucDevNum = 1;
		g_stPrl.ucSelfId = 0;
	} else {
		g_stPrl.ucDevNum = 1;
		g_stPrl.ucSelfId = ePrlPackIdNull;
		PRL_RETURN_FALSE;
	}
    
//    for(uint8_t i=0;i<CFG_MAX_PRL_NUM;i++) {
//		g_ausPrlComTick[i] = PYCAN_PRL_TIMEOUT;
//	}
	
    
    PRL_RETURN_TRUE;
}

bool prl_proc(void){
    
    #define BTN_REQ_TICK	10
    // 并机协商状态的处理 
    static uint8_t ucReqTick = 0;	
    if(eAppStatConsult == g_stLocalStat) {
        uint8_t aucData[8] = {0};
		aucData[0] = g_stPrl.ucDevNum;
		ucReqTick++;
		if(ucReqTick > 250){
			ucReqTick=0;
		}
        if(ucReqTick > BTN_REQ_TICK) {
			ucReqTick=0;
			if(ePrlPackId0 == g_stPrl.ucSelfId) {                                   //并机确认
				//FDCAN1_SendMsg(ePyCanPrlCfm, aucData, FDCAN_DLC_BYTES_8);
				//HAL_Delay(10);
				//FDCAN1_SendMsg(ePyCanPrlCfm, aucData, FDCAN_DLC_BYTES_8);
				//HAL_Delay(10);
				//FDCAN1_SendMsg(ePyCanPrlCfm, aucData, FDCAN_DLC_BYTES_8);
				g_stCfg.stPrl.ePrlSelfIdx = ePrlPackId0;
				g_stCfg.stPrl.ucPrlPackNum = g_stPrl.ucDevNum;
				cfg_prl_save();
				//g_stAppArrayRVal.eAppStat = eAppStatRun;
				PRL_DEBUG("并机协商确认广播");
			}
		} else {
			if(ePrlPackId0 == g_stPrl.ucSelfId) {
				CAN0_SendMsg(ePyCanPrlReq, aucData, 8);
			}
			if(1 == g_ucReqFlag && SET == CODE_OP_IN_READ ){
				g_stPrl.ucSelfId = g_stPrl.ucDevNum;
				g_ucReqFlag++;
				CODE_OUT_H;
				CAN0_SendMsg(ePyCanPrlGuess, aucData, 8);
				PRL_DEBUG("并机猜想, 猜想ID=%d", g_stPrl.ucSelfId);
			}
		}
		PRL_RETURN_TRUE;
    }
    
    
    ///* 按钮休眠操作处理 */
	static uint8_t ucSWTick = 0;
    if(SW_WK_READ != 0){
        if(ucSWTick * g_stCfg.stLocal.usCyclePeriod < 500) {
            ucSWTick++;
        }
        if(ucSWTick * g_stCfg.stLocal.usCyclePeriod >= 500) {
            //SW_EN_LED_L;
            cfg_save();
			System_Sleep();
        }   
    }
    
    PRL_RETURN_TRUE;

}

void prl_recv_proc(uint32_t uiId, uint8_t* pucData){
    uint8_t ucNode = uiId >> 12 & 0x0F;
    uint32_t uiFunc =  uiId & 0xFFFF0FFF;  
    switch(uiFunc){
    case ePyCanPrlReq:                          //并机协商 从机
        if(ucNode != ePrlPackId0) {
			PRL_DEBUG("并机协商报文, 节点名称非法, 节点名称为%d", pucData[0]);
			break;
		}
		if(eAppStatConsult == g_stLocalStat && ePrlPackId0 == g_stPrl.ucSelfId) {
			PRL_DEBUG("并机协商-主机模式收到意外报文, 报文功能码%02x", uiFunc);
			break;
		}
		if(eAppStatConsult != g_stLocalStat) {
			PRL_DEBUG("切换至并机协商-从机模式");
			g_stLocalStat = eAppStatConsult;
			//g_stPrl.ucSelfId = ePrlPackIdNull;
            
            g_stPrl.ucSelfId = ePrlPackIdBoard;
            PRL_DEBUG("切换至并机协商-从机模式 = %d",g_stPrl.ucSelfId);
		}
		if(0 == g_ucReqFlag && SET == CODE_OP_IN_READ){     //gaibain
		    g_stPrl.ucDevNum = pucData[0];
			g_ucReqFlag++;
            
		}
            break;
    case ePyCanPrlGuess:                        //并机协商-猜想
        if(ePrlPackId0 != g_stPrl.ucSelfId) {
			break;
		}
		if(pucData[0] > ePrlPackId7) {
				PRL_DEBUG("非法的并机猜想ID, 猜想ID=%d", pucData[0]);
			break;
		}
		if(g_stPrl.ucDevNum == pucData[0]) {
			PRL_DEBUG("并机协商-主机模式收到正确的协商猜想");
			g_stPrl.ucDevNum++;
		} else {
			PRL_DEBUG("错误的并机猜想, 已协商节点数=%d, 猜想ID=%d", g_stPrl.ucDevNum, pucData[0]);
			break;
		}
            break;
    case ePyCanPrlCfm:                          //并机协商-确认
        if(ePrlPackId0 == g_stPrl.ucSelfId) {
			PRL_DEBUG("并机协商确认-主机模式收到意外报文, 报文功能码%02x", uiFunc);
			break;
		}
		if(g_stPrl.ucSelfId > ePrlPackId7) {
			PRL_DEBUG("本机未获得协商ID, 但协商已完成");
			g_stLocalStat = eLocalStatRun;
			g_stPrl.ucSelfId = g_stCfg.stPrl.ePrlSelfIdx;
			g_stPrl.ucDevNum = g_stCfg.stPrl.ucPrlPackNum;
			break;
		}
		g_stCfg.stPrl.ePrlSelfIdx = (PRL_PACK_E)g_stPrl.ucSelfId;
		g_stCfg.stPrl.ucPrlPackNum = pucData[0];
        PRL_DEBUG("并机协商确认wg_stCfg.stPrl.ePrlSelfIdx = %d",g_stCfg.stPrl.ePrlSelfIdx);
        PRL_DEBUG("并机协商确认g_stCfg.stPrl.ucPrlPackNum = %d",g_stCfg.stPrl.ucPrlPackNum);
		if(0 == g_stCfg.stPrl.ucPrlPackNum || g_stCfg.stPrl.ucPrlPackNum > CFG_MAX_PRL_NUM) {
			PRL_DEBUG("并机协商确认报文的并机节点个数非法, \
				报文内容: %02x %02x %02x %02x %02x %02x %02x %02x", \
				pucData[0], pucData[1], pucData[2], pucData[3], \
				pucData[4], pucData[5], pucData[6], pucData[7]);
			break;
		}
		cfg_prl_save();
		g_stLocalStat = eLocalStatRun;
        break;
    }



}





























