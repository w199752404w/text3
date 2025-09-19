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
* Description    : �ж��Ƿ�Ϊ����
* Input          : None
* Output         : None
* Return         : result, 1����Ϊ���� 0����������
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
* Description    : �ж��Ƿ�Ϊ�ӻ�
* Input          : None
* Output         : None
* Return         : result, 1����Ϊ�ӻ� 0�����Ǵӻ�
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
* Description    : �ж��Ƿ�Ϊ����
* Input          : None
* Output         : None
* Return         : result, 1����Ϊ���� 0�����ǵ���
*******************************************************************************/
bool prl_single() {
	if(0 == g_stPrl.ucSelfId && 1 == g_stCfg.stPrl.ucPrlPackNum) {
		return true;
	} else {
		return false;
	}
}





bool prl_init(void){
    if(SW2_WK_READ == SET){             //�Ƿ�������         �л�������Э������״̬
		g_stLocalStat = eAppStatConsult;
		g_stPrl.ucSelfId = ePrlPackId0;
		g_stPrl.ucDevNum = 1;
		/*����*/
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
    // ����Э��״̬�Ĵ��� 
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
			if(ePrlPackId0 == g_stPrl.ucSelfId) {                                   //����ȷ��
				//FDCAN1_SendMsg(ePyCanPrlCfm, aucData, FDCAN_DLC_BYTES_8);
				//HAL_Delay(10);
				//FDCAN1_SendMsg(ePyCanPrlCfm, aucData, FDCAN_DLC_BYTES_8);
				//HAL_Delay(10);
				//FDCAN1_SendMsg(ePyCanPrlCfm, aucData, FDCAN_DLC_BYTES_8);
				g_stCfg.stPrl.ePrlSelfIdx = ePrlPackId0;
				g_stCfg.stPrl.ucPrlPackNum = g_stPrl.ucDevNum;
				cfg_prl_save();
				//g_stAppArrayRVal.eAppStat = eAppStatRun;
				PRL_DEBUG("����Э��ȷ�Ϲ㲥");
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
				PRL_DEBUG("��������, ����ID=%d", g_stPrl.ucSelfId);
			}
		}
		PRL_RETURN_TRUE;
    }
    
    
    ///* ��ť���߲������� */
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
    case ePyCanPrlReq:                          //����Э�� �ӻ�
        if(ucNode != ePrlPackId0) {
			PRL_DEBUG("����Э�̱���, �ڵ����ƷǷ�, �ڵ�����Ϊ%d", pucData[0]);
			break;
		}
		if(eAppStatConsult == g_stLocalStat && ePrlPackId0 == g_stPrl.ucSelfId) {
			PRL_DEBUG("����Э��-����ģʽ�յ����ⱨ��, ���Ĺ�����%02x", uiFunc);
			break;
		}
		if(eAppStatConsult != g_stLocalStat) {
			PRL_DEBUG("�л�������Э��-�ӻ�ģʽ");
			g_stLocalStat = eAppStatConsult;
			//g_stPrl.ucSelfId = ePrlPackIdNull;
            
            g_stPrl.ucSelfId = ePrlPackIdBoard;
            PRL_DEBUG("�л�������Э��-�ӻ�ģʽ = %d",g_stPrl.ucSelfId);
		}
		if(0 == g_ucReqFlag && SET == CODE_OP_IN_READ){     //gaibain
		    g_stPrl.ucDevNum = pucData[0];
			g_ucReqFlag++;
            
		}
            break;
    case ePyCanPrlGuess:                        //����Э��-����
        if(ePrlPackId0 != g_stPrl.ucSelfId) {
			break;
		}
		if(pucData[0] > ePrlPackId7) {
				PRL_DEBUG("�Ƿ��Ĳ�������ID, ����ID=%d", pucData[0]);
			break;
		}
		if(g_stPrl.ucDevNum == pucData[0]) {
			PRL_DEBUG("����Э��-����ģʽ�յ���ȷ��Э�̲���");
			g_stPrl.ucDevNum++;
		} else {
			PRL_DEBUG("����Ĳ�������, ��Э�̽ڵ���=%d, ����ID=%d", g_stPrl.ucDevNum, pucData[0]);
			break;
		}
            break;
    case ePyCanPrlCfm:                          //����Э��-ȷ��
        if(ePrlPackId0 == g_stPrl.ucSelfId) {
			PRL_DEBUG("����Э��ȷ��-����ģʽ�յ����ⱨ��, ���Ĺ�����%02x", uiFunc);
			break;
		}
		if(g_stPrl.ucSelfId > ePrlPackId7) {
			PRL_DEBUG("����δ���Э��ID, ��Э�������");
			g_stLocalStat = eLocalStatRun;
			g_stPrl.ucSelfId = g_stCfg.stPrl.ePrlSelfIdx;
			g_stPrl.ucDevNum = g_stCfg.stPrl.ucPrlPackNum;
			break;
		}
		g_stCfg.stPrl.ePrlSelfIdx = (PRL_PACK_E)g_stPrl.ucSelfId;
		g_stCfg.stPrl.ucPrlPackNum = pucData[0];
        PRL_DEBUG("����Э��ȷ��wg_stCfg.stPrl.ePrlSelfIdx = %d",g_stCfg.stPrl.ePrlSelfIdx);
        PRL_DEBUG("����Э��ȷ��g_stCfg.stPrl.ucPrlPackNum = %d",g_stCfg.stPrl.ucPrlPackNum);
		if(0 == g_stCfg.stPrl.ucPrlPackNum || g_stCfg.stPrl.ucPrlPackNum > CFG_MAX_PRL_NUM) {
			PRL_DEBUG("����Э��ȷ�ϱ��ĵĲ����ڵ�����Ƿ�, \
				��������: %02x %02x %02x %02x %02x %02x %02x %02x", \
				pucData[0], pucData[1], pucData[2], pucData[3], \
				pucData[4], pucData[5], pucData[6], pucData[7]);
			break;
		}
		cfg_prl_save();
		g_stLocalStat = eLocalStatRun;
        break;
    }



}





























