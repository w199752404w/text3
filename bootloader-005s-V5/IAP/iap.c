#include "iap.h"
#include "bsp_gpio.h"
#include "string.h"
#include "stdio.h"
#include "flash.h"
#include "bsp_usart.h"
#include "bsp_gd25q32.h"
#include "bsp_wdg.h"
#include "systick.h"
extern uint64_t need_updata;

typedef  void (*pFunction)(void);


//设备参数信息
UPGRADE_PARAM_DATA g_upgradeparam_data;
CFG_S g_stCfg; 

//目前接收的升级包数
int g_curfilepacknum = 0;
uint8_t databuf_8[256];
uint8_t data_vf_8[256];



void write_upgradeparam_save_flash(void)
{	
	MEM_FlashWrite(FLASH_UPGRADEPARAMS_ADDR, (uint8_t*)&g_upgradeparam_data, sizeof(UPGRADE_PARAM_DATA));
}

void read_upgradeparam_save_flash(void)
{
	MEM_FlashRead(FLASH_UPGRADEPARAMS_ADDR, (uint8_t*)&g_upgradeparam_data, sizeof(UPGRADE_PARAM_DATA));
	MEM_FlashRead(FLASH_UPGRADEPARAMS_ADDR, (uint8_t*)&g_stCfg, sizeof(CFG_S));
}


/*
退出更新模式 开始根据情况执行相应程序
读取flash中的 标志位决定是执行 app 还是执行 iap程序的业务逻辑区

*/


/*
升级主程序逻辑
1 尝试连接升级服务器
2 

*/
void update_logic()
{
	uint32_t i;
	uint8_t data_8[4];
	unsigned int readcount = 0;                                    //读写的页数

	readcount = g_upgradeparam_data.uiLen / 256 + 15;               // 片外flash数据的页数，256个字节是一页
//	readcount = 0x1388;
	
	BSP_DEBUG("g_upgradeparam_data.total_packetcount:%d\r\n",readcount);

	MEM_FlashRead(g_upgradeparam_data.uiAddr,(uint8_t*)databuf_8,4);   //读取片外flash数据 
	if(databuf_8[0] != 0xFF && databuf_8[1] != 0xFF &&databuf_8[2] != 0xFF && databuf_8[3] != 0xFF){
	
			/* 擦除APP存储区 */
			//gd32_flash_erase(FLASH_APP_ADDR, 0x08040000); 
			FLASH_If_Erase(FLASH_APP_ADDR);
			
			/* 检测擦除操作是否成功 */
			gd32_flash_read(FLASH_APP_ADDR ,(uint8_t*)data_8,4); 	
			if(data_8[0] == 0xff && data_8[1] == 0xff && data_8[2] == 0xff && data_8[3] == 0xff){
					BSP_DEBUG("Erase    OK !!! \r\n");
			}else{
					BSP_DEBUG("Erase Error !!! \r\n");
					BSP_DEBUG("%0x %0x %0x %0x",data_8[0],data_8[1],data_8[2],data_8[3]);
			}
			
		 //	copy_download_to_app();
			
			/* 拷贝片外FLASH到APP存储区 */
			for(i = 0;i < readcount;i++)
			{
				memset(databuf_8,0,sizeof(databuf_8));
				MEM_FlashRead(g_upgradeparam_data.uiAddr + i*256,(uint8_t*)databuf_8,256);   //读取片外flash数据 
				delay_1ms(1);	
				gd32_flash_write(APP_START_ADDRESS + i*256,(uint8_t*)databuf_8,256);
		//	gd32_wdgt_feed_dog(EWdgType_fwdg);	//feed dog
				WDG_DONE_H;
				delay_1ms(10);
		    WDG_DONE_L; 
		//  FLASH_Program(FLASH_APP_ADDR + i*256, 64,(uint32_t*)databuf_8);	
		//  FLASH_Write(FLASH_APP_ADDR + i*256, (uint16_t*)databuf_8 ,256); 
		//	  gd32_flash_read(APP_START_ADDRESS + i*256,(uint8_t*)data_vf_8,256);
		//		for(int j=0;j<256;j++)
		//		{
		//			printf("%02x ",data_vf_8[j]);
		//		}
		//		while(0 != memcmp(databuf_8, data_vf_8, 256)){
		//			gd32_flash_write(APP_START_ADDRESS + i*256,(uint8_t*)databuf_8,256);
		//			gd32_flash_read(APP_START_ADDRESS + i*256,(uint8_t*)data_vf_8,256);
		//			for(int j=0;j<256;j++)
		//			{
		//				printf("gab:%02x ",data_vf_8[j]);
		//			}
			}	 
	}

	//写完之后修改flash标志，表示不用再升级了	
//	g_upgradeparam_data.uiAddr = 0; 
//  g_upgradeparam_data.uiLen = 0;
//  g_upgradeparam_data.ucUpdate = 0;
//  write_upgradeparam_save_flash();
// delay_1ms(1000);
// memset(&g_upgradeparam_data,0,sizeof(g_upgradeparam_data)); 
 write_upgradeparam_save_flash();
 delay_1ms(1000);
// read_upgradeparam_save_flash();
//跳转到app区	
	run();
	
}

void jump_to_app(uint32_t app_addr)
{
	pFunction JumpToApplication;                               //定义跳转函数指针
  __IO uint32_t JumpAddress;                                 //应用程序中断向量表的地址
	__disable_irq();                                           //关闭总中断
	__set_PRIMASK(1);                                          //关闭全局中断	
	if(((*(uint32_t*)app_addr)&0x2FFE0000)==0x20000000)        //检查栈顶地址是否合法
	{
			JumpAddress = *(__IO uint32_t*) (app_addr + 4);
			JumpToApplication = (pFunction) JumpAddress;
			/* Initialize user application's Stack Pointer */
			__set_MSP(*(__IO uint32_t*) app_addr);            //设置主堆栈指针 
 			JumpToApplication();
		  BSP_DEBUG("Jump error.\r\n");
	}
	else
	{
		BSP_DEBUG("No Jump \r\n");
	}
	
}

void run(void)
{		
    BSP_DEBUG ("\r\nJump app!!!!!!!!!!\r\n");
		jump_to_app(FLASH_APP_ADDR);
}





