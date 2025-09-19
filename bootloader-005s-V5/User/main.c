#include "gd32f10x.h"
#include "bsp_gpio.h"
#include "systick.h"
#include "flash.h"
#include "settings.h"
#include "bsp_usart.h"
#include "bsp_exti.h"
#include "iap.h"
#include "bsp_gd25q32.h"
#include "bsp_wdg.h"
#include "stdio.h"
#include "string.h"

void set_default_upgradeparam(void);

int main()
{
	rcu_ahb_clock_config(RCU_AHB_CKSYS_DIV1);                 //Set the AHB main frequency to 1 division, that is, the main frequency is 108M
	systick_config();//1ms systick
	MX_GPIO_Init();
//USART_Init(eBLEPort, 115200, USART_WL_8BIT, USART_STB_1BIT, USART_PM_NONE );//Initialize Bluetooth serial port
  FLASH_Init();      // FLASH解锁
	for(uint16_t i=0;i<5000;i++){
		if(BSP_W25Qx_Init()) {
			}else{
				break;
			}
	}
	delay_1ms(20);
//BSP_W25Qx_Erase_Chip(); 
  read_upgradeparam_save_flash();         //读取片外flash的数据到定义的结构体中
	BSP_DEBUG("This is bootloader\r\n");
	BSP_DEBUG("g_upgradeparam_data.uiAddr=0x%x\r\n",g_upgradeparam_data.uiAddr); 
	BSP_DEBUG("g_upgradeparam_data.uiLen=0x%x\r\n",g_upgradeparam_data.uiLen);
	BSP_DEBUG("g_upgradeparam_data.ucUpdate=0x%x\r\n",g_upgradeparam_data.ucUpdate);
 //set_default_upgradeparam();
	if(g_upgradeparam_data.ucUpdate == 0x01){
	  update_logic();
	}else{
		set_default_upgradeparam();
		run();
	}
	 
	
	run();
  while (1)
  {
    set_default_upgradeparam();
		BSP_DEBUG("Bootloader error.\r\n");
		delay_1ms(1000);

  }
	
}

void set_default_upgradeparam(void) 
{
 memset(&g_upgradeparam_data,0,sizeof(g_upgradeparam_data)); 
// strcpy((char*)(g_upgradeparam_data.upgrade_system_addr),"106.14.5.188");
// strcpy((char*)(g_upgradeparam_data.mideware_version),"1.0.0.1000"); 
// g_upgradeparam_data.upgrade_system_port = 14390; 
// g_upgradeparam_data.update_result = 1;
// g_upgradeparam_data.update_flag = 0;  
// g_upgradeparam_data.write_flag = 1;
// write_upgradeparam_save_flash();
}



