
#ifndef __IAP_H__
#define __IAP_H__
#include <stdint.h>

#define FLASH_BOOT_ADDR				0x08000000	//启动区 16KB
#define FLASH_APP_ADDR				0x08004000	//app区 
#define FLASH_APP_BACK_ADDR		0x08014000	//app区

#define FLASH_UPGRADEPARAMS_ADDR   0      //片外flash
//#define FLASH_UPGRADEPARAMS_ADDR  (0x08000000 + 124*1024)  //éy??2?êy?? 128KB
#define FLASH_PARAMS_ADDR         (0x08000000 + 256*1024)  //éè±?2?êy?? 256KB



#define DEVICE_ID_LEN	32	//设备ID长度
#define IMEI_NUM_LEN	15 //设备imei号	
#define VERSION_LEN	32	  //消息版本长度
#define PRODUCT_NAME	10	//产品名称
#define PRODUCT_TYPE	10	//产品型号 缺少产品id

#define FILE_NAME_LEN	64 //程序名

#define MAX_PACKET_LEN	512 //包最大长度

#define DEVICE_COMMAND_HEAD_FLAG		0xD5D5D5D5//包头

#define EEPROM_FILED_LENGTH 32		//段长度

#define UART_FRAME_LEN 24 //串口通信帧长度

#define MAX_IPADDR_LEN	16  //IP字符串最大长度
#define IMEI_LEN  		  16  //imei号长度

#pragma pack(1)
//升级相关参数结构体
typedef struct _UPGRADE_PARAM_DATA
{
	uint32_t uiAddr;	/* 固件存放地址, 0-片内, 其他-片外地址 */
	uint32_t uiLen;		/* 固件字节数 */
	uint8_t ucUpdate;	/* 非0: 下次重启时需要通过Flash升级, 升级后清0 */
  uint8_t aucReserve[247];	/* 保证整个结构体大小为256字节 */
}UPGRADE_PARAM_DATA;

//	unsigned char  write_flag; //是否写过flash标志
//	unsigned char mideware_version[EEPROM_FILED_LENGTH]; //固件版本号，占32个字节
//	unsigned char upgrade_system_addr[MAX_IPADDR_LEN]; //升级服务器域名，占16个字节
//	unsigned short upgrade_system_port; //升级服务器端口
//	unsigned char  update_flag; //升级标志
//	unsigned char  update_result; //升级结果 0--未升级或升级失败 1--升级成功,可以跳转
//	unsigned int   total_packetcount; //升级固件总包数
//	unsigned int   cur_file_packnum; //当前下载的包的个数
//	uint8_t aucReserve[247];	/* 保证整个结构体大小为256字节 */
typedef struct {
	/* 0. OTA配置, 0.25kB */
	UPGRADE_PARAM_DATA stOta;
	/* 1. start after reset, 0.25kB */
	uint16_t usGoRun;		/* 0- sleep power off, 1- reset power off, 2- Switch power off, 3- going to run */
	uint8_t ucReserve[254];
} CFG_S;
extern CFG_S g_stCfg;
extern UPGRADE_PARAM_DATA g_upgradeparam_data;

#pragma pack()//回复keil原来的数据对齐方式



void write_upgradeparam_save_flash(void);
void read_upgradeparam_save_flash(void);
void jump_to_app(uint32_t app_addr);
void run(void);
void update_logic(void);
void App_to_Flash(void);

#endif

