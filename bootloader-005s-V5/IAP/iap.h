
#ifndef __IAP_H__
#define __IAP_H__
#include <stdint.h>

#define FLASH_BOOT_ADDR				0x08000000	//������ 16KB
#define FLASH_APP_ADDR				0x08004000	//app�� 
#define FLASH_APP_BACK_ADDR		0x08014000	//app��

#define FLASH_UPGRADEPARAMS_ADDR   0      //Ƭ��flash
//#define FLASH_UPGRADEPARAMS_ADDR  (0x08000000 + 124*1024)  //��y??2?��y?? 128KB
#define FLASH_PARAMS_ADDR         (0x08000000 + 256*1024)  //������?2?��y?? 256KB



#define DEVICE_ID_LEN	32	//�豸ID����
#define IMEI_NUM_LEN	15 //�豸imei��	
#define VERSION_LEN	32	  //��Ϣ�汾����
#define PRODUCT_NAME	10	//��Ʒ����
#define PRODUCT_TYPE	10	//��Ʒ�ͺ� ȱ�ٲ�Ʒid

#define FILE_NAME_LEN	64 //������

#define MAX_PACKET_LEN	512 //����󳤶�

#define DEVICE_COMMAND_HEAD_FLAG		0xD5D5D5D5//��ͷ

#define EEPROM_FILED_LENGTH 32		//�γ���

#define UART_FRAME_LEN 24 //����ͨ��֡����

#define MAX_IPADDR_LEN	16  //IP�ַ�����󳤶�
#define IMEI_LEN  		  16  //imei�ų���

#pragma pack(1)
//������ز����ṹ��
typedef struct _UPGRADE_PARAM_DATA
{
	uint32_t uiAddr;	/* �̼���ŵ�ַ, 0-Ƭ��, ����-Ƭ���ַ */
	uint32_t uiLen;		/* �̼��ֽ��� */
	uint8_t ucUpdate;	/* ��0: �´�����ʱ��Ҫͨ��Flash����, ��������0 */
  uint8_t aucReserve[247];	/* ��֤�����ṹ���СΪ256�ֽ� */
}UPGRADE_PARAM_DATA;

//	unsigned char  write_flag; //�Ƿ�д��flash��־
//	unsigned char mideware_version[EEPROM_FILED_LENGTH]; //�̼��汾�ţ�ռ32���ֽ�
//	unsigned char upgrade_system_addr[MAX_IPADDR_LEN]; //����������������ռ16���ֽ�
//	unsigned short upgrade_system_port; //�����������˿�
//	unsigned char  update_flag; //������־
//	unsigned char  update_result; //������� 0--δ����������ʧ�� 1--�����ɹ�,������ת
//	unsigned int   total_packetcount; //�����̼��ܰ���
//	unsigned int   cur_file_packnum; //��ǰ���صİ��ĸ���
//	uint8_t aucReserve[247];	/* ��֤�����ṹ���СΪ256�ֽ� */
typedef struct {
	/* 0. OTA����, 0.25kB */
	UPGRADE_PARAM_DATA stOta;
	/* 1. start after reset, 0.25kB */
	uint16_t usGoRun;		/* 0- sleep power off, 1- reset power off, 2- Switch power off, 3- going to run */
	uint8_t ucReserve[254];
} CFG_S;
extern CFG_S g_stCfg;
extern UPGRADE_PARAM_DATA g_upgradeparam_data;

#pragma pack()//�ظ�keilԭ�������ݶ��뷽ʽ



void write_upgradeparam_save_flash(void);
void read_upgradeparam_save_flash(void);
void jump_to_app(uint32_t app_addr);
void run(void);
void update_logic(void);
void App_to_Flash(void);

#endif

