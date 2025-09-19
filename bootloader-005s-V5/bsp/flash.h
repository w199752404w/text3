#ifndef __FLASH_H__
#define __FLASH_H__
#include "type.h"
#include "gd32f10x.h"


#define BOOT_START_ADDRESS      0x08000000U
#define BOOT_END_ADDRESS        0x08005000U
#define BOOT_SIZE  BOOT_END_ADDRESS - BOOT_START_ADDRESS

#define SETTINGS_START_ADDRESS  0x08005000U
#define SETTINGS_END_ADDRESS    0x08007000U
#define SETTINGS_SIZE SETTINGS_END_ADDRESS - SETTINGS_START_ADDRESS


#define APP_START_ADDRESS        0x08004000U
#define APP_END_ADDRESS          0x08040000U
#define APP_SIZE  APP_END_ADDRESS - APP_START_ADDRESS

#define DOWNLOAD_START_ADDRESS  (0x08007000U + 101*1024)
#define DOWNLOAD_END_ADDRESS    (0x08007000U + 101*1024+100*1024)
#define DOWNLOAD_SIZE DOWNLOAD_END_ADDRESS - DOWNLOAD_START_ADDRESS


#define USER_FLASH_LAST_PAGE_ADDRESS  0x08040000


/* FLASH大小 : 256K */
#define GD32_FLASH_SIZE         0x00040000UL
/* FLASH起始地址 */
#define GD32_FLASH_BASE         0x08000000UL
/* FLASH结束地址 */
#define GD32_FLASH_END          (GD32_FLASH_BASE | STM32_FLASH_SIZE)

#define FLASH_PAGE_SIZE          ((uint32_t)0x800)

#define STM32_FLASH_PAGE_NUM     (GD32_FLASH_SIZE / FLASH_PAGE_SIZE)

void FLASH_If_Init(void);
uint32_t FLASH_If_Erase(uint32_t StartSector);
MI_BOOL gd32_flash_erase(MI_U32 start_addr, MI_U32 end_addr);
MI_BOOL gd32_flash_write(MI_U32 dest_addr, MI_U8 *src, MI_U32 Len);
MI_BOOL gd32_flash_read(MI_U32 dest_addr, MI_U8* buff, MI_U32 Len);
MI_BOOL copy_download_to_app(void);



/// 移植修改区 ///
/* FLASH大小：256K */
#define GD32FLASH_SIZE  0x00040000UL

/* FLASH起始地址 */
#define GD32FLASH_BASE  FLASH_BASE

/* FLASH结束地址 */
#define GD32FLASH_END   (GD32FLASH_BASE | GD32FLASH_SIZE)

/* FLASH页大小：2K */
#define GD32FLASH_PAGE_SIZE ((uint32_t)0x800)

/* FLASH总页数 */
#define GD32FLASH_PAGE_NUM  (GD32FLASH_SIZE / GD32FLASH_PAGE_SIZE)

/// 导出函数声明 
void FLASH_Init(void);
uint32_t FLASH_Read(uint32_t Address, void *Buffer, uint32_t Size);
uint32_t FLASH_Write(uint32_t Address, const uint16_t *Buffer, uint32_t NumToWrite);
int FLASH_ErasePage(uint32_t PageAddress, uint32_t NbPages);
uint32_t FLASH_WriteNotErase(uint32_t Address, const uint16_t *Buffer, uint32_t NumToWrite);



#define FMC_PAGE_SIZE           ((uint16_t)0x800)
typedef enum {FAILED = 0, PASSED = !FAILED} TestStatus;
TestStatus FLASH_Program(uint32_t WRITE_START_ADDR, uint16_t Size, uint32_t * data);

#endif //__FLASH_H__
