#include "type.h"
#include "flash.h"
#include "systick.h"
#include "iap.h"
#include "bsp_gd25q32.h"
// ��������
MI_BOOL gd32_flash_erase(MI_U32 start_addr, MI_U32 end_addr)
{
    int page_num = 0; 
    /* unlock the flash program/erase controller */
    fmc_unlock();

    /* clear all pending flags */
    fmc_flag_clear(FMC_FLAG_BANK0_END);
    fmc_flag_clear(FMC_FLAG_BANK0_WPERR);
    fmc_flag_clear(FMC_FLAG_BANK0_PGERR);

    /* erase the flash pages */
    while(start_addr < end_addr)
    {
        page_num = (start_addr - GD32_FLASH_BASE) / FLASH_PAGE_SIZE;

        fmc_page_erase((page_num * FLASH_PAGE_SIZE) + BOOT_START_ADDRESS);
        fmc_flag_clear(FMC_FLAG_BANK0_END);
        fmc_flag_clear(FMC_FLAG_BANK0_WPERR);
        fmc_flag_clear(FMC_FLAG_BANK0_PGERR);

        start_addr += FLASH_PAGE_SIZE;
    }

    /* lock the main FMC after the erase operation */
    fmc_lock();
    return MI_TRUE;
}


//FLASH����
uint32_t FLASH_If_Erase(uint32_t StartSector)
{
  uint32_t flashaddress;
  
  flashaddress = StartSector;
  fmc_unlock();
  while (flashaddress <= (uint32_t) USER_FLASH_LAST_PAGE_ADDRESS) 
  {
    if (fmc_page_erase(flashaddress) == FMC_READY)
    {
      flashaddress += FLASH_PAGE_SIZE;
    }
    else
    {
      return (1);
    }
  }
	fmc_lock();
  return (0);
}


// д����
MI_BOOL gd32_flash_write(MI_U32 dest_addr, MI_U8 *src, MI_U32 Len)
{
    MI_U32 i = 0;

    fmc_unlock();

    for(i = 0; i < Len; i += 4)
    {
        /* Device voltage range supposed to be [2.7V to 3.6V], the operation will
        be done by byte */
        int ret = fmc_word_program((MI_U32)(dest_addr+i), *(uint32_t*)(src+i));
        if(FMC_READY == ret)
        {
            fmc_flag_clear(FMC_FLAG_BANK0_END);
            fmc_flag_clear(FMC_FLAG_BANK0_WPERR);
            fmc_flag_clear(FMC_FLAG_BANK0_PGERR);
            /* Check the written value */
            if(*(uint32_t *)(src + i) != *(uint32_t*)(dest_addr+i))
            {
                /* Flash content doesn't match SRAM content */
                fmc_lock();  
                printf("write error 1\r\n");             
                return MI_FALSE;
            }
        }
        else
        {
            fmc_lock(); 
            printf("write error2\r\n");           
            /* Error occurred while writing data in Flash memory */
            return MI_FALSE;
        }
    }
    fmc_lock();            

   return MI_TRUE;
}

// ������
MI_BOOL gd32_flash_read(MI_U32 dest_addr, MI_U8* buff, MI_U32 Len)
{

    MI_U32 i;
    for(i = 0; i < Len; i++)
    {
        buff[i] = *(__IO MI_U8*)(dest_addr + i);
    }
    /* Return a valid address to avoid HardFault */
    return MI_TRUE;
}

MI_BOOL copy_download_to_app(void)
{
    MI_U8 buffer[1024] = {0};  //ÿ�δ�download��1K�ֽڣ�Ȼ��д��1K
    MI_U8 w_count = 0;
    MI_U16 w_len = sizeof(buffer);
    MI_U32 app_size = APP_SIZE; 

    w_count = app_size / w_len;
    // ��App�������
    gd32_flash_erase(APP_START_ADDRESS,APP_END_ADDRESS);
    printf("w_count == %d\r\n",APP_SIZE);
    printf("w_count == %d\r\n",w_len);
    printf("w_count == %d\r\n",w_count);

    for (int i = 0;i < w_count;i++)
    {
       // gd32_flash_read(DOWNLOAD_START_ADDRESS + (w_len * i),buffer,w_len);
			 	MEM_FlashRead(g_upgradeparam_data.uiAddr + (w_len * i),buffer,w_len);   //��ȡƬ��flash���� 
        delay_1ms(50);
        gd32_flash_write(APP_START_ADDRESS + (w_len * i),buffer,w_len);
        printf("update.............................%d [100] \r\n",((i+1) * 100 /w_count));
    }

    return MI_TRUE;
}



__align(4) static uint16_t FlashBuffer[GD32FLASH_PAGE_SIZE >> 1];
/// ��ʼ��FLASH
void FLASH_Init(void)
{
	fmc_unlock();
    fmc_flag_clear(FMC_FLAG_BANK0_END);
    fmc_flag_clear(FMC_FLAG_BANK0_WPERR);
    fmc_flag_clear(FMC_FLAG_BANK0_PGERR);
	fmc_lock();
}

/**
 * ��FLASH
 * @param  Address ��ַ
 * @param  Buffer  ��Ŷ�ȡ������
 * @param  Size    Ҫ��ȡ�����ݴ�С����λ�ֽ�
 * @return         �����ɹ����ֽ���
 */
uint32_t FLASH_Read(uint32_t Address, void *Buffer, uint32_t Size)
{
    uint32_t nread = Size;
    uint8_t* d = (uint8_t *)Buffer;
    const uint8_t* s = (const uint8_t *)Address;

    if (!Buffer || Address < GD32FLASH_BASE || Address >= GD32FLASH_END)
        return 0;

    while (nread >= sizeof(uint32_t) && (((uint32_t)s) <= (GD32FLASH_END - 4)))
    {
        *(volatile uint32_t *)d = *(volatile uint32_t *)s;
        d += sizeof(uint32_t);
        s += sizeof(uint32_t);
        nread -= sizeof(uint32_t);
    }

    while (nread && (((uint32_t)s) < GD32FLASH_END))
    {
        *d++ = *s++;
        nread--;
    }

    return Size - nread;
}

/**
 * дFLASH
 * @param  Address    д����ʼ��ַ��������Ҫ��2�ֽڶ��룡����
 * @param  Buffer     ��д������ݣ�������Ҫ��2�ֽڶ��룡����
 * @param  NumToWrite Ҫд�������������λ�����֣�������Ҫ��2�ֽڶ��룡����
 * @return            ʵ��д�������������λ���ֽ�
 */
 uint32_t FLASH_Write(uint32_t Address, const uint16_t *Buffer, uint32_t NumToWrite)
{
    uint32_t i = 0;
    uint32_t pagepos = 0;         // ҳλ��
    uint32_t pageoff = 0;         // ҳ��ƫ�Ƶ�ַ
    uint32_t pagefre = 0;         // ҳ�ڿ���ռ�
    uint32_t offset = 0;          // Address��FLASH�е�ƫ��
    uint32_t nwrite = NumToWrite; // ��¼ʣ��Ҫд���������
    const uint16_t *pBuffer = (const uint16_t *)Buffer;

    /* �Ƿ���ַ */
    if (Address < GD32FLASH_BASE || Address > (GD32FLASH_END - 2) || NumToWrite == 0 || pBuffer == NULL)
        return 0;

    /* ����FLASH */
    fmc_unlock();

    /* ����ƫ�Ƶ�ַ */
    offset = Address - GD32FLASH_BASE;

    /* ���㵱ǰҳλ�� */
    pagepos = offset / GD32FLASH_PAGE_SIZE; 

    /* ����Ҫд���ݵ���ʼ��ַ�ڵ�ǰҳ�ڵ�ƫ�Ƶ�ַ */
    pageoff = ((offset % GD32FLASH_PAGE_SIZE) >> 1);

    /* ���㵱ǰҳ�ڿ���ռ� */
    pagefre = ((GD32FLASH_PAGE_SIZE >> 1) - pageoff);

    /* Ҫд������������ڵ�ǰҳ������ */
    if (nwrite <= pagefre)
        pagefre = nwrite;

    while (nwrite != 0)
    {
        /* ����Ƿ�ҳ */
        if (pagepos >= GD32FLASH_PAGE_NUM)
            break;

        /* ��ȡһҳ */
        FLASH_Read(GD32FLASH_BASE + pagepos * GD32FLASH_PAGE_SIZE, FlashBuffer, GD32FLASH_PAGE_SIZE);

        /* ����Ƿ���Ҫ���� */
        for (i = 0; i < pagefre; i++)
        {
            if (*(FlashBuffer + pageoff + i) != 0xFFFF) /* FLASH������Ĭ������ȫΪ0xFF */
                break;
        }

        if (i < pagefre)
        {
            uint32_t count = 0;
            uint32_t index = 0;

			if(FLASH_ErasePage(GD32FLASH_BASE + pagepos * GD32FLASH_PAGE_SIZE, 1) < 0)
				break;

            /* ���Ƶ����� */
            for (index = 0; index < pagefre; index++)
            {
                *(FlashBuffer + pageoff + index) = *(pBuffer + index);
            }

            /* д��FLASH */
            count = FLASH_WriteNotErase(GD32FLASH_BASE + pagepos * GD32FLASH_PAGE_SIZE, FlashBuffer, GD32FLASH_PAGE_SIZE >> 1);
            if (count != (GD32FLASH_PAGE_SIZE >> 1))
            {
                nwrite -= count;
                break;
            }
        }
        else
        {
            /* ���������ֱ��д */
            uint32_t count = FLASH_WriteNotErase(Address, pBuffer, pagefre);
            if (count != pagefre)
            {
                nwrite -= count;
                break;
            }
        }

        pBuffer += pagefre;         /* ��ȡ��ַ����         */
        Address += (pagefre << 1);  /* д���ַ����         */
        nwrite -= pagefre;          /* ����ʣ��δд�������� */

        pagepos++;     /* ��һҳ           */
        pageoff = 0;   /* ҳ��ƫ�Ƶ�ַ����  */

        /* ����ʣ���������´�д�������� */
        pagefre = nwrite >= (GD32FLASH_PAGE_SIZE >> 1) ? (GD32FLASH_PAGE_SIZE >> 1) : nwrite;
    }

    /* ����FLASH */
    fmc_lock();

    return ((NumToWrite - nwrite) << 1);
}

uint32_t FLASH_WriteNotErase(uint32_t Address, const uint16_t *Buffer, uint32_t NumToWrite)
{
    uint32_t nwrite = NumToWrite;
    uint32_t addrmax = GD32FLASH_END - 2;

    while (nwrite)
    {
        if (Address > addrmax)
            break;

		fmc_halfword_program(Address, *Buffer);
		
        if ((*(__IO uint16_t*) Address) != *Buffer)
            break;

        nwrite--;
        Buffer++;
        Address += 2;
    }
    return (NumToWrite - nwrite);
}

int FLASH_ErasePage(uint32_t PageAddress, uint32_t NbPages)
{
	while(NbPages--)
	{
		if(fmc_page_erase(PageAddress) != FMC_READY)
		{
			return -1;
		}
		PageAddress += GD32FLASH_PAGE_SIZE;
	}
	return 0;
}


TestStatus FLASH_Program(uint32_t WRITE_START_ADDR, uint16_t Size, uint32_t * data)
{
    uint32_t Address;
    TestStatus TransferStatus = FAILED;
    uint32_t i;
    TransferStatus = PASSED; 
    /* Unlock the Flash Bank1 Program Erase controller */
    fmc_unlock();
    
    /* Clear All pending flags */
    fmc_flag_clear(FMC_FLAG_BANK0_END | FMC_FLAG_BANK0_WPERR | FMC_FLAG_BANK0_PGERR);
    
    
    /* Program Flash Bank1 */
    Address = WRITE_START_ADDR;
    i = 0;
    while(Address < (WRITE_START_ADDR + Size))
    {
        fmc_word_program(Address, data[i]);
        i++;
        Address = Address + 4; 
        fmc_flag_clear(FMC_FLAG_BANK0_END | FMC_FLAG_BANK0_WPERR | FMC_FLAG_BANK0_PGERR);
    }
    
    fmc_lock();
    
    return TransferStatus;
}


