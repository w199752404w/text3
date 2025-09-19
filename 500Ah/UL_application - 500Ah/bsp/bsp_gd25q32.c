/*******************************************************************************
* GD25Q32  32 M-Bit 4M Byte 
* 
* 256 Byte One Page
* 
*******************************************************************************/
#include "bsp_gd25q32.h"
#include <stdio.h>
#include <string.h>
#include "systick.h"

/*
static void delay_us(volatile uint32_t us)
{
    volatile uint32_t i, j;
    
    for(i = 0; i < us; i++)
    {
        for(j = 0; j < 26; j++);
    }
}
*/

void SPI_GPIO_Init() {
	rcu_periph_clock_enable(RCU_GPIOA);
	rcu_periph_clock_enable(RCU_GPIOC);
	
  gpio_init(GPIOA, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_4 |GPIO_PIN_5 | GPIO_PIN_7); /* SPI GPIO config:   CS/PA4 SCK/PA5, MOSI/PA7 */
	gpio_init(GPIOA, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_6);                                   /* SPI GPIO config: MISO/PA6 */
	gpio_init(GPIOC, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_4 | GPIO_PIN_9);                                        /* SPI GPIO config: WP/PC4 HOLD/PC 9,*/
	
	SPI_WP_HIGH; 
	SPI_HOLD_HIGH;
	W25Qx_Disable();
	SPI_SCK_HIGH;
	SPI_MOSI_HIGH;
}

//uint8_t spi_master_send_recv_byte(uint8_t spi_byte)
//{
//  uint8_t i;
//	uint8_t redata = 0;
//	
//	for(i=0;i<8;i++)
//	{
//	//	SPI_SCK_LOW;
//	//delay_us(5);
//		for(uint8_t j = 0; j < 80; j++);
//		if(spi_byte & 0x80)
//		{
//			SPI_MOSI_HIGH;
//		}
//		else
//		{
//			SPI_MOSI_LOW;
//		}
//		spi_byte <<= 1;
//		SPI_SCK_HIGH;
//  //delay_us(5); 
//		for(uint8_t j = 0; j < 50; j++);
//		redata<<=1;
//		if(SPI_MISO_READ)
//		{
//			redata++;
//		}
//		SPI_SCK_LOW;
//		for(uint8_t j = 0; j < 50; j++);
//	}
//	SPI_SCK_LOW;	
//	return redata;
//}
//uint8_t spi_master_send_recv_byte(uint8_t spi_byte)
//{
//  uint8_t i;
//	uint8_t redata = 0;	
//	for(i=0;i<8;i++)
//	{
//		if(spi_byte & 0x80)
//		{
//			SPI_MOSI_HIGH;
//		}
//		else
//		{
//			SPI_MOSI_LOW;
//		}
//		spi_byte <<= 1;
//		SPI_SCK_HIGH;
//    delay_us(2); 
//		if(SPI_MISO_READ)
//		{
//			redata |= 0x01;
//		}
//  	delay_us(2); 
//		SPI_SCK_LOW;
//  //delay_us(2);
//		if(i<7){
//			redata<<=1;
//		}
//	}	
//	return redata;
//}

uint8_t spi_master_send_recv_byte(uint8_t spi_byte)
{
  uint8_t i;
	uint8_t redata = 0;
	
	for(i=0;i<8;i++)
	{
		SPI_SCK_LOW;
  //delay_us(2);
		if(spi_byte & 0x80)
		{
			SPI_MOSI_HIGH;
		}
		else
		{
			SPI_MOSI_LOW;
		}
		spi_byte <<= 1;
		SPI_SCK_HIGH;
  //delay_us(2); 
		redata<<=1;
		if(SPI_MISO_READ)
		{
			redata++;
		}
	}
	SPI_SCK_LOW;	
	return redata;
}


void spi_master_recv_some_bytes( uint8_t *pbdata, uint16_t recv_length)
{
	uint8_t *temp_data;
  temp_data= pbdata;

	while (recv_length--)
	{
		*temp_data = spi_master_send_recv_byte(0xFF); 	//Send 0xff to provide clock for slave devices
		temp_data++;
	}
	
}

/**
  * @brief  Reads current status of the W25Q128FV.
  * @retval W25Q128FV memory status
  */
uint8_t BSP_W25Qx_GetStatus(void) {
	uint8_t cmd[] = {READ_STATUS_REG1_CMD};
	uint8_t status;
	W25Qx_Enable();
	status=spi_master_send_recv_byte(cmd[0]);
	W25Qx_Disable();
	/* Check the value of the register */
	if((status & W25Q128FV_FSR_BUSY) != 0) {
    return W25Qx_BUSY;
	} else {
		return W25Qx_OK;
	}
}


/**
  * @brief  This function reset the W25Qx.
  * @retval None
  */
void BSP_W25Qx_Reset(void) {
	uint8_t cmd[2] = {RESET_ENABLE_CMD, RESET_MEMORY_CMD};
	W25Qx_Enable();
	/* Send the reset command */
	spi_master_send_recv_byte(cmd[0]);
	spi_master_send_recv_byte(cmd[1]);
	W25Qx_Disable();
}

/**
  * @brief  Initializes the W25Q128FV interface.
  * @retval None
  */
uint8_t BSP_W25Qx_Init(void) {
  SPI_GPIO_Init();
	BSP_W25Qx_Reset();
	return BSP_W25Qx_GetStatus();
}

/**
  * @brief  This function send a Write Enable and wait it is effective.
  * @retval None
  */
uint8_t BSP_W25Qx_WriteEnable(void) {
	uint8_t cmd[] = {WRITE_ENABLE_CMD};
//	uint32_t tickstart = HAL_GetTick();
	while(BSP_W25Qx_GetStatus());
	/*Select the FLASH: Chip Select low */
	W25Qx_Enable();
	/* Send the read ID command */
  spi_master_send_recv_byte(cmd[0]);	
	/*Deselect the FLASH: Chip Select high */
	W25Qx_Disable();
	/* Wait the end of Flash writing */
	while(BSP_W25Qx_GetStatus());
	/* Check for the Timeout */
//	if((HAL_GetTick() - tickstart) > W25Qx_TIMEOUT_VALUE) {
//		return W25Qx_TIMEOUT;
//	}
	return W25Qx_OK;
}

/**
  * @brief  Read Manufacture/Device ID.
	* @param  return value address
  * @retval None
  */
void BSP_W25Qx_Read_ID(uint8_t *ID) {
	uint8_t cmd[4] = {READ_ID_CMD,0x00,0x00,0x00};
	W25Qx_Enable();
	/* Send the read ID command */
	spi_master_send_recv_byte(cmd[0]);
	spi_master_send_recv_byte(cmd[1]);
	spi_master_send_recv_byte(cmd[2]);
	spi_master_send_recv_byte(cmd[3]);
	/* Reception of the data */
	spi_master_recv_some_bytes( ID, 2);
	W25Qx_Disable();
}


/************************************************************************
  Function Name £ºFlash_Sector_Erase
  Description   £ºFlash sector erase
  Input         £º@Sector_Num: The sector code to be erased
 *************************************************************************/
uint8_t BSP_W25Qx_Erase_Sector(uint16_t Sector_Num) {
	uint8_t  cSendCmd[8];
	uint32_t tWriteAddr;
//uint32_t tickstart = HAL_GetTick();
	tWriteAddr = Sector_Num * SECTOR_SIZE;
	
	cSendCmd[0] = SECTOR_ERASE_CMD;
	cSendCmd[1] = (tWriteAddr & 0xff0000) >> 16;
	cSendCmd[2] = (tWriteAddr & 0xff00) >> 8;
	cSendCmd[3] = tWriteAddr & 0xff;
	while(BSP_W25Qx_GetStatus());
	BSP_W25Qx_WriteEnable();
	W25Qx_Enable();
	spi_master_send_recv_byte(cSendCmd[0]);
	spi_master_send_recv_byte(cSendCmd[1]);
	spi_master_send_recv_byte(cSendCmd[2]);
	spi_master_send_recv_byte(cSendCmd[3]);
	W25Qx_Disable();
	delay_1ms(100);                         //Simulate SPI, must
	/* Wait the end of Flash writing */
	while(BSP_W25Qx_GetStatus());
	/* Check for the Timeout */
//	if((HAL_GetTick() - tickstart) > W25Q128FV_SECTOR_ERASE_MAX_TIME) {
//		return W25Qx_TIMEOUT;
//	}
  
	return W25Qx_OK;
}

/**
  * @brief  Erases the specified block of the QSPI memory. 
  * @param  BlockAddress: Block address to erase  
  * @retval QSPI memory status
  */
//uint8_t BSP_W25Qx_Erase_Block(uint32_t Address) {
//	uint8_t cmd[4];
//	uint32_t tickstart = HAL_GetTick();
//	cmd[0] = SECTOR_ERASE_CMD;
//	cmd[1] = (uint8_t)(Address >> 16);
//	cmd[2] = (uint8_t)(Address >> 8);
//	cmd[3] = (uint8_t)(Address);
//	
//	/* Enable write operations */
//	BSP_W25Qx_WriteEnable();
//	
//	/*Select the FLASH: Chip Select low */
//	W25Qx_Enable();
//	/* Send the read ID command */
//	HAL_SPI_Transmit(&SpiHandle, cmd, 4, W25Qx_TIMEOUT_VALUE);
//	/*Deselect the FLASH: Chip Select high */
//	W25Qx_Disable();
//	
//	/* Wait the end of Flash writing */
//	while(BSP_W25Qx_GetStatus() == W25Qx_BUSY) {
//	}
//	/* Check for the Timeout */
//	if((HAL_GetTick() - tickstart) > W25Q128FV_SECTOR_ERASE_MAX_TIME) {
//		return W25Qx_TIMEOUT;
//	}
//	return W25Qx_OK;
//}

/**
  * @brief  Erases the entire QSPI memory.This function will take a very long time.
  * @retval QSPI memory status
  */
uint8_t BSP_W25Qx_Erase_Chip(void) {
	uint8_t cmd[4];
//uint32_t tickstart = HAL_GetTick();
	cmd[0] = CHIP_ERASE_CMD;	
	/* Enable write operations */
	BSP_W25Qx_WriteEnable();	
	/*Select the FLASH: Chip Select low */
	W25Qx_Enable();
	/* Send the read ID command */
	spi_master_send_recv_byte(cmd[0]);
	/*Deselect the FLASH: Chip Select high */
	W25Qx_Disable();
	
	/* Wait the end of Flash writing */
	while(BSP_W25Qx_GetStatus() == W25Qx_BUSY) {
	}
	/* Check for the Timeout */
//	if((HAL_GetTick() - tickstart) > W25Q128FV_SECTOR_ERASE_MAX_TIME) {
//		return W25Qx_TIMEOUT;
//	}
	
	return W25Qx_OK;
}

/**
  *@brief  Use SPI to send one byte of data
  *@Param  byte: The data to be sent
  *@retval returns the received data
  */
void SPI_FLASH_SendByte(uint8_t byte) {
  spi_master_send_recv_byte(byte);

}

/**
  *@ brief Use SPI to read one byte of data
  *@ param None
  *@ retval returns the received data
  */
uint8_t SPI_FLASH_ReadByte(void) {
	return spi_master_send_recv_byte(0xff);
}

/************************************************************************
  Function Name: Flash_ PageWrite
  Function function: Flash page writing
  Entrance parameters:
  TBuf -------->Pending data pointer
  TWriteAddr -->Address sent to Flash (byte aligned) (maximum 0X3FFFFF for 4-byte addresses (i.e. 4M bytes))
  TNum -------->Number of data sent (maximum 256)
 *************************************************************************/
uint8_t Flash_PageWrite(uint8_t* tBuf, uint32_t tWriteAddr, uint16_t tNum) {
	uint8_t cSendCmd[4];
//	uint32_t tickstart = HAL_GetTick();
	if (tWriteAddr > ADDRESS_MAX) {
		return W25Qx_ERROR;
	}
	while(BSP_W25Qx_GetStatus());
	BSP_W25Qx_WriteEnable();
	cSendCmd[0] = PAGE_PROG_CMD;
	cSendCmd[1] = (tWriteAddr & 0xff0000) >> 16;
	cSendCmd[2] = (tWriteAddr & 0xff00) >> 8;
	cSendCmd[3] = tWriteAddr & 0xff;
	W25Qx_Enable();
	
	for(int i =0;i<4;i++) {
		SPI_FLASH_SendByte(cSendCmd[i]);
	}
	for(int i =0;i<tNum;i++) {
		SPI_FLASH_SendByte(tBuf[i]);
	}
	
	W25Qx_Disable();
	/* Wait the end of Flash writing */
  while(BSP_W25Qx_GetStatus());
	/* Check for the Timeout */
//	if((HAL_GetTick() - tickstart) > W25Q128FV_SECTOR_ERASE_MAX_TIME) {
//		return W25Qx_TIMEOUT;
//	}

	return W25Qx_OK;
} 

/************************************************************************
  Function Name: Flash_ Write_ NoCheck
  Function function: Write data of the specified length starting from the specified address, but ensure that the address does not exceed the limit! With automatic page wrapping function
  Note: Erase the address to be written before writing
  Entrance parameters:
  TBuf -------->Pending data pointer
  TWriteAddr -->Address sent to Flash (4-byte address with a maximum of 1FFFFFF (i.e. 2M bytes))
  TNum -------->Number of data sent (65536)
  Return value:
 *************************************************************************/
void Flash_Write_NoCheck(uint8_t* pBuffer, uint32_t WriteAddr, uint32_t NumByteToWrite) {
	uint16_t Page_Remain;
	Page_Remain = PAGE_SIZE - WriteAddr % PAGE_SIZE;   //Calculate the remaining bytes on the current page
	if (NumByteToWrite <= Page_Remain) {
		Page_Remain = NumByteToWrite;
	}
	while(1) {
		Flash_PageWrite(pBuffer, WriteAddr, Page_Remain);
		delay_1ms(2);
		if (NumByteToWrite == Page_Remain) {
			break;   /*Indicates that the current data has been written*/
		} else {
			pBuffer += Page_Remain;
			WriteAddr += Page_Remain;
			NumByteToWrite -= Page_Remain;
			if (NumByteToWrite > PAGE_SIZE) {
				Page_Remain = PAGE_SIZE;             //Will write data greater than 256
			} else {
				Page_Remain = NumByteToWrite;       //Will write less than 256 data
			}
		}
	}
}

/************************************************************************
  Function Name: Flash_ Write
  Function function: Write data of the specified length starting from the specified address (this function has an erase operation!);
  Entrance parameters:
  PBuffer -------->Data buffer pointer
  WriteAddr -->Any Flash address, don't worry if it has been erased
  NumByteToWrite ->Number of data to write to (maximum 65535, which is one block size)
 *************************************************************************/
void MEM_FlashWrite(uint32_t WriteAddr, uint8_t *pBuffer, uint16_t NumByteToWrite) {
	uint32_t i;
	uint32_t Sector_Addr;
	uint32_t Sector_Num;
	uint32_t Sector_Offset;
	uint32_t Sector_Remain;
	/*calculate the beginning address of the current sector*/
	Sector_Addr = (WriteAddr / SECTOR_SIZE) * SECTOR_SIZE;	
	/*calculate the Sector number according to the WriteAddr*/
	Sector_Num = WriteAddr / SECTOR_SIZE;
	/*calculate the offest number of the sector*/
	Sector_Offset = WriteAddr % SECTOR_SIZE;
	/*calculate the remain of the sector*/
	Sector_Remain = SECTOR_SIZE - Sector_Offset;	
	if (NumByteToWrite <= Sector_Remain) {
		Sector_Remain = NumByteToWrite;
	}
	uint8_t Flash_Buff[SECTOR_SIZE];
	memset(&Flash_Buff, 0, sizeof(Flash_Buff));
	while(1) {
		/*copy the data writen previously into the sector to Flash_Buff*/
		MEM_FlashRead(Sector_Addr, Flash_Buff, SECTOR_SIZE);
		for(i = 0; i < Sector_Remain; i++) {
			if(Flash_Buff[Sector_Offset + i] != 0xFF && Flash_Buff[Sector_Offset + i] != pBuffer[i]) {
				break;
			}
		}
		if(i < Sector_Remain) {	/* i is broken */
			BSP_W25Qx_Erase_Sector(Sector_Num);
			memcpy(Flash_Buff + Sector_Offset, pBuffer, Sector_Remain);
			Flash_Write_NoCheck(Flash_Buff, Sector_Addr, SECTOR_SIZE);
		} else {
			Flash_Write_NoCheck(pBuffer, Sector_Addr + Sector_Offset, Sector_Remain);
		}
		if(NumByteToWrite == Sector_Remain) {
			break;
		} else {
			Sector_Addr += SECTOR_SIZE;
			Sector_Num++;	      //Next sector code
			Sector_Offset = 0;
			pBuffer += Sector_Remain;
			NumByteToWrite -= Sector_Remain;
			Sector_Remain = SECTOR_SIZE;  //Default write 4096 bytes
			if (NumByteToWrite < SECTOR_SIZE) {
				Sector_Remain = NumByteToWrite;  //When the data written this time is less than one sector size
			}
		}
	}
}



///************************************************************************
//Function Name: Flash_ Read
//Function function: Read data at the beginning of any specified address throughout the entire Flash range
//Entrance parameters:
//PBuf -------->Receive data buffer pointer
//TWriteAddr -->Read Flash address
//TNum -------->Number of read data
// *************************************************************************/
void MEM_FlashRead(uint32_t tReadAddr, uint8_t* pBuf, uint16_t tNum) {
	uint8_t cSend[4];
	cSend[0] = READ_CMD;
	cSend[1] = (tReadAddr & 0xff0000) >> 16;
	cSend[2] = (tReadAddr & 0xff00) >> 8;
	cSend[3] = tReadAddr & 0xff;
	while(BSP_W25Qx_GetStatus());
	W25Qx_Enable();
	for(int i =0;i<4;i++) {
		SPI_FLASH_SendByte(cSend[i]);
	}
	for(int i =0;i<tNum;i++) {
		pBuf[i]=SPI_FLASH_ReadByte();
	}
	W25Qx_Disable();
}

