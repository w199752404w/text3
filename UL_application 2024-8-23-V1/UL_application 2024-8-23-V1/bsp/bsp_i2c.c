#include "bsp_i2c.h"
#include "systick.h"
#include "stdio.h"
#include "stdbool.h"


/******************************************************************************
* Function Name  : void Delay_us(unsigned long i)
* Description    : us level delay
* Input          : i
* Output         : None
*******************************************************************************/	
static void Delay_us(unsigned long i)
{
	unsigned long j;
	for(;i>0;i--)
	{
			for(j=30;j>0;j--);
	}
}

void IIC_Init(void) {
    rcu_periph_clock_enable(RCU_GPIOB);
//    i2c_clock_config(I2C0, 100000, I2C_DTCY_2);
	gpio_init(IIC_SCL_PORT, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, SCL_PIN); //SCL，push-pull   
	gpio_init(IIC_SDA_PORT, GPIO_MODE_OUT_OD, GPIO_OSPEED_50MHZ, SDA_PIN); //SDA，Open drain output
	I2C_SDA_1();
	I2C_SCL_1();
}

/*******************************************************************************
* Function Name  : i2c_Start
* Description    : Analog IIC start signal
* Input          : None
* Output         : None
*******************************************************************************/
void i2c_Start(void) {
	I2C_SDA_1();
	I2C_SCL_1();
	Delay_us(5);
	I2C_SDA_0();
	Delay_us(5);
	I2C_SCL_0();
	Delay_us(5);
}

/*******************************************************************************
* Function Name  : i2c_Stop
* Description    : Analog IIC stop signal
* Input          : None
* Output         : None
*******************************************************************************/
void i2c_Stop(void) {
	I2C_SDA_0();
	I2C_SCL_1();
	Delay_us(5);
	I2C_SDA_1();
	Delay_us(5);
}

/*******************************************************************************
* Function Name  : i2c_SendByte
* Description    : Simulate IIC sending a byte
* Input          : Send_Byte  ：Eight bit data sent
*				   				 i2c_timeout: Set IIC timeout time
* Output         : 0: IIC send failed 1: IIC send succeeded
*******************************************************************************/
uint8_t i2c_SendByte(uint8_t Send_Byte) {
	uint8_t i,time_flg;
	for (i = 0; i < 8; i++) {
		if (Send_Byte & 0x80) {
			I2C_SDA_1();
		} else {
			I2C_SDA_0();
		}
		Delay_us(5);
		I2C_SCL_1();
		Delay_us(5);
		I2C_SCL_0();
		Send_Byte <<= 1;			// Move left by one bit
	}
	Delay_us(5);
	I2C_SDA_1();
	I2C_SCL_1();
	Delay_us(5);
	while(I2C_SDA_READ()) {
		time_flg++;
		delay_1ms(1);        //Delay 1ms
		if(time_flg > I2C_TIMEOUT) {
			I2C_SCL_0();
			Delay_us(5);
			return 0;
		}
	}
	I2C_SCL_0();
	Delay_us(5);
	return 1;
}



/*******************************************************************************
* Function Name  : i2c_ReadByte
* Description    : Simulate IIC reading one byte
* Input          : None
* Output         : dat ：Read Octet Data
*******************************************************************************/
uint8_t i2c_ReadByte(void) {
	uint8_t i,dat;
	dat = 0;
	I2C_SDA_1();
	Delay_us(5);
	for (i = 0; i < 8; i++) {
		I2C_SCL_1();
		Delay_us(5);
		dat <<= 1;
		dat |= I2C_SDA_READ();
		Delay_us(5);
		I2C_SCL_0();
		Delay_us(5);
	}
	return dat;
}

/*******************************************************************************
* Function Name  : i2c_Ack
* Description    : Analog IIC response signal
* Input          : None
* Output         : None
*******************************************************************************/
void i2c_Ack(void) {
	I2C_SDA_0();	/* CPU driver SDA = 0 */
	I2C_SCL_1();	/* CPU generates 1 clock*/
	Delay_us(5);
	I2C_SCL_0();
	Delay_us(5);
	//	I2C_SDA_1();	/* CPU releases SDA bus */
}

/*******************************************************************************
* Function Name  : i2c_NoAck
* Description    : Analog IIC non response signal
* Input          : None
* Output         : None
*******************************************************************************/
void i2c_NoAck(void) {
	I2C_SDA_1();	/* CPU driver SDA = 0 */
	Delay_us(5);
	I2C_SCL_1();	/* CPU generates 1 clock */
	Delay_us(5);
	I2C_SCL_0();
	Delay_us(5);
}

/*******************************************************************************
* Function Name  : IIC_WriteData
* Description    : Write data to the slave
* Input          : Dec_Add 	   : The address of the slave device
*				   				 Reg_Add 	   ：Memory address
*				   				 pData       ：Write to the first address of the data buffer
*				   				 Dat_Len     ：The length of the written data
* Output         : None
* Return         : ucAck ：Slave answers, non-answers
*******************************************************************************/
//void IIC_WriteData(uint8_t Dev_Add,uint8_t Reg_Add,uint8_t *pData,uint8_t Dat_Len )
//{
//		uint8_t i;
//    i2c_Start();                          
//    i2c_SendByte(Dev_Add);       
//		i2c_SendByte(Reg_Add);
//    for(i = 0;i < Dat_Len;i++)
//		i2c_SendByte(*pData++);                       			             
//    i2c_Stop();     
//}

/*******************************************************************************
* Function Name  : IIC_ReadData
* Description    : Read data from the slave
* Input          : Dec_Add 	   : The address of the slave device
*				  				 Reg_Add 	   ：Memory address
*				   				 pData       ：Write to the first address of the data buffer
*				  				 Dat_Len     ：The length of the written data
* Output         : None
* Return         : None
*******************************************************************************/
//void IIC_ReadData(uint8_t Dev_Add,uint8_t Reg_Add,uint8_t *pData,uint8_t Dat_Len )
//{
//	uint8_t i;
//  i2c_Start();
//	i2c_SendByte(Dev_Add);
//	i2c_SendByte(Reg_Add);
//	i2c_Start();
//	Dev_Add &= (~0x01);
//	Dev_Add |= 1;
//	i2c_SendByte(Dev_Add);
//	for(i = 0;i < Dat_Len; i++)
//	{
//		*pData++ = i2c_ReadByte();
//		if(Dat_Len - i - 1)
//			i2c_Ack();
//		else i2c_NoAck();
//	}
//	i2c_Stop();  
//}
