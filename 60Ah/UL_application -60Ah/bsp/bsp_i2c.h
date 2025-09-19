#ifndef   _BSP_I2C_H_
#define   _BSP_I2C_H_

#include "gd32f10x.h"
#include "stdbool.h"

#define I2C_TIMEOUT   5					                                                              //5ms timeout

/* Define macros for reading and writing SCL and SDA */
#define IIC_SCL_PORT   GPIOB
#define IIC_SDA_PORT   GPIOB
#define SCL_PIN        GPIO_PIN_8                                                          //Simulate the SCL signal of IIC 
#define SDA_PIN        GPIO_PIN_9                                                          //Simulate the SDA signal of IIC
#define I2C_SCL_1()       gpio_bit_set(IIC_SCL_PORT,SCL_PIN);			                         // SCL = 1 
#define I2C_SCL_0()       gpio_bit_reset(IIC_SCL_PORT,SCL_PIN);	  		                     // SCL = 0 

#define I2C_SDA_1()       gpio_bit_set(IIC_SDA_PORT,SDA_PIN);			                         // SDA = 1 
#define I2C_SDA_0()       gpio_bit_reset(IIC_SDA_PORT,SDA_PIN);	 	      	                 // SDA = 0 

#define I2C_SDA_READ()    gpio_input_bit_get(IIC_SDA_PORT,SDA_PIN)						             // Read SDA port line status
#define I2C_SCL_READ() 	  gpio_input_bit_get(IIC_SDA_PORT,SCL_PIN)					               // Reading SCL port line status 

void IIC_Init(void);
void i2c_Start(void);
void i2c_Stop(void);
uint8_t i2c_SendByte(uint8_t Send_Byte);
uint8_t i2c_ReadByte(void);
void i2c_Ack(void);
void i2c_NoAck(void); 
static void i2c_Delay(void);		

uint8_t i2c_ReadByte(void);
//void IIC_WriteData(uint8_t Dev_Add,uint8_t Reg_Add,uint8_t *pData,uint8_t Dat_Len );			//Standard IIC Writing
//void IIC_ReadData (uint8_t Dev_Add,uint8_t Reg_Add,uint8_t *pData,uint8_t Dat_Len );			//Standard IIC Reading

#endif
