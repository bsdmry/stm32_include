/*
 * i2c.h
 *
 *  Created on: 27 авг. 2017 г.
 *      Author: igor
 */
#include <string.h>
#include <stdint.h>

#ifdef STM32F00X
#include "stm32f0xx.h"
#include "stm32f0xx_rcc.h"
#include "stm32f0xx_gpio.h"
#endif

#ifdef STM32F10X
#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_i2c.h"
#endif

#ifndef I2C_H_
#define I2C_H_


GPIO_InitTypeDef i2c_pins;
I2C_InitTypeDef i2c;

typedef enum _I2C_Direction {I2C_Transmitter=0, I2C_Receiver=1} I2C_Direction;

void hw_i2c_init(){
	   #ifdef STM32F00X
	   RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
	   #endif
	   #ifdef STM32F10X
	   RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	   RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
       #endif
	   RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1,ENABLE);

	   i2c_pins.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_8;
	   i2c_pins.GPIO_Speed = GPIO_Speed_50MHz;
	   #ifdef STM32F00X
	   i2c_pins.GPIO_Mode = GPIO_Mode_AF;
	   i2c_pins.GPIO_OType = GPIO_OType_OD;
	   i2c_pins.GPIO_PuPd = GPIO_PuPd_NOPULL;

	   GPIO_Init(GPIOB, &i2c_pins);

	   GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_1); //ENABLE ALTERNATE FUNCTION
	   GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_1);

	   I2C_StructInit(&i2c);
	   i2c.I2C_Ack = I2C_Ack_Enable;
	   i2c.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	   i2c.I2C_Mode = I2C_Mode_SMBusHost;
	   i2c.I2C_OwnAddress1 = 0x00;
	   i2c.I2C_AnalogFilter = I2C_AnalogFilter_Enable;
	   i2c.I2C_DigitalFilter = 0;
	   i2c.I2C_Timing = 0x0010020B; // Расчитано в Excel-таблице I2C_Timing_Configuration_V1.0.1.xls от ST
	   #endif
	   #ifdef STM32F10X
	   i2c_pins.GPIO_Mode = GPIO_Mode_AF_OD;
	   GPIO_Init(GPIOB, &i2c_pins);
	   GPIO_PinRemapConfig(GPIO_Remap_I2C1, ENABLE); //ENABLE ALTERNATE FUNCTION
	   i2c.I2C_Mode = I2C_Mode_I2C;
	   i2c.I2C_DutyCycle = I2C_DutyCycle_2;
	   i2c.I2C_OwnAddress1 = 0x00;
	   i2c.I2C_Ack = I2C_Ack_Enable;
	   i2c.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	   i2c.I2C_ClockSpeed = 100000;
	   #endif
	   I2C_Init(I2C1, &i2c);
	   I2C_AcknowledgeConfig(I2C1, ENABLE);
	   I2C_Cmd(I2C1, ENABLE);
}


#ifdef STM32F00X
void I2C_write(uint8_t dev_addr, uint8_t address, uint8_t *data, uint8_t data_cnt)
{
	uint8_t dev_wr_addr = (dev_addr<<1);
	while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY) == SET);
	I2C_TransferHandling(I2C1, dev_wr_addr, 1, I2C_Reload_Mode, I2C_Generate_Start_Write);
	while(I2C_GetFlagStatus(I2C1, I2C_FLAG_TXIS) == RESET);
	I2C_SendData(I2C1, address);
	while(I2C_GetFlagStatus(I2C1, I2C_FLAG_TCR) == RESET);
	I2C_TransferHandling(I2C1, dev_wr_addr, 1, I2C_AutoEnd_Mode, I2C_No_StartStop);
	for(int8_t cnt = 0; cnt<data_cnt; cnt++){
		while(I2C_GetFlagStatus(I2C1, I2C_FLAG_TXIS) == RESET);
		I2C_SendData(I2C1, data[cnt]);
	}
	while(I2C_GetFlagStatus(I2C1, I2C_FLAG_STOPF) == RESET);
	I2C_ClearFlag(I2C1, I2C_FLAG_STOPF);
}
void I2C_write8(uint8_t dev_addr, uint8_t address, uint8_t data)
{
	uint8_t dev_wr_addr = (dev_addr<<1);
	while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY) == SET);
	I2C_TransferHandling(I2C1, dev_wr_addr, 1, I2C_Reload_Mode, I2C_Generate_Start_Write);
	while(I2C_GetFlagStatus(I2C1, I2C_FLAG_TXIS) == RESET);
	I2C_SendData(I2C1, address);
	while(I2C_GetFlagStatus(I2C1, I2C_FLAG_TCR) == RESET);
	I2C_TransferHandling(I2C1, dev_wr_addr, 1, I2C_AutoEnd_Mode, I2C_No_StartStop);
	while(I2C_GetFlagStatus(I2C1, I2C_FLAG_TXIS) == RESET);
	I2C_SendData(I2C1, data);
	while(I2C_GetFlagStatus(I2C1, I2C_FLAG_STOPF) == RESET);
	I2C_ClearFlag(I2C1, I2C_FLAG_STOPF);
}
void I2C_write_byte(uint8_t dev_addr, uint8_t data)
{
	uint8_t dev_wr_addr = (dev_addr<<1);
	while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY) == SET);
	I2C_TransferHandling(I2C1, dev_wr_addr, 1, I2C_AutoEnd_Mode, I2C_Generate_Start_Write);
	while(I2C_GetFlagStatus(I2C1, I2C_FLAG_TXIS) == RESET);
	I2C_SendData(I2C1, data);
	//while(I2C_GetFlagStatus(I2C1, I2C_ISR_TC) == RESET);
	while(I2C_GetFlagStatus(I2C1, I2C_FLAG_STOPF) == RESET);
	I2C_ClearFlag(I2C1, I2C_FLAG_STOPF);
}

void I2C_read(uint8_t dev_addr, uint8_t Reg, uint8_t *Data, uint8_t DCnt)
{
	uint8_t dev_wr_addr = (dev_addr<<1);
	uint8_t dev_rd_addr = dev_wr_addr | (1<<0);

	while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY) == SET);
	I2C_TransferHandling(I2C1, dev_wr_addr, 1, I2C_SoftEnd_Mode, I2C_Generate_Start_Write);
	while(I2C_GetFlagStatus(I2C1, I2C_FLAG_TXIS) == RESET);
	I2C_SendData(I2C1, Reg);
	while(I2C_GetFlagStatus(I2C1, I2C_FLAG_TC) == RESET);
	I2C_TransferHandling(I2C1, dev_rd_addr, DCnt, I2C_AutoEnd_Mode, I2C_Generate_Start_Read);
	for(int8_t Cnt = 0; Cnt<DCnt; Cnt++){
		while(I2C_GetFlagStatus(I2C1, I2C_FLAG_RXNE) == RESET);
		Data[Cnt] = I2C_ReceiveData(I2C1);
	}
	while(I2C_GetFlagStatus(I2C1, I2C_FLAG_STOPF) == RESET);
	I2C_ClearFlag(I2C1, I2C_FLAG_STOPF);
}
#endif
#ifdef STM32F10X
void I2C_write(uint8_t dev_addr, uint8_t address, uint8_t *data, uint8_t data_cnt)
{
	uint8_t dev_wr_addr = (dev_addr<<1);
	I2C_GenerateSTART(I2C1,ENABLE);
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));
	I2C_Send7bitAddress(I2C1, dev_wr_addr, I2C_Direction_Transmitter);
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
	I2C_SendData(I2C1,address);
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
	for(int8_t cnt = 0; cnt<data_cnt; cnt++){
		I2C_SendData(I2C1, data[cnt]);
		while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
	}
	I2C_GenerateSTOP(I2C1,ENABLE);
}
void I2C_write8(uint8_t dev_addr, uint8_t address, uint8_t data)
{
	uint8_t dev_wr_addr = (dev_addr<<1);
	I2C_GenerateSTART(I2C1,ENABLE);
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));
	I2C_Send7bitAddress(I2C1, dev_wr_addr, I2C_Direction_Transmitter);
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
	I2C_SendData(I2C1,address);
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
	I2C_SendData(I2C1,data);
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
	I2C_GenerateSTOP(I2C1,ENABLE);
}
void I2C_write_byte(uint8_t dev_addr, uint8_t data)
{
	uint8_t dev_wr_addr = (dev_addr<<1);
	I2C_GenerateSTART(I2C1,ENABLE);
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));
	I2C_Send7bitAddress(I2C1, dev_wr_addr, I2C_Direction_Transmitter);
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
	I2C_SendData(I2C1,data);
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
	I2C_GenerateSTOP(I2C1,ENABLE);
}
void I2C_read(uint8_t dev_addr, uint8_t Reg, uint8_t *Data, uint8_t DCnt)
{
	uint8_t dev_wr_addr = (dev_addr<<1);
	uint8_t dev_rd_addr = dev_wr_addr | (1<<0);
	while (I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY));
	I2C_AcknowledgeConfig(I2C1, ENABLE);
	I2C_GenerateSTART(I2C1,ENABLE);
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));
	I2C_Send7bitAddress(I2C1, dev_wr_addr, I2C_Direction_Transmitter);
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
	I2C_SendData(I2C1,Reg);
	while (!I2C_GetFlagStatus(I2C1,I2C_FLAG_TXE));

	I2C_GenerateSTART(I2C1, ENABLE);
	while (!I2C_GetFlagStatus(I2C1,I2C_FLAG_SB));
	I2C_Send7bitAddress(I2C1, dev_rd_addr, I2C_Direction_Receiver);
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
	for(int8_t Cnt = 0; Cnt<DCnt; Cnt++){
		while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));
		Data[Cnt] = I2C_ReceiveData(I2C1);
	}
	I2C_NACKPositionConfig(I2C1, I2C_NACKPosition_Current);
	I2C_AcknowledgeConfig(I2C1, DISABLE);
	I2C_GenerateSTOP(I2C1,ENABLE);
	while(I2C_GetFlagStatus(I2C1, I2C_FLAG_STOPF));
}
#endif


void I2C_read8(uint8_t dev_addr, uint8_t Reg, uint8_t *result){
	uint8_t tmp[1];
	uint16_t rr;
	I2C_read(dev_addr, Reg, tmp, 1);
	*result = tmp;
}

void I2C_read16(uint8_t dev_addr, uint8_t Reg, uint16_t *result){
	uint8_t tmp[2];
	uint16_t rr;
	I2C_read(dev_addr, Reg, tmp, 2);
	rr = (((uint16_t)tmp[0] << 8) | tmp[1]);
	*result = rr;
}

#endif /* I2C_H_ */
