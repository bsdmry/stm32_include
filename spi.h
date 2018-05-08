#ifndef SPI_H_
#define SPI_H_

#ifdef STM32F10X
#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_spi.h"
#endif

#define MOSI GPIO_Pin_7
#define MISO GPIO_Pin_6
#define SCK GPIO_Pin_5
#define CS GPIO_Pin_4


GPIO_InitTypeDef spi_pins;
SPI_InitTypeDef spi;
void hw_spi_init(){
	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1,ENABLE);
	// MISO + SCLK + MOSI
	spi_pins.GPIO_Pin   = MOSI | MISO |  SCK;
	spi_pins.GPIO_Mode  = GPIO_Mode_AF_PP; //Alternate function
	spi_pins.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &spi_pins);

    // CS
    spi_pins.GPIO_Pin   = CS;
    spi_pins.GPIO_Mode  = GPIO_Mode_Out_PP;
    spi_pins.GPIO_Speed = GPIO_Speed_50MHz;

    GPIO_Init(GPIOA, &spi_pins);
    GPIO_SetBits(GPIOA, CS);
    //GPIO_PinRemapConfig(GPIO_Remap_SPI1, ENABLE); //ENABLE ALTERNATE FUNCTION
    SPI_I2S_DeInit(SPI1);

    SPI_StructInit(&spi);
    spi.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    spi.SPI_Mode = SPI_Mode_Master;
    spi.SPI_DataSize = SPI_DataSize_8b;
    spi.SPI_CPOL = SPI_CPOL_Low;
    spi.SPI_CPHA = SPI_CPHA_1Edge;
    spi.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;
    spi.SPI_NSS = SPI_NSS_Soft;
    spi.SPI_FirstBit = SPI_FirstBit_MSB;
    spi.SPI_CRCPolynomial = 7;
    SPI_Init(SPI1, &spi);

    SPI_Cmd(SPI1, ENABLE);
    SPI_NSSInternalSoftwareConfig(SPI1, SPI_NSSInternalSoft_Set);
}

void wait_miso(void){
	while(GPIO_ReadInputDataBit(GPIOA, MISO));
}

void cs_l(void){
	GPIO_ResetBits(GPIOA, CS);
}
void cs_h(void){
	GPIO_SetBits(GPIOA, CS);
}

uint16_t spi_transaction(uint16_t data){
	  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
	  SPI_I2S_SendData(SPI1, data);
	  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
	  return SPI_I2S_ReceiveData(SPI1);
}
//-=---------------------------------
uint16_t spi_read16(uint16_t data){
	  uint16_t reply;
	  cs_l();
	  reply = spi_transaction(data);
	  cs_h();
	  return reply;
}

void spi_write16(uint16_t data){
	spi_read16(data);
}

uint8_t spi_read8(uint8_t data){
	  uint8_t reply;
	  cs_l();
	  reply = (uint8_t)spi_transaction((uint16_t)data);
	  cs_h();
	  return reply;
}

uint8_t spi_write8(uint8_t data){
	  spi_read8(data);
}
//--------------------------------
uint16_t spi_write_reg16(uint16_t reg, uint16_t data){
	  uint16_t reply;
	  cs_l();
	  spi_transaction(reg);
	  reply = spi_transaction(data);
	  cs_h();
	  return reply;
}

uint8_t spi_write_reg8(uint8_t reg, uint8_t data){
	uint8_t reply;
	cs_l();
	spi_transaction((uint16_t)reg);
	reply = (uint8_t)spi_transaction((uint16_t)data);
	cs_h();
	return reply;
}


uint8_t spi_read_reg8(uint8_t reg){
	return spi_write_reg8(reg, 0xFF);
}

uint8_t spi_read_reg16(uint16_t reg){
	return spi_write_reg16(reg, 0xFFFF);
}

#endif /* SPI_H_ */
