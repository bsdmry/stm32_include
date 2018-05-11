/*
 * lcd5110.h
 *
 *  Created on: 7 мая 2018 г.
 *      Author: igor
 */

#ifndef LCD5110_H_
#define LCD5110_H_
#include <stdio.h>
#ifdef STM32F00X
#include "stm32f0xx.h"
#include "stm32f0xx_rcc.h"
#include "stm32f0xx_gpio.h"
#endif

#ifdef STM32F10X
#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#endif
#include "spi.h"
#include "lcd5110_font.h"
#define LCD_COMMAND 0
#define LCD_DATA 1

#define LCD_SETYADDR 0x40
#define LCD_SETXADDR 0x80
#define LCD_DISPLAY_BLANK 0x08
#define LCD_DISPLAY_NORMAL 0x0C
#define LCD_DISPLAY_ALL_ON 0x09
#define LCD_DISPLAY_INVERTED 0x0D
#define LCD_WIDTH 84
#define LCD_HEIGHT 48
#define LCD_SIZE LCD_WIDTH * LCD_HEIGHT / 8
#define RST_PIN GPIO_Pin_0
#define DC_PIN GPIO_Pin_1

typedef struct {
        uint8_t** values;
        uint8_t size;
        uint8_t line_width;
        uint8_t x_pos;
        uint8_t y_pos;
        uint8_t wr_index;
} Bargraph;

Bargraph* bargraph_cfg(uint8_t bg_size, uint8_t bg_x, uint8_t bg_y, uint8_t width){
        Bargraph* bg = malloc(sizeof(Bargraph));
        bg->size = bg_size;
        bg->x_pos = bg_x;
        bg->y_pos = bg_y;
        bg->wr_index = 0;
        if ((bg_size*width) <=  LCD_WIDTH){
                bg->line_width = width;
        }else{
                bg->line_width = 1;
        }
        bg->values = malloc(sizeof(uint8_t*)*bg_size);
        return bg;
}


void bargraph_add(Bargraph* bg, uint8_t data){
		if bg->wr_index = (bg->size -1){
			for (uint8_t i = 1; i < (bg->size -1); i++) {
				bg->values[i - 1] = bg->values[i];
			}
			bg->values[(bg->size -1)] = data;
		}else{
			bg->values[bg->wr_index] = data;
			bg->wr_index++;
		}
}



void init_lcd_pins(void){
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	GPIO_InitTypeDef lcd_pins;
	lcd_pins.GPIO_Pin   = RST_PIN | DC_PIN; //RST | DC
	lcd_pins.GPIO_Mode  = GPIO_Mode_Out_PP;
	lcd_pins.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &lcd_pins);
}

void lcd5110_write(uint8_t data, uint8_t mode){
	if(mode == LCD_COMMAND){
		GPIO_ResetBits(GPIOB, DC_PIN);
		spi_write8(data);
	}else{
		GPIO_SetBits(GPIOB, DC_PIN);
		spi_write8(data);
	}
}

void lcd5110_clr(void){
	  for(int i = 0; i < 504; i++){
		  lcd5110_write(0x00, LCD_DATA);
	}
}

void lcd5110_writeChar(char c){
  for(int i = 0; i < 6; i++){
	  lcd5110_write(ASCII[c - 0x20][i], LCD_DATA);
  }
}

void lcd5110_goXY(uint8_t x, uint8_t y){
	lcd5110_write(0x80 | x, LCD_COMMAND); //Column.
	lcd5110_write(0x40 | y, LCD_COMMAND); //Row.
  }

void lcd5110_print(unsigned char *str, uint8_t size, uint8_t x, uint8_t y, uint8_t font_x2){
	if(font_x2){
		lcd5110_goXY(x, y);
		for(int i = 0; i < size; i++){
			  for(int b = 0; b < 10; b++){
				  lcd5110_write(ASCII2[str[i] - 0x20][b], LCD_DATA);
			  };
		}
		lcd5110_goXY(x, y+1);
		for(int i = 0; i < size; i++){
			  for(int b = 10; b < 20; b++){
				  lcd5110_write(ASCII2[str[i] - 0x20][b], LCD_DATA);
			  };
		}
	}else{
		lcd5110_goXY(x, y);
		for(int i = 0; i < size; i++){
			lcd5110_writeChar(str[i]);
		}
	}
  }

void bargraph_show(Bargraph* bg){
	lcd5110_goXY(bg->x_pos, bg->y_pos);
	for (uint8_t i = 0; i < bg->size; i++) {
		lcd5110_write(bg->values[i], LCD_DATA);
	}
}

void lcd5110_init(void){
	 GPIO_ResetBits(GPIOB, RST_PIN);
	 GPIO_SetBits(GPIOB, RST_PIN);

	 lcd5110_write(0x21, LCD_COMMAND); //LCD extended commands.
	 //lcd5110_write(0xB8, LCD_COMMAND); //set LCD Vop(Contrast).
	 lcd5110_write(0xb2, LCD_COMMAND); //set LCD Vop(Contrast).
	 lcd5110_write(0x04, LCD_COMMAND); //set temp coefficent.
	 lcd5110_write(0x14, LCD_COMMAND); //LCD bias mode 1:40.
	 lcd5110_write(0x20, LCD_COMMAND); //LCD basic commands.
	 lcd5110_write(LCD_DISPLAY_NORMAL, LCD_COMMAND); //LCD normal.
	 lcd5110_clr();
}

#endif /* LCD5110_H_ */
