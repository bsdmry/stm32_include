/*
 * pcf8574.h

 *
 *  Created on: 20 апр. 2018 г.
 *      Author: igor
 */
#include <stdio.h>
#include <string.h>
#include "i2c.h"
#include "delay.h"
#ifndef PCF8574_H_
#define PCF8574_H_
//#define PCF_ADDR 0x7E
#define PCF_ADDR 0x3F
#define LCD_BACKLIGHT 0x08
#define ENABLE 0x04
#define LCD_WIDTH 20   // Maximum characters per line
#define LCD_CHR  1 // Mode - Sending command
#define LCD_CMD  0 // Mode - Sending data
#define LCD_LINE_1  0x80 // LCD RAM address for the 1st line
#define LCD_LINE_2  0xC0 // LCD RAM address for the 2nd lin

void lcd_byte(uint8_t byte, uint8_t flag){
        // Send byte to data pins
        // bits = the data
        // mode = 1 for character
        //        0 for command
        uint8_t bits_high = (flag | (byte & 0xF0) | LCD_BACKLIGHT);
        uint8_t bits_low = (flag | ((byte<<4) & 0xF0) | LCD_BACKLIGHT);
        I2C_write_byte(PCF_ADDR, bits_high);
        I2C_write_byte(PCF_ADDR, (bits_high | ENABLE));
        I2C_write_byte(PCF_ADDR, (bits_high & ~ENABLE));
        // Low bits
        I2C_write_byte(PCF_ADDR, bits_low);
        I2C_write_byte(PCF_ADDR, (bits_low | ENABLE));
        I2C_write_byte(PCF_ADDR, (bits_low & ~ENABLE));
}

void lcd_init(void){
        lcd_byte(0x33, LCD_CMD); // 110011 Initialise
        lcd_byte(0x32, LCD_CMD); // 110010 Initialise
        lcd_byte(0x06, LCD_CMD); // 000110 Cursor move direction
        lcd_byte(0x0C, LCD_CMD); // 001100 Display On,Cursor Off, Blink Off
        lcd_byte(0x28, LCD_CMD); // 101000 Data length, number of lines, font size
        lcd_byte(0x01, LCD_CMD); // 000001 Clear display
        timer_sleep(5);
}


void lcd_string (unsigned char *msg, uint8_t num,  uint8_t pos){
        // Send string to display
        unsigned char message[16];
        memset(message, 0x20, sizeof(message)); //clears buffer
        memcpy(message, msg, num);
        lcd_byte(pos, LCD_CMD);
        for (uint8_t p = 0; p <= 15  ; p++){
                lcd_byte((uint8_t)message[p], LCD_CHR);
        }
}

#endif /* PCF8574_H_ */
