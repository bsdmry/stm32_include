/*
 * bme280.h
 *
 *  Created on: 21 апр. 2018 г.
 *      Author: https://github.com/LonelyWolf/stm32
 */

#ifndef BME280_H_
#define BME280_H_
#include "delay.h"
#include "i2c.h"

#define BME280_I2C_PORT I2C2 // I2C port where the BME280 connected
// All possible I2C device address values
#define BME280_ADDR_G (uint8_t)0x76 // I2C address when SDO connected to GND
#define BME280_ADDR_V (uint8_t)0x77 // I2C address when SDO connected to VDDIO

#define BME280_ADDR BME280_ADDR_G

// BME280 registers
#define BME280_REG_CALIB00              (uint8_t)0x88 // Calibration data calib00
#define BME280_REG_CALIB25              (uint8_t)0xA1 // Calibration data calib25
#define BME280_REG_CALIB26              (uint8_t)0xE1 // Calibration data calib26
#define BME280_REG_ID                   (uint8_t)0xD0 // Chip ID
#define BME280_REG_RESET                (uint8_t)0xE0 // Software reset control register
#define BME280_REG_CTRL_HUM             (uint8_t)0xF2 // Humidity measure control register
#define BME280_REG_STATUS               (uint8_t)0xF3 // Device status register
#define BME280_REG_CTRL_MEAS            (uint8_t)0xF4 // Pressure and temperature measure control register
#define BME280_REG_CONFIG               (uint8_t)0xF5 // Configuration register
#define BME280_REG_PRESS_MSB            (uint8_t)0xF7 // Pressure readings MSB
#define BME280_REG_PRESS_LSB            (uint8_t)0xF8 // Pressure readings LSB
#define BME280_REG_PRESS_XLSB           (uint8_t)0xF9 // Pressure readings XLSB
#define BME280_REG_TEMP_MSB             (uint8_t)0xFA // Temperature data MSB
#define BME280_REG_TEMP_LSB             (uint8_t)0xFB // Temperature data LSB
#define BME280_REG_TEMP_XLSB            (uint8_t)0xFC // Temperature data XLSB
#define BME280_REG_HUM_MSB              (uint8_t)0xFD // Humidity data MSB
#define BME280_REG_HUM_LSB 				(uint8_t)0xFE // Humidity data LSB

// BME280 register bits

// Software reset
#define BME280_SOFT_RESET_KEY           (uint8_t)0xB6

// Humidity oversampling control register (0xF2)
#define BME280_OSRS_H_MSK               (uint8_t)0x07 // 'osrs_h' mask
#define BME280_OSRS_H_SKIP              (uint8_t)0x00 // Skipped
#define BME280_OSRS_H_x1                (uint8_t)0x01 // x1
#define BME280_OSRS_H_x2                (uint8_t)0x02 // x2
#define BME280_OSRS_H_x4                (uint8_t)0x03 // x4
#define BME280_OSRS_H_x8                (uint8_t)0x04 // x8
#define BME280_OSRS_H_x16               (uint8_t)0x05 // x16

// Status register (0xF3)
#define BME280_STATUS_MSK               (uint8_t)0x09 // Mask to clear unused bits
#define BME280_STATUS_MEASURING         (uint8_t)0x08 // Status register bit 3 (conversion is running)
#define BME280_STATUS_IM_UPDATE         (uint8_t)0x01 // Status register bit 0 (NVM data being copied to image registers)

// Pressure and temperature control register (0xF4)
//   Temperature oversampling (osrs_t [7:5])
#define BME280_OSRS_T_MSK               (uint8_t)0xE0 // 'osrs_t' mask
#define BME280_OSRS_T_SKIP              (uint8_t)0x00 // Skipped
#define BME280_OSRS_T_x1                (uint8_t)0x20 // x1
#define BME280_OSRS_T_x2                (uint8_t)0x40 // x2
#define BME280_OSRS_T_x4                (uint8_t)0x60 // x4
#define BME280_OSRS_T_x8                (uint8_t)0x80 // x8
#define BME280_OSRS_T_x16               (uint8_t)0xA0 // x16
//   Pressure oversampling (osrs_p [4:2])
#define BME280_OSRS_P_MSK               (uint8_t)0x1C // 'osrs_p' mask
#define BME280_OSRS_P_SKIP              (uint8_t)0x00 // Skipped
#define BME280_OSRS_P_x1                (uint8_t)0x04 // x1
#define BME280_OSRS_P_x2                (uint8_t)0x08 // x2
#define BME280_OSRS_P_x4                (uint8_t)0x0C // x4
#define BME280_OSRS_P_x8                (uint8_t)0x10 // x8
#define BME280_OSRS_P_x16               (uint8_t)0x14 // x16
//   Sensor mode of the device (mode [1:0])
#define BME280_MODE_MSK                 (uint8_t)0x03 // 'mode' mask
#define BME280_MODE_SLEEP               (uint8_t)0x00 // Sleep mode
#define BME280_MODE_FORCED              (uint8_t)0x01 // Forced mode
#define BME280_MODE_NORMAL              (uint8_t)0x03 // Normal mode

// Configuration register: set rate, filter and interface options (0xF5)
//   Inactive duration in normal mode (t_sb [7:5])
#define BME280_STBY_MSK                 (uint8_t)0xE0 // 't_sb' mask
#define BME280_STBY_0p5ms               (uint8_t)0x00 // 0.5ms
#define BME280_STBY_62p5ms              (uint8_t)0x20 // 62.5ms
#define BME280_STBY_125ms               (uint8_t)0x40 // 125ms
#define BME280_STBY_250ms               (uint8_t)0x60 // 250ms
#define BME280_STBY_500ms               (uint8_t)0x80 // 500ms
#define BME280_STBY_1s                  (uint8_t)0xA0 // 1s
#define BME280_STBY_10ms                (uint8_t)0xC0 // 10ms
#define BME280_STBY_20ms                (uint8_t)0xE0 // 20ms
//   Time constant of the IIR filter (filter [4:2])
#define BME280_FILTER_MSK               (uint8_t)0x1C // 'filter' mask
#define BME280_FILTER_OFF               (uint8_t)0x00 // Off
#define BME280_FILTER_2                 (uint8_t)0x04 // 2
#define BME280_FILTER_4                 (uint8_t)0x08 // 4
#define BME280_FILTER_8                 (uint8_t)0x0C // 8
#define BME280_FILTER_16                (uint8_t)0x10 // 16

// Constant for Pascals to millimeters of mercury conversion
#define BME_MMHG_Q0_20 					(uint32_t)7865; // 0.00750061683 in Q0.20 format

	uint16_t T1, P1;
	int16_t  T2, T3, P2, P3, P4, P5, P6, P7, P8, P9, H2, H4, H5;
	uint8_t  H1, H3;
	int8_t H6;
	static int32_t t_fine;

void read_bme280_calibration(void){
	uint8_t cal1[24];
	uint8_t cal2[8];
	I2C_read(BME280_ADDR, BME280_REG_CALIB00, cal1, 24);
	I2C_read(BME280_ADDR, BME280_REG_CALIB26, cal2, 7);
	I2C_read(BME280_ADDR, BME280_REG_CALIB25, &H1, 1);

	T1 = (((uint16_t)cal1[1] << 8) | cal1[0]);
	T2 = (int16_t)(((uint16_t)cal1[3] << 8) | cal1[2]);
	T3 = (int16_t)(((uint16_t)cal1[5] << 8) | cal1[4]);

	P1 = (((uint16_t)cal1[7] << 8) | cal1[6]);
	P2 = (int16_t)(((uint16_t)cal1[9] << 8) | cal1[8]);
	P3 = (int16_t)(((uint16_t)cal1[11] << 8) | cal1[10]);
	P4 = (int16_t)(((uint16_t)cal1[13] << 8) | cal1[12]);
	P5 = (int16_t)(((uint16_t)cal1[15] << 8) | cal1[14]);
	P6 = (int16_t)(((uint16_t)cal1[17] << 8) | cal1[16]);
	P7 = (int16_t)(((uint16_t)cal1[19] << 8) | cal1[18]);
	P8 = (int16_t)(((uint16_t)cal1[21] << 8) | cal1[20]);
	P9 = (int16_t)(((uint16_t)cal1[23] << 8) | cal1[22]);

	H2 = (int16_t)((((int8_t)cal2[1]) << 8) | cal2[0]);
	H3 = cal2[2];
	H4 = (int16_t)((((int8_t)cal2[3]) << 4) | (cal2[4] & 0x0f));
	H5 = (int16_t)((((int8_t)cal2[5]) << 4) | (cal2[4] >> 4));
	H6 = (int8_t)cal2[6];
}

void bme280_read_all(int32_t *UT, int32_t *UP, int32_t *UH) {
	uint8_t buf[8];

	// Clear result values
	*UT = 0x80000;
	*UP = 0x80000;
	*UH = 0x8000;

	I2C_read(BME280_ADDR, BME280_REG_PRESS_MSB, buf, 8);
	*UP = (int32_t)((buf[0] << 12) | (buf[1] << 4) | (buf[2] >> 4));
	*UT = (int32_t)((buf[3] << 12) | (buf[4] << 4) | (buf[5] >> 4));
	*UH = (int32_t)((buf[6] <<  8) |  buf[7]);
}

void bme280_reset(void) {
	I2C_write8(BME280_ADDR, BME280_REG_RESET, BME280_SOFT_RESET_KEY);
}

uint8_t bm280_getVersion(void) {
	uint8_t version = 0;
	I2C_read8(BME280_ADDR, BME280_REG_ID, &version);
	return version;
}

// Set inactive duration in normal mode (Tstandby)
// input:
//   tsb - new inactive duration (one of BME280_STBY_x values)
void bme280_setStandby(uint8_t tsb) {
	uint8_t reg = 0;
	// Read the 'config' (0xF5) register and clear 'filter' bits
	I2C_read8(BME280_ADDR, BME280_REG_CONFIG, &reg);
	reg = reg & ~BME280_STBY_MSK;
	// Configure new standby value
	reg |= tsb & BME280_STBY_MSK;
	// Write value back to the register
	I2C_write8(BME280_ADDR, BME280_REG_CONFIG, reg);
}

// Set oversampling of temperature data
// input:
//   osrs - new oversampling value (one of BME280_OSRS_T_Xx values)
void bme280_setOSRST(uint8_t osrs) {
	uint8_t reg = 0;
	// Read the 'ctrl_meas' (0xF4) register and clear 'osrs_t' bits
	I2C_read8(BME280_ADDR, BME280_REG_CTRL_MEAS, &reg);
	reg = reg & ~BME280_OSRS_T_MSK;
	// Configure new oversampling value
	reg |= osrs & BME280_OSRS_T_MSK;
	// Write value back to the register
	I2C_write8(BME280_ADDR, BME280_REG_CTRL_MEAS, reg);
}

// Set oversampling of pressure data
// input:
//   osrs - new oversampling value (one of BME280_OSRS_P_Xx values)
void bme280_setOSRSP(uint8_t osrs) {
	uint8_t reg = 0;
	// Read the 'ctrl_meas' (0xF4) register and clear 'osrs_p' bits
	I2C_read8(BME280_ADDR, BME280_REG_CTRL_MEAS, &reg);
	reg = reg & ~BME280_OSRS_P_MSK;
	// Configure new oversampling value
	reg |= osrs & BME280_OSRS_P_MSK;
	// Write value back to the register
	I2C_write8(BME280_ADDR, BME280_REG_CTRL_MEAS, reg);
}

// Set oversampling of humidity data
// input:
//   osrs - new oversampling value (one of BME280_OSRS_H_Xx values)
void bme280_setOSRSH(uint8_t osrs) {
	uint8_t reg = 0;
	// Read the 'ctrl_hum' (0xF2) register and clear 'osrs_h' bits
	I2C_read8(BME280_ADDR, BME280_REG_CTRL_HUM, &reg);
	reg = reg & ~BME280_OSRS_H_MSK;
	// Configure new oversampling value
	reg |= osrs & BME280_OSRS_H_MSK;
	// Write value back to the register
	I2C_write8(BME280_ADDR, BME280_REG_CTRL_HUM, reg);

	// Changes to 'ctrl_hum' register only become effective after a write to 'ctrl_meas' register
	// Thus read a value of the 'ctrl_meas' register and write it back after write to the 'ctrl_hum'
	// Read the 'ctrl_meas' (0xF4) register
	I2C_read8(BME280_ADDR, BME280_REG_CTRL_MEAS, &reg);
	// Write back value of 'ctrl_meas' register to activate changes in 'ctrl_hum' register
	I2C_write8(BME280_ADDR, BME280_REG_CTRL_MEAS, reg);
}

// Set sensor mode of the BME280 chip
// input:
//   mode - new mode (BME280_MODE_SLEEP, BME280_MODE_FORCED or BME280_MODE_NORMAL)
void bme280_setMode(uint8_t mode) {
	uint8_t reg = 0;
	// Read the 'ctrl_meas' (0xF4) register and clear 'mode' bits
	I2C_read8(BME280_ADDR, BME280_REG_CTRL_MEAS, &reg);
	reg = reg & ~BME280_MODE_MSK;
	// Configure new mode
	reg |= mode & BME280_MODE_MSK;
	// Write value back to the register
	I2C_write8(BME280_ADDR, BME280_REG_CTRL_MEAS, reg);
}

// Calculate temperature from raw value, resolution is 0.01 degree
// input:
//   UT - raw temperature value
// return: temperature in Celsius degrees (value of '5123' equals '51.23C')
// note: code from the BME280 datasheet (rev 1.1)
int32_t bme280_calcT(int32_t UT) {
	t_fine  = ((((UT >> 3) - ((int32_t)T1 << 1))) * ((int32_t)T2)) >> 11;
	t_fine += (((((UT >> 4) - ((int32_t)T1)) * ((UT >> 4) - ((int32_t)T1))) >> 12) * ((int32_t)T3)) >> 14;

	return ((t_fine * 5) + 128) >> 8;
}

float bme280_calcTf(int32_t UT) {
	float v_x1,v_x2;

	v_x1 = (((float)UT) / 16384.0 - ((float)T1) / 1024.0) * ((float)T2);
	v_x2 = ((float)UT) / 131072.0 - ((float)T1) / 8192.0;
	v_x2 = (v_x2 * v_x2) * ((float)T3);
	t_fine = (uint32_t)(v_x1 + v_x2);

	return ((v_x1 + v_x2) / 5120.0);
}

// Calculate pressure from raw value using floats, resolution is 0.001 Pa
// input:
//   UP - raw pressure value
// return: pressure in Pa (value of '99158.968' represents 99158.968Pa)
// note: BME280_CalcT of BME280_CalcTf must be called before calling this function
// note: code from the BME280 datasheet (rev 1.1)
float bme280_calcPf(uint32_t UP) {
	float v_x1, v_x2, p;

	v_x1 = ((float)t_fine / 2.0) - 64000.0;
	v_x2 = v_x1 * v_x1 * ((float)P6) / 32768.0;
	v_x2 = v_x2 + v_x1 * ((float)P5) * 2.0;
	v_x2 = (v_x2 / 4.0) + (((float)P4) * 65536.0);
	v_x1 = (((float)P3) * v_x1 * v_x1 / 524288.0 + ((float)P2) * v_x1) / 524288.0;
	v_x1 = (1.0 + v_x1 / 32768.0) * ((float)P1);
	p = 1048576.0 - (float)UP;
	if (v_x1 == 0) return 0; // Avoid exception caused by division by zero
	p = (p - (v_x2 / 4096.0)) * 6250.0 / v_x1;
	v_x1 = ((float)P9) * p * p / 2147483648.0;
	v_x2 = p * ((float)P8) / 32768.0;
	p += (v_x1 + v_x2 + ((float)P7)) / 16.0;

	return (p * 0.00750061683);
}

// Calculate humidity from raw value using floats, resolution is 0.001 %RH
// input:
//   UH - raw humidity value
// return: humidity in %RH (value of '46.333' represents 46.333%RH)
// note: BME280_CalcT or BME280_CalcTf must be called before calling this function
// note: code from the BME280 datasheet (rev 1.1)
float bme280_calcHf(uint32_t UH) {
	float h;

	h = (((float)t_fine) - 76800.0);
	if (h == 0) return 0;
	h = (UH - (((float)H4) * 64.0 + ((float)H5) / 16384.0 * h));
	h = h * (((float)H2) / 65536.0 * (1.0 + ((float)H6) / 67108864.0 * h * (1.0 + ((float)H3) / 67108864.0 * h)));
	h = h * (1.0 - ((float)H1) * h / 524288.0);
	if (h > 100.0) {
		h = 100.0;
	} else if (h < 0.0) {
		h = 0.0;
	}

	return h;
}

void bme280_init(void){
	bme280_reset();
	timer_sleep(2000);
	bme280_setStandby(BME280_STBY_1s);
	bme280_setOSRST(BME280_OSRS_T_x4);
	bme280_setOSRSP(BME280_OSRS_P_x2);
	bme280_setOSRSH(BME280_OSRS_H_x1);

	bme280_setMode(BME280_MODE_NORMAL);
	read_bme280_calibration();
}

#endif /* BME280_H_ */
