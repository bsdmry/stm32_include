/*
 * bme280.h
 *
 *  Created on: 21 апр. 2018 г.
 *      Author: igor
 */

#ifndef BME280_H_
#define BME280_H_

#define BME280_I2C_PORT I2C2 // I2C port where the BME280 connected
// All possible I2C device address values
#define BME280_ADDR_G (uint8_t)0x76 // I2C address when SDO connected to GND
#define BME280_ADDR_V (uint8_t)0x77 // I2C address when SDO connected to VDDIO

#define BME280_ADDR BME280_ADDR_V

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

void read_bme280_calibration(void){
	uint8_t cal1[24];
	uint8_t cal2[8];
	I2C_read(BME280_ADDR, BME280_REG_CALIB00, cal1, 24);
	I2C_read(BME280_ADDR, BME280_REG_CALIB26, cal2, 7);
	I2C_read(BME280_ADDR, BME280_REG_CALIB25, &H1, 1);

	T1 = (((uint16_t)cal1[0] << 8) | cal1[1]);
	T2 = (int16_t)(((uint16_t)cal1[2] << 8) | cal1[3]);
	T2 = (int16_t)(((uint16_t)cal1[4] << 8) | cal1[5]);

	P1 = (((uint16_t)cal1[6] << 8) | cal1[7]);
	P2 = (int16_t)(((uint16_t)cal1[8] << 8) | cal1[9]);
	P3 = (int16_t)(((uint16_t)cal1[10] << 8) | cal1[11]);
	P4 = (int16_t)(((uint16_t)cal1[12] << 8) | cal1[13]);
	P5 = (int16_t)(((uint16_t)cal1[14] << 8) | cal1[15]);
	P6 = (int16_t)(((uint16_t)cal1[16] << 8) | cal1[17]);
	P7 = (int16_t)(((uint16_t)cal1[18] << 8) | cal1[19]);
	P8 = (int16_t)(((uint16_t)cal1[20] << 8) | cal1[21]);
	P9 = (int16_t)(((uint16_t)cal1[22] << 8) | cal1[23]);

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

	// Send 'press_msb' register address
	I2C_write_byte(BME280_ADDR, BME280_REG_PRESS_MSB);

	// Read the 'press', 'temp' and 'hum' registers
	if (I2Cx_Read(BME280_I2C_PORT,&buf[0],8,BME280_ADDR)) {
		*UP = (int32_t)((buf[0] << 12) | (buf[1] << 4) | (buf[2] >> 4));
		*UT = (int32_t)((buf[3] << 12) | (buf[4] << 4) | (buf[5] >> 4));
		*UH = (int32_t)((buf[6] <<  8) |  buf[7]);

	}
}
#endif /* BME280_H_ */
