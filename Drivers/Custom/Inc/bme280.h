/*
 * BME280.h
 *
 *  Created on: Aug 1, 2024
 *      Author: Andreas Cnaus
 */

#ifndef INC_BME280_H_
#define INC_BME280_H_


#include <stdint.h>
#include "stm32wbxx_hal.h"	/* needed for IC2 */

/* Definitions */
#define BME280_I2C_ADDR		   (0x76 << 1)	//  7-bit I2C device address (shifted left by 1)
#define BME280_CHIP_ID 		 	0x60

// Conversion factors from integer to floating point number
#define kP						256		// pressure conversation factor
#define kT	 					100		// temperature conversation factor
#define kH						1024	// pressure conversation factor

/* Register addresses */
#define BME280_REG_HUM_LSB		0xFE
#define BME280_REG_HUM_MSB		0xFD
#define BME280_REG_TEMP_XLSB	0xFC
#define BME280_REG_TEMP_LSB		0xFB
#define BME280_REG_TEMP_MSB		0xFA
#define BME280_REG_PRESS_XLSB	0xF9
#define BME280_REG_PRESS_LSB	0xF8
#define BME280_REG_PRESS_MSB	0xF7
#define BME280_REG_CONFIG		0xF5
#define BME280_REG_CTRL_MEAS	0xF4
#define BME280_REG_STATUS		0xF3	// read only
#define BME280_REG_CTRL_HUM		0xF2
#define BME280_REG_RESET		0xE0	// write only
#define BME280_REG_ID			0xD0	// read only

// Temperature and Pressure calibration data
#define BME280_REG_CALIB00		0x88
#define BME280_REG_CALIB25	    0xA1
// Humidity calibration data
#define BME280_REG_CALIB26		0xE1
#define BME280_REG_CALIB41		0xF0

#define NOF_T_CALIB_DATA	    6
#define NOF_P_CALIB_DATA	    20
#define NOF_H_CALIB_DATA		7

/* Register definitions */
typedef union {	// device configuration register
	uint8_t val;
	struct {
		uint8_t spi3w_en : 1;	// enables 3-wire SPI interface when set to 1
		uint8_t bit1     : 1;	// reserved
		uint8_t filter   : 3;	// controls IIR-filter
		uint8_t t_sb	 : 3;	// controls standby time
	};
} MBE280_config_reg;

typedef union {	// measurement control register
	uint8_t val;
	struct {
		uint8_t mode 	: 2;	// controls sensor mode of the device
		uint8_t osrs_p	: 3;	// controls oversampling of pressure data
		uint8_t osrs_t	: 3;	// controls oversampling of temperature data
	};
} BME280_ctrl_meas_reg;

typedef union {	// humidity control register
	uint8_t val;
	struct {
		uint8_t osrs_h 	: 3;	// controls oversampling of humidity data
	};
} MBE280_ctrl_hum_reg;


/* Settings definitions */
typedef enum {	// oversampling settings for pressure, humidity and temperature
	BME280_OS_SKIP 	= 0,	// skipped (output set to 0x8000)
	BME280_OS1		= 1,	// oversampling x1
	BME280_OS2		= 2,	// oversampling x2
	BME280_OS4		= 3,	// oversampling x4
	BME280_OS8		= 4,	// oversampling x8
	BME280_OS16		= 5		// oversampling x16
} BME280_os_set;

typedef enum {	// standby time settings in milliseconds
	BME280_TSB_0MS5 	= 0,	// 0.5ms
	BME280_TSB_T62MS5 	= 1,	// 62.5ms
	BME280_TSB_T125MS	= 2,	// 125ms
	BME280_TSB_T250MS	= 3,	// 250ms
	BME280_TSB_T500MS	= 4,	// 500ms
	BME280_TSB_T1000MS	= 5, 	// 1000ms
	BME280_TSB_10MS		= 6,	// 10ms
	BME280_TSB_20MS		= 7		// 20ms
} BME280_tsb_set;

typedef enum {	// filter coefficient settings
	BME280_FILTER_OFF		= 0, 	// filter off
	BME280_FILTER_N2		= 1,	// number of filter coefficients = 2
	BME280_FILTER_N4		= 2,	// number of filter coefficients = 4
	BME280_FILTER_N8		= 3,	// number of filter coefficients = 8
	BME280_FILTER_N16		= 4,	// number of filter coefficients = 16
} BME280_filter_set;

typedef enum {	// operation mode
	BME280_MODE_SLEEP 	= 0,	// sleep mode, the sensor doesn’t perform measurements and remains in a low-power state
	BME280_MODE_FORCED  = 1,	// forced mode, the sensor performs a single measurement cycle and then returns to sleep mode
	BME280_MODE_NORMAL  = 3     // normal mode, the sensor alternates between active measurements and standby periods
} BME280_mode_set;

/* Data type definitions */
typedef int32_t 	BME280_S32_t;
typedef uint32_t	BME280_U32_t;
typedef int64_t		BME280_S64_t;

/* Command result definitions */
typedef enum {
	BME280_CMD_OK,
	BME280_ERROR_INVALID_INPUT_PARAM,
	BME280_ERROR_WRONG_DEVID,
	BME280_ERROR_CALIB_DATA_READ,
	BME280_ERROR_DEV_CONFIG,
	BME280_ERROR_MEAS_CONFIG,
	BME280_ERROR_MEAS_DATA_READ
} BME280_cmd_res;

typedef struct {	// Device Structure
	I2C_HandleTypeDef *hi2c1;
	BME280_S32_t adc_T;	// raw temperature data
	BME280_S32_t adc_P;	// raw pressure data
	BME280_S32_t adc_H;	// raw humidity data

	BME280_U32_t P;	// compensated pressure data
	BME280_S32_t T;	// compensated temperature data
	BME280_U32_t H;	// compensated humidity data
} BME280;


/* Function definitions */
BME280_cmd_res BME280_init(BME280 *dev, I2C_HandleTypeDef *hi2c);

BME280_cmd_res BME280_read_env_data(BME280 *dev);
BME280_U32_t BME280_compensate_P_int64(BME280_S32_t adc_P);
BME280_S32_t BME280_compensate_T_int32(BME280_S32_t adc_T);
BME280_U32_t BME280_compensate_H_int32(BME280_S32_t adc_H);

/* low level functions */
HAL_StatusTypeDef BME280_read_register(I2C_HandleTypeDef *hi2c, uint8_t reg_addrs, uint8_t *data);
HAL_StatusTypeDef BME280_read_registers(I2C_HandleTypeDef *hi2c, uint8_t reg_addrs, uint8_t *data, uint8_t length);
HAL_StatusTypeDef BME280_write_register(I2C_HandleTypeDef *hi2c, uint8_t reg_addrs, uint8_t *data);

#endif /* INC_BME280_H_ */
