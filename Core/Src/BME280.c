/*
 * BME280.c
 *
 *  Created on: Aug 1, 2024
 *      Author: Andreas Cnaus
 */

#include "BME280.h"

// Temperature compensation parameters
uint16_t dig_T1;
int16_t  dig_T2;
int16_t  dig_T3;
// Pressure compensation parameters
uint16_t dig_P1;
int16_t	 dig_P2;
int16_t	 dig_P3;
int16_t	 dig_P4;
int16_t	 dig_P5;
int16_t	 dig_P6;
int16_t	 dig_P7;
int16_t	 dig_P8;
int16_t	 dig_P9;
// Humidity compensation parameters
uint8_t  dig_H1;
int16_t  dig_H2;
uint8_t  dig_H3;
int16_t  dig_H4;
int16_t  dig_H5;
int8_t   dig_H6;

BME280_S32_t t_fine = 0;	// carries fine temperature

/*
 * @brief Initialize the calibration coefficients with the data from the sensor memory
 * and configure the measuring sensor.
 * In addition, the i2c handle is stored in the device for future quick access.
 *
 * @param pointer to device structure
 * @param HAL-I2C handle
 * @retval BME280_CMD_OK by success, error code else
 */
BME280_cmd_res BME280_init(BME280 *dev, I2C_HandleTypeDef *i2c1)
{
	// check input parameters
	if (i2c1 == NULL) {
		return BME280_ERROR_INVALID_INPUT_PARAM;
	}

	// initialize device structure
	dev->hi2c1 = i2c1;
	dev->adc_T = 0;
	dev->adc_P = 0;
	dev->adc_H = 0;
	dev->T = 0;
	dev->P = 0;
	dev->H = 0;

	HAL_StatusTypeDef status = HAL_OK;
	uint8_t err_num = 0;
	uint8_t reg_data = 0;

	// read and verify the chip-Id
	status = BME280_read_register(dev->hi2c1, BME280_REG_ID, &reg_data);

	if (reg_data != BME280_CHIP_ID) return BME280_ERROR_WRONG_DEVID;

	/* Read calibration data */
	// temperature and pressure calibration data
	const uint8_t nof_tp_data = BME280_REG_CALIB25 - BME280_REG_CALIB00 + 1;
	uint8_t calib0[nof_tp_data];	// calibration data from register: 0x88 to 0xA1
	uint8_t reg_start_addrs = BME280_REG_CALIB00;

	status = BME280_read_registers(dev->hi2c1, reg_start_addrs, calib0, nof_tp_data);
    err_num += (status != HAL_OK);

    // humidity calibration data
    const uint8_t nof_h_data = NOF_H_CALIB_DATA;
    uint8_t calib1[nof_h_data];	// calibration data from register: 0xE1 to 0xE8
    reg_start_addrs = BME280_REG_CALIB26;

    status = BME280_read_registers(dev->hi2c1, reg_start_addrs, calib1, nof_h_data);
    err_num += (status != HAL_OK);

    if (err_num != 0) return BME280_ERROR_CALIB_DATA_READ;	// fatal error, return

    /* Create compensation parameters */
    // Temperature compensation parameters
    dig_T1 = (calib0[1] << 8 | calib0[0]);
    dig_T2 = (calib0[3] << 8 | calib0[2]);
    dig_T3 = (calib0[5] << 8 | calib0[4]);

    // Pressure compensation parameters
    dig_P1 = (calib0[7] << 8 | calib0[6]);
    dig_P2 = (calib0[9] << 8 | calib0[8]);
    dig_P3 = (calib0[11] << 8 | calib0[10]);
    dig_P4 = (calib0[13] << 8 | calib0[12]);
    dig_P5 = (calib0[15] << 8 | calib0[14]);
    dig_P6 = (calib0[17] << 8 | calib0[16]);
    dig_P7 = (calib0[19] << 8 | calib0[18]);
    dig_P8 = (calib0[21] << 8 | calib0[20]);
    dig_P9 = (calib0[23] << 8 | calib0[22]);

    // Humidity compensation parameters
    dig_H1 = calib0[25];
    dig_H2 = (calib1[1] << 8 | calib1[0]);
    dig_H3 = calib1[2];
    dig_H4 = (calib1[3] << 4 | (calib1[4] & 0x0F));
    dig_H5 = (calib1[5] << 4 | (calib1[4] & 0xF0) >> 4);
    dig_H6 = calib1[4];

    /* Set device and measurement configuration */
    // Device configuration
    MBE280_config_reg conf_reg = {0};
    conf_reg.spi3w_en = 0; // I2C interface is used
    conf_reg.filter = BME280_FILTER_OFF;	// IR-filter inactive
    conf_reg.t_sb = BME280_TSB_T1000MS;	// standby time=1s in normal mode operation
    status = BME280_write_register(dev->hi2c1, BME280_REG_CONFIG, &conf_reg.val);
	err_num += (status != HAL_OK);

	// humidity measurement configuration
	MBE280_ctrl_hum_reg ctrl_hum_reg = {0};
	ctrl_hum_reg.osrs_h = BME280_OS1;	// no oversampling for humidity
	status = BME280_write_register(dev->hi2c1, BME280_REG_CTRL_HUM, &ctrl_hum_reg.val);
	err_num += (status != HAL_OK);

	// temperature, pressure measurement and operation mode configuration
	BME280_ctrl_meas_reg ctrl_meas_reg = {0};
	ctrl_meas_reg.mode = BME280_MODE_NORMAL;	// the sensor alternates between active measurements and standby periods
	ctrl_meas_reg.osrs_p = BME280_OS1;	// no oversampling for pressure
	ctrl_meas_reg.osrs_t = BME280_OS1;	// no oversampling for temperature
    status = BME280_write_register(dev->hi2c1, BME280_REG_CTRL_MEAS, &ctrl_meas_reg.val);
	err_num += (status != HAL_OK);

	// error handling
	if (err_num == 1) return BME280_ERROR_DEV_CONFIG;
	else if (err_num > 1) return BME280_ERROR_MEAS_CONFIG;

	return BME280_CMD_OK;
}

/*
 * @brief Read the environmental measurement data (temperature, pressure, humidity)
 * and store these raw values ​​in the given device structure
 *
 * @param pointer to device structure
 * @retval BME280_CMD_OK by success, error code else
 */
BME280_cmd_res BME280_read_env_data(BME280 *dev)
{
	// check input parameters
	if (dev == NULL || dev->hi2c1 == NULL) {
		return BME280_ERROR_INVALID_INPUT_PARAM;
	}

	// read measurement data from registers
	HAL_StatusTypeDef status = HAL_OK;
	uint8_t reg_start_addrs = BME280_REG_PRESS_MSB;
	const uint8_t nof_meas_data = BME280_REG_HUM_LSB - BME280_REG_PRESS_MSB + 1 ;
	uint8_t meas_data[nof_meas_data];
	status = BME280_read_registers(dev->hi2c1, reg_start_addrs, &meas_data[0], nof_meas_data);

	if (status != HAL_OK) return BME280_ERROR_MEAS_DATA_READ;	// fatal error, return

	dev->adc_P = (((uint32_t)meas_data[0] << 12) | ((uint32_t)meas_data[1] << 4) | ((meas_data[2] & 0xF0) >> 4));	// raw pressure data
	dev->adc_T = (((uint32_t)meas_data[3] << 12) | ((uint32_t)meas_data[4] << 4) | ((meas_data[5] & 0xF0) >> 4));	// raw temperature data
	dev->adc_H = ((uint32_t)meas_data[6] << 8) | (meas_data[7]);	// raw humidity data

	// Compensate raw measurement data
	dev->P = BME280_compensate_P_int64(dev->adc_P);
	dev->T = BME280_compensate_T_int32(dev->adc_T);
	dev->H = BME280_compensate_H_int32(dev->adc_H);

	return BME280_CMD_OK;
}

/*
 * @brief Returns pressure in Pa as unsigned 32 bit integer in @Q24.8 format
 * (24 integer bits and 8 fractional bits).
 *
 * @param un-compensated pressure data as 32bit integer
 * @retval Output value of "24674867" represents 24674867/256 = 96386.2Pa = 963.862hPa
 */
BME280_U32_t BME280_compensate_P_int64(BME280_S32_t adc_P)
{
	BME280_S64_t var1, var2, p;
	var1 = ((BME280_S64_t)t_fine) - 128000;
	var2 = var1 * var1 * (BME280_S64_t)dig_P6;
	var2 = var2 + ((var1*(BME280_S64_t)dig_P5)<<17);
	var2 = var2 + (((BME280_S64_t)dig_P4)<<35);
	var1 = ((var1 * var1 * (BME280_S64_t)dig_P3)>>8) + ((var1 * (BME280_S64_t)dig_P2)<<12);
	var1 = (((((BME280_S64_t)1)<<47)+var1))*((BME280_S64_t)dig_P1)>>33;

	if (var1 == 0) {
		return 0; // avoid exception caused by division by zero
	}

	p = 1048576- adc_P;
	p = (((p<<31)- var2)*3125)/var1;
	var1 = (((BME280_S64_t)dig_P9) * (p>>13) * (p>>13)) >> 25;
	var2 = (((BME280_S64_t)dig_P8) * p) >> 19;
	p = ((p + var1 + var2) >> 8) + (((BME280_S64_t)dig_P7)<<4);
	return ( BME280_U32_t)p;
}

/*
 * @brief Returns temperature in DegC, resolution is 0.01 DegC.
 *
 * @param un-copmensated temperature data as 32bit integer
 * @retval Output value of "5123" equals to 5123/100 = 51.23 DegC
 */
BME280_S32_t BME280_compensate_T_int32(BME280_S32_t adc_T)
{
	BME280_S32_t var1, var2, T;
	var1 = ((((adc_T >> 3) - ((BME280_S32_t)dig_T1 << 1))) * ((BME280_S32_t)dig_T2)) >> 11;
	var2 = (((((adc_T >> 4) - ((BME280_S32_t)dig_T1)) - ((adc_T >> 4) - ((BME280_S32_t)dig_T1))) >> 12) * ((BME280_S32_t)dig_T3)) >> 14;
	t_fine = var1 + var2;
	T = (t_fine * 5 + 128) >> 8;
	return T;
}

/* @brief Returns humidity in %RH as unsigned 32 bit integer in Q22.10 format
 *  (22 integer and 10 fractional bits).
 *
 * @param  un-copmensated humidity data as 32bit integer
 * @retval Output value of “47445” represents 47445/1024 = 46.333 %RH
*/
BME280_U32_t BME280_compensate_H_int32(BME280_S32_t adc_H)
{
	BME280_S32_t v_x1_u32r;
	v_x1_u32r = (t_fine - ((BME280_S32_t)76800));
	v_x1_u32r = (((((adc_H << 14) - (((BME280_S32_t)dig_H4) << 20) - (((BME280_S32_t)dig_H5) *
	v_x1_u32r)) + ((BME280_S32_t)16384)) >> 15) * (((((((v_x1_u32r *
	((BME280_S32_t)dig_H6)) >> 10) * (((v_x1_u32r * ((BME280_S32_t)dig_H3)) >> 11) +
	((BME280_S32_t)32768))) >> 10) + ((BME280_S32_t)2097152)) * ((BME280_S32_t)dig_H2) +
	8192) >> 14));
	 v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) *
	((BME280_S32_t)dig_H1)) >> 4));
	 v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
	 v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
	 return ( BME280_U32_t)(v_x1_u32r>>12);
}

/**
  * @brief  Read one byte of data from the given register address
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  reg_addrs Internal memory address
  * @param  data Pointer to data buffer
  * @retval HAL status
  */
HAL_StatusTypeDef BME280_read_register(I2C_HandleTypeDef *hi2c, uint8_t reg_addrs, uint8_t *data)
{
	return HAL_I2C_Mem_Read(hi2c, BME280_I2C_ADDR, reg_addrs, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);
}

/**
  * @brief  Read N byte of data starting from a given register address
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  reg_addrs Internal memory address
  * @param  pData Pointer to data buffer
  * @param  length number of bytes to be read from the memory
  * @retval HAL status
  */
HAL_StatusTypeDef BME280_read_registers(I2C_HandleTypeDef *hi2c, uint8_t reg_addrs, uint8_t *data, uint8_t length)
{
	return HAL_I2C_Mem_Read(hi2c, BME280_I2C_ADDR, reg_addrs, I2C_MEMADD_SIZE_8BIT, data, length, HAL_MAX_DELAY);
}

/**
  * @brief  Write one byte of data to the given memory address
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  reg_addrs Internal memory address
  * @param  data Pointer to data buffer
  * @retval HAL status
  */
HAL_StatusTypeDef BME280_write_register(I2C_HandleTypeDef *hi2c, uint8_t reg_addrs, uint8_t *data)
{
	return HAL_I2C_Mem_Write(hi2c, BME280_I2C_ADDR, reg_addrs, I2C_MEMADD_SIZE_8BIT, data, 1,HAL_MAX_DELAY);
}

