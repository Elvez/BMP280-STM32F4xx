/*
 * baroLib.c
 *
 *  Created on: Jan 5, 2021
 *      Author: Pravesh Narayan
 */


#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "userHW.h"
#include "baroLib.h"
#include <stdbool.h>
#include <stdlib.h>

void baroInitDefault(bmpConfig params_)
{
#ifdef sleepMode
	params_->mode_ = MODE_SLEEP;
#endif

#ifdef forceMode
	params_->mode_ = MODE_FORCED;
#endif

#ifdef normalMode
	params_.mode_ = MODE_NORMAL;
#endif

	params_.filter_ = FILTER_OFF;
	params_.pressOS_ = STANDARD;
	params_.tempOS_ = STANDARD;
	params_.humOS_ = STANDARD;
	params_.standby_ = STANDBY_250;
}

static bool readReg16(BMP280_HandleTypedef *device_, uint8_t addr_, uint16_t *value_)
{
	uint16_t txBuffer_;
	uint8_t rxBuffer_[2];

	txBuffer_ = (device_->addr << 1);
	if(HAL_I2C_Mem_Read(device_->i2c, txBuffer_, addr_, 1, rxBuffer_, 2, 5000) == HAL_OK)
	{
		*value_ = (uint16_t)((rxBuffer_[1] << 8) | rxBuffer_[0]);
		return true;
	}
	else
		return false;
}

static inline int readData(BMP280_HandleTypedef *device_, uint8_t addr_, uint8_t *value_, uint8_t len_)
{
	uint16_t txBuffer_;

	txBuffer_ = (device_->addr << 1);

	if(HAL_I2C_Mem_Read(device_->i2c, txBuffer_, addr_, 1, value_, len_, 5000) == HAL_OK)
		return 0;
	else
		return 1;
}

static bool readCalibData(BMP280_HandleTypedef *device_)
{
	if(readReg16(device_, calibReg_, &device_->digT1)
			&& readReg16(device_, 0x8a, (uint16_t *) &device_->digT2)
			&& readReg16(device_, 0x8c, (uint16_t *) &device_->digT3)
			&& readReg16(device_, 0x8e, &device_->digP1)
			&& readReg16(device_, 0x90, (uint16_t *) &device_->digP2)
			&& readReg16(device_, 0x92, (uint16_t *) &device_->digP3)
			&& readReg16(device_, 0x94, (uint16_t *) &device_->digP4)
			&& readReg16(device_, 0x96, (uint16_t *) &device_->digP5)
			&& readReg16(device_, 0x98, (uint16_t *) &device_->digP6)
			&& readReg16(device_, 0x9a, (uint16_t *) &device_->digP7)
			&& readReg16(device_, 0x9c, (uint16_t *) &device_->digP8)
			&& readReg16(device_, 0x9e, (uint16_t *) &device_->digP9))
	{
		return true;
	}
	return false;
}

static bool readHumCalibData(BMP280_HandleTypedef *device_)
{
	uint16_t h4, h5;

	if(!readData(device_, 0xa1, &device_->digH1, 1)
			&& readReg16(device_, 0xe1, (uint16_t *) &device_->digH2)
			&& !readData(device_, 0xe3, &device_->digH3, 1)
			&& readReg16(device_, 0xe4, &h4)
			&& readReg16(device_, 0xe5, &h5)
			&& !readData(device_, 0xe7, (uint8_t *) &device_->digH6, 1)) {
		device_->digH4 = (h4 & 0x00ff) << 4 | (h4 & 0x0f00) >> 8;
		device_->digH5 = h5 >> 4;

		return true;
	}

	return false;
}

static int writeReg8(BMP280_HandleTypedef *device_, uint8_t addr_, uint8_t value_)
{
	uint16_t txBuffer_;

	txBuffer_ = (device_->addr << 1);

	if(HAL_I2C_Mem_Write(device_->i2c, txBuffer_, addr_, 1, &value_, 1, 10000) == HAL_OK)
		return false;
	else
		return true;
}

bool baroInit(BMP280_HandleTypedef *device_, bmpConfig *params_)
{
	if(device_->addr != baseAdd__ && device_->addr != baseAdd_)
	{
		sendMessage("Failure in base address\n\r");
		return false;
	}

	if(readData(device_, identity_, &device_->id, 1))
	{
		sendMessage("Failure in reading device ID\n\r");
		return false;
	}

	if(device_->id != chipID_)
	{
		sendMessage("Failure in device ID\n\r");
		return false;
	}

	// Soft reset.
	if(writeReg8(device_, resetAdd_, bmpReset_))
	{
		sendMessage("Failure in soft reset\n\r");
		return false;
	}

	// Wait until finished copying over the NVP data.
	while(1)
	{
		uint8_t status;

		if (!readData(device_, status_, &status, 1) && (status & 1) == 0)
			break;
	}

	if(!readCalibData(device_))
	{
		sendMessage("Failure in reading calibration data\n\r");
		return false;
	}

	if(device_->id == chipID_ && !readHumCalibData(device_))
	{
		sendMessage("Failure in reading calibration(hum) data\n\r");
		return false;
	}

	uint8_t config = (params_->standby_ << 5) | (params_->filter_ << 2);
	if(writeReg8(device_, config_, config))
	{
		return false;
	}

	if(params_->mode_ == MODE_FORCED)
	{
		params_->mode_ = MODE_SLEEP;  // initial mode for forced is sleep
	}

	uint8_t ctrl = (params_->tempOS_ << 5) | (params_->pressOS_ << 2) | (params_->mode_);
	if(writeReg8(device_, ctrlMeas_, ctrl))
	{
		return false;
	}

	return true;
}

bool baroForceMeas(BMP280_HandleTypedef *device_)
{
	uint8_t ctrl;

	if(readData(device_, ctrlMeas_, &ctrl, 1))
		return false;

	ctrl &= ~0b11;  // clear two lower bits
	ctrl |= MODE_FORCED;

	if(writeReg8(device_, ctrlMeas_, ctrl))
	{
		return false;
	}

	return true;
}

bool isBaroMeasuring(BMP280_HandleTypedef *device_)
{
	uint8_t status;

	if(readData(device_, status_, &status, 1))
		return false;

	if(status & (1 << 3))
	{
		return true;
	}

	return false;
}

static inline int32_t compensateTemp(BMP280_HandleTypedef *device_, int32_t adcTemp, int32_t *fineTemp)
{
	int32_t var1, var2;

	var1 = ((((adcTemp >> 3) - ((int32_t) device_->digT1 << 1)))
			* (int32_t) device_->digT2) >> 11;
	var2 = (((((adcTemp >> 4) - (int32_t) device_->digT1)
			* ((adcTemp >> 4) - (int32_t) device_->digT1)) >> 12)
			* (int32_t) device_->digT3) >> 14;

	*fineTemp = var1 + var2;

	return (*fineTemp * 5 + 128) >> 8;
}

static inline uint32_t compensatePress(BMP280_HandleTypedef *device_, int32_t adcPress, int32_t fineTemp)
{
	int64_t var1, var2, press;

	var1 = (int64_t) fineTemp - 128000;
	var2 = var1 * var1 * (int64_t) device_->digP6;
	var2 = var2 + ((var1 * (int64_t) device_->digP5) << 17);
	var2 = var2 + (((int64_t) device_->digP4) << 35);
	var1 = ((var1 * var1 * (int64_t) device_->digP3) >> 8)
			+ ((var1 * (int64_t) device_->digP2) << 12);
	var1 = (((int64_t) 1 << 47) + var1) * ((int64_t) device_->digP1) >> 33;

	if(var1 == 0)
		return 0;  // avoid exception caused by division by zero

	press = 1048576 - adcPress;
	press = (((press << 31) - var2) * 3125) / var1;
	var1 = ((int64_t) device_->digP9 * (press >> 13) * (press >> 13)) >> 25;
	var2 = ((int64_t) device_->digP8 * press) >> 19;

	press = ((press + var1 + var2) >> 8) + ((int64_t) device_->digP7 << 4);
	return press;
}

bool baroReadFixed(BMP280_HandleTypedef *device_, int32_t *temperature_,
        uint32_t *pressure_, uint32_t *humidity_)
{
	int32_t adcPressure;
	int32_t adcTemp;
	uint8_t data[8];
	int32_t fineTemp;

	// Only the BME280 supports reading the humidity.
	if(device_->id != chipID_)
	{
		if (humidity_)
			*humidity_ = 0;
		humidity_ = NULL;
	}

	// Need to read in one sequence to ensure they match.
	size_t size = humidity_ ? 8 : 6;
	if (readData(device_, pressMSB_, data, size)) {
		return false;
	}

	adcPressure = data[0] << 12 | data[1] << 4 | data[2] >> 4;
	adcTemp = data[3] << 12 | data[4] << 4 | data[5] >> 4;


	*temperature_ = compensateTemp(device_, adcTemp, &fineTemp);
	*pressure_ = compensatePress(device_, adcPressure, fineTemp);

	if(humidity_)
	{

	}

	return true;
}

bool baroReadFloat(BMP280_HandleTypedef *device_, float *temperature_,
		float *pressure_, float *humidity_)
{
	int32_t fixedTemperature;
	uint32_t fixedPressure;
	uint32_t fixedHumidity;

	if(baroReadFixed(device_, &fixedTemperature, &fixedPressure, humidity_ ? &fixedHumidity : NULL))
	{
		*temperature_ = (float) fixedTemperature / 100;
		*pressure_ = (float) fixedPressure / 256;
		if(humidity_)
			*humidity_ = (float) fixedHumidity / 1024;
		return true;
	}

	return false;
}
