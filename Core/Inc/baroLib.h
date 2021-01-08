/*
 * baroLib.h
 *
 *  Created on: Jan 5, 2021
 *      Author: Pravesh Narayan
 */

#ifndef INC_BAROLIB_H_
#define INC_BAROLIB_H_

#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "userHW.h"
#include <stdbool.h>
#include <stdlib.h>

//#define 	handheldLowPower
//#define 	handheldDynamic
//#define 	weatherMonitoring
//#define 	elevatorDetection
//#define 	dropDetection
#define 	indoorNavigation

//#define	sleepMode
//#define 	forcedMode
#define		normalMode

#define 	baseAdd__	0x76
#define		baseAdd_	0x77
#define		baseWrite_	0xEE
#define		baseRead_	0xEF
#define 	chipID_		0x58

#define		resetAdd_	0xE0
#define		bmpReset_	0xB6
#define		identity_	0xD0
#define 	calibReg_	0x88
#define		status_		0xF3
#define		ctrlMeas_	0xF4
#define		config_		0xF5

#define		pressMSB_	0xF7
#define		pressLSB_	0xF8
#define		pressXLSB_	0xF9
#define 	pressReg_	(pressMSB_)

#define		tempMSB_	0xFA
#define		tempLSB_	0xFB
#define		tempXLSB_	0xFC
#define 	tempReg_    (tempMSB_)


typedef enum {
    MODE_SLEEP = 0,
    MODE_FORCED = 1,
	MODE_NORMAL = 3
} operationMode;

typedef enum {
    FILTER_OFF = 0,
    FILTER_2 = 1,
    FILTER_4 = 2,
    FILTER_8 = 3,
    FILTER_16 = 4
} filterConfig;

typedef enum {
    SKIPPED = 0,
    ULTRA_LOW_POWER = 1,
    LOW_POWER = 2,
    STANDARD = 3,
    HIGH_RES = 4,
    ULTRA_HIGH_RES = 5
} overSamplingConfig;

typedef enum {
    STANDBY_05 = 0,
    STANDBY_62 = 1,
    STANDBY_125 = 2,
    STANDBY_250 = 3,
    STANDBY_500 = 4,
    STANDBY_1000 = 5,
    STANDBY_2000 = 6,
    STANDBY_4000 = 7,
} standbyTime;

typedef struct {
	operationMode mode_;
	filterConfig filter_;
	overSamplingConfig pressOS_;
	overSamplingConfig tempOS_;
	overSamplingConfig humOS_;
	standbyTime standby_;
} bmpConfig;

typedef struct {
    uint16_t digT1;
    int16_t  digT2;
    int16_t  digT3;
    uint16_t digP1;
    int16_t  digP2;
    int16_t  digP3;
    int16_t  digP4;
    int16_t  digP5;
    int16_t  digP6;
    int16_t  digP7;
    int16_t  digP8;
    int16_t  digP9;

    /* Humidity compensation for BME280 */
    uint8_t  digH1;
    int16_t  digH2;
    uint8_t  digH3;
    int16_t  digH4;
    int16_t  digH5;
    int8_t   digH6;

    uint16_t addr;

    I2C_HandleTypeDef* i2c;

    bmpConfig params_;

    uint8_t  id;

} BMP280_HandleTypedef;


void baroInitDefault(bmpConfig params_);

bool baroInit(BMP280_HandleTypedef *device_, bmpConfig *params_);

bool baroForceMeas(BMP280_HandleTypedef *device_);

bool isBaroMeasuring(BMP280_HandleTypedef *device_);

bool baroReadFixed(BMP280_HandleTypedef *device_, int32_t *temperature_,
                       uint32_t *pressure_, uint32_t *humidity_);

bool baroReadFloat(BMP280_HandleTypedef *device_, float *temperature_,
		float *pressure_, float *humidity_);



#endif /* INC_BAROLIB_H_ */
