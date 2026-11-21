/*
 * QMC5883L.h
 *
 *  Created on: Sep 25, 2026
 *      Author: USER
 */

#ifndef INC_QMC5883L_H_
#define INC_QMC5883L_H_

#include"main.h"
#include "FreeRTOS.h"
#include "task.h"

/*all those registers were brought from the QMC5883L datahseet*/

#define QMC5883L_ADDR      (0x0D << 1)  // I2C address, shifted for STM32 HAL

/* Data / status registers */
#define QMC5883L_DATA_X_L  0x00
#define QMC5883L_DATA_X_H  0x01
#define QMC5883L_DATA_Y_L  0x02
#define QMC5883L_DATA_Y_H  0x03
#define QMC5883L_DATA_Z_L  0x04
#define QMC5883L_DATA_Z_H  0x05
#define QMC5883L_STATUS    0x06
#define QMC5883L_TEMP_L    0x07
#define QMC5883L_TEMP_H    0x08

/* Config registers */
#define QMC5883L_CONTROL1  0x09
#define QMC5883L_CONTROL2  0x0A
#define QMC5883L_SET_RESET 0x0B
#define QMC5883L_CHIP_ID   0x0D   // expected 0xFF

/* Status bits (register 0x06) */
#define QMC5883L_STATUS_DRDY  0x01  // new data ready
#define QMC5883L_STATUS_OVL   0x02  // data overflow
#define QMC5883L_STATUS_DOR   0x04  // data skipped/overrun

/* Control1 (0x09) fields — OR these together */
/*Bit:   7  6  |  5  4  |  3  2  |  1  0
  Field: OSR   |  RNG   |  ODR   |  MODE
  we should shift each register depending on its order*/
#define QMC5883L_MODE_STANDBY     0x00 //first bit
#define QMC5883L_MODE_CONTINUOUS  0x01

#define QMC5883L_ODR_10HZ   0x00 //0x00<<2=0x00
#define QMC5883L_ODR_50HZ   0x04 //0x01<<2=0x04
#define QMC5883L_ODR_100HZ  0x08 //0x10<<2=0x08
#define QMC5883L_ODR_200HZ  0x0C //0x11<<2=0x0C

#define QMC5883L_RNG_2G     0x00 // 12000 LSB/Gauss , 0x00<<4=0x00
#define QMC5883L_RNG_8G     0x10 // 3000  LSB/Gauss , 0x01<<4=0x10

#define QMC5883L_OSR_512    0x00 //0x00<<6
#define QMC5883L_OSR_256    0x40 //0x01<<6
#define QMC5883L_OSR_128    0x80 //0x10<<6
#define QMC5883L_OSR_64     0xC0 //0x11<<6


/* Control2 (0x0A) */
#define QMC5883L_SOFT_RESET 0x80


/*structures*/
typedef struct {
    int16_t mag_x, mag_y, mag_z;   // Raw counts
} QMC5883L_RawData;

typedef struct {
    float mag_x_ut, mag_y_ut, mag_z_ut;   // Field strength, microtesla (µT)
} QMC5883L_PhysicalData;


/*
 * Full magnetometer calibration: hard iron + soft iron.
 *
 *   hard_*  : center offset (µT), subtracted first
 *   soft_*  : per-axis scale (dimensionless), applied after centering
 *
 * Apply on a raw µT reading:
 *   x_corr = (x_raw - hard_x) * soft_x it is scale(xraw-offset)
 *   y_corr = (y_raw - hard_y) * soft_y
 *   z_corr = (z_raw - hard_z) * soft_z
 *
 * After correction, the XY locus over a full rotation is a
 * circle centered at origin with radius ~1.0
 */
typedef struct {
    float hard_x, hard_y, hard_z;   /* center (µT) */
    float soft_x, soft_y, soft_z;   /* per-axis scale */
} QMC5883L_Calibration;

HAL_StatusTypeDef QMC5883L_Init(I2C_HandleTypeDef *hi2c,
                               uint8_t odr, uint8_t range, uint8_t osr);
HAL_StatusTypeDef QMC5883L_ReadChipID(I2C_HandleTypeDef *hi2c, uint8_t *chip_id);
uint8_t           QMC5883L_DataReady(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef QMC5883L_ReadRaw(I2C_HandleTypeDef *hi2c, QMC5883L_RawData *data);
void QMC5883L_ConvertToPhysical(const QMC5883L_RawData *raw,
                                QMC5883L_PhysicalData *phys,
                                uint8_t range);
HAL_StatusTypeDef QMC5883L_Calibrate(I2C_HandleTypeDef *hi2c,
                                     uint8_t range,
                                     uint32_t duration_ms,
                                     QMC5883L_Calibration *cal);
void QMC5883L_ApplyCalibration(QMC5883L_PhysicalData *phys,
                               const QMC5883L_Calibration *cal);
float QMC5883L_ComputeHeading(const QMC5883L_PhysicalData *phys);

#endif /* INC_QMC5883L_H_ */
