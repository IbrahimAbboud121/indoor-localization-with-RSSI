/*
 * MMU-9250.h
 *
 *  Created on: Sep 21, 2026
 *      Author: USER
 */

#ifndef INC_MPU_9250_H_
#define INC_MPU_9250_H_

#include"main.h"
#include "FreeRTOS.h"
#include "task.h"

/* ============================================================
 * MPU 9250 REGISTER MAP & CONFIGURATION VALUES
 * All addresses and values are from the MPU-9250 Datasheet.
 * ============================================================ */

#define MPU9250_ADDR     (0x68 << 1)   // I2C address (AD0=GND), shifted for STM32 HAL

/*Configuration Registers (write to set up sensor) */
#define SMPLRT_DIV       0x19   // Sample Rate Divider: ODR = 1000Hz / (1 + value)
#define CONFIG           0x1A   // DLPF bandwidth for gyro + temp sensor
#define GYRO_CONFIG      0x1B   // Gyro full-scale range (FS_SEL) + self-test
#define ACCEL_CONFIG     0x1C   // Accel full-scale range (AFS_SEL) + self-test
#define PWR_MGMT_1       0x6B   // Sleep, reset, and clock source control
#define WHO_AM_I         0x75   // Read-only device ID (genuine MPU-9250 = 0x71)
#define ACCEL_CONFIG_2   0x1D

/* Accelerometer Data (read-only, 16-bit signed) */
#define ACCEL_XOUT_H     0x3B   // Accel X high byte (MSB)
#define ACCEL_XOUT_L     0x3C   // Accel X low byte  (LSB)
#define ACCEL_YOUT_H     0x3D   // Accel Y high byte (MSB)
#define ACCEL_YOUT_L     0x3E   // Accel Y low byte  (LSB)
#define ACCEL_ZOUT_H     0x3F   // Accel Z high byte (MSB)
#define ACCEL_ZOUT_L     0x40   // Accel Z low byte  (LSB)

/*  Temperature Data (read-only, 16-bit signed) */
#define TEMP_OUT_H       0x41   // Temp high byte (MSB)
#define TEMP_OUT_L       0x42   // Temp low byte  (LSB)

/* Gyroscope Data (read-only, 16-bit signed)  */
#define GYRO_XOUT_H      0x43   // Gyro X high byte (MSB)
#define GYRO_XOUT_L      0x44   // Gyro X low byte  (LSB)
#define GYRO_YOUT_H      0x45   // Gyro Y high byte (MSB)
#define GYRO_YOUT_L      0x46   // Gyro Y low byte  (LSB)
#define GYRO_ZOUT_H      0x47   // Gyro Z high byte (MSB)
#define GYRO_ZOUT_L      0x48   // Gyro Z low byte  (LSB)

/*  Accelerometer Full-Scale Range (write into ACCEL_CONFIG) */
#define ACCEL_FS_2G      0x00   // ±2g,  16384 LSB/g (highest resolution)
#define ACCEL_FS_4G      0x08   // ±4g,   8192 LSB/g (recommended)
#define ACCEL_FS_8G      0x10   // ±8g,   4096 LSB/g
#define ACCEL_FS_16G     0x18   // ±16g,  2048 LSB/g (lowest resolution)

/*  Gyroscope Full-Scale Range (write into GYRO_CONFIG) */
#define GYRO_FS_250DPS   0x00   // ±250 °/s,  131.0 LSB/(°/s) (highest resolution)
#define GYRO_FS_500DPS   0x08   // ±500 °/s,   65.5 LSB/(°/s) (recommended)
#define GYRO_FS_1000DPS  0x10   // ±1000 °/s,  32.8 LSB/(°/s)
#define GYRO_FS_2000DPS  0x18   // ±2000 °/s,  16.4 LSB/(°/s) (lowest resolution)



/* Raw 16-bit signed values straight from the sensor registers.
 * Units: LSB (no conversion applied yet).
 * Range depends on the configured full-scale setting. */
typedef struct {
    int16_t accel_x, accel_y, accel_z;   // Raw accel counts
    int16_t gyro_x,  gyro_y,  gyro_z;    // Raw gyro counts
} MPU9250_RawData;


/* Converted values in physical units.
 * Use these for math, filtering, and fusion algorithms. */
typedef struct {
    float accel_x_g,   accel_y_g,   accel_z_g;     // Acceleration in g
    float gyro_x_dps,  gyro_y_dps,  gyro_z_dps;    // Angular rate in °/s
} MPU9250_PhysicalData;


/* Calibration offsets, computed once at startup while stationary.
 * Units match MPU9250_PhysicalData (g and °/s). */
typedef struct {
    float gyro_bias_x, gyro_bias_y, gyro_bias_z;     // °/s
    float accel_bias_x, accel_bias_y, accel_bias_z;  // g
} MPU9250_Bias;





HAL_StatusTypeDef MPU9250_Init(I2C_HandleTypeDef *hi2c, uint8_t accel_fs, uint8_t gyro_fs);
HAL_StatusTypeDef MPU9250_ReadWhoAmI(I2C_HandleTypeDef *hi2c, uint8_t *who_am_i);
HAL_StatusTypeDef MPU9250_ReadRaw(I2C_HandleTypeDef *hi2c, MPU9250_RawData *data);
void MPU9250_ConvertToPhysical(const MPU9250_RawData *raw, MPU9250_PhysicalData *phys,
                                uint8_t accel_fs, uint8_t gyro_fs);

HAL_StatusTypeDef MPU9250_CalibrateGyroAccel(I2C_HandleTypeDef *hi2c,
                                              uint8_t accel_fs, uint8_t gyro_fs,
                                              uint16_t num_samples,
                                              MPU9250_Bias *bias);

void MPU9250_ApplyBias(MPU9250_PhysicalData *phys, const MPU9250_Bias *bias);
uint8_t MPU9250_IsStationary(const MPU9250_PhysicalData *phys,
                              float gyro_thresh_dps, float accel_thresh_g);

#endif /* INC_MPU_9250_H_ */
