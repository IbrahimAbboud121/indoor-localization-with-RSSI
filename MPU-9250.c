/*
 * MPU-9250.c
 *
 *  Created on: Sep 22, 2026
 *      Author: USER
 */



#include"main.h"
#include"MPU-9250.h"
#include <stdio.h>
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"


/* ---------- Low-level helpers ---------- */
static HAL_StatusTypeDef write_reg(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t value)
{
	// in interrupt mode when it is over , the interrupt function is called
    if (HAL_I2C_Mem_Write_IT(hi2c, MPU9250_ADDR, reg,
                              I2C_MEMADD_SIZE_8BIT, &value, 1) != HAL_OK) {
        return HAL_ERROR;
    }
    /* Block here (yields to scheduler) until the ISR notifies us, or timeout */
    return (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100)) == 1) ? HAL_OK : HAL_TIMEOUT;
}

static HAL_StatusTypeDef read_regs(I2C_HandleTypeDef *hi2c, uint8_t reg,
                                   uint8_t *buf, uint16_t len)
{
    if (HAL_I2C_Mem_Read_IT(hi2c, MPU9250_ADDR, reg,
                             I2C_MEMADD_SIZE_8BIT, buf, len) != HAL_OK) {
        return HAL_ERROR;
    }
    return (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100)) == 1) ? HAL_OK : HAL_TIMEOUT;
}

HAL_StatusTypeDef MPU9250_Init(I2C_HandleTypeDef *hi2c, uint8_t accel_fs, uint8_t gyro_fs){

	     /* Reset the device */
	    if (write_reg(hi2c, PWR_MGMT_1, 0x80) != HAL_OK) return HAL_ERROR;
	    vTaskDelay(pdMS_TO_TICKS(100));   // Wait for reset to complete

	    /* Wake up + select X-gyro PLL as clock source */
	    if (write_reg(hi2c, PWR_MGMT_1, 0x01) != HAL_OK) return HAL_ERROR;
	    vTaskDelay(pdMS_TO_TICKS(100));   // Wait for reset to complete

	    /*  Sample rate = 200 Hz (1000 / (1 + 4)) */
	     if (write_reg(hi2c, SMPLRT_DIV, 0x04) != HAL_OK) return HAL_ERROR;

	    /* DLPF = 41 Hz bandwidth for gyro + temp */
	     if (write_reg(hi2c, CONFIG, 0x04) != HAL_OK) return HAL_ERROR;

	    /*  Gyro full-scale range */
	     if (write_reg(hi2c, GYRO_CONFIG, gyro_fs) != HAL_OK) return HAL_ERROR;

	    /*  Accel full-scale range */
	    if (write_reg(hi2c, ACCEL_CONFIG, accel_fs) != HAL_OK) return HAL_ERROR;

	    /*A_DLPF_CFG = 3 -> ~41Hz accel bandwidth, matching the gyro's filtering*/
	    if (write_reg(hi2c, ACCEL_CONFIG_2, 0x04) != HAL_OK) return HAL_ERROR;


	    	    return HAL_OK;
}



HAL_StatusTypeDef MPU9250_ReadWhoAmI(I2C_HandleTypeDef *hi2c, uint8_t *who_am_i)
{
	*who_am_i = 0;
    return read_regs(hi2c, WHO_AM_I, who_am_i, 1);
}

HAL_StatusTypeDef MPU9250_ReadRaw(I2C_HandleTypeDef *hi2c, MPU9250_RawData *data)
{
    uint8_t buf[14];
    HAL_StatusTypeDef status;

    status = read_regs(hi2c, ACCEL_XOUT_H, buf, 14);
    if (status != HAL_OK) return status;
    /*and 14 because we have each one of them(for example gyro_x) is 16bits so the value is
     * stored in two 8 bits registers so for the accel we have buf[0]=>buf[5] then
     *  buf[6] and buf[7] for thr temp and then from 8 to 14 for gyro*/

    /* Combine high and low bytes into signed 16-bit values.
     * Byte order in buf: [0]=AX_H [1]=AX_L [2]=AY_H ... [13]=GZ_L */
    data->accel_x = (int16_t)((buf[0]  << 8) | buf[1]);
    data->accel_y = (int16_t)((buf[2]  << 8) | buf[3]);
    data->accel_z = (int16_t)((buf[4]  << 8) | buf[5]);
    /* buf[6] and buf[7] are temperature — skip them */
    data->gyro_x  = (int16_t)((buf[8]  << 8) | buf[9]);
    data->gyro_y  = (int16_t)((buf[10] << 8) | buf[11]);
    data->gyro_z  = (int16_t)((buf[12] << 8) | buf[13]);

    return HAL_OK;
}


void MPU9250_ConvertToPhysical(const MPU9250_RawData *raw, MPU9250_PhysicalData *phys,
                                uint8_t accel_fs, uint8_t gyro_fs)
{
    /* Pick the accel sensitivity based on the configured range */
    float accel_scale;
    switch (accel_fs) {
        case ACCEL_FS_2G:  accel_scale = 16384.0f; break;
        case ACCEL_FS_4G:  accel_scale =  8192.0f; break;
        case ACCEL_FS_8G:  accel_scale =  4096.0f; break;
        case ACCEL_FS_16G: accel_scale =  2048.0f; break;
        default:           accel_scale =  8192.0f; break;  // Fallback
    }

    /* Pick the gyro sensitivity based on the configured range */
    float gyro_scale;
    switch (gyro_fs) {
        case GYRO_FS_250DPS:  gyro_scale = 131.0f; break;
        case GYRO_FS_500DPS:  gyro_scale =  65.5f; break;
        case GYRO_FS_1000DPS: gyro_scale =  32.8f; break;
        case GYRO_FS_2000DPS: gyro_scale =  16.4f; break;
        default:              gyro_scale =  65.5f; break;  // Fallback
    }

    phys->accel_x_g   = raw->accel_x / accel_scale;
    phys->accel_y_g   = raw->accel_y / accel_scale;
    phys->accel_z_g   = raw->accel_z / accel_scale;

    phys->gyro_x_dps  = raw->gyro_x  / gyro_scale;
    phys->gyro_y_dps  = raw->gyro_y  / gyro_scale;
    phys->gyro_z_dps  = raw->gyro_z  / gyro_scale;
}


HAL_StatusTypeDef MPU9250_CalibrateGyroAccel(I2C_HandleTypeDef *hi2c,
                                              uint8_t accel_fs, uint8_t gyro_fs,
                                              uint16_t num_samples,
                                              MPU9250_Bias *bias)
{
    /* IMPORTANT: sensor must be completely still and level during this call.
     * Call this once at boot before starting the EKF/telemetry tasks. */
    MPU9250_RawData raw;
    MPU9250_PhysicalData phys;
    double sum_gx = 0, sum_gy = 0, sum_gz = 0;
    double sum_ax = 0, sum_ay = 0, sum_az = 0;

    for (uint16_t i = 0; i < num_samples; i++) {
        if (MPU9250_ReadRaw(hi2c, &raw) != HAL_OK) return HAL_ERROR;
        MPU9250_ConvertToPhysical(&raw, &phys, accel_fs, gyro_fs);

        sum_gx += phys.gyro_x_dps;
        sum_gy += phys.gyro_y_dps;
        sum_gz += phys.gyro_z_dps;

        sum_ax += phys.accel_x_g;
        sum_ay += phys.accel_y_g;
        sum_az += phys.accel_z_g;

        vTaskDelay(pdMS_TO_TICKS(2));// ~ match your sample rate, don't hammer the bus
    }

    /* Gyro bias: true stationary value should be 0 on all axes */
    bias->gyro_bias_x = (float)(sum_gx / num_samples);
    bias->gyro_bias_y = (float)(sum_gy / num_samples);
    bias->gyro_bias_z = (float)(sum_gz / num_samples);

    /* Accel bias: assumes tag is lying flat, Z-axis pointing up during calibration.
     * X and Y should read 0g at rest; Z should read +1g, so we only remove the
     * *excess* on Z, not the whole 1g. */
    bias->accel_bias_x = (float)(sum_ax / num_samples);
    bias->accel_bias_y = (float)(sum_ay / num_samples);
    bias->accel_bias_z = (float)(sum_az / num_samples) - 1.0f;

    return HAL_OK;
}

void MPU9250_ApplyBias(MPU9250_PhysicalData *phys, const MPU9250_Bias *bias)
{
    phys->gyro_x_dps  -= bias->gyro_bias_x;
    phys->gyro_y_dps  -= bias->gyro_bias_y;
    phys->gyro_z_dps  -= bias->gyro_bias_z;

    phys->accel_x_g   -= bias->accel_bias_x;
    phys->accel_y_g   -= bias->accel_bias_y;
    phys->accel_z_g   -= bias->accel_bias_z;
}

uint8_t MPU9250_IsStationary(const MPU9250_PhysicalData *phys,
                              float gyro_thresh_dps, float accel_thresh_g)
{
    float gyro_mag = sqrtf(phys->gyro_x_dps * phys->gyro_x_dps +
                            phys->gyro_y_dps * phys->gyro_y_dps +
                            phys->gyro_z_dps * phys->gyro_z_dps);

    float accel_mag = sqrtf(phys->accel_x_g * phys->accel_x_g +
                             phys->accel_y_g * phys->accel_y_g +
                             phys->accel_z_g * phys->accel_z_g);

    uint8_t gyro_still  = (gyro_mag  < gyro_thresh_dps);
    uint8_t accel_still = (fabsf(accel_mag - 1.0f) < accel_thresh_g);

    return (gyro_still && accel_still);
}
