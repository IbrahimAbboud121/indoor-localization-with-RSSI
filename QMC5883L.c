/*
 * QMC5883L.c
 *
 *  Created on: Sep 25, 2026
 *      Author: USER
 */


#include"QMC5883L.h"
#include"main.h"
#include"math.h"
#include"stdio.h"
#include "FreeRTOS.h"
#include "task.h"


/* ---------- Low-level helpers (interrupt mode) ---------- */
static HAL_StatusTypeDef write_reg(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t value)
{
    if (HAL_I2C_Mem_Write_IT(hi2c, QMC5883L_ADDR, reg,
                              I2C_MEMADD_SIZE_8BIT, &value, 1) != HAL_OK) {
        return HAL_ERROR;
    }
    return (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100)) == 1) ? HAL_OK : HAL_TIMEOUT;
}

static HAL_StatusTypeDef read_regs(I2C_HandleTypeDef *hi2c, uint8_t reg,
                                   uint8_t *buf, uint16_t len)
{
    if (HAL_I2C_Mem_Read_IT(hi2c, QMC5883L_ADDR, reg,
                             I2C_MEMADD_SIZE_8BIT, buf, len) != HAL_OK) {
        return HAL_ERROR;
    }
    return (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100)) == 1) ? HAL_OK : HAL_TIMEOUT;
}


HAL_StatusTypeDef QMC5883L_Init(I2C_HandleTypeDef *hi2c, uint8_t odr, uint8_t range, uint8_t osr)
{
    /* Soft reset first */
    if (write_reg(hi2c, QMC5883L_CONTROL2, QMC5883L_SOFT_RESET) != HAL_OK) return HAL_ERROR;
    vTaskDelay(pdMS_TO_TICKS(10));

    /* SET/RESET period — datasheet says just write 0x01, no further explanation given */
    if (write_reg(hi2c, QMC5883L_SET_RESET, 0x01) != HAL_OK) return HAL_ERROR;

    /* Control1: continuous mode + chosen ODR/range/OSR */
    uint8_t ctrl1 = QMC5883L_MODE_CONTINUOUS | odr | range | osr;
    if (write_reg(hi2c, QMC5883L_CONTROL1, ctrl1) != HAL_OK) return HAL_ERROR;

    vTaskDelay(pdMS_TO_TICKS(10)); // let first conversion complete
    return HAL_OK;
}

HAL_StatusTypeDef QMC5883L_ReadChipID(I2C_HandleTypeDef *hi2c, uint8_t *chip_id)
{
    *chip_id = 0;
    return read_regs(hi2c, QMC5883L_CHIP_ID, chip_id, 1);
}

uint8_t QMC5883L_DataReady(I2C_HandleTypeDef *hi2c)
{
    uint8_t status = 0;
    if (read_regs(hi2c, QMC5883L_STATUS, &status, 1) != HAL_OK) return 0;
    return (status & QMC5883L_STATUS_DRDY) ? 1 : 0;
}

HAL_StatusTypeDef QMC5883L_ReadRaw(I2C_HandleTypeDef *hi2c, QMC5883L_RawData *data)
{
    uint8_t buf[6];
    HAL_StatusTypeDef status = read_regs(hi2c, QMC5883L_DATA_X_L, buf, 6);
    if (status != HAL_OK) return status;

    /* QMC5883L is little-endian: LSB first, unlike MPU9250's MSB-first! */
    data->mag_x = (int16_t)((buf[1] << 8) | buf[0]);
    data->mag_y = (int16_t)((buf[3] << 8) | buf[2]);
    data->mag_z = (int16_t)((buf[5] << 8) | buf[4]);

    return HAL_OK;
}

void QMC5883L_ConvertToPhysical(const QMC5883L_RawData *raw, QMC5883L_PhysicalData *phys, uint8_t range)
{
    /* Sensitivity depends on range: 2G -> 12000 LSB/Gauss, 8G -> 3000 LSB/Gauss.
     * 1 Gauss = 100 microtesla, so convert straight to LSB/µT. */
    float lsb_per_ut = (range == QMC5883L_RNG_8G) ? 30.0f : 120.0f;

    phys->mag_x_ut = raw->mag_x / lsb_per_ut;
    phys->mag_y_ut = raw->mag_y / lsb_per_ut;
    phys->mag_z_ut = raw->mag_z / lsb_per_ut;
}


float QMC5883L_ComputeHeading(const QMC5883L_PhysicalData *phys)
{
    /* Tilt-uncompensated heading — only valid when the sensor is flat/level.
     * Full implementation later should use accel-derived pitch/roll to tilt-compensate. */
    float heading_rad = atan2f(phys->mag_y_ut, phys->mag_x_ut);
    float heading_deg = heading_rad * (180.0f / (float)M_PI);
    if (heading_deg < 0) heading_deg += 360.0f;
    return heading_deg;
}


/* ============================================================
 * FULL CALIBRATION — hard iron + soft iron
 *
 * User rotates the sensor slowly through multiple full 360°
 * turns in a flat horizontal plane during duration_ms.
 *
 * Hard iron: center of min/max range on each axis
 * Soft iron: per-axis scale so all axes span a common range
 *
 * Quality: 1.0 if X and Y spans are equal (perfect circle),
 *          0.0 if one axis is twice as wide as the other.
 * ============================================================ */
HAL_StatusTypeDef QMC5883L_Calibrate(I2C_HandleTypeDef *hi2c,
                                     uint8_t range,
                                     uint32_t duration_ms,
                                     QMC5883L_Calibration *cal)
{
    QMC5883L_RawData raw;
    QMC5883L_PhysicalData phys;

    float x_min =  1e9f, x_max = -1e9f;
    float y_min =  1e9f, y_max = -1e9f;
    float z_min =  1e9f, z_max = -1e9f;

    uint32_t start = HAL_GetTick();

    while ((HAL_GetTick() - start) < duration_ms) {
        if (QMC5883L_ReadRaw(hi2c, &raw) != HAL_OK) {
        	vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }
        QMC5883L_ConvertToPhysical(&raw, &phys, range);

        if (phys.mag_x_ut < x_min) x_min = phys.mag_x_ut;
        if (phys.mag_x_ut > x_max) x_max = phys.mag_x_ut;
        if (phys.mag_y_ut < y_min) y_min = phys.mag_y_ut;
        if (phys.mag_y_ut > y_max) y_max = phys.mag_y_ut;
        if (phys.mag_z_ut < z_min) z_min = phys.mag_z_ut;
        if (phys.mag_z_ut > z_max) z_max = phys.mag_z_ut;

        vTaskDelay(pdMS_TO_TICKS(20));
    }

    cal->hard_x = (x_max + x_min) / 2.0f;
    cal->hard_y = (y_max + y_min) / 2.0f;
    cal->hard_z = (z_max + z_min) / 2.0f;

    float sx = (x_max - x_min) / 2.0f;
    float sy = (y_max - y_min) / 2.0f;
    float sz = (z_max - z_min) / 2.0f;

    float reference = (sx + sy) / 2.0f;
    cal->soft_x = (sx > 0.01f) ? (reference / sx) : 1.0f;
    cal->soft_y = (sy > 0.01f) ? (reference / sy) : 1.0f;
    cal->soft_z = (sz > 0.01f) ? (reference / sz) : 1.0f;

    return HAL_OK;
}

void QMC5883L_ApplyCalibration(QMC5883L_PhysicalData *phys,
                               const QMC5883L_Calibration *cal)
{
    float xc = phys->mag_x_ut - cal->hard_x;
    float yc = phys->mag_y_ut - cal->hard_y;
    float zc = phys->mag_z_ut - cal->hard_z;

    phys->mag_x_ut = xc * cal->soft_x;
    phys->mag_y_ut = yc * cal->soft_y;
    phys->mag_z_ut = zc * cal->soft_z;
}
