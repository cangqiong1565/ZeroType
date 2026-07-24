#include "SPL06.h"

#include <math.h>
#include <stddef.h>

static SPL06_Status_t SPL06_ReadRegs(SPL06_t *dev,
                                     uint8_t reg,
                                     uint8_t *buf,
                                     uint16_t len)
{
    if ((dev == NULL) || (dev->hi2c == NULL) || (buf == NULL))
    {
        return SPL06_ERROR;
    }

    if (HAL_I2C_Mem_Read(dev->hi2c,
                         dev->address,
                         reg,
                         I2C_MEMADD_SIZE_8BIT,
                         buf,
                         len,
                         SPL06_I2C_TIMEOUT_MS) != HAL_OK)
    {
        return SPL06_ERROR;
    }

    return SPL06_OK;
}

static SPL06_Status_t SPL06_WriteReg(SPL06_t *dev,
                                     uint8_t reg,
                                     uint8_t value)
{
    if ((dev == NULL) || (dev->hi2c == NULL))
    {
        return SPL06_ERROR;
    }

    if (HAL_I2C_Mem_Write(dev->hi2c,
                          dev->address,
                          reg,
                          I2C_MEMADD_SIZE_8BIT,
                          &value,
                          1U,
                          SPL06_I2C_TIMEOUT_MS) != HAL_OK)
    {
        return SPL06_ERROR;
    }

    return SPL06_OK;
}

static int32_t SPL06_SignExtend(uint32_t value, uint8_t bits)
{
    uint32_t sign_bit = 1UL << (bits - 1U);

    if ((value & sign_bit) != 0U)
    {
        value |= (~0UL << bits);
    }

    return (int32_t)value;
}

static SPL06_Status_t SPL06_Read24(SPL06_t *dev,
                                   uint8_t reg,
                                   int32_t *value)
{
    uint8_t buf[3];
    uint32_t raw;

    if (value == NULL)
    {
        return SPL06_ERROR;
    }

    if (SPL06_ReadRegs(dev, reg, buf, sizeof(buf)) != SPL06_OK)
    {
        return SPL06_ERROR;
    }

    raw = ((uint32_t)buf[0] << 16) |
          ((uint32_t)buf[1] << 8) |
          (uint32_t)buf[2];

    *value = SPL06_SignExtend(raw, 24U);

    return SPL06_OK;
}

static SPL06_Status_t SPL06_WaitReady(SPL06_t *dev)
{
    uint8_t meas_cfg;
    uint32_t start_tick = HAL_GetTick();

    for (;;)
    {
        if (SPL06_ReadRegs(dev, SPL06_REG_MEAS_CFG, &meas_cfg, 1U) != SPL06_OK)
        {
            return SPL06_ERROR;
        }

        if ((meas_cfg & (SPL06_MEAS_COEF_RDY | SPL06_MEAS_SENSOR_RDY)) ==
            (SPL06_MEAS_COEF_RDY | SPL06_MEAS_SENSOR_RDY))
        {
            return SPL06_OK;
        }

        if ((HAL_GetTick() - start_tick) > SPL06_READY_TIMEOUT_MS)
        {
            return SPL06_TIMEOUT;
        }

        HAL_Delay(1U);
    }
}

static SPL06_Status_t SPL06_ReadCalib(SPL06_t *dev)
{
    uint8_t coef[18];

    if (SPL06_ReadRegs(dev, SPL06_REG_COEF, coef, sizeof(coef)) != SPL06_OK)
    {
        return SPL06_ERROR;
    }

    dev->calib.c0 = (int16_t)SPL06_SignExtend(
        ((uint32_t)coef[0] << 4) | ((uint32_t)coef[1] >> 4),
        12U
    );

    dev->calib.c1 = (int16_t)SPL06_SignExtend(
        (((uint32_t)coef[1] & 0x0FU) << 8) | (uint32_t)coef[2],
        12U
    );

    dev->calib.c00 = SPL06_SignExtend(
        ((uint32_t)coef[3] << 12) |
        ((uint32_t)coef[4] << 4) |
        ((uint32_t)coef[5] >> 4),
        20U
    );

    dev->calib.c10 = SPL06_SignExtend(
        (((uint32_t)coef[5] & 0x0FU) << 16) |
        ((uint32_t)coef[6] << 8) |
        (uint32_t)coef[7],
        20U
    );

    dev->calib.c01 = (int16_t)(((uint16_t)coef[8] << 8) | coef[9]);
    dev->calib.c11 = (int16_t)(((uint16_t)coef[10] << 8) | coef[11]);
    dev->calib.c20 = (int16_t)(((uint16_t)coef[12] << 8) | coef[13]);
    dev->calib.c21 = (int16_t)(((uint16_t)coef[14] << 8) | coef[15]);
    dev->calib.c30 = (int16_t)(((uint16_t)coef[16] << 8) | coef[17]);

    return SPL06_OK;
}

static uint8_t SPL06_NormalizeAddress(uint16_t address)
{
    if (address <= 0x7FU)
    {
        return (uint8_t)(address << 1);
    }

    return (uint8_t)address;
}

SPL06_Status_t SPL06_Init(SPL06_t *dev,
                          I2C_HandleTypeDef *hi2c,
                          uint16_t address)
{
    uint8_t id;
    uint8_t coef_srce = SPL06_TMP_EXT_BIT;
    uint8_t tmp_cfg;

    if ((dev == NULL) || (hi2c == NULL))
    {
        return SPL06_ERROR;
    }

    dev->hi2c = hi2c;
    dev->address = SPL06_NormalizeAddress(address);
    dev->pressure_scale = SPL06_SCALE_8X;
    dev->temperature_scale = SPL06_SCALE_8X;

    if (SPL06_WriteReg(dev, SPL06_REG_RESET, SPL06_RESET_SOFT) != SPL06_OK)
    {
        return SPL06_ERROR;
    }

    HAL_Delay(50U);

    if (SPL06_ReadID(dev, &id) != SPL06_OK)
    {
        return SPL06_ERROR;
    }

    if ((id & SPL06_ID_MASK) != SPL06_ID_EXPECTED)
    {
        return SPL06_ID_ERROR;
    }

    if (SPL06_WaitReady(dev) != SPL06_OK)
    {
        return SPL06_TIMEOUT;
    }

    if (SPL06_ReadCalib(dev) != SPL06_OK)
    {
        return SPL06_ERROR;
    }

    /*
     * COEF_SRCE bit7 tells which temperature source was used for the factory
     * calibration coefficients. TMP_CFG bit7 should match it.
     */
    (void)SPL06_ReadRegs(dev, SPL06_REG_COEF_SRCE, &coef_srce, 1U);
    tmp_cfg = (uint8_t)((coef_srce & SPL06_TMP_EXT_BIT) |
                        (SPL06_RATE_32HZ << 4) |
                        SPL06_OSR_8X);

    if (SPL06_WriteReg(dev,
                       SPL06_REG_PRS_CFG,
                       (uint8_t)((SPL06_RATE_32HZ << 4) | SPL06_OSR_8X)) != SPL06_OK)
    {
        return SPL06_ERROR;
    }

    if (SPL06_WriteReg(dev, SPL06_REG_TMP_CFG, tmp_cfg) != SPL06_OK)
    {
        return SPL06_ERROR;
    }

    /*
     * OSR is 8x, so pressure/temperature shift bits are not needed.
     * FIFO and interrupts are also disabled for the first polling driver.
     */
    if (SPL06_WriteReg(dev, SPL06_REG_CFG_REG, 0x00U) != SPL06_OK)
    {
        return SPL06_ERROR;
    }

    if (SPL06_WriteReg(dev, SPL06_REG_MEAS_CFG, SPL06_MODE_CONT_BOTH) != SPL06_OK)
    {
        return SPL06_ERROR;
    }

    return SPL06_OK;
}

SPL06_Status_t SPL06_ReadRaw(SPL06_t *dev,
                             int32_t *raw_pressure,
                             int32_t *raw_temperature)
{
    if ((raw_pressure == NULL) || (raw_temperature == NULL))
    {
        return SPL06_ERROR;
    }

    if (SPL06_Read24(dev, SPL06_REG_PSR_B2, raw_pressure) != SPL06_OK)
    {
        return SPL06_ERROR;
    }

    if (SPL06_Read24(dev, SPL06_REG_TMP_B2, raw_temperature) != SPL06_OK)
    {
        return SPL06_ERROR;
    }

    return SPL06_OK;
}

SPL06_Status_t SPL06_Read(SPL06_t *dev,
                          SPL06_Data_t *data)
{
    float prs_sc;
    float tmp_sc;
    float temperature;
    float pressure;

    if ((dev == NULL) || (data == NULL))
    {
        return SPL06_ERROR;
    }

    if (SPL06_ReadRaw(dev,
                      &data->raw_pressure,
                      &data->raw_temperature) != SPL06_OK)
    {
        return SPL06_ERROR;
    }

    prs_sc = (float)data->raw_pressure / dev->pressure_scale;
    tmp_sc = (float)data->raw_temperature / dev->temperature_scale;

    temperature = ((float)dev->calib.c0 * 0.5f) +
                  ((float)dev->calib.c1 * tmp_sc);

    pressure = (float)dev->calib.c00 +
               prs_sc * ((float)dev->calib.c10 +
               prs_sc * ((float)dev->calib.c20 +
               prs_sc * (float)dev->calib.c30)) +
               tmp_sc * (float)dev->calib.c01 +
               tmp_sc * prs_sc * ((float)dev->calib.c11 +
               prs_sc * (float)dev->calib.c21);

    data->temperature_deg_c = temperature;
    data->pressure_pa = pressure;

    return SPL06_OK;
}

SPL06_Status_t SPL06_ReadID(SPL06_t *dev,
                            uint8_t *id)
{
    if (id == NULL)
    {
        return SPL06_ERROR;
    }

    return SPL06_ReadRegs(dev, SPL06_REG_ID, id, 1U);
}

float SPL06_PressureToAltitude(float pressure_pa,
                               float reference_pressure_pa)
{
    if ((pressure_pa <= 0.0f) || (reference_pressure_pa <= 0.0f))
    {
        return 0.0f;
    }

    return 44330.0f *
           (1.0f - powf(pressure_pa / reference_pressure_pa, 0.19029495f));
}
