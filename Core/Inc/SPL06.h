#ifndef ZEROTYPE_SPL06_H
#define ZEROTYPE_SPL06_H

#include <stdint.h>

#include "i2c.h"

#define SPL06_REG_PSR_B2         0x00U
#define SPL06_REG_TMP_B2         0x03U
#define SPL06_REG_PRS_CFG        0x06U
#define SPL06_REG_TMP_CFG        0x07U
#define SPL06_REG_MEAS_CFG       0x08U
#define SPL06_REG_CFG_REG        0x09U
#define SPL06_REG_RESET          0x0CU
#define SPL06_REG_ID             0x0DU
#define SPL06_REG_COEF           0x10U
#define SPL06_REG_COEF_SRCE      0x28U

#define SPL06_ID_MASK            0xF0U
#define SPL06_ID_EXPECTED        0x10U

#define SPL06_MEAS_COEF_RDY      0x80U
#define SPL06_MEAS_SENSOR_RDY    0x40U

#define SPL06_RESET_SOFT         0x09U

#define SPL06_MODE_CONT_BOTH     0x07U

#define SPL06_RATE_32HZ          0x05U
#define SPL06_OSR_8X             0x03U
#define SPL06_SCALE_8X           7864320.0f

#define SPL06_TMP_EXT_BIT        0x80U
#define SPL06_I2C_TIMEOUT_MS     100U
#define SPL06_READY_TIMEOUT_MS   100U

#define SPL06_I2C_ADDR_7BIT      0x76U
#define SPL06_I2C_ADDR_HAL       (SPL06_I2C_ADDR_7BIT << 1)

typedef enum
{
    SPL06_OK = 0,
    SPL06_ERROR,
    SPL06_ID_ERROR,
    SPL06_TIMEOUT,
} SPL06_Status_t;

typedef struct
{
    int16_t c0;
    int16_t c1;
    int32_t c00;
    int32_t c10;
    int16_t c01;
    int16_t c11;
    int16_t c20;
    int16_t c21;
    int16_t c30;
} SPL06_Calib_t;

typedef struct
{
    I2C_HandleTypeDef *hi2c;
    uint16_t address;

    SPL06_Calib_t calib;

    float pressure_scale;
    float temperature_scale;
} SPL06_t;

typedef struct
{
    int32_t raw_pressure;
    int32_t raw_temperature;

    float pressure_pa;
    float temperature_deg_c;
} SPL06_Data_t;

SPL06_Status_t SPL06_Init(SPL06_t *dev,
                          I2C_HandleTypeDef *hi2c,
                          uint16_t address);

SPL06_Status_t SPL06_ReadRaw(SPL06_t *dev,
                             int32_t *raw_pressure,
                             int32_t *raw_temperature);

SPL06_Status_t SPL06_Read(SPL06_t *dev,
                          SPL06_Data_t *data);

SPL06_Status_t SPL06_ReadID(SPL06_t *dev,
                            uint8_t *id);

float SPL06_PressureToAltitude(float pressure_pa,
                               float reference_pressure_pa);

#endif /* ZEROTYPE_SPL06_H */
