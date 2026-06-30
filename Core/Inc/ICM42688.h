#ifndef ICM42688_H
#define ICM42688_H

#include <stdint.h>
#include <stdbool.h>

/* ================================================================
 * 寄存器地址定义 — Bank 0 (所有地址从数据手册 v1.7)
 * ================================================================ */

#define ICM42688_BANK_SEL             0x76
#define ICM42688_DEVICE_CONFIG         0x11
#define ICM42688_PWR_MGMT0             0x4E
#define ICM42688_INTF_CONFIG1          0x4D
#define ICM42688_GYRO_CONFIG0          0x4F
#define ICM42688_ACCEL_CONFIG0         0x50
#define ICM42688_GYRO_ACCEL_CONFIG0    0x52
#define ICM42688_INT_CONFIG            0x14
#define ICM42688_INT_CONFIG0           0x63
#define ICM42688_INT_CONFIG1           0x64
#define ICM42688_INT_SOURCE0           0x65
#define ICM42688_WHO_AM_I              0x75
#define ICM42688_TEMP_DATA1            0x1D
#define ICM42688_ACCEL_DATA_X1         0x1F
#define ICM42688_GYRO_DATA_X1          0x25

/* Bank 1 */
#define ICM42688_GYRO_CONFIG_STATIC3   0x0C
#define ICM42688_GYRO_CONFIG_STATIC4   0x0D
#define ICM42688_GYRO_CONFIG_STATIC5   0x0E

/* Bank 2 */
#define ICM42688_ACCEL_CONFIG_STATIC2  0x03
#define ICM42688_ACCEL_CONFIG_STATIC3  0x04
#define ICM42688_ACCEL_CONFIG_STATIC4  0x05

/* ================================================================
 * 寄存器位域
 * ================================================================ */

/* DEVICE_CONFIG */
#define DEVICE_CONFIG_SOFT_RESET       (1 << 0)

/* PWR_MGMT0 */
#define PWR_MGMT0_ACCEL_MODE_LN        (3 << 0)
#define PWR_MGMT0_GYRO_MODE_LN         (3 << 2)
#define PWR_MGMT0_TEMP_DISABLE         (1 << 5)

/* INTF_CONFIG1 — AFSR stall fix */
#define INTF_CONFIG1_AFSR_MASK         0xC0
#define INTF_CONFIG1_AFSR_DISABLE      0x40

/* GYRO_ACCEL_CONFIG0 — UI filters */
#define GYRO_UI_FILT_BW_LOW_LATENCY    (15 << 0)
#define ACCEL_UI_FILT_BW_LOW_LATENCY   (15 << 4)

/* INT_CONFIG */
#define INT1_MODE_PULSED               (0 << 2)
#define INT1_DRIVE_CIRCUIT_PP          (1 << 1)
#define INT1_POLARITY_ACTIVE_HIGH      (1 << 0)

/* INT_CONFIG0 */
#define UI_DRDY_INT_CLEAR_ON_SBR       (0 << 4)

/* INT_CONFIG1 */
#define INT_ASYNC_RESET_BIT            4
#define INT_TDEASSERT_DISABLE_BIT      5
#define INT_TPULSE_DURATION_BIT        6

/* INT_SOURCE0 */
#define UI_DRDY_INT1_EN_ENABLED        (1 << 3)

/* GYRO_CONFIG0 / ACCEL_CONFIG0 */
#define FS_SEL_2000DPS_16G             (0 << 5)
#define ODR_1KHZ                       6

/* WHO_AM_I */
#define ICM42688P_WHO_AM_I             0x47

/* ================================================================
 * 数据结构
 * ================================================================ */

typedef struct {
    int16_t accel_x, accel_y, accel_z;
    int16_t gyro_x,  gyro_y,  gyro_z;
    int16_t temperature;
} IMU_RawData;

typedef struct {
    float accel_x, accel_y, accel_z;  /* g */
    float gyro_x,  gyro_y,  gyro_z;  /* rad/s */
    float temperature;               /* °C */
} IMU_SensorData;

typedef struct {
    float roll, pitch, yaw;          /* degree */
} AttitudeData;

typedef enum {
    ICM42688_OK = 0,
    ICM42688_ERR_SPI,
    ICM42688_ERR_WHOAMI,
    ICM42688_ERR_NULL,
}ICM42688_Status;
/* ================================================================
 * API
 * ================================================================ */

ICM42688_Status ICM42688_Init(void);
uint8_t ICM42688_WhoAmI(void);
ICM42688_Status ICM42688_ReadRaw(IMU_RawData *raw);
void ICM42688_ConvertRaw(const IMU_RawData *raw, IMU_SensorData *sensor);
ICM42688_Status ICM42688_CalibrateBias(void);
void ICM42688_UpdateBias(const IMU_RawData *raw);

/* 全局 bias 供外部引用 */
extern float gyro_bias[3];

#endif /* ICM42688_H */
