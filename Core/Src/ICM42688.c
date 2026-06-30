#include "ICM42688.h"
#include "spi.h"
#include "main.h"
#include <math.h>
#include "gpio.h"

/* ================================================================
 * 全局变量
 * ================================================================ */

float gyro_bias[3] = {0.0f, 0.0f, 0.0f};

/* ================================================================
 * 底层 SPI 读写
 * CS = PA15 (manual), SPI3
 * ================================================================ */

static inline void cs_low(void)
{
    HAL_GPIO_WritePin(ICM42688_CS_GPIO_Port, ICM42688_CS_Pin, GPIO_PIN_RESET);
}

static inline void cs_high(void)
{
    HAL_GPIO_WritePin(ICM42688_CS_GPIO_Port, ICM42688_CS_Pin, GPIO_PIN_SET);
}

static ICM42688_Status write_reg(uint8_t reg, uint8_t data)
{
    uint8_t buf[2] = {reg & 0x7F, data};

    cs_low();
    HAL_StatusTypeDef ret = HAL_SPI_Transmit(&hspi3, buf, 2, 10);
    cs_high();

    if (ret != HAL_OK) {
        return ICM42688_ERR_SPI;
    }

    return ICM42688_OK;
}

static ICM42688_Status read_reg(uint8_t reg,uint8_t *data)
{
    if (data == NULL) {
        return ICM42688_ERR_NULL;
    }

    uint8_t tx[2] = {reg | 0x80, 0x00};
    uint8_t rx[2] = {0};
    cs_low();
    HAL_StatusTypeDef ret = HAL_SPI_TransmitReceive(&hspi3, tx, rx, 2, 10);
    cs_high();

    if (ret != HAL_OK) {
        return ICM42688_ERR_SPI;
    }

    *data = rx[1];
    return ICM42688_OK;
}

static ICM42688_Status write_bank(uint8_t bank)
{
    return write_reg(ICM42688_BANK_SEL, (bank & 0x07) << 4);
}

static ICM42688_Status write_mreg(uint8_t reg, uint8_t mask, uint8_t value)
{
    uint8_t orig = 0;
    ICM42688_Status status = read_reg(reg,&orig);
    if (status != ICM42688_OK) {
        return status;
    }
    uint8_t mod  = (orig & ~mask) | (value & mask);
    return write_reg(reg, mod);
}

/* ================================================================
 * 初始化 — 对标 Betaflight icm426xxGyroInit
 * ================================================================ */

ICM42688_Status ICM42688_Init(void)
{
    ICM42688_Status state;
    uint8_t who = 0;
    uint8_t v = 0;

    /* 1. 上电延时 */
    HAL_Delay(2);

    /* 2. 软复位 */
    state = write_bank(0);
    if (state != ICM42688_OK) return state;

    state = write_reg(ICM42688_DEVICE_CONFIG, DEVICE_CONFIG_SOFT_RESET);
    if (state != ICM42688_OK) return state;

    HAL_Delay(1);

    /* 3. 关 gyro+accel 以便配置 */
    state = write_bank(0);
    if (state != ICM42688_OK) return state;

    state = write_reg(ICM42688_PWR_MGMT0, 0x00);
    if (state != ICM42688_OK) return state;

    /* 4. 验证 WHO_AM_I */
    state = read_reg(ICM42688_WHO_AM_I, &who);
    if (state != ICM42688_OK) return state;

    if (who != ICM42688P_WHO_AM_I) {
        return ICM42688_ERR_WHOAMI;
    }

    /* 5. 禁用 AFSR (防 gyro 输出卡死)，保留 INTF_CONFIG1 其他位 */
    state = write_bank(0);
    if (state != ICM42688_OK) return state;

    state = write_mreg(ICM42688_INTF_CONFIG1,
                       INTF_CONFIG1_AFSR_MASK,
                       INTF_CONFIG1_AFSR_DISABLE);
    if (state != ICM42688_OK) return state;

    /* 6. Bank 1 — 配置 gyro Anti-Alias Filter (536Hz, 对标BF OPTION_1)
     *    ICM-42688P AAF 参数: delt=12, deltSq=144, bitshift=8 */
    state = write_bank(1);
    if (state != ICM42688_OK) return state;

    state = write_reg(ICM42688_GYRO_CONFIG_STATIC3, 12);
    if (state != ICM42688_OK) return state;

    state = write_reg(ICM42688_GYRO_CONFIG_STATIC4, 144 & 0xFF);
    if (state != ICM42688_OK) return state;

    state = write_reg(ICM42688_GYRO_CONFIG_STATIC5, ((144 >> 8) & 0x0F) | ((8 & 0x0F) << 4));
    if (state != ICM42688_OK) return state;

    /* 7. Bank 2 — 配置 accel Anti-Alias Filter (536Hz)
     *    注意 ACCEL_CONFIG_STATIC2 用 delt << 1 */
    state = write_bank(2);
    if (state != ICM42688_OK) return state;

    state = write_reg(ICM42688_ACCEL_CONFIG_STATIC2, 24);
    if (state != ICM42688_OK) return state;

    state = write_reg(ICM42688_ACCEL_CONFIG_STATIC3, 144 & 0xFF);
    if (state != ICM42688_OK) return state;

    state = write_reg(ICM42688_ACCEL_CONFIG_STATIC4, ((144 >> 8) & 0x0F) | ((8 & 0x0F) << 4));
    if (state != ICM42688_OK) return state;

    /* 8. Bank 0 — UI 滤波器 低延迟 */
    state = write_bank(0);
    if (state != ICM42688_OK) return state;

    state = write_reg(ICM42688_GYRO_ACCEL_CONFIG0,
                      GYRO_UI_FILT_BW_LOW_LATENCY | ACCEL_UI_FILT_BW_LOW_LATENCY);
    if (state != ICM42688_OK) return state;

    /* 9. 中断引脚配置 */
    state = write_reg(ICM42688_INT_CONFIG,
                      INT1_MODE_PULSED | INT1_DRIVE_CIRCUIT_PP | INT1_POLARITY_ACTIVE_HIGH);
    if (state != ICM42688_OK) return state;

    state = write_reg(ICM42688_INT_CONFIG0, UI_DRDY_INT_CLEAR_ON_SBR);
    if (state != ICM42688_OK) return state;

    /* 10. INT_CONFIG1: 清 ASYNC_RESET, 设脉冲宽度 8us */
    state = read_reg(ICM42688_INT_CONFIG1, &v);
    if (state != ICM42688_OK) return state;

    v &= ~(1 << INT_ASYNC_RESET_BIT);
    v |= (1 << INT_TPULSE_DURATION_BIT);
    v |= (1 << INT_TDEASSERT_DISABLE_BIT);
    state = write_reg(ICM42688_INT_CONFIG1, v);
    if (state != ICM42688_OK) return state;

    /* 11. 使能 INT1 数据就绪 */
    state = write_reg(ICM42688_INT_SOURCE0, UI_DRDY_INT1_EN_ENABLED);
    if (state != ICM42688_OK) return state;

    /* 12. 设置 GYRO_CONFIG0: FS=±2000dps, ODR=1kHz */
    state = write_reg(ICM42688_GYRO_CONFIG0, FS_SEL_2000DPS_16G | ODR_1KHZ);
    if (state != ICM42688_OK) return state;

    /* 13. 设置 ACCEL_CONFIG0: FS=±16g, ODR=1kHz */
    state = write_reg(ICM42688_ACCEL_CONFIG0, FS_SEL_2000DPS_16G | ODR_1KHZ);
    if (state != ICM42688_OK) return state;

    /* 14. 开启 gyro + accel (低噪声模式, 保留温度传感器) */
    state = write_reg(ICM42688_PWR_MGMT0,
                      PWR_MGMT0_GYRO_MODE_LN | PWR_MGMT0_ACCEL_MODE_LN);
    if (state != ICM42688_OK) return state;

    HAL_Delay(15);
    return ICM42688_OK;
}

uint8_t ICM42688_WhoAmI(void)
{
    uint8_t who = 0;

    if (write_bank(0) != ICM42688_OK) {
        return 0;
    }

    if (read_reg(ICM42688_WHO_AM_I,&who) != ICM42688_OK) {
        return 0;
    }

    return who;
}

/* ================================================================
 * 数据读取 — 从 TEMP_DATA1 (0x1D) 突发读 14 字节
 * 布局: temp[2] + accel_x[2] + accel_y[2] + accel_z[2]
 *               + gyro_x[2]  + gyro_y[2]  + gyro_z[2]
 * ================================================================ */

ICM42688_Status ICM42688_ReadRaw(IMU_RawData *raw)
{
    if (raw == NULL) {
        return ICM42688_ERR_NULL;
    }
    uint8_t tx[15] = {ICM42688_TEMP_DATA1 | 0x80};
    uint8_t rx[15] = {0};

    cs_low();
    HAL_StatusTypeDef ret = HAL_SPI_TransmitReceive(&hspi3, tx, rx, 15, 10);
    cs_high();

    if (ret != HAL_OK) {
        return ICM42688_ERR_SPI;
    }

    raw->temperature = (int16_t)((rx[1]  << 8) | rx[2]);
    raw->accel_x     = (int16_t)((rx[3]  << 8) | rx[4]);
    raw->accel_y     = (int16_t)((rx[5]  << 8) | rx[6]);
    raw->accel_z     = (int16_t)((rx[7]  << 8) | rx[8]);
    raw->gyro_x      = (int16_t)((rx[9]  << 8) | rx[10]);
    raw->gyro_y      = (int16_t)((rx[11] << 8) | rx[12]);
    raw->gyro_z      = (int16_t)((rx[13] << 8) | rx[14]);

    return ICM42688_OK;
}

/* ================================================================
 * 坐标系变换 + 物理单位转换
 * 安装方向: CW180_DEG_FLIP -> ( x, -y, -z)
 * 即: 飞控 X=+芯片X, 飞控 Y=-芯片Y, 飞控 Z=-芯片Z
 *
 * Scale: accel /2048 → g, gyro /16.4 → dps → *π/180 → rad/s
 * ================================================================ */

void ICM42688_ConvertRaw(const IMU_RawData *raw, IMU_SensorData *sensor)
{
    /* 坐标系映射保持右手系: FC_X=chip_X, FC_Y=-chip_Y, FC_Z=-chip_Z */
    /* ---- 加速度计 (g) + LPF fc≈50Hz ---- */
    static float ax_f = 0, ay_f = 0, az_f = 0;
    float ax =  raw->accel_x / 2048.0f;
    float ay =  raw->accel_y / 2048.0f;
    float az = -raw->accel_z / 2048.0f;
    ax_f += 0.24f * (ax - ax_f);
    ay_f += 0.24f * (ay - ay_f);
    az_f += 0.24f * (az - az_f);
    sensor->accel_x = ax_f;
    sensor->accel_y = ay_f;
    sensor->accel_z = az_f;

    /* ---- 陀螺仪: 减 bias, 转 dps, LPF, 转 rad/s ---- */
    static float gx_f=0, gy_f=0, gz_f=0;
    float gx = -(raw->gyro_x - gyro_bias[0]) / 16.4f;
    float gy = -(raw->gyro_y - gyro_bias[1]) / 16.4f;
    float gz =  (raw->gyro_z - gyro_bias[2]) / 16.4f;
    gx_f += 0.56f * (gx - gx_f);
    gy_f += 0.56f * (gy - gy_f);
    gz_f += 0.56f * (gz - gz_f);
    sensor->gyro_x =  gx_f * 0.017453293f;
    sensor->gyro_y =  gy_f * 0.017453293f;
    sensor->gyro_z =  gz_f * 0.017453293f;

    /* ---- 温度 (°C) ---- */
    sensor->temperature = raw->temperature / 132.48f + 25.0f;
}

/* ================================================================
 * Gyro bias 校准 — 静止状态采集 1000 帧，均值作为零偏
 * 前 200 帧丢弃（预热）
 * ================================================================ */
ICM42688_Status ICM42688_CalibrateBias(void)
{
    const uint16_t  WARMUP   = 200;
    const uint16_t  SAMPLES  = 1000;

    int64_t sum[3] = {0, 0, 0};
    IMU_RawData raw;

    /* 预热 — ODR=1kHz，每 1ms 一次 */
    for (uint16_t i = 0; i < WARMUP; i++) {
        ICM42688_ReadRaw(&raw);
        HAL_Delay(1);
    }

    const uint16_t MAX_ATTEMPTS = 2000;
    uint16_t valid = 0;
    uint16_t attempts = 0;

    while ((valid < SAMPLES) && (attempts < MAX_ATTEMPTS)) {
        attempts++;

        if (ICM42688_ReadRaw(&raw) == ICM42688_OK) {
            sum[0] += raw.gyro_x;
            sum[1] += raw.gyro_y;
            sum[2] += raw.gyro_z;
            valid++;
        }

        HAL_Delay(1);
    }

    if (valid < SAMPLES) {
        return ICM42688_ERR_SPI;
    }

    gyro_bias[0] = (float)sum[0] / valid;
    gyro_bias[1] = (float)sum[1] / valid;
    gyro_bias[2] = (float)sum[2] / valid;

    return ICM42688_OK;
}

/* ================================================================
 * 动态 bias 更新 — 静止时缓慢调整, 对标 YbImu 6轴方案
 * 静止判定: gyro 三轴幅度 < 3 deg/s 持续 100 帧
 * ================================================================ */
void ICM42688_UpdateBias(const IMU_RawData *raw)
{
    static uint32_t still_count = 0;
    static int64_t  sum[3] = {0, 0, 0};
    const  float    GYRO_LSB_PER_DPS = 16.4f;
    const  float    GYRO_THRESH_LSB  = 3.0f * GYRO_LSB_PER_DPS;
    const  float    GYRO_THRESH_SQ   = GYRO_THRESH_LSB * GYRO_THRESH_LSB;

    float gx = raw->gyro_x - gyro_bias[0];
    float gy = raw->gyro_y - gyro_bias[1];
    float gz = raw->gyro_z - gyro_bias[2];

    if ((gx*gx + gy*gy + gz*gz) < GYRO_THRESH_SQ) {
        sum[0] += raw->gyro_x;
        sum[1] += raw->gyro_y;
        sum[2] += raw->gyro_z;
        still_count++;

        /* 每累积 100 帧静止 → 更新 bias (指数滑动平均, α=0.01) */
        if (still_count >= 100) {
            float new_bx = (float)sum[0] / 100.0f;
            float new_by = (float)sum[1] / 100.0f;
            float new_bz = (float)sum[2] / 100.0f;
            gyro_bias[0] += 0.01f * (new_bx - gyro_bias[0]);
            gyro_bias[1] += 0.01f * (new_by - gyro_bias[1]);
            gyro_bias[2] += 0.01f * (new_bz - gyro_bias[2]);
            sum[0] = sum[1] = sum[2] = 0;
            still_count = 0;
        }
    } else {
        /* 运动 → 重置累计 */
        sum[0] = sum[1] = sum[2] = 0;
        still_count = 0;
    }
}
