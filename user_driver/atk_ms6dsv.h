#ifndef ATK_MS6DSV_H
#define ATK_MS6DSV_H

#include <stdint.h>

/* ATK-MS6DSV 以 SA0 电平选择 7 位 I2C 地址。 */
#define ATK_MS6DSV_ADDRESS_SA0_LOW       (0x6AU)
#define ATK_MS6DSV_ADDRESS_SA0_HIGH      (0x6BU)
#define ATK_MS6DSV_DEVICE_ID             (0x70U)

typedef struct {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
    int16_t temperature;
} ATK_MS6DSV_RawData;

/**
 * 初始化 ATK-MS6DSV。必须在 SYSCFG_DL_init() 之后调用且只调用一次。
 * 返回 1 表示识别和配置成功，返回 0 表示未找到模块或通信失败。
 */
uint8_t ATK_MS6DSV_Init(void);

/** 轮询读取一帧六轴和温度原始数据，成功返回 1。 */
uint8_t ATK_MS6DSV_Update(void);

/** 复制最近一次成功读取的原始数据。 */
void ATK_MS6DSV_GetRawData(ATK_MS6DSV_RawData *data);

/** 返回模块在线状态。 */
uint8_t ATK_MS6DSV_IsOnline(void);

/** 状态：0=未识别，1=数据有效，2=读取失败，3=配置失败。 */
uint8_t ATK_MS6DSV_GetStatus(void);

/** 返回 WHO_AM_I，正常值为 0x70。 */
uint8_t ATK_MS6DSV_GetWhoAmI(void);

/** 返回自动探测到的 7 位地址，正常为 0x6A 或 0x6B。 */
uint8_t ATK_MS6DSV_GetAddress(void);

/** ±2 g 配置下，将加速度原始值换算为微 g，避免使用浮点运算。 */
int32_t ATK_MS6DSV_AccelRawToUg(int16_t raw);

/** ±2000 dps 配置下，将角速度原始值换算为毫度每秒。 */
int32_t ATK_MS6DSV_GyroRawToMdps(int16_t raw);

/** 将温度原始值换算为 0.01 摄氏度。 */
int32_t ATK_MS6DSV_TemperatureRawToCentiDeg(int16_t raw);

#endif /* ATK_MS6DSV_H */
