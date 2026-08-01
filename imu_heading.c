#include "imu_heading.h"

#include "atk_ms6dsv.h"
#include "car_config.h"

static int32_t g_bias_sum;
static uint16_t g_bias_samples;
static int16_t g_bias_raw;
static uint8_t g_ready;
static uint32_t g_last_update_ms;
static int32_t g_angle_mdeg;
static int32_t g_rate_mdps;

void ImuHeading_Init(uint32_t now_ms)
{
    g_bias_sum = 0;
    g_bias_samples = 0U;
    g_bias_raw = 0;
    g_ready = 0U;
    g_angle_mdeg = 0;
    g_rate_mdps = 0;
    g_last_update_ms = now_ms;
    (void)ATK_MS6DSV_Init();
}

void ImuHeading_Update(uint32_t now_ms)
{
    ATK_MS6DSV_RawData raw;
    uint32_t elapsed_ms = now_ms - g_last_update_ms;
    int32_t corrected_raw;

    if (elapsed_ms < IMU_UPDATE_PERIOD_MS) {
        return;
    }
    g_last_update_ms = now_ms;

    if (ATK_MS6DSV_Update() == 0U) {
        g_rate_mdps = 0;
        return;
    }
    ATK_MS6DSV_GetRawData(&raw);

    if (g_ready == 0U) {
        g_bias_sum += raw.gyro_z;
        g_bias_samples++;
        if (g_bias_samples >= IMU_CALIBRATION_SAMPLES) {
            g_bias_raw = (int16_t)(g_bias_sum /
                (int32_t)IMU_CALIBRATION_SAMPLES);
            g_ready = 1U;
            g_angle_mdeg = 0;
        }
        return;
    }

    corrected_raw = ((int32_t)raw.gyro_z - g_bias_raw) *
        GYRO_Z_CLOCKWISE_SIGN;
    if ((corrected_raw < IMU_GYRO_DEADBAND_RAW) &&
        (corrected_raw > -IMU_GYRO_DEADBAND_RAW)) {
        corrected_raw = 0;
    }
    g_rate_mdps = corrected_raw * 70L;
    g_angle_mdeg += (int32_t)(((int64_t)g_rate_mdps * elapsed_ms) / 1000LL);
}

void ImuHeading_ResetAngle(void)
{
    g_angle_mdeg = 0;
}

uint8_t ImuHeading_IsReady(void)
{
    return g_ready;
}

uint8_t ImuHeading_IsOnline(void)
{
    return ATK_MS6DSV_IsOnline();
}

int32_t ImuHeading_GetAngleMdeg(void)
{
    return g_angle_mdeg;
}

int32_t ImuHeading_GetRateMdps(void)
{
    return g_rate_mdps;
}

int16_t ImuHeading_GetBiasRaw(void)
{
    return g_bias_raw;
}
