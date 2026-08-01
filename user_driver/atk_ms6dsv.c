#include "atk_ms6dsv.h"

#include "ti_msp_dl_config.h"

#define MS6DSV_REG_WHO_AM_I              (0x0FU)
#define MS6DSV_REG_CTRL1                 (0x10U)
#define MS6DSV_REG_CTRL2                 (0x11U)
#define MS6DSV_REG_CTRL3                 (0x12U)
#define MS6DSV_REG_CTRL6                 (0x15U)
#define MS6DSV_REG_CTRL8                 (0x17U)
#define MS6DSV_REG_OUT_TEMP_L            (0x20U)

#define MS6DSV_CTRL3_SW_RESET            (0x01U)
#define MS6DSV_CTRL3_IF_INC              (0x04U)
#define MS6DSV_CTRL3_BDU                 (0x40U)
#define MS6DSV_ODR_60_HZ                 (0x05U)
#define MS6DSV_GYRO_FS_2000_DPS          (0x04U)
#define MS6DSV_ACCEL_FS_2_G              (0x00U)

#define MS6DSV_RESET_TIMEOUT_LOOPS       (100U)
#define MS6DSV_RECONNECT_INTERVAL        (50U)
#define MS6DSV_MAX_READ_FAILURES         (10U)

/* Dedicated software I2C bus. OLED remains on PA0/PA1 hardware I2C0. */
#define MS6DSV_SCL_PORT                  GPIOA
#define MS6DSV_SCL_PIN                   DL_GPIO_PIN_15
#define MS6DSV_SCL_IOMUX                 IOMUX_PINCM37
#define MS6DSV_SDA_PORT                  GPIOA
#define MS6DSV_SDA_PIN                   DL_GPIO_PIN_16
#define MS6DSV_SDA_IOMUX                 IOMUX_PINCM38

static uint8_t g_ms6dsv_online;
static uint8_t g_ms6dsv_status;
static uint8_t g_ms6dsv_who_am_i;
static uint8_t g_ms6dsv_address = ATK_MS6DSV_ADDRESS_SA0_LOW;
static uint16_t g_ms6dsv_reconnect_count;
static uint8_t g_ms6dsv_read_fail_count;
static ATK_MS6DSV_RawData g_ms6dsv_raw_data;

static void ms6dsv_i2c_delay(void)
{
    /* The official driver uses a 2 us I2C half-period. */
    DL_Common_delayCycles(CPUCLK_FREQ / 500000U);
}

/* Both lines use open-drain behavior: drive low or release to the pull-up. */
static void ms6dsv_scl(uint8_t high)
{
    if (high != 0U) {
        DL_GPIO_disableOutput(MS6DSV_SCL_PORT, MS6DSV_SCL_PIN);
    } else {
        DL_GPIO_clearPins(MS6DSV_SCL_PORT, MS6DSV_SCL_PIN);
        DL_GPIO_enableOutput(MS6DSV_SCL_PORT, MS6DSV_SCL_PIN);
    }
}

static void ms6dsv_sda(uint8_t high)
{
    if (high != 0U) {
        DL_GPIO_disableOutput(MS6DSV_SDA_PORT, MS6DSV_SDA_PIN);
    } else {
        DL_GPIO_clearPins(MS6DSV_SDA_PORT, MS6DSV_SDA_PIN);
        DL_GPIO_enableOutput(MS6DSV_SDA_PORT, MS6DSV_SDA_PIN);
    }
}

static uint8_t ms6dsv_read_sda(void)
{
    return ((DL_GPIO_readPins(MS6DSV_SDA_PORT, MS6DSV_SDA_PIN) &
        MS6DSV_SDA_PIN) != 0U) ? 1U : 0U;
}

static uint8_t ms6dsv_read_scl(void)
{
    return ((DL_GPIO_readPins(MS6DSV_SCL_PORT, MS6DSV_SCL_PIN) &
        MS6DSV_SCL_PIN) != 0U) ? 1U : 0U;
}

static void ms6dsv_i2c_start(void)
{
    ms6dsv_sda(1U);
    ms6dsv_scl(1U);
    ms6dsv_i2c_delay();
    ms6dsv_sda(0U);
    ms6dsv_i2c_delay();
    ms6dsv_scl(0U);
    ms6dsv_i2c_delay();
}

static void ms6dsv_i2c_stop(void)
{
    ms6dsv_sda(0U);
    ms6dsv_i2c_delay();
    ms6dsv_scl(1U);
    ms6dsv_i2c_delay();
    ms6dsv_sda(1U);
    ms6dsv_i2c_delay();
}

static uint8_t ms6dsv_i2c_send_byte(uint8_t value)
{
    uint8_t bit;
    uint8_t acknowledged;

    for (bit = 0U; bit < 8U; bit++) {
        ms6dsv_sda((value & 0x80U) != 0U ? 1U : 0U);
        ms6dsv_i2c_delay();
        ms6dsv_scl(1U);
        ms6dsv_i2c_delay();
        ms6dsv_scl(0U);
        value <<= 1;
    }
    ms6dsv_sda(1U);
    ms6dsv_i2c_delay();
    ms6dsv_scl(1U);
    ms6dsv_i2c_delay();
    acknowledged = (uint8_t)(ms6dsv_read_sda() == 0U);
    ms6dsv_scl(0U);
    ms6dsv_i2c_delay();
    return acknowledged;
}

static uint8_t ms6dsv_i2c_read_byte(uint8_t acknowledge)
{
    uint8_t bit;
    uint8_t value = 0U;

    ms6dsv_sda(1U);
    for (bit = 0U; bit < 8U; bit++) {
        value <<= 1;
        ms6dsv_scl(1U);
        ms6dsv_i2c_delay();
        value |= ms6dsv_read_sda();
        ms6dsv_scl(0U);
        ms6dsv_i2c_delay();
    }
    ms6dsv_sda(acknowledge != 0U ? 0U : 1U);
    ms6dsv_i2c_delay();
    ms6dsv_scl(1U);
    ms6dsv_i2c_delay();
    ms6dsv_scl(0U);
    ms6dsv_sda(1U);
    ms6dsv_i2c_delay();
    return value;
}

static uint8_t ms6dsv_i2c_gpio_init(void)
{
    uint8_t pulse;

    DL_I2C_disableController(MS6DSV_INST);
    /* Input mode enables INENA; output-enable is toggled only to pull low. */
    DL_GPIO_initDigitalInputFeatures(MS6DSV_SCL_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(MS6DSV_SDA_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    ms6dsv_sda(1U);
    ms6dsv_scl(1U);
    ms6dsv_i2c_delay();
    if (ms6dsv_read_scl() == 0U) {
        g_ms6dsv_status = 7U;
        return 0U;
    }

    /* Recover a slave left halfway through a transfer. */
    for (pulse = 0U; pulse < 9U; pulse++) {
        ms6dsv_scl(0U);
        ms6dsv_i2c_delay();
        ms6dsv_scl(1U);
        ms6dsv_i2c_delay();
    }
    ms6dsv_i2c_stop();
    if (ms6dsv_read_scl() == 0U) {
        g_ms6dsv_status = 7U;
        return 0U;
    }
    if (ms6dsv_read_sda() == 0U) {
        g_ms6dsv_status = 6U;
        return 0U;
    }
    return 1U;
}

static uint8_t ms6dsv_read_registers(
    uint8_t reg, uint8_t *data, uint8_t length)
{
    uint8_t index;

    if ((data == 0) || (length == 0U)) {
        return 0U;
    }
    ms6dsv_i2c_start();
    if ((ms6dsv_i2c_send_byte((uint8_t)(g_ms6dsv_address << 1)) == 0U) ||
        (ms6dsv_i2c_send_byte(reg) == 0U)) {
        ms6dsv_i2c_stop();
        return 0U;
    }
    ms6dsv_i2c_start();
    if (ms6dsv_i2c_send_byte(
            (uint8_t)((g_ms6dsv_address << 1) | 1U)) == 0U) {
        ms6dsv_i2c_stop();
        return 0U;
    }
    for (index = 0U; index < length; index++) {
        data[index] = ms6dsv_i2c_read_byte(
            (uint8_t)(index + 1U < length));
    }
    ms6dsv_i2c_stop();
    return 1U;
}

static uint8_t ms6dsv_write_register(uint8_t reg, uint8_t value)
{
    ms6dsv_i2c_start();
    if ((ms6dsv_i2c_send_byte((uint8_t)(g_ms6dsv_address << 1)) == 0U) ||
        (ms6dsv_i2c_send_byte(reg) == 0U) ||
        (ms6dsv_i2c_send_byte(value) == 0U)) {
        ms6dsv_i2c_stop();
        return 0U;
    }
    ms6dsv_i2c_stop();
    return 1U;
}

static uint8_t ms6dsv_update_register(
    uint8_t reg, uint8_t clear_mask, uint8_t set_mask)
{
    uint8_t value;

    if (ms6dsv_read_registers(reg, &value, 1U) == 0U) {
        return 0U;
    }
    value = (uint8_t)((value & (uint8_t)(~clear_mask)) | set_mask);
    return ms6dsv_write_register(reg, value);
}

uint8_t ATK_MS6DSV_Init(void)
{
    static const uint8_t addresses[2] = {
        ATK_MS6DSV_ADDRESS_SA0_LOW, ATK_MS6DSV_ADDRESS_SA0_HIGH
    };
    uint8_t address_index;
    uint8_t retry;
    uint8_t ctrl3;
    uint8_t received_response = 0U;
    uint16_t reset_timeout;

    g_ms6dsv_online = 0U;
    g_ms6dsv_status = 0U;
    g_ms6dsv_who_am_i = 0U;
    if (ms6dsv_i2c_gpio_init() == 0U) {
        return 0U;
    }

    /* The device only needs a short power-up settling time here. */
    DL_Common_delayCycles(CPUCLK_FREQ / 20U);
    for (address_index = 0U; address_index < 2U; address_index++) {
        g_ms6dsv_address = addresses[address_index];
        for (retry = 0U; retry < 3U; retry++) {
            if (ms6dsv_read_registers(MS6DSV_REG_WHO_AM_I,
                    &g_ms6dsv_who_am_i, 1U) != 0U) {
                received_response = 1U;
                if (g_ms6dsv_who_am_i == ATK_MS6DSV_DEVICE_ID) {
                    break;
                }
            }
            DL_Common_delayCycles(CPUCLK_FREQ / 100U);
        }
        if (g_ms6dsv_who_am_i == ATK_MS6DSV_DEVICE_ID) {
            break;
        }
    }
    if (g_ms6dsv_who_am_i != ATK_MS6DSV_DEVICE_ID) {
        g_ms6dsv_status = (received_response != 0U) ? 5U : 4U;
        return 0U;
    }

    if (ms6dsv_write_register(MS6DSV_REG_CTRL3,
            MS6DSV_CTRL3_SW_RESET) == 0U) {
        g_ms6dsv_status = 3U;
        return 0U;
    }
    reset_timeout = MS6DSV_RESET_TIMEOUT_LOOPS;
    do {
        DL_Common_delayCycles(CPUCLK_FREQ / 1000U);
        if (ms6dsv_read_registers(MS6DSV_REG_CTRL3, &ctrl3, 1U) == 0U) {
            g_ms6dsv_status = 3U;
            return 0U;
        }
    } while (((ctrl3 & MS6DSV_CTRL3_SW_RESET) != 0U) &&
             (--reset_timeout != 0U));
    if (reset_timeout == 0U) {
        g_ms6dsv_status = 3U;
        return 0U;
    }

    if ((ms6dsv_update_register(MS6DSV_REG_CTRL3,
             MS6DSV_CTRL3_IF_INC | MS6DSV_CTRL3_BDU,
             MS6DSV_CTRL3_IF_INC | MS6DSV_CTRL3_BDU) == 0U) ||
        (ms6dsv_update_register(MS6DSV_REG_CTRL8,
             0x03U, MS6DSV_ACCEL_FS_2_G) == 0U) ||
        (ms6dsv_update_register(MS6DSV_REG_CTRL6,
             0x0FU, MS6DSV_GYRO_FS_2000_DPS) == 0U) ||
        (ms6dsv_update_register(MS6DSV_REG_CTRL1,
             0x0FU, MS6DSV_ODR_60_HZ) == 0U) ||
        (ms6dsv_update_register(MS6DSV_REG_CTRL2,
             0x0FU, MS6DSV_ODR_60_HZ) == 0U)) {
        g_ms6dsv_status = 3U;
        return 0U;
    }

    g_ms6dsv_online = 1U;
    g_ms6dsv_status = 1U;
    g_ms6dsv_reconnect_count = 0U;
    g_ms6dsv_read_fail_count = 0U;
    return 1U;
}

uint8_t ATK_MS6DSV_Update(void)
{
    uint8_t bytes[14];

    if (g_ms6dsv_online == 0U) {
        g_ms6dsv_reconnect_count++;
        if (g_ms6dsv_reconnect_count >= MS6DSV_RECONNECT_INTERVAL) {
            g_ms6dsv_reconnect_count = 0U;
            (void)ATK_MS6DSV_Init();
        }
        return 0U;
    }
    if (ms6dsv_read_registers(MS6DSV_REG_OUT_TEMP_L, bytes, 14U) == 0U) {
        g_ms6dsv_status = 2U;
        g_ms6dsv_read_fail_count++;
        if (g_ms6dsv_read_fail_count >= MS6DSV_MAX_READ_FAILURES) {
            g_ms6dsv_online = 0U;
            g_ms6dsv_reconnect_count = MS6DSV_RECONNECT_INTERVAL;
        }
        return 0U;
    }

    g_ms6dsv_raw_data.temperature =
        (int16_t)(((uint16_t)bytes[1] << 8) | bytes[0]);
    g_ms6dsv_raw_data.gyro_x =
        (int16_t)(((uint16_t)bytes[3] << 8) | bytes[2]);
    g_ms6dsv_raw_data.gyro_y =
        (int16_t)(((uint16_t)bytes[5] << 8) | bytes[4]);
    g_ms6dsv_raw_data.gyro_z =
        (int16_t)(((uint16_t)bytes[7] << 8) | bytes[6]);
    g_ms6dsv_raw_data.accel_x =
        (int16_t)(((uint16_t)bytes[9] << 8) | bytes[8]);
    g_ms6dsv_raw_data.accel_y =
        (int16_t)(((uint16_t)bytes[11] << 8) | bytes[10]);
    g_ms6dsv_raw_data.accel_z =
        (int16_t)(((uint16_t)bytes[13] << 8) | bytes[12]);

    g_ms6dsv_status = 1U;
    g_ms6dsv_read_fail_count = 0U;
    return 1U;
}

void ATK_MS6DSV_GetRawData(ATK_MS6DSV_RawData *data)
{
    if (data != 0) {
        *data = g_ms6dsv_raw_data;
    }
}

uint8_t ATK_MS6DSV_IsOnline(void)
{
    return g_ms6dsv_online;
}

uint8_t ATK_MS6DSV_GetStatus(void)
{
    return g_ms6dsv_status;
}

uint8_t ATK_MS6DSV_GetWhoAmI(void)
{
    return g_ms6dsv_who_am_i;
}

uint8_t ATK_MS6DSV_GetAddress(void)
{
    return g_ms6dsv_address;
}

int32_t ATK_MS6DSV_AccelRawToUg(int16_t raw)
{
    return (int32_t)raw * 61L;
}

int32_t ATK_MS6DSV_GyroRawToMdps(int16_t raw)
{
    return (int32_t)raw * 70L;
}

int32_t ATK_MS6DSV_TemperatureRawToCentiDeg(int16_t raw)
{
    return 2500L + (((int32_t)raw * 100L) / 256L);
}
