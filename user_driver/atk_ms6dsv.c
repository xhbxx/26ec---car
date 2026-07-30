#include "atk_ms6dsv.h"

#include "ti_msp_dl_config.h"

/* LSM6DSV16X 主寄存器页中本工程需要使用的寄存器。 */
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

#define MS6DSV_I2C_TIMEOUT_LOOPS         (100000U)
#define MS6DSV_RESET_TIMEOUT_LOOPS       (100U)
#define MS6DSV_RECONNECT_INTERVAL        (200U)
#define MS6DSV_MAX_READ_FAILURES         (10U)

static uint8_t g_ms6dsv_online = 0U;
static uint8_t g_ms6dsv_status = 0U;
static uint8_t g_ms6dsv_who_am_i = 0U;
static uint8_t g_ms6dsv_address = ATK_MS6DSV_ADDRESS_SA0_LOW;
static uint16_t g_ms6dsv_reconnect_count = 0U;
static uint8_t g_ms6dsv_read_fail_count = 0U;
static ATK_MS6DSV_RawData g_ms6dsv_raw_data = {0};

/** 复位 I2C 控制器的本次传输，防止 NACK 后总线一直保持忙状态。 */
static void ms6dsv_reset_transfer(void)
{
    DL_I2C_resetControllerTransfer(MS6DSV_INST);
    DL_I2C_flushControllerTXFIFO(MS6DSV_INST);
    DL_I2C_flushControllerRXFIFO(MS6DSV_INST);
}

/** 带超时等待 I2C 控制器进入指定状态。 */
static uint8_t ms6dsv_wait_for_status(uint32_t status_mask)
{
    uint32_t timeout = MS6DSV_I2C_TIMEOUT_LOOPS;
    uint32_t status;

    do {
        status = DL_I2C_getControllerStatus(MS6DSV_INST);
        if ((status & DL_I2C_CONTROLLER_STATUS_ERROR) != 0U) {
            ms6dsv_reset_transfer();
            return 0U;
        }
        if ((status & status_mask) != 0U) {
            return 1U;
        }
    } while (--timeout != 0U);

    ms6dsv_reset_transfer();
    return 0U;
}

/** 等待 START、数据和 STOP 全部完成。 */
static uint8_t ms6dsv_wait_transfer_done(void)
{
    uint32_t timeout = MS6DSV_I2C_TIMEOUT_LOOPS;
    uint32_t status;

    do {
        status = DL_I2C_getControllerStatus(MS6DSV_INST);
        if ((status & DL_I2C_CONTROLLER_STATUS_ERROR) != 0U) {
            ms6dsv_reset_transfer();
            return 0U;
        }
        if ((status & DL_I2C_CONTROLLER_STATUS_BUSY) == 0U) {
            break;
        }
    } while (--timeout != 0U);

    if (timeout == 0U) {
        ms6dsv_reset_transfer();
        return 0U;
    }
    return ms6dsv_wait_for_status(DL_I2C_CONTROLLER_STATUS_IDLE);
}

/** 向一个 LSM6DSV16X 寄存器写入一个字节。 */
static uint8_t ms6dsv_write_register(uint8_t reg, uint8_t value)
{
    uint8_t tx_data[2] = {reg, value};

    if (ms6dsv_wait_for_status(DL_I2C_CONTROLLER_STATUS_IDLE) == 0U) {
        return 0U;
    }

    DL_I2C_fillControllerTXFIFO(MS6DSV_INST, tx_data, 2U);
    DL_I2C_startControllerTransfer(MS6DSV_INST, g_ms6dsv_address,
        DL_I2C_CONTROLLER_DIRECTION_TX, 2U);
    delay_cycles(8U); /* MSPM0 DriverLib I2C_ERR_13 规避要求。 */
    return ms6dsv_wait_transfer_done();
}

/** 从指定寄存器开始连续读取，CTRL3.IF_INC 负责自动递增地址。 */
static uint8_t ms6dsv_read_registers(
    uint8_t reg, uint8_t *data, uint8_t length)
{
    uint32_t timeout;
    uint32_t status;
    uint8_t index;

    if ((data == 0) || (length == 0U)) {
        return 0U;
    }
    if (ms6dsv_wait_for_status(DL_I2C_CONTROLLER_STATUS_IDLE) == 0U) {
        return 0U;
    }

    DL_I2C_transmitControllerData(MS6DSV_INST, reg);
    DL_I2C_startControllerTransfer(MS6DSV_INST, g_ms6dsv_address,
        DL_I2C_CONTROLLER_DIRECTION_TX, 1U);
    delay_cycles(8U);
    if (ms6dsv_wait_transfer_done() == 0U) {
        return 0U;
    }

    DL_I2C_startControllerTransfer(MS6DSV_INST, g_ms6dsv_address,
        DL_I2C_CONTROLLER_DIRECTION_RX, length);
    delay_cycles(8U);

    for (index = 0U; index < length; index++) {
        timeout = MS6DSV_I2C_TIMEOUT_LOOPS;
        while (DL_I2C_isControllerRXFIFOEmpty(MS6DSV_INST)) {
            status = DL_I2C_getControllerStatus(MS6DSV_INST);
            if (((status & DL_I2C_CONTROLLER_STATUS_ERROR) != 0U) ||
                (--timeout == 0U)) {
                ms6dsv_reset_transfer();
                return 0U;
            }
        }
        data[index] = DL_I2C_receiveControllerData(MS6DSV_INST);
    }
    return ms6dsv_wait_transfer_done();
}

/** 修改寄存器中的指定位，保留其余位不变。 */
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
    uint16_t reset_timeout;

    g_ms6dsv_online = 0U;
    g_ms6dsv_status = 0U;
    g_ms6dsv_who_am_i = 0U;

    /* 模块上电可能比 MCU 慢，延时后自动探测 SA0 的两种地址。 */
    DL_Common_delayCycles(CPUCLK_FREQ / 5U);
    for (address_index = 0U; address_index < 2U; address_index++) {
        g_ms6dsv_address = addresses[address_index];
        for (retry = 0U; retry < 3U; retry++) {
            if ((ms6dsv_read_registers(MS6DSV_REG_WHO_AM_I,
                    &g_ms6dsv_who_am_i, 1U) != 0U) &&
                (g_ms6dsv_who_am_i == ATK_MS6DSV_DEVICE_ID)) {
                break;
            }
            DL_Common_delayCycles(CPUCLK_FREQ / 100U);
        }
        if (g_ms6dsv_who_am_i == ATK_MS6DSV_DEVICE_ID) {
            break;
        }
    }
    if (g_ms6dsv_who_am_i != ATK_MS6DSV_DEVICE_ID) {
        return 0U;
    }

    /* 官方初始化顺序：软件复位，等待复位完成，再使能 BDU 和地址自增。 */
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

    /* 0x20 起依次为温度、陀螺仪 XYZ、加速度 XYZ，均为小端有符号数。 */
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
