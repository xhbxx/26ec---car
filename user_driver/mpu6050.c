#include "mpu6050.h"

#include "ti_msp_dl_config.h"

#define MPU6050_ADDRESS              (0x68U)
#define MPU6050_REG_SMPLRT_DIV       (0x19U)
#define MPU6050_REG_CONFIG           (0x1AU)
#define MPU6050_REG_GYRO_CONFIG      (0x1BU)
#define MPU6050_REG_ACCEL_CONFIG     (0x1CU)
#define MPU6050_REG_INT_ENABLE       (0x38U)
#define MPU6050_REG_ACCEL_XOUT_H     (0x3BU)
#define MPU6050_REG_PWR_MGMT_1       (0x6BU)
#define MPU6050_REG_WHO_AM_I         (0x75U)
#define MPU6050_WHO_AM_I_VALUE       (0x68U)
#define MPU6050_I2C_TIMEOUT_LOOPS    (100000U)

static volatile uint8_t g_mpu6050_data_ready = 0U;
static uint8_t g_mpu6050_online = 0U;
static MPU6050_RawData g_mpu6050_raw_data = {0};

/* Recover the controller after NACK, a disconnected sensor, or a bus timeout. */
static void MPU6050_ResetTransfer(void)
{
    DL_I2C_resetControllerTransfer(MPU6050_INST);
    DL_I2C_flushControllerTXFIFO(MPU6050_INST);
    DL_I2C_flushControllerRXFIFO(MPU6050_INST);
}

/* Wait for the controller state selected by status_mask without blocking forever. */
static uint8_t MPU6050_WaitForStatus(uint32_t status_mask)
{
    uint32_t timeout = MPU6050_I2C_TIMEOUT_LOOPS;
    uint32_t status;

    do {
        status = DL_I2C_getControllerStatus(MPU6050_INST);
        if ((status & DL_I2C_CONTROLLER_STATUS_ERROR) != 0U) {
            MPU6050_ResetTransfer();
            return 0U;
        }
        if ((status & status_mask) != 0U) {
            return 1U;
        }
    } while (--timeout != 0U);

    MPU6050_ResetTransfer();
    return 0U;
}

/* Write one MPU6050 register using the dedicated I2C1 controller. */
static uint8_t MPU6050_WriteRegister(uint8_t reg, uint8_t value)
{
    uint8_t tx_data[2] = {reg, value};

    if (MPU6050_WaitForStatus(DL_I2C_CONTROLLER_STATUS_IDLE) == 0U) {
        return 0U;
    }

    DL_I2C_fillControllerTXFIFO(MPU6050_INST, tx_data, 2U);
    DL_I2C_startControllerTransfer(MPU6050_INST, MPU6050_ADDRESS,
        DL_I2C_CONTROLLER_DIRECTION_TX, 2U);
    delay_cycles(8U); /* I2C_ERR_13 workaround required by MSPM0 DriverLib. */

    return MPU6050_WaitForStatus(DL_I2C_CONTROLLER_STATUS_IDLE);
}

/* Select a register, then receive consecutive bytes from the MPU6050. */
static uint8_t MPU6050_ReadRegisters(uint8_t reg, uint8_t *data, uint8_t length)
{
    uint32_t timeout;
    uint32_t status;
    uint8_t index;

    if ((data == 0) || (length == 0U)) {
        return 0U;
    }

    if (MPU6050_WaitForStatus(DL_I2C_CONTROLLER_STATUS_IDLE) == 0U) {
        return 0U;
    }

    DL_I2C_fillControllerTXFIFO(MPU6050_INST, &reg, 1U);
    DL_I2C_startControllerTransfer(MPU6050_INST, MPU6050_ADDRESS,
        DL_I2C_CONTROLLER_DIRECTION_TX, 1U);
    delay_cycles(8U);
    if (MPU6050_WaitForStatus(DL_I2C_CONTROLLER_STATUS_IDLE) == 0U) {
        return 0U;
    }

    DL_I2C_startControllerTransfer(MPU6050_INST, MPU6050_ADDRESS,
        DL_I2C_CONTROLLER_DIRECTION_RX, length);
    delay_cycles(8U);

    for (index = 0U; index < length; index++) {
        timeout = MPU6050_I2C_TIMEOUT_LOOPS;
        while (DL_I2C_isControllerRXFIFOEmpty(MPU6050_INST)) {
            status = DL_I2C_getControllerStatus(MPU6050_INST);
            if (((status & DL_I2C_CONTROLLER_STATUS_ERROR) != 0U) ||
                (--timeout == 0U)) {
                MPU6050_ResetTransfer();
                return 0U;
            }
        }
        data[index] = DL_I2C_receiveControllerData(MPU6050_INST);
    }

    return MPU6050_WaitForStatus(DL_I2C_CONTROLLER_STATUS_IDLE);
}

uint8_t MPU6050_Init(void)
{
    uint8_t who_am_i = 0U;

    g_mpu6050_online = 0U;
    g_mpu6050_data_ready = 0U;

    if ((MPU6050_ReadRegisters(MPU6050_REG_WHO_AM_I, &who_am_i, 1U) == 0U) ||
        (who_am_i != MPU6050_WHO_AM_I_VALUE)) {
        return 0U;
    }

    /* Reset, use X-axis gyro clock, sample at 100 Hz, and enable data-ready INT. */
    if (MPU6050_WriteRegister(MPU6050_REG_PWR_MGMT_1, 0x80U) == 0U) {
        return 0U;
    }
    DL_Common_delayCycles(CPUCLK_FREQ / 10U);

    if ((MPU6050_WriteRegister(MPU6050_REG_PWR_MGMT_1, 0x01U) == 0U) ||
        (MPU6050_WriteRegister(MPU6050_REG_SMPLRT_DIV, 9U) == 0U) ||
        (MPU6050_WriteRegister(MPU6050_REG_CONFIG, 0x03U) == 0U) ||
        (MPU6050_WriteRegister(MPU6050_REG_GYRO_CONFIG, 0x08U) == 0U) ||
        (MPU6050_WriteRegister(MPU6050_REG_ACCEL_CONFIG, 0x00U) == 0U) ||
        (MPU6050_WriteRegister(MPU6050_REG_INT_ENABLE, 0x01U) == 0U)) {
        return 0U;
    }

    g_mpu6050_online = 1U;
    return 1U;
}

void MPU6050_OnInterrupt(void)
{
    g_mpu6050_data_ready = 1U;
}

uint8_t MPU6050_Update(void)
{
    uint8_t bytes[14];

    if ((g_mpu6050_online == 0U) || (g_mpu6050_data_ready == 0U)) {
        return 0U;
    }
    g_mpu6050_data_ready = 0U;

    if (MPU6050_ReadRegisters(MPU6050_REG_ACCEL_XOUT_H, bytes, 14U) == 0U) {
        /* Keep the device online so the next data-ready interrupt can retry. */
        return 0U;
    }

    g_mpu6050_raw_data.accel_x = (int16_t)((bytes[0] << 8) | bytes[1]);
    g_mpu6050_raw_data.accel_y = (int16_t)((bytes[2] << 8) | bytes[3]);
    g_mpu6050_raw_data.accel_z = (int16_t)((bytes[4] << 8) | bytes[5]);
    g_mpu6050_raw_data.temperature = (int16_t)((bytes[6] << 8) | bytes[7]);
    g_mpu6050_raw_data.gyro_x = (int16_t)((bytes[8] << 8) | bytes[9]);
    g_mpu6050_raw_data.gyro_y = (int16_t)((bytes[10] << 8) | bytes[11]);
    g_mpu6050_raw_data.gyro_z = (int16_t)((bytes[12] << 8) | bytes[13]);
    return 1U;
}

void MPU6050_GetRawData(MPU6050_RawData *data)
{
    if (data != 0) {
        *data = g_mpu6050_raw_data;
    }
}

uint8_t MPU6050_IsOnline(void)
{
    return g_mpu6050_online;
}
