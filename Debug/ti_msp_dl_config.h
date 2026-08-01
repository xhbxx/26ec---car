/*
 * Copyright (c) 2023, Texas Instruments Incorporated - http://www.ti.com
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ============ ti_msp_dl_config.h =============
 *  Configured MSPM0 DriverLib module declarations
 *
 *  DO NOT EDIT - This file is generated for the MSPM0G350X
 *  by the SysConfig tool.
 */
#ifndef ti_msp_dl_config_h
#define ti_msp_dl_config_h

#define CONFIG_MSPM0G350X
#define CONFIG_MSPM0G3507

#if defined(__ti_version__) || defined(__TI_COMPILER_VERSION__)
#define SYSCONFIG_WEAK __attribute__((weak))
#elif defined(__IAR_SYSTEMS_ICC__)
#define SYSCONFIG_WEAK __weak
#elif defined(__GNUC__)
#define SYSCONFIG_WEAK __attribute__((weak))
#endif

#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>
#include <ti/driverlib/m0p/dl_core.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 *  ======== SYSCFG_DL_init ========
 *  Perform all required MSP DL initialization
 *
 *  This function should be called once at a point before any use of
 *  MSP DL.
 */


/* clang-format off */

#define POWER_STARTUP_DELAY                                                (16)


#define GPIO_HFXT_PORT                                                     GPIOA
#define GPIO_HFXIN_PIN                                             DL_GPIO_PIN_5
#define GPIO_HFXIN_IOMUX                                         (IOMUX_PINCM10)
#define GPIO_HFXOUT_PIN                                            DL_GPIO_PIN_6
#define GPIO_HFXOUT_IOMUX                                        (IOMUX_PINCM11)
#define CPUCLK_FREQ                                                     80000000
/* Defines for SYSPLL_ERR_01 Workaround */
/* Represent 1.000 as 1000 */
#define FLOAT_TO_INT_SCALE                                               (1000U)
#define FCC_EXPECTED_RATIO                                                  2500
#define FCC_UPPER_BOUND                       (FCC_EXPECTED_RATIO * (1 + 0.003))
#define FCC_LOWER_BOUND                       (FCC_EXPECTED_RATIO * (1 - 0.003))

bool SYSCFG_DL_SYSCTL_SYSPLL_init(void);


/* Defines for PWMAB */
#define PWMAB_INST                                                         TIMG0
#define PWMAB_INST_IRQHandler                                   TIMG0_IRQHandler
#define PWMAB_INST_INT_IRQN                                     (TIMG0_INT_IRQn)
#define PWMAB_INST_CLK_FREQ                                             40000000
/* GPIO defines for channel 0 */
#define GPIO_PWMAB_C0_PORT                                                 GPIOA
#define GPIO_PWMAB_C0_PIN                                         DL_GPIO_PIN_12
#define GPIO_PWMAB_C0_IOMUX                                      (IOMUX_PINCM34)
#define GPIO_PWMAB_C0_IOMUX_FUNC                     IOMUX_PINCM34_PF_TIMG0_CCP0
#define GPIO_PWMAB_C0_IDX                                    DL_TIMER_CC_0_INDEX
/* GPIO defines for channel 1 */
#define GPIO_PWMAB_C1_PORT                                                 GPIOA
#define GPIO_PWMAB_C1_PIN                                         DL_GPIO_PIN_13
#define GPIO_PWMAB_C1_IOMUX                                      (IOMUX_PINCM35)
#define GPIO_PWMAB_C1_IOMUX_FUNC                     IOMUX_PINCM35_PF_TIMG0_CCP1
#define GPIO_PWMAB_C1_IDX                                    DL_TIMER_CC_1_INDEX



/* Defines for MOTOR_PID */
#define MOTOR_PID_INST                                                   (TIMA0)
#define MOTOR_PID_INST_IRQHandler                               TIMA0_IRQHandler
#define MOTOR_PID_INST_INT_IRQN                                 (TIMA0_INT_IRQn)
#define MOTOR_PID_INST_LOAD_VALUE                                       (39999U)




/* Defines for OLED */
#define OLED_INST                                                           I2C0
#define OLED_INST_IRQHandler                                     I2C0_IRQHandler
#define OLED_INST_INT_IRQN                                         I2C0_INT_IRQn
#define OLED_BUS_SPEED_HZ                                                 100000
#define GPIO_OLED_SDA_PORT                                                 GPIOA
#define GPIO_OLED_SDA_PIN                                          DL_GPIO_PIN_0
#define GPIO_OLED_IOMUX_SDA                                       (IOMUX_PINCM1)
#define GPIO_OLED_IOMUX_SDA_FUNC                        IOMUX_PINCM1_PF_I2C0_SDA
#define GPIO_OLED_SCL_PORT                                                 GPIOA
#define GPIO_OLED_SCL_PIN                                          DL_GPIO_PIN_1
#define GPIO_OLED_IOMUX_SCL                                       (IOMUX_PINCM2)
#define GPIO_OLED_IOMUX_SCL_FUNC                        IOMUX_PINCM2_PF_I2C0_SCL

/* Defines for MS6DSV */
#define MS6DSV_INST                                                         I2C1
#define MS6DSV_INST_IRQHandler                                   I2C1_IRQHandler
#define MS6DSV_INST_INT_IRQN                                       I2C1_INT_IRQn
#define MS6DSV_BUS_SPEED_HZ                                               100000
#define GPIO_MS6DSV_SDA_PORT                                               GPIOA
#define GPIO_MS6DSV_SDA_PIN                                       DL_GPIO_PIN_16
#define GPIO_MS6DSV_IOMUX_SDA                                    (IOMUX_PINCM38)
#define GPIO_MS6DSV_IOMUX_SDA_FUNC                     IOMUX_PINCM38_PF_I2C1_SDA
#define GPIO_MS6DSV_SCL_PORT                                               GPIOA
#define GPIO_MS6DSV_SCL_PIN                                       DL_GPIO_PIN_15
#define GPIO_MS6DSV_IOMUX_SCL                                    (IOMUX_PINCM37)
#define GPIO_MS6DSV_IOMUX_SCL_FUNC                     IOMUX_PINCM37_PF_I2C1_SCL


/* Defines for TELEMETRY_UART */
#define TELEMETRY_UART_INST                                                UART0
#define TELEMETRY_UART_INST_FREQUENCY                                   40000000
#define TELEMETRY_UART_INST_IRQHandler                          UART0_IRQHandler
#define TELEMETRY_UART_INST_INT_IRQN                              UART0_INT_IRQn
#define GPIO_TELEMETRY_UART_RX_PORT                                        GPIOA
#define GPIO_TELEMETRY_UART_TX_PORT                                        GPIOA
#define GPIO_TELEMETRY_UART_RX_PIN                                DL_GPIO_PIN_11
#define GPIO_TELEMETRY_UART_TX_PIN                                DL_GPIO_PIN_10
#define GPIO_TELEMETRY_UART_IOMUX_RX                             (IOMUX_PINCM22)
#define GPIO_TELEMETRY_UART_IOMUX_TX                             (IOMUX_PINCM21)
#define GPIO_TELEMETRY_UART_IOMUX_RX_FUNC               IOMUX_PINCM22_PF_UART0_RX
#define GPIO_TELEMETRY_UART_IOMUX_TX_FUNC               IOMUX_PINCM21_PF_UART0_TX
#define TELEMETRY_UART_BAUD_RATE                                        (115200)
#define TELEMETRY_UART_IBRD_40_MHZ_115200_BAUD                              (21)
#define TELEMETRY_UART_FBRD_40_MHZ_115200_BAUD                              (45)





/* Port definition for Pin Group LED */
#define LED_PORT                                                         (GPIOA)

/* Defines for STATUS: GPIOA.22 with pinCMx 47 on package pin 18 */
#define LED_STATUS_PIN                                          (DL_GPIO_PIN_22)
#define LED_STATUS_IOMUX                                         (IOMUX_PINCM47)
/* Defines for IR_1: GPIOA.14 with pinCMx 36 on package pin 7 */
#define Infrared_IR_1_PORT                                               (GPIOA)
#define Infrared_IR_1_PIN                                       (DL_GPIO_PIN_14)
#define Infrared_IR_1_IOMUX                                      (IOMUX_PINCM36)
/* Defines for IR_2: GPIOB.16 with pinCMx 33 on package pin 4 */
#define Infrared_IR_2_PORT                                               (GPIOB)
#define Infrared_IR_2_PIN                                       (DL_GPIO_PIN_16)
#define Infrared_IR_2_IOMUX                                      (IOMUX_PINCM33)
/* Defines for IR_3: GPIOB.14 with pinCMx 31 on package pin 2 */
#define Infrared_IR_3_PORT                                               (GPIOB)
#define Infrared_IR_3_PIN                                       (DL_GPIO_PIN_14)
#define Infrared_IR_3_IOMUX                                      (IOMUX_PINCM31)
/* Defines for IR_4: GPIOB.15 with pinCMx 32 on package pin 3 */
#define Infrared_IR_4_PORT                                               (GPIOB)
#define Infrared_IR_4_PIN                                       (DL_GPIO_PIN_15)
#define Infrared_IR_4_IOMUX                                      (IOMUX_PINCM32)
/* Defines for AIN1: GPIOA.9 with pinCMx 20 on package pin 55 */
#define DC_MOTOR_AIN1_PORT                                               (GPIOA)
#define DC_MOTOR_AIN1_PIN                                        (DL_GPIO_PIN_9)
#define DC_MOTOR_AIN1_IOMUX                                      (IOMUX_PINCM20)
/* Defines for AIN2: GPIOA.8 with pinCMx 19 on package pin 54 */
#define DC_MOTOR_AIN2_PORT                                               (GPIOA)
#define DC_MOTOR_AIN2_PIN                                        (DL_GPIO_PIN_8)
#define DC_MOTOR_AIN2_IOMUX                                      (IOMUX_PINCM19)
/* Defines for STBY: GPIOB.24 with pinCMx 52 on package pin 23 */
#define DC_MOTOR_STBY_PORT                                               (GPIOB)
#define DC_MOTOR_STBY_PIN                                       (DL_GPIO_PIN_24)
#define DC_MOTOR_STBY_IOMUX                                      (IOMUX_PINCM52)
/* Defines for BIN1: GPIOA.7 with pinCMx 14 on package pin 49 */
#define DC_MOTOR_BIN1_PORT                                               (GPIOA)
#define DC_MOTOR_BIN1_PIN                                        (DL_GPIO_PIN_7)
#define DC_MOTOR_BIN1_IOMUX                                      (IOMUX_PINCM14)
/* Defines for BIN2: GPIOB.13 with pinCMx 30 on package pin 1 */
#define DC_MOTOR_BIN2_PORT                                               (GPIOB)
#define DC_MOTOR_BIN2_PIN                                       (DL_GPIO_PIN_13)
#define DC_MOTOR_BIN2_IOMUX                                      (IOMUX_PINCM30)
/* Port definition for Pin Group ENCODER */
#define ENCODER_PORT                                                     (GPIOB)

/* Defines for LEFT_PULSE: GPIOB.8 with pinCMx 25 on package pin 60 */
// pins affected by this interrupt request:["LEFT_PULSE","RIGHT_PULSE"]
#define ENCODER_INT_IRQN                                        (GPIOB_INT_IRQn)
#define ENCODER_INT_IIDX                        (DL_INTERRUPT_GROUP1_IIDX_GPIOB)
#define ENCODER_LEFT_PULSE_IIDX                              (DL_GPIO_IIDX_DIO8)
#define ENCODER_LEFT_PULSE_PIN                                   (DL_GPIO_PIN_8)
#define ENCODER_LEFT_PULSE_IOMUX                                 (IOMUX_PINCM25)
/* Defines for RIGHT_PULSE: GPIOB.9 with pinCMx 26 on package pin 61 */
#define ENCODER_RIGHT_PULSE_IIDX                             (DL_GPIO_IIDX_DIO9)
#define ENCODER_RIGHT_PULSE_PIN                                  (DL_GPIO_PIN_9)
#define ENCODER_RIGHT_PULSE_IOMUX                                (IOMUX_PINCM26)
/* Defines for MODE: GPIOA.23 with pinCMx 53 on package pin 24 */
#define BUTTONS_MODE_PORT                                                (GPIOA)
#define BUTTONS_MODE_PIN                                        (DL_GPIO_PIN_23)
#define BUTTONS_MODE_IOMUX                                       (IOMUX_PINCM53)
/* Defines for START: GPIOB.18 with pinCMx 44 on package pin 15 */
#define BUTTONS_START_PORT                                               (GPIOB)
#define BUTTONS_START_PIN                                       (DL_GPIO_PIN_18)
#define BUTTONS_START_IOMUX                                      (IOMUX_PINCM44)


/* clang-format on */

void SYSCFG_DL_init(void);
void SYSCFG_DL_initPower(void);
void SYSCFG_DL_GPIO_init(void);
void SYSCFG_DL_SYSCTL_init(void);

bool SYSCFG_DL_SYSCTL_SYSPLL_init(void);
void SYSCFG_DL_PWMAB_init(void);
void SYSCFG_DL_MOTOR_PID_init(void);
void SYSCFG_DL_OLED_init(void);
void SYSCFG_DL_MS6DSV_init(void);
void SYSCFG_DL_TELEMETRY_UART_init(void);


bool SYSCFG_DL_saveConfiguration(void);
bool SYSCFG_DL_restoreConfiguration(void);

#ifdef __cplusplus
}
#endif

#endif /* ti_msp_dl_config_h */
