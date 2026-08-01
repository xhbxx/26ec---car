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



#define CPUCLK_FREQ                                                     32000000




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


/* Defines for PRINT */
#define PRINT_INST                                                         UART2
#define PRINT_INST_FREQUENCY                                            32000000
#define PRINT_INST_IRQHandler                                   UART2_IRQHandler
#define PRINT_INST_INT_IRQN                                       UART2_INT_IRQn
#define GPIO_PRINT_RX_PORT                                                 GPIOA
#define GPIO_PRINT_TX_PORT                                                 GPIOA
#define GPIO_PRINT_RX_PIN                                         DL_GPIO_PIN_22
#define GPIO_PRINT_TX_PIN                                         DL_GPIO_PIN_21
#define GPIO_PRINT_IOMUX_RX                                      (IOMUX_PINCM47)
#define GPIO_PRINT_IOMUX_TX                                      (IOMUX_PINCM46)
#define GPIO_PRINT_IOMUX_RX_FUNC                       IOMUX_PINCM47_PF_UART2_RX
#define GPIO_PRINT_IOMUX_TX_FUNC                       IOMUX_PINCM46_PF_UART2_TX
#define PRINT_BAUD_RATE                                                 (115200)
#define PRINT_IBRD_32_MHZ_115200_BAUD                                       (17)
#define PRINT_FBRD_32_MHZ_115200_BAUD                                       (23)
/* Defines for MOTOR_UART */
#define MOTOR_UART_INST                                                    UART0
#define MOTOR_UART_INST_FREQUENCY                                       32000000
#define MOTOR_UART_INST_IRQHandler                              UART0_IRQHandler
#define MOTOR_UART_INST_INT_IRQN                                  UART0_INT_IRQn
#define GPIO_MOTOR_UART_RX_PORT                                            GPIOA
#define GPIO_MOTOR_UART_TX_PORT                                            GPIOA
#define GPIO_MOTOR_UART_RX_PIN                                    DL_GPIO_PIN_31
#define GPIO_MOTOR_UART_TX_PIN                                    DL_GPIO_PIN_28
#define GPIO_MOTOR_UART_IOMUX_RX                                  (IOMUX_PINCM6)
#define GPIO_MOTOR_UART_IOMUX_TX                                  (IOMUX_PINCM3)
#define GPIO_MOTOR_UART_IOMUX_RX_FUNC                   IOMUX_PINCM6_PF_UART0_RX
#define GPIO_MOTOR_UART_IOMUX_TX_FUNC                   IOMUX_PINCM3_PF_UART0_TX
#define MOTOR_UART_BAUD_RATE                                            (115200)
#define MOTOR_UART_IBRD_32_MHZ_115200_BAUD                                  (17)
#define MOTOR_UART_FBRD_32_MHZ_115200_BAUD                                  (23)
/* Defines for LLM_UART */
#define LLM_UART_INST                                                      UART1
#define LLM_UART_INST_FREQUENCY                                         32000000
#define LLM_UART_INST_IRQHandler                                UART1_IRQHandler
#define LLM_UART_INST_INT_IRQN                                    UART1_INT_IRQn
#define GPIO_LLM_UART_RX_PORT                                              GPIOA
#define GPIO_LLM_UART_TX_PORT                                              GPIOA
#define GPIO_LLM_UART_RX_PIN                                       DL_GPIO_PIN_9
#define GPIO_LLM_UART_TX_PIN                                       DL_GPIO_PIN_8
#define GPIO_LLM_UART_IOMUX_RX                                   (IOMUX_PINCM20)
#define GPIO_LLM_UART_IOMUX_TX                                   (IOMUX_PINCM19)
#define GPIO_LLM_UART_IOMUX_RX_FUNC                    IOMUX_PINCM20_PF_UART1_RX
#define GPIO_LLM_UART_IOMUX_TX_FUNC                    IOMUX_PINCM19_PF_UART1_TX
#define LLM_UART_BAUD_RATE                                              (115200)
#define LLM_UART_IBRD_32_MHZ_115200_BAUD                                    (17)
#define LLM_UART_FBRD_32_MHZ_115200_BAUD                                    (23)
/* Defines for VEHICLE_UART */
#define VEHICLE_UART_INST                                                  UART3
#define VEHICLE_UART_INST_FREQUENCY                                     32000000
#define VEHICLE_UART_INST_IRQHandler                            UART3_IRQHandler
#define VEHICLE_UART_INST_INT_IRQN                                UART3_INT_IRQn
#define GPIO_VEHICLE_UART_RX_PORT                                          GPIOA
#define GPIO_VEHICLE_UART_TX_PORT                                          GPIOA
#define GPIO_VEHICLE_UART_RX_PIN                                  DL_GPIO_PIN_13
#define GPIO_VEHICLE_UART_TX_PIN                                  DL_GPIO_PIN_14
#define GPIO_VEHICLE_UART_IOMUX_RX                               (IOMUX_PINCM35)
#define GPIO_VEHICLE_UART_IOMUX_TX                               (IOMUX_PINCM36)
#define GPIO_VEHICLE_UART_IOMUX_RX_FUNC                IOMUX_PINCM35_PF_UART3_RX
#define GPIO_VEHICLE_UART_IOMUX_TX_FUNC                IOMUX_PINCM36_PF_UART3_TX
#define VEHICLE_UART_BAUD_RATE                                          (115200)
#define VEHICLE_UART_IBRD_32_MHZ_115200_BAUD                                (17)
#define VEHICLE_UART_FBRD_32_MHZ_115200_BAUD                                (23)





/* Port definition for Pin Group Bianma */
#define Bianma_PORT                                                      (GPIOB)

/* Defines for ENC_A: GPIOB.24 with pinCMx 52 on package pin 23 */
#define Bianma_ENC_A_PIN                                        (DL_GPIO_PIN_24)
#define Bianma_ENC_A_IOMUX                                       (IOMUX_PINCM52)
/* Defines for ENC_B: GPIOB.20 with pinCMx 48 on package pin 19 */
#define Bianma_ENC_B_PIN                                        (DL_GPIO_PIN_20)
#define Bianma_ENC_B_IOMUX                                       (IOMUX_PINCM48)
/* Defines for ENC_SW: GPIOB.19 with pinCMx 45 on package pin 16 */
#define Bianma_ENC_SW_PIN                                       (DL_GPIO_PIN_19)
#define Bianma_ENC_SW_IOMUX                                      (IOMUX_PINCM45)


/* clang-format on */

void SYSCFG_DL_init(void);
void SYSCFG_DL_initPower(void);
void SYSCFG_DL_GPIO_init(void);
void SYSCFG_DL_SYSCTL_init(void);
void SYSCFG_DL_OLED_init(void);
void SYSCFG_DL_PRINT_init(void);
void SYSCFG_DL_MOTOR_UART_init(void);
void SYSCFG_DL_LLM_UART_init(void);
void SYSCFG_DL_VEHICLE_UART_init(void);


bool SYSCFG_DL_saveConfiguration(void);
bool SYSCFG_DL_restoreConfiguration(void);

#ifdef __cplusplus
}
#endif

#endif /* ti_msp_dl_config_h */
