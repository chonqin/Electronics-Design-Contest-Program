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
#define FCC_EXPECTED_RATIO                                                  2000
#define FCC_UPPER_BOUND                       (FCC_EXPECTED_RATIO * (1 + 0.003))
#define FCC_LOWER_BOUND                       (FCC_EXPECTED_RATIO * (1 - 0.003))

bool SYSCFG_DL_SYSCTL_SYSPLL_init(void);


/* Defines for PWM_MOTOR */
#define PWM_MOTOR_INST                                                     TIMA1
#define PWM_MOTOR_INST_IRQHandler                               TIMA1_IRQHandler
#define PWM_MOTOR_INST_INT_IRQN                                 (TIMA1_INT_IRQn)
#define PWM_MOTOR_INST_CLK_FREQ                                         80000000
/* GPIO defines for channel 0 */
#define GPIO_PWM_MOTOR_C0_PORT                                             GPIOA
#define GPIO_PWM_MOTOR_C0_PIN                                     DL_GPIO_PIN_15
#define GPIO_PWM_MOTOR_C0_IOMUX                                  (IOMUX_PINCM37)
#define GPIO_PWM_MOTOR_C0_IOMUX_FUNC                 IOMUX_PINCM37_PF_TIMA1_CCP0
#define GPIO_PWM_MOTOR_C0_IDX                                DL_TIMER_CC_0_INDEX
/* GPIO defines for channel 1 */
#define GPIO_PWM_MOTOR_C1_PORT                                             GPIOA
#define GPIO_PWM_MOTOR_C1_PIN                                     DL_GPIO_PIN_24
#define GPIO_PWM_MOTOR_C1_IOMUX                                  (IOMUX_PINCM54)
#define GPIO_PWM_MOTOR_C1_IOMUX_FUNC                 IOMUX_PINCM54_PF_TIMA1_CCP1
#define GPIO_PWM_MOTOR_C1_IDX                                DL_TIMER_CC_1_INDEX



/* Defines for TIMER_ENCODER_TICK */
#define TIMER_ENCODER_TICK_INST                                          (TIMG0)
#define TIMER_ENCODER_TICK_INST_IRQHandler                        TIMG0_IRQHandler
#define TIMER_ENCODER_TICK_INST_INT_IRQN                        (TIMG0_INT_IRQn)
#define TIMER_ENCODER_TICK_INST_LOAD_VALUE                                 (9999U)
/* Defines for TIMER_IMU_TICK */
#define TIMER_IMU_TICK_INST                                              (TIMA0)
#define TIMER_IMU_TICK_INST_IRQHandler                          TIMA0_IRQHandler
#define TIMER_IMU_TICK_INST_INT_IRQN                            (TIMA0_INT_IRQn)
#define TIMER_IMU_TICK_INST_LOAD_VALUE                                  (39999U)




/* Defines for I2C_OLED */
#define I2C_OLED_INST                                                       I2C1
#define I2C_OLED_INST_IRQHandler                                 I2C1_IRQHandler
#define I2C_OLED_INST_INT_IRQN                                     I2C1_INT_IRQn
#define I2C_OLED_BUS_SPEED_HZ                                             400000
#define GPIO_I2C_OLED_SDA_PORT                                             GPIOA
#define GPIO_I2C_OLED_SDA_PIN                                     DL_GPIO_PIN_30
#define GPIO_I2C_OLED_IOMUX_SDA                                   (IOMUX_PINCM5)
#define GPIO_I2C_OLED_IOMUX_SDA_FUNC                    IOMUX_PINCM5_PF_I2C1_SDA
#define GPIO_I2C_OLED_SCL_PORT                                             GPIOA
#define GPIO_I2C_OLED_SCL_PIN                                     DL_GPIO_PIN_29
#define GPIO_I2C_OLED_IOMUX_SCL                                   (IOMUX_PINCM4)
#define GPIO_I2C_OLED_IOMUX_SCL_FUNC                    IOMUX_PINCM4_PF_I2C1_SCL


/* Defines for UART_0 */
#define UART_0_INST                                                        UART0
#define UART_0_INST_FREQUENCY                                            4000000
#define UART_0_INST_IRQHandler                                  UART0_IRQHandler
#define UART_0_INST_INT_IRQN                                      UART0_INT_IRQn
#define GPIO_UART_0_RX_PORT                                                GPIOA
#define GPIO_UART_0_TX_PORT                                                GPIOA
#define GPIO_UART_0_RX_PIN                                        DL_GPIO_PIN_11
#define GPIO_UART_0_TX_PIN                                        DL_GPIO_PIN_10
#define GPIO_UART_0_IOMUX_RX                                     (IOMUX_PINCM22)
#define GPIO_UART_0_IOMUX_TX                                     (IOMUX_PINCM21)
#define GPIO_UART_0_IOMUX_RX_FUNC                      IOMUX_PINCM22_PF_UART0_RX
#define GPIO_UART_0_IOMUX_TX_FUNC                      IOMUX_PINCM21_PF_UART0_TX
#define UART_0_BAUD_RATE                                                (115200)
#define UART_0_IBRD_4_MHZ_115200_BAUD                                        (2)
#define UART_0_FBRD_4_MHZ_115200_BAUD                                       (11)
/* Defines for UART_1 */
#define UART_1_INST                                                        UART1
#define UART_1_INST_FREQUENCY                                           40000000
#define UART_1_INST_IRQHandler                                  UART1_IRQHandler
#define UART_1_INST_INT_IRQN                                      UART1_INT_IRQn
#define GPIO_UART_1_RX_PORT                                                GPIOA
#define GPIO_UART_1_TX_PORT                                                GPIOA
#define GPIO_UART_1_RX_PIN                                         DL_GPIO_PIN_9
#define GPIO_UART_1_TX_PIN                                         DL_GPIO_PIN_8
#define GPIO_UART_1_IOMUX_RX                                     (IOMUX_PINCM20)
#define GPIO_UART_1_IOMUX_TX                                     (IOMUX_PINCM19)
#define GPIO_UART_1_IOMUX_RX_FUNC                      IOMUX_PINCM20_PF_UART1_RX
#define GPIO_UART_1_IOMUX_TX_FUNC                      IOMUX_PINCM19_PF_UART1_TX
#define UART_1_BAUD_RATE                                                  (9600)
#define UART_1_IBRD_40_MHZ_9600_BAUD                                       (260)
#define UART_1_FBRD_40_MHZ_9600_BAUD                                        (27)




/* Defines for SPI_IMU */
#define SPI_IMU_INST                                                       SPI0
#define SPI_IMU_INST_IRQHandler                                 SPI0_IRQHandler
#define SPI_IMU_INST_INT_IRQN                                     SPI0_INT_IRQn
#define GPIO_SPI_IMU_PICO_PORT                                            GPIOA
#define GPIO_SPI_IMU_PICO_PIN                                    DL_GPIO_PIN_14
#define GPIO_SPI_IMU_IOMUX_PICO                                 (IOMUX_PINCM36)
#define GPIO_SPI_IMU_IOMUX_PICO_FUNC                 IOMUX_PINCM36_PF_SPI0_PICO
#define GPIO_SPI_IMU_POCI_PORT                                            GPIOB
#define GPIO_SPI_IMU_POCI_PIN                                    DL_GPIO_PIN_19
#define GPIO_SPI_IMU_IOMUX_POCI                                 (IOMUX_PINCM45)
#define GPIO_SPI_IMU_IOMUX_POCI_FUNC                 IOMUX_PINCM45_PF_SPI0_POCI
/* GPIO configuration for SPI_IMU */
#define GPIO_SPI_IMU_SCLK_PORT                                            GPIOA
#define GPIO_SPI_IMU_SCLK_PIN                                    DL_GPIO_PIN_12
#define GPIO_SPI_IMU_IOMUX_SCLK                                 (IOMUX_PINCM34)
#define GPIO_SPI_IMU_IOMUX_SCLK_FUNC                 IOMUX_PINCM34_PF_SPI0_SCLK



/* Port definition for Pin Group GPIO_IMU_CS */
#define GPIO_IMU_CS_PORT                                                 (GPIOB)

/* Defines for PB25: GPIOB.25 with pinCMx 56 on package pin 27 */
#define GPIO_IMU_CS_PB25_PIN                                    (DL_GPIO_PIN_25)
#define GPIO_IMU_CS_PB25_IOMUX                                   (IOMUX_PINCM56)
/* Port definition for Pin Group GPIO_IMU_INT */
#define GPIO_IMU_INT_PORT                                                (GPIOA)

/* Defines for PA16: GPIOA.16 with pinCMx 38 on package pin 9 */
// pins affected by this interrupt request:["PA16"]
#define GPIO_IMU_INT_INT_IRQN                                   (GPIOA_INT_IRQn)
#define GPIO_IMU_INT_INT_IIDX                   (DL_INTERRUPT_GROUP1_IIDX_GPIOA)
#define GPIO_IMU_INT_PA16_IIDX                              (DL_GPIO_IIDX_DIO16)
#define GPIO_IMU_INT_PA16_PIN                                   (DL_GPIO_PIN_16)
#define GPIO_IMU_INT_PA16_IOMUX                                  (IOMUX_PINCM38)
/* Port definition for Pin Group GPIO_RING */
#define GPIO_RING_PORT                                                   (GPIOA)

/* Defines for PIN_27: GPIOA.27 with pinCMx 60 on package pin 31 */
#define GPIO_RING_PIN_27_PIN                                    (DL_GPIO_PIN_27)
#define GPIO_RING_PIN_27_IOMUX                                   (IOMUX_PINCM60)
/* Defines for LED1: GPIOA.7 with pinCMx 14 on package pin 49 */
#define GPIO_LED_LED1_PORT                                               (GPIOA)
#define GPIO_LED_LED1_PIN                                        (DL_GPIO_PIN_7)
#define GPIO_LED_LED1_IOMUX                                      (IOMUX_PINCM14)
/* Defines for LED2: GPIOB.2 with pinCMx 15 on package pin 50 */
#define GPIO_LED_LED2_PORT                                               (GPIOB)
#define GPIO_LED_LED2_PIN                                        (DL_GPIO_PIN_2)
#define GPIO_LED_LED2_IOMUX                                      (IOMUX_PINCM15)
/* Port definition for Pin Group GPIO_MOTOR */
#define GPIO_MOTOR_PORT                                                  (GPIOB)

/* Defines for AIN1: GPIOB.10 with pinCMx 27 on package pin 62 */
#define GPIO_MOTOR_AIN1_PIN                                     (DL_GPIO_PIN_10)
#define GPIO_MOTOR_AIN1_IOMUX                                    (IOMUX_PINCM27)
/* Defines for AIN2: GPIOB.13 with pinCMx 30 on package pin 1 */
#define GPIO_MOTOR_AIN2_PIN                                     (DL_GPIO_PIN_13)
#define GPIO_MOTOR_AIN2_IOMUX                                    (IOMUX_PINCM30)
/* Defines for BIN1: GPIOB.15 with pinCMx 32 on package pin 3 */
#define GPIO_MOTOR_BIN1_PIN                                     (DL_GPIO_PIN_15)
#define GPIO_MOTOR_BIN1_IOMUX                                    (IOMUX_PINCM32)
/* Defines for BIN2: GPIOB.16 with pinCMx 33 on package pin 4 */
#define GPIO_MOTOR_BIN2_PIN                                     (DL_GPIO_PIN_16)
#define GPIO_MOTOR_BIN2_IOMUX                                    (IOMUX_PINCM33)
/* Port definition for Pin Group GPIO_ENCODER */
#define GPIO_ENCODER_PORT                                                (GPIOB)

/* Defines for E1A: GPIOB.6 with pinCMx 23 on package pin 58 */
// pins affected by this interrupt request:["E1A","E1B","E2A","E2B"]
#define GPIO_ENCODER_INT_IRQN                                   (GPIOB_INT_IRQn)
#define GPIO_ENCODER_INT_IIDX                   (DL_INTERRUPT_GROUP1_IIDX_GPIOB)
#define GPIO_ENCODER_E1A_IIDX                                (DL_GPIO_IIDX_DIO6)
#define GPIO_ENCODER_E1A_PIN                                     (DL_GPIO_PIN_6)
#define GPIO_ENCODER_E1A_IOMUX                                   (IOMUX_PINCM23)
/* Defines for E1B: GPIOB.7 with pinCMx 24 on package pin 59 */
#define GPIO_ENCODER_E1B_IIDX                                (DL_GPIO_IIDX_DIO7)
#define GPIO_ENCODER_E1B_PIN                                     (DL_GPIO_PIN_7)
#define GPIO_ENCODER_E1B_IOMUX                                   (IOMUX_PINCM24)
/* Defines for E2A: GPIOB.9 with pinCMx 26 on package pin 61 */
#define GPIO_ENCODER_E2A_IIDX                                (DL_GPIO_IIDX_DIO9)
#define GPIO_ENCODER_E2A_PIN                                     (DL_GPIO_PIN_9)
#define GPIO_ENCODER_E2A_IOMUX                                   (IOMUX_PINCM26)
/* Defines for E2B: GPIOB.8 with pinCMx 25 on package pin 60 */
#define GPIO_ENCODER_E2B_IIDX                                (DL_GPIO_IIDX_DIO8)
#define GPIO_ENCODER_E2B_PIN                                     (DL_GPIO_PIN_8)
#define GPIO_ENCODER_E2B_IOMUX                                   (IOMUX_PINCM25)
/* Defines for PIN_18: GPIOB.18 with pinCMx 44 on package pin 15 */
#define GPIO_KEY_PIN_18_PORT                                             (GPIOB)
#define GPIO_KEY_PIN_18_PIN                                     (DL_GPIO_PIN_18)
#define GPIO_KEY_PIN_18_IOMUX                                    (IOMUX_PINCM44)
/* Defines for PIN_13: GPIOA.13 with pinCMx 35 on package pin 6 */
#define GPIO_KEY_PIN_13_PORT                                             (GPIOA)
#define GPIO_KEY_PIN_13_PIN                                     (DL_GPIO_PIN_13)
#define GPIO_KEY_PIN_13_IOMUX                                    (IOMUX_PINCM35)
/* Defines for PIN_17: GPIOA.17 with pinCMx 39 on package pin 10 */
#define GPIO_KEY_PIN_17_PORT                                             (GPIOA)
#define GPIO_KEY_PIN_17_PIN                                     (DL_GPIO_PIN_17)
#define GPIO_KEY_PIN_17_IOMUX                                    (IOMUX_PINCM39)
/* Defines for X1: GPIOA.22 with pinCMx 47 on package pin 18 */
#define GPIO_TRACk_X1_PORT                                               (GPIOA)
#define GPIO_TRACk_X1_PIN                                       (DL_GPIO_PIN_22)
#define GPIO_TRACk_X1_IOMUX                                      (IOMUX_PINCM47)
/* Defines for X2: GPIOB.20 with pinCMx 48 on package pin 19 */
#define GPIO_TRACk_X2_PORT                                               (GPIOB)
#define GPIO_TRACk_X2_PIN                                       (DL_GPIO_PIN_20)
#define GPIO_TRACk_X2_IOMUX                                      (IOMUX_PINCM48)
/* Defines for X3: GPIOB.21 with pinCMx 49 on package pin 20 */
#define GPIO_TRACk_X3_PORT                                               (GPIOB)
#define GPIO_TRACk_X3_PIN                                       (DL_GPIO_PIN_21)
#define GPIO_TRACk_X3_IOMUX                                      (IOMUX_PINCM49)
/* Defines for X4: GPIOB.22 with pinCMx 50 on package pin 21 */
#define GPIO_TRACk_X4_PORT                                               (GPIOB)
#define GPIO_TRACk_X4_PIN                                       (DL_GPIO_PIN_22)
#define GPIO_TRACk_X4_IOMUX                                      (IOMUX_PINCM50)
/* Defines for X5: GPIOB.23 with pinCMx 51 on package pin 22 */
#define GPIO_TRACk_X5_PORT                                               (GPIOB)
#define GPIO_TRACk_X5_PIN                                       (DL_GPIO_PIN_23)
#define GPIO_TRACk_X5_IOMUX                                      (IOMUX_PINCM51)
/* Defines for X6: GPIOB.24 with pinCMx 52 on package pin 23 */
#define GPIO_TRACk_X6_PORT                                               (GPIOB)
#define GPIO_TRACk_X6_PIN                                       (DL_GPIO_PIN_24)
#define GPIO_TRACk_X6_IOMUX                                      (IOMUX_PINCM52)
/* Defines for X7: GPIOA.25 with pinCMx 55 on package pin 26 */
#define GPIO_TRACk_X7_PORT                                               (GPIOA)
#define GPIO_TRACk_X7_PIN                                       (DL_GPIO_PIN_25)
#define GPIO_TRACk_X7_IOMUX                                      (IOMUX_PINCM55)
/* Defines for X8: GPIOB.27 with pinCMx 58 on package pin 29 */
#define GPIO_TRACk_X8_PORT                                               (GPIOB)
#define GPIO_TRACk_X8_PIN                                       (DL_GPIO_PIN_27)
#define GPIO_TRACk_X8_IOMUX                                      (IOMUX_PINCM58)


/* clang-format on */

void SYSCFG_DL_init(void);
void SYSCFG_DL_initPower(void);
void SYSCFG_DL_GPIO_init(void);
void SYSCFG_DL_SYSCTL_init(void);
void SYSCFG_DL_SYSCTL_CLK_init(void);

bool SYSCFG_DL_SYSCTL_SYSPLL_init(void);
void SYSCFG_DL_PWM_MOTOR_init(void);
void SYSCFG_DL_TIMER_ENCODER_TICK_init(void);
void SYSCFG_DL_TIMER_IMU_TICK_init(void);
void SYSCFG_DL_I2C_OLED_init(void);
void SYSCFG_DL_UART_0_init(void);
void SYSCFG_DL_UART_1_init(void);
void SYSCFG_DL_SPI_IMU_init(void);


bool SYSCFG_DL_saveConfiguration(void);
bool SYSCFG_DL_restoreConfiguration(void);

#ifdef __cplusplus
}
#endif

#endif /* ti_msp_dl_config_h */
