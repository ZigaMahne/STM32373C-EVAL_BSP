/******************************************************************************
 * File Name   : MX_Device.h
 * Date        : 19/02/2025 13:06:30
 * Description : STM32Cube MX parameter definitions
 * Note        : This file is generated with a generator out of the
 *               STM32CubeMX project and its generated files (DO NOT EDIT!)
 ******************************************************************************/

#ifndef MX_DEVICE_H__
#define MX_DEVICE_H__

/* MX_Device.h version */
#define MX_DEVICE_VERSION                       0x01000000


/*------------------------------ I2C1           -----------------------------*/
#define MX_I2C1                                 1

/* Filter Settings */
#define MX_I2C1_ANF_ENABLE                      1
#define MX_I2C1_DNF                             0

/* Peripheral Clock Frequency */
#define MX_I2C1_PERIPH_CLOCK_FREQ               8000000

/* Pins */

/* I2C1_SCL */
#define MX_I2C1_SCL_Pin                         PB6
#define MX_I2C1_SCL_GPIO_Pin                    GPIO_PIN_6
#define MX_I2C1_SCL_GPIOx                       GPIOB
#define MX_I2C1_SCL_GPIO_Mode                   GPIO_MODE_AF_OD
#define MX_I2C1_SCL_GPIO_PuPd                   GPIO_PULLUP
#define MX_I2C1_SCL_GPIO_Speed                  GPIO_SPEED_FREQ_LOW
#define MX_I2C1_SCL_GPIO_AF                     GPIO_AF4_I2C1

/* I2C1_SDA */
#define MX_I2C1_SDA_Pin                         PB7
#define MX_I2C1_SDA_GPIO_Pin                    GPIO_PIN_7
#define MX_I2C1_SDA_GPIOx                       GPIOB
#define MX_I2C1_SDA_GPIO_Mode                   GPIO_MODE_AF_OD
#define MX_I2C1_SDA_GPIO_PuPd                   GPIO_PULLUP
#define MX_I2C1_SDA_GPIO_Speed                  GPIO_SPEED_FREQ_LOW
#define MX_I2C1_SDA_GPIO_AF                     GPIO_AF4_I2C1

/*------------------------------ I2C2           -----------------------------*/
#define MX_I2C2                                 1

/* Pins */

/* I2C2_SCL */
#define MX_I2C2_SCL_Pin                         PA9
#define MX_I2C2_SCL_GPIO_Pin                    GPIO_PIN_9
#define MX_I2C2_SCL_GPIOx                       GPIOA
#define MX_I2C2_SCL_GPIO_Mode                   GPIO_MODE_AF_OD
#define MX_I2C2_SCL_GPIO_PuPd                   GPIO_PULLUP
#define MX_I2C2_SCL_GPIO_Speed                  GPIO_SPEED_FREQ_LOW
#define MX_I2C2_SCL_GPIO_AF                     GPIO_AF4_I2C2

/* I2C2_SDA */
#define MX_I2C2_SDA_Pin                         PA10
#define MX_I2C2_SDA_GPIO_Pin                    GPIO_PIN_10
#define MX_I2C2_SDA_GPIOx                       GPIOA
#define MX_I2C2_SDA_GPIO_Mode                   GPIO_MODE_AF_OD
#define MX_I2C2_SDA_GPIO_PuPd                   GPIO_PULLUP
#define MX_I2C2_SDA_GPIO_Speed                  GPIO_SPEED_FREQ_LOW
#define MX_I2C2_SDA_GPIO_AF                     GPIO_AF4_I2C2

/* I2C2_SMBA */
#define MX_I2C2_SMBA_Pin                        PA8
#define MX_I2C2_SMBA_GPIO_Pin                   GPIO_PIN_8
#define MX_I2C2_SMBA_GPIOx                      GPIOA
#define MX_I2C2_SMBA_GPIO_Mode                  GPIO_MODE_AF_OD
#define MX_I2C2_SMBA_GPIO_PuPd                  GPIO_PULLUP
#define MX_I2C2_SMBA_GPIO_Speed                 GPIO_SPEED_FREQ_LOW
#define MX_I2C2_SMBA_GPIO_AF                    GPIO_AF4_I2C2

/*------------------------------ SPI3           -----------------------------*/
#define MX_SPI3                                 1

/* Peripheral Clock Frequency */
#define MX_SPI3_PERIPH_CLOCK_FREQ               36000000

/* Pins */

/* SPI3_MISO */
#define MX_SPI3_MISO_Pin                        PC11
#define MX_SPI3_MISO_GPIO_Pin                   GPIO_PIN_11
#define MX_SPI3_MISO_GPIOx                      GPIOC
#define MX_SPI3_MISO_GPIO_Mode                  GPIO_MODE_AF_PP
#define MX_SPI3_MISO_GPIO_PuPd                  GPIO_NOPULL
#define MX_SPI3_MISO_GPIO_Speed                 GPIO_SPEED_FREQ_LOW
#define MX_SPI3_MISO_GPIO_AF                    GPIO_AF6_SPI3

/* SPI3_MOSI */
#define MX_SPI3_MOSI_Pin                        PC12
#define MX_SPI3_MOSI_GPIO_Pin                   GPIO_PIN_12
#define MX_SPI3_MOSI_GPIOx                      GPIOC
#define MX_SPI3_MOSI_GPIO_Mode                  GPIO_MODE_AF_PP
#define MX_SPI3_MOSI_GPIO_PuPd                  GPIO_NOPULL
#define MX_SPI3_MOSI_GPIO_Speed                 GPIO_SPEED_FREQ_LOW
#define MX_SPI3_MOSI_GPIO_AF                    GPIO_AF6_SPI3

/* SPI3_SCK */
#define MX_SPI3_SCK_Pin                         PC10
#define MX_SPI3_SCK_GPIO_Pin                    GPIO_PIN_10
#define MX_SPI3_SCK_GPIOx                       GPIOC
#define MX_SPI3_SCK_GPIO_Mode                   GPIO_MODE_AF_PP
#define MX_SPI3_SCK_GPIO_PuPd                   GPIO_NOPULL
#define MX_SPI3_SCK_GPIO_Speed                  GPIO_SPEED_FREQ_LOW
#define MX_SPI3_SCK_GPIO_AF                     GPIO_AF6_SPI3

/*------------------------------ USART2         -----------------------------*/
#define MX_USART2                               1

/* Virtual mode */
#define MX_USART2_VM                            VM_ASYNC
#define MX_USART2_VM_ASYNC                      1

/* Pins */

/* USART2_CTS */
#define MX_USART2_CTS_Pin                       PD3
#define MX_USART2_CTS_GPIO_Pin                  GPIO_PIN_3
#define MX_USART2_CTS_GPIOx                     GPIOD
#define MX_USART2_CTS_GPIO_Mode                 GPIO_MODE_AF_PP
#define MX_USART2_CTS_GPIO_PuPd                 GPIO_NOPULL
#define MX_USART2_CTS_GPIO_Speed                GPIO_SPEED_FREQ_LOW
#define MX_USART2_CTS_GPIO_AF                   GPIO_AF7_USART2

/* USART2_RTS */
#define MX_USART2_RTS_Pin                       PD4
#define MX_USART2_RTS_GPIO_Pin                  GPIO_PIN_4
#define MX_USART2_RTS_GPIOx                     GPIOD
#define MX_USART2_RTS_GPIO_Mode                 GPIO_MODE_AF_PP
#define MX_USART2_RTS_GPIO_PuPd                 GPIO_NOPULL
#define MX_USART2_RTS_GPIO_Speed                GPIO_SPEED_FREQ_LOW
#define MX_USART2_RTS_GPIO_AF                   GPIO_AF7_USART2

/* USART2_RX */
#define MX_USART2_RX_Pin                        PD6
#define MX_USART2_RX_GPIO_Pin                   GPIO_PIN_6
#define MX_USART2_RX_GPIOx                      GPIOD
#define MX_USART2_RX_GPIO_Mode                  GPIO_MODE_AF_PP
#define MX_USART2_RX_GPIO_PuPd                  GPIO_NOPULL
#define MX_USART2_RX_GPIO_Speed                 GPIO_SPEED_FREQ_LOW
#define MX_USART2_RX_GPIO_AF                    GPIO_AF7_USART2

/* USART2_TX */
#define MX_USART2_TX_Pin                        PD5
#define MX_USART2_TX_GPIO_Pin                   GPIO_PIN_5
#define MX_USART2_TX_GPIOx                      GPIOD
#define MX_USART2_TX_GPIO_Mode                  GPIO_MODE_AF_PP
#define MX_USART2_TX_GPIO_PuPd                  GPIO_NOPULL
#define MX_USART2_TX_GPIO_Speed                 GPIO_SPEED_FREQ_LOW
#define MX_USART2_TX_GPIO_AF                    GPIO_AF7_USART2

/*------------------------------ USB            -----------------------------*/
#define MX_USB                                  1

/* Handle */
#define MX_USB_HANDLE                           hpcd_USB_FS

/* Pins */

/* USB_DM */
#define MX_USB_DM_Pin                           PA11
#define MX_USB_DM_GPIO_Pin                      GPIO_PIN_11
#define MX_USB_DM_GPIOx                         GPIOA
#define MX_USB_DM_GPIO_Mode                     GPIO_MODE_AF_PP
#define MX_USB_DM_GPIO_PuPd                     GPIO_NOPULL
#define MX_USB_DM_GPIO_Speed                    GPIO_SPEED_FREQ_HIGH
#define MX_USB_DM_GPIO_AF                       GPIO_AF14_USB

/* USB_DP */
#define MX_USB_DP_Pin                           PA12
#define MX_USB_DP_GPIO_Pin                      GPIO_PIN_12
#define MX_USB_DP_GPIOx                         GPIOA
#define MX_USB_DP_GPIO_Mode                     GPIO_MODE_AF_PP
#define MX_USB_DP_GPIO_PuPd                     GPIO_NOPULL
#define MX_USB_DP_GPIO_Speed                    GPIO_SPEED_FREQ_HIGH
#define MX_USB_DP_GPIO_AF                       GPIO_AF14_USB

#endif  /* MX_DEVICE_H__ */
