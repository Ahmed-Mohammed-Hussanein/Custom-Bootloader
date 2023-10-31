/*
 * GPIO_Peripheral_Pins.h
 *
 *  Created on: Sep 19, 2023
 *      Author: Ahmed
 */

#ifndef GPIO_PERIPHERAL_PINS_H_
#define GPIO_PERIPHERAL_PINS_H_

#include <stm32f103c8_GPIO_Driver.h>
#include "stm32f10xxx_device_header.h"

// ==================================================================
// =============================== USART ============================
// ==================================================================
#define USART1_PORT			GPIOA
#define USART1_TX			GPIO_PIN_9
#define USART1_RX			GPIO_PIN_10
#define USART1_CTS			GPIO_PIN_11
#define USART1_RTS			GPIO_PIN_12

#define USART2_PORT			GPIOA
#define USART2_TX			GPIO_PIN_2
#define USART2_RX			GPIO_PIN_3
#define USART2_CTS			GPIO_PIN_0
#define USART2_RTS			GPIO_PIN_1

#define USART3_PORT			GPIOB
#define USART3_TX			GPIO_PIN_10
#define USART3_RX			GPIO_PIN_11
#define USART3_CTS			GPIO_PIN_13
#define USART3_RTS			GPIO_PIN_14


// ==================================================================
// =============================== SPI ==============================
// ==================================================================
#define SPI1_PORT			GPIOA
#define SPI1_NSS			GPIO_PIN_4
#define SPI1_SCK			GPIO_PIN_5
#define SPI1_MISO			GPIO_PIN_6
#define SPI1_MOSI			GPIO_PIN_7

#define SPI2_PORT			GPIOB
#define SPI2_NSS			GPIO_PIN_12
#define SPI2_SCK			GPIO_PIN_13
#define SPI2_MISO			GPIO_PIN_14
#define SPI2_MOSI			GPIO_PIN_15


// ==================================================================
// =============================== I2C ==============================
// ==================================================================
#define I2C1_PORT				GPIOB
#define I2C1_SCL				GPIO_PIN_6
#define I2C1_SDA				GPIO_PIN_7

#define I2C2_PORT				GPIOB
#define I2C2_SCL				GPIO_PIN_10
#define I2C2_SDA				GPIO_PIN_11

#endif /* GPIO_PERIPHERAL_PINS_H_ */
