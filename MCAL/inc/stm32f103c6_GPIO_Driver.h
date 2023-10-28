/*
 * stm32f103c6_GPIO_Driver.h
 *
 *  Created on: Nov 4, 2022
 *      Author: Ahmed
 */

#ifndef INC_STM32F103C6_GPIO_DRIVER_H_
#define INC_STM32F103C6_GPIO_DRIVER_H_

// =============================================
// ================== Includes =================
// =============================================

#include "Platform_Types.h"
#include "Utils.h"

#include "stm32f10xxx_device_header.h"

// ==========================================================
// ================== User Type Definitions =================
// ==========================================================

typedef struct
{
	uint8_t GPIO_PinNumber;							// This specifies which pin in GPIO to be configured.
												   //  This parameter must be a value of @ref GPIO_PIN_define.

	uint8_t GPIO_PinMode;								 // This specifies what the mode of GPIO pin is and if configured as output what the speed of the pin is.
													//  This parameter must be a value of @ref GPIO_MODE_define.

}	GPIO_PinConfig_t;


// ===================================================================
// ================== Macros Configuration Reference =================
// ===================================================================

// @ref GPIO_PIN_define.
#define GPIO_PIN_0								0
#define GPIO_PIN_1								1
#define GPIO_PIN_2								2
#define GPIO_PIN_3								3
#define GPIO_PIN_4								4
#define GPIO_PIN_5								5
#define GPIO_PIN_6								6
#define GPIO_PIN_7								7
#define GPIO_PIN_8								8
#define GPIO_PIN_9								9
#define GPIO_PIN_10								10
#define GPIO_PIN_11								11
#define GPIO_PIN_12								12
#define GPIO_PIN_13								13
#define GPIO_PIN_14								14
#define GPIO_PIN_15								15

// @ref GPIO_MODE_define.
#define GPIO_MODE_OUTPUT_GP_PP_10MHz			0x1
#define GPIO_MODE_OUTPUT_GP_PP_2MHz				0x2
#define GPIO_MODE_OUTPUT_GP_PP_50MHz			0x3

#define GPIO_MODE_OUTPUT_GP_OD_10MHz			0x5
#define GPIO_MODE_OUTPUT_GP_OD_2MHz				0x6
#define GPIO_MODE_OUTPUT_GP_OD_50MHz			0x7

#define GPIO_MODE_OUTPUT_AF_PP_10MHz			0x9
#define GPIO_MODE_OUTPUT_AF_PP_2MHz				0xA
#define GPIO_MODE_OUTPUT_AF_PP_50MHz			0xB

#define GPIO_MODE_OUTPUT_AF_OD_10MHz			0xD
#define GPIO_MODE_OUTPUT_AF_OD_2MHz				0xE
#define GPIO_MODE_OUTPUT_AF_OD_50MHz			0xF

#define GPIO_MODE_INPUT_ANALOG					0x0
#define GPIO_MODE_INPUT_FLOATING				0x4
#define GPIO_MODE_INPUT_PUR						0x8
#define GPIO_MODE_INPUT_PDR						0xC


/* This specifies the output and input states */
// @ref GPIO_STATE_define
#define GPIO_STATE_LOW						0
#define GPIO_STATE_HIGH						1

// ===================================================
// ================== APIs Functions =================
// ===================================================
void MCAL_GPIO_Init(GPIO_TypeDef *GPIOx, GPIO_PinConfig_t *pinConfig);
void MCAL_GPIO_DeInit(GPIO_TypeDef *GPIOx);

void MCAL_GPIO_writePin(GPIO_TypeDef *GPIOx, uint8_t PinNumber, uint8_t value);
void MCAL_GPIO_writePort(GPIO_TypeDef *GPIOx, uint16_t value);

uint8_t MCAL_GPIO_readPin(GPIO_TypeDef *GPIOx, uint8_t PinNumber);
uint16_t MCAL_GPIO_readPort(GPIO_TypeDef *GPIOx);

void MCAL_GPIO_togglePin(GPIO_TypeDef *GPIOx, uint8_t PinNumber);
void MCAL_GPIO_togglePort(GPIO_TypeDef *GPIOx);



#endif /* INC_STM32F103C6_GPIO_DRIVER_H_ */
