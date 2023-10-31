/*
 * stm32f103c6_GPIO_Driver.c
 *
 *  Created on: Nov 4, 2022
 *      Author: Ahmed
 */

// =============================================
// ================== Includes =================
// =============================================

#include <stm32f103c8_GPIO_Driver.h>
#include "Platform_Types.h"


/*
 * =====================================================================================
 * ================================= APIs Function Definition ==========================
 * =====================================================================================
 */

/**================================================================
* @Fn				- MCAL_GPIO_Init
* @brief			- This is used to configure and initialize a pin or a port in GPIO.
* @param [in] 		- GPIOx: This specifies which port is used for configuration at @ref GPIOx_define.
* @param [in] 		- pinConfig: This carries the configuration for the pin or port at @ref GPIO_PIN_define.
* @retval 			- None.
* Note				- None.
*/

void MCAL_GPIO_Init(GPIO_TypeDef *GPIOx, GPIO_PinConfig_t *pinConfig)
{
	uint8_t configRegister;

	if(pinConfig->GPIO_PinMode == GPIO_MODE_INPUT_PDR)
	{
		configRegister = GPIO_MODE_INPUT_PUR;
	}
	else
	{
		configRegister = pinConfig->GPIO_PinMode;
	}


	switch(pinConfig->GPIO_PinMode)
	{
	case GPIO_MODE_INPUT_PDR:
		CLEAR_BIT(GPIOx->ODR, pinConfig->GPIO_PinNumber);
//		GPIOx->ODR &= ~(1 << pinConfig->GPIO_PinNumber);

		break;

	case GPIO_MODE_INPUT_PUR:
		SET_BIT(GPIOx->ODR, pinConfig->GPIO_PinNumber);
//		GPIOx->ODR |= (1 << pinConfig->GPIO_PinNumber);

		break;

	default:
		break;
	}

	/*	CNF		MODE
	 * 	00		01			General Purpose Output Push Pull 10MHz
	 *  00		10			General Purpose Output Push Pull 2MHz
	 *  00		11			General Purpose Output Push Pull 50MHz
	 *
	 *  01		01			General Purpose Output Open Drain 10MHz
	 *  01		10			General Purpose Output Open Drain 2MHz
	 *  01		11			General Purpose Output Open Drain 50MHz
	 *
	 *  10		01			Alternative Function Output Push Pull 10MHz
	 *  10		10			Alternative Function Output Push Pull 2MHz
	 *  10		11			Alternative Function Output Push Pull 50MHz
	 *
	 *  11		01			Alternative Function Output Open Drain 10MHz
	 *  11		10			Alternative Function Output Open Drain 2MHz
	 *  11		11			Alternative Function Output Open Drain 50MHz
	 *
	 *  00		00			Input Analog
	 *  01		00			Input Floating
	 *  10		00			Input PUR
	 *  10		00			Input PDR
	 *
	 *
	 */

	switch(pinConfig->GPIO_PinNumber / 8)
	{
	case 0:
		WRITE_MASK_POS(GPIOx->CRL, 0xF, (pinConfig->GPIO_PinNumber * 4), configRegister);
//		GPIOx->CRL &= ~(0x0000000F << (pinConfig->GPIO_PinNumber * 4));
//		GPIOx->CRL |= (uint32_t)configRegister << (pinConfig->GPIO_PinNumber * 4);

		break;

	case 1:
		WRITE_MASK_POS(GPIOx->CRH, 0xF, ((pinConfig->GPIO_PinNumber - 8) * 4), configRegister);
//		GPIOx->CRH &= ~(0x0000000F << ((pinConfig->GPIO_PinNumber - 8) * 4));
//		GPIOx->CRH |= (uint32_t)configRegister << ((pinConfig->GPIO_PinNumber - 8) * 4);

		break;

	default:
		break;
	}
}

/**================================================================
* @Fn				- MCAL_GPIO_DeInit
* @brief			- This is used to reset all pins for specific GPIO.
* @param [in] 		- GPIOx: This specifies which port is used for configuration where x = A, B, or C.
* @retval 			- None.
* Note				- None.
*/
void MCAL_GPIO_DeInit(GPIO_TypeDef *GPIOx)
{
	if(GPIOx == GPIOA)
	{
		RCC_GPIOA_RESET();
	}
	else if(GPIOx == GPIOB)
	{
		RCC_GPIOB_RESET();
	}
	else
	{
		RCC_GPIOC_RESET();
	}
}


/**================================================================
* @Fn				- MCAL_GPIO_writePin
* @brief			- This is used to write a value to a pin in GPIO if the pin is in OUTPUT mode.
* @param [in] 		- GPIOx: This specifies which port is used for configuration at @ref GPIOx_define.
* @param [in] 		- pinNumber: This is used to determine the pin number in the port at @ref GPIO_PIN_define.
* @param [in] 		- value: The value needed to be written on the pin at @ref GPIO_STATE_define.
* @retval 			- None.
* Note				- None.
*/
void MCAL_GPIO_writePin(GPIO_TypeDef *GPIOx, uint8_t PinNumber, uint8_t value)
{
	switch(value)
	{
		case GPIO_STATE_LOW:
			CLEAR_BIT(GPIOx->ODR, PinNumber);
			break;

		case GPIO_STATE_HIGH:
			SET_BIT(GPIOx->ODR, PinNumber);
			break;

		default:
			break;
	}
}




/**================================================================
* @Fn				- MCAL_GPIO_writePort
* @brief			- This is used to write a value to a port GPIO if the port is in OUTPUT mode.
* @param [in] 		- GPIOx: This specifies which port is used for configuration at @ref GPIOx_define.
* @param [in] 		- value: The value needed to be written on the pin at @ref GPIO_STATE_define.
* @retval 			- None.
* Note				- None.
*/
void MCAL_GPIO_writePort(GPIO_TypeDef *GPIOx, uint16_t value)
{
	GPIOx->ODR = value;
}




/**================================================================
* @Fn				- MCAL_GPIO_readPin
* @brief			- This is used to read a value from a pin in GPIO.
* @param [in] 		- GPIOx: This specifies which port is used for configuration at @ref GPIOx_define.
* @param [in] 		- pinNumber: This is used to determine the pin number in the port at @ref GPIO_PIN_define.
* @retval 			- None.
* Note				- None.
*/
uint8_t MCAL_GPIO_readPin(GPIO_TypeDef *GPIOx, uint8_t PinNumber)
{
	return READ_BIT(GPIOx->IDR, PinNumber);
}



/**================================================================
* @Fn				- MCAL_GPIO_readPort
* @brief			- This is used to read a value from a port in GPIO.
* @param [in] 		- GPIOx: This specifies which port is used for configuration at @ref GPIOx_define.
* @retval 			- None.
* Note				- None.
*/
uint16_t MCAL_GPIO_readPort(GPIO_TypeDef *GPIOx)
{
	return (uint16_t)GPIOx->IDR;
}



/**================================================================
* @Fn				- MCAL_GPIO_togglePin
* @brief			- This is used to toggle the state of the pin in GPIO if the pin is in OUTPUT mode.
* @param [in] 		- GPIOx: This specifies which port is used for configuration at @ref GPIOx_define.
* @param [in] 		- pinNumber: This is used to determine the pin number in the port at @ref GPIO_PIN_define.
* @retval 			- None.
* Note				- None.
*/
void MCAL_GPIO_togglePin(GPIO_TypeDef *GPIOx, uint8_t PinNumber)
{
	TOGGLE_BIT(GPIOx->ODR, PinNumber);
}



/**================================================================
* @Fn				- MCAL_GPIO_togglePort
* @brief			- This is used to toggle the state of the port in GPIO if the port is in OUTPUT mode.
* @param [in] 		- GPIOx: This specifies which port is used for configuration at @ref GPIOx_define.
* @retval 			- None.
* Note				- None.
*/
void MCAL_GPIO_togglePort(GPIO_TypeDef *GPIOx)
{
	GPIOx->ODR ^= 0xFF;
}
