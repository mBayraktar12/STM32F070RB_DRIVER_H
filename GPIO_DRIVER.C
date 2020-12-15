/*
 * stm32f070xx_gpio_driver.c
 *
 * 
 *      Author: MERT
 */

#include <stm32f070xx_gpio_driver.h>

/*******************************************************************
 * @fn 					-GPIO_PeriClockControl
 *
 * @brief				- This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]			- base address of the gpio peripheral
 * @param[in]			- ENABLE or DISABLE macros
 * @param[in]			-
 *
 * return				- none
 *
 * @Note				- none
 *
 *
 */

void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
	if( EnorDi == ENABLE )
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}
		else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}
		else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}
		else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		}
		else if(pGPIOx == GPIOF)
		{
			GPIOF_PCLK_EN();
		}
	}
	else
	{
		if(pGPIOx == GPIOA)
				{
					GPIOA_PCLK_DI();
				}
				else if(pGPIOx == GPIOB)
				{
					GPIOB_PCLK_DI();
				}
				else if(pGPIOx == GPIOC)
				{
					GPIOC_PCLK_DI();
				}
				else if(pGPIOx == GPIOD)
				{
					GPIOD_PCLK_DI();
				}
				else if(pGPIOx == GPIOF)
				{
					GPIOF_PCLK_DI();
				}
	}

}

//Init and De-init
/*******************************************************************
 * @fn 					-GPIO_Init
 *
 * @brief				- This function initializes the GPIO.
 *
 * @param[in]			- base address of the gpio handle
 * @param[in]			-
 * @param[in]			-
 *
 * return				- none
 *
 * @Note				- none
 *
 *
 */

void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp = 0; //temp register
	//1. configure the mode of the pin.

	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		//the non interrupt mode
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);//clearing
		pGPIOHandle->pGPIOx->MODER |= temp; //Setting. We should not affect other bits of this register. That's why we should use bitwise or operation.
	}
	else
	{
		//(interrupt mode)
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT){
			//1.configure the FTSR
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//Clear the corresponding RTSR bit.
			EXTI->RTSR &= ~ ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT){
			//1.configure the RTSR
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//Clear the corresponding FTSR bit
			EXTI->FTSR &= ~ ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT){
			//1.configure the RFTSR
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->FTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		//2. configure the GPIO port selection in SYSCFG_EXTICR
		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1] = portcode << (temp2 * 4);

		//3. enable the exti interrupt delivery using IMR
		EXTI->IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}

	temp = 0;

	//2. configure the speed
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);//clearing
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;
	temp = 0;

	//3. configure the pupd settings

	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);//clearing
	pGPIOHandle->pGPIOx->PUPDR |= temp;
	temp = 0;

	//4. configure the optype

	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);//clearing
	pGPIOHandle->pGPIOx->OTYPER |= temp;
	temp = 0;

	//5. configure the alt. functionality.

	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		//configure the alt. func. registers
		uint8_t temp1, temp2;
		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << (4 * temp2));//clearing
		pGPIOHandle->pGPIOx->AFR[temp1] |= pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp2);
	}
}
/*******************************************************************
 * @fn 					-GPIO_DeInit
 *
 * @brief				- This function deinitializes the GPIO
 *
 * @param[in]			- base address of the gpio peripheral
 * @param[in]			-
 * @param[in]			-
 *
 * return				- none
 *
 * @Note				- none
 *
 *
 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
		if(pGPIOx == GPIOA)
		{
			GPIOA_REG_RESET();
		}
		else if(pGPIOx == GPIOB)
		{
			GPIOB_REG_RESET();
		}
		else if(pGPIOx == GPIOC)
		{
			GPIOC_REG_RESET();
		}
		else if(pGPIOx == GPIOD)
		{
			GPIOD_REG_RESET();
		}
		else if(pGPIOx == GPIOF)
		{
			GPIOF_REG_RESET();
		}

}

//Data read and write
/*******************************************************************
 * @fn 					-GPIO_ReadFromInputPin
 *
 * @brief				- This function reads the data from the input pin
 *
 * @param[in]			- base address of the gpio peripheral
 * @param[in]			- Pin Number of the GPIO
 * @param[in]			-
 *
 * return				- 0 or 1
 *
 * @Note				- none
 *
 *
 */

uint8_t  GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value;

	value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001);

	return value;

}
/*******************************************************************
 * @fn 					-GPIO_ReadFromInputPort
 *
 * @brief				- This function reads the data from the input port.
 *
 * @param[in]			- base address of the gpio peripheral
 * @param[in]			-
 * @param[in]			-
 *
 * return				- none
 *
 * @Note				- none
 *
 *
 */
uint16_t  GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;

	value = (uint16_t)pGPIOx->IDR;

	return value;
}
/*******************************************************************
 * @fn 					-GPIO_WriteToOutputPin
 *
 * @brief				- This function writes the data to the output pin.
 *
 * @param[in]			- base address of the gpio peripheral
 * @param[in]			- Pin Number of the GPIO
 * @param[in]			- Value to write to the output.
 *
 * return				- none
 *
 * @Note				- none
 *
 *
 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
	if(Value == GPIO_PIN_SET)
	{
		pGPIOx->ODR |= ( 1 << PinNumber);
	}
	else
	{
		pGPIOx->ODR &= ~( 1 << PinNumber);
	}
}
/*******************************************************************
 * @fn 					-GPIO_WriteToOutputPort
 *
 * @brief				- This function writes data to the output port.
 *
 * @param[in]			- base address of the gpio peripheral
 * @param[in]			- Value to write to the output.
 * @param[in]			-
 *
 * return				- none
 *
 * @Note				- none
 *
 *
 */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
	pGPIOx->ODR = Value;
}
/*******************************************************************
 * @fn 					-GPIO_ToggleOutputPin
 *
 * @brief				- This function toggles the output pins statement.
 *
 * @param[in]			- base address of the gpio peripheral
 * @param[in]			- Pin number of the GPIO.
 * @param[in]			-
 *
 * return				- none
 *
 * @Note				- none
 *
 *
 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= ( 1 << PinNumber );
}

//IRQ Config. and ISR Handling
/*******************************************************************
 * @fn 					-GPIO_IRQConfig
 *
 * @brief				- This function configures the interrupt which sending to the GPIO.
 *
 * @param[in]			- IRQ number
 * @param[in]			- IRQ Priority
 * @param[in]			- ENABLE or DISABLE macros
 *
 * return				- none
 *
 * @Note				- none
 *
 *
 */

void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			//PROGRAM ISER0 REGISTER
			*NVIC_ISER0 |= (1 << IRQNumber);
		}
		else if(IRQNumber > 31 && IRQNumber < 64) //32 TO 63
		{
			//PROGRAM ISER1 REGISTER
			*NVIC_ISER1 |= (1 << IRQNumber % 32);
		}
		else if(IRQNumber >= 64 && IRQNumber < 96)
		{
			//PROGRAM ISER2 REGISTER
			*NVIC_ISER2 |= (1 << IRQNumber % 64);
		}
	}
	else
	{
		if(IRQNumber <= 31)
	{
		//PROGRAM ICER0 REGISTER
		*NVIC_ICER0 |= (1 << IRQNumber);
	}
	else if(IRQNumber > 31 && IRQNumber < 64) //32 TO 63
	{
		//PROGRAM ICER1 REGISTER
		*NVIC_ICER1 |= (1 << IRQNumber % 32);
	}
	else if(IRQNumber >= 64 && IRQNumber < 96)
	{
		//PROGRAM ICER2 REGISTER
		*NVIC_ICER2 |= (1 << IRQNumber % 64);
	}

	}

}
/*******************************************************************
 * @fn 					-GPIO_IRQPriorityConfig
 *
 * @brief				- This function configures the priority of the IRQ
 *
 * @param[in]			- Number of the IRQ
 * @param[in]			-Priority of the IRQ
 * @param[in]			-
 *
 * return				- none
 *
 * @Note				- none
 *
 *
 */
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	//1.first find out the ipr register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;

	uint8_t shift_amount = ( 8 * iprx_section ) + ( 8 - NO_PR_BITS_IMPLEMENTED );
	*(NVIC_PR_BASE_ADDR + (iprx)) |= (IRQPriority << shift_amount);
}
/*******************************************************************
 * @fn 					-GPIO_IRQHandling
 *
 * @brief				- This function handles the irq
 *
 * @param[in]			- Pin number of the GPIO.
 * @param[in]			-
 * @param[in]			-
 *
 * return				- none
 *
 * @Note				- none
 *
 *
 */
void GPIO_IRQHandling(uint8_t PinNumber)
{
	//clear the EXTI PR register corresponding to the pin number
	if(EXTI->PR & ( 1 << PinNumber))
	{
		//clear
		EXTI->PR |= ( 1 << PinNumber);
	}

}
