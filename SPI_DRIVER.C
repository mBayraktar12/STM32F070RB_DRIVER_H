/*
 * stm32f070xx_spi_driver.c
 *
 *  
 *      Author: MERT
 */
#include <stm32f070xx_spi_driver.h>

/*******************************************************************
 * @fn 					-SPI_PeriClockControl
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

void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if( EnorDi == ENABLE )
		{
			if(pSPIx == SPI1)
			{
				SPI1_PCLK_EN();
			}
			else if(pSPIx == SPI2)
			{
				SPI2_PCLK_EN();
			}
		}
		else
		{
			//TODO
		}

}

//Init and De-init
/*******************************************************************
 * @fn 					-SPI_Init
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

void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	//First configure the SPI_CR1 registet
	uint32_t tempreg = 0;

	//1. CONFIGURE THE DEVICE MODE
	tempreg |= pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR; //master

	//2. CONFIGURE THE BUS CONFIG
	if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{
		//BIDI MODE SHOULD BE CLEARED
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
	}
	else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{
		//BIDI MODE SHOULD BE SET
		tempreg |= (1 << SPI_CR1_BIDIMODE);
	}
	else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
	{
		//BIDI MODE SHOULD BE CLEARED
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
		//RXONLY BIT MUST BE SET
		tempreg |= (1 << SPI_CR1_RXONLY);
	}

	//3. CONFIGURE THE SPI SERIAL CLOCK SPEED (BAUD RATE)
	tempreg |= pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR;

	//4. CONFIGURE THE DFF
	tempreg |= pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_BR;

	//5. CONFIGURE THE CPOL
	tempreg |= pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL;

	//6. CONFIGURE THE CPHA
	tempreg |= pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA;

	pSPIHandle->pSPIx->CR1 = tempreg;
}
/*******************************************************************
 * @fn 					-SPI_DeInit
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
void SPI_DeInit(SPI_RegDef_t *pSPIx)
{

}

//Data read and write
/*******************************************************************
 * @fn 					-SPI_SendData
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

void  SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len)
{

}

/*******************************************************************
 * @fn 					-SPI_ReceiveData
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
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len)
{

}

//IRQ Config. and ISR Handling
/*******************************************************************
 * @fn 					-SPI_IRQConfig
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

void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{

}
/*******************************************************************
 * @fn 					-SPI_IRQPriorityConfig
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
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{

}
/*******************************************************************
 * @fn 					-SPI_IRQHandling
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
void SPI_IRQHandling(SPI_Handle_t *pHandle)
{


}


