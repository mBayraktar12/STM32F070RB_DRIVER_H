/*
 * stm32070rb.h
 *
 * 
 *      Author: MERT
 */

#ifndef INC_STM32F070XX_H_
#define INC_STM32F070XX_H_
#include <stdint.h>

//ARM CORTEX Mx PROCESSOR NVIC ISERx REGISTER ADRESSES
#define NVIC_ISER0 					 ((volatile uint32_t*)0xE000E100)
#define NVIC_ISER1 					 ((volatile uint32_t*)0xE000E104)
#define NVIC_ISER2 					 ((volatile uint32_t*)0xE000E108)
#define NVIC_ISER3 					 ((volatile uint32_t*)0xE000E10C)

//ARM CORTEX Mx PROCESSOR NVIC ICERx REGISTER ADRESSES
#define NVIC_ICER0					 ((volatile uint32_t*)0xE000E180)
#define NVIC_ICER1					 ((volatile uint32_t*)0xE000E180)
#define NVIC_ICER2					 ((volatile uint32_t*)0xE000E180)
#define NVIC_ICER3					 ((volatile uint32_t*)0xE000E180)

//ARM CORTEX Mx PROCESSOR PRIORITY REGISTER ADRESS CALC.
#define NVIC_PR_BASE_ADDR 			 ((volatile uint32_t*)0xE000E400)

#define NO_PR_BITS_IMPLEMENTED 			4



#define FLASH_BASEADDR               0x08000000U
#define SRAM1_BASEADDR				 0x20000000U
#define ROM_BASEADDR 				 0x1FFFC800U
#define SRAM 						 SRAM1_BASEADDR

#define PERIPH_BASEADDR				 0x40000000U
#define APBPERIPH_BASEADDR			 PERIPH_BASEADDR
#define AHB1PERIPH_BASEADDR			 0x40020000U
#define AHB2PERIPH_BASEADDR			 0x48000000U

#define GPIOA_BASEADDR				 (AHB2PERIPH_BASEADDR + 0x0000)
#define GPIOB_BASEADDR				 (AHB2PERIPH_BASEADDR + 0x0400)
#define GPIOC_BASEADDR				 (AHB2PERIPH_BASEADDR + 0x0800)
#define GPIOD_BASEADDR				 (AHB2PERIPH_BASEADDR + 0x0C00)
#define GPIOF_BASEADDR				 (AHB2PERIPH_BASEADDR + 0x1400)

#define RCC_BASEADDR 				 (AHB1PERIPH_BASEADDR + 0x1000)

#define I2C1_BASEADDR				 (APBPERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR				 (APBPERIPH_BASEADDR + 0x5800)

#define SPI1_BASEADDR				 0x40013000U
#define SPI2_BASEADDR				 (APBPERIPH_BASEADDR + 0x3800)

#define USART1_BASEADDR				 0x40013800U
#define USART2_BASEADDR				 (APBPERIPH_BASEADDR + 0x4400)
#define USART3_BASEADDR				 (APBPERIPH_BASEADDR + 0x4800)
#define USART4_BASEADDR				 (APBPERIPH_BASEADDR + 0x4C00)
#define USART5_BASEADDR				 (APBPERIPH_BASEADDR + 0x5000)
#define USART6_BASEADDR				 0x40011400U

#define EXTI_BASEADDR				 0x40010400U
#define SYSCFG_BASEADDR				 0x40010000U

//peripheral register definition structure for GPIO
typedef struct {

	volatile uint32_t MODER;
	volatile uint32_t OTYPER;
	volatile uint32_t OSPEEDR;
	volatile uint32_t PUPDR;
	volatile uint32_t IDR;
	volatile uint32_t ODR;
	volatile uint32_t BSRR;
	volatile uint32_t LCKR;
	volatile uint32_t AFR[2];
	volatile uint32_t BRR;
}GPIO_RegDef_t;

//peripheral register definition structure for RCC
typedef struct{

	volatile uint32_t CR;
	volatile uint32_t CFGR;
	volatile uint32_t CIR;
	volatile uint32_t APB2RSTR;
	volatile uint32_t APB1RSTR;
	volatile uint32_t AHBENR;
	volatile uint32_t APB2ENR;
	volatile uint32_t APB1ENR;
	volatile uint32_t BDCR;
	volatile uint32_t CSR;
	volatile uint32_t AHBRSTR;
	volatile uint32_t CFGR2;
	volatile uint32_t CFGR3;
	volatile uint32_t CR2;
}RCC_RegDef_t;

//peripheral register definition structure for EXTI
typedef struct{

	volatile uint32_t IMR;
	volatile uint32_t EMR;
	volatile uint32_t RTSR;
	volatile uint32_t FTSR;
	volatile uint32_t SWIER;
	volatile uint32_t PR;
}EXTI_RegDef_t;

//peripheral register definition structure for SPI
typedef struct
{
	volatile uint32_t CR1;
	volatile uint32_t CR2;
	volatile uint32_t SR;
	volatile uint32_t DR;
	volatile uint32_t CRCPR;
	volatile uint32_t RXCRCR;
	volatile uint32_t TXCRCR;
}SPI_RegDef_t;

//peripheral register definition structure for SYSCFG
typedef struct{

	volatile uint32_t CFGR1;
	volatile uint32_t CFGR2;
	volatile uint32_t EXTICR[3];
}SYSCFG_RegDef_t;

//Peripheral definitions (Peripheral base adresses typecasted to xxx_RegDef_t)
#define GPIOA  		 ((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB  		 ((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC  		 ((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD  		 ((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOF  		 ((GPIO_RegDef_t*)GPIOF_BASEADDR)

#define RCC			((RCC_RegDef_t*)RCC_BASEADDR)
#define EXTI		((EXTI_RegDef_t*)EXTI_BASEADDR)
#define SYSCFG		((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)

#define SPI1		((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2 		((SPI_RegDef_t*)SPI2_BASEADDR)

/*CLOCK ENABLE MACROS */

#define GPIOA_PCLK_EN()  (RCC->AHBENR |= (1 << 17))
#define GPIOB_PCLK_EN()  (RCC->AHBENR |= (1 << 18))
#define GPIOC_PCLK_EN()  (RCC->AHBENR |= (1 << 19))
#define GPIOD_PCLK_EN()  (RCC->AHBENR |= (1 << 20))
#define GPIOF_PCLK_EN()  (RCC->AHBENR |= (1 << 22))

#define I2C1_PCLK_EN()   (RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN()   (RCC->APB1ENR |= (1 << 22))

#define SPI1_PCLK_EN()   (RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN()   (RCC->APB1ENR |= (1 << 14))

#define USART1_PCLK_EN() (RCC->APB2ENR |= (1 << 14))
#define USART2_PCLK_EN() (RCC->APB1ENR |= (1 << 17))
#define USART3_PCLK_EN() (RCC->APB1ENR |= (1 << 18))
#define USART4_PCLK_EN() (RCC->APB1ENR |= (1 << 19))
#define USART5_PCLK_EN() (RCC->APB1ENR |= (1 << 20))
#define USART6_PCLK_EN() (RCC->APB2ENR |= (1 << 5))

#define SYSCFG_PCLK_EN() (RCC->APB2ENR |= (1 << 0))

/*CLOCK DISABLE MACROS*/
#define GPIOA_PCLK_DI()  (RCC->AHBENR &= ~(1 << 17))
#define GPIOB_PCLK_DI()  (RCC->AHBENR &= ~(1 << 18))
#define GPIOC_PCLK_DI()  (RCC->AHBENR &= ~(1 << 19))
#define GPIOD_PCLK_DI()  (RCC->AHBENR &= ~(1 << 20))
#define GPIOF_PCLK_DI()  (RCC->AHBENR &= ~(1 << 22))

//MACROS TO RESET THE GPIOx PERIPHERALS
#define GPIOA_REG_RESET() do{(RCC->AHBRSTR |= (1 << 17)); (RCC->AHBRSTR &= ~(1 << 17)); }while(0)
#define GPIOB_REG_RESET() do{(RCC->AHBRSTR |= (1 << 18)); (RCC->AHBRSTR &= ~(1 << 18)); }while(0)
#define GPIOC_REG_RESET() do{(RCC->AHBRSTR |= (1 << 19)); (RCC->AHBRSTR &= ~(1 << 19)); }while(0)
#define GPIOD_REG_RESET() do{(RCC->AHBRSTR |= (1 << 20)); (RCC->AHBRSTR &= ~(1 << 20)); }while(0)
#define GPIOF_REG_RESET() do{(RCC->AHBRSTR |= (1 << 22)); (RCC->AHBRSTR &= ~(1 << 22)); }while(0)

//Returns port code for given GPIOx base adresses
#define GPIO_BASEADDR_TO_CODE(x)	((x == GPIOA)?0:\
									 (x == GPIOB)?1:\
									 (x == GPIOC)?2:\
									 (x == GPIOD)?3:\
									 (x == GPIOF)?4:0)


//IRQ Numbers of STM32F070RB Macros
#define IRQ_NO_EXTI0_1 		5
#define IRQ_NO_EXTI2_3 		6
#define IRQ_NO_EXTI4_15		7

//IRQ PRIORITY MACROS
#define NVIC_IRQ_PRI0		0
#define NVIC_IRQ_PRI14		14


//some generic macros
#define ENABLE 				1
#define DISABLE 			0
#define SET 				ENABLE
#define RESET 				DISABLE
#define GPIO_PIN_SET    	SET
#define GPIO_PIN_RESET  	RESET

//CLOCK DISABLE MACROS
#define I2C1_PCLK_DI()   (RCC->APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DI()   (RCC->APB1ENR &= ~(1 << 22))

#define SPI1_PCLK_DI()   (RCC->APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_DI()   (RCC->APB1ENR &= ~(1 << 14))

#define USART1_PCLK_DI() (RCC->APB2ENR &= ~(1 << 14))
#define USART2_PCLK_DI() (RCC->APB1ENR &= ~(1 << 17))
#define USART3_PCLK_DI() (RCC->APB1ENR &= ~(1 << 18))
#define USART4_PCLK_DI() (RCC->APB1ENR &= ~(1 << 19))
#define USART5_PCLK_DI() (RCC->APB1ENR &= ~(1 << 20))
#define USART6_PCLK_DI() (RCC->APB2ENR &= ~(1 << 5))

#define SYSCFG_PCLK_DI() (RCC->APB2ENR &= ~(1 << 0))

//BIT POSITION DEFINITONS OF SPI PERIPHERAL

#define SPI_CR1_CPHA		0
#define SPI_CR1_CPOL		1
#define SPI_CR1_MSTR		2
#define SPI_CR1_BR			3
#define SPI_CR1_SPE			6
#define SPI_CR1_LSBFIRST	7
#define SPI_CR1_SSI			8
#define SPI_CR1_SSM			9
#define SPI_CR1_RXONLY		10
#define SPI_CR1_CRCL		11
#define SPI_CR1_CRCNEXT		12
#define SPI_CR1_CRCEN		13
#define SPI_CR1_BIDIOE		14
#define SPI_CR1_BIDIMODE	15

#define SPI_CR2_RXDMAEN		0
#define SPI_CR2_TXDMAEN		1
#define SPI_CR2_SSOE		2
#define SPI_CR2_NSSP		3
#define SPI_CR2_FRF			4
#define SPI_CR2_ERRIE		5
#define SPI_CR2_RXNEIE		6
#define SPI_CR2_TXNEIE		7
#define SPI_CR2_DS			8
#define SPI_CR2_FRXTH		12
#define SPI_CR2_LDMA_RX		13
#define SPI_CR2_LDMA_TX		14
#define RESERVED			15

#define SPI_SR_RXNE			0
#define SPI_SR_TXNE			1
#define RESERVED			2
#define RESERVED			3
#define SPI_SR_CRCERR		4
#define SPI_SR_MODF			5
#define SPI_SR_OVR			6
#define SPI_SR_BSY			7
#define SPI_SR_FRE			8
#define SPI_SR_FRVL			9
#define SPI_SR_FTVL			11
#define RESERVED			13
#define RESERVED			14
#define RESERVED			15




#include <stm32f070xx_gpio_driver.h>
#include <stm32f070xx_spi_driver.h>


#endif /* INC_STM32F070XX_H_ */
