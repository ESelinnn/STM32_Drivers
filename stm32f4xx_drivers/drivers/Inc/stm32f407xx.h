/*
 * stm32f407xx.h
 *
 *  Created on: Mar 15, 2021
 *      Author: emine
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include<stdint.h> //for data types
#include<stddef.h>
#define __vo 			volatile
#define __weak 			__attribute__((weak))

/**********************************START: Processor Specific Details**************************************************** */
/*ARM CORTEX M4 Processor NVIC ISERx Register Addresses */

#define NVIC_ISER0		( (__vo uint32_t*) 0xE000E100 )
#define NVIC_ISER1		(( __vo uint32_t*) 0xE000E104)
#define NVIC_ISER2		(( __vo uint32_t*) 0xE000E108)
#define NVIC_ISER3		(( __vo uint32_t*) 0xE000E10C)

/*ARM CORTEX M4 Processor NVIC ICERx Register Addresses */

#define NVIC_ICER0		(( __vo uint32_t*) 0XE000E180)
#define NVIC_ICER1		(( __vo uint32_t*) 0xE000E184)
#define NVIC_ICER2		(( __vo uint32_t*) 0xE000E188)
#define NVIC_ICER3		(( __vo uint32_t*) 0xE000E18C)

/*ARM CORTEX M4 Processor Priority Register Address Calculation */

#define NVIC_PR_BASE_ADDR (( __vo uint32_t*)0XE000E400)

/*ARM CORTEX M4 Processor number ofPriority  bits implemented in priority register */

#define NO_PR_BITS_IMPLEMENTED		4

/* MCU's EMBEDDED MEMORIES BASE ADDRESSES MACROS*/

#define FLASH_BASEADDR		0x08000000U // main memory
#define SRAM1_BASEADDR		0x20000000U //112KB
#define SRAM2_BASEADDR		0x2001C000U //16KB
#define ROM_BASEADDR		0x1FFF0000U // system memory 30KB
#define SRAM 				SRAM1_BASEADDR

/* BASE ADDRESS OF BUS DOMAIN MACROS*/

#define PERIPH_BASEADDR			0x40000000U
#define APB1PERIPH_BASEADDR		PERIPH_BASEADDR
#define APB2PERIPH_BASEADDR		0x40010000U
#define AHB1PERIPH_BASEADDR		0x40020000U
#define AHB2PERIPH_BASEADDR		0x50000000U

/*BASE ADRESSES OF PERIPHERALS HANGS ON AHB1 BUS*/

#define GPIOA_BASEADDR		(AHB1PERIPH_BASEADDR + 0X0000)
#define GPIOB_BASEADDR		(AHB1PERIPH_BASEADDR + 0X0400)
#define GPIOC_BASEADDR		(AHB1PERIPH_BASEADDR + 0X0800)
#define GPIOD_BASEADDR		(AHB1PERIPH_BASEADDR + 0X0C00)
#define GPIOE_BASEADDR		(AHB1PERIPH_BASEADDR + 0X1000)
#define GPIOF_BASEADDR		(AHB1PERIPH_BASEADDR + 0X1400)
#define GPIOG_BASEADDR		(AHB1PERIPH_BASEADDR + 0X1800)
#define GPIOH_BASEADDR		(AHB1PERIPH_BASEADDR + 0X1C00)
#define GPIOI_BASEADDR		(AHB1PERIPH_BASEADDR + 0X2000)

#define RCC_BASEADDR 		(AHB1PERIPH_BASEADDR + 0X3800)

/*BASE ADRESSES OF PERIPHERALS HANGS ON APB1 BUS*/

#define SPI2_BASEADDR		(APB1PERIPH_BASEADDR + 0X3800)
#define SPI3_BASEADDR		(APB1PERIPH_BASEADDR + 0X3C00)

#define USART2_BASEADDR		(APB1PERIPH_BASEADDR + 0X4400)
#define USART3_BASEADDR		(APB1PERIPH_BASEADDR + 0X4800)
#define UART4_BASEADDR		(APB1PERIPH_BASEADDR + 0X4C00)
#define UART5_BASEADDR		(APB1PERIPH_BASEADDR + 0X5000)

#define I2C1_BASEADDR		(APB1PERIPH_BASEADDR + 0X5400)
#define I2C2_BASEADDR		(APB1PERIPH_BASEADDR + 0X5800)
#define I2C3_BASEADDR		(APB1PERIPH_BASEADDR + 0X5C00)

#define UART7_BASEADDR		(APB1PERIPH_BASEADDR + 0X7800)
#define UART8_BASEADDR		(APB1PERIPH_BASEADDR + 0X7C00)

/*BASE ADRESSES OF PERIPHERALS HANGS ON APB2 BUS*/

#define USART1_BASEADDR		(APB2PERIPH_BASEADDR + 0X1000)
#define USART6_BASEADDR		(APB2PERIPH_BASEADDR + 0X1400)

#define SPI1_BASEADDR		(APB2PERIPH_BASEADDR + 0X3000)
//#define SPI4_BASEADDR		(APB2PERIPH_BASEADDR + 0X3400)

#define SYSCFG_BASEADDR		(APB2PERIPH_BASEADDR + 0X3800)
#define EXTI_BASEADDR		(APB2PERIPH_BASEADDR + 0x3C00)

//#define SPI5_BASEADDR		(APB2PERIPH_BASEADDR + 0X5000)
//#define SPI6_BASEADDR		(APB2PERIPH_BASEADDR + 0X5400)


/*PERIPHERAL REGISTER DEFINITION STRUCTURES */


typedef struct
{
	__vo uint32_t MODER;		/*GPIO port mode register,					Address offset: 0x00 */
	__vo uint32_t OTYPER;		/*GPIO port output type register,			Address offset: 0x04 */
	__vo uint32_t OSPEEDR;		/*GPIO port output speed register,			Address offset: 0x08 */
	__vo uint32_t PUPDR;		/*GPIO port pull-up/pull-down register, 	Address offset: 0x0C */
	__vo uint32_t IDR;			/*GPIO port input data register, 			Address offset: 0x10 */
	__vo uint32_t ODR;			/*GPIO port output data register,			Address offset: 0x14 */
	__vo uint32_t BSRR;			/*GPIO port bit set/reset register,			Address offset: 0x18 */
	__vo uint32_t LCKR;			/*GPIO port configuration lock register,	Address offset: 0x1C */
	__vo uint32_t AFR[2];		/*!< AFR[0] : GPIO alternate function low register, AF[1] : GPIO alternate function high register    		Address offset: 0x20-0x24 */
	//__vo uint32_t AFRL;		/*GPIO alternate function low register, 	Address offset: 0x20 */
	//__vo uint32_t AFRH;		/*GPIO alternate function high register,	Address offset: 0x24 */

}GPIO_RegDef_t;

/*PERIPHERAL REGISTER DEFINITION STRUCTURES FOR RCC */

typedef struct
{

	__vo uint32_t CR; 			/*RCC clock control register 									Address offset: 0x00*/
	__vo uint32_t PLLCFGR; 		/*RCC PLL configuration register 								Address offset: 0x04 */
	__vo uint32_t CFGR; 		/*RCC clock configuration register 								Address offset: 0x08 */
	__vo uint32_t CIR; 			/*RCC clock interrupt register 									Address offset: 0x0C */
	__vo uint32_t AHB1RSTR; 	/*RCC AHB1 peripheral reset register 							Address offset: 0x10 */
	__vo uint32_t AHB2RSTR; 	/*RCC AHB2 peripheral reset register 							Address offset: 0x14 */
	__vo uint32_t AHB3RSTR; 	/*RCC AHB3 peripheral reset register 							Address offset: 0x18 */
		 uint32_t RESERVED0;	/*0x1C */
	__vo uint32_t APB1RSTR; 	/*RCC APB1 peripheral reset register 							Address offset: 0x20 */
	__vo uint32_t APB2RSTR; 	/*RCC APB2 peripheral reset register 							Address offset: 0x24 */
		 uint32_t RESERVED1[2]; /*0x28-0x2C */
	__vo uint32_t AHB1ENR; 		/*RCC AHB1 peripheral clock enable register 					Address offset: 0x30 */
	__vo uint32_t AHB2ENR; 		/*RCC AHB2 peripheral clock enable register 					Address offset: 0x34 */
	__vo uint32_t AHB3ENR; 		/*RCC AHB3 peripheral clock enable register 					Address offset: 0x38 */
		 uint32_t RESERVED2;	/*0x3C */
	__vo uint32_t APB1ENR; 		/*RCC APB1 peripheral clock enable register 					Address offset: 0x40 */
	__vo uint32_t APB2ENR; 		/*RCC APB2 peripheral clock enable register 					Address offset: 0x44 */
		 uint32_t RESERVED3[2]; /*0x48-0x4C */
	__vo uint32_t AHB1LPENR; 	/*RCC AHB1 peripheral clock enable in low power mode register 	Address offset: 0x50 */
	__vo uint32_t AHB2LPENR; 	/*RCC AHB2 peripheral clock enable in low power mode register 	Address offset: 0x54 */
	__vo uint32_t AHB3LPENR; 	/*RCC AHB3 peripheral clock enable in low power mode register 	Address offset: 0x58 */
	 	 uint32_t RESERVED4;	/*0x5C */
	__vo uint32_t APB1LPENR; 	/*RCC APB1 peripheral clock enable in low power mode register 	Address offset: 0x60 */
	__vo uint32_t APB2LPENR; 	/*RCC APB2 peripheral clock enabled in low power mode register 	Address offset: 0x64 */
		 uint32_t RESERVED5[2];	/*0x68-0x6C */
	__vo uint32_t BDCR; 		/*RCC Backup domain control register 							Address offset: 0x70 */
	__vo uint32_t CSR; 			/*RCC clock control & status register 							Address offset: 0x74 */
		 uint32_t RESERVED6[2];	/*0x78-0x7C */
	__vo uint32_t SSCGR; 		/*RCC spread spectrum clock generation 							Address offset: 0x80 */
	__vo uint32_t PLLI2SCFGR; 	/*RCC PLLI2S configuration register 							Address offset: 0x84 */

}RCC_RegDef_t;


/*PERIPHERAL REGISTER DEFINITION STRUCTURES FOR EXTI */

typedef struct
{
	__vo uint32_t IMR;		/*Interrupt mask register,					Address offset: 0x00 */
	__vo uint32_t EMR;		/*Event mask register,			 			Address offset: 0x04 */
	__vo uint32_t RTSR;		/*Rising trigger selection register,		Address offset: 0x08 */
	__vo uint32_t FTSR;		/*Falling trigger selection register, 		Address offset: 0x0C */
	__vo uint32_t SWIER;	/*Software interrupt event register, 		Address offset: 0x10 */
	__vo uint32_t PR;		/*Pending register,							Address offset: 0x14 */

}EXTI_RegDef_t;

/*PERIPHERAL REGISTER DEFINITION STRUCTURES FOR SYSCFG */

typedef struct
{
	__vo uint32_t MEMRMP;		/*memory remap register,						Address offset: 0x00 */
	__vo uint32_t PMC;			/*peripheral mode configuration register,		Address offset: 0x04 */
	__vo uint32_t EXTICR[4];	/*external interrupt configuration register,	Address offset: 0x08-0X14 */
	__vo uint32_t RESERVED[2];  /* 												Address offset: 0x18-0x1C */
	__vo uint32_t CMPCR;		/*Compensation cell control register,			Address offset: 0x20 */

}SYSCFG_RegDef_t;


typedef struct
{
	__vo uint32_t CR1;			/*SPI control register 1,						Address offset: 0x00 */
	__vo uint32_t CR2;			/*SPI control register 2,						Address offset: 0x04 */
	__vo uint32_t SR;			/*SPI status register,							Address offset: 0x08*/
	__vo uint32_t DR;  			/*SPI data register, 							Address offset:0X0C */
	__vo uint32_t CRCPR;		/*SPI CRC polynomial register,					Address offset: 0x10 */
	__vo uint32_t RXCRCR;		/*SPI RX CRC register,							Address offset: 0x14 */
	__vo uint32_t TXCRCR;		/*SPI TX CRC register,							Address offset: 0x18 */
	__vo uint32_t I2SCFGR;		/*SPI_I2S configuration register,				Address offset: 0x1C */
	__vo uint32_t I2SPR;		/*SPI_I2S prescaler register,					Address offset: 0x20 */

}SPI_RegDef_t;


typedef struct
{
	__vo uint32_t CR1;			/*I2C control register 1,						Address offset: 0x00 */
	__vo uint32_t CR2;			/*I2C control register 2,						Address offset: 0x04 */
	__vo uint32_t OAR1;			/*I2C Own address register 1,					Address offset: 0x08*/
	__vo uint32_t OAR2;  		/*I2C Own address register 2,					Address offset:0X0C */
	__vo uint32_t DR;			/*I2C Data register,							Address offset: 0x10 */
	__vo uint32_t SR1;			/*I2C Status register 1,						Address offset: 0x14 */
	__vo uint32_t SR2;			/*I2C Status register 2,						Address offset: 0x18 */
	__vo uint32_t CCR;			/*I2C Clock control register,					Address offset: 0x1C */
	__vo uint32_t TRISE;		/*I2C TRISE register,							Address offset: 0x20 */
	__vo uint32_t FLTR;			/*I2C FLTR register,							Address offset: 0x24 */

}I2C_RegDef_t;




#define GPIOA 		( (GPIO_RegDef_t*) GPIOA_BASEADDR )
#define GPIOB 		( (GPIO_RegDef_t*) GPIOB_BASEADDR )
#define GPIOC 		( (GPIO_RegDef_t*) GPIOC_BASEADDR )
#define GPIOD 		( (GPIO_RegDef_t*) GPIOD_BASEADDR )
#define GPIOE 		( (GPIO_RegDef_t*) GPIOE_BASEADDR )
#define GPIOF 		( (GPIO_RegDef_t*) GPIOF_BASEADDR )
#define GPIOG 		( (GPIO_RegDef_t*) GPIOG_BASEADDR )
#define GPIOH 		( (GPIO_RegDef_t*) GPIOH_BASEADDR )
#define GPIOI 		( (GPIO_RegDef_t*) GPIOI_BASEADDR )

#define RCC			( (RCC_RegDef_t*) RCC_BASEADDR )

#define EXTI		( (EXTI_RegDef_t*) EXTI_BASEADDR )

#define SYSCFG		((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)

#define SPI1		((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2		((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3		((SPI_RegDef_t*)SPI3_BASEADDR)

#define I2C1		((I2C_RegDef_t*)I2C1_BASEADDR)
#define I2C2		((I2C_RegDef_t*)I2C2_BASEADDR)
#define I2C3		((I2C_RegDef_t*)I2C3_BASEADDR)

/*Clock Enable Macros for GPIOx peripherals */

#define GPIOA_PCLK_EN() 	( RCC->AHB1ENR |=( 1<< 0) ) //GPIOA PERIPHERAL CLOCK ENABLE
#define GPIOB_PCLK_EN() 	( RCC->AHB1ENR |=( 1<< 1) )
#define GPIOC_PCLK_EN() 	( RCC->AHB1ENR |=( 1<< 2) )
#define GPIOD_PCLK_EN() 	( RCC->AHB1ENR |=( 1<< 3) )
#define GPIOE_PCLK_EN() 	( RCC->AHB1ENR |=( 1<< 4) )
#define GPIOF_PCLK_EN() 	( RCC->AHB1ENR |=( 1<< 5) )
#define GPIOG_PCLK_EN() 	( RCC->AHB1ENR |=( 1<< 6) )
#define GPIOH_PCLK_EN() 	( RCC->AHB1ENR |=( 1<< 7) )
#define GPIOI_PCLK_EN() 	( RCC->AHB1ENR |=( 1<< 8) )

/*Clock Enable Macros for I2Cx peripherals */

#define I2C1_PCLK_EN() 	( RCC->APB1ENR |=( 1<< 21) )
#define I2C2_PCLK_EN() 	( RCC->APB1ENR |=( 1<< 22) )
#define I2C3_PCLK_EN() 	( RCC->APB1ENR |=( 1<< 23) )

/*Clock Enable Macros for SPIx peripherals */

#define SPI1_PCLK_EN() 	( RCC->APB2ENR |=( 1<< 12) )

#define SPI2_PCLK_EN() 	( RCC->APB1ENR |=( 1<< 14) )
#define SPI3_PCLK_EN() 	( RCC->APB1ENR |=( 1<< 15) )

/*Clock Enable Macros for USARTx peripherals */

#define USART2_PCLK_EN() 	( RCC->APB1ENR |=( 1<< 17) )
#define USART3_PCLK_EN() 	( RCC->APB1ENR |=( 1<< 18) )

#define UART4_PCLK_EN() 	( RCC->APB1ENR |=( 1<< 19) )
#define UART5_PCLK_EN() 	( RCC->APB1ENR |=( 1<< 20) )

#define USART1_PCLK_EN() 	( RCC->APB2ENR |=( 1<< 4) )
#define USART6_PCLK_EN() 	( RCC->APB2ENR |=( 1<< 5) )

/*Clock Enable Macros for SYSCFG peripherals */

#define SYSCFG_PCLK_EN() 	( RCC->APB2ENR |=( 1<< 14) )


/*Clock Disable Macros for GPIOx peripherals */

#define GPIOA_PCLK_DN() 	( RCC->AHB1ENR &= ~( 1<< 0) ) //GPIOA PERIPHERAL CLOCK DISABLE
#define GPIOB_PCLK_DN() 	( RCC->AHB1ENR &= ~( 1<< 1) ) //GPIOA PERIPHERAL CLOCK DISABLE
#define GPIOC_PCLK_DN() 	( RCC->AHB1ENR &= ~( 1<< 2) ) //GPIOA PERIPHERAL CLOCK DISABLE
#define GPIOD_PCLK_DN() 	( RCC->AHB1ENR &= ~( 1<< 3) ) //GPIOA PERIPHERAL CLOCK DISABLE
#define GPIOE_PCLK_DN() 	( RCC->AHB1ENR &= ~( 1<< 4) ) //GPIOA PERIPHERAL CLOCK DISABLE
#define GPIOF_PCLK_DN() 	( RCC->AHB1ENR &= ~( 1<< 5) ) //GPIOA PERIPHERAL CLOCK DISABLE
#define GPIOG_PCLK_DN() 	( RCC->AHB1ENR &= ~( 1<< 6) ) //GPIOA PERIPHERAL CLOCK DISABLE
#define GPIOH_PCLK_DN() 	( RCC->AHB1ENR &= ~( 1<< 7) ) //GPIOA PERIPHERAL CLOCK DISABLE
#define GPIOI_PCLK_DN() 	( RCC->AHB1ENR &= ~( 1<< 8) ) //GPIOA PERIPHERAL CLOCK DISABLE

/*Clock Disable Macros for I2Cx peripherals */

#define I2C1_PCLK_DN() 	( RCC->APB1ENR &= ~( 1<< 21) )
#define I2C2_PCLK_DN() 	( RCC->APB1ENR &= ~( 1<< 22) )
#define I2C3_PCLK_DN() 	( RCC->APB1ENR &= ~( 1<< 23) )

/*Clock Disable Macros for SPIx peripherals */

#define SPI1_PCLK_DN() 	( RCC->APB2ENR &= ~( 1<< 12) )

#define SPI2_PCLK_DN() 	( RCC->APB1ENR &= ~( 1<< 14) )
#define SPI3_PCLK_DN() 	( RCC->APB1ENR &= ~( 1<< 15) )

/*Clock Disable Macros for USARTx peripherals */

#define USART2_PCLK_DN() 	( RCC->APB1ENR &= ~( 1<< 17) )
#define USART3_PCLK_DN() 	( RCC->APB1ENR &= ~( 1<< 18) )

#define UART4_PCLK_DN() 	( RCC->APB1ENR &= ~( 1<< 19) )
#define UART5_PCLK_DN() 	( RCC->APB1ENR &= ~( 1<< 20) )

#define USART1_PCLK_DN() 	( RCC->APB2ENR &= ~( 1<< 4) )
#define USART6_PCLK_DN() 	( RCC->APB2ENR &= ~( 1<< 5) )

/*Clock Disable Macros for SYSCFG peripherals */

#define SYSCFG_PCLK_DN() 	( RCC->APB2ENR &= ~( 1<< 14) )

/*Macros to reset GPIOx Peripherals */

#define GPIOA_REG_RESET()		do{ ( RCC->AHB1RSTR |= ( 1<< 0)); ( RCC->AHB1RSTR &= ~( 1<< 0) );} while(0)
#define GPIOB_REG_RESET()		do{ ( RCC->AHB1RSTR |= ( 1<< 1)); ( RCC->AHB1RSTR &= ~( 1<< 1) );} while(0)
#define GPIOC_REG_RESET()		do{ ( RCC->AHB1RSTR |= ( 1<< 2)); ( RCC->AHB1RSTR &= ~( 1<< 2) );} while(0)
#define GPIOD_REG_RESET()		do{ ( RCC->AHB1RSTR |= ( 1<< 3)); ( RCC->AHB1RSTR &= ~( 1<< 3) );} while(0)
#define GPIOE_REG_RESET()		do{ ( RCC->AHB1RSTR |= ( 1<< 4)); ( RCC->AHB1RSTR &= ~( 1<< 4) );} while(0)
#define GPIOF_REG_RESET()		do{ ( RCC->AHB1RSTR |= ( 1<< 5)); ( RCC->AHB1RSTR &= ~( 1<< 5) );} while(0)
#define GPIOG_REG_RESET()		do{ ( RCC->AHB1RSTR |= ( 1<< 6)); ( RCC->AHB1RSTR &= ~( 1<< 6) );} while(0)
#define GPIOH_REG_RESET()		do{ ( RCC->AHB1RSTR |= ( 1<< 7)); ( RCC->AHB1RSTR &= ~( 1<< 7) );} while(0)
#define GPIOI_REG_RESET()		do{ ( RCC->AHB1RSTR |= ( 1<< 8)); ( RCC->AHB1RSTR &= ~( 1<< 8) );} while(0)

//

#define SPI1_REG_RESET()		do{ (RCC->APB2RSTR |= (1<<12)); ( RCC->APB2RSTR &=~( 1<<12 ) );} while(0)

#define SPI2_REG_RESET()		do{ (RCC->APB1RSTR |= (1<<14)); ( RCC->APB1RSTR &=~( 1<<14 ) );} while(0)
#define SPI3_REG_RESET()		do{ (RCC->APB1RSTR |= (1<<15)); ( RCC->APB1RSTR &=~( 1<<15 ) );} while(0)


#define GPIO_BASEADDR_TO_CODE(x)			( (x == GPIOA) ? 0 :\
											  (x == GPIOB) ? 1 :\
											  (x == GPIOC) ? 2 :\
											  (x == GPIOD) ? 3 :\
											  (x == GPIOE) ? 4 :\
											  (x == GPIOF) ? 5 :\
											  (x == GPIOG) ? 6 :\
											  (x == GPIOH) ? 7 :\
											  (x == GPIOI) ? 8 :0 )

/*IRQ(Interrupt request) numbers of STM32F407X MCU*/

#define IRQ_NO_EXTI0		6
#define IRQ_NO_EXTI1		7
#define IRQ_NO_EXTI2		8
#define IRQ_NO_EXTI3		9
#define IRQ_NO_EXTI4		10
#define IRQ_NO_EXTI9_5		23
#define IRQ_NO_EXTI15_10	40

/*SOME GENERIC MACROS */
#define NVIC_IRQ_PRI0	0
#define NVIC_IRQ_PRI15	15
#define NVIC_IRQ_PRI13	13



/*SOME GENERIC MACROS */

#define ENABLE 			1
#define DISABLE 		0
#define SET 			ENABLE
#define RESET 			DISABLE
#define GPIO_PIN_SET 	SET
#define GPIO_PIN_RESET 	RESET
#define FLAG_RESET 		RESET
#define FLAG_SET		SET

/*BIT POSITIONS DEFINITIONS OF SPI PERIPHERAL */

/*BIT POSITIONS DEFINITIONS OF SPI_CR1 */

#define SPI_CR1_CPHA		0
#define SPI_CR1_CPOL		1
#define SPI_CR1_MSTR		2
#define SPI_CR1_BR			3
#define SPI_CR1_SPE			6
#define SPI_CR1_LSBFIRST	7
#define SPI_CR1_SSI			8
#define SPI_CR1_SSM			9
#define SPI_CR1_RXONLY		10
#define SPI_CR1_DFF			11
#define SPI_CR1_CRNEXT		12
#define SPI_CR1_CRCEN		13
#define SPI_CR1_BIDIOE		14
#define SPI_CR1_BIDIMODE	15

/*BIT POSITIONS DEFINITIONS OF SPI_CR2 */

#define SPI_CR2_RXDMAEN		0
#define SPI_CR2_TXDMAEN		1
#define SPI_CR2_SSOE		2
#define SPI_CR2_FRF			4
#define SPI_CR2_ERRIE		5
#define SPI_CR2_RXNEIE		6
#define SPI_CR2_TXEIE		7

/*BIT POSITIONS DEFINITIONS OF SPI_SR */

#define SPI_SR_RXNE			0
#define SPI_SR_TXE			1
#define SPI_SR_CHSIDE		2
#define SPI_SR_UDR			3
#define SPI_SR_CRCERR		4
#define SPI_SR_MODF			5
#define SPI_SR_OVR			6
#define SPI_SR_BSY			7
#define SPI_SR_FRE			8


/*BIT POSITIONS DEFINITIONS OF I2C PERIPHERAL */

/*BIT POSITIONS DEFINITIONS OF I2C_CR1 */

#define I2C_CR1_PE					0
#define I2C_CR1_SMBUS				1
#define I2C_CR1_SMBTYPE				3
#define I2C_CR1_ENARP				4
#define I2C_CR1_ENPEC				5
#define I2C_CR1_ENGC				6
#define I2C_CR1_NOSTRETCH			7
#define I2C_CR1_START				8
#define I2C_CR1_STOP				9
#define I2C_CR1_ACK					10
#define I2C_CR1_POS					11
#define I2C_CR1_PEC					12
#define I2C_CR1_ALERT				13
#define I2C_CR1_SWRST				15

/*BIT POSITIONS DEFINITIONS OF I2C_CR2 */

#define I2C_CR2_FREQ					0
#define I2C_CR2_ITERREN					8
#define I2C_CR2_ITEVTEN					9
#define I2C_CR2_ITBUFEN					10
#define I2C_CR2_DMAEN					11
#define I2C_CR2_LAST					12

/* Bit position definitions I2C_OAR1 */
#define I2C_OAR1_ADD0    				 0
#define I2C_OAR1_ADD71 				 	 1
#define I2C_OAR1_ADD98  			 	 8
#define I2C_OAR1_ADDMODE   			 	15


/*BIT POSITIONS DEFINITIONS OF I2C_SR1 */

#define I2C_SR1_SB						0
#define I2C_SR1_ADDR					1
#define I2C_SR1_BTF						2
#define I2C_SR1_ADD10					3
#define I2C_SR1_STOPF					4
#define I2C_SR1_RXNE					6
#define I2C_SR1_TXE						7
#define I2C_SR1_BERR					8
#define I2C_SR1_ARLO					9
#define I2C_SR1_AF						10
#define I2C_SR1_OVR						11
#define I2C_SR1_PECERR					12
#define I2C_SR1_TIMEOUT					14
#define I2C_SR1_SMBALERT				15

/*BIT POSITIONS DEFINITIONS OF I2C_SR2 */

#define I2C_SR2_MSL						0
#define I2C_SR2_BUSY 					1
#define I2C_SR2_TRA 					2
#define I2C_SR2_GENCALL 				4
#define I2C_SR2_DUALF 					7

/* Bit position definitions I2C_CCR*/
#define I2C_CCR_CCR 					 0
#define I2C_CCR_DUTY 					14
#define I2C_CCR_FS  				 	15














#include "stm32f407xx_gpio_driver.h"
#include "stm32f407xx_spi_driver.h"
#include "stm32f407xx_i2c_driver.h"


#endif /* INC_STM32F407XX_H_ */
