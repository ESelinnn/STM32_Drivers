/*
 * stm32f407xx_gpip_driver.c
 *
 *  Created on: Mar 15, 2021
 *      Author: emine
 */

#include "stm32f407xx_gpio_driver.h"


/*Peripheral Clock Setup */

void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
	if(EnorDi==ENABLE)
	{
		if(pGPIOx==GPIOA)
		{
			GPIOA_PCLK_EN();
		}else if(pGPIOx==GPIOB)
		{
			GPIOB_PCLK_EN();
		}else if(pGPIOx==GPIOC)
		{
			GPIOC_PCLK_EN();
		}else if(pGPIOx==GPIOD)
		{
			GPIOD_PCLK_EN();
		}else if(pGPIOx==GPIOE)
		{
			GPIOE_PCLK_EN();
		}else if(pGPIOx==GPIOF)
		{
			GPIOF_PCLK_EN();
		}else if(pGPIOx==GPIOG)
		{
			GPIOG_PCLK_EN();
		}else if(pGPIOx==GPIOH)
		{
			GPIOH_PCLK_EN();
		}else if(pGPIOx==GPIOI)
		{
			GPIOI_PCLK_EN();
		}
	}
	else
	{
		if(pGPIOx==GPIOA)
		{
			GPIOA_PCLK_DN();
		}else if(pGPIOx==GPIOB)
		{
			GPIOB_PCLK_DN();
		}else if(pGPIOx==GPIOC)
		{
			GPIOC_PCLK_DN();
		}else if(pGPIOx==GPIOD)
		{
			GPIOD_PCLK_DN();
		}else if(pGPIOx==GPIOE)
		{
			GPIOE_PCLK_DN();
		}else if(pGPIOx==GPIOF)
		{
			GPIOF_PCLK_DN();
		}else if(pGPIOx==GPIOG)
		{
			GPIOG_PCLK_DN();
		}else if(pGPIOx==GPIOH)
		{
			GPIOH_PCLK_DN();
		}else if(pGPIOx==GPIOI)
		{
			GPIOI_PCLK_DN();
		}
	}
}

/*Init and De-Init */

void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp=0;

	// enable the peripheral clock
	GPIO_PeriClockControl(pGPIOHandle-> pGPIOx, ENABLE);
	//1. configure the mode of gpio pin

	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode<=GPIO_MODE_ANALOG )
	{
		temp=(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <<(2 * pGPIOHandle ->GPIO_PinConfig.GPIO_PinNumber));
		//we are writing the pin mode to the specific pin number (0..15)
		pGPIOHandle-> pGPIOx->MODER &= ~(0x3 << pGPIOHandle ->GPIO_PinConfig.GPIO_PinNumber); //clearing
		pGPIOHandle-> pGPIOx->MODER |=temp; //setting

	}else
	{
		//interrupt mode
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode==GPIO_MODE_IT_FT)
		{
			//1. configure the FTSR
			EXTI->FTSR |= (1<< pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber);
			//clear the correspunding RTSR bit
			EXTI->RTSR &=~(1<< pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber);

		}else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode==GPIO_MODE_IT_RT)
		{
			//1. Configure the RTSR
			EXTI->RTSR |= (1<< pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber);
			//clear the correspunding RTSR bit
			EXTI->FTSR &=~(1<< pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber);

		}else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode==GPIO_MODE_IT_RFT)
		{
			//1. Configure the FTSR AND RTSR
			EXTI->RTSR |= (1<< pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber);
			EXTI->FTSR |= (1<< pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber);
		}
		//2. configure the GPIO port selection in SYSCFG_EXTICR

		uint8_t temp1= (pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber) / 4;
		uint8_t temp2= (pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber) % 4;
		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle -> pGPIOx);
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1] |= portcode << (temp2 * 4);

		//3. enable the EXTI interrupt delivery using IMR(interrupt master register)

		EXTI->IMR |= (1<< pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber);
	}


	//2. configure the speed

	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed<< (2* pGPIOHandle-> GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle-> pGPIOx->OSPEEDR &= ~(0x3 << pGPIOHandle ->GPIO_PinConfig.GPIO_PinNumber); //clearing
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;


	//3. configure the pupd settings

	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl<< (2* pGPIOHandle-> GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle-> pGPIOx->PUPDR &= ~(0x3 << pGPIOHandle ->GPIO_PinConfig.GPIO_PinNumber); //clearing
	pGPIOHandle->pGPIOx->PUPDR |= temp;


	//4. configure the optype

	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType<< pGPIOHandle-> GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle-> pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle ->GPIO_PinConfig.GPIO_PinNumber); //clearing
	pGPIOHandle->pGPIOx->OTYPER |= temp;


	//5. configure the alt functionality
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		//configure the alt function
		uint32_t temp1,temp2;
		temp1= pGPIOHandle-> GPIO_PinConfig.GPIO_PinNumber/8;
		temp2= pGPIOHandle-> GPIO_PinConfig.GPIO_PinNumber%8;
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0XF << (4* temp2));
		pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle-> GPIO_PinConfig.GPIO_PinAltFunMode<< (4* temp2));
	}

}
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if(pGPIOx==GPIOA)
	{
		GPIOA_REG_RESET();
	}else if(pGPIOx==GPIOB)
	{
		GPIOB_REG_RESET();
	}else if(pGPIOx==GPIOC)
	{
		GPIOC_REG_RESET();
	}else if(pGPIOx==GPIOD)
	{
		GPIOD_REG_RESET();
	}else if(pGPIOx==GPIOE)
	{
		GPIOE_REG_RESET();
	}else if(pGPIOx==GPIOF)
	{
		GPIOF_REG_RESET();
	}else if(pGPIOx==GPIOG)
	{
		GPIOG_REG_RESET();
	}else if(pGPIOx==GPIOH)
	{
		GPIOH_REG_RESET();
	}else if(pGPIOx==GPIOI)
	{
		GPIOI_REG_RESET();
	}

}

/*Data Read and Write */

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value;
	value=(uint8_t) (( pGPIOx-> IDR >> PinNumber) & 0x00000001); // whichever the pin that we need to read it ll shift to the 0. register bit
	return value;									//and we can read it from there

}
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t Value;
	Value= (uint16_t)pGPIOx-> IDR ; //read from input port
	return Value;

}
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber,uint8_t Value)
{
	if(Value == GPIO_PIN_SET)
	{
		// write 1 to the output data register at the bit field corresponding to the pin number
		pGPIOx-> ODR |= (1<< PinNumber);
	}else
	{
		//write 0
		pGPIOx-> ODR &= ~(1<<PinNumber);
	}

}
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
	pGPIOx-> ODR = Value;

}
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx-> ODR ^= (1<< PinNumber);

}

/*IRQ Configurations and ISR Handling */

void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if(EnorDi== ENABLE)
	{
		if(IRQNumber<=31)
		{
			//program ISER0 Register
			*NVIC_ISER0 |= (1 << IRQNumber);

		}else if (IRQNumber> 31 && IRQNumber< 64)
		{
			//program ISER1 Register
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));

		}else if(IRQNumber> 64 && IRQNumber< 96)
		{
			//program ISER2 Register
			*NVIC_ISER2 |= (1 << (IRQNumber % 64));
		}
		// We dont need to write till ISER7 because our MCU doesnt have that much interrupt but processor have!
	}else
	{
		if(IRQNumber<=31)
		{
			//program ICER0 Register
			*NVIC_ICER0 |= (1 << IRQNumber);

		}else if (IRQNumber> 31 && IRQNumber< 64)
		{
			//program ICER1 Register
			*NVIC_ICER1 |= (1 << ( IRQNumber % 32));

		}else if(IRQNumber> 64 && IRQNumber< 96)
		{
			//program ICER2 Register
			*NVIC_ICER2 |= (1 << ( IRQNumber % 64));
		}
	}

}
void GPIO_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority)
{
	uint8_t IPRx =IRQNumber /4;
	uint8_t IPRx_section = IRQNumber % 4;
	uint8_t shift_amount = (8 * IPRx_section ) + ( 8 - NO_PR_BITS_IMPLEMENTED);
	 *( NVIC_PR_BASE_ADDR +IPRx ) |= ( IRQPriority <<  shift_amount);


}
void GPIO_IRQHandling(uint8_t PinNumber)
{
	// clear the exti pr register corresponding to the pin number
	if(EXTI-> PR & (1 << PinNumber ))
	{
		//clear
		EXTI->PR |= (1 << PinNumber);
	}

}












