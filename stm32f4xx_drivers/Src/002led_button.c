/*
 * 002led_button.c
 *
 *  Created on: 16 Mar 2021
 *      Author: emine
 */

#include "stm32f407xx.h"
#define HIGH ENABLE
#define LOW	 DISABLE
#define BUTTON_PRESSED HIGH
void delay(void)
{
	for(uint32_t i=0;i <500000/2; i++);
}
int main(void)
{
	GPIO_Handle_t gpioled;

	gpioled.pGPIOx=GPIOD;
	gpioled.GPIO_PinConfig.GPIO_PinNumber= GPIO_PIN_NO_12;
	gpioled.GPIO_PinConfig.GPIO_PinMode= GPIO_MODE_OUT;
	gpioled.GPIO_PinConfig.GPIO_PinOPType= GPIO_OP_TYPE_PP;
	gpioled.GPIO_PinConfig.GPIO_PinPuPdControl =GPIO_NO_PUPD;
	gpioled.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	GPIO_PeriClockControl(GPIOD, ENABLE);
	GPIO_Init(&gpioled);


	GPIO_Handle_t BUTTON;

	BUTTON.pGPIOx= GPIOA;
	BUTTON.GPIO_PinConfig.GPIO_PinNumber= GPIO_PIN_NO_0;
	BUTTON.GPIO_PinConfig.GPIO_PinMode= GPIO_MODE_IN;
	BUTTON.GPIO_PinConfig.GPIO_PinPuPdControl= GPIO_PIN_PD;
	BUTTON.GPIO_PinConfig.GPIO_PinSpeed= GPIO_SPEED_FAST;

	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_Init(&BUTTON);

	while(1)
	{

		if(GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0)== BUTTON_PRESSED )
		{
			delay();
			GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);

		}
	}
	return 0;
}
