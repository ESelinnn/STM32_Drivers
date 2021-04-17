 /*
 * 005button_interrupt.c
 *
 *  Created on: 19 Mar 2021
 *      Author: emine
 */
#include <string.h>
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
	GPIO_Handle_t gpioled ,BUTTON;
	memset(&gpioled, 0, sizeof(gpioled)); // this will set the the elements of the structure gpioled incase of bad values
	memset(&BUTTON, 0, sizeof(BUTTON)); 	//#include <string.h>


	gpioled.pGPIOx=GPIOD;
	gpioled.GPIO_PinConfig.GPIO_PinNumber= GPIO_PIN_NO_13;
	gpioled.GPIO_PinConfig.GPIO_PinMode= GPIO_MODE_OUT;
	gpioled.GPIO_PinConfig.GPIO_PinOPType= GPIO_OP_TYPE_PP;
	gpioled.GPIO_PinConfig.GPIO_PinPuPdControl =GPIO_NO_PUPD;
	gpioled.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	GPIO_PeriClockControl(GPIOD, ENABLE);
	GPIO_Init(&gpioled);




	BUTTON.pGPIOx= GPIOA;
	BUTTON.GPIO_PinConfig.GPIO_PinNumber= GPIO_PIN_NO_0;
	BUTTON.GPIO_PinConfig.GPIO_PinMode= GPIO_MODE_IT_FT;
	BUTTON.GPIO_PinConfig.GPIO_PinPuPdControl= GPIO_PIN_PD;
	BUTTON.GPIO_PinConfig.GPIO_PinSpeed= GPIO_SPEED_FAST;

	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_Init(&BUTTON);

	//IRQ configuration
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI0, NVIC_IRQ_PRI13);
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI0, ENABLE);

	while(1);


}


void EXTI0_IRQHandler(void)
{
	delay(); //200ms
	GPIO_IRQHandling(GPIO_PIN_NO_0);
	GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_13);
}
