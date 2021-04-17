/*
 * main.c
 *
 *  Created on: 18 Mar 2021
 *      Author: emine
 */

#include "stm32f407xx.h"

int main(void)
 {
	return 0;
 }

  void EXTI0_IRQHandler(void) //ISR implementation
  {
	  //handle the interrupt
	  GPIO_IRQHandling(0); //0 is pin number
  }
