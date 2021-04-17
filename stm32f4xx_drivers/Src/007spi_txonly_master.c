/*
 * 007spi_txonly_master.c
 *
 *  Created on: 30 Mar 2021
 *      Author: emine
 */


#include <string.h>
#include "stm32f407xx.h"
void delay(void)
{
	for(uint32_t i=0;i <500000/2; i++);
}
void SPI2_GPIOInits(void)
{
	GPIO_Handle_t SPIPins;
	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode= GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	//SCLK
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Init(&SPIPins);

	//MOSI
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	GPIO_Init(&SPIPins);

	//MISO
	//SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	//GPIO_Init(&SPIPins);

	//NSS
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIO_Init(&SPIPins);

}

void GPIO_ButtonInit(void)
{
	GPIO_Handle_t BUTTON;

	BUTTON.pGPIOx= GPIOA;
	BUTTON.GPIO_PinConfig.GPIO_PinNumber= GPIO_PIN_NO_0;
	BUTTON.GPIO_PinConfig.GPIO_PinMode= GPIO_MODE_IN;
	BUTTON.GPIO_PinConfig.GPIO_PinPuPdControl= GPIO_PIN_PD;
	BUTTON.GPIO_PinConfig.GPIO_PinSpeed= GPIO_SPEED_FAST;


	GPIO_Init(&BUTTON);

}

void SPI2_Inits(void)
{
	SPI_Handle_t SPI2Handle;
	SPI2Handle.pSPIx = SPI2;

	SPI2Handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2Handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2Handle.SPIConfig.SPI_CPHA = SPI_CPHA_FIRST_EDGE;
	SPI2Handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2Handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV8; //generates sclk of 8mhz
	SPI2Handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI2Handle.SPIConfig.SPI_SSM = SPI_SSM_DI;//hardware slave managment

	SPI_Init(&SPI2Handle);
}
int main(void)
	{

		char user_data[] ="HELLO WORLD"; // UP TO 255 CHARACTERS

		SPI2_GPIOInits();
		GPIO_ButtonInit();
		SPI2_Inits();

		// this makes NSS signal internally high and avoids MOF error
		// Error causes device mode became master to slave
		//SPI_SSIConfig(SPI2,ENABLE);

		/* Making SSOE 1 does NSS output enable
		 * the NSS pin is automatically managed by the hardware
		 * i.e when SPE=1 , NSS will be pulled to low
		 * and NSS pin will be high when SPE=0 */
		SPI_SSOEConfig(SPI2, ENABLE);
		while(1)
		{

			while(! GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));

			delay();
			//enable the SPI2 peripheral
			SPI_PeriClockControl(SPI2, ENABLE);

			//first send length information
			uint8_t dataLen = strlen(user_data);
			SPI_SendData(SPI2, &dataLen,1);
			//to send data
			SPI_SendData(SPI2, (uint8_t*)user_data, strlen(user_data));

			//lets confirm SPI is not busy
			while( SPI_GetFlagStatus(SPI2,SPI_BUSY_FLAG) );

			//disable the SPI2 peripheral
			SPI_PeriClockControl(SPI2, DISABLE);

		}

		return 0;
	}
