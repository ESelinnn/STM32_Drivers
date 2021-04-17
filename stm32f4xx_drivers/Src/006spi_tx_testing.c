/*
 * 006spi_tx_testing.c
 *
 *  Created on: 28 Mar 2021
 *      Author: emine
 *
 *      PB15 -> SPI2_MOSI
 *      PB14 -> SPI2_MISO
 *      PB13 -> SPI2_SCLK
 *      PB12 -> SPI2_NSS
 *      ALT function mode : 5
 */
#include <string.h>
#include "stm32f407xx.h"

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
	//SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	//GPIO_Init(&SPIPins);

}

void SPI2_Inits(void)
{
	SPI_Handle_t SPI2Handle;
	SPI2Handle.pSPIx = SPI2;

	SPI2Handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2Handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2Handle.SPIConfig.SPI_CPHA = SPI_CPHA_FIRST_EDGE;
	SPI2Handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2Handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV2; //generates sclk of 8mhz
	SPI2Handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI2Handle.SPIConfig.SPI_SSM = SPI_SSM_EN;

	SPI_Init(&SPI2Handle);
}
int main(void)
	{

		char user_data[] ="HELLO WORLD";

		SPI2_GPIOInits();

		SPI2_Inits();
		// this makes NSS signal internally high and avoids MOF error
		// Error causes device mode became master to slave
		SPI_SSIConfig(SPI2,ENABLE);

		//enable the SPI2 peripheral
		SPI_PeriClockControl(SPI2, ENABLE);

		SPI_SendData(SPI2, (uint8_t*)user_data, strlen(user_data));

		//lets confirm SPI is not busy
		while(SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG));

		//disable the SPI2 peripheral
		SPI_PeriClockControl(SPI2, DISABLE);

		while(1);

		return 0;
	}
