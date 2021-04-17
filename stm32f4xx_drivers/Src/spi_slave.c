/*
 * spi_slave.c
 *
 *  Created on: 31 Mar 2021
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
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	GPIO_Init(&SPIPins);

	//NSS
	//SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	//GPIO_Init(&SPIPins);

}

void GPIOInits(void)
{
	GPIO_Handle_t gpioled;

	gpioled.pGPIOx=GPIOD;
	gpioled.GPIO_PinConfig.GPIO_PinNumber= GPIO_PIN_NO_1;
	gpioled.GPIO_PinConfig.GPIO_PinMode= GPIO_MODE_OUT;
	gpioled.GPIO_PinConfig.GPIO_PinOPType= GPIO_OP_TYPE_OD;
	gpioled.GPIO_PinConfig.GPIO_PinPuPdControl =GPIO_PIN_PU;
	gpioled.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	GPIO_PeriClockControl(GPIOD, ENABLE);
	GPIO_Init(&gpioled);
}

void SPI2_Inits(void)
{
	SPI_Handle_t SPI2Handle;
	SPI2Handle.pSPIx = SPI2;

	SPI2Handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2Handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_SLAVE;
	SPI2Handle.SPIConfig.SPI_CPHA = SPI_CPHA_FIRST_EDGE;
	SPI2Handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	//SPI2Handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV8; //generates sclk of 8mhz
	SPI2Handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI2Handle.SPIConfig.SPI_SSM = SPI_SSM_EN;//hardware slave management


	SPI_Init(&SPI2Handle);
}
uint8_t SPI_VerifyResponse(uint8_t commandcode)
{
	if(commandcode== 0x50)
	{
		//ACK
		return 1;
	}
	return 0;
}
uint8_t SPI_LedResponse(uint8_t led_status)
{
	if(led_status== 0x60)
	{
		//ACK
		return 1;
	}
	return 0;
}
int main(void)
	{
		uint8_t dummy_write = 0xFF;
		uint8_t dummy_read;

		SPI2_GPIOInits();
		GPIOInits();
		SPI2_Inits();


		/* Making SSOE 1 does NSS output enable
		 * the NSS pin is automatically managed by the hardware
		 * i.e when SPE=1 , NSS will be pulled to low
		 * and NSS pin will be high when SPE=0 */
		//SPI_SSOEConfig(SPI2, ENABLE);
		SPI_SSIConfig(SPI2,DISABLE);
		while(1)
		{
			//enable the SPI2 peripheral
			SPI_PeriClockControl(SPI2, ENABLE);

			uint8_t commandcode;
			uint8_t ackbyte =0xf5;
			uint8_t led_status;
			//while(!(SPI2 -> DR == 0));
			//send some dummy bits to fetch the response from the master
			SPI_SendData(SPI2, &dummy_write, 1); //0xff
			//Receive command
			SPI_ReceiveData(SPI2,&commandcode,1);
			if(SPI_VerifyResponse(commandcode))
			{
				SPI_SendData(SPI2, &ackbyte, 1); //0xf5
				//Do dummy read to clear off the RXNE
				SPI_ReceiveData(SPI2,&dummy_read,1); //0xff

			}

			//send some dummy bits to fetch the response from the master
			SPI_SendData(SPI2, &dummy_write, 1);
			//Receive command
			SPI_ReceiveData(SPI2,&led_status,1);
			if(SPI_LedResponse(led_status))
			{
				GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);
				//SPI_SendData(SPI2, &ackbyte, 1);
				//Do dummy read to clear off the RXNE
				//SPI_ReceiveData(SPI2,&dummy_read,1);

			}

			//lets confirm SPI is not busy
			while( SPI_GetFlagStatus(SPI2,SPI_BUSY_FLAG) );

			//disable the SPI2 peripheral
			SPI_PeriClockControl(SPI2, DISABLE);

		}

		return 0;
	}
