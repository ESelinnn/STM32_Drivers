/*
 * stm32f407xx_i2c_driver.c
 *
 *  Created on: 5 Nis 2021
 *      Author: emine
 */
#include "stm32f407xx_i2c_driver.h"

uint16_t AHB_PreScaler[8] = { 2,4,8,16,64,128,256,512};
uint8_t APB1_PreScaler[4] = { 2,4,8,16};

static  void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);
static void I2C_ExecuteAddressPhase(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_ClearADDRFlag(I2C_RegDef_t *pI2Cx);
static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);

static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx)
{
	 pI2Cx-> CR1 |= (1 << I2C_CR1_START);
}
static void I2C_ExecuteAddressPhase(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{
	SlaveAddr = SlaveAddr << 1; //from 1. bit to 7. bit in case of 7 bit slave address
	SlaveAddr &= ~(1); // slaveaddr is  slave address + r/nw bit=0
	pI2Cx -> DR =SlaveAddr;

}
static void I2C_ClearADDRFlag(I2C_RegDef_t *pI2Cx)
{
	uint32_t dummyread = pI2Cx->SR1;
	dummyread = pI2Cx->SR2;
	(void) dummyread; // for error: (unused variable) we must write this line
}


static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx-> CR1 |= (1 << I2C_CR1_STOP);
}

void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pI2Cx->CR1 |= (1 << I2C_CR1_PE);
		//pI2cBaseAddress->CR1 |= I2C_CR1_PE_Bit_Mask;
	}else
	{
		pI2Cx->CR1 &= ~(1 << 0);
	}

}
/*********************************************************************
 * @fn      		  - I2C_PeriClockControl
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -
 */
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if(EnorDi==ENABLE)
			{
				if(pI2Cx==I2C1)
				{
					I2C1_PCLK_EN();
				}else if(pI2Cx==I2C2)
				{
					I2C2_PCLK_EN();
				}else if(pI2Cx==I2C3)
				{
					I2C3_PCLK_EN();
				}
			}
		else
		{
			if(pI2Cx==I2C1)
			{
				I2C1_PCLK_DN();
			}else if(pI2Cx==I2C2)
			{
				I2C2_PCLK_DN();
			}else if(pI2Cx==I2C3)
			{
				I2C3_PCLK_DN();
			}

		}
}

uint32_t RCC_GetPCLK1Value(void)
{
	uint32_t pclk1,SystemClk;

	uint8_t clksrc, temp, ahbp, apb1;

	clksrc = ( ( RCC->CFGR >> 2 ) & 0X3); // CHANGE THE BIT POSISTION OF 2. AND 3. BİTS TO 0. AND 1.
										  //AND USE AND 11 TO INDENTIFY THE VALUE
	if( clksrc ==0)
	{
		SystemClk = 16000000; //HSI
	}else if(clksrc ==1)
	{
		SystemClk = 8000000; //HSE
	}else if(clksrc ==2)
	{
		SystemClk = RCC_GetPLLOutputClock(); //PLL didn't write
	}
	//ahb
	temp = ((RCC -> CFGR >> 4 ) & 0XF);

	if(temp < 8)
	{
		ahbp = 1;
	}else
	{
		ahbp = AHB_PreScaler[temp-8];
	}
	//apb1
	temp = ((RCC -> CFGR >> 10 ) & 0x7);

	if(temp < 4)
	{
		apb1 = 1;
	}else
	{
		apb1 = APB1_PreScaler[temp-4];
	}


	pclk1 = (SystemClk / ahbp)/ apb1;

	return pclk1;

}
uint32_t RCC_GetPLLOutputClock(void)
{

	return 0;
}
/*********************************************************************
 * @fn      		  - I2C_Init
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -
 */
void I2C_Init(I2C_Handle_t *pI2CHandle)
{

	uint32_t tempreg = 0;

	//enable teh clock for I2C peripheral
	I2C_PeriClockControl(pI2CHandle ->pI2Cx, ENABLE);
	//I2C_PeripheralControl(I2C1,ENABLE);
	// ack control bit
	tempreg |= pI2CHandle -> I2C_Config.I2C_AckControl << 10;
	pI2CHandle -> pI2Cx ->CR1 |= tempreg;

	//configure the FREQ  field of CR2
	tempreg =0;
	tempreg |= RCC_GetPCLK1Value() / 1000000U ; // e.g. 16MHz
	pI2CHandle -> pI2Cx -> CR2 |= (tempreg & 0x3f); //mask all the bits accept first 5 bits

	//program the device own address
	tempreg =0;
	tempreg |= pI2CHandle -> I2C_Config.I2C_DeviceAddress << 1;
	tempreg |= (1 << 14);
	pI2CHandle-> pI2Cx -> OAR1 |= tempreg;

	//CCR Calculations
	uint16_t ccr_value =0;
	tempreg =0;
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		//MODE is standard mode
		ccr_value = (RCC_GetPCLK1Value() / (2 * pI2CHandle -> I2C_Config.I2C_SCLSpeed) );
		tempreg |= (ccr_value & 0xfff); //mask all the bits accept first 12 bits

	}else
	{
		//fast modes
		tempreg |= (1 << 15);
		tempreg |=( pI2CHandle -> I2C_Config.I2C_FMDutyCycle << 14 ) ;
		if( pI2CHandle -> I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2) //0
		{
			ccr_value = (RCC_GetPCLK1Value() / (3 * pI2CHandle -> I2C_Config.I2C_SCLSpeed) );


		}else if( pI2CHandle -> I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_16_9) //1
		{
			ccr_value = (RCC_GetPCLK1Value() / (25 * pI2CHandle -> I2C_Config.I2C_SCLSpeed) );

		}
		tempreg |= (ccr_value & 0xfff); //mask all the bits accept first 12 bits
	}
	pI2CHandle -> pI2Cx -> CCR =tempreg;
	tempreg =0;
	//Trise Configuration
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		//MODE is standard mode
		tempreg =( RCC_GetPCLK1Value() / 1000000U) +1;
	}
	else
	{
		tempreg =(( RCC_GetPCLK1Value() *300)/1000000000U) +1;
	}

	pI2CHandle -> pI2Cx -> TRISE = (tempreg & 0X3F);

}
/*********************************************************************
 * @fn      		  - I2C_DeInit
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -
 */
void I2C_DeInit(I2C_RegDef_t *pI2Cx)
{

}


uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName)
{
	if(pI2Cx ->SR1 & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;

}

/*********************************************************************
 * @fn      		  - I2C_MasterSendData
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -
 */
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle,uint8_t *pTxBuffer, uint32_t Len, uint8_t  SlaveAddr)
{
	// 1. Generate the START condition
	I2C_GenerateStartCondition(pI2CHandle-> pI2Cx);
	//2. confirm that start generation is completed by checking the SB flag in the SR1
	//   Note: Until SB is cleared SCL will be stretched (pulled to LOW)
	while(! I2C_GetFlagStatus(pI2CHandle-> pI2Cx, I2C_FLAG_SB ));
	//3. Send the address of the slave with r/nw bit set to w(0) (total 8 bits )
	I2C_ExecuteAddressPhase(pI2CHandle -> pI2Cx, SlaveAddr);
	//4. Confirm that address phase is completed by checking the ADDR flag in the SR1
	while(! I2C_GetFlagStatus(pI2CHandle-> pI2Cx, I2C_FLAG_ADDR ));
	//5. clear the ADDR flag according to its software sequence
	I2C_ClearADDRFlag(pI2CHandle-> pI2Cx);
	//6. send the data until len becomes 0
	while(Len> 0)
	{
		while(! I2C_GetFlagStatus(pI2CHandle-> pI2Cx, I2C_FLAG_TXE ));
		pI2CHandle -> pI2Cx-> DR = *pTxBuffer;
		pTxBuffer++;
		Len--;
	}
	//7. when Len becomes zero wait for TXE=1 and BTF=1 before generating the STOP condition
	//   Note: TXE=1 , BTF=1 , means that both SR and DR are empty and next transmission should begin
	//   when BTF=1 SCL will be stretched (pulled to LOW)
	while(! I2C_GetFlagStatus(pI2CHandle-> pI2Cx, I2C_FLAG_TXE ));

	while(! I2C_GetFlagStatus(pI2CHandle-> pI2Cx, I2C_FLAG_BTF ));

	//8. Generate STOP condition and master need not to wait for the completion of stop condition.
	//   Note: generating STOP, automatically clears the BTF
	I2C_GenerateStopCondition(pI2CHandle-> pI2Cx);
}



