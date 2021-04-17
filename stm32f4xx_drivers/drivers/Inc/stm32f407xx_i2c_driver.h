/*
 * stm32f407xx_i2c_driver.h
 *
 *  Created on: 5 Nis 2021
 *      Author: emine
 */

#ifndef INC_STM32F407XX_I2C_DRIVER_H_
#define INC_STM32F407XX_I2C_DRIVER_H_
//above we have guards for prevent inclusion of multiple definitions during the preprocessor stage of the compilation
//derlemenin önişlemci aşamasında birden çok tanımın dahil edilmesini önlemek için korumalarımız var
#include "stm32f407xx.h"

/*
 * Configuration structure for I2Cx peripheral
 */
typedef struct
{
	uint32_t I2C_SCLSpeed;
	uint8_t  I2C_DeviceAddress;
	uint8_t  I2C_AckControl;
	uint8_t  I2C_FMDutyCycle;

}I2C_Config_t;

/*
 *Handle structure for I2Cx peripheral
 */
typedef struct
{
	I2C_RegDef_t 	*pI2Cx;
	I2C_Config_t 	I2C_Config;
	uint8_t 		*pTxBuffer; /* !< To store the app. Tx buffer address > */
	uint8_t 		*pRxBuffer;	/* !< To store the app. Rx buffer address > */
	uint32_t 		TxLen;		/* !< To store Tx len > */
	uint32_t 		RxLen;		/* !< To store Tx len > */
	uint8_t 		TxRxState;	/* !< To store Communication state > */
	uint8_t 		DevAddr;	/* !< To store slave/device address > */
    uint32_t        RxSize;		/* !< To store Rx size  > */
    uint8_t         Sr;			/* !< To store repeated start value  > */
}I2C_Handle_t;

/*
 * @I2C_SCLSpeed
 */
#define I2C_SCL_SPEED_SM 	100000 //STANDARD MODE
#define I2C_SCL_SPEED_FM4K 	400000 //FAST MODE
#define I2C_SCL_SPEED_FM2K  200000

/*
 * @I2C_AckControl
 */
#define I2C_ACK_ENABLE        1
#define I2C_ACK_DISABLE       0


/*
 * @I2C_FMDutyCycle
 */
#define I2C_FM_DUTY_2        0
#define I2C_FM_DUTY_16_9     1

/*I2C RELATED STATUS FLAGS DEFINITIONS */
#define I2C_FLAG_TXE			( 1 << I2C_SR1_TXE )
#define I2C_FLAG_RXNE			( 1<< I2C_SR1_RXNE )
#define I2C_FLAG_SB				( 1 << I2C_SR1_SB )
#define I2C_FLAG_OVR			( 1 << I2C_SR1_OVR )
#define I2C_FLAG_AF				( 1 << I2C_SR1_AF )
#define I2C_FLAG_ARLO			( 1 << I2C_SR1_ARLO )
#define I2C_FLAG_BERR			( 1 << I2C_SR1_BERR )
#define I2C_FLAG_STOPF			( 1 << I2C_SR1_STOPF )
#define I2C_FLAG_ADD10			( 1 << I2C_SR1_ADD10 )
#define I2C_FLAG_BTF			( 1 << I2C_SR1_BTF )
#define I2C_FLAG_ADDR			( 1 << I2C_SR1_ADDR )
#define I2C_FLAG_TIMEOUT		( 1 << I2C_SR1_TIMEOUT )



/*APIs supported by this driver */

/*Peripheral Clock Setup */

void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);

/*Init and De-Init */

void I2C_Init(I2C_Handle_t *pI2CHandle);
void I2C_DeInit(I2C_RegDef_t *pI2Cx);

/*Data Send and Receive */

void I2C_MasterSendData(I2C_Handle_t *pI2CHandle,uint8_t *pTxBuffer, uint32_t Len, uint8_t  SlaveAddr);


/*IRQ Configurations and ISR Handling */

void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void SI2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);


/*Other Peripherals and Control APIs */

uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName);

void I2C_PeripheralControl (I2C_RegDef_t *pI2Cx, uint8_t EnOrDi);


//Application callback

void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle,uint8_t AppEv);

uint32_t RCC_GetPLLOutputClock(void);







#endif /* INC_STM32F407XX_I2C_DRIVER_H_ */
