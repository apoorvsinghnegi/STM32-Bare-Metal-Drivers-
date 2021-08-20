/*
 * stm32f446xx_gpio.h
 *
 *  Created on: Apr 8, 2021
 *      Author: Apoorv singh negi
 */

#ifndef INC_STM32F446XX_GPIO_H_
#define INC_STM32F446XX_GPIO_H_

#include "stm32f446xx.h"

/* this handles configuration settings for a GPIO pin */
typedef struct{
	uint8_t GPIO_PinNumber; //possible values from @GPIO_PIN_NUMBERS
	uint8_t GPIO_PinMode;  //possible values from  @GPIO_PIN_MODES
	uint8_t GPIO_PinSpeed; //possible values from  @GPIO_PIN_SPEED
	uint8_t GPIO_PinPuPdControl;//possible values from @GPIO_PU_PD_CONFIG
	uint8_t GPIO_PinOPType;//possible values from @GPIO_OUTPUT_TYPE
	uint8_t GPIO_PinAltFuncMode;//alternate function mode is only valid when pinmode is set to alternate function mode
}GPIO_PinConfig_t;



/* this is to handle structure for a GPIO pin  */
typedef struct{
	GPIO_RegDef_t *pGPIOx;
    GPIO_PinConfig_t GPIO_PinConfig;

}GPIO_Handle_t;

/*  @GPIO_PIN_MODES
 GPIO pin possible modes
 **/
#define GPIO_MODE_IN     0
#define GPIO_MODE_OUT    1
#define GPIO_MODE_ALTFN  2
#define GPIO_MODE_ANALOG 3
#define GPIO_MODE_IT_FT  4  //falling edge interrupt
#define GPIO_MODE_IT_RT  5  //rising edge interrupt
#define GPIO_MODE_IT_RFT 6  //rising-falling edge interrupt


/*@GPIO_OUTPUT_TYPE
 GPIO pin possible output types */

#define GPIO_OP_TYPE_PP  0         //GPIO output type push pull
#define GPIO_OP_TYPE_OD  1       //GPIO output type open drain

/* @GPIO_PIN_SPEED
 GPIO pin possible output speeds */

#define GPIO_SPEED_LOW     0
#define GPIO_SPEED_MEDIUM  1
#define GPIO_SPEED_FAST    2
#define GPIO_SPEED_HIGH    3

/*@GPIO_PU_PD_CONFIG
 GPIO pin pull up and pull down configuration macros*/

#define GPIO_NO_PUPD  0   //no pull up and pull down
#define GPIO_PIN_PU   1  //pull up config
#define GPIO_PIN_PD   2  // pull down config

/*@GPIO_PIN_NUMBERS
 * GPIO pin number
 */
#define GPIO_PIN_NO_0   0
#define GPIO_PIN_NO_1   1
#define GPIO_PIN_NO_2   2
#define GPIO_PIN_NO_3   3
#define GPIO_PIN_NO_4   4
#define GPIO_PIN_NO_5   5
#define GPIO_PIN_NO_6   6
#define GPIO_PIN_NO_7   7
#define GPIO_PIN_NO_8   8
#define GPIO_PIN_NO_9   9
#define GPIO_PIN_NO_10  10
#define GPIO_PIN_NO_11  11
#define GPIO_PIN_NO_12  12
#define GPIO_PIN_NO_13  13
#define GPIO_PIN_NO_14  14
#define GPIO_PIN_NO_15  15




/*APIs supported by this driver */

/*peripheral clock setup */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);//enable or disable clocks for particular gpio base address

/*Init and De-init */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);//intialize GPIO port
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);//de-intialize(reset) all the registers of a given GPIO peripheral

/*data read and write */

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);//input value or pin state can either be 0 or 1
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx) ;//returns content of input data register which is of 16 bits
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);//we can write 0 or 1 value
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/*IRQ config and ISR Handling */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber,uint8_t EnorDi );
void GPIO_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);







#endif /* INC_STM32F446XX_GPIO_H_ */
