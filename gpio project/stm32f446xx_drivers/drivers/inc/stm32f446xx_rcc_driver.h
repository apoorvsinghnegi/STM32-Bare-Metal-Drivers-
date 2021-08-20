/*
 * stm32f446xx_rcc_driver.h
 *
 *  Created on: 02-Jun-2021
 *      Author: Apoorv singh negi
 */

#ifndef INC_STM32F446XX_RCC_DRIVER_H_
#define INC_STM32F446XX_RCC_DRIVER_H_


#include "stm32f446xx.h"


//returns APB1 clock value
uint32_t RCC_GetPCLK1Value(void);


//returns APB2 clock value
uint32_t RCC_GetPCLK2Value(void);


uint32_t  RCC_GetPLLOutputClock(void);


#endif /* INC_STM32F446XX_RCC_DRIVER_H_ */
