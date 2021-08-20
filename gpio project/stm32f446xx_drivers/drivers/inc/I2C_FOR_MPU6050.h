/*
 * I2C_FOR_MPU6050.h
 *
 *  Created on: 28-Jun-2021
 *      Author: Apoorv singh negi
 */

#ifndef INC_I2C_FOR_MPU6050_H_
#define INC_I2C_FOR_MPU6050_H_

#include "stm32f446xx.h"
#include <stdint.h>

void I2C_Config(void);
void I2C_start(void);
void I2C_Write(uint8_t data);
void I2C_Address(uint8_t Address);
void I2C_stop(void);
void I2C_WriteMulti(uint8_t * data, uint8_t size);
void I2C_Read (uint8_t Address, uint8_t *buffer, uint8_t size);

#endif /* INC_I2C_FOR_MPU6050_H_ */
