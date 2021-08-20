/*
 * I2C.h
 *
 *  Created on: 30-Jun-2021
 *      Author: Apoorv singh negi
 */

#ifndef INC_I2C_H_
#define INC_I2C_H_

#include <stdint.h>

void I2C_Config (void);

void I2C_Start (void);

void I2C_Write (uint8_t data);

void I2C_Address (uint8_t Address);

void I2C_Stop (void);

void I2C_WriteMulti (uint8_t *data, uint8_t size);

void I2C_Read (uint8_t Address, uint8_t *buffer, uint8_t size);

#endif /* INC_I2C_H_ */
