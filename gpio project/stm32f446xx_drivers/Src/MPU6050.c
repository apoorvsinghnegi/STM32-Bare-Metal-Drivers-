/*
 * MPU6050.c
 *
 *  Created on: 15-Jun-2021
 *      Author: Apoorv singh negi
 */


/*
 *
 *
 *  Created on: June 10, 2021
 *      Author: Gopal Agarwal
 */


#include<stdio.h>
#include<string.h>
#include "stm32f446xx.h"
#include "Rccconfig.h"
#include"delay.h"



#define MY_ADDR 0x61;			// Nucleo Board Address
#define SLAVE_ADDR  0x68		// AD0 Kept Low

void delay(void)
{
	for(uint32_t i = 0 ; i < 500000/2 ; i ++);
}

I2C_Handle_t I2C1Handle;

//rcv buffer
uint8_t rcv_buf[32];

//some data
uint8_t Power_data[] = {0x6B, 0x08};
uint8_t Acc_data[] = {0x1C, 0x00};
uint8_t Acc_data1[] = {0x3B};


void I2C1_GPIOInits(void)
{
	GPIO_Handle_t I2CPins;

	I2CPins.pGPIOx = GPIOB;
	I2CPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	I2CPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	I2CPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	I2CPins.GPIO_PinConfig.GPIO_PinAltFuncMode = 4;
	I2CPins. GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	//scl
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
	GPIO_Init(&I2CPins);


	//sda
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;
	GPIO_Init(&I2CPins);

}

void I2C1_Inits(void)
{
	I2C1Handle.pI2Cx = I2C1;
	I2C1Handle.I2C_Config.I2C_AckControl = I2C_ACK_ENABLE;
	I2C1Handle.I2C_Config.I2C_DeviceAddress = MY_ADDR;
	I2C1Handle.I2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY_16_9;
	I2C1Handle.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_FM4K;

	I2C_Init(&I2C1Handle);
}


int main(void)
{
	I2C1_GPIOInits();

	I2C1_Inits();

	I2C_PeripheralControl(I2C1,ENABLE);

	//ack bit is made 1 after PE=1
	I2C_ManageAcking(I2C1,I2C_ACK_ENABLE);


		I2C_MasterSendData(&I2C1Handle,Power_data,2,SLAVE_ADDR,0);
		I2C_MasterSendData(&I2C1Handle,Acc_data,2,SLAVE_ADDR,0);

		I2C_MasterSendData(&I2C1Handle,Acc_data1,1,SLAVE_ADDR,0);
}
