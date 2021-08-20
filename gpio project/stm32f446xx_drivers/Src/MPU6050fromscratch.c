/*
 * MPU6050fromscratch.c
 *
 *  Created on: 28-Jun-2021
 *      Author: Apoorv singh negi
 */


#include<stdio.h>
#include<string.h>
#include "I2C_FOR_MPU6050.h"


//this contains the address of the device, the register in which we want to write the data and the data we want
// to write
//function to write data to MPU6050
void MPU_Write (uint8_t Address, uint8_t Reg, uint8_t Data)
{
	I2C_start();//start the I2C
	I2C_Address(Address);//send the address of the device
	I2C_Write(Reg);//send the register where we want to write the data
	I2C_Write(Data);//send the data that we want to write
	I2C_stop();//stop the I2C
}

//to read the data from the device
//address of the device, the register from where we want to read the data from, buffer where we want to save the data,
//size of the data to be received


void MPU_Read (uint8_t Address, uint8_t Reg, uint8_t *buffer, uint8_t size)
{
	I2C_start();//start the I2C
	I2C_Address(Address);//Send the address of the device
	I2C_Write(Reg);
	I2C_start();  // repeated start
	I2C_Read(Address+0x01, buffer, size);//to read the address
	I2C_stop();
}



void MPU6050_Init (void)
{
	uint8_t check;
	uint8_t Data;

	// check device ID WHO_AM_I

	MPU_Read (MPU6050_ADDR,WHO_AM_I_REG, &check, 1);

	if (check == 104)  // 0x68 will be returned by the sensor if everything goes well
	{
		// power management register 0X6B we should write all 0's to wake the sensor up
		Data = 0;
		MPU_Write (MPU6050_ADDR, PWR_MGMT_1_REG, Data);

		// Set DATA RATE of 1KHz by writing SMPLRT_DIV register
		Data = 0x07;
		MPU_Write(MPU6050_ADDR, SMPLRT_DIV_REG, Data);

		// Set accelerometer configuration in ACCEL_CONFIG Register
		// XA_ST=0,YA_ST=0,ZA_ST=0, FS_SEL=0 -> ? 2g
		Data = 0x00;
		MPU_Write(MPU6050_ADDR, ACCEL_CONFIG_REG, Data);

		// Set Gyroscopic configuration in GYRO_CONFIG Register
		// XG_ST=0,YG_ST=0,ZG_ST=0, FS_SEL=0 -> ? 250 ?/s
		Data = 0x00;
		MPU_Write(MPU6050_ADDR, GYRO_CONFIG_REG, Data);
	}

}

void MPU6050_Read_Accel (void)
{

	uint8_t Rx_data[6];

	// Read 6 BYTES of data starting from ACCEL_XOUT_H register

	MPU_Read (MPU6050_ADDR, ACCEL_XOUT_H_REG, Rx_data, 6);

	Accel_X_RAW = (int16_t)(Rx_data[0] << 8 | Rx_data [1]);
	Accel_Y_RAW = (int16_t)(Rx_data[2] << 8 | Rx_data [3]);
	Accel_Z_RAW = (int16_t)(Rx_data[4] << 8 | Rx_data [5]);

	/*** convert the RAW values into acceleration in 'g'
	     we have to divide according to the Full scale value set in FS_SEL
	     I have configured FS_SEL = 0. So I am dividing by 16384.0
	     for more details check ACCEL_CONFIG Register              ****/

	Ax = Accel_X_RAW/16384.0;
	Ay = Accel_Y_RAW/16384.0;
	Az = Accel_Z_RAW/16384.0;
}


int main ()
{
	SysClockConfig ();
	TIM6Config ();
	I2C_Config ();

	MPU6050_Init ();
	while (1)
	{
		MPU6050_Read_Accel ();
		Delay_ms (1000);
	}
}



