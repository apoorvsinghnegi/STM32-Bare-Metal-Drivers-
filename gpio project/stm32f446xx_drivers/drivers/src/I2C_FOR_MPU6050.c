/*
 * I2C_FOR_MPU6050.c
 *
 *  Created on: 28-Jun-2021
 *      Author: Apoorv singh negi
 */


#include<stdio.h>
#include<string.h>
#include "stm32f446xx.h"
#include "I2C_FOR_MPU6050.h"

void I2C_Config(void){

	//what happens if we don't enable their clocks
	//1. enable clocks  for I2C1 and GPIOB
	 RCC->AHB1ENR |= (1 << 1 ); //for GPIOB
	 RCC->APB1ENR |= (1 << 21); //for I2C1

	//2.Select Alternate Function in MODER Register-->using PB8 and PB9
	 GPIOB->MODER |= (2<<16) | (2<<18);
	 // Bits (17:16)= 1:0 --> Alternate Function for Pin PB8; Bits (19:18)= 1:0 --> Alternate Function for Pin PB9

    //3. Select Open Drain Output
	 GPIOB->OTYPER |= (1<<8) | (1<<9);

	 //4. Select High SPEED for the PINs

	 GPIOB->OSPEEDER |= (3<<16) | (3<<18);// Bits (17:16)= 1:1 --> High Speed for PIN PB8; Bits (19:18)= 1:1 --> High Speed for PIN PB9

	 //5.Select Pull-up for both the Pins as we are here using open drain configuration
	 //The Internal Pull up registers can be controlled by using PUDPR Register

	 GPIOB->PUPDR |= (1<<16) | (1<<18);  // Bits (17:16)= 0:1 --> Pull up for PIN PB8; Bits (19:18)= 0:1 --> pull up for PIN PB9

	 //6.Configure the Alternate Function in AFR Register
	//We have already set the pins in the alternate functions mode, but we haven’t defined what those functions should be. This can be done by modifying the AFR Registers

	 GPIOB->AFR[1] |= (4<<0) | (4<<4);  // Bits (3:2:1:0) = 0:1:0:0 --> AF4 for pin PB8;  Bits (7:6:5:4) = 0:1:0:0 --> AF4 for pin PB9



	 //reset the I2C before intialising
	 I2C1->I2C_CR1 |= (1<<15); //reset the I2C
	 I2C1->I2C_CR1 &= ~(1<<15); //normal I2C operation


	 //setting the I2C clock-->I2C connected to APB1 bus
	 // Program the peripheral input clock in I2C_CR2 Register in order to generate correct timings

     I2C1->I2C_CR2 |= (45 << 0);// PCLK1 FREQUENCY in MHz--> we are setting the clock to 45Mhz i.e.APB1 peripheral clock


     // Configure the clock control registers
     I2C1->I2C_CCR = 225<<0;  // check calculation in the copy

     // Configure the rise time register
     I2C1->I2C_TRISE = 46;  // check calculation in copy
     //in the above steps we have completed the clock configuration for I2C

     // Program the I2C_CR1 register to enable the peripheral
     I2C1->I2C_CR1 |= (1<<0);  // Enable I2C

}
//generating the start condition for I2C communication
void I2C_start(void)
{
	/**** STEPS FOLLOWED  ************
	1. Enable the ACK
	2. Send the START condition
	3. Wait for the SB ( Bit 0 in SR1) to set. This indicates that the start condition is generated
	*/

	I2C1->I2C_CR1 |= (1<<10);  // Enable the ACK
	I2C1->I2C_CR1 |= (1<<8);  // Generate START condition

	while(!(I2C1->I2C_SR1 & (1 << 0))); //Wait for SB bit to set, until SB bit is set it would remain inside the while loop

}


//function to write data to the I2C device
//see master sequence diagram for master transmitter
void I2C_Write(uint8_t data)
{
	/**** STEPS FOLLOWED  ************
	1. Wait for the TXE (bit 7 in SR1) to set. This indicates that the DR is empty
	2. Send the DATA to the DR Register
	3. Wait for the BTF (bit 2 in SR1) to set. This indicates the end of LAST DATA transmission
	*/
 while(!(I2C1->I2C_SR1 & (1 << 7)));//Wait for the TXE (bit 7 in SR1) to set.
 I2C1->I2C_DR = data;//data register is of 8 bits hence we load data there
 while(!(I2C1->I2C_SR1 & (1 << 2)));//Wait for the BTF (bit 2 in SR1) to set.


}

//function to send the slave address
void I2C_Address(uint8_t Address)
{
	/**** STEPS FOLLOWED  ************
	1. Send the Slave Address to the DR Register
	2. Wait for the ADDR (bit 1 in SR1) to set. This indicates the end of address transmission
	3. clear the ADDR by reading the SR1 and SR2
	*/

	I2C1->I2C_DR = Address;  //  send the address
	while (!(I2C1->I2C_SR1 & (1<<1)));  // wait for ADDR bit to set
	uint8_t temp = I2C1->I2C_SR1 | I2C1->I2C_SR2;  // read SR1 and SR2 to clear the ADDR bit

}

//function to stop the I2C transmission process

void I2C_stop(void)
{

	I2C1->I2C_CR1 |= (1 << 9); //TO stop the I2C process;

}

//if we want to write multiple bytes of data rather than a single byte of data

void I2C_WriteMulti(uint8_t * data, uint8_t size)
{

	 while(!(I2C1->I2C_SR1 & (1 << 7)));//Wait for the TXE (bit 7 in SR1) to set.
	 while(size)
	 {
        while(!(I2C1->I2C_SR1 & (1 << 7)));//Wait for the TXE (bit 7 in SR1) to set.
	    I2C1->I2C_DR =(volatile uint32_t) *data++;//data register is of 8 bits hence we load data there
        size--;
	 }


	while(!(I2C1->I2C_SR1 & (1 << 2)));//Wait for the BTF (bit 2 in SR1) to set.

}

//function to read from I2C-->1) address of slave, 2) buffer to store the data into 3) size of the data we want to receive

//see transfer sequence for master receive
void I2C_Read (uint8_t Address, uint8_t *buffer, uint8_t size)
{
/**** STEPS FOLLOWED  ************
1. If only 1 BYTE needs to be Read
	a) Write the slave Address, and wait for the ADDR bit (bit 1 in SR1) to be set
	b) the Acknowledge disable is made during EV6 (before ADDR flag is cleared) and the STOP condition generation is made after EV6
	c) Wait for the RXNE (Receive Buffer not Empty) bit to set
	d) Read the data from the DR
2. If Multiple BYTES needs to be read
  a) Write the slave Address, and wait for the ADDR bit (bit 1 in SR1) to be set
	b) Clear the ADDR bit by reading the SR1 and SR2 Registers
	c) Wait for the RXNE (Receive buffer not empty) bit to set
	d) Read the data from the DR
	e) Generate the Acknowlegment by settint the ACK (bit 10 in SR1)
	f) To generate the nonacknowledge pulse after the last received data byte, the ACK bit must be cleared just after reading the
		 second last data byte (after second last RxNE event)
	g) In order to generate the Stop/Restart condition, software must set the STOP/START bit
	   after reading the second last data byte (after the second last RxNE event)
*/

	int remaining = size;// to keep track of remaining bytes

/**** STEP 1 ****/
	if (size == 1)//if we want to read only 1 byte
	{
		/**** STEP 1-a ****/
		I2C1->I2C_DR = Address;  //  send the address
		while (!(I2C1->I2C_SR1 & (1<<1)));  // wait for ADDR bit to set

		/**** STEP 1-b ****/
		I2C1->I2C_CR1 &= ~(1<<10);  // clear the ACK bit
		uint8_t temp = I2C1->I2C_SR1 | I2C1->I2C_SR2;  // read SR1 and SR2 to clear the ADDR bit.... EV6 condition-->clear the address flag
		I2C1->I2C_CR1 |= (1<<9);  // Stop I2C-->send the stop condition

		/**** STEP 1-c ****/
		while (!(I2C1->I2C_SR1 & (1<<6)));  // wait for RxNE to set

		/**** STEP 1-d ****/
		buffer[size-remaining] = I2C1->I2C_DR;  // Read the data from the DATA REGISTER

	}

/**** STEP 2 ****/
	else
	{
		/**** STEP 2-a ****/
		I2C1->I2C_DR = Address;  //  send the address
		while (!(I2C1->I2C_SR1 & (1<<1)));  // wait for ADDR bit to set

		/**** STEP 2-b ****/
		uint8_t temp = I2C1->I2C_SR1 | I2C1->I2C_SR2;  // read SR1 and SR2 to clear the ADDR bit

		while (remaining>2)
		{
			/**** STEP 2-c ****/
			while (!(I2C1->I2C_SR1 & (1<<6)));  // wait for RxNE to set

			/**** STEP 2-d ****/
			buffer[size-remaining] = I2C1->I2C_DR;  // copy the data into the buffer

			/**** STEP 2-e ****/
			I2C1->I2C_CR1 |= 1<<10;  // Set the ACK bit to Acknowledge the data received

			remaining--;
		}

		// Read the SECOND LAST BYTE
		while (!(I2C1->I2C_SR1 & (1<<6)));  // wait for RxNE to set
		buffer[size-remaining] = I2C1->I2C_DR;

		/**** STEP 2-f ****/
		I2C1->I2C_CR1 &= ~(1<<10);  // clear the ACK bit

		/**** STEP 2-g ****/
		I2C1->I2C_CR1 |= (1<<9);  // Stop I2C

		remaining--;

		// Read the Last BYTE
		while (!(I2C1->I2C_SR1 & (1<<6)));  // wait for RxNE to set
		buffer[size-remaining] = I2C1->I2C_DR;  // copy the data into the buffer
	}

}

