/*
 * spi_cmd_handling.c
 *
 *  Created on: 12-May-2021
 *      Author: Apoorv singh negi
 */

//command code
#define COMMAND_LED_CTRL           0x50
#define COMMAND_SENSOR_READ        0x51
#define COMMAND_LED_READ           0x52
#define COMMAND_PRINT              0x53
#define COMMAND_ID_READ            0x54

#define LED_ON     1
#define LED_OFF    0

//arduino analog pins

#define ANALOG_PIN0         0
#define ANALOG_PIN1         1
#define ANALOG_PIN2         2
#define ANALOG_PIN3         3
#define ANALOG_PIN4         4

//arduino LED
#define LED_PIN   9   //we have to connect LED to pin 9 arduino



#include "stm32f446xx.h"
#include "stm32f446xx_gpio.h"
#include "stm32f446xx_spi_driver.h"
#include"string.h"

void delay(void)
{
	for( uint32_t i = 0; i < 50000/2; i++);

}

//Intialising GPIO pins in alternate functionality mode to work as SPI peripheral
void SPI2_GPIOInits(void)
{


	GPIO_Handle_t SPIPins;
	SPIPins.pGPIOx = GPIOB;
    SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;//as we are using SPI which is alternate functionality mode
    SPIPins.GPIO_PinConfig.GPIO_PinAltFuncMode = 5; // as alternate functionality mode is AF5
    SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;//setting output type as push pull configuration
    SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD; //no pull up, pull down configuration is required
    SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;// we can put speed according to our choice

    //pin configuration for SCLK
    SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
    GPIO_Init(&SPIPins);

    //pin configuration for MOSI
    SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
    GPIO_Init(&SPIPins);


    //pin configuration for MISO
    SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
    GPIO_Init(&SPIPins);

    //pin configuration for NSS
    SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
     GPIO_Init(&SPIPins);

}

void SPI2_Inits(void)
{
	SPI_Handle_t SPI2Handle;
	SPI2Handle.pSPIx = SPI2;  //as we are using SPI2
	SPI2Handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD; // To configure the communication as full duplex
	SPI2Handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV8; // To use maximum speed 2Mhz
	SPI2Handle.SPIConfig.SPI_Device_Mode = SPI_DEVICE_MODE_MASTER ; // as we want SPI in master mode only and there are no slaves
	SPI2Handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI2Handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2Handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2Handle.SPIConfig.SPI_SSM = SPI_SSM_DI ; // hardware slave management enabled for NSS

	SPI_Init(&SPI2Handle);

}

void GPIO_ButtonInit(void)
{
	 GPIO_Handle_t GPIOBtn;
	 //GPIO button configurations
	    GPIOBtn.pGPIOx = GPIOC;  //using GPIOC as our user push button in our board is connected to PC13 which is GPIO port C
		GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;//As internal push button is connected to PC13 in our board
		GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN; //to use push button we have to configure that pin as input mode
		GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;//here, it doesn't matter whether we use fast or slow speed
		GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;//schematic itself contains pull down register hence, there is no need to enable pull up pull down register


		GPIO_Init(&GPIOBtn); //initialize all the parameters of GPIO port as defined by us above we use this method to do it

}

uint8_t SPI_VerifyResponse(uint8_t ackbyte)
{
if(ackbyte == 0xF5)
{
  return 1;
}
	return 0;
}



//we don't use miso and nss lines as we are only using master here with no slaves
int main(void){

	uint8_t dummy_write  =  0xff;
	uint8_t dummy_read;

    GPIO_ButtonInit(); // to intialize the button interface of SPI

	SPI2_GPIOInits();// to intialize the GPIO pins to behave as SPI2 pins

	SPI2_Inits();   // to intialize the SPI2 peripheral


     /*making SSOE 1 does NSS Output Enable
      * the NSS pin is automatically managed by hardware
      * i.e. when SPE = 1, NSS will be pulled to low
      * and NSS pin will be high when SPE=0
      */
	SPI_SSOEConfig(SPI2, ENABLE);

	while(1)
	{

	    // wait till button is pressed
	    while( ! GPIO_ReadFromInputPin(GPIOC,GPIO_PIN_NO_13));

        delay(); // to avoid button debouncing related issues

	    SPI_PeripheralControl(SPI2, ENABLE); //enable the SPI2 peripheral

	    //1.CMD_LED_CTRL   <pin no(1)>   <value(1)>
	    uint8_t commndcode = COMMAND_LED_CTRL;
	    SPI_SendData(SPI2, &commndcode, 1);// when it is received by slave it check whether this command is present in slave or not
	                                       // if slave supports this command it would send ACK


	    //due to this transmission of 1byte it resulted in 1 garbage byte collection in rx buffer of master
	    // and RXNE flag was set. so we have to do dummy read and clear the bit
        //this method is defined in datasheet
	    SPI_RecieveData(SPI2, &dummy_read, 1);




	    //send some dummy bits( 1 byte) to fetch the response from the slave
	    //as we are using 8 bits data transfer hence, 1 byte of data is only required but if we use 16 bits data transfer then we require 2 bytes of dummy data

	    uint8_t Ackbyte;
	    uint8_t args[2];

	    SPI_SendData(SPI2, &dummy_write, 1);
	    //when send data api call return it means response from the slave has arrived from the master.So read it nex

        // read the ack byte recieved
	    SPI_RecieveData(SPI2, &Ackbyte, 1);

	    if(SPI_VerifyResponse(Ackbyte))
	    {
	    	//send arguments
	    	args[0] = LED_PIN;
	    	args[1] = LED_ON;
	    	 SPI_SendData(SPI2, args, 2);

	    }

	    //end of command led_ctrl

	    //2.CMD_SENSOR_READ   <analog pin number(1) >
	    //here analog read returns value between 0 to 255 where 0 means 0v and 255 means 5v

	    // wait till button is pressed
	  	    while( ! GPIO_ReadFromInputPin(GPIOC,GPIO_PIN_NO_13));

	          delay(); // to avoid button debouncing related issues

	          commndcode = COMMAND_SENSOR_READ;

	          //send command
	          SPI_SendData(SPI2, &commndcode, 1);

	          //dummy read to clear RXNE
	          SPI_RecieveData(SPI2, &dummy_read, 1);




	          //send some dummy bits( 1 byte) to fetch the response from the slave
	          //as we are using 8 bits data transfer hence, 1 byte of data is only required but if we use 16 bits data transfer then we require 2 bytes of dummy data

             //this send data is used to fetch the response from the slave
	         SPI_SendData(SPI2, &dummy_write, 1);
	         //when send data api call return it means response from the slave has arrived from the master.So read it nex

	         // read the ack byte recieved
	          SPI_RecieveData(SPI2, &Ackbyte, 1);

	          if(SPI_VerifyResponse(Ackbyte))
	         	 {
	         	    	//send arguments
	         	    args[0] = ANALOG_PIN0;

	         	    SPI_SendData(SPI2, args, 1); //sending 1 byte of data


	         	    SPI_RecieveData(SPI2, &dummy_read, 1); //do dummy read to clear off RXNE for above send data

					//here we are asking for data from slave immediately but in real slaves does adc conversion
					//to read the analog value so master should wait for sometime before sending the dummy byte
				    //to fetch the result of analog read

					//insert delay so slave is ready with data
					 delay();


			        //send some dummy bits( 1 byte) to fetch the response from  the slave

				    SPI_SendData(SPI2, &dummy_write, 1);

				    uint8_t analog_read;
				    SPI_RecieveData(SPI2, &analog_read, 1);


                   }




	    //before closing the communication of SPI peripheral with external world let us confirm whether busy flag is set or reset
		//i.e. whether SPI has completed its communication or not. we can't abruptly close the communication

	    while( SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG) ); //if it returns 1 which means SPI is busy else SPI is free
	    SPI_PeripheralControl(SPI2, DISABLE);//after sending data to external world disable the peripheral

	}

	return 0;

}
