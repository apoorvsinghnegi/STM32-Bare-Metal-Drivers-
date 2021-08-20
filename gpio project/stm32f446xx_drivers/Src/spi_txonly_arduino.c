/*
 * spi_txonly_arduino.c // here we are sending data to arduino which act as slave and stm32 which act as master
 *
 *  Created on: 11-May-2021
 *      Author: Apoorv singh negi
 */



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


//we don't use miso and nss lines as we are only using master here with no slaves
int main(void){
	char user_data[] = "hello world";

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

	    //first send the length information of data to arduino peripheral so it knows how much data to fetch
         // to send more data change uint8_t data byte by some bigger datatype like uint16_t,etc
	    uint8_t dataLen = strlen(user_data);
	    SPI_SendData(SPI2, &dataLen, 1 );


	    SPI_SendData(SPI2, (uint8_t*)user_data, strlen(user_data));     //2nd parameter is pointer to tx buffer

	    //before closing the communication of SPI peripheral with external world let us confirm whether busy flag is set or reset
		//i.e. whether SPI has completed its communication or not. we can't abruptly close the communication

	    while( SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG) ); //if it returns 1 which means SPI is busy else SPI is free
	    SPI_PeripheralControl(SPI2, DISABLE);//after sending data to external world disable the peripheral

	}

	return 0;

}
