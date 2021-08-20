//it is toggling but after 3-4 times the button is pressed it doesn't toggle properly--doubt

#include "stm32f446xx.h"
#include "stm32f446xx_gpio.h"
#include<stdint.h>
#define LOW         0
#define BTN_PRESSED LOW

void delay(void){
for( uint32_t i = 0; i < 50000/4; i++);
}


int main(void)
{

	GPIO_Handle_t GpioLed,GPIOBtn;
	GpioLed.pGPIOx = GPIOA;  //using GPIOA as our LED in our board is connected to PA5 which is GPIO port A
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;//As internal LED is connected to PA5 in our board
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT; //to use LED we have to configure that pin as output mode
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;//here, it doesn't matter whether we use fast or slow speed
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;//we are using open drain condition
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;// as in open drain we don't have pull up state so we enable or configure the internal pull up register

	GPIO_PeriClockControl(GPIOA,ENABLE);// to enable clock of port A
	GPIO_Init(&GpioLed); //initialize GPIO port A using above configurations


	GPIOBtn.pGPIOx = GPIOC;  //using GPIOC as our user b=push button in our board is connected to PC13 which is GPIO port C
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;//As internal push button is connected to PC13 in our board
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN; //to use push button we have to configure that pin as input mode
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;//here, it doesn't matter whether we use fast or slow speed
	GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;//schematic itself contains pull down register hence, there is no need to enable pull up pull down register

	GPIO_PeriClockControl(GPIOC,ENABLE);  // to enable clock of port c using our API
	GPIO_Init(&GPIOBtn); //initialize all the parameters of GPIO port as defined by us above we use this method to do it

	while(1){
		if((GPIO_ReadFromInputPin(GPIOC,GPIO_PIN_NO_13)) == BTN_PRESSED){

       delay();//this is used to remove button de-bouncing as it may happen that we have pressed button once but due to some external factors or noise this loops execute multiple times hence we wait for sometime so that this process is over
		GPIO_ToggleOutputPin(GPIOA,GPIO_PIN_NO_5);
		}
	}

}

/*
 * 002led_button.c
 *
 *  Created on: 13-Apr-2021
 *      Author: Apoorv singh negi
 */


