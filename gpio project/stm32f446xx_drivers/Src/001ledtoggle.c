/*
 * 001ledtoggle.c
 *
 *  Created on: 13-Apr-2021
 *      Author: Apoorv singh negi
 */
//led would toggle but at a very low intensity due to large internal pull up resistor(40k) connected to it.
#include "stm32f446xx.h"
#include "stm32f446xx_gpio.h"
#include<stdint.h>

void delay(void){
for( int i = 0; i < 50000; i++);
}


int main(void)
{

	GPIO_Handle_t GpioLed;
	GpioLed.pGPIOx = GPIOA;  //using GPIOA as our LED in our board is connected to PA5 which is GPIO port A
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;//As internal LED is connected to PA5 in our board
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT; //to use LED we have to configure that pin as output mode
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;//here, it doesn't matter whether we use fast or slow speed
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;//we are using open drain condition
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;// as in open drain we don't have pull up state so we enable or congfigure the internal pull up register

	GPIO_PeriClockControl(GPIOA,ENABLE);
	GPIO_Init(&GpioLed);

	while(1){

		GPIO_ToggleOutputPin(GPIOA,GPIO_PIN_NO_5);
		delay();
	}

}

