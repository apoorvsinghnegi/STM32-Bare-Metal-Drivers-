//it is toggling but after 3-4 times the button is pressed it doesn't toggle properly--doubt

#include "stm32f446xx.h"
#include "stm32f446xx_gpio.h"
#include"string.h"
#include <stdint.h>


void delay(void)
{
for( int t = 0; t < 50000; t++);
}




int main(void)
{

	GPIO_Handle_t GpioLed,GPIOBtn;
	memset(&GpioLed,0,sizeof(GpioLed));//to set memory of this whole element as 0
	memset(&GPIOBtn,0,sizeof(GPIOBtn));//to set the memory of this whole element as 0
	GpioLed.pGPIOx = GPIOA;  //using GPIOA as our LED in our board is connected to PA5 which is GPIO port A
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;//As internal LED is connected to PA5 in our board
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT; //to use LED we have to configure that pin as output mode
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;//here, it doesn't matter whether we use fast or slow speed
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;//we are using open drain condition
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;// as in open drain we don't have pull up state so we enable or configure the internal pull up register

	GPIO_PeriClockControl(GPIOA,ENABLE);// to enable clock of port A
	GPIO_Init(&GpioLed); //initialize GPIO port A using above configurations


	/*using GPIO to trgger an intterupt when button is pressed */

	GPIOBtn.pGPIOx = GPIOC;  //using GPIOC as our user push button in our board is connected to PC13 which is GPIO port C
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;//As internal push button is connected to PC13 in our board
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT; //to use push button we have to configure that pin as input mode
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;//here, it doesn't matter whether we use fast or slow speed
	GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;//schematic itself contains pull down register hence, there is no need to enable pull up pull down register

	GPIO_PeriClockControl(GPIOC,ENABLE);  // to enable clock of port c using our API
	GPIO_Init(&GPIOBtn); //initialize all the parameters of GPIO port as defined by us above we use this method to do it

	//priority is not used here as only single interrupt is used here

	 GPIO_IRQPriorityConfig(IRQ_NO_EXTI15_10,NVIC_IRQ_PRIO15);
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI15_10, ENABLE ); //button connected to pin13 hence irq no13 is used
	while(1);

void EXTI15_10_IRQHandler(void)
{
	//call driver supplied GPIO interrupt handling API
    delay();
	GPIO_IRQHandling(GPIO_PIN_NO_13);
	GPIO_ToggleOutputPin(GPIOA,GPIO_PIN_NO_5);
}

}

/*
 * 002led_button.c
 *
 *  Created on: 13-Apr-2021
 *      Author: Apoorv singh negi
 */


