/*
 * stm32f446xx_gpio.c
 *
 *  Created on: Apr 8, 2021
 *      Author: Apoorv singh negi
 */
#include "stm32f446xx.h"
#include "stm32f446xx_gpio.h"
/*peripheral clock setup */

/***********************************
 *@function -  peripheral clock controller
 *@brief-  enable or disable clock for given GPIO port
 *@parameter 1 - base address of  GPIO peripheral
 *@parameter 2-  enable or disable clock macros for that peripheral
 *@return - none
 */

void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE){
		if(pGPIOx == GPIOA){
			GPIOA_PCLK_EN();
		}
		else if(pGPIOx == GPIOB){
			GPIOB_PCLK_EN();
		}
		else if(pGPIOx == GPIOC){
					GPIOC_PCLK_EN();
				}
		else if(pGPIOx == GPIOD){
					GPIOD_PCLK_EN();
				}
		else if(pGPIOx == GPIOE){
						GPIOE_PCLK_EN();
					}
		else if(pGPIOx == GPIOF){
						GPIOF_PCLK_EN();
					}
		else if(pGPIOx == GPIOG){
						GPIOG_PCLK_EN();
					}
		else if(pGPIOx == GPIOH){
						GPIOH_PCLK_EN();
					}
	}
	else{
		if(pGPIOx == GPIOA){
				GPIOA_PCLK_DI();
			}
			else if(pGPIOx == GPIOB){
				GPIOB_PCLK_DI();
			}
			else if(pGPIOx == GPIOC){
						GPIOC_PCLK_DI();
					}
			else if(pGPIOx == GPIOD){
						GPIOD_PCLK_DI();
					}
			else if(pGPIOx == GPIOE){
							GPIOE_PCLK_DI();
						}
			else if(pGPIOx == GPIOF){
							GPIOF_PCLK_DI();
						}
			else if(pGPIOx == GPIOG){
							GPIOG_PCLK_DI();
						}
			else if(pGPIOx == GPIOH){
							GPIOH_PCLK_DI();
						}

	}


}
/***********************************
 *@function - GPIO initialize of different registers present in GPIO
 *@brief-  enable different registers of GPIO e.g. pinmode, pinspeed,etc.
 *@parameter 1 - can contain register definition of a particular port or pin configuration
 *@return - none
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp = 0;
	//enable peripheral clock For a particular gpio port

	GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);

	// 1. configure the modes of GPIO pin
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)//for non-interrupt mode
	{
		temp = ( pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << ( 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //pin mode will be given by user
		pGPIOHandle->pGPIOx->MODER &= ~( 0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clearing that particular bit of the MODER register
		pGPIOHandle->pGPIOx->MODER |= temp; //setting the particular value of MODER register

	}

	else
	{
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT )
	{  //1. configure FTSR(falling trigger selection register)
       EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);//only falling edge detection
       EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);//as we want falling edge detection so we clear rising edge detection register if it is by chance high


	}

	else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
	{   //1. configure RTSR(rising trigger selection register)
		EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);//only rising edge detection
		EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);//as we want rising edge detection so we clear rising edge detection register if it is by chance high

	}
	else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
	{    //we want detection of both the rising and falling edges, hence enable both the registers
		EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);// falling edge detection
		EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);//rising edge detection

	}

	//2.configure the GPIO port selection in SYSCFG_EXTICR
	 uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;// this gives which EXTI1 register we should use i.e. EXTI-0,1,2,3
	 uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;// this tells out of which EXTI1 register we should configure a particular EXTI line present in that register
	 uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);// this convert given port address(a,b,c,...) to code which is used in EXTI registers
	 SYSCFG_PCLK_EN(); //enabling clock for this SYSCFG register
	 SYSCFG->EXTI1CR[temp1] = portcode << (temp2 * 4); // write the value of that particular port address to that register



	//3. enable the exti interrupt delivery using IMR
	   EXTI->IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);




	}

	temp = 0;

	//2. configure the speed
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed  << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDER &= ~( 0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OSPEEDER |= temp;
	temp = 0;


	//3. configure the pupd settings
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl  << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~( 0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->PUPDR |= temp;
	temp = 0;


	//4. configure the output type
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType  << ( pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OTYPER &= ~( 0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER |= temp;
	temp = 0;


	//5. configure the alternate functionality
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN){
		uint8_t temp1, temp2;
		temp1 =  pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8; //this tells whether we have to use AFR[0] or AFR[1]
		temp2 =  pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8 ; //this tell to which position we have to shift value of the register
	    pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << (4 * temp2)); //clearing
        pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFuncMode << (4 * temp2));


	}


}

/***********************************
 *@function - GPIO de intialize register
 *@brief-  de-intialize(reset) all the registers of a given GPIO peripheral
 *@parameter 1 - base address of the GPIO port
 *@return - none
 */



void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)//
{

	if(pGPIOx == GPIOA)
	{
		GPIOA_REG_RESET();
	}
	else if(pGPIOx == GPIOB)
	{
		GPIOB_REG_RESET();
	}
	else if(pGPIOx == GPIOC)
	{
				GPIOC_REG_RESET();
	}
	else if(pGPIOx == GPIOD)
	{
				GPIOD_REG_RESET();
	}
	else if(pGPIOx == GPIOE)
	{
					GPIOE_REG_RESET();
	}
	else if(pGPIOx == GPIOF)
	{
					GPIOF_REG_RESET();
	}
	else if(pGPIOx == GPIOG)
	{
					GPIOG_REG_RESET();
	}
	else if(pGPIOx == GPIOH)
	{
					GPIOH_REG_RESET();
	}

}

/*data read and write */

/***********************************
 *@function - GPIO Read from input pin register
 *@brief-  read the value from input data register of  particular pin
 *@parameter 1 - base address of the GPIO port
 *@parameter 2- pin number of the GPIO port from where we have to read the value
 *@return - 0 or 1
 */



uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
uint8_t value;
value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001);//here, we are shifting the value present in the input data register to the least significant bit and then we are masking all the bits of the register to retain value present only at the least significant bit
return value;

}

/***********************************
 *@function - GPIO Read from input port
 *@brief-  read the value from input data register of  particular port
 *@parameter 1 - base address of the GPIO port
 *@return - returns content of input data register which is of 16 bits
 */

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;
	value = (uint16_t)pGPIOx->IDR;
	return value;

}

/***********************************
 *@function - GPIO write to output pin
 *@brief-  write value to output data register of particular pin of a  particular port
 *@parameter 1 - base address of the GPIO port
 *@parameter 2- the pin number
 *@parameter 3- the value that we have to write which can be 0 or 1
 *@return - none
 */

void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
	if(Value == GPIO_PIN_SET)
	{
		pGPIOx->ODR |= (1 << PinNumber);
	}
	else
	{
		pGPIOx->ODR &= ~(1 << PinNumber);
	}

}

/***********************************
 *@function - GPIO write to output port
 *@brief-  write value to output data register of a  particular port
 *@parameter 1 - base address of the GPIO port
 *@parameter 2- the value that we have to write
 *@return - none
 */


void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
pGPIOx->ODR = Value;
}


/***********************************
 *@function - GPIO toggle output pin
 *@brief-  on or off value of a particular GPIO pin
 *@parameter 1 - base address of the GPIO port
 *@parameter 2- pin number where we have to toggle the value
 *@return - none
 */




void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{

	pGPIOx->ODR  ^= (1 << PinNumber);

}



/***********************************
 *@function - GPIO IRQ configuration which happens at processor side on a particular IRQ number
 *@brief-  configure the interrupt i.e. enable or disable that interrupt at a particular IRQ number
 *@parameter 1 - IRQ number
 *@parameter 2 - Enable or Disable a particular interrupt of that particular IRQ number
 *@return - none
 */


void GPIO_IRQInterruptConfig(uint8_t IRQNumber,uint8_t EnorDi )
{
	if(EnorDi == ENABLE)
	{
		if(IRQNumber <= 31 )
		{
			//program ISER0 register
			*NVIC_ISER0 |= (1 << IRQNumber );  // as each bit defines a particular IRQ number so we shift it by a single bit

		}
		else if(IRQNumber > 31 && IRQNumber < 64)
		{
		  //program ISER1 register
			*NVIC_ISER1 |= (1 << (IRQNumber % 32) ); //% 32 as iser1 start from 32 IRQ no
		}
		else if( IRQNumber >= 64 && IRQNumber < 96)
		{
		//program ISER2 register
			*NVIC_ISER2 |= (1 << (IRQNumber % 64) ); // %64 as ISER2 start from 64 IRQ no

		}
	}
	else
	{
		if(IRQNumber <= 31 )
				{
					//program ICER0 register
		        	*NVIC_ICER0 |= (1 << IRQNumber );
				}
				else if(IRQNumber > 31 && IRQNumber < 64)
				{
				  //program ICER1 register
					*NVIC_ICER1 |= (1 << (IRQNumber % 32) ); //% 32 as iser1 start from 32 IRQ no
				}
				else if( IRQNumber >= 64 && IRQNumber <96)
				{
				//program ICER2 register
					*NVIC_ICER2 |= (1 << (IRQNumber % 64) ); // %64 as ISER2 start from 64 IRQ no

				}

	}
}

/***********************************
*@function - GPIO IRQ Priority configuration which happens at processor side on a particular IRQ number
*@brief-
*@parameter 1 - IRQ number
*@parameter 2-  IRQ Priority number
*@return - none
*/


void GPIO_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority)
{
  //1. first let's find out the IPR register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;

	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR  + iprx)  |= (IRQPriority << shift_amount);//when we add +1 to address it increase by +4 rather than +1
}


/***********************************
*@function - GPIO IRQ interrupt handling
*@brief-  handles the interrupt present in a particular IRQ no of a particular pin
*@parameter 1 - Pin Number
*@return - none
*/


void GPIO_IRQHandling(uint8_t PinNumber)
{
   //clear the exti pr register corresponding to the pin number
	if(EXTI->PR & ( 1 << PinNumber )) //if it is set then interrupt is pending
	{
		//clear the pending register
		EXTI->PR |= ( 1 << PinNumber);

	}
}





