/*
 * stm32f446xx_spi_driver.c
 *
 *  Created on: 08-May-2021
 *      Author: Apoorv singh negi
 */

#include "stm32f446xx.h"
#include "stm32f446xx_spi_driver.h"


static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle); //static is used to indicate that they are private helper functions, they can't be used outside
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle);


/***********************************
 *@function -  peripheral clock controller
 *@brief-  enable or disable clock for given SPI peripheral
 *@parameter 1 - base address of  SPI peripheral
 *@parameter 2-  enable or disable clock macros for that peripheral
 *@return - none
 */

void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE){
		if(pSPIx == SPI1){
			SPI1_PCLK_EN();
		}
		else if(pSPIx == SPI2){
			SPI2_PCLK_EN();
		}
		else if(pSPIx == SPI3){
			SPI3_PCLK_EN();
				}
		else if (pSPIx == SPI4)
				{
				SPI4_PCLK_EN();
					}


	}
	else{
		if(pSPIx == SPI1){
			SPI1_PCLK_DI();
			}
			else if(pSPIx == SPI2){
				SPI2_PCLK_DI();
			}
			else if(pSPIx == SPI3){
				SPI3_PCLK_DI();
					}
			else if(pSPIx == SPI4){
						SPI4_PCLK_DI();
							}

          }
}

/***********************************
 *@function - SPI initialization of different registers present in SPI
 *@brief-  enable different registers of SPI e.g. bus_configuration, clkspeed,etc.
 *@parameter 1 - can contain register definition of a particular SPI or pin configuration of that SPI
 *@return - none
 */

void SPI_Init(SPI_Handle_t *pSPIHandle){

  //configure SPI1 register
 uint32_t temp = 0;

 //enable clock for SPI peripheral
 SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);


 //1.configure the device mode

 temp |= pSPIHandle->SPIConfig.SPI_Device_Mode << SPI_CR1_MSTR  ;

 //2. configure the bus configuration

 if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD)
 {
	 //BIDI mode should be cleared
	 temp &= ~(SPI_CR1_BIDIMODE);
 }
 else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD)
 {
	 //BIDI mode should be set
	 temp |= (SPI_CR1_BIDIMODE);
 }
 else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY )
 {
	 //BIDI mode should be cleared
	 temp &= ~(SPI_CR1_BIDIMODE);

	 // RXONLY mode should be set
     temp |= ( SPI_CR1_RXONLY);

//3. configure the SPI serial clock speed (baud rate)
      temp |=  pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR;

//4. configure the  DFF
      temp |= pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF;

//5. configure the CPOL

      temp |= pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL;

//6. configure the CPOH

      temp |= pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA;

      pSPIHandle->pSPIx->CR1 =  temp;
 }
}

 /***********************************
  *@function - SPI de intialize register
  *@brief-  de-intialize(reset) all the registers of a given SPI peripheral
  *@parameter 1 - base address of the GPIO port
  *@return - none
  */

void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
	if(pSPIx == SPI1)
		{
		SPI1_REG_RESET();
		}
		else if(pSPIx == SPI2)
		{
			SPI2_REG_RESET();
		}
		else if(pSPIx == SPI3)
		{
					SPI3_REG_RESET();
		}
		else if(pSPIx == SPI4)
		{
					SPI4_REG_RESET();
		}
}


uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName)
{
   if(pSPIx->SR & FlagName)
   {
	   return FLAG_SET;
   }
	return FLAG_RESET;


}
/***********************************
 *@function -  SPI send data
 *@brief-  To send data to external world from microcontroller using SPI protocol
 *@parameter 1 - base address of  SPI peripheral
 *@parameter 2-   pointer to the Transmit buffer
 *@parameter 3-  Length of data to send
 *@return - none
 *@note - this is a blocking call
 */


void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len)
{
	while(Len > 0)
	{


		//1. wait until TXE is set
	    while(SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET);
	    //2. check the DFF bit in CR1 register
	    if( (pSPIx->CR1 & ( 1 << SPI_CR1_DFF )))
	    {
	    	//it is 16bit DFF register
	    	//1. load the data in the DR register
	    	pSPIx->DR = *((uint16_t*)pTxBuffer);  //we are typecasting to 16 bit data as here DR is a 16 bit data but in function definition we have defined it as 8 bit hence there is a need of typecasting
	    	Len--;
	    	Len--; // 2 bytes of data added to tx buffer hence we decrease the len by 2
	    	(uint16_t*)pTxBuffer++; //here we typecasted to 16 bits as we are sending to bytes of data so address must also be incremented by 2
	    }
	    else
	    {
	    	//it is 8bit DFF register
	    	pSPIx->DR = *(pTxBuffer);  //we are not  typecasting as it is already a 8 bit data
	         Len--;
	         pTxBuffer++;
         }
      }
}
/***********************************
 *@function -  SPI peripheral control
 *@brief-  To enable to disable spi peripheral
 *@parameter 1 - base address of  SPI peripheral
 *@parameter 2-   variable indicating whether to enable or disable SPI peripheral
 *@return - none
 *@note -
 */

void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
   if(EnorDi == ENABLE)
   {
	 pSPIx->CR1 |= (1 << SPI_CR1_SPE );
   }
   else
   {
	   pSPIx->CR1 &= ~(1 << SPI_CR1_SPE );
   }

}

/***********************************
 *@function -  SPI  SSI peripheral control
 *@brief-  To enable to disable SPI SSI peripheral
 *@parameter 1 - base address of  SPI peripheral
 *@parameter 2-   variable indicating whether to enable or disable SPI peripheral
 *@return - none
 *@note -
 */
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
   if(EnorDi == ENABLE)
   {
	 pSPIx->CR1 |= (1 << SPI_CR1_SSI );
   }
   else
   {
	   pSPIx->CR1 &= ~(1 << SPI_CR1_SSI );
   }

}


/***********************************
 *@function -  SPI  SSOE peripheral control
 *@brief-  To enable to disable SPI SSI peripheral
 *@parameter 1 - base address of  SPI peripheral
 *@parameter 2-   variable indicating whether to enable or disable SPI peripheral
 *@return - none
 *@note -
 */
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
   if(EnorDi == ENABLE)
   {
	 pSPIx->CR2 |= (1 << SPI_CR2_SSOE );
   }
   else
   {
	   pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE );
   }

}

/***********************************
 *@function -  SPI receive data
 *@brief-  To receive data from the external world and send to  microcontroller using SPI protocol
 *@parameter 1 - base address of  SPI peripheral
 *@parameter 2-   pointer to the Transmit buffer
 *@parameter 3-  Length of data to send
 *@return - none
 *@note - this is a blocking call
 */


void SPI_RecieveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len)
{
	while(Len > 0)
	{


		//1. wait until RXNE is set
	    while(SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == FLAG_RESET);
	    //2. check the DFF bit in CR1 register
	    if( (pSPIx->CR1 & ( 1 << SPI_CR1_DFF )))
	    {
	    	//it is 16bit DFF register
	    	//1. load the data from  DR register to Rx buffer address
	    	*((uint16_t*)pRxBuffer) =  pSPIx->DR ;  //we are typecasting to 16 bit data as here DR is a 16 bit data but in function definition we have defined it as 8 bit hence there is a need of typecasting
	    	Len--;
	    	Len--; // 2 bytes of data added to tx buffer hence we decrease the len by 2
	    	(uint16_t*)pRxBuffer++; //here we typecasted to 16 bits as we are sending to bytes of data so address must also be incremented by 2
	    }
	    else
	    {
	    	//it is 8bit DFF register
	    	  *(pRxBuffer) = pSPIx->DR;  //we are not  typecasting as it is already a 8 bit data
	         Len--;
	         pRxBuffer++;
         }
      }
}

/***********************************
 *@function - SPI IRQ configuration which happens at processor side on a particular IRQ number
 *@brief-  configure the interrupt i.e. enable or disable that interrupt at a particular IRQ number
 *@parameter 1 - IRQ number
 *@parameter 2 - Enable or Disable a particular interrupt of that particular IRQ number
 *@return - none
 */



void SPI_IRQInterruptConfig(uint8_t IRQNumber,uint8_t EnorDi )
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
*@function - SPI IRQ Priority configuration which happens at processor side on a particular IRQ number
*@brief-
*@parameter 1 - IRQ number
*@parameter 2-  IRQ Priority number
*@return - none
*/




void SPI_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority)
{
  //1. first let's find out the IPR register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;

	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR  + iprx)  |= (IRQPriority << shift_amount);//when we add +1 to address it increase by +4 rather than +1
}



uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len)
{
	uint8_t state =  pSPIHandle->TxState;
	if(state != SPI_BUSY_IN_TX)
	{



       //1. save the tx buffer address and len info in the variables created in spi handle structure
	   pSPIHandle->pTxBuffer = pTxBuffer;
	   pSPIHandle->TxLen = Len;

	    //2. mark the state as busy
	    pSPIHandle->TxState = SPI_BUSY_IN_TX;

	    //3. enable the TXIE control bit to get interrupt when TXE flag is set in SR

	    pSPIHandle->pSPIx->CR2 |= ( 1 <<  SPI_CR2_TXEIE);

	    //4. data handled by ISR CODE

	}

	return state;


}



uint8_t SPI_RecieveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len){


	uint8_t state =  pSPIHandle->RxState;
	if(state != SPI_BUSY_IN_RX)
	{



       //1. save the tx buffer address and len info in the variables created in spi handle structure
	   pSPIHandle->pRxBuffer = pRxBuffer;
	   pSPIHandle->RxLen = Len;

	    //2. mark the state as busy
	    pSPIHandle->RxState = SPI_BUSY_IN_RX;

	    //3. enable the TXIE control bit to get interrupt when TXE flag is set in SR

	    pSPIHandle->pSPIx->CR2 |= ( 1 <<  SPI_CR2_RXNEIE);

	    //4. data handled by ISR CODE

	}

	return state;


}


/***********************************
*@function -SPI IRQ interrupt handling
*@brief-  handles the interrupt present in a particular IRQ no of a particular pin
*@parameter 1 - SPI Handle type of variable
*@return - none
*/

void SPI_IRQHandling(SPI_Handle_t  *pHandle)
{
	uint8_t temp1, temp2;
	// first lets check for TXE
	 temp1 = pHandle->pSPIx->SR & ( 1 << SPI_SR_TXE);
	 temp2 = pHandle->pSPIx->CR2 & (1 <<  SPI_CR2_TXEIE);

	 if( temp1 && temp2)
	 {
	     //handle the interrupt txe

		 spi_txe_interrupt_handle(pHandle);
      }

		// lets check for RXNE
		 temp1 = pHandle->pSPIx->SR & ( 1 << SPI_SR_RXNE);
		 temp2 = pHandle->pSPIx->CR2 & (1 <<  SPI_CR2_RXNEIE);

		 if( temp1 && temp2)
			 {
			     //handle the interrupt txe

				 spi_rxne_interrupt_handle(pHandle);
		      }

		 //check for ovr flag

		 temp1 = pHandle->pSPIx->SR & ( 1 << SPI_SR_OVR);
	     temp2 = pHandle->pSPIx->CR2 & (1 <<  SPI_CR2_ERRIE);


		 if( temp1 && temp2)
			 {
			     //handle the interrupt txe

				 spi_ovr_err_interrupt_handle(pHandle);
		      }

}

//some helper function implementations

static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle)
{

	 if( (pSPIHandle->pSPIx->CR1 & ( 1 << SPI_CR1_DFF )))
		    {
		    	//it is 16bit DFF register
		    	//1. load the data in the DR register
		 pSPIHandle->pSPIx->DR = *((uint16_t*)pSPIHandle->pTxBuffer);  //we are typecasting to 16 bit data as here DR is a 16 bit data but in function definition we have defined it as 8 bit hence there is a need of typecasting
		 pSPIHandle->TxLen--;
		 pSPIHandle->TxLen--; // 2 bytes of data added to tx buffer hence we decrease the len by 2
		    	(uint16_t*)pSPIHandle->pTxBuffer++; //here we typecasted to 16 bits as we are sending to bytes of data so address must also be incremented by 2
		    }
		    else
		    {
		    	//it is 8bit DFF register
		    	 pSPIHandle->pSPIx->DR = *(pSPIHandle->pTxBuffer);  //we are not  typecasting as it is already a 8 bit data
		    	 pSPIHandle->TxLen--;
		    	 pSPIHandle->pTxBuffer++;
	         }

	       if( ! pSPIHandle->TxLen )
	       {
	    	//close the SPI transmission and inform application that Tx is over
	    	   //first we didn't want more interrupts so we close the flag
	    	   //this prevents setting of TXE flag
	    	   SPI_CloseTransmission(pSPIHandle);
	    	   SPI_ApplicationEventCallBack(pSPIHandle, SPI_EVENT_TX_CMPLT);//to call back the application

	       }

}
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle)
{

    //2. check the DFF bit in CR1 register
    if( (pSPIHandle->pSPIx->CR1 & ( 1 << SPI_CR1_DFF )))
    {
    	//it is 16bit DFF register
    	//1. load the data from  DR register to Rx buffer address
    	*((uint16_t*)pSPIHandle->pRxBuffer) = pSPIHandle->pSPIx->DR ;  //we are typecasting to 16 bit data as here DR is a 16 bit data but in function definition we have defined it as 8 bit hence there is a need of typecasting
    	pSPIHandle->RxLen-=2; // 2 bytes of data added to tx buffer hence we decrease the len by 2
    	(uint16_t*)pSPIHandle->pRxBuffer++; //here we typecasted to 16 bits as we are sending to bytes of data so address must also be incremented by 2
    }
    else
    {
    	//it is 8bit DFF register
    	  *(pSPIHandle->pRxBuffer) = pSPIHandle->pSPIx->DR;  //we are not  typecasting as it is already a 8 bit data
    	  pSPIHandle->RxLen--;
    	  pSPIHandle->pRxBuffer++;
     }

    if( ! pSPIHandle->RxLen )
  	       {
  	    	//close the SPI transmission and inform application that Rx is over
  	    	   //first we didn't want more interrupts so we close the flag
  	    	   //this prevents setting of RXNIE flag
    	       SPI_CloseReception(pSPIHandle);
  	    	   SPI_ApplicationEventCallBack(pSPIHandle, SPI_EVENT_RX_CMPLT);//to call back the application

  	       }


}



static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	uint8_t temp;
	//1.CLEAR the ovr flag
	if(pSPIHandle->TxState != SPI_BUSY_IN_TX)
	{
		temp = pSPIHandle->pSPIx->DR;
		temp = pSPIHandle->pSPIx->SR;

	}
	(void)temp;


	//2.inform the application
	 SPI_ApplicationEventCallBack(pSPIHandle, SPI_EVENT_OVR_ERROR);//to call back the application
	 //IF SPI PERIPHERAL IS BUSY IS TRANSMISSION AND OVERLAP HAPPENS THEN, application call back will be send
	 // and it has to clear ovr flag on its own and close the communication rather than executing
	 //above block of code

}


void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx)
{


	        uint8_t temp;
			temp = pSPIx->DR;
			temp = pSPIx->SR;
			(void)temp;



}
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle)
{

	//close the SPI transmission and inform application that Tx is over
		    	   //first we didn't want more interrupts so we close the flag
		    	   //this prevents setting of TXE flag
		    	   pSPIHandle->pSPIx->CR2 &= ~( 1 <<  SPI_CR2_TXEIE);
		    	   pSPIHandle->pTxBuffer = NULL;
		    	   pSPIHandle->TxLen = 0;
		    	   pSPIHandle->TxState = SPI_READY; //communication is over so it is ready.

}
void SPI_CloseReception(SPI_Handle_t *pSPIHandle)
{

	//close the SPI transmission and inform application that Rx is over
	  	    	   //first we didn't want more interrupts so we close the flag
	  	    	   //this prevents setting of RXNEIE flag
	  	    	   pSPIHandle->pSPIx->CR2 &= ~( 1 <<  SPI_CR2_RXNEIE);
	  	    	   pSPIHandle->pRxBuffer = NULL;
	  	    	   pSPIHandle->RxLen = 0;
	  	    	   pSPIHandle->RxState = SPI_READY; //communication is over so it is ready.


}


//the task is complete so what to do next
__weak void SPI_ApplicationEventCallBack(SPI_Handle_t *pSPIHandle, uint8_t AppEv)
{
     //weak implementation.application may override this function

}

