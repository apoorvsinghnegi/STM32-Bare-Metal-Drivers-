/*
 * stm32f446xx_rcc_driver.c
 *
 *  Created on: 02-Jun-2021
 *      Author: Apoorv singh negi
 */


#include "stm32f446xx_rcc_driver.h"





uint16_t AHB_PreScaler[8] = {2,4,8,16,64,128,256,512};
uint8_t APB1_Prescaler[4] = { 2, 4 , 8, 16};







//to get the pclk value for our system
uint32_t RCC_GetPCLK1Value(void)
{

	uint32_t pclk1,SystemClk;
	uint32_t clksrc,temp,ahbp,apb1p;

	clksrc = ((RCC->CFGR >> 2) & 0x03);  // we are shifting each bit of clock configuration by 2 places to right and bit mask with 0x03 to get bit 0 and bit 1 only
	                                     // and make all other bits to zero.

	if(clksrc == 0)
	{
		SystemClk = 16000000;
	}
	else if(clksrc == 1)
	{

		SystemClk = 8000000;
	}

		else if(clksrc == 2)
		{

			SystemClk = RCC_GetPLLOutputClock();

		}
           //for AHB prescaler
          temp = ((RCC->CFGR >> 4 ) & 0xF);// we right shift the value by 4 to get prescaler value and then we bit mask it

          if(temp < 8)
          {
        	  ahbp = 1;
          }
          else
          {
        	  ahbp = AHB_PreScaler[temp-8];

          }

      // for APB1 prescaler
          temp = ((RCC->CFGR >> 10 ) & 0x7);// we right shift the value by 4 to get prescaler value and then we bit mask it

                 if(temp < 4)
                 {
               	  apb1p = 1;
                 }
                 else
                 {
               	  apb1p = APB1_Prescaler[temp-4];

                 }

                 pclk1 = (SystemClk / ahbp) /apb1p;

                 return pclk1;  // frequency of clock

}


uint32_t RCC_GetPCLK2Value(void)
{
	uint32_t SystemClock=0,tmp,pclk2;
		uint8_t clk_src = ( RCC->CFGR >> 2) & 0X3;

		uint8_t ahbp,apb2p;

		if(clk_src == 0)
		{
			SystemClock = 16000000;
		}else
		{
			SystemClock = 8000000;
		}
		tmp = (RCC->CFGR >> 4 ) & 0xF;

		if(tmp < 0x08)
		{
			ahbp = 1;
		}else
		{
	       ahbp = AHB_PreScaler[tmp-8];
		}

		tmp = (RCC->CFGR >> 13 ) & 0x7;
		if(tmp < 0x04)
		{
			apb2p = 1;
		}else
		{
			apb2p =  APB1_Prescaler[tmp-4];
		}

		pclk2 = (SystemClock / ahbp )/ apb2p;

		return pclk2;


}



uint32_t  RCC_GetPLLOutputClock()
{

	return 0;
}

