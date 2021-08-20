/*
 * stm32f446xx_spi_driver.h
 *
 *  Created on: 08-May-2021
 *      Author: Apoorv singh negi
 */

#ifndef INC_STM32F446XX_SPI_DRIVER_H_
#define INC_STM32F446XX_SPI_DRIVER_H_

#include "stm32f446xx.h"

/* Configuration structure for SPIx peripheral for each peripheral */

typedef struct
{
	uint8_t SPI_Device_Mode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_SclkSpeed;
	uint8_t SPI_DFF;
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;

}SPI_Config_t;

/* Handle structure for SPIx peripheral */

typedef struct
{
	SPI_RegDef_t   *pSPIx;  //holds base address for SPIx(0,1,2) peripheral(small p indicates here it is a pointer type variable)
	SPI_Config_t    SPIConfig;
	uint8_t   *pTxBuffer;    //to store the application tx buffer address
	uint8_t   *pRxBuffer;   //to store the application rx buffer address
	uint32_t  TxLen ;      //to store tx length
	uint32_t  RxLen ;     // to store rx length
	uint8_t   TxState ;  // to store tx state
	uint8_t   RxState;  // to store rx state
}SPI_Handle_t;


/*************************************************************************
 * APIs supported by this driver
 */

/*peripheral clock setup */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);//enable or disable clocks for particular SPI base address

/*Init and De-init */
void SPI_Init(SPI_Handle_t *pSPIHandle);//intialize SPI Peripheral
void SPI_DeInit(SPI_RegDef_t *pSPIx);//de-intialize(reset) all the registers of a given SPI peripheral

/* data send and receive */

//to send data to the external world
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);  //ptxbuffer = pointer to the transmitter buffer, Len = size or length  of data transfer=
//to recieve data from the external world
void SPI_RecieveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len);



//to send data to the external world using interrupts
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len);  //ptxbuffer = pointer to the transmitter buffer, Len = size or length  of data transfer=
//to recieve data from the external world using interrupts
uint8_t SPI_RecieveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len);


/*IRQ config and ISR Handling */
void SPI_IRQInterruptConfig(uint8_t IRQNumber,uint8_t EnorDi );
void SPI_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t  *pHandle);


/* other peripheral control APIs  */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx);
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle);
void SPI_CloseReception(SPI_Handle_t *pSPIHandle);

/* application call back function */

void SPI_ApplicationEventCallBack(SPI_Handle_t *pSPIHandle, uint8_t AppEv);//should be implemented by application


/* @SPI_Device mode  */

#define SPI_DEVICE_MODE_MASTER   1
#define SPI_DEVICE_MODE_SLAVE    0

/* @SPI_BusConfig */

#define SPI_BUS_CONFIG_FD               1    //full duplex
#define SPI_BUS_CONFIG_HD               2   // half duplex
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY   3 // simplex recieve mode only


/* @SPI_SclkSpeed */

#define SPI_SCLK_SPEED_DIV2        0
#define SPI_SCLK_SPEED_DIV4        1
#define SPI_SCLK_SPEED_DIV8        2
#define SPI_SCLK_SPEED_DIV16       3
#define SPI_SCLK_SPEED_DIV32       4
#define SPI_SCLK_SPEED_DIV64       5
#define SPI_SCLK_SPEED_DIV128      6
#define SPI_SCLK_SPEED_DIV256      7


/* @SPI DFF */

#define SPI_DFF_8BITS      0
#define SPI_DFF_16BITS     1



/* @SPI CPOL */

#define SPI_CPOL_HIGH      1
#define SPI_CPOL_LOW       0


/* @SPI CPHA */

#define SPI_CPHA_HIGH      1
#define SPI_CPHA_LOW       0


/* @SPI SSM */

#define SPI_SSM_EN         1
#define SPI_SSM_DI         0


/* SPI related status flag definition */

#define SPI_TXE_FLAG      ( 1 << SPI_SR_TXE )  //this flag gives masking info of txe field in SR register
#define SPI_RXNE_FLAG     ( 1 << SPI_SR_RXNE)  // this flag gives the masking info of rxne field in SR register
#define SPI_BUSY_FLAG     ( 1 << SPI_SR_BSY )  // this flag gives the masking info of busy flag in SR register


/* possible SPI application states */

#define SPI_READY            0
#define SPI_BUSY_IN_RX       1
#define SPI_BUSY_IN_TX       2

/* possible SPI application events */

#define  SPI_EVENT_TX_CMPLT 1 // to define that transmission is complete
#define  SPI_EVENT_RX_CMPLT 2 // to define reception is complete
#define  SPI_EVENT_OVR_ERROR 3 //to define overflow error
#define  SPI_EVENT_CRC_ERROR 4 // to define CRC error








#endif /* INC_STM32F446XX_SPI_DRIVER_H_ */
