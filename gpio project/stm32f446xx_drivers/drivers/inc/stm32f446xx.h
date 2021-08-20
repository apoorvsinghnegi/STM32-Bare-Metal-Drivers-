/*
 * stm32f446xx.h
 *
 *  Created on: Apr 7, 2021
 *      Author: Apoorv singh negi
 */

#ifndef INC_STM32F446XX_H_
#define INC_STM32F446XX_H_
#include<stdint.h>
#include<stddef.h>



#define __vo     volatile
#define __weak   __attribute__((weak))
/*********** PROCESSOR SPECIFIC DETAILS   **************/
/* ARM Cortex Mx processor NVIC ISERx specific register addresses */

#define NVIC_ISER0         ( (__vo uint32_t*) 0xE000E100 )
#define NVIC_ISER1         ( (__vo uint32_t*) 0xE000E104 )
#define NVIC_ISER2         ( (__vo uint32_t*) 0xE000E108 )
#define NVIC_ISER3         ( (__vo uint32_t*) 0xE000E10C )


/* ARM Cortex Mx processor NVIC ISECx specific register addresses */

#define NVIC_ICER0         ( (__vo uint32_t*) 0XE000E180 )
#define NVIC_ICER1         ( (__vo uint32_t*) 0XE000E184 )
#define NVIC_ICER2         ( (__vo uint32_t*) 0XE000E188 )
#define NVIC_ICER3         ( (__vo uint32_t*) 0XE000E18C )

/* ARM Cortex Mx processor Priority register address calculation */

#define NVIC_PR_BASE_ADDR  ((__vo uint32_t*)0xE000E400)

#define NO_PR_BITS_IMPLEMENTED    4



/* define base addresses of flash and SRAM memories */

#define FLASH_BASEADDR     0x08000000U
#define SRAM1_BASEADDR     0x20000000U
#define SRAM2_BASEADDR     0x2001C000U
#define SRAM               SRAM1_BASEADDR
#define ROM_BASEADDR       0x1FFF0000U

/* define base addresses of AHBx and APBx bus peripherals */

#define PERIPH_BASE        0x40000000U
#define APB1PERIPH_BASE    PERIPH_BASE
#define APB2PERIPH_BASE    0x40010000U
#define AHB1PERIPH_BASE    0x40020000U
#define AHB2PERIPH_BASE    0x50000000U

/*define base addresses of those peripherals which are hanging on AHB1 bus */
#define GPIOA_BASEADDR     (AHB1PERIPH_BASE + 0x0000)
#define GPIOB_BASEADDR     (AHB1PERIPH_BASE + 0x0400)
#define GPIOC_BASEADDR     (AHB1PERIPH_BASE + 0x0800)
#define GPIOD_BASEADDR     (AHB1PERIPH_BASE + 0x0C00)
#define GPIOE_BASEADDR     (AHB1PERIPH_BASE + 0x1000)
#define GPIOF_BASEADDR     (AHB1PERIPH_BASE + 0x1400)
#define GPIOG_BASEADDR     (AHB1PERIPH_BASE + 0x1800)
#define GPIOH_BASEADDR     (AHB1PERIPH_BASE + 0x1C00)
#define RCC_BASEADDR       (AHB1PERIPH_BASE + 0x3800)

/*define base addresses of those peripherals which are hanging on APB1 bus */
#define I2C1_BASEADDR       (APB1PERIPH_BASE + 0x5400)
#define I2C2_BASEADDR       (APB1PERIPH_BASE + 0x5800)
#define I2C3_BASEADDR       (APB1PERIPH_BASE + 0x5C00)
#define SPI2_BASEADDR       (APB1PERIPH_BASE + 0x3800)
#define SPI3_BASEADDR       (APB1PERIPH_BASE + 0x3C00)
#define USART2_BASEADDR     (APB1PERIPH_BASE + 0x4400)
#define USART3_BASEADDR     (APB1PERIPH_BASE + 0x4800)
#define UART4_BASEADDR      (APB1PERIPH_BASE + 0x4C00)
#define UART5_BASEADDR      (APB1PERIPH_BASE + 0x5000)

/* define base addresses of those peripherals which are hanging on APB2 bus */
#define EXTI_BASEADDR       (APB2PERIPH_BASE + 0x3C00)
#define SYSCFG_BASEADDR     (APB2PERIPH_BASE + 0x3800)
#define SPI1_BASEADDR       (APB2PERIPH_BASE + 0x3000)
#define SPI4_BASEADDR       (APB2PERIPH_BASE + 0x3400)
#define USART1_BASEADDR     (APB2PERIPH_BASE + 0x1000)
#define USART6_BASEADDR     (APB2PERIPH_BASE + 0x1400)

/******peripheral register definition structures *******************/

/* peripheral register definition for GPIO */
typedef struct
{
	__vo uint32_t MODER;         /* GPIO port mode register                  */
	__vo uint32_t OTYPER;       /*  GPIO port output type register          */
	__vo uint32_t OSPEEDER;    /* GPIO port output speed register          */
	__vo uint32_t PUPDR;      /*  GPIO port pull-up/pull-down register    */
	__vo uint32_t IDR;       /* GPIO port input data register            */
	__vo uint32_t ODR;      /*  GPIO port output data register          */
	__vo uint32_t BSRR;    /* GPIO port bit set/reset register         */
	__vo uint32_t LCKR;   /*  GPIO port configuration lock register   */
	__vo uint32_t AFR[2];/* GPIO alternate function  register        */
}GPIO_RegDef_t;


/* peripheral register definition for EXTI */
typedef struct
{
	__vo uint32_t IMR;       /* EXTI Interrupt mask register            */
	__vo uint32_t EMR;      /* EXTI Event mask register                */
	__vo uint32_t RTSR;    /* EXTI Rising trigger selection register  */
	__vo uint32_t FTSR;   /* EXTI Falling trigger selection register */
	__vo uint32_t SWIER; /* EXTI Software interrupt event register  */
	__vo uint32_t PR;   /* EXTI Pending register                   */

}EXTI_RegDef_t;


/*peripheral register definition for SYSCFG  */
typedef struct
{
	__vo uint32_t MEMRMP;                     /* SYSCFG memory remap register                                   */
	__vo uint32_t PMC;                       /*  SYSCFG peripheral mode configuration register                 */
	__vo uint32_t EXTI1CR[4];               /*   SYSCFG external interrupt configuration register from 1 to 4 */
	     uint32_t RESERVED1[2];            /*    memory reserved                                             */
	__vo uint32_t CMPCR;                  /* Compensation cell control register                             */
	     uint32_t RESERVED2[2];          /* memory reserved                                                */
	__vo uint32_t CFGR;                 /* SYSCFG configuration register                                  */

}SYSCFG_RegDef_t;


/* peripheral register definition for RCC */
typedef struct
{
	__vo uint32_t CR;        /*         RCC clock control register                               */
	__vo uint32_t PLLCFGR;  /*    RCC PLL configuration register                                */
	__vo uint32_t CFGR;    /*     RCC clock configuration register                             */
	__vo uint32_t CIR;    /*    RCC clock interrupt register                                  */
	__vo uint32_t AHB1RSTR;      /*    RCC AHB1 peripheral reset register                                */
	__vo uint32_t AHB2RSTR;     /*     RCC AHB2 peripheral reset register                               */
	__vo uint32_t AHB3RSTR;    /*      RCC AHB3 peripheral reset register                              */
	     uint32_t RESERVED0;  /*       RESERVED                                                       */
	__vo uint32_t APB1RSTR;  /*               RCC APB1 peripheral reset register                     */
	__vo uint32_t APB2RSTR; /* RCC APB2 peripheral reset register                                   */
	     uint32_t RESERVED1[2];         /*      RESERVED                                               */
	__vo uint32_t AHB1ENR;             /*     RCC AHB1 peripheral clock enable register               */
	__vo uint32_t AHB2ENR;            /*RCC AHB2 peripheral clock enable register                    */
	__vo uint32_t AHB3ENR;           /*RCC AHB3 peripheral clock enable register                    */
	     uint32_t RESERVED2;        /*  RESERVED                                                   */
	__vo uint32_t APB1ENR;         /*RCC APB1 peripheral clock enable register                    */
	__vo uint32_t APB2ENR;        /*RCC APB2 peripheral clock enable register                    */
	     uint32_t RESERVED3[2];  /*RESERVED                                                     */
	__vo uint32_t AHB1LPENR;    /*RCC AHB1 peripheral clock enable in low power mode register  */
	__vo uint32_t AHB2LPENR;   /*RCC AHB2 peripheral clock enable in low power mode register  */
	__vo uint32_t AHB3LPENR;  /*RCC AHB3 peripheral clock enable in low power mode register  */
	__vo uint32_t RESERVED4; /*                    RESERVED                                 */
	__vo uint32_t APB1LPENR;/*RCC APB1 peripheral clock enable in low power mode register  */
	__vo uint32_t APB2LPENR;/*RCC APB2 peripheral clock enabled in low power mode register*/
	__vo uint32_t RESERVED5[2];       /*                   RESERVED                            */
	__vo uint32_t BDCR;              /*RCC Backup domain control register                     */
	__vo uint32_t CSR;              /* RCC clock control & status register                   */
	__vo uint32_t RESERVED6[2];    /*   RESERVED                                            */
	__vo uint32_t SSCGR;          /*RCC spread spectrum clock generation register          */
	__vo uint32_t PLLI2SCFGR;    /* RCC PLLI2S configuration register                     */
	__vo uint32_t PLLSAICFGR;   /*RCC PLL configuration register                         */
	__vo uint32_t DCKCFGR;     /* RCC dedicated clock configuration register            */
	__vo uint32_t CKGATENR;   /*RCC clocks gated enable register                       */
	__vo uint32_t DCKCFGR2;  /*RCC dedicated clocks configuration register 2          */

}RCC_RegDef_t;


/* peripheral register definition for SPI */
typedef struct
{
	__vo uint32_t CR1;           /* SPI control register          */
	__vo uint32_t CR2;          /*SPI control register 2         */
	__vo uint32_t SR;          /*SPI status register            */
	__vo uint32_t DR;         /*SPI data register              */
	__vo uint32_t CRCPR;     /* SPI CRC polynomial register   */
	__vo uint32_t RXCRCR;   /* SPI RX CRC register           */
	__vo uint32_t TXCRCR;  /* SPI TX CRC register           */
	__vo uint32_t I2SCFGR;/*SPI_I2S configuration register */
	__vo uint32_t I2SPR; /* SPI_I2S prescaler register    */

} SPI_RegDef_t;


/*peripheral register definition for i2c */
typedef struct
{
	__vo uint32_t I2C_CR1;      /*I2C control register 1*/
	__vo uint32_t I2C_CR2;     /*I2C control register 2*/
	__vo uint32_t I2C_OAR1;   /* I2C own address register 1  */
	__vo uint32_t I2C_OAR2;  /* I2C own address register 2 */
	__vo uint32_t I2C_DR;   /* I2C data register*/
	__vo uint32_t I2C_SR1; /*I2C status register 1 */
	__vo uint32_t I2C_SR2; /*I2C status register 2 */
	__vo uint32_t I2C_CCR; /* I2C clock control register */
	__vo uint32_t I2C_TRISE; /*I2C TRISE register */
	__vo uint32_t I2C_FLTR; /* I2C FLTR register */

}I2C_RegDef_t;

/*peripheral register definition for usart */
typedef struct
{
	__vo uint32_t  USART_SR;
	__vo uint32_t  USART_DR;
	__vo uint32_t  USART_BRR;
	__vo uint32_t  USART_CR1;
	__vo uint32_t  USART_CR2;
	__vo uint32_t  USART_CR3;
	__vo uint32_t  USART_GTPR;

}USART_RegDef_t;

/* peripheral definitions (peripheral base address typecasted to xxx_RegDef_  */

#define GPIOA    ((GPIO_RegDef_t*) GPIOA_BASEADDR )
#define GPIOB    ((GPIO_RegDef_t*) GPIOB_BASEADDR )
#define GPIOC    ((GPIO_RegDef_t*) GPIOC_BASEADDR )
#define GPIOD    ((GPIO_RegDef_t*) GPIOD_BASEADDR )
#define GPIOE    ((GPIO_RegDef_t*) GPIOE_BASEADDR )
#define GPIOF    ((GPIO_RegDef_t*) GPIOF_BASEADDR )
#define GPIOG    ((GPIO_RegDef_t*) GPIOG_BASEADDR )
#define GPIOH    ((GPIO_RegDef_t*) GPIOH_BASEADDR )

#define RCC      ((RCC_RegDef_t*)  RCC_BASEADDR   )
#define EXTI     ((EXTI_RegDef_t*)  EXTI_BASEADDR  )
#define SYSCFG   ((SYSCFG_RegDef_t*) SYSCFG_BASEADDR)

#define SPI1     ((SPI_RegDef_t*) SPI1_BASEADDR)
#define SPI2     ((SPI_RegDef_t*) SPI2_BASEADDR)
#define SPI3     ((SPI_RegDef_t*) SPI3_BASEADDR)
#define SPI4     ((SPI_RegDef_t*) SPI4_BASEADDR)

#define I2C1      ((I2C_RegDef_t*)I2C1_BASEADDR)
#define I2C2      ((I2C_RegDef_t*)I2C2_BASEADDR)
#define I2C3      ((I2C_RegDef_t*)I2C3_BASEADDR)


#define USART1  			((USART_RegDef_t*)USART1_BASEADDR)
#define USART2  			((USART_RegDef_t*)USART2_BASEADDR)
#define USART3  			((USART_RegDef_t*)USART3_BASEADDR)
#define UART4  				((USART_RegDef_t*)UART4_BASEADDR)
#define UART5  				((USART_RegDef_t*)UART5_BASEADDR)
#define USART6  			((USART_RegDef_t*)USART6_BASEADDR)

/* clock enable macros for GPIOx peripherals  */
#define GPIOA_PCLK_EN()   (RCC->AHB1ENR |= (1 << 0 ))
#define GPIOB_PCLK_EN()   (RCC->AHB1ENR |= (1 << 1 ))
#define GPIOC_PCLK_EN()   (RCC->AHB1ENR |= (1 << 2 ))
#define GPIOD_PCLK_EN()   (RCC->AHB1ENR |= (1 << 3 ))
#define GPIOE_PCLK_EN()   (RCC->AHB1ENR |= (1 << 4 ))
#define GPIOF_PCLK_EN()   (RCC->AHB1ENR |= (1 << 5 ))
#define GPIOG_PCLK_EN()   (RCC->AHB1ENR |= (1 << 6 ))
#define GPIOH_PCLK_EN()   (RCC->AHB1ENR |= (1 << 7 ))

/* clock enable macros for I2Cx peripherals   */
#define I2C1_PCLK_EN()     (RCC->APB1ENR |= (1 << 21 ))
#define I2C2_PCLK_EN()     (RCC->APB1ENR |= (1 << 22 ))
#define I2C3_PCLK_EN()     (RCC->APB1ENR |= (1 << 23 ))

/* clock enable macros for SPIx peripherals   */
#define SPI1_PCLK_EN()     (RCC->APB2ENR |= (1 << 12 ))
#define SPI2_PCLK_EN()     (RCC->APB1ENR |= (1 << 14 ))
#define SPI3_PCLK_EN()     (RCC->APB1ENR |= (1 << 15 ))
#define SPI4_PCLK_EN()     (RCC->APB2ENR |= (1 << 13 ))


/* clock enable macros for USARTx peripherals  */
#define USART2_PCLK_EN()     (RCC->APB1ENR |= (1 << 17 ))
#define USART3_PCLK_EN()     (RCC->APB1ENR |= (1 << 18 ))
#define USART1_PCLK_EN()     (RCC->APB2ENR |= (1 << 4  ))
#define USART6_PCLK_EN()     (RCC->APB2ENR |= (1 << 5  ))
#define UART4_PCLK_EN()      (RCC->APB1ENR |= (1 << 19 ))
#define UART5_PCLK_EN()      (RCC->APB1ENR |= (1 << 20 ))

/* clock enable macros for SYSCFG peripherals  */
#define SYSCFG_PCLK_EN()     (RCC->APB2ENR |= (1 << 14 ))

/* clock disable macros for GPIOx peripherals  */

#define GPIOA_PCLK_DI()   (RCC->AHB1ENR &= ~(1 << 0 ))
#define GPIOB_PCLK_DI()   (RCC->AHB1ENR &= ~(1 << 1 ))
#define GPIOC_PCLK_DI()   (RCC->AHB1ENR &= ~(1 << 2 ))
#define GPIOD_PCLK_DI()   (RCC->AHB1ENR &= ~(1 << 3 ))
#define GPIOE_PCLK_DI()   (RCC->AHB1ENR &= ~(1 << 4 ))
#define GPIOF_PCLK_DI()   (RCC->AHB1ENR &= ~(1 << 5 ))
#define GPIOG_PCLK_DI()   (RCC->AHB1ENR &= ~(1 << 6 ))
#define GPIOH_PCLK_DI()   (RCC->AHB1ENR &= ~(1 << 7 ))

/* clock disable macros for I2Cx peripherals   */
#define I2C1_PCLK_DI()     (RCC->APB1ENR &= ~(1 << 21 ))
#define I2C2_PCLK_DI()     (RCC->APB1ENR &= ~(1 << 22 ))
#define I2C3_PCLK_DI()     (RCC->APB1ENR &= ~(1 << 23 ))

/* clock disable macros for SPIx peripherals   */
#define SPI1_PCLK_DI()     (RCC->APB2ENR &= ~(1 << 12 ))
#define SPI2_PCLK_DI()     (RCC->APB1ENR &= ~(1 << 14 ))
#define SPI3_PCLK_DI()     (RCC->APB1ENR &= ~(1 << 15 ))
#define SPI4_PCLK_DI()     (RCC->APB2ENR &= ~(1 << 13 ))


/* clock disable macros for USARTx peripherals  */
#define USART2_PCLK_DI()     (RCC->APB1ENR &= ~(1 << 17 ))
#define USART3_PCLK_DI()     (RCC->APB1ENR &= ~(1 << 18 ))
#define USART1_PCLK_DI()     (RCC->APB2ENR &= ~(1 << 4 ))
#define USART6_PCLK_DI()     (RCC->APB2ENR &= ~(1 << 5 ))
#define UART4_PCLK_DI()      (RCC->APB1ENR &= ~(1 << 19 ))
#define UART5_PCLK_DI()      (RCC->APB1ENR &= ~(1 << 20 ))


/* clock enable macros for SYSCFG peripherals  */
#define SYSCFG_PCLK_DI()     (RCC->APB2ENR &= ~(1 << 14 ))


/*Macros to reset GPIOx peripherals
 * To reset means we have to first make its value zero and then make it one as we can't keep that
 * peripheral reset for the whole time*/
#define GPIOA_REG_RESET()        do{ (RCC->AHB1RSTR |= ( 1<<0 )); (RCC->AHB1RSTR &= ~(1<<0)); } while(0)
#define GPIOB_REG_RESET()        do{ (RCC->AHB1RSTR |= ( 1<<1 )); (RCC->AHB1RSTR &= ~(1<<0)); } while(0)
#define GPIOC_REG_RESET()        do{ (RCC->AHB1RSTR |= ( 1<<2 )); (RCC->AHB1RSTR &= ~(1<<0)); } while(0)
#define GPIOD_REG_RESET()        do{ (RCC->AHB1RSTR |= ( 1<<3 )); (RCC->AHB1RSTR &= ~(1<<0)); } while(0)
#define GPIOE_REG_RESET()        do{ (RCC->AHB1RSTR |= ( 1<<4 )); (RCC->AHB1RSTR &= ~(1<<0)); } while(0)
#define GPIOF_REG_RESET()        do{ (RCC->AHB1RSTR |= ( 1<<5 )); (RCC->AHB1RSTR &= ~(1<<0)); } while(0)
#define GPIOG_REG_RESET()        do{ (RCC->AHB1RSTR |= ( 1<<6 )); (RCC->AHB1RSTR &= ~(1<<0)); } while(0)
#define GPIOH_REG_RESET()        do{ (RCC->AHB1RSTR |= ( 1<<7 )); (RCC->AHB1RSTR &= ~(1<<0)); } while(0)


#define SPI1_REG_RESET()         do{ (RCC->APB2RSTR |= ( 1 << 12));  (RCC->APB2RSTR  &= ~( 1 <<12));  }     while(0)
#define SPI2_REG_RESET()         do{ (RCC->APB1RSTR |= ( 1 << 14));  (RCC->APB1RSTR  &= ~( 1<< 14));  } 	 while(0)
#define SPI3_REG_RESET()         do{ (RCC->APB1RSTR |= ( 1 << 15));  (RCC->APB1RSTR  &= ~( 1 << 15)); }     while(0)
#define SPI4_REG_RESET()         do{ (RCC->APB2RSTR |=  ( 1 << 13)); (RCC->APB2RSTR   &=  ~( 1 << 13));}     while(0)


/*return port code for given GPIOx address */
#define GPIO_BASEADDR_TO_CODE(x)      ((x == GPIOA)?0:\
		                               (x == GPIOB)?1:\
				                       (x == GPIOC)?2:\
						               (x == GPIOD)?3:\
								       (x == GPIOE)?4:\
									   (x == GPIOF)?5:\
								       (x == GPIOG)?6:0)


/* interrupt request numbers for MCU */
#define IRQ_NO_EXTI0      6
#define IRQ_NO_EXTI1      7
#define IRQ_NO_EXTI2      8
#define IRQ_NO_EXTI3      9
#define IRQ_NO_EXTI4      10
#define IRQ_NO_EXTI9_5    23
#define IRQ_NO_EXTI15_10  40
#define IRQ_NO_SPI1       35
#define IRQ_NO_SPI2       36
#define IRQ_NO_SPI3       51
#define IRQ_NO_SPI4       84
#define IRQ_NO_I2C1_EV     31
#define IRQ_NO_I2C1_ER     32
#define IRQ_NO_I2C2_EV     33
#define IRQ_NO_I2C2_ER     34
#define IRQ_NO_I2C3_EV     72
#define IRQ_NO_I2C3_ER     73



/* defining the priorities- macros for priority level */
#define NVIC_IRQ_PRIO      0
#define NVIC_IRQ_PRIO1     1
#define NVIC_IRQ_PRIO2     2
#define NVIC_IRQ_PRIO3     3
#define NVIC_IRQ_PRIO4     4
#define NVIC_IRQ_PRIO5     5
#define NVIC_IRQ_PRIO6     6
#define NVIC_IRQ_PRIO7     7
#define NVIC_IRQ_PROI8     8
#define NVIC_IRQ_PRIO9     9
#define NVIC_IRQ_PRIO10   10
#define NVIC_IRQ_PRIO11   11
#define NVIC_IRQ_PRIO12   12
#define NVIC_IRQ_PRIO13   13
#define NVIC_IRQ_PRIO14   14
#define NVIC_IRQ_PRIO15   15




//some generic macros
#define ENABLE          1
#define DISABLE         0
#define SET             ENABLE
#define RESET           DISABLE
#define GPIO_PIN_SET    SET
#define GPIO_PIN_RESET  RESET
#define FLAG_RESET      RESET
#define FLAG_SET        SET

//bit positions peripherals for SPI1 peripheral

#define  SPI_CR1_CPHA      0
#define  SPI_CR1_CPOL      1
#define  SPI_CR1_MSTR      2
#define  SPI_CR1_BR        3
#define  SPI_CR1_SPE       6
#define  SPI_CR1_LSBFIRST  7
#define  SPI_CR1_SSI       8
#define  SPI_CR1_SSM       9
#define  SPI_CR1_RXONLY    10
#define  SPI_CR1_DFF       11
#define  SPI_CR1_CRCNEXT   12
#define  SPI_CR1_CRCEN     13
#define  SPI_CR1_BIDIOE    14
#define  SPI_CR1_BIDIMODE  15

//bit positions peripherals for SPI2 peripheral

#define  SPI_CR2_RXDMAEN   0
#define  SPI_CR2_TXDMAEN   1
#define  SPI_CR2_SSOE      2
#define  SPI_CR2_FRF       4
#define  SPI_CR2_ERRIE     5
#define  SPI_CR2_RXNEIE    6
#define  SPI_CR2_TXEIE     7

//bit positions peripherals for SPI status registers

#define  SPI_SR_RXNE      0
#define  SPI_SR_TXE       1
#define  SPI_SR_CHSIDE    2
#define  SPI_SR_UDR       3
#define  SPI_SR_CRCERR    4
#define  SPI_SR_MODF      5
#define  SPI_SR_OVR       6
#define  SPI_SR_BSY       7
#define  SPI_SR_FRE       8

/****** bit position definition for I2C peripheral *****/

/* bit position for I2C_CR1 register */

#define I2C_CR1_PE            0
#define I2C_CR1_NOSTRETCH     7
#define I2C_CR1_START         8
#define I2C_CR1_STOP          9
#define I2C_CR1_ACK           10
#define I2C_CR1_SWRST         15

/*
 * Bit position definitions I2C_CR2
 */
#define I2C_CR2_FREQ				 	0
#define I2C_CR2_ITERREN				 	8
#define I2C_CR2_ITEVTEN				 	9
#define I2C_CR2_ITBUFEN 			    10

/*
 * Bit position definitions I2C_OAR1
 */
#define I2C_OAR1_ADD0    				 0
#define I2C_OAR1_ADD71 				 	 1
#define I2C_OAR1_ADD98  			 	 8
#define I2C_OAR1_ADDMODE   			 	15

/*
 * Bit position definitions I2C_SR1
 */

#define I2C_SR1_SB 					 	0
#define I2C_SR1_ADDR 				 	1
#define I2C_SR1_BTF 					2
#define I2C_SR1_ADD10 					3
#define I2C_SR1_STOPF 					4
#define I2C_SR1_RXNE 					6
#define I2C_SR1_TXE 					7
#define I2C_SR1_BERR 					8
#define I2C_SR1_ARLO 					9
#define I2C_SR1_AF 					 	10
#define I2C_SR1_OVR 					11
#define I2C_SR1_TIMEOUT 				14

/*
 * Bit position definitions I2C_SR2
 */
#define I2C_SR2_MSL						0
#define I2C_SR2_BUSY 					1
#define I2C_SR2_TRA 					2
#define I2C_SR2_GENCALL 				4
#define I2C_SR2_DUALF 					7

/*
 * Bit position definitions I2C_CCR
 */
#define I2C_CCR_CCR 					 0
#define I2C_CCR_DUTY 					14
#define I2C_CCR_FS  				 	15


/******************************************************************************************
 *Bit position definitions of USART peripheral
 ******************************************************************************************/

/*
 * Bit position definitions USART_CR1
 */
#define USART_CR1_SBK					0
#define USART_CR1_RWU 					1
#define USART_CR1_RE  					2
#define USART_CR1_TE 					3
#define USART_CR1_IDLEIE 				4
#define USART_CR1_RXNEIE  				5
#define USART_CR1_TCIE					6
#define USART_CR1_TXEIE					7
#define USART_CR1_PEIE 					8
#define USART_CR1_PS 					9
#define USART_CR1_PCE 					10
#define USART_CR1_WAKE  				11
#define USART_CR1_M 					12
#define USART_CR1_UE 					13
#define USART_CR1_OVER8  				15



/*
 * Bit position definitions USART_CR2
 */
#define USART_CR2_ADD   				0
#define USART_CR2_LBDL   				5
#define USART_CR2_LBDIE  				6
#define USART_CR2_LBCL   				8
#define USART_CR2_CPHA   				9
#define USART_CR2_CPOL   				10
#define USART_CR2_STOP   				12
#define USART_CR2_LINEN   				14


/*
 * Bit position definitions USART_CR3
 */
#define USART_CR3_EIE   				0
#define USART_CR3_IREN   				1
#define USART_CR3_IRLP  				2
#define USART_CR3_HDSEL   				3
#define USART_CR3_NACK   				4
#define USART_CR3_SCEN   				5
#define USART_CR3_DMAR  				6
#define USART_CR3_DMAT   				7
#define USART_CR3_RTSE   				8
#define USART_CR3_CTSE   				9
#define USART_CR3_CTSIE   				10
#define USART_CR3_ONEBIT   				11

/*
 * Bit position definitions USART_SR
 */

#define USART_SR_PE        				0
#define USART_SR_FE        				1
#define USART_SR_NF        				2
#define USART_SR_ORE       				3
#define USART_SR_IDLE       			4
#define USART_SR_RXNE        			5
#define USART_SR_TC        				6
#define USART_SR_TXE        			7
#define USART_SR_LBD        			8
#define USART_SR_CTS        			9





#include "stm32f446xx_gpio.h"
#include "stm32f446xx_spi_driver.h"
#include "stm32f446xx_i2c_driver.h"
#include "stm32f446xx_usart_driver.h"
#include "stm32f446xx_rcc_driver.h"


#endif /* INC_STM32F446XX_H_ */
