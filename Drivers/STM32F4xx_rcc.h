/*
 * STM32F4xx_rcc.h
 *
 *  Created on: 5 Jul 2022
 *      Author: efakb
 */

#ifndef STM32F4XX_RCC_H_
#define STM32F4XX_RCC_H_

#include "STM32F4xx.h"

/*
 * System Core Clock Definition
 */

//volatile uint32_t System_Clock; //Default system core clock speed at startup in Hz (8MHz)

/**********************************************************************************************************************/
/*/
 * Reset and Clock Control (RCC) register typedefs.
 */


//RCC clock control register (RCC_CR)
typedef struct
{

	volatile uint32_t HSION:1;
	volatile uint32_t HSIRDY:1;
	uint32_t reserved0:1;
	volatile uint32_t HSITRM:5;
	volatile uint32_t HSICAL:8;
	volatile uint32_t HSEON:1;
	volatile uint32_t HSERDY:1;
	volatile uint32_t HSEBYP:1;
	volatile uint32_t CSSON:1;
	uint32_t reserved1:4;
	volatile uint32_t PLLON:1;
	volatile uint32_t PLLRDY:1;
	volatile uint32_t PLLI2SON:1;
	volatile uint32_t PLLI2SRDY:1;
	volatile uint32_t PLLSAION:1;
	volatile uint32_t PLLSAIRDY:1;
	uint32_t reserved2:2;

}RCC_CR_t;

//RCC PLL configuration register (RCC_PLLCFGR)
typedef struct
{

	volatile uint32_t PLLM:6;
	volatile uint32_t PLLN:9;
	uint32_t reserved0:1;
	volatile uint32_t PLLP:2;
	uint32_t reserved1:4;
	volatile uint32_t PLLSRC:1;
	uint32_t reserved2:1;
	volatile uint32_t PLLQ:4;
	uint32_t reserved3:4;

}RCC_PLLCFGR_t;

//RCC clock configuration register (RCC_CFGR)
typedef struct
{

	volatile uint32_t SW:2;
	volatile uint32_t SWS:2;
	volatile uint32_t HPRE:4;
	uint32_t reserved0:2;
	volatile uint32_t PPRE1:3;
	volatile uint32_t PPRE2:3;
	volatile uint32_t RTCPRE:5;
	volatile uint32_t MCO1:2;
	volatile uint32_t I2SSCR:1;
	volatile uint32_t MCO1PRE:3;
	volatile uint32_t MCO2PRE:3;
	volatile uint32_t MCO2:2;

}RCC_CFGR_t;


//RCC clock interrupt register (RCC_CIR)
typedef struct
{

	volatile uint32_t LSIRDYF:1;
	volatile uint32_t LSERDYF:1;
	volatile uint32_t HSIRDYF:1;
	volatile uint32_t HSERDYF:1;
	volatile uint32_t PLLRDYF:1;
	volatile uint32_t PLLI2SRDYF:1;
	volatile uint32_t PLLSAIRDYF:1;
	volatile uint32_t CSSF:1;
	volatile uint32_t LSIRDYIE:1;
	volatile uint32_t LSERDYIE:1;
	volatile uint32_t HSIRDYIE:1;
	volatile uint32_t HSERDYIE:1;
	volatile uint32_t PLLRDYIE:1;
	volatile uint32_t PLLI2SRDYIE:1;
	volatile uint32_t PLLSAIRDYIE:1;
	uint32_t reserved0:1;
	volatile uint32_t LSIRDYC:1;
	volatile uint32_t LSERDYC:1;
	volatile uint32_t HSIRDYC:1;
	volatile uint32_t HSERDYC:1;
	volatile uint32_t PLLRDYC:1;
	volatile uint32_t PLLI2SRDYC:1;
	volatile uint32_t PLLSAIRDYC:1;
	volatile uint32_t CSSC:1;
	uint32_t reserved1:8;

}RCC_CIR_t;

//RCC AHB1 peripheral reset register (RCC_AHB1RSTR)
typedef struct
{

	volatile uint32_t GPIOARST:1;
	volatile uint32_t GPIOBRST:1;
	volatile uint32_t GPIOCRST:1;
	volatile uint32_t GPIODRST:1;
	volatile uint32_t GPIOERST:1;
	volatile uint32_t GPIOFRST:1;
	volatile uint32_t GPIOGRST:1;
	volatile uint32_t GPIOHRST:1;
	volatile uint32_t GPIOIRST:1;
	volatile uint32_t GPIOJRST:1;
	volatile uint32_t GPIOKRST:1;
	uint32_t reserved0:1;
	volatile uint32_t CRCRST:1;
	uint32_t reserved1:8;
	volatile uint32_t DMA1RST:1;
	volatile uint32_t DMA2RST:1;
	volatile uint32_t DMA2DRST:1;
	uint32_t reserved5:1;
	volatile uint32_t ETHMACRST:1;
	uint32_t reserved6:3;
	volatile uint32_t OTGHSRST:1;
	uint32_t reserved7:2;

}RCC_AHB1RSTR_t;

//RCC AHB2 peripheral reset register (RCC_AHB2RSTR)
typedef struct
{

	volatile uint32_t DCMIRST:1;
	uint32_t reserved0:3;
	volatile uint32_t CRYPRST:1;
	volatile uint32_t HASHRST:1;
	volatile uint32_t RNGRST:1;
	volatile uint32_t OTGFSRST:1;
	uint32_t reserved1:24;


}RCC_AHB2RSTR_t;

//RCC AHB3 peripheral reset register (RCC_AHB3RSTR)
typedef struct
{

	volatile uint32_t FMCRST:1;
	uint32_t reserved0:31;

}RCC_AHB3RSTR_t;

//RCC APB1 peripheral reset register (RCC_APB1RSTR)
typedef struct
{

	volatile uint32_t TIM2RST:1;
	volatile uint32_t TIM3RST:1;
	volatile uint32_t TIM4RST:1;
	volatile uint32_t TIM5RST:1;
	volatile uint32_t TIM6RST:1;
	volatile uint32_t TIM7RST:1;
	volatile uint32_t TIM12RST:1;
	volatile uint32_t TIM13RST:1;
	volatile uint32_t TIM14RST:1;
	uint32_t reserved0:2;
	volatile uint32_t WWDGRST:1;
	uint32_t reserved1:2;
	volatile uint32_t SPI2RST:1;
	volatile uint32_t SPI3RST:1;
	uint32_t reserved2:1;
	volatile uint32_t USART2RST:1;
	volatile uint32_t USART3RST:1;
	volatile uint32_t UART4RST:1;
	volatile uint32_t UART5RST:1;
	volatile uint32_t I2C1RST:1;
	volatile uint32_t I2C2RST:1;
	volatile uint32_t I2C3RST:1;
	uint32_t reserved3:1;
	volatile uint32_t CAN1RST:1;
	volatile uint32_t CAN2RST:1;
	uint32_t reserved4:1;
	volatile uint32_t PWRRST:1;
	volatile uint32_t DACRST:1;
	volatile uint32_t UART7RST:1;
	volatile uint32_t UART8RST:1;

}RCC_APB1RSTR_t;

//RCC APB2 peripheral reset register (RCC_APB2RSTR)
typedef struct
{

	volatile uint32_t TIM1RST:1;
	volatile uint32_t TIM8RST:1;
	uint32_t reserved0:2;
	volatile uint32_t USART1RST:1;
	volatile uint32_t USART6RST:1;
	uint32_t reserved1:2;
	volatile uint32_t ADCRST:1;
	uint32_t reserved2:2;
	volatile uint32_t SDIORST:1;
	volatile uint32_t SPI1RST:1;
	volatile uint32_t SPI4RST:1;
	volatile uint32_t SYSCFGRST:1;
	uint32_t reserved3:1;
	volatile uint32_t TIM9RST:1;
	volatile uint32_t TIM10RST:1;
	volatile uint32_t TIM11RST:1;
	uint32_t reserved4:1;
	volatile uint32_t SPI5RST:1;
	volatile uint32_t SPI6RST:1;
	volatile uint32_t SAI1RST:1;
	uint32_t reserved5:3;
	volatile uint32_t LTDCRST:1;
	uint32_t reserved6:5;

}RCC_APB2RSTR_t;

//RCC AHB1 peripheral clock enable register (RCC_AHB1ENR)
typedef struct
{

	volatile uint32_t GPIOAEN:1;
	volatile uint32_t GPIOBEN:1;
	volatile uint32_t GPIOCEN:1;
	volatile uint32_t GPIODEN:1;
	volatile uint32_t GPIOEEN:1;
	volatile uint32_t GPIOFEN:1;
	volatile uint32_t GPIOGEN:1;
	volatile uint32_t GPIOHEN:1;
	volatile uint32_t GPIOIEN:1;
	volatile uint32_t GPIOJEN:1;
	volatile uint32_t GPIOKEN:1;
	uint32_t reserved0:1;
	volatile uint32_t CRCEN:1;
	uint32_t reserved1:5;
	volatile uint32_t BKPSRAMEN:1;
	uint32_t reserved2:1;
	volatile uint32_t CCMDATARAMEN:1;
	volatile uint32_t DMA1EN:1;
	volatile uint32_t DMA2EN:1;
	volatile uint32_t DMA2DEN:1;
	uint32_t reserved3:1;
	volatile uint32_t ETHMACEN:1;
	volatile uint32_t ETHMACTXEN:1;
	volatile uint32_t ETHMACRXEN:1;
	volatile uint32_t ETHMACPTPEN:1;
	volatile uint32_t OTGHSEN:1;
	volatile uint32_t OTGHSULPIEN:1;
	uint32_t reserved4:1;

}RCC_AHB1ENR_t;

//RCC AHB2 peripheral clock enable register (RCC_AHB2ENR)
typedef struct
{

	volatile uint32_t DCMIEN:1;
	uint32_t reserved0:3;
	volatile uint32_t CRYPEN:1;
	volatile uint32_t HASHEN:1;
	volatile uint32_t RNGEN:1;
	volatile uint32_t OTGFSEN:1;
	uint32_t reserved1:24;

}RCC_AHB2ENR_t;

//RCC AHB3 peripheral clock enable register (RCC_AHB3ENR)
typedef struct
{

	volatile uint32_t FMCEN:1;
	uint32_t reserved0:31;

}RCC_AHB3ENR_t;

//RCC APB1 peripheral clock enable register (RCC_APB1ENR)
typedef struct
{

	volatile uint32_t TIM2EN:1;
	volatile uint32_t TIM3EN:1;
	volatile uint32_t TIM4EN:1;
	volatile uint32_t TIM5EN:1;
	volatile uint32_t TIM6EN:1;
	volatile uint32_t TIM7EN:1;
	volatile uint32_t TIM12EN:1;
	volatile uint32_t TIM13EN:1;
	volatile uint32_t TIM14EN:1;
	uint32_t reserved0:2;
	volatile uint32_t WWDGEN:1;
	uint32_t reserved1:2;
	volatile uint32_t SPI2EN:1;
	volatile uint32_t SPI3EN:1;
	uint32_t reserved2:1;
	volatile uint32_t USART2EN:1;
	volatile uint32_t USART3EN:1;
	volatile uint32_t UART4EN:1;
	volatile uint32_t UART5EN:1;
	volatile uint32_t I2C1EN:1;
	volatile uint32_t I2C2EN:1;
	volatile uint32_t I2C3EN:1;
	uint32_t reserved3:1;
	volatile uint32_t CAN1EN:1;
	volatile uint32_t CAN2EN:1;
	uint32_t reserved4:1;
	volatile uint32_t PWREN:1;
	volatile uint32_t DACEN:1;
	volatile uint32_t UART7EN:1;
	volatile uint32_t UART8EN:1;

}RCC_APB1ENR_t;

//RCC APB2 peripheral clock enable register (RCC_APB2ENR)
typedef struct
{

	volatile uint32_t TIM1EN:1;
	volatile uint32_t TIM8EN:1;
	uint32_t reserved0:2;
	volatile uint32_t USART1EN:1;
	volatile uint32_t USART6EN:1;
	uint32_t reserved1:2;
	volatile uint32_t ADC1EN:1;
	volatile uint32_t ADC2EN:1;
	volatile uint32_t ADC3EN:1;
	volatile uint32_t SDIOEN:1;
	volatile uint32_t SPI1EN:1;
	volatile uint32_t SPI4EN:1;
	volatile uint32_t SYSCFGEN:1;
	uint32_t reserved3:1;
	volatile uint32_t TIM9EN:1;
	volatile uint32_t TIM10EN:1;
	volatile uint32_t TIM11EN:1;
	uint32_t reserved4:1;
	volatile uint32_t SPI5EN:1;
	volatile uint32_t SPI6EN:1;
	volatile uint32_t SAI1EN:1;
	uint32_t reserved5:3;
	volatile uint32_t LTDCEN:1;
	uint32_t reserved6:5;

}RCC_APB2ENR_t;

//RCC AHB1 peripheral clock enable in low power mode register (RCC_AHB1LPENR)
typedef struct
{

	volatile uint32_t GPIOALPEN:1;
	volatile uint32_t GPIOBLPEN:1;
	volatile uint32_t GPIOCLPEN:1;
	volatile uint32_t GPIODLPEN:1;
	volatile uint32_t GPIOELPEN:1;
	volatile uint32_t GPIOFLPEN:1;
	volatile uint32_t GPIOGLPEN:1;
	volatile uint32_t GPIOHLPEN:1;
	volatile uint32_t GPIOILPEN:1;
	volatile uint32_t GPIOJLPEN:1;
	volatile uint32_t GPIOKLPEN:1;
	uint32_t reserved0:1;
	volatile uint32_t CRCLPEN:1;
	uint32_t reserved1:2;
	volatile uint32_t FLITFLPEN:1;
	volatile uint32_t SRAM1LPEN:1;
	volatile uint32_t SRAM2LPEN:1;
	volatile uint32_t BKPSRAMLPEN:1;
	volatile uint32_t SRAM3LPEN:1;
	uint32_t reserved2:1;
	volatile uint32_t DMA1LPEN:1;
	volatile uint32_t DMA2LPEN:1;
	volatile uint32_t DMA2DLPEN:1;
	uint32_t reserved3:1;
	volatile uint32_t ETHMACLPEN:1;
	volatile uint32_t ETHTXLPEN:1;
	volatile uint32_t ETHRXLPEN:1;
	volatile uint32_t ETHPTPLPEN:1;
	volatile uint32_t OTGHSLPEN:1;
	volatile uint32_t OTGHSULPILPEN:1;
	uint32_t reserved4:1;

}RCC_AHB1LPENR_t;

//RCC AHB2 peripheral clock enable in low power mode register (RCC_AHB2LPENR)
typedef struct
{

	volatile uint32_t DCMILPEN:1;
	uint32_t reserved0:3;
	volatile uint32_t CRYPLPEN:1;
	volatile uint32_t HASHLPEN:1;
	volatile uint32_t RNGLPEN:1;
	volatile uint32_t OTGFSLPEN:1;
	uint32_t reserved1:24;

}RCC_AHB2LPENR_t;

//RCC AHB3 peripheral clock enable in low power mode register (RCC_AHB3LPENR)
typedef struct
{

	volatile uint32_t FMCLPEN:1;
	uint32_t reserved0:31;

}RCC_AHB3LPENR_t;

//RCC APB1 peripheral clock enable in low power mode register (RCC_APB1LPENR)
typedef struct
{

	volatile uint32_t TIM2LPEN:1;
	volatile uint32_t TIM3LPEN:1;
	volatile uint32_t TIM4LPEN:1;
	volatile uint32_t TIM5LPEN:1;
	volatile uint32_t TIM6LPEN:1;
	volatile uint32_t TIM7LPEN:1;
	volatile uint32_t TIM12LPEN:1;
	volatile uint32_t TIM13LPEN:1;
	volatile uint32_t TIM14LPEN:1;
	uint32_t reserved0:2;
	volatile uint32_t WWDGLPEN:1;
	uint32_t reserved1:2;
	volatile uint32_t SPI2LPEN:1;
	volatile uint32_t SPI3LPEN:1;
	uint32_t reserved2:1;
	volatile uint32_t USART2LPEN:1;
	volatile uint32_t USART3LPEN:1;
	volatile uint32_t UART4LPEN:1;
	volatile uint32_t UART5LPEN:1;
	volatile uint32_t I2C1LPEN:1;
	volatile uint32_t I2C2LPEN:1;
	volatile uint32_t I2C3LPEN:1;
	uint32_t reserved3:1;
	volatile uint32_t CAN1LPEN:1;
	volatile uint32_t CAN2LPEN:1;
	uint32_t reserved4:1;
	volatile uint32_t PWRLPEN:1;
	volatile uint32_t DACLPEN:1;
	volatile uint32_t UART7LPEN:1;
	volatile uint32_t UART8LPEN:1;

}RCC_APB1LPENR_t;

//RCC APB2 peripheral clock enable in low power mode register (RCC_APB2LPENR)
typedef struct
{

	volatile uint32_t TIM1LPEN:1;
	volatile uint32_t TIM8LPEN:1;
	uint32_t reserved0:2;
	volatile uint32_t USART1LPEN:1;
	volatile uint32_t USART6LPEN:1;
	uint32_t reserved1:2;
	volatile uint32_t ADC1LPEN:1;
	volatile uint32_t ADC2LPEN:1;
	volatile uint32_t ADC3LPEN:1;
	volatile uint32_t SDIOLPEN:1;
	volatile uint32_t SPI1LPEN:1;
	volatile uint32_t SPI4LPEN:1;
	volatile uint32_t SYSCFGLPEN:1;
	uint32_t reserved3:1;
	volatile uint32_t TIM9LPEN:1;
	volatile uint32_t TIM10LPEN:1;
	volatile uint32_t TIM11LPEN:1;
	uint32_t reserved4:1;
	volatile uint32_t SPI5LPEN:1;
	volatile uint32_t SPI6LPEN:1;
	volatile uint32_t SAI1LPEN:1;
	uint32_t reserved5:3;
	volatile uint32_t LTDCLPEN:1;
	uint32_t reserved6:5;

}RCC_APB2LPENR_t;

//RCC Backup domain control register (RCC_BDCR)
typedef struct
{

	volatile uint32_t LSEON:1;
	volatile uint32_t LSERDY:1;
	volatile uint32_t LSEBYP:1;
	uint32_t reserved0:5;
	volatile uint32_t RTCSEL:2;
	uint32_t reserved1:5;
	volatile uint32_t RTCEN:1;
	volatile uint32_t BDRST:1;
	uint32_t reserved3:15;

}RCC_BDCR_t;

//RCC clock control & status register (RCC_CSR)
typedef struct
{

	volatile uint32_t LSION:1;
	volatile uint32_t LSIRDY:1;
	uint32_t reserved0:22;
	volatile uint32_t RMVF:1;
	volatile uint32_t BORRSTF:1;
	volatile uint32_t PINRSTF:1;
	volatile uint32_t PORRSTF:1;
	volatile uint32_t SFTRSTF:1;
	volatile uint32_t IWDGRSTF:1;
	volatile uint32_t WWDGRSTF:1;
	volatile uint32_t LPWRRSTF:1;

}RCC_CSR_t;

//RCC spread spectrum clock generation register (RCC_SSCGR)

typedef struct
{

	volatile uint32_t MODPER:13;
	volatile uint32_t INCSTEP:15;
	uint32_t reserved0:2;
	volatile uint32_t SPREADSEL:1;
	volatile uint32_t SSCGEN:1;

}RCC_SSCGR_t;

//RCC clock control & status register (RCC_PLLI2SCFGR)
typedef struct
{

	uint32_t reserved0:6;
	volatile uint32_t PLLI2SN:9;
	uint32_t reserved1:9;
	volatile uint32_t PLLI2SQ:4;
	volatile uint32_t PLLI2SR:3;
	uint32_t reserved2:1;

}RCC_PLLI2SCFGR_t;

//RCC PLL configuration register (RCC_PLLSAICFGR)
typedef struct
{

	uint32_t reserved0:6;
	volatile uint32_t PLLSAIN:9;
	uint32_t reserved1:9;
	volatile uint32_t PLLSAIQ:4;
	volatile uint32_t PLLSAIR:3;
	uint32_t reserved2:1;

}RCC_PLLSAICFGR_t;

//RCC Dedicated Clock Configuration Register (RCC_DCKCFGR)
typedef struct
{

	volatile uint32_t PLLS2DIVQ:5;
	uint32_t reserved0:3;
	volatile uint32_t PLLSAIDIVQ:5;
	uint32_t reserved1:3;
	volatile uint32_t PLLSAIDIVR:2;
	uint32_t reserved2:2;
	volatile uint32_t SAI1ASRC:2;
	volatile uint32_t SAI1BSRC:2;
	volatile uint32_t TIMPRE:1;
	uint32_t reserved3:7;

}RCC_DCKCFGR_t;

//Whole RCC registers.
typedef struct
{

	volatile RCC_CR_t CR;
	volatile RCC_PLLCFGR_t PLLCFGR;
	volatile RCC_CFGR_t CFGR;
	volatile RCC_CIR_t CIR;
	volatile RCC_AHB1RSTR_t AHB1RSTR;
	volatile RCC_AHB2RSTR_t AHB2RSTR;
	volatile RCC_AHB3RSTR_t AHB3RSTR;
	uint32_t reserved0;
	volatile RCC_APB1RSTR_t APB1RSTR;
	volatile RCC_APB2RSTR_t APB2RSTR;
	uint32_t reserved1[2];
	volatile RCC_AHB1ENR_t AHB1ENR;
	volatile RCC_AHB2ENR_t AHB2ENR;
	volatile RCC_AHB3ENR_t AHB3ENR;
	uint32_t reserved2;
	volatile RCC_APB1ENR_t APB1ENR;
	volatile RCC_APB2ENR_t APB2ENR;
	uint32_t reserved3[2];
	volatile RCC_AHB1LPENR_t AHB1LPENR;
	volatile RCC_AHB2LPENR_t AHB2LPENR;
	volatile RCC_AHB3LPENR_t AHB3LPENR;
	uint32_t reserved4;
	volatile RCC_APB1LPENR_t APB1LPENR;
	volatile RCC_APB2LPENR_t APB2LPENR;
	uint32_t reserved5[2];
	volatile RCC_BDCR_t BDCR;
	volatile RCC_CSR_t CSR;
	uint32_t reserved6[2];
	volatile RCC_SSCGR_t SSCGR;
	volatile RCC_PLLI2SCFGR_t PLLI2SCFGR;
	volatile RCC_PLLSAICFGR_t PLLSAICFGR;
	volatile RCC_DCKCFGR_t DCKCFGR;

} RCC_RegDef_t;

/********************************************************************************************************/

//RCC Pointer Definition

#define RCC ((RCC_RegDef_t*)RCC_BASEADDR)

/********************************************************************************************************/
/********************************************************************************************************/

/*
 *  AHB1 Clock Enable Macros
 */

#define GPIOA_CLK_EN() (RCC->AHB1ENR.GPIOAEN |= 1)
#define GPIOB_CLK_EN() (RCC->AHB1ENR.GPIOBEN |= 1)
#define GPIOC_CLK_EN() (RCC->AHB1ENR.GPIOCEN |= 1)
#define GPIOD_CLK_EN() (RCC->AHB1ENR.GPIODEN |= 1)
#define GPIOE_CLK_EN() (RCC->AHB1ENR.GPIOEEN |= 1)
#define GPIOF_CLK_EN() (RCC->AHB1ENR.GPIOFEN |= 1)
#define GPIOG_CLK_EN() (RCC->AHB1ENR.GPIOGEN |= 1)
#define GPIOH_CLK_EN() (RCC->AHB1ENR.GPIOHEN |= 1)
#define GPIOI_CLK_EN() (RCC->AHB1ENR.GPIOIEN |= 1)
#define GPIOJ_CLK_EN() (RCC->AHB1ENR.GPIOJEN |= 1)
#define GPIOK_CLK_EN() (RCC->AHB1ENR.GPIOKEN |= 1)
#define CRC_CLK_EN() (RCC->AHB1ENR.CRCEN |= 1)
#define BKPSRAM_CLK_EN() (RCC->AHB1ENR.BKPSRAMEN |= 1)
#define DMA1_CLK_EN() (RCC->AHB1ENR.DMA1EN |= 1)
#define DMA2_CLK_EN() (RCC->AHB1ENR.DMA2EN |= 1)
#define DMA2D_CLK_EN() (RCC->AHB1ENR.DMA2DEN |= 1)
#define ETHMAC_CLK_EN() (RCC->AHB1ENR.ETHMACEN |= 1)
#define ETHMACTX_CLK_EN() (RCC->AHB1ENR.ETHMACTXEN |= 1)
#define ETHMACRX_CLK_EN() (RCC->AHB1ENR.ETHMACRXEN |= 1)
#define ETHMACPTP_CLK_EN() (RCC->AHB1ENR.ETHMACPTPEN |= 1)
#define OTGHS_CLK_EN() (RCC->AHB1ENR.OTGHSEN |= 1)
#define OTGHS_CLK_EN() (RCC->AHB1ENR.OTGHSEN |= 1)
#define OTGHSULPI_CLK_EN() (RCC->AHB1ENR.OTGHSULPIEN |= 1)

/*
 *  AHB1 Clock Disable Macros
 */

#define GPIOA_CLK_DISABLE() (RCC->AHB1ENR.GPIOAEN &= 0)
#define GPIOB_CLK_DISABLE() (RCC->AHB1ENR.GPIOBEN &= 0)
#define GPIOC_CLK_DISABLE() (RCC->AHB1ENR.GPIOCEN &= 0)
#define GPIOD_CLK_DISABLE() (RCC->AHB1ENR.GPIODEN &= 0)
#define GPIOE_CLK_DISABLE() (RCC->AHB1ENR.GPIOEEN &= 0)
#define GPIOF_CLK_DISABLE() (RCC->AHB1ENR.GPIOFEN &= 0)
#define GPIOG_CLK_DISABLE() (RCC->AHB1ENR.GPIOGEN &= 0)
#define GPIOH_CLK_DISABLE() (RCC->AHB1ENR.GPIOHEN &= 0)
#define GPIOI_CLK_DISABLE() (RCC->AHB1ENR.GPIOIEN &= 0)
#define GPIOJ_CLK_DISABLE() (RCC->AHB1ENR.GPIOJEN &= 0)
#define GPIOK_CLK_DISABLE() (RCC->AHB1ENR.GPIOKEN &= 0)
#define CRC_CLK_DISABLE() (RCC->AHB1ENR.CRCEN &= 0)
#define BKPSRAM_CLK_DISABLE() (RCC->AHB1ENR.BKPSRAMEN &= 0)
#define DMA1_CLK_DISABLE() (RCC->AHB1ENR.DMA1EN &= 0)
#define DMA2_CLK_DISABLE() (RCC->AHB1ENR.DMA2EN &= 0)
#define DMA2D_CLK_DISABLE() (RCC->AHB1ENR.DMA2DEN &= 0)
#define ETHMAC_CLK_DISABLE() (RCC->AHB1ENR.ETHMACEN &= 0)
#define ETHMACTX_CLK_DISABLE() (RCC->AHB1ENR.ETHMACTXEN &= 0)
#define ETHMACRX_CLK_DISABLE() (RCC->AHB1ENR.ETHMACRXEN &= 0)
#define ETHMACPTP_CLK_DISABLE() (RCC->AHB1ENR.ETHMACPTPEN &= 0)
#define OTGHS_CLK_DISABLE() (RCC->AHB1ENR.OTGHSEN &= 0)
#define OTGHS_CLK_DISABLE() (RCC->AHB1ENR.OTGHSEN &= 0)
#define OTGHSULPI_CLK_DISABLE() (RCC->AHB1ENR.OTGHSULPIEN &= 0)

/*
 *  AHB1 Configurations Reset Macros
 */

#define GPIOA_CONFIG_RESET() do{RCC->AHB1RSTR.GPIOARST |= 1; RCC->AHB1RSTR.GPIOARST &= 0; }while(0)
#define GPIOB_CONFIG_RESET() do{RCC->AHB1RSTR.GPIOBRST |= 1; RCC->AHB1RSTR.GPIOBRST &= 0; }while(0)
#define GPIOC_CONFIG_RESET() do{RCC->AHB1RSTR.GPIOCRST |= 1; RCC->AHB1RSTR.GPIOCRST &= 0; }while(0)
#define GPIOD_CONFIG_RESET() do{RCC->AHB1RSTR.GPIODRST |= 1; RCC->AHB1RSTR.GPIODRST &= 0; }while(0)
#define GPIOE_CONFIG_RESET() do{RCC->AHB1RSTR.GPIOERST |= 1; RCC->AHB1RSTR.GPIOERST &= 0; }while(0)
#define GPIOF_CONFIG_RESET() do{RCC->AHB1RSTR.GPIOFRST |= 1; RCC->AHB1RSTR.GPIOFRST &= 0; }while(0)
#define GPIOG_CONFIG_RESET() do{RCC->AHB1RSTR.GPIOGRST |= 1; RCC->AHB1RSTR.GPIOGRST &= 0; }while(0)
#define GPIOH_CONFIG_RESET() do{RCC->AHB1RSTR.GPIOHRST |= 1; RCC->AHB1RSTR.GPIOHRST &= 0; }while(0)
#define GPIOI_CONFIG_RESET() do{RCC->AHB1RSTR.GPIOIRST |= 1; RCC->AHB1RSTR.GPIOIRST &= 0; }while(0)
#define GPIOJ_CONFIG_RESET() do{RCC->AHB1RSTR.GPIOJRST |= 1; RCC->AHB1RSTR.GPIOJRST &= 0; }while(0)
#define GPIOK_CONFIG_RESET() do{RCC->AHB1RSTR.GPIOKRST |= 1; RCC->AHB1RSTR.GPIOKRST &= 0; }while(0)
#define CRC_CONFIG_RESET() do{RCC->AHB1RSTR.CRCRST |= 1; RCC->AHB1RSTR.CRCRST &= 0; }while(0)
#define DMA1_CONFIG_RESET() do{RCC->AHB1RSTR.DMA1RST |= 1; RCC->AHB1RSTR.DMA1RST &= 0; }while(0)
#define DMA2_CONFIG_RESET() do{RCC->AHB1RSTR.DMA2RST |= 1; RCC->AHB1RSTR.DMA2RST &= 0; }while(0)
#define DMA2D_CONFIG_RESET() do{RCC->AHB1RSTR.DMA2DRST |= 1; RCC->AHB1RSTR.DMA2DRST &= 0; }while(0)
#define ETHMAC_CONFIG_RESET() do{RCC->AHB1RSTR.ETHMACRST |= 1; RCC->AHB1RSTR.ETHMACRST &= 0; }while(0)
#define OTGHS_CONFIG_RESET() do{RCC->AHB1RSTR.OTGHSRST |= 1; RCC->AHB1RSTR.OTGHSRST &= 0; }while(0)

/********************************************************************************************************/


#endif /* STM32F4XX_RCC_H_ */
