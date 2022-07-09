/*
 * STM32F4xx.h
 *
 *  Created on: Jun 30, 2022
 *      Author: efakb
 */

#ifndef STM32F4XX_H_
#define STM32F4XX_H_

#include <stdint.h>

/******************************************BASE ADRESSES OF REGISTERS***********************************************/
//Memory map of the STM32F4 uC. This is from memory map section of the reference manual.

//BASE ADDRESS
#define APB1PERIPH_BASEADDR				0x40000000U

//TIM2-7,12-14 REGISTER
#define TIM2_BASEADDR					0x40000000U
#define TIM3_BASEADDR					0x40000400U
#define TIM4_BASEADDR					0x40000800U
#define TIM5_BASEADDR					0x40000C00U
#define TIM6_BASEADDR					0x40001000U
#define TIM7_BASEADDR					0x40001400U
#define TIM12_BASEADDR					0x40001800U
#define TIM13_BASEADDR					0x40001C00U
#define TIM14_BASEADDR					0x40002000U

//RTC BACKUP REGISTER
#define RTCBKP_BASEADDR					0x40002800U

//WATCHDOG REGISTER
#define WWDG_BASEADDR					0x40002C00U
#define IWDG_BASEADDR					0x40003000U

//SPI and I2S REGISTER
#define I2S2ext_BASEADDR				0x40003400U
#define SPI2_BASEADDR					0x40003800U
#define SPI3_BASEADDR					0x40003C00U
#define I2S3ext_BASEADDR				0x40004000U

//USART2,3 and UART4,5 REGISTER
#define USART2_BASEADDR					0x40004400U
#define USART3_BASEADDR					0x40004800U
#define UART4_BASEADDR					0x40004C00U
#define UART5_BASEADDR					0x40005000U

//I2C1-3 REGISTER
#define I2C1_BASEADDR					0x40005400U
#define I2C2_BASEADDR					0x40005800U
#define I2C3_BASEADDR					0x40005C00U

//CAN1-2 REGISTER
#define CAN1_BASEADDR					0x40006400U
#define CAN2_BASEADDR					0x40006800U

//PWR REGISTER
#define PWR_BASEADDR					0x40007000U

//DAC REGISTER
#define DAC_BASEADDR					0x40007400U

//UART7,8 REGISTER
#define UART7_BASEADDR					0x40007800U
#define UART8_BASEADDR					0x40007C00U


/*
 *  Base addresses of peripherals that is connected to APB2 Bus
 */

//BASE ADDRESS
#define APB2PERIPH_BASEADDR				0x40010000U

//TIM1,8 REGISTER
#define TIM1_BASEADDR					0x40010000U
#define TIM8_BASEADDR					0x40010400U

//USART1,6 REGISTER
#define USART1_BASEADDR					0x40011000U
#define USART6_BASEADDR					0x40011400U

//ADC1,2,3 REGISTER
#define ADC1_BASEADDR					0x40012000U
#define ADC2_BASEADDR					0x40012100U
#define ADC3_BASEADDR					0x40012200U

//SDIO REGISTER
#define SDIO_BASEADDR					0x40012C00U

//SPI1,4 REGISTER
#define SPI1_BASEADDR					0x40013000U
#define SPI4_BASEADDR					0x40013400U

//SYSCONFIG REGISTER
#define SYSCONFIG_BASEADDR				0x40013800U

//EXTI REGISTER
#define EXTI_BASEADDR					0x40013C00U

//TIM9,10,11 REGISTER
#define TIM9_BASEADDR					0x40014000U
#define TIM10_BASEADDR					0x40014400U
#define TIM11_BASEADDR					0x40014800U

//SPI5,6 REGISTER
#define SPI5_BASEADDR					0x40015000U
#define SPI6_BASEADDR					0x40015400U

//SAI REGISTER
#define SAI_BASEADDR					0x40015800U

//LCD-TFT REGISTER
#define LCDTFT_BASEADDR					0x40016800U

/*
 *  Base addresses of peripherals that is connected to AHB1 Bus
 */

//BASE ADDRESS
#define AHB1PERIPH_BASEADDR				0x40020000U

//GPIOA-K REGISTER
#define GPIOA_BASEADDR					(AHB1PERIPH_BASEADDR + 0x0000U)
#define GPIOB_BASEADDR					(GPIOA_BASEADDR + 0x0400U)
#define GPIOC_BASEADDR					(GPIOB_BASEADDR + 0x0800U)
#define GPIOD_BASEADDR					(GPIOC_BASEADDR + 0x0C00U)
#define GPIOE_BASEADDR					(GPIOD_BASEADDR + 0x1000U)
#define GPIOF_BASEADDR					(GPIOE_BASEADDR + 0x1400U)
#define GPIOG_BASEADDR					(GPIOF_BASEADDR + 0x1800U)
#define GPIOH_BASEADDR					(GPIOG_BASEADDR + 0x1C00U)
#define GPIOI_BASEADDR					(GPIOH_BASEADDR + 0x2000U)
#define GPIOJ_BASEADDR					(GPIOI_BASEADDR + 0x2400U)
#define GPIOK_BASEADDR					(GPIOJ_BASEADDR + 0x2800U)

//CRC REGISTER
#define CRC_BASEADDR					0x40023000U

//RCC REGISTER
#define RCC_BASEADDR 					0x40023800U

//FLASH INTERFACE REGISTER
#define FLASH_BASEADDR					0x40023C00U

//BKPSRAM REGISTER
#define BKPSRAM_BASEADDR				0x40024000U

//DMA1,2 REGISTER
#define DMA1_BASEADDR					0x40026000U
#define DMA2_BASEADDR					0x40026400U

//ETHERNET MAC REGISTER
#define ETHERNETMAC_BASEADDR			0x40028000U

//DMA2D REGISTER
#define DMA2D_BASEADDR					0x4002B000U

//USB OTG HS REGISTER
#define USBOTGHS_BASEADDR				0x40040000U

/*
 *  Base addresses of peripherals that is connected to AHB2 Bus
 */

//BASE ADDRESS
#define AHB2PERIPH_BASEADDR				0x50000000U

//USB OTG FS REGISTER
#define USBOTGFS_BASEADDR				0x50000000U

//DCMI REGISTER
#define DCMI_BASEADDR					0x50050000U

//CRYP REGISTER
#define CRYP_BASEADDR					0x50060000U

//HASH REGISTER
#define HASH_BASEADDR					0x50060400U

//RNG REGISTER
#define RNG_BASEADDR					0x50060800U

/*
 *  Base addresses of peripherals that is connected to AHB3 Bus
 */

//BASE ADDRESS
#define AHB3PERIPH_BASEADDR				0xA0000000U


/*
 * Power control registers (STM32F405xx/07xx and STM32F415xx/17xx)
 */

//TODO: Implement power control registers for (STM32F42xxx and STM32F43xxx)

//PWR power control register (PWR_CR) for STM32F405xx/07xx and STM32F415xx/17xx
typedef struct
{

	volatile uint32_t LPDS:1;
	volatile uint32_t PDDS:1;
	volatile uint32_t CWUF:1;
	volatile uint32_t CSBF:1;
	volatile uint32_t PVDE:1;
	volatile uint32_t PLS:3;
	volatile uint32_t DBP:1;
	volatile uint32_t FPDS:1;
	uint32_t reserved0:4;
	volatile uint32_t VOS:1;
	uint32_t reserved1:17;

}PWR_CR_t;


//PWR power control/status register (PWR_CSR) for STM32F405xx/07xx and STM32F415xx/17xx
typedef struct
{

	volatile uint32_t WUF:1;
	volatile uint32_t SBF:1;
	volatile uint32_t PVDO:1;
	volatile uint32_t BRR:1;
	uint32_t reserved0:4;
	volatile uint32_t EWUP:1;
	volatile uint32_t BRE:1;
	uint32_t reserved1:4;
	volatile uint32_t VOSRDY:1;
	uint32_t reserved2:17;

}PWR_CSR_t;

//Whole RCC registers.
typedef struct
{

	volatile PWR_CR_t CR;
	volatile PWR_CSR_t CSR;


} PWR_RegDef_t;


/********************************************************************************************************/

/*
 * Flash control registers (STM32F405xx/07xx and STM32F415xx/17xx)
 */

//TODO: Implement flash control registers for (STM32F42xxx and STM32F43xxx)

//Flash access control register (FLASH_ACR) for STM32F405xx/07xx and STM32F415xx/17xx
typedef struct
{

	volatile uint32_t LATENCY:3;
	uint32_t reserved0:5;
	volatile uint32_t PRFTEN:1;
	volatile uint32_t ICEN:1;
	volatile uint32_t DCEN:1;
	volatile uint32_t ICRST:1;
	volatile uint32_t DCRST:1;
	uint32_t reserved1:19;

}FLASH_ACR_t;


//Flash status register (FLASH_SR) for STM32F405xx/07xx and STM32F415xx/17xx
typedef struct
{

	volatile uint32_t EOP:1;
	volatile uint32_t OPERR:1;
	uint32_t reserved0:2;
	volatile uint32_t WRPERR:1;
	volatile uint32_t PGAERR:1;
	volatile uint32_t PGPERR:1;
	volatile uint32_t PGSERR:1;
	uint32_t reserved1:8;
	volatile uint32_t BUSY:1;
	uint32_t reserved2:15;

}FLASH_SR_t;

//Flash control register (FLASH_CR) for STM32F405xx/07xx and STM32F415xx/17xx
typedef struct
{

	volatile uint32_t PG:1;
	volatile uint32_t SER:1;
	volatile uint32_t MER:1;
	uint32_t reserved0:1;
	volatile uint32_t PSIZE:2;
	uint32_t reserved1:6;
	volatile uint32_t STRT:1;
	uint32_t reserved2:7;
	volatile uint32_t EOPIE:1;
	volatile uint32_t ERRIE:1;
	uint32_t reserved3:5;
	volatile uint32_t LOCK:1;

}FLASH_CR_t;

//Flash option control register (FLASH_OPTCR) for STM32F405xx/07xx and STM32F415xx/17xx
//TODO: Implement flash control registers for (STM32F42xxx and STM32F43xxx)
typedef struct
{

	volatile uint32_t OPTLOCK:1;
	volatile uint32_t OPTRST:1;
	volatile uint32_t BOR_LEV:2;
	uint32_t reserved0:1;
	volatile uint32_t WDG_SW:1;
	volatile uint32_t nRST_STOP:1;
	volatile uint32_t nRST_STDBY:1;
	volatile uint32_t RDP:8;
	volatile uint32_t nWRP:12;
	uint32_t reserved1:4;

}FLASH_OPTCR_t;


//Whole FLASH registers.
typedef struct
{

	volatile FLASH_ACR_t ACR;
	volatile uint32_t KEYR;
	volatile uint32_t OPT_KEYR;
	volatile FLASH_SR_t SR;
	volatile FLASH_CR_t CR;
	volatile FLASH_OPTCR_t OPTCR;

} FLASH_RegDef_t;


/********************************************************************************************************/

/*
 * Peripheral Pointer Definitions
 */


#define PWR ((PWR_RegDef_t*)PWR_BASEADDR)
#define FLASH ((FLASH_RegDef_t*)FLASH_BASEADDR)


#endif /* STM32F4XX_H_ */
