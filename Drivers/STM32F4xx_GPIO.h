/*
 * STM32F4xx.h
 *
 *  Created on: 7 Jul 2022
 *      Author: efakb
 */

#ifndef STM32F4XX_GPIO_H_
#define STM32F4XX_GPIO_H_

#include "STM32F4xx_rcc.h"

/**********************************************************************************************************************/
/*
 * General Purpose Input Output (GPIOx) register typedefs.
 */

typedef struct
{

	volatile uint32_t MODER;
	volatile uint32_t OTYPER;
	volatile uint32_t OSPEEDR;
	volatile uint32_t PUPDR;
	volatile uint32_t IDR;
	volatile uint32_t ODR;
	volatile uint32_t BSRR;
	volatile uint32_t LCKR;
	volatile uint64_t AFR;
	uint32_t reserved[245];

}GPIOx_RegDef_t;

typedef struct
{

	GPIOx_RegDef_t PORT[11];

}GPIO_RegDef_t;


/********************************************************************************************************/

//GPIO Pointer Definitions

#define GPIO ((GPIO_RegDef_t*)GPIOA_BASEADDR)

/********************************************************************************************************/

/*
 * Function enumerations
 */

/*
 * @GPIO_PIN_STATES
 * Pin state enumeration
 */

typedef enum {LOW=0, RESET=0, FALSE=0, HIGH=1, SET=1, TRUE=1}Pin_State;

/*
 * @GPIO_PORT_NAMES
 * Port name enumeration
 */
typedef enum {GPIOA=0, PORTA=0, GPIOB=1, PORTB=1, GPIOC=2, PORTC=2, GPIOD=3, PORTD=3, GPIOE=4, PORTE=4,
	GPIOF=5, PORTF=5, GPIOG=6, PORTG=6, GPIOH=7, PORTH=7, GPIOI=8, PORTI=8, GPIOJ=9, PORTJ=9, GPIOK=10,
	PORTK=10}Port_Name;

/*
 * @GPIO_PIN_MODES
 * GPIO pin mode enumeration
 */

typedef enum {MODE_INPUT=0, MODE_OUTPUT=1, MODE_ALTFN=2, MODE_ANALOG=3}Pin_Mode;

/*
 * @GPIO_OTYPE
 * GPIO pin output type enumeration
 */

typedef enum {OTYPE_PUSHPULL=0, OTYPE_OPENDRAIN=1}Pin_Output_Type;

/*
 * @GPIO_SPEED
 * GPIO pin possible Speed Modes
 */

typedef enum {OSPEED_LOW=0, OSPEED_MEDIUM=1, OSPEED_HIGH=2, OSPEED_VERYHIGH=3}Pin_Output_Speed;

/*
 * @GPIO_PULLUPDOWN
 * GPIO pin Pull Up Pull Down enumeration
 */

typedef enum {PUPD_FLOAT=0,PUPD_PULLUP=1,PUPD_PULLDOWN=2}Pin_PUPD;

/********************************************************************************************************/
/*
 * Standard Definitions
 */

#define LOW 	0
#define HIGH 	1
#define RESET 	0
#define SET		1

/********************************************************************************************************/

/*
 *  PIN Definitions
 */

#define PA0 0
#define PA1 1
#define PA2 2
#define PA3 3
#define PA4 4
#define PA5 5
#define PA6 6
#define PA7 7
#define PA8 8
#define PA9 9
#define PA10 10
#define PA11 11
#define PA12 12
#define PA13 13
#define PA14 14
#define PA15 15

#define PB0 0
#define PB1 1
#define PB2 2
#define PB3 3
#define PB4 4
#define PB5 5
#define PB6 6
#define PB7 7
#define PB8 8
#define PB9 9
#define PB10 10
#define PB11 11
#define PB12 12
#define PB13 13
#define PB14 14
#define PB15 15

#define PC0 0
#define PC1 1
#define PC2 2
#define PC3 3
#define PC4 4
#define PC5 5
#define PC6 6
#define PC7 7
#define PC8 8
#define PC9 9
#define PC10 10
#define PC11 11
#define PC12 12
#define PC13 13
#define PC14 14
#define PC15 15

#define PD0 0
#define PD1 1
#define PD2 2
#define PD3 3
#define PD4 4
#define PD5 5
#define PD6 6
#define PD7 7
#define PD8 8
#define PD9 9
#define PD10 10
#define PD11 11
#define PD12 12
#define PD13 13
#define PD14 14
#define PD15 15

#define PE0 0
#define PE1 1
#define PE2 2
#define PE3 3
#define PE4 4
#define PE5 5
#define PE6 6
#define PE7 7
#define PE8 8
#define PE9 9
#define PE10 10
#define PE11 11
#define PE12 12
#define PE13 13
#define PE14 14
#define PE15 15

#define PF0 0
#define PF1 1
#define PF2 2
#define PF3 3
#define PF4 4
#define PF5 5
#define PF6 6
#define PF7 7
#define PF8 8
#define PF9 9
#define PF10 10
#define PF11 11
#define PF12 12
#define PF13 13
#define PF14 14
#define PF15 15

#define PG0 0
#define PG1 1
#define PG2 2
#define PG3 3
#define PG4 4
#define PG5 5
#define PG6 6
#define PG7 7
#define PG8 8
#define PG9 9
#define PG10 10
#define PG11 11
#define PG12 12
#define PG13 13
#define PG14 14
#define PG15 15

#define PH0 0
#define PH1 1
#define PH2 2
#define PH3 3
#define PH4 4
#define PH5 5
#define PH6 6
#define PH7 7
#define PH8 8
#define PH9 9
#define PH10 10
#define PH11 11
#define PH12 12
#define PH13 13
#define PH14 14
#define PH15 15

#define PI0 0
#define PI1 1
#define PI2 2
#define PI3 3
#define PI4 4
#define PI5 5
#define PI6 6
#define PI7 7
#define PI8 8
#define PI9 9
#define PI10 10
#define PI11 11
#define PI12 12
#define PI13 13
#define PI14 14
#define PI15 15

/********************************************************************************************************/

/*
 * @GPIO_ALTFN
 * GPIO pin possible Alternate Functions
 */

#define GPIO_AF0 	0
#define GPIO_AF1 	1
#define GPIO_AF2 	2
#define GPIO_AF3 	3
#define GPIO_AF4 	4
#define GPIO_AF5 	5
#define GPIO_AF6 	6
#define GPIO_AF7 	7
#define GPIO_AF8 	8
#define GPIO_AF9 	9
#define GPIO_AF10 	10
#define GPIO_AF11 	11
#define GPIO_AF12 	12
#define GPIO_AF13 	13
#define GPIO_AF14 	14
#define GPIO_AF15 	15

/********************************************************************************************************/


/*
 *  Function Prototypes
 */

void GPIO_Port_CLK_Enable(Port_Name PORT);
void GPIO_Port_CLK_Disable(Port_Name PORT);
void GPIO_Pin_Initialize(Port_Name PORT, uint8_t PIN_NUMBER, Pin_Mode MODE, Pin_Output_Type OTYPE, Pin_Output_Speed OSPEED, Pin_PUPD PULLUPDOWN);
void GPIO_Pin_Set_Alternate_Function(Port_Name PORT, uint8_t PIN_NUMBER, uint8_t ALTFN);
void GPIO_Port_DeInitialize(Port_Name PORT);
void GPIO_Pin_DeInitialize(Port_Name PORT, uint8_t PIN_NUMBER);
uint8_t GPIO_Pin_Read(Port_Name PORT, uint8_t PIN_NUMBER);
uint16_t GPIO_Port_Read(Port_Name PORT);
void GPIO_Pin_Write(Port_Name PORT, uint8_t PIN_NUMBER, Pin_State State);
void GPIO_Port_Write(Port_Name PORT, uint16_t Value);
void GPIO_Pin_Toggle(Port_Name PORT, uint8_t PIN_NUMBER);

/********************************************************************************************************/


#endif /* STM32F4XX_GPIO_H_ */
