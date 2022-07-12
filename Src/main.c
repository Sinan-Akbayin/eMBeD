/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Auto-generated by STM32CubeIDE
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

#include <stdint.h>
#include <stdio.h>


#include "STM32F4xx_GPIO.h"

static void SysClockConfig(void);

uint8_t a;

GPIO_Obj_t red_led,button;

int main(void)
{

	SysClockConfig();

	GPIO_Init_Interfaces(&red_led);

	GPIO_Init_Interfaces(&button);

	red_led.Interfaces.config(&red_led, PORTD, PD12, MODE_OUTPUT, OTYPE_PUSHPULL, OSPEED_MEDIUM, PUPD_FLOAT);

	button.Interfaces.config(&button,PORTA, PA0, MODE_INPUT, OTYPE_PUSHPULL, OSPEED_VERYHIGH, PUPD_FLOAT);

	red_led.Interfaces.init(&red_led);

	button.Interfaces.init(&button);
	
	//GPIO_Pin_Initialize(PORTA, PA0, MODE_INPUT, OTYPE_PUSHPULL, OSPEED_VERYHIGH, PUPD_FLOAT);

	//GPIO_Pin_Initialize(PORTD, PD12, MODE_OUTPUT, OTYPE_PUSHPULL, OSPEED_MEDIUM, PUPD_FLOAT);

    /* Loop forever */
	while(1){

		red_led.Interfaces.write(&red_led, button.Interfaces.read(&button));

		//GPIO_Pin_Write(PORTD, PD12, GPIO_Pin_Read(PORTA,PA0));

	}

}

void SysClockConfig(void)
{

	RCC->CR.HSEON = 1;				//Enabling the high speed external oscillator (HSE)

	while(!RCC->CR.HSERDY);			//Waiting for the HSE to stabilize

	RCC->APB1ENR.PWREN = 1;			//Enabling the power interface clock

	PWR->CR.VOS = 1;				//Set to voltage scale 1 (for high power)

	FLASH->ACR.PRFTEN = 1;			//Prefetch is enabled

	FLASH->ACR.ICEN = 1; 			//Instruction cache is enabled is enabled

	FLASH->ACR.DCEN = 1;			//Data cache is enabled

	FLASH->ACR.LATENCY = 5;			//Five wait states (see Table 10 on page 80 in RM0090)

	RCC->CFGR.HPRE = 0;				//Setting the AHB1 clock divider to 1

	RCC->CFGR.PPRE1 = 0b101;		//Setting the APB1 clock divider to 4

	RCC->CFGR.PPRE2 = 0b100;		//Setting the APB2 clock divider to 2

	RCC->PLLCFGR.PLLM = 8;			//Setting the M value of the PLL

	RCC->PLLCFGR.PLLN = 336;		//Setting the N value of the PLL

	RCC->PLLCFGR.PLLP = 0;			//Setting the P value of the PLL

	RCC->PLLCFGR.PLLQ = 7;			//Setting the Q value of the PLL

	RCC->PLLCFGR.PLLSRC = 1;		//Setting HSE as the PLL source

	RCC->CR.PLLON = 1;				//Enabling the PLL

	while(!RCC->CR.PLLRDY);			//Waiting for the PLL to stabilize

	RCC->CFGR.SW = 2;				//PLL selected as system clock

	while (!(RCC->CFGR.SWS==2)); 	//Waiting for source switch to be made

	//System_Clock = 168000000;		//Setting the system clock to new value

}
