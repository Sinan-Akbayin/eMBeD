/*
 * STM32F4xx_GPIO.c
 *
 *  Created on: 7 Jul 2022
 *      Author: efakb
 */


#include "STM32F4xx_GPIO.h"

static void GPIO_Config(void *sturct_base_address,Port_Name PORT, uint8_t PIN_NUMBER, Pin_Mode MODE, Pin_Output_Type OTYPE, Pin_Output_Speed OSPEED, Pin_PUPD PULLUPDOWN);

static void GPIO_Init(void *sturct_base_address);

static Pin_State GPIO_Read(void *sturct_base_address);

static void GPIO_Write(void *sturct_base_address, Pin_State State);

void GPIO_Init_Interfaces(GPIO_Obj_t *GPIO_OBJ){

	GPIO_OBJ->Interfaces.config = GPIO_Config;

	GPIO_OBJ->Interfaces.init 	= GPIO_Init;

	GPIO_OBJ->Interfaces.read 	= GPIO_Read;

	GPIO_OBJ->Interfaces.write  = GPIO_Write;

}
void GPIO_Config(void *sturct_base_address, Port_Name PORT, uint8_t PIN_NUMBER, Pin_Mode MODE, Pin_Output_Type OTYPE, Pin_Output_Speed OSPEED, Pin_PUPD PULLUPDOWN){

	GPIO_Obj_t *GPIO_OBJ = sturct_base_address;

	GPIO_OBJ->PORT = PORT;

	GPIO_OBJ->PIN_NUMBER = PIN_NUMBER;

	GPIO_OBJ->MODE = MODE;

	GPIO_OBJ->OTYPE = OTYPE;

	GPIO_OBJ->OSPEED = OSPEED;

	GPIO_OBJ->PULLUPDOWN = PULLUPDOWN;

}
void GPIO_Init(void *sturct_base_address){

	GPIO_Obj_t *GPIO_OBJ = sturct_base_address;

	GPIO_Pin_Initialize(GPIO_OBJ->PORT, GPIO_OBJ->PIN_NUMBER, GPIO_OBJ->MODE, GPIO_OBJ->OTYPE, GPIO_OBJ->OSPEED , GPIO_OBJ->PULLUPDOWN);

}
Pin_State GPIO_Read(void *sturct_base_address){

	GPIO_Obj_t *GPIO_OBJ = sturct_base_address;

	return GPIO_Pin_Read(GPIO_OBJ->PORT, GPIO_OBJ->PIN_NUMBER);

}
void GPIO_Write(void *sturct_base_address,Pin_State State){

	GPIO_Obj_t *GPIO_OBJ = sturct_base_address;

	GPIO_Pin_Write(GPIO_OBJ->PORT, GPIO_OBJ->PIN_NUMBER, State);

}

void GPIO_Port_CLK_Enable(Port_Name PORT){

	switch(PORT){
	case 0:
		RCC->AHB1ENR.GPIOAEN = 1;
		break;
	case 1:
		RCC->AHB1ENR.GPIOBEN = 1;
		break;
	case 2:
		RCC->AHB1ENR.GPIOCEN = 1;
		break;
	case 3:
		RCC->AHB1ENR.GPIODEN = 1;
		break;
	case 4:
		RCC->AHB1ENR.GPIOEEN = 1;
		break;
	case 5:
		RCC->AHB1ENR.GPIOFEN = 1;
		break;
	case 6:
		RCC->AHB1ENR.GPIOGEN = 1;
		break;
	case 7:
		RCC->AHB1ENR.GPIOHEN = 1;
		break;
	case 8:
		RCC->AHB1ENR.GPIOIEN = 1;
		break;
	case 9:
		RCC->AHB1ENR.GPIOJEN = 1;
		break;
	case 10:
		RCC->AHB1ENR.GPIOKEN = 1;
		break;
	}

}

void GPIO_Port_CLK_Disable(Port_Name PORT){

	switch(PORT){
	case 0:
		RCC->AHB1ENR.GPIOAEN = 0;
		break;
	case 1:
		RCC->AHB1ENR.GPIOBEN = 0;
		break;
	case 2:
		RCC->AHB1ENR.GPIOCEN = 0;
		break;
	case 3:
		RCC->AHB1ENR.GPIODEN = 0;
		break;
	case 4:
		RCC->AHB1ENR.GPIOEEN = 0;
		break;
	case 5:
		RCC->AHB1ENR.GPIOFEN = 0;
		break;
	case 6:
		RCC->AHB1ENR.GPIOGEN = 0;
		break;
	case 7:
		RCC->AHB1ENR.GPIOHEN = 0;
		break;
	case 8:
		RCC->AHB1ENR.GPIOIEN = 0;
		break;
	case 9:
		RCC->AHB1ENR.GPIOJEN = 0;
		break;
	case 10:
		RCC->AHB1ENR.GPIOKEN = 0;
		break;
	}

}

void GPIO_Pin_Initialize(Port_Name PORT, uint8_t PIN_NUMBER, Pin_Mode MODE, Pin_Output_Type OTYPE, Pin_Output_Speed OSPEED, Pin_PUPD PULLUPDOWN)
{

	GPIO_Port_CLK_Enable(PORT);								//Enabling the corresponding GPIO Clock

	GPIO->PORT[PORT].MODER &= ~(0x3<<(PIN_NUMBER*2));		//Clearing the mode register

	GPIO->PORT[PORT].MODER |= MODE<<(PIN_NUMBER*2);			//Setting the mode register

	GPIO->PORT[PORT].OTYPER &= ~(0x1<<PIN_NUMBER);			//Clearing the output type register

	GPIO->PORT[PORT].OTYPER |= OTYPE<<PIN_NUMBER;			//Setting the output type register

	GPIO->PORT[PORT].OSPEEDR &= ~(0x3<<(PIN_NUMBER*2));		//Clearing the output speed register

	GPIO->PORT[PORT].OSPEEDR |=  OSPEED<<(PIN_NUMBER*2);	//Setting the output speed register

	GPIO->PORT[PORT].PUPDR &= ~(0x3<<(PIN_NUMBER*2));		//Clearing the pull up pull down register

	GPIO->PORT[PORT].PUPDR |=  PULLUPDOWN<<(PIN_NUMBER*2);  //Setting the pull up pull down register

}

void GPIO_Pin_Set_Alternate_Function(Port_Name PORT, uint8_t PIN_NUMBER, uint8_t ALTFN)
{

	GPIO->PORT[PORT].AFR &= ~(0xF<<(PIN_NUMBER*4));		//Clearing the alternate function down register

	GPIO->PORT[PORT].AFR |=  ALTFN<<(PIN_NUMBER*4);  	//Setting the pull up pull down register

}

void GPIO_Port_DeInitialize(Port_Name PORT)
{
	switch(PORT){
	case 0:
		RCC->AHB1ENR.GPIOAEN = 0;
		RCC->AHB1RSTR.GPIOARST = 1;
		RCC->AHB1RSTR.GPIOARST = 0;
		break;
	case 1:
		RCC->AHB1ENR.GPIOBEN = 0;
		RCC->AHB1RSTR.GPIOBRST = 1;
		RCC->AHB1RSTR.GPIOBRST = 0;
		break;
	case 2:
		RCC->AHB1ENR.GPIOCEN = 0;
		RCC->AHB1RSTR.GPIOCRST = 1;
		RCC->AHB1RSTR.GPIOCRST = 0;
		break;
	case 3:
		RCC->AHB1ENR.GPIODEN = 0;
		RCC->AHB1RSTR.GPIODRST = 1;
		RCC->AHB1RSTR.GPIODRST = 0;
		break;
	case 4:
		RCC->AHB1ENR.GPIOEEN = 0;
		RCC->AHB1RSTR.GPIOERST = 1;
		RCC->AHB1RSTR.GPIOERST = 0;
		break;
	case 5:
		RCC->AHB1ENR.GPIOFEN = 0;
		RCC->AHB1RSTR.GPIOFRST = 1;
		RCC->AHB1RSTR.GPIOFRST = 0;
		break;
	case 6:
		RCC->AHB1ENR.GPIOGEN = 0;
		RCC->AHB1RSTR.GPIOGRST = 1;
		RCC->AHB1RSTR.GPIOGRST = 0;
		break;
	case 7:
		RCC->AHB1ENR.GPIOHEN = 0;
		RCC->AHB1RSTR.GPIOHRST = 1;
		RCC->AHB1RSTR.GPIOHRST = 0;
		break;
	case 8:
		RCC->AHB1ENR.GPIOIEN = 0;
		RCC->AHB1RSTR.GPIOIRST = 1;
		RCC->AHB1RSTR.GPIOIRST = 0;
		break;
	case 9:
		RCC->AHB1ENR.GPIOJEN = 0;
		RCC->AHB1RSTR.GPIOJRST = 1;
		RCC->AHB1RSTR.GPIOJRST = 0;
		break;
	case 10:
		RCC->AHB1ENR.GPIOKEN = 0;
		RCC->AHB1RSTR.GPIOKRST = 1;
		RCC->AHB1RSTR.GPIOKRST = 0;
		break;
	}

}

void GPIO_Pin_DeInitialize(Port_Name PORT, uint8_t PIN_NUMBER){

	GPIO_Port_CLK_Enable(PORT);								//Enabling the corresponding GPIO Clock

	GPIO->PORT[PORT].MODER &= ~(0x3<<(PIN_NUMBER*2));		//Clearing the mode register

	GPIO->PORT[PORT].OTYPER &= ~(0x1<<PIN_NUMBER);			//Clearing the output type register

	GPIO->PORT[PORT].OSPEEDR &= ~(0x3<<(PIN_NUMBER*2));		//Clearing the output speed register

	GPIO->PORT[PORT].PUPDR &= ~(0x3<<(PIN_NUMBER*2));		//Clearing the pull up pull down register

	if (((PORT==GPIOA) &&(PIN_NUMBER == 15 || PIN_NUMBER == 14 || PIN_NUMBER == 13)) || ((PORT==GPIOB) &&(PIN_NUMBER == 4 || PIN_NUMBER ==3)))
	{

		GPIO->PORT[PORT].MODER |= (0x2 << PIN_NUMBER*2);	//Reset to value different than zero

		GPIO->PORT[PORT].OSPEEDR |= (0x2 << PIN_NUMBER*2);	//Reset to value different than zero

		GPIO->PORT[PORT].PUPDR |= (0x2 << PIN_NUMBER*2);	//Reset to value different than zero

	}
}

Pin_State GPIO_Pin_Read(Port_Name PORT, uint8_t PIN_NUMBER)
{

	return (Pin_State)((GPIO->PORT[PORT].IDR >> PIN_NUMBER) & 0x1);

}

uint16_t GPIO_Port_Read(Port_Name PORT)
{

	return (uint16_t)((GPIO->PORT[PORT].IDR));

}

void GPIO_Pin_Write(Port_Name PORT, uint8_t PIN_NUMBER, Pin_State State)
{

	GPIO->PORT[PORT].ODR = (GPIO->PORT[PORT].ODR & ~(0x1 << PIN_NUMBER)) | (State << PIN_NUMBER);

}

void GPIO_Port_Write(Port_Name PORT, uint16_t Value){

	GPIO->PORT[PORT].ODR = Value;

}

void GPIO_Pin_Toggle(Port_Name PORT, uint8_t PIN_NUMBER){

	GPIO->PORT[PORT].ODR ^= ( 1 << PIN_NUMBER);
}
