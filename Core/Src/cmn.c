/*
 * cmn.c
 *
 *  Created on: Jan 21, 2023
 *      Author: ALOK Ranjan
 */


#include "cmn.h"

void delayms(uint32_t dly)
{
  uint32_t i,j=0;
  for(i=0;i<dly;i++)
  for(j=0;j<16000;j++);
}




void SetInput(GPIO_TypeDef *gpio,uint32_t pin)
{
	gpio->MODER&=~(1<<(pin*2));
	gpio->MODER&=~(1<<((pin*2)+1));
}




void SetPullup(GPIO_TypeDef *gpio,uint32_t pin)
{
	gpio->PUPDR |=(1<<(pin*2));
	gpio->PUPDR &=~(1<<((pin*2)+1));
}

uint8_t GetStatus(GPIO_TypeDef *gpio, uint32_t pin)
{
	return((gpio->IDR)&(1UL<<pin));
}

void SetAltFunc(GPIO_TypeDef *gpio, uint32_t pin, uint8_t num)
{
	gpio->MODER&=~(1<<(pin*2));
	gpio->MODER|=(1<<((pin*2)+1));

	if(pin<8)
		gpio->AFR[0] |= num<<(pin*4);
	else
		gpio->AFR[1] |= num<<((pin-8)*4);
}



