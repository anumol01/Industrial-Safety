#include "lcd.h"
/*
 * lcd.c
 *
 *  Created on: 16-Jan-2023
 *      Author: Alok Ranjan
 */

void SetOutput(GPIO_TypeDef *gpio,uint32_t pin)
{
	gpio->MODER|=(1<<(pin*2));
	gpio->MODER&=~(1<<((pin*2)+1));
}

void SetBit(GPIO_TypeDef *gpio,uint32_t pin)
{
	gpio->ODR|=(1<<pin);
}

void ClrBit(GPIO_TypeDef *gpio,uint32_t pin)
{
	gpio->ODR&=~(1<<pin);
}

void CplBit(GPIO_TypeDef *gpio,uint32_t pin)
{
	gpio->ODR^=(1<<pin);
}

void DelayLcd(void)
{
	uint32_t i=0;
	for(i=0;i<16800;i++);
}

void LcdInit(void)
{
	RCC->AHB1ENR |=(RCC_AHB1ENR_GPIOAEN);
	RCC->AHB1ENR |=(RCC_AHB1ENR_GPIOBEN);
	SetOutput(PORT_RS,PIN_RS);
	SetOutput(PORT_EN,PIN_EN);
	SetOutput(PORT_D4,PIN_D4);
	SetOutput(PORT_D5,PIN_D5);
	SetOutput(PORT_D6,PIN_D6);
	SetOutput(PORT_D7,PIN_D7);

	LcdFxn(0,0x33);
	LcdFxn(0,0x32);
	LcdFxn(0,0x28);
	LcdFxn(0,0x0c);
	LcdFxn(0,0x01);
}

void LcdFxn(uint8_t cmd,uint8_t val)
{

	if(val&(1<<7))
	SetBit(PORT_D7,PIN_D7);
	else
	ClrBit(PORT_D7,PIN_D7);

	if(val&(1<<6))
	SetBit(PORT_D6,PIN_D6);
	else
	ClrBit(PORT_D6,PIN_D6);

	if(val&(1<<5))
	SetBit(PORT_D5,PIN_D5);
	else
	ClrBit(PORT_D5,PIN_D5);

	if(val&(1<<4))
	SetBit(PORT_D4,PIN_D4);
	else
	ClrBit(PORT_D4,PIN_D4);

	if(cmd)   SetBit(PORT_RS,PIN_RS);
	else	  ClrBit(PORT_RS,PIN_RS);

	SetBit(PORT_EN,PIN_EN);
	DelayLcd();
	ClrBit(PORT_EN,PIN_EN);
	DelayLcd();


	if(val&(1<<3))
	SetBit(PORT_D7,PIN_D7);
	else
	ClrBit(PORT_D7,PIN_D7);

	if(val&(1<<2))
	SetBit(PORT_D6,PIN_D6);
	else
	ClrBit(PORT_D6,PIN_D6);

	if(val&(1<<1))
	SetBit(PORT_D5,PIN_D5);
	else
	ClrBit(PORT_D5,PIN_D5);

	if(val&(1<<0))
	SetBit(PORT_D4,PIN_D4);
	else
	ClrBit(PORT_D4,PIN_D4);

	if(cmd)   SetBit(PORT_RS,PIN_RS);
	else	  ClrBit(PORT_RS,PIN_RS);

	SetBit(PORT_EN,PIN_EN);
	DelayLcd();
	ClrBit(PORT_EN,PIN_EN);
	DelayLcd();

}

void lprint(uint8_t add, char *str)
{
   uint8_t  i=0;
   LcdFxn(0,add);

   while(str[i]!=0)
   {
	   LcdFxn(1,str[i]);
	   str++;
	   if(i>16)
		   break;
   }
}

void aprint(uint32_t dval,uint32_t addr)
{
long int var=0;
unsigned char d1,d2,d3,d4=0;
var=dval;
d1=var%10;
var=var/10;
d2=var%10;
var=var/10;
d3=var%10;
d4=var/10;
LcdFxn(0,addr);
LcdFxn(1,d4|0x30);
LcdFxn(1,d3|0x30);
LcdFxn(1,d2|0x30);
LcdFxn(1,d1|0x30);
}
