/*
 * lcd.h
 *
 *  Created on: 16-Jan-2023
 *      Author: Alok Ranjan
 */

#ifndef LCD_H_
#define LCD_H_

#include "stm32f405xx.h"

#define PORT_RS		GPIOA
#define PIN_RS		0

#define PORT_EN		GPIOA
#define PIN_EN		1

#define PORT_D4		GPIOB
#define PIN_D4		12

#define PORT_D5		GPIOB
#define PIN_D5		13

#define PORT_D6		GPIOB
#define PIN_D6		14

#define PORT_D7		GPIOB
#define PIN_D7		15

void SetOutput(GPIO_TypeDef *gpio,uint32_t pin);

void SetBit(GPIO_TypeDef *gpio,uint32_t pin);

void ClrBit(GPIO_TypeDef *gpio,uint32_t pin);
void CplBit(GPIO_TypeDef *gpio,uint32_t pin);
void DelayLcd(void);
void LcdInit(void);
void LcdFxn(uint8_t cmd,uint8_t val);
void lprint(uint8_t add, char *str);
void aprint(uint32_t dval,uint32_t addr);




#endif /* LCD_H_ */
