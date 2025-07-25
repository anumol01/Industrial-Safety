/*
 * cmn.h
 *
 *  Created on: Jan 21, 2023
 *      Author: ALOK Ranjan
 */

#ifndef CMN_H_
#define CMN_H_

#include "stm32f405xx.h"

#define __NOP() asm("NOP")

void SetOutput(GPIO_TypeDef *gpio,uint32_t pin);
void SetInput(GPIO_TypeDef *gpio,uint32_t pin);
void SetBit(GPIO_TypeDef *gpio,uint32_t pin);
void SetPullup(GPIO_TypeDef *gpio,uint32_t pin);
void ClrBit(GPIO_TypeDef *gpio,uint32_t pin);
void CplBit(GPIO_TypeDef *gpio,uint32_t pin);
uint8_t GetStatus(GPIO_TypeDef *gpio,uint32_t pin);
void SetAltFunc(GPIO_TypeDef *gpio, uint32_t pin, uint8_t num);
void delayms(uint32_t dly);

#endif /* CMN_H_ */
