/*
 * lcd.h
 *
 *  Created on: Jul 12, 2025
 *      Author: Zhang
 */


#ifndef LCD_H_
#define LCD_H_

#include "main.h"

// Control pins

// Register Select (RS)  - A10
// Enable          (E)   - B5
// D0              (LSB) - B4
// D1                    - B10
// D2                    - A8
// D3                    - A9
// D4                    - C7
// D5                    - B6
// D6                    - A7
// D7              (MSB) - A6

// RCC AHB1ENR -> ENABLE GPIOA, GPIOB, GPIOC
// GPIOx MODER -> SET TO OUTPUT (01)

typedef struct{
	GPIO_TypeDef* port;
	uint32_t pin;
} LCD_PIN;

typedef struct{
	LCD_PIN rs;
	LCD_PIN e;
	LCD_PIN d7;
	LCD_PIN d6;
	LCD_PIN d5;
	LCD_PIN d4;
	LCD_PIN d3;
	LCD_PIN d2;
	LCD_PIN d1;
	LCD_PIN d0;
} LCD;

// pulses (E) to latch data/ instruction
void LCD_PulseEnable(LCD*);

// writes 8 data pins, accepts 8 bits, and calls LCD_PulseEnable
void LCD_WRITE_8(LCD*, uint8_t data);

// given 10 bit full command, runs on the LCD
void LCD_RUN_CMD(LCD*, uint8_t rs, uint8_t rw,uint8_t data);

// initializes LCD as 8 bit instruction, right moving, 2 display line
void LCD_INIT(LCD*);

// prints string to LCD
void LCD_PRINT(LCD*, char * string);

// sets DDRAM address to a certain position
void LCD_SET_CURSOR_POS(LCD*, uint8_t row, uint8_t col);

// clears LCD buffer
void LCD_CLEAR(LCD*);

#endif /* LCD_H_ */
