/*
 * lcd.c
 *
 *  Created on: Jul 12, 2025
 *      Author: Zhang
 */

#include "lcd.h"

void LCD_WRITE_PIN(LCD_PIN* pin, uint8_t state){
	state ? LL_GPIO_SetOutputPin(pin->port, pin->pin) : LL_GPIO_ResetOutputPin(pin->port, pin->pin);
}

void LCD_PulseEnable(LCD* lcd) {
	LCD_WRITE_PIN(&(lcd->e), 1);
    delay_us(2);
	LCD_WRITE_PIN(&(lcd->e), 0);
    delay_us(1);
}

void LCD_WRITE_8(LCD* lcd, uint8_t data) {
    LCD_WRITE_PIN(&(lcd->d7), (data >> 7) & 1);
    LCD_WRITE_PIN(&(lcd->d6), (data >> 6) & 1);
    LCD_WRITE_PIN(&(lcd->d5), (data >> 5) & 1);
    LCD_WRITE_PIN(&(lcd->d4), (data >> 4) & 1);
    LCD_WRITE_PIN(&(lcd->d3), (data >> 3) & 1);
    LCD_WRITE_PIN(&(lcd->d2), (data >> 2) & 1);
    LCD_WRITE_PIN(&(lcd->d1), (data >> 1) & 1);
    LCD_WRITE_PIN(&(lcd->d0), (data >> 0) & 1);

    LCD_PulseEnable(lcd);
}

void LCD_RUN_CMD(LCD* lcd, uint8_t rs, uint8_t rw, uint8_t data){
	// if writing, don't run command
	if (rw != 0 ) return;

	LCD_WRITE_PIN(&(lcd->rs), rs);

	LCD_WRITE_8(lcd, data);

	delay_us(40);
}

void LCD_INIT(LCD* lcd){
	LL_mDelay(50); // give time for power on

	LCD_RUN_CMD(lcd, 0, 0, 0b00000001); // clear display
	LL_mDelay(2);

	LCD_RUN_CMD(lcd, 0, 0, 0b00111000); // function set to 8 bit, 2 line display, 5x8 dot character font
	LCD_RUN_CMD(lcd, 0, 0, 0b00001100); // turns on display
	LCD_RUN_CMD(lcd, 0, 0, 0b00000110); // sets to increment by one, shift to right
}

void LCD_CLEAR(LCD* lcd){
	LCD_RUN_CMD(lcd, 0, 0, 0b00000001); // clear display
	LL_mDelay(2);
}

// function to print char string to LCD
void LCD_PRINT(LCD* lcd, char * string){
	LCD_WRITE_PIN(&(lcd->rs), 1);

	uint8_t count = 0;

	while (*string){
		if (count == 16){
			LCD_RUN_CMD(lcd, 0, 0, 0b11000000);
		}

		LCD_WRITE_PIN(&(lcd->rs), 1);
		LCD_WRITE_8(lcd, (uint8_t)*string++);
		delay_us(40);
		count++;
	}
}

// 0 indexed row and column
void LCD_SET_CURSOR_POS(LCD* lcd, uint8_t row, uint8_t col){
	uint8_t DDRAM_addr = 0;

    if (row == 0) {
        DDRAM_addr = col;
    } else if (row == 1) {
        DDRAM_addr = (0x40 + col);
    } else {
        DDRAM_addr = col;
    }

    LCD_RUN_CMD(lcd, 0, 0, 0x80 | DDRAM_addr);
}
