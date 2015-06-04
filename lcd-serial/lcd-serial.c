/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2014 Chuck McManis <cmcmanis@mcmanis.com>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdint.h>
#include <math.h>
#include "clock.h"
#include "console.h"
#include "sdram.h"
#include "lcd-spi.h"
#include "gfx.h"
#include "stmpe811.h"

/* Convert degrees to radians */
#define d2r(d) ((d) * 6.2831853 / 360.0)

/*
 * This is our example, the heavy lifing is actually in lcd-spi.c but
 * this drives that code.
 */
int main(void)
{
	//int p1, p2, p3;
	stmpe811_t stmpe811_data;

	clock_setup();
	console_setup(115200);
	sdram_init();
	lcd_spi_init();
	console_puts("LCD Initialized\n");
	console_puts("Should have a checker pattern, press any key to proceed\n");
	msleep(2000);
	(void) console_getc(1);
/*	gfx_init(lcd_draw_pixel, 240, 320);
	gfx_fillScreen(LCD_GREY);
	gfx_fillRoundRect(10, 10, 220, 220, 5, LCD_WHITE);
	gfx_drawRoundRect(10, 10, 220, 220, 5, LCD_RED);
	gfx_fillCircle(20, 250, 10, LCD_RED);
	gfx_fillCircle(120, 250, 10, LCD_GREEN);
	gfx_fillCircle(220, 250, 10, LCD_BLUE);
	gfx_setTextSize(2);
	gfx_setCursor(15, 25);
	gfx_puts("STM32F4-DISCO");
	gfx_setTextSize(1);
	gfx_setCursor(15, 49);
	gfx_puts("We're gonna rock this");
	gfx_setCursor(15, 60);
	gfx_puts("board till sunrise!");
	lcd_show_frame();
	msleep(2000);
	gfx_fillScreen(LCD_GREEN);
	gfx_fillRoundRect(10, 10, 220, 220, 5, LCD_WHITE);
	gfx_drawRoundRect(10, 10, 220, 220, 5, LCD_RED);
	gfx_setTextSize(2);
	gfx_setCursor(15, 25);
	gfx_puts("FLUFSOR!");
	gfx_setTextSize(1);
	gfx_setCursor(15, 49);
	gfx_puts("This board super-");
	gfx_setCursor(15, 60);
	gfx_puts("rocks!");
	lcd_show_frame();
	msleep(2000);
	gfx_fillScreen(LCD_WHITE);
	gfx_fillRoundRect(10, 10, 220, 220, 5, LCD_WHITE);
	gfx_drawRoundRect(10, 10, 220, 220, 5, LCD_BLACK);
	gfx_setTextSize(2);
	gfx_setCursor(15, 25);
	gfx_puts("FLUFSOR!");
	gfx_setTextSize(1);
	gfx_setCursor(15, 49);
	gfx_puts("This board super-");
	gfx_setCursor(15, 60);
	gfx_puts("rocks!");
	lcd_show_frame();
	msleep(2000);
	gfx_fillScreen(LCD_RED);
	gfx_fillRoundRect(10, 10, 220, 220, 5, LCD_WHITE);
	gfx_drawRoundRect(10, 10, 220, 220, 5, LCD_GREEN);
	gfx_setTextSize(2);
	gfx_setCursor(15, 25);
	gfx_puts("FLUFSOR!");
	gfx_setTextSize(1);
	gfx_setCursor(15, 49);
	gfx_puts("This board super-");
	gfx_setCursor(15, 60);
	gfx_puts("rocks!");
	lcd_show_frame();
	msleep(2000);
	gfx_fillScreen(LCD_GREY);
	gfx_fillRoundRect(10, 10, 220, 220, 5, LCD_WHITE);
	gfx_drawRoundRect(10, 10, 220, 220, 5, LCD_GREEN);
	gfx_setTextSize(2);
	gfx_setCursor(15, 25);
	gfx_puts("FLUFSOR!");
	gfx_setTextSize(1);
	gfx_setCursor(15, 49);
	gfx_puts("This board super-");
	gfx_setCursor(15, 60);
	gfx_puts("rocks!");
	lcd_show_frame();
	console_puts("Now it has a bit of structured graphics.\n");
	console_puts("Press a key for some simple animation.\n");
	msleep(2000); */
	gfx_fillScreen(LCD_WHITE);
	lcd_show_frame();
	msleep(2000);
	gfx_fillScreen(LCD_RED);
	lcd_show_frame();
	msleep(2000);
	gfx_fillScreen(LCD_GREEN);
	lcd_show_frame();
	msleep(2000);
	gfx_fillScreen(LCD_BLACK);
	lcd_show_frame();
	if( stmpe811_init() != stmpe811_state_ok ) {
		console_puts("STMPE811 Error!");
	}
	stmpe811_data.orientation = stmpe811_portrait_2;
	console_puts("Press on touchscreen pl0x\n");
	while(1) {
		if( stmpe811_read_touch(&stmpe811_data) == stmpe811_state_pressed ) {
			gfx_fillScreen(LCD_WHITE);
			lcd_show_frame();
			//console_puts("PRESSED!\n")
		} else {
			gfx_fillScreen(LCD_BLACK);
			lcd_show_frame();
			console_puts("NOT PRESSED!");
		}

	}


}
