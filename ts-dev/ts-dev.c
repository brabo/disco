/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2014 Chuck McManis <cmcmanis@mcmanis.com>
 *               2015 brabo <brabo.sil@gmail.com>
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
#include <stdio.h>
#include <math.h>
#include <errno.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/usart.h>
#include "clock.h"
#include "sdram.h"
#include "lcd-spi.h"
#include "gfx.h"
#include "stmpe811.h"

/* Convert degrees to radians */
#define d2r(d) ((d) * 6.2831853 / 360.0)

int _write(int file, char *ptr, int len);

int _write(int file, char *ptr, int len)
{
	int i;

	if (file == 1) {
		for (i = 0; i < len; i++)
			usart_send_blocking(USART1, ptr[i]);
		return i;
	}

	errno = EIO;
	return -1;
}

static void usart_clock_setup(void)
{
	/* Enable GPIOG clock for LED & USARTs. */
	rcc_periph_clock_enable(RCC_GPIOG);
	rcc_periph_clock_enable(RCC_GPIOA);

	/* Enable clocks for USART2. */
	rcc_periph_clock_enable(RCC_USART1);
}

static void usart_setup(void)
{
	/* Setup USART2 parameters. */
	usart_set_baudrate(USART1, 115200);
	usart_set_databits(USART1, 8);
	usart_set_stopbits(USART1, USART_STOPBITS_1);
	usart_set_mode(USART1, USART_MODE_TX);
	usart_set_parity(USART1, USART_PARITY_NONE);
	usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);

	/* Finally enable the USART. */
	usart_enable(USART1);
}

static void gpio_setup(void)
{

	/* Setup GPIO pins for USART1 transmit. */
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO9);

	/* Setup USART1 TX pin as alternate function. */
	gpio_set_af(GPIOA, GPIO_AF7, GPIO9);
}

/*
 * This is our example, the heavy lifing is actually in lcd-spi.c but
 * this drives that code.
 */
int main(void)
{

	clock_setup();
	//console_setup(115200);
	usart_clock_setup();
	gpio_setup();
	usart_setup();
	sdram_init();
	lcd_spi_init();
	gfx_init(lcd_draw_pixel, 240, 320);

	/* First we initialize the touchscreen driver (stmpe811) */
	stmpe811_t stmpe811_data;
	if (stmpe811_init() != stmpe811_state_ok) {
		printf("\r\nSTMPE811 error!\r\n");
	}
	//stmpe811_temp_init();

	/*
	 * Loop and read for touch (pressed state).
	 * If pressed, turn screen green and output coordinates over console.
	 * Otherwise, turn screen red.
	 */
	while (1) {
		stmpe811_get_temp();
		if (stmpe811_read_touch(&stmpe811_data) == stmpe811_state_pressed) {
			gfx_fill_screen(LCD_GREEN);
			lcd_show_frame();

			/*
			 * To have the correct X/Y coordinates,
			 * calibration should be performed.
			 */
			stmpe811_data.x = stmpe811_read_x(stmpe811_data.x);
			stmpe811_data.y = stmpe811_read_y(stmpe811_data.y);
			printf("PRESSED  @  0x%04X X  --  0x%04X Y\r\n", stmpe811_data.x, stmpe811_data.y);
		} else {
			gfx_fill_screen(LCD_RED);
			lcd_show_frame();
		}
	}
}
