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
#include <errno.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/rng.h>
#include "clock.h"

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

void print_bits(size_t const size, void const *const ptr)
{
    unsigned char *b = (unsigned char*) ptr;
    unsigned char byte;
    int i, j;

    printf("ret=0x");
    for (i=size-1;i>=0;i--)
    {
        for (j=7;j>=0;j--)
        {
            byte = b[i] & (1<<j);
            byte >>= j;
            printf("%u", byte);
        }
    }
    printf("\r\n");
}


int main(void)
{
	clock_setup();
	usart_clock_setup();
	gpio_setup();
	usart_setup();

	uint32_t rnd;
	uint32_t random;
	//*random = rnd;
	rcc_periph_clock_enable(RCC_RNG);
	printf("enabling rng..\r\n");
	rng_disable();
	rng_enable();
	//uint32_t reg = RNG_CR;
	//printf("RNG_CR=0x%08X\r\n", reg);
	//RNG_CR = 0x04;RNG_CR |= RNG_CR_RNGEN;
	//RNG_CR |= RNG_CR_RNGEN;
	//reg = RNG_CR;
	//printf("RNG_CR=0x%08X\r\n", reg);
	//msleep(500);
	printf("OK!\r\nstarting loop..\r\n");

	while (1) {
		if ((RNG_CR & RNG_CR_RNGEN) != 0x04) {
			printf("not enabled!!\r\n");
			msleep(500);
			continue;
		}
		uint8_t ret = rng_get_random(&random);
		//printf("ret=0x%02X random=0x%08X\r\nret=0x", ret, random);
		print_bits(4, &random);
/*		unsigned char *b;
		b = random;
		int size = 4;
		unsigned char byte;
		int i, j;
		for (i=size-1;i>=0;i--) {
			for (j=7;j>=0;j--) {
				byte = b[i] & (1<<j);
				byte >>= j;
				printf("%u", byte);
			}
		}
		printf("\r\n");*/
		msleep(1000);
	}

	return 0;
}
