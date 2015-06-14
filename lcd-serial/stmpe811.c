/**
Copyright (C) 2015 brabo

This program is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public License
as published by the Free Software Foundation; either version 2
of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.	
**/

#include <stdint.h>
#include <stdio.h>
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include "stmpe811.h"
#include "clock.h"
#include "console.h"



void stmpe811_i2c_init() {

	rcc_periph_clock_enable(RCC_GPIOA);
	//rcc_periph_clock_enable(RCC_I2C3);

	//gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
	//	      GPIO_CNF_OUTPUT_ALTFN_OPENDRAIN,
	//	      GPIO8);


	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO8);
	gpio_set_output_options(GPIOA, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, GPIO8);
	gpio_set_af(GPIOA, GPIO_AF4, GPIO8);

	rcc_periph_clock_enable(RCC_GPIOC);

	//gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_50_MHZ,
	//	      GPIO_CNF_OUTPUT_ALTFN_OPENDRAIN,
	//	      GPIO9);
	uint32_t reg = RCC_I2C3;

	gpio_mode_setup(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO9);
	//gpio_set_af(GPIOC, GPIO_AF4, GPIO9);
	gpio_set_output_options(GPIOC, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, GPIO9);
	gpio_set_af(GPIOC, GPIO_AF4, GPIO9);
//i2c_peripheral_disable(I2C1);
	rcc_periph_clock_enable(RCC_I2C3);
	rcc_peripheral_reset(&reg, RCC_APB1RSTR_I2C3RST);
	rcc_peripheral_clear_reset(&reg, RCC_APB1RSTR_I2C3RST);
	//rcc_periph_reset_pulse(I2C3);
	i2c_peripheral_disable(I2C3);
	uint tmp = 0;

	tmp = RCC_PLLCFGR & RCC_PLLCFGR_PLLSRC;
	tmp = tmp >> 22;
	printf("PLLSRC=%08X\r\n", tmp);
	tmp = RCC_PLLCFGR & 0x3f; // & (0x2 << 0x6);
	printf("PLLM=%08X\r\n", tmp);
	tmp = RCC_PLLCFGR & 0x7FC0;
	tmp = tmp >> 6;
	printf("PLLN=%08X\r\n", tmp);
	tmp = RCC_PLLCFGR & (0x2 << RCC_PLLCFGR_PLLP_SHIFT);
	tmp = tmp >> RCC_PLLCFGR_PLLP_SHIFT;
	printf("PLLP=%08X\r\n", tmp);
	//RCC_CFGR_HPRE
	tmp = RCC_CFGR & (0x2 << 0x2);
	//tmp = tmp >> 2;
	printf("SYSCLK=%08X\r\n", tmp);
	tmp = RCC_CFGR & (RCC_CFGR_HPRE_MASK <<RCC_CFGR_HPRE_SHIFT);
	tmp = tmp >> 4;
	printf("HPRE=%08X\r\n", tmp); // 0x0 
	tmp = RCC_CFGR & (RCC_CFGR_PPRE1_MASK <<RCC_CFGR_PPRE1_SHIFT);
	tmp = tmp >> 10;
	printf("PPRE1=%08X\r\n", tmp); // 0x5 = 101 = AHB divided by 4
	i2c_set_clock_frequency(I2C3, I2C_CR2_FREQ_10MHZ);
	//i2c_peripheral_enable(I2C3);
	
	i2c_set_fast_mode(I2C3);
	i2c_set_ccr(I2C3, 0x34);
	i2c_set_trise(I2C3, 0x0b);
	i2c_enable_ack(I2C3);
	i2c_set_own_7bit_slave_address(I2C3, 0x4000);

	i2c_peripheral_enable(I2C3);
	console_puts("stmpe811 i2c init done!\n");

	//gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ,
	//	      GPIO_CNF_OUTPUT_ALTFN_OPENDRAIN,
	//	      GPIO_I2C1_SCL | GPIO_I2C1_SDA);

}


void i2c_start(uint32_t i2c, uint8_t address, uint8_t mode) {
	//console_puts("first start.. ");
	//I2C_SR2(i2c);
	i2c_send_start(i2c);
	while (!((I2C_SR1(i2c) & I2C_SR1_SB)));
	//while (!((I2C_SR1(i2c) & I2C_SR1_SB)
	//	& (I2C_SR2(i2c) & (I2C_SR2_MSL | I2C_SR2_BUSY))));

	i2c_disable_ack(i2c);
	//while (!((I2C_SR1(I2C1) & I2C_SR1_SB)
	//	& (I2C_SR2(I2C1) & (I2C_SR2_MSL | I2C_SR2_BUSY))));

	//console_puts("OK!\n");

	i2c_send_7bit_address(i2c, address, mode);
	while (!(I2C_SR1(i2c) & I2C_SR1_ADDR));
	//while (!(I2C_SR1(I2C3) & I2C_SR1_ADDR));
	//while (!(I2C_SR1(I2C3) & I2C_SR1_TxE));
	//while (!(I2C_SR1(I2C3) & I2C_SR1_ADDR));

	I2C_SR2(i2c);

}


void i2c_write(uint32_t i2c, uint8_t address, uint8_t reg, uint8_t data) {
	//console_puts("initiating write\n");
	i2c_start(i2c, address, I2C_WRITE);

	i2c_send_data(i2c, reg);
	while (!(I2C_SR1(i2c) & I2C_SR1_TxE));

	i2c_send_data(i2c, data);
	while (!(I2C_SR1(i2c) & I2C_SR1_TxE));

	i2c_send_stop(i2c);
	//console_puts("end of write\n");
}


uint8_t i2c_read(uint32_t i2c, uint8_t address, uint8_t reg) {
	console_puts(".\n");
	uint8_t data;

	i2c_enable_ack(i2c);
	i2c_start(i2c, address, I2C_WRITE);

	i2c_send_data(i2c, reg);
	while (!(I2C_SR1(i2c) & I2C_SR1_TxE));

	i2c_send_stop(i2c);

	i2c_enable_ack(i2c);
	i2c_start(i2c, address, I2C_READ);

	//I2C_SR2(i2c);	
	i2c_disable_ack(i2c);
	i2c_send_stop(i2c);
	data = i2c_get_data(i2c);

	//i2c_send_stop(i2c);
	//console_puts("end of read\n");
	return data;
}


void i2c_reads(uint32_t i2c, uint8_t address, uint8_t reg, uint8_t *data, uint8_t count) {
	console_puts(".");
	uint8_t i;

	i2c_enable_ack(i2c);
	i2c_start(i2c, address, I2C_WRITE);

	i2c_send_data(i2c, reg);
	while (!(I2C_SR1(i2c) & I2C_SR1_TxE));

	i2c_send_stop(i2c);

	i2c_enable_ack(i2c);
	i2c_start(i2c, address, I2C_READ);

	for ( i = 0; i < count - 1; i++ ) {

		i2c_disable_ack(i2c);
		i2c_send_stop(i2c);

		data[i] = i2c_get_data(i2c);
	}
	i++;

	i2c_enable_ack(i2c);
	data[i] = i2c_get_data(i2c);
	//i2c_send_stop(i2c);
	//console_puts("end of read\n");

}


void stmpe811_write(uint8_t reg, uint8_t data) {
	i2c_write(I2C3, STMPE811_ADDRESS, reg, data); 
}


uint8_t stmpe811_read(uint8_t reg) {
	uint8_t data;
	data = i2c_read(I2C3, STMPE811_ADDRESS, reg);
	return data; 
}


void stmpe811_reads(uint8_t reg, uint8_t *data, uint8_t count) {
	//uint8_t data;
	i2c_reads(I2C3, STMPE811_ADDRESS, reg, data, count);
	//return data; 
}


void stmpe811_reset(void) {
	//console_puts("initiating stmpe811 reset\n");
	stmpe811_write(STMPE811_SYS_CTRL1, 0x02);
	msleep(5);
	stmpe811_write(STMPE811_SYS_CTRL1, 0x00);
	msleep(2);	
}


stmpe811_state_t stmpe811_init(void) {
	uint8_t data, data2[2], mode;

	uint32_t reg = RCC_I2C3;

	stmpe811_i2c_init();

	//i2c_reset(I2C3);
	//rcc_peripheral_reset(&reg, RCC_APB1RSTR_I2C3RST);
	//rcc_peripheral_clear_reset(&reg, RCC_APB1RSTR_I2C3RST);	
	//rcc_periph_reset_pulse(I2C3);
	console_puts("i2c reset done\n");

	msleep(50);

	stmpe811_reset();
	//console_puts("reset done\n");

	//stmpe811_reads(STMPE811_CHIP_ID, data2, 2);
	//if ((data2[0] << 8 | data2[1]) != STMPE811_CHIP_ID_VALUE) {
	//	uint tmp01 = (data2[0] << 8 | data2[1]);
	//	printf("STMPE811 DEVICE ID: 0x%08X\r\n", tmp01);
	//	return stmpe811_state_error;
	//}

	rcc_peripheral_reset(&reg, RCC_APB1RSTR_I2C3RST);
	rcc_peripheral_clear_reset(&reg, RCC_APB1RSTR_I2C3RST);
	stmpe811_reset();

	mode = stmpe811_read(STMPE811_SYS_CTRL2);
	mode &= ~(0x01);
	stmpe811_write(STMPE811_SYS_CTRL2, mode);
	mode = stmpe811_read(STMPE811_SYS_CTRL2);
	mode &= ~(0x02);
	stmpe811_write(STMPE811_SYS_CTRL2, mode);
	stmpe811_write(STMPE811_ADC_CTRL1, 0x49);
	msleep(2);

	stmpe811_write(STMPE811_ADC_CTRL2, 0x01);

	mode = stmpe811_read(STMPE811_GPIO_AF);
	mode |= 0x1E;
	stmpe811_write(STMPE811_GPIO_AF, mode);

	stmpe811_write(STMPE811_TSC_CFG, 0x9A);
	stmpe811_write(STMPE811_FIFO_TH, 0x01);
	stmpe811_write(STMPE811_FIFO_STA, 0x01);
	stmpe811_write(STMPE811_FIFO_STA, 0x00);
	stmpe811_write(STMPE811_TSC_FRACTION_Z, 0x01);
	stmpe811_write(STMPE811_TSC_I_DRIVE, 0x01);
	stmpe811_write(STMPE811_TSC_CTRL, 0x03);
	stmpe811_write(STMPE811_INT_STA, 0xFF);

	msleep(2);

	return stmpe811_state_ok;
}


void stmpe811_reset_fifo(void) {
	stmpe811_write(STMPE811_FIFO_STA, 0x01);
	stmpe811_write(STMPE811_FIFO_STA, 0x00);
}


uint16_t stmpe811_read_x(uint16_t x) {
	uint8_t data[2];
	int16_t val, dx;
	data[1] = stmpe811_read(STMPE811_TSC_DATA_X);
	data[0] = stmpe811_read(STMPE811_TSC_DATA_X + 1);
	val = (data[1] << 8 | (data[0] & 0xFF));
	
	if (val <= 3000) {
		val = 3900 - val;
	} else {
		val = 3800 - val;
	}
	
	val /= 15;
	
	if (val > 239) {
		val = 239;
	} else if (val < 0) {
		val = 0;
	}
	
	dx = (val > x) ? (val - x) : (x - val);
	if (dx > 4) {
		return val;
	}
	return x;
}


uint16_t stmpe811_read_y(uint16_t y) {
	uint8_t data[2];
	int16_t val, dy;
	data[1] = stmpe811_read(STMPE811_TSC_DATA_Y);
	data[0] = stmpe811_read(STMPE811_TSC_DATA_Y + 1);
	val = (data[1] << 8 | (data[0] & 0xFF));

	val -= 360;
	val = val / 11;

	if (val <= 0) {
		val = 0;
	} else if (val >= 320) {
		val = 319;
	}
	
	dy = (val > y) ? (val - y) : (y - val);
	if (dy > 4) {
		return val;
	}
	return y;
}


stmpe811_state_t stmpe811_read_touch(stmpe811_t *stmpe811_data) {
	uint8_t val;

	stmpe811_data->last_pressed = stmpe811_data->pressed;

	val = stmpe811_read(STMPE811_TSC_CTRL);
	if(( val & 0x80 ) == 0 ) {
		stmpe811_data->pressed = stmpe811_state_released;

		stmpe811_reset_fifo();
		console_puts("not pressed!\n");

		return stmpe811_state_released;
	}

	console_puts("pressed!\n");
	printf("VAL=0x%02X\n", val);

	if (stmpe811_data->orientation == stmpe811_portrait_1) {
		stmpe811_data->x = 239 - stmpe811_read_x(stmpe811_data->x);
		stmpe811_data->y = 319 - stmpe811_read_y(stmpe811_data->y);
	} else if (stmpe811_data->orientation == stmpe811_portrait_2) {
		stmpe811_data->x = stmpe811_read_x(stmpe811_data->x);
		stmpe811_data->y = stmpe811_read_y(stmpe811_data->y);
	} else if (stmpe811_data->orientation == stmpe811_landscape_1) {
		stmpe811_data->y = stmpe811_read_x(stmpe811_data->y);
		stmpe811_data->x = 319 - stmpe811_read_y(stmpe811_data->x);
	} else if (stmpe811_data->orientation == stmpe811_landscape_2) {
		stmpe811_data->y = 239 - stmpe811_read_x(stmpe811_data->y);
		stmpe811_data->x = stmpe811_read_y(stmpe811_data->x);
	}

	stmpe811_reset_fifo();

	if (stmpe811_data->orientation == stmpe811_portrait_1 || stmpe811_data->orientation == stmpe811_portrait_2) {
		//Portrait
		if (stmpe811_data->x > 0 && stmpe811_data->x < 239 && stmpe811_data->y > 0 && stmpe811_data->y < 319) {
			stmpe811_data->pressed = stmpe811_state_pressed;
			return stmpe811_state_pressed;
		}
	} else {
		//Landscape
		if (stmpe811_data->x > 0 && stmpe811_data->x < 319 && stmpe811_data->y > 0 && stmpe811_data->y < 239) {	
			stmpe811_data->pressed = stmpe811_state_pressed;
			return stmpe811_state_pressed;
		}
	}
	
	stmpe811_data->pressed = stmpe811_state_released;
	
	return stmpe811_state_released;
}