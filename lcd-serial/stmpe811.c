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
	rcc_periph_clock_enable(RCC_GPIOC);
	rcc_periph_clock_enable(RCC_I2C3);
	
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO8);
	gpio_set_output_options(GPIOA, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, GPIO8);
	gpio_set_af(GPIOA, GPIO_AF4, GPIO8);

	gpio_mode_setup(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO9);
	gpio_set_output_options(GPIOC, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, GPIO9);
	gpio_set_af(GPIOC, GPIO_AF4, GPIO9);

	i2c_peripheral_disable(I2C3);
	i2c_reset(I2C3);

	i2c_set_fast_mode(I2C3);
	i2c_set_clock_frequency(I2C3, I2C_CR2_FREQ_42MHZ);
	i2c_set_ccr(I2C3, 35);
	i2c_set_trise(I2C3, 43);
	i2c_peripheral_enable(I2C3);
	i2c_set_own_7bit_slave_address(I2C3, 0x00);
}


uint8_t i2c_start(uint32_t i2c, uint8_t address, uint8_t mode) {

	i2c_send_start(i2c);

	/* Wait for master mode selected */
	while (!((I2C_SR1(i2c) & I2C_SR1_SB)
		& (I2C_SR2(i2c) & (I2C_SR2_MSL | I2C_SR2_BUSY))));

	i2c_send_7bit_address(i2c, address, mode);

	/* Waiting for address is transferred. */
	while (!(I2C_SR1(i2c) & I2C_SR1_ADDR)) {

	}

	/* Cleaning ADDR condition sequence. */
	uint32_t reg32 = I2C_SR2(i2c);
	(void) reg32; /* unused */

	return 0;
}


uint8_t i2c_write(uint32_t i2c, uint8_t address, uint8_t reg, uint8_t data) {
	i2c_start(i2c, address, I2C_WRITE);

	i2c_send_data(i2c, reg);

	while (!(I2C_SR1(i2c) & (I2C_SR1_BTF)));
	i2c_send_data(i2c, data);

	while (!(I2C_SR1(i2c) & (I2C_SR1_BTF)));

	i2c_send_stop(i2c);

	return 0;
}


uint32_t i2c_read(uint32_t i2c, uint8_t address, uint8_t reg) {
	console_puts(".");

	while ((I2C_SR2(i2c) & I2C_SR2_BUSY));

	i2c_start(i2c, address, I2C_WRITE);
	i2c_send_data(i2c, reg);

	while (!(I2C_SR1(i2c) & (I2C_SR1_BTF)));

	i2c_start(i2c, address, I2C_READ);

	i2c_send_stop(i2c);

	while (!(I2C_SR1(i2c) & I2C_SR1_RxNE));

	uint32_t result = i2c_get_data(i2c);

	I2C_SR1(i2c) &= ~I2C_SR1_AF;

	return result;
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


uint32_t stmpe811_read(uint8_t reg) {
	uint32_t data;
	data = i2c_read(I2C3, STMPE811_ADDRESS, reg);
	return data; 
}


void stmpe811_reads(uint8_t reg, uint8_t *data, uint8_t count) {
	//uint8_t data;
	i2c_reads(I2C3, STMPE811_ADDRESS, reg, data, count);
	//return data; 
}


void stmpe811_reset(uint8_t addr) {
	uint8_t data;

	data = i2c_read(I2C3, addr, STMPE811_SYS_CTRL1);

	data |= 0x02;

	i2c_write(I2C3, addr, STMPE811_SYS_CTRL1, data);

	data &= ~(0x02);

	i2c_write(I2C3, addr, STMPE811_SYS_CTRL1, data);
	msleep(2);
}


stmpe811_state_t stmpe811_init(void) {
	uint32_t data, data2[4], mode;

	uint32_t reg = RCC_I2C3;

	stmpe811_i2c_init();


	msleep(50);

	//int i;

	//for( i = 0x0; i <= 0xFF; i++ ) {
	//	data = i2c_read(I2C3, STMPE811_ADDRESS, i);
	//	printf("\nADDR: 0x%04X  REG: 0x%04X  DATA : 0x%04X\n", STMPE811_ADDRESS, i, data);	
	//	msleep(2);
	//}



	mode = stmpe811_read(STMPE811_SYS_CTRL1); // enable soft-reset
	printf("\n\nsys_ctrl1 == 0x%02X  --->  ", mode);
	mode |= 0x02;
	printf("0x%02X  ==  ", mode);
	stmpe811_write(STMPE811_SYS_CTRL1, mode);
	mode = stmpe811_read(STMPE811_SYS_CTRL1);
	printf("0x%02X\n\n", mode);

	mode = stmpe811_read(STMPE811_SYS_CTRL1); // disable soft-reset
	printf("sys_ctrl1 == 0x%02X  --->  ", mode);
	mode &= ~(0x02);
	printf("0x%02X  ==  ", mode);
	stmpe811_write(STMPE811_SYS_CTRL1, mode);
	mode = stmpe811_read(STMPE811_SYS_CTRL1);
	printf("0x%02X\n\n", mode);



	mode = stmpe811_read(STMPE811_SYS_CTRL2); // disable ts & gpio
	printf("sys_ctrl2 == 0x%02X  --->  ", mode);
	mode = 0x0C;
	printf("0x%02X  ==  ", mode);
	stmpe811_write(STMPE811_SYS_CTRL2, mode);
	mode = stmpe811_read(STMPE811_SYS_CTRL2);
	printf("0x%02X\n\n", mode);


	mode = stmpe811_read(STMPE811_INT_EN);
	printf("int_en == 0x%02X  --->  ", mode);
	stmpe811_write(STMPE811_INT_EN, 0x07); // touch-detection status (interrupt)
	mode = stmpe811_read(STMPE811_INT_EN);
	printf("0x07  ==  0X%02X\n\n", mode);

	mode = stmpe811_read(STMPE811_ADC_CTRL1);
	printf("int_adc_ctrl1 == 0x%02X  --->  ", mode);
	stmpe811_write(STMPE811_ADC_CTRL1, 0x49); // set sample-time to 80 times number of clock and 12-bit adc
	mode = stmpe811_read(STMPE811_ADC_CTRL1);
	printf("0x49  ==  0x%02X\n\n", mode);

	mode = stmpe811_read(STMPE811_ADC_CTRL2);
	printf("int_adc_ctrl2 == 0x%02X  --->  ", mode);
	stmpe811_write(STMPE811_ADC_CTRL2, 0x01); // set ADC clock speed to 3.25 MHz
	mode = stmpe811_read(STMPE811_ADC_CTRL2);
	printf("0x01  ==  0x%02X\n\n", mode);

	mode = stmpe811_read(STMPE811_GPIO_AF);
	printf("int_gpio_af == 0x%02X  --->  ", mode);
	stmpe811_write(STMPE811_GPIO_AF, 0x00); // set GPIO AF to function as ts/adc
	mode = stmpe811_read(STMPE811_GPIO_AF);
	printf("0x00  ==  0x%02X\n\n", mode);

	mode = stmpe811_read(STMPE811_TSC_CFG);
	printf("int_tsc_cfg == 0x%02X  --->  ", mode);
	stmpe811_write(STMPE811_TSC_CFG, 0x9A); // tsc cfg set to avg control = 4 samples
	mode = stmpe811_read(STMPE811_TSC_CFG); //                touch detection delay = 500 microseconds
	printf("0x9A  ==  0x%02X\n\n", mode);   //                panel driver settling time = 500 microcseconds

	mode = stmpe811_read(STMPE811_FIFO_TH);
	printf("int_fifo_th == 0x%02X  --->  ", mode);
	stmpe811_write(STMPE811_FIFO_TH, 0x01); // set fifo threshold
	mode = stmpe811_read(STMPE811_FIFO_TH);
	printf("0x01  ==  0x%02X\n\n", mode);

	mode = stmpe811_read(STMPE811_FIFO_STA);
	printf("int_fifo_sta == 0x%02X  --->  ", mode);
	stmpe811_write(STMPE811_FIFO_STA, 0x01); // set fifo status reset
	mode = stmpe811_read(STMPE811_FIFO_STA);
	printf("0x01  ==  0x%02X\n\n", mode);

	mode = stmpe811_read(STMPE811_FIFO_STA);
	printf("int_fifo_sta == 0x%02X  --->  ", mode);
	stmpe811_write(STMPE811_FIFO_STA, 0x00); // reset fifo status reset
	mode = stmpe811_read(STMPE811_FIFO_STA);
	printf("0x00  ==  0x%02X\n\n", mode);

	mode = stmpe811_read(STMPE811_TSC_FRACTION_Z);
	printf("int_tsc_fraction_z == 0x%02X  --->  ", mode);
	stmpe811_write(STMPE811_TSC_FRACTION_Z, 0x07); // set fractional part to 7, whole part to 1
	mode = stmpe811_read(STMPE811_TSC_FRACTION_Z);
	printf("0x07  ==  0x%02X\n\n", mode);

	mode = stmpe811_read(STMPE811_TSC_I_DRIVE);
	printf("int_tsc_i_drive == 0x%02X  --->  ", mode);
	stmpe811_write(STMPE811_TSC_I_DRIVE, 0x01); // set current limit value to 50 mA
	mode = stmpe811_read(STMPE811_TSC_I_DRIVE);
	printf("0x01  ==  0x%02X\n\n", mode);

	mode = stmpe811_read(STMPE811_TSC_CTRL);
	printf("int_tsc_ctrl == 0x%02X  --->  ", mode);
	stmpe811_write(STMPE811_TSC_CTRL, 0x01); // enable TSC (touchscreen clock)
	mode = stmpe811_read(STMPE811_TSC_CTRL);
	printf("0x01  ==  0x%02X\n\n", mode);

	mode = stmpe811_read(STMPE811_INT_STA);
	printf("int_int_sta == 0x%02X  --->  ", mode);
	stmpe811_write(STMPE811_INT_STA, 0xFF); // clear everything
	mode = stmpe811_read(STMPE811_INT_STA);
	printf("0xFF  ==  0x%02X\n\n", mode);

	mode = stmpe811_read(STMPE811_INT_CTRL);
	printf("int_int_ctrl == 0x%02X  --->  ", mode);
	stmpe811_write(STMPE811_INT_CTRL, 0x01); // enable global interrupts
	mode = stmpe811_read(STMPE811_INT_CTRL);
	printf("0x01  ==  0x%02X\n\n", mode);

	msleep(2);

	return stmpe811_state_ok;
}


void stmpe811_reset_fifo(void) {
	uint8_t sta;
	sta = stmpe811_read(STMPE811_FIFO_STA);
	sta |= 0x01;
	stmpe811_write(STMPE811_FIFO_STA, sta);
	sta &= ~(0x01);
	stmpe811_write(STMPE811_FIFO_STA, sta);
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

	//console_puts("read val..");
	val = stmpe811_read(STMPE811_TSC_CTRL);
	//console_puts("OK!\n");
	//printf("VAL=0x%02X\n", val);
	if(( val & 0x80 ) == 0 ) {
		stmpe811_data->pressed = stmpe811_state_released;

		stmpe811_reset_fifo();
		//console_puts("not pressed!\n");

		return stmpe811_state_released;
	}

	//console_puts("pressed!\n");
	//printf("VAL=0x%02X\n", val);

/*	if (stmpe811_data->orientation == stmpe811_portrait_1) {
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
	} */
	stmpe811_data->x = stmpe811_read_x(stmpe811_data->x);
	stmpe811_data->y = 319 - stmpe811_read_y(stmpe811_data->y);
	printf("coords:  0x%04X X - 0x%04X Y\n", stmpe811_data->x, stmpe811_data->y);

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