/*
 * lcd_16x2.c
 *
 *  Created on: Nov 7, 2023
 *      Author: Joppe
 */

#include "lcd_16x2.hpp"

LCD_Handle lcd_create (I2C_HandleTypeDef dev, uint8_t addr)
{
	LCD_Handle lcd = {
		.device = dev,
		.address = addr,
	};
	return lcd;
}

void lcd_send_cmd (LCD_Handle lcd, char cmd, osMutexId_t mutex)
{
  char data_u, data_l;
	uint8_t data_t[4]; // Create a char array of 4 elements
	data_u = (cmd&0xf0); // data_u will contain the first 4 bits of cmd in the leftmost 4 bitspaces (the last 4 bits must be empty)
	data_l = ((cmd<<4)&0xf0); // data_l will contain the last 4 bits of cmd in the leftmost 4 bitspaces (see above)
	// The data to send to the LCD should look like this:
	// b7 b6 b5 b4 b3 b2 b1 b0
	// d3 d2 d1 d0 ?? en ?? rs
	data_t[0] = data_u|0x0C;  // 0b DATA 1100 (BL on, EN on, RW off, RS off)
	data_t[1] = data_u|0x08;  // 0b DATA 1000 (BL on, EN off, RW off, RS off)
	data_t[2] = data_l|0x0C;  //en=1, rs=0
	data_t[3] = data_l|0x08;  //en=0, rs=0
	osMutexAcquire(mutex, 20);
	HAL_I2C_Master_Transmit(&lcd.device, lcd.address, (uint8_t *) data_t, 4, 50); // Send the char array to the I2C master to transmit
	osMutexRelease(mutex);
	//{
	//	HAL_GPIO_TogglePin(GPIOE, LED2_Pin);
	//}
}

void lcd_send_data (LCD_Handle lcd, char data, osMutexId_t mutex)
{
	char data_u, data_l;
	uint8_t data_t[4];
	data_u = (data&0xf0);
	data_l = ((data<<4)&0xf0);
	data_t[0] = data_u|0x0D;  // 0b DATA 1101 = BL=1, EN=1, RW=0, RS=1
	data_t[1] = data_u|0x09;  // 0b DATA 1001 = BL=1, EN=0, RW=0, RS=1
	data_t[2] = data_l|0x0D;  //en=1, rs=1
	data_t[3] = data_l|0x09;  //en=0, rs=1
	osMutexAcquire(mutex, 10);
	HAL_StatusTypeDef ret = HAL_I2C_Master_Transmit(&lcd.device, lcd.address,(uint8_t *) data_t, 4, 50);
	osMutexRelease(mutex);
	if (ret == HAL_ERROR)
	{
		HAL_GPIO_WritePin(GPIOE, LED1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOE, LED2_Pin, GPIO_PIN_RESET);
	}
	else
	{
		HAL_GPIO_WritePin(GPIOE, LED1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOE, LED2_Pin, GPIO_PIN_SET);
	}
}

void lcd_init (LCD_Handle lcd, osMutexId_t mutex)
{
	// 4 bit initialisation
	osDelay(15);  // wait for >15ms
	lcd_send_cmd (lcd, 0b00110000, mutex);
	osDelay(5);  // wait for >4.1ms
	lcd_send_cmd (lcd, 0b00110000, mutex);
	osDelay(1);  // wait for >100us
	lcd_send_cmd (lcd, 0b00110000, mutex);
	osDelay(10);
	lcd_send_cmd (lcd, 0b00100000, mutex);  // 4bit mode
	osDelay(10);

  // display initialisation
	lcd_send_cmd (lcd, 0b00011100, mutex); // Function set --> DL=0 (4 bit mode), N = 1 (2 line display) F = 0 (5x8 characters)
	osDelay(1);
	lcd_send_cmd (lcd, 0b00001000, mutex); //Display on/off control --> D=0,C=0, B=0  ---> display off
	osDelay(1);
	lcd_send_cmd (lcd, 0b00000001, mutex);  // clear display
	osDelay(1);
	osDelay(1);
	lcd_send_cmd (lcd, 0b00000110, mutex); //Entry mode set --> I/D = 1 (increment cursor) & S = 0 (no shift)
	osDelay(1);
	lcd_send_cmd (lcd, 0b00001100, mutex); //Display on/off control --> D = 1, C and B = 0. (Cursor and blink, last two bits)
}

void lcd_clear_screen (LCD_Handle lcd, osMutexId_t mutex)
{
	lcd_send_cmd(lcd, 0x01, mutex);
	osDelay(2);
}

void lcd_send_string (LCD_Handle lcd, char *str, osMutexId_t mutex)
{
	while (*str) lcd_send_data (lcd, *str++, mutex);
}

void lcd_set_cursor (LCD_Handle lcd, uint8_t position, osMutexId_t mutex)
{
		position = position | 0x80;
		char data_u, data_l;
		uint8_t data_t[4];
		data_u = (position&0xf0);
		data_l = ((position<<4)&0xf0);
		data_t[0] = data_u|0x0C;  // 0b DATA 1100 = BL=1, EN=1, RW=0, RS=0
		data_t[1] = data_u|0x08;  // 0b DATA 1000 = BL=1, EN=0, RW=0, RS=0
		data_t[2] = data_l|0x0C;  //en=1, rs=1
		data_t[3] = data_l|0x08;  //en=0, rs=1
		osMutexAcquire(mutex, 10);
		HAL_I2C_Master_Transmit (&lcd.device, lcd.address,(uint8_t *) data_t, 4, 100);
		osMutexRelease(mutex);
		osDelay(2);

}
