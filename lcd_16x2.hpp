/*
 * lcd_16x2.h
 *
 *  Created on: Nov 7, 2023
 *      Author: Joppe
 */

#ifndef LCD_16X2_H_
#define LCD_16X2_H_

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_i2c.h"
#include "cmsis_os2.h"
#include "main.h"

#define CMD_CLEAR_SCREEN 0x01
#define CMD_CURSOR_RESET 0x02


typedef struct {
	I2C_HandleTypeDef 			device;			/*!< I2C device handle 						*/
	uint8_t 					address;		/*!< I2C slave device address 				*/
}LCD_Handle;

LCD_Handle lcd_create (I2C_HandleTypeDef dev, uint8_t addr);

void lcd_send_cmd (LCD_Handle lcd, char cmd, osMutexId_t mutex);

void lcd_send_data (LCD_Handle lcd, char data, osMutexId_t mutex);

void lcd_init (LCD_Handle lcd, osMutexId_t mutex);

void lcd_clear_screen (LCD_Handle lcd, osMutexId_t mutex);

void lcd_send_string (LCD_Handle lcd, char *str, osMutexId_t mutex);

void lcd_set_cursor (LCD_Handle lcd, uint8_t position, osMutexId_t mutex);

#endif /* LCD_16X2_H_ */
