/*
 * i2c_lcd.c
 *
 *  Created on: Aug 30, 2025
 *      Author: jerel
 */
#include "i2c_lcd.h"


static uint8_t bl_state = LCD_BL; // backlight ON por defecto


static void lcd_write_byte(uint8_t data);
static void lcd_send(uint8_t value, uint8_t mode_rs);
static void lcd_pulse_enable(uint8_t data);
static void lcd_cmd(uint8_t cmd);
static void lcd_data(uint8_t data);


static void i2c_write(uint8_t val) {
HAL_I2C_Master_Transmit(&hi2c1, I2C_LCD_ADDR, &val, 1, HAL_MAX_DELAY);
}


static void lcd_pulse_enable(uint8_t data) {
i2c_write(data | LCD_EN | bl_state);
HAL_Delay(1);
i2c_write((data & ~LCD_EN) | bl_state);
HAL_Delay(1);
}


static void lcd_write_nibble(uint8_t nibble, uint8_t mode_rs) {
// D7..D4 del LCD están en P7..P4 del PCF; por eso usamos (nibble & 0xF0)
uint8_t out = (nibble & 0xF0) | mode_rs; // RW=0 siempre (escritura)
i2c_write(out | bl_state);
lcd_pulse_enable(out);
}


static void lcd_send(uint8_t value, uint8_t mode_rs) {
lcd_write_nibble(value & 0xF0, mode_rs);
lcd_write_nibble((value << 4) & 0xF0, mode_rs);
}


static void lcd_cmd(uint8_t cmd) { lcd_send(cmd, 0); }
static void lcd_data(uint8_t data) { lcd_send(data, LCD_RS); }


void lcd_backlight(uint8_t on) {
bl_state = on ? LCD_BL : 0;
i2c_write(bl_state);
}


void lcd_clear(void) {
lcd_cmd(0x01); // clear
HAL_Delay(2); // >1.53 ms
}


void lcd_put_cur(uint8_t row, uint8_t col) {
uint8_t addr = (row == 0 ? 0x00 : 0x40) + col;
lcd_cmd(0x80 | addr);
}


void lcd_init(void) {
HAL_Delay(50); // >40 ms después de VCC sube


// Secuencia de arranque en 4‑bit
lcd_write_nibble(0x30, 0); HAL_Delay(5);
lcd_write_nibble(0x30, 0); HAL_Delay(1);
lcd_write_nibble(0x30, 0); HAL_Delay(1);
lcd_write_nibble(0x20, 0); HAL_Delay(1); // 4‑bit


// Función: 2 líneas, 5x8 dots
lcd_cmd(0x28);
// Display ON, cursor OFF, blink OFF
lcd_cmd(0x0C);
// Entry mode: incrementar, sin shift
lcd_cmd(0x06);
lcd_clear();
}


void lcd_write_char(char ch) { lcd_data((uint8_t)ch); }


void lcd_send_string(const char *str) {
while (*str) {
lcd_write_char(*str++);
}
}

