#ifndef I2C_LCD_H_
#define I2C_LCD_H_
#include "stm32f1xx_hal.h"        // incluye HAL general
#include "stm32f1xx_hal_i2c.h"


// Dirección 7‑bit del PCF8574T (típico 0x27). HAL usa dirección <<1.
#ifndef I2C_LCD_ADDR
#define I2C_LCD_ADDR (0x27 << 1)
#endif


// Mapa de pines del backpack más común:
// P0=RS, P1=RW, P2=EN, P3=BL, P4=D4, P5=D5, P6=D6, P7=D7
#define LCD_RS (1u<<0)
#define LCD_RW (1u<<1)
#define LCD_EN (1u<<2)
#define LCD_BL (1u<<3)


extern I2C_HandleTypeDef hi2c1; // creado por CubeMX


void lcd_init(void);
void lcd_clear(void);
void lcd_backlight(uint8_t on);
void lcd_put_cur(uint8_t row, uint8_t col);
void lcd_send_string(const char *str);
void lcd_write_char(char ch);


#endif /* I2C_LCD_H_ */
