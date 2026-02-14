#ifndef LCM1602_I2C_H
#define LCM1602_I2C_H

#include <stdint.h>

#ifndef I2C_ADDR
#define I2C_ADDR  0x27   // change if your board is different
#endif

void LCD_I2C_Init(void);
void LCD_I2C_Clear(void);
void LCD_I2C_Home(void);
void LCD_I2C_SetCursor(uint8_t row, uint8_t col); // row:0..1, col:0..15
void LCD_I2C_Print(const char *s);
void LCD_I2C_PrintAt(uint8_t row, uint8_t col, const char *s);
void LCD_I2C_Backlight(uint8_t on);

#endif
