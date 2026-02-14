#include <stdio.h>
#include "DrvSYS.h"
#include "Driver_I2C.h"
#include "LCM1602_I2C.h"

// ------------- Configuration -------------
#define I2C_ADDR   0x27   // 0x27 << 1 = 0x4E (change if your module differs)
#define LCD_DELAY_US   200

#define LCD_RS  0x01
#define LCD_RW  0x02
#define LCD_EN  0x04
#define LCD_BL  0x08
//------------------------------------------

static uint8_t g_lcd_backlight = LCD_BL;

// -------------------------------------------------
// Helper: Write one byte to the PCF8574 (no register index)
// -------------------------------------------------
static void LCD_WriteRaw(uint8_t data)
{
     I2C_WriteRawByte(I2C_ADDR, data);
}

// -------------------------------------------------
// Pulse the enable pin to latch data into LCD
// -------------------------------------------------
static void LCD_Pulse(uint8_t data)
{
    LCD_WriteRaw(data | LCD_EN | g_lcd_backlight);
    DrvSYS_Delay(LCD_DELAY_US);
    LCD_WriteRaw((data & ~LCD_EN) | g_lcd_backlight);
    DrvSYS_Delay(LCD_DELAY_US);
}

// -------------------------------------------------
// Send 4-bit nibble (upper bits of value) with RS control
// -------------------------------------------------
static void LCD_Write4(uint8_t nibble, uint8_t rs)
{
    uint8_t data = (nibble & 0xF0) | (rs ? LCD_RS : 0x00) | g_lcd_backlight;
    LCD_WriteRaw(data);
    LCD_Pulse(data);
}

// -------------------------------------------------
// Send a full byte (two nibbles)
// -------------------------------------------------
static void LCD_Send(uint8_t value, uint8_t rs)
{
    LCD_Write4(value & 0xF0, rs);
    LCD_Write4((value << 4) & 0xF0, rs);
}

static void LCD_Cmd(uint8_t cmd)   { LCD_Send(cmd, 0); }
static void LCD_Data(uint8_t data) { LCD_Send(data, 1); }

// -------------------------------------------------
// Public Functions
// -------------------------------------------------
void LCD_I2C_Backlight(uint8_t on)
{
    g_lcd_backlight = on ? LCD_BL : 0x00;
    LCD_WriteRaw(g_lcd_backlight);
}

void LCD_I2C_Init(void)
{
    DrvSYS_Delay(15000); // wait >15 ms after power up

    // Initialize 4-bit mode sequence
    LCD_Write4(0x30, 0);
    DrvSYS_Delay(5000);
    LCD_Write4(0x30, 0);
    DrvSYS_Delay(200);
    LCD_Write4(0x30, 0);
    DrvSYS_Delay(200);
    LCD_Write4(0x20, 0);
    DrvSYS_Delay(200);

    // Function set: 4-bit, 2-line, 5x8 font
    LCD_Cmd(0x28);
    // Display off
    LCD_Cmd(0x08);
    // Clear
    LCD_Cmd(0x01);
    DrvSYS_Delay(2000);
    // Entry mode set
    LCD_Cmd(0x06);
    // Display on, cursor off, blink off
    LCD_Cmd(0x0C);

    LCD_I2C_Backlight(1);
}

void LCD_I2C_Clear(void)
{
    LCD_Cmd(0x01);
    DrvSYS_Delay(2000);
}

void LCD_I2C_Home(void)
{
    LCD_Cmd(0x02);
    DrvSYS_Delay(2000);
}

void LCD_I2C_SetCursor(uint8_t row, uint8_t col)
{
    uint8_t addr = (row == 0 ? 0x00 : 0x40) + (col & 0x0F);
    LCD_Cmd(0x80 | addr);
}

void LCD_I2C_Print(const char *s)
{
    while (*s) LCD_Data((uint8_t)*s++);
}

void LCD_I2C_PrintAt(uint8_t row, uint8_t col, const char *s)
{
    LCD_I2C_SetCursor(row, col);
    LCD_I2C_Print(s);
}

