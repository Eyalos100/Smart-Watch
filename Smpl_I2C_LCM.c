#include <stdio.h>
#include <stdint.h>
#include "DrvSYS.h"
#include "DrvGPIO.h"
#include "Driver_I2C.h"
#include "LCM1602_I2C.h"               // <-- new


int32_t main (void)
{
    char TEXT1[16]="I Love youuuuuuu";

    UNLOCKREG();
    SYSCLK->PWRCON.XTL12M_EN=1;
    DrvSYS_Delay(5000);
    SYSCLK->CLKSEL0.HCLK_S=0;
    LOCKREG();


    DrvGPIO_InitFunction(E_FUNC_I2C0);  // Set I2C pins (same as before). :contentReference[oaicite:8]{index=8}
    I2C_Open(100000);                   // 100kHz bus. :contentReference[oaicite:9]{index=9}

    LCD_I2C_Init();
    LCD_I2C_PrintAt(0,0,"hi");
   
    while(1)
    {
			LCD_I2C_PrintAt(1,0, TEXT1);              // show x,y first
      DrvSYS_Delay(100000); // small refresh delay
    }
}
