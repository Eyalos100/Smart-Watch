#include <stdio.h>
#include "DrvSYS.h"
#include "DrvGPIO.h"
#include "Driver_I2C.h"
#include "LCM1602_I2C.h"  
#include "MPU6050.h"

#define ENABLE_I2C0
#if defined(ENABLE_I2C0)
#define I2C_PORT_NO   0
#define I2C_CTRL_SIG  (__IO uint32_t *)&I2C0->I2CON
#define I2C_DATAPORT  I2C0->I2CDAT
#define I2C_FLAG_SI   I2C0->I2CON.SI
#define I2C_FLAG_STO  I2C0->I2CON.STO
#endif

#define I2C_STA              0x20
#define I2C_STO              0x10
#define I2C_SI               0x08
#define I2C_AA               0x04


void I2C_Open(uint32_t u32BusClock)
{	
    uint32_t divider; 
	  SystemCoreClock = DrvSYS_GetHCLKFreq();	
    divider = (uint32_t) (((SystemCoreClock*10)/(u32BusClock * 4) + 5) / 10 - 1);    
    if (divider < 4) divider = 4;
    
    if (I2C_PORT_NO==I2C_PORT0) {
        SYSCLK->APBCLK.I2C0_EN = 1;
        SYS->IPRSTC2.I2C0_RST = 1;
        SYS->IPRSTC2.I2C0_RST = 0;  
        I2C0->I2CON.ENS1 = 1;
        I2C0->I2CLK = divider;
    } else if (I2C_PORT_NO==I2C_PORT1) {			
        SYSCLK->APBCLK.I2C1_EN = 1;
        SYS->IPRSTC2.I2C1_RST = 1;
        SYS->IPRSTC2.I2C1_RST = 0;
        I2C1->I2CON.ENS1 = 1;
        I2C1->I2CLK = divider;			
    }
}

void I2C_Close(void)
{
    if (I2C_PORT_NO==I2C_PORT0) {
        I2C0->I2CON.ENS1 = 0;
        SYS->IPRSTC2.I2C0_RST = 1;
        SYS->IPRSTC2.I2C0_RST = 0;
        SYSCLK->APBCLK.I2C0_EN = 0;			
    } else if (I2C_PORT_NO==I2C_PORT1) {
        I2C1->I2CON.ENS1 = 0;
        SYS->IPRSTC2.I2C1_RST = 1;
        SYS->IPRSTC2.I2C1_RST = 0;
        SYSCLK->APBCLK.I2C1_EN = 0;
    }
}

void I2C_Ctrl(uint8_t start, uint8_t stop, uint8_t intFlag, uint8_t ack)
{
    uint32_t Reg = 0;        
    if (start)   Reg |= I2C_STA;
    if (stop)    Reg |= I2C_STO;
    if (intFlag) Reg |= I2C_SI;
    if (ack)     Reg |= I2C_AA;

		*(I2C_CTRL_SIG) = (*(I2C_CTRL_SIG) & ~0x3C) | Reg;
}

void I2C_WriteRawByte(uint8_t addr7, uint8_t data)
{
    // START
    I2C_Ctrl(1, 0, 1, 0);
    while (I2C_FLAG_SI == 0);

    // ????? ?????? (8-???): addr7<<1 | 0
    I2C_DATAPORT = (addr7 << 1);
    I2C_Ctrl(0, 0, 1, 0);
    while (I2C_FLAG_SI == 0);

    // ??? ??????? (?-PCF8574 ?? ?? ?? ?????)
    I2C_DATAPORT = data;
    I2C_Ctrl(0, 0, 1, 0);
    while (I2C_FLAG_SI == 0);

    // STOP
    I2C_Ctrl(0, 1, 1, 0);
    while (I2C_FLAG_STO);
}

void I2C_Write(uint8_t INDEX,uint8_t DATA)
{
  //send i2c start
	I2C_Ctrl(1, 0, 1, 0);	  //set start
	while (I2C_FLAG_SI==0);	//poll si flag

	//send to Write port
	I2C_DATAPORT = I2C_ADDR;
  I2C_Ctrl(0, 0, 1, 0);   //clr si flag
  while(I2C_FLAG_SI==0);	//poll si flag

	//send write address
	I2C_DATAPORT = INDEX;	
	I2C_Ctrl(0, 0, 1, 0);   //clr si 	
	while(I2C_FLAG_SI==0);	//poll si flag

	//send write data
	I2C_DATAPORT = DATA;	
	I2C_Ctrl(0, 0, 1, 0);   //clr si 	
	while(I2C_FLAG_SI==0);	//poll si flag

  //send i2c stop
	I2C_Ctrl(0, 1, 1, 0);   //send stop	
	while(I2C_FLAG_STO); 	
}

uint8_t I2C_Read(uint8_t INDEX)
{
  uint8_t TEMP;
	//send i2c start
  I2C_Ctrl(1, 0, 1, 0);	 	//set start
	while (I2C_FLAG_SI==0);	//poll si flag
	 
  //send to Write port
	I2C_DATAPORT = I2C_ADDR;
  I2C_Ctrl(0, 0, 1, 0);	  //clr si
  while(I2C_FLAG_SI==0);  //poll si flag

	//send INDEX
	I2C_DATAPORT = INDEX;
  I2C_Ctrl(0, 0, 1, 0);	  //clr si
  while(I2C_FLAG_SI==0);	//poll si flag

	//send i2c start
  I2C_Ctrl( 1, 0, 1, 0);	//set start
	while (I2C_FLAG_SI==0);	//poll si flag

 	//send to Read port
	I2C_DATAPORT = (I2C_ADDR+1);
  I2C_Ctrl(0, 0, 1, 0);	  //clr si
  while(I2C_FLAG_SI==0);	//poll si flag
		
	//receive data
	I2C_DATAPORT = 0xFF;
	I2C_Ctrl(0, 0, 1, 0);   //clr si	
	while(I2C_FLAG_SI==0);	//poll si flag
	TEMP = I2C_DATAPORT;

	//send i2c stop
 	I2C_Ctrl(0, 1, 1, 0);   //clr si and set stop
	while(I2C_FLAG_STO);

	return TEMP;
}
