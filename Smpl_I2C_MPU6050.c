//
// Smpl_I2C_MPU6050_acc : 3-axis Accelerometer
//
// MPU6050 : 3-axis Gyroscope + 3-axis accelerometer + temperature
// Interface: I2C
// pin1: Vcc to Vcc (+5V)
// pin2: Gnd to Gnd
// pin3: SCL to I2C0_SCL/GPA9
// pin4: SDA to I2C0_SDA/GPA8
// pin5: XDA -- N.C.
// pin6: XCL -- N.C.
// pin7: AD0 -- N.C.
// pin8: INT -- N.C.

#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include "DrvSYS.h"
#include "DrvGPIO.h"
#include "LCD_Driver.h"
#include "Driver_I2C.h"
#include "MPU6050.h"

void Init_MPU6050()
{
	I2C_Write(MPU6050_PWR_MGMT_1, 0x00);	// CLL_SEL=0: internal 8MHz, TEMP_DIS=0, SLEEP=0 
	I2C_Write(MPU6050_SMPLRT_DIV, 0x07);  // Gyro output sample rate = Gyro Output Rate/(1+SMPLRT_DIV)
	I2C_Write(MPU6050_CONFIG, 0x06);      // set TEMP_OUT_L, DLPF=2 (Fs=1KHz)
	I2C_Write(MPU6050_GYRO_CONFIG, 0x18); // bit[4:3] 0=+-250d/s,1=+-500d/s,2=+-1000d/s,3=+-2000d/s
	I2C_Write(MPU6050_ACCEL_CONFIG, 0x01);// bit[4:3] 0=+-2g,1=+-4g,2=+-8g,3=+-16g, ACC_HPF=On (5Hz)
}

int32_t main (void)
{
	char TEXT1[16], TEXT2[16], TEXT3[16];
  uint8_t tmpL, tmpH;
  int16_t tmp;
	float accX, accY, accZ;

	UNLOCKREG();
	SYSCLK->PWRCON.XTL12M_EN=1;
	DrvSYS_Delay(5000);					// Waiting for 12M Xtal stalble
	SYSCLK->CLKSEL0.HCLK_S=0;
	LOCKREG();

	Initial_panel(); 
	clr_all_panel();                
  print_lcd(0,"MPU6050 Acc +-2g");

	DrvGPIO_InitFunction(E_FUNC_I2C0);  // set I2C0 pins 
	I2C_Open(50000);	                  // set I2C0 to 50KHz
	Init_MPU6050();                     // Initialize MPU6050
	
	while(1)
	{									  
    tmpL = I2C_Read(MPU6050_ACCEL_XOUT_L); // read Accelerometer X_Low  value
    tmpH = I2C_Read(MPU6050_ACCEL_XOUT_H); // read Accelerometer X_High value
		tmp = (tmpH<<8)+tmpL;
		accX = (float) tmp/32768 *2;

    tmpL = I2C_Read(MPU6050_ACCEL_YOUT_L); // read Accelerometer Y_Low  value
    tmpH = I2C_Read(MPU6050_ACCEL_YOUT_H); // read Accelerometer Y_High value
 		tmp = (tmpH<<8)+tmpL;
		accY = (float) tmp/32768 *2;

    tmpL = I2C_Read(MPU6050_ACCEL_ZOUT_L); // read Accelerometer Z_Low  value
    tmpH = I2C_Read(MPU6050_ACCEL_ZOUT_H); // read Accelerometer Z_High value
 		tmp = (tmpH<<8)+tmpL;
		accZ = (float) tmp/32768 *2;
		
    // print to LCD
		sprintf(TEXT1,"accX: %f", accX); print_lcd(1,TEXT1);
		sprintf(TEXT2,"accY: %f", accY); print_lcd(2,TEXT2);
		sprintf(TEXT3,"accZ: %f", accZ); print_lcd(3,TEXT3);
	}
}
