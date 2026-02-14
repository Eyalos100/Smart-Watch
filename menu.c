// =============================================================================================
// HARDWARE MAPPING (NUC140 Pin Assignments)
// =============================================================================================
// heart
// pin1 Vss:						(to Gnd)
// pin2 Vdd: to +5V					(to +5V)
// pin3 signal: 77 (ADC Channel 6)

// joystick
// x : 76 (ADC Channel 5)
// y : 75 (ADC Channel 4)
// B : 50 (GPIO Pin GPB9)

// temp
// signal : 78 (ADC Channel 7)

// HC05 Bluetooth module
// pin4 : TXD    to NUC140 UART0-RX (GPB0) 32
// pin5 : RXD    to NUC140 UART0-TX (GPB1) 33

// IC2 pins
// SCL : 11
// SDA : 12

#include <stdio.h>
#include <stdint.h>
#include <string.h> // Added for string comparison (strcmp) and buffer clearing (memset)
#include "NUC1xx.h"
#include "Driver\DrvGPIO.h"
#include "Driver\DrvSYS.h"
#include "Driver\DrvUART.h"
#include <math.h> // Needed for sqrt() function used in fall_chk()
#include "DrvADC.h"
#include "Driver_I2C.h"
#include "LCM1602_I2C.h" 
#include "MPU6050.h"

// =============================================================================================
// UART globals
// =============================================================================================
volatile uint8_t comRbuf[16]; 	// Buffer for incoming Bluetooth data
volatile uint16_t comRbytes = 0; // Number of bytes received
volatile uint16_t comRhead 	= 0;
volatile uint16_t comRtail 	= 0;
char cmd[4] = {0}; 				// Command buffer (size 4 for 3 chars + '\0')

// =============================================================================================
// Clock globals
// =============================================================================================
static uint32_t currentMillis=0;
unsigned long previousMillis = 0;
int maxvalue = 0;
int BPM = 80;
int count = 0;
// =============================================================================================
// Fall Check globals
// =============================================================================================
// Define thresholds for the two-stage fall detection algorithm
// Low-G Phase (Freefall): Sensor measures close to 0g
#define FREEFALL_THRESHOLD_G (0.3) 
// High-G Phase (Impact): Sensor measures a sharp spike in deceleration
// Set lower than 2.5g in previous versions to potentially increase sensitivity
#define IMPACT_THRESHOLD_G (1) 
// Normal Reset Threshold: If magnitude returns to near 1g without impact, reset detection
#define NORMAL_G_HIGH (1.2)
#define NORMAL_G_LOW (0.8)

// =======================================================================
// Initialization functions
// =======================================================================
void Init_MPU6050()
{
	I2C_Write(MPU6050_PWR_MGMT_1, 0x00);	// CLL_SEL=0: internal 8MHz, TEMP_DIS=0, SLEEP=0 
	I2C_Write(MPU6050_SMPLRT_DIV, 0x07);  // Gyro output sample rate = Gyro Output Rate/(1+SMPLRT_DIV)
	I2C_Write(MPU6050_CONFIG, 0x06);      // set TEMP_OUT_L, DLPF=2 (Fs=1KHz)
	I2C_Write(MPU6050_GYRO_CONFIG, 0x18); // bit[4:3] 0=+-250d/s,1=+-500d/s,2=+-1000d/s,3=+-2000d/s
	I2C_Write(MPU6050_ACCEL_CONFIG, 0x01);// bit[4:3] 0=+-2g,1=+-4g,2=+-8g,3=+-16g, ACC_HPF=On (5Hz)
}

void Init_GPIO(void)
{
  DrvGPIO_Open(E_GPA, 0, E_IO_OUTPUT);
  DrvGPIO_Open(E_GPA, 1, E_IO_OUTPUT);
  DrvGPIO_Open(E_GPA, 2, E_IO_OUTPUT);
  DrvGPIO_ClrBit(E_GPA, 0);
  DrvGPIO_ClrBit(E_GPA, 1);
  DrvGPIO_ClrBit(E_GPA, 2);
}

void UART_INT_HANDLE(void)
{
	while(UART0->ISR.RDA_IF==1) 
	{
		comRbuf[comRbytes]=UART0->DATA;
		comRbytes++;		
		if (comRbytes==15) {	
			sprintf(cmd,"%s",comRbuf);
		  comRbytes=0;
		}
	}
}

// =======================================================================
// FALL DETECTION FUNCTION (Two-Stage)
// =======================================================================
/**
 * @brief Implements a two-stage fall detection algorithm (Low-G followed by High-G).
 * @return int Fall_Flag: 1 if a fall (low-G -> high-G sequence) is confirmed, 0 otherwise.
 */
int fall_chk(){
	float accX, accY, accZ;
	uint8_t tmpL, tmpH;
	int16_t tmp;
	float magnitude;
    
    // State machine tracker: 0=Normal, 1=Low-G Detected (awaiting impact)
    static uint8_t fall_phase = 0; 

	// Read X-Axis
	tmpL = I2C_Read(MPU6050_ACCEL_XOUT_L); 
    tmpH = I2C_Read(MPU6050_ACCEL_XOUT_H); 
	tmp = (tmpH<<8)+tmpL;
	accX = (float) tmp/32768 * 2.0; // Convert raw reading to g-force 

	// Read Y-Axis
	tmpL = I2C_Read(MPU6050_ACCEL_YOUT_L); 
    tmpH = I2C_Read(MPU6050_ACCEL_YOUT_H); 
	tmp = (tmpH<<8)+tmpL;
	accY = (float) tmp/32768 * 2.0; 

	// Read Z-Axis
	tmpL = I2C_Read(MPU6050_ACCEL_ZOUT_L); 
    tmpH = I2C_Read(MPU6050_ACCEL_ZOUT_H); 
	tmp = (tmpH<<8)+tmpL;
	accZ = (float) tmp/32768 * 2.0;
	
	// Calculate the magnitude (vector sum of accelerations)
	magnitude = sqrt(accX*accX + accY*accY + accZ*accZ);

    if (fall_phase == 0) {
        // Phase 0: Normal operation - Looking for Low-G event
        if (magnitude < FREEFALL_THRESHOLD_G) {
            fall_phase = 1; // Low-G detected. Move to Phase 1.
        }
    } else if (fall_phase == 1) {
        // Phase 1: Low-G detected - Looking for High-G Impact
        
        if (magnitude > IMPACT_THRESHOLD_G) {
            // High-G Impact Confirmed! Fall Detected.
            fall_phase = 0; // Reset state machine
            return 1;       // Return 1 to signal confirmed fall
            
        } else if (magnitude > NORMAL_G_LOW && magnitude < NORMAL_G_HIGH) { 
            // The magnitude returned to near 1g without a high-G impact (false alarm). 
            fall_phase = 0; // Reset state.
        }
    }
	
	return 0; // No confirmed fall
}

// =======================================================================
// Temperature screen function
// =======================================================================
void tmp_chk(int16_t value){
	char Menu[11]= "Temperature";
	char val[2];
	sprintf(val, "%d", value); // Convert integer temp value to string
	LCD_I2C_PrintAt(0,2,Menu); // Display Menu title at row 0, col 2
	LCD_I2C_PrintAt(1,0,val);  // Display actual value at row 1, col 0
}

// =======================================================================
// Heart rate screen function
// =======================================================================
void heart_chk(int16_t value){
	char Menu[15];
	char val[4];
	sprintf(Menu, "Heart BPM");
	LCD_I2C_PrintAt(0,0,Menu); // Display Menu title at row 0, col 2
	sprintf(val, "%d", value); // Convert integer temp value to string
	LCD_I2C_PrintAt(1,0,val);  // Display actual value at row 1, col 0
}

// =======================================================================
// HEART RATE CONVERSION FUNCTION (Simple Peak Detection)
// =======================================================================

void convert_raw_to_bpm(uint16_t current_raw_value) 
{
    if (count == 4)
    {
      BPM = (count*60000)/(currentMillis-previousMillis);
      previousMillis = currentMillis;
      count=0;
    }
    if (current_raw_value >= maxvalue)
    {
      maxvalue = current_raw_value;
    }
    else if (maxvalue - current_raw_value >= 120)  
    {
      count++;
      maxvalue=0;
    }
}

// =======================================================================
// Home screen function
// =======================================================================
void home(uint8_t hour, uint8_t min, uint8_t sec){ // Added sec as argument
	char Menu[16];
	char options[16]= "Move left/right"; // Instructions for navigation
	
	// Format the time as HH:MM:SS
	sprintf(Menu, "Time: %02d:%02d:%02d", hour, min, sec);
	
	LCD_I2C_PrintAt(0,0,Menu); // Display the active clock
	LCD_I2C_PrintAt(1,0,options);
}

// =======================================================================
// Emergency text screen function
// =======================================================================
void emergency_text()
{
	uint8_t  SW; // Joystick button
	char Menu[16]= "To send SOS";
	LCD_I2C_PrintAt(0,0,Menu);
	sprintf(Menu,"press button");
	LCD_I2C_PrintAt(1,0,Menu);
	SW = DrvGPIO_GetBit(E_GPB,9);
	if (SW)
	{
		sprintf(Menu,"please help");
		DrvUART_Write(UART_PORT0,Menu,11);
	}
	
}

// =======================================================================
// Timer functions
// =======================================================================
// Timer0 initialize to tick every 1ms
void InitTIMER0(void)
{
	/* Step 1. Enable and Select Timer clock source */          
	SYSCLK->CLKSEL1.TMR0_S = 0;	//Select 12Mhz for Timer0 clock source 
	SYSCLK->APBCLK.TMR0_EN =1;	//Enable Timer0 clock source

	/* Step 2. Select Operation mode */	
	TIMER0->TCSR.MODE = 2;		//Select once mode for operation mode

	/* Step 3. Select Time out period = (Period of timer clock input) * (8-bit Prescale + 1) * (24-bit TCMP)*/
	TIMER0->TCSR.PRESCALE = 11;	// Set Prescale [0~255]
	TIMER0->TCMPR = 1000;		// Set TCMPR [0~16777215]
	//Timeout period = (1 / 12MHz) * ( 11 + 1 ) * 1,000 = 1 ms

	/* Step 4. Enable interrupt */
	TIMER0->TCSR.IE = 1;
	TIMER0->TISR.TIF = 1;		//Write 1 to clear for safty		
	NVIC_EnableIRQ(TMR0_IRQn);	//Enable Timer0 Interrupt

	/* Step 5. Enable Timer module */
	TIMER0->TCSR.CRST = 1;	//Reset up counter
	TIMER0->TCSR.CEN = 1;		//Enable Timer0

//	TIMER0->TCSR.TDR_EN=1;		// Enable TDR function
}

void TMR0_IRQHandler(void) // Timer0 interrupt subroutine 
{
	currentMillis++;
	TIMER0->TISR.TIF =1; 	   
}

// =======================================================================
// Main function
// =======================================================================
int32_t main (void)
{
	int i;
	char TEXT[15];
	float temp_value;		// Variable to store filtered temperature ADC value
	uint16_t heart_value;  // Variable to store filtered heart ADC value
	uint8_t rState=0;		// Main state variable (1=Home, 2=Temp, 3=Heart, 4=Emergency message, 5=Fall)
	uint8_t rFlag=0;		// Debounce flag for joystick movement (0=ready, 1=wait for center)
	uint16_t Vx, Vy;		// Joystick X and Y raw ADC values
	STR_UART_T sParam;
	uint8_t hour=0;//clock hour
	uint8_t min=0;//clock minute
	uint8_t sec=0;//clock second
	uint8_t set_state = 0; // 0=Set Hour, 1=Set Minute
	uint32_t loop_counter = 0; //Counter for heart rate time tracking
	uint8_t  SW=0; // Joystick button
 	
	 // === System Initialization ===
	UNLOCKREG(); // Unlock system registers for clock configuration
	DrvSYS_Open(48000000);
	LOCKREG(); // Lock system registers
	
	DrvGPIO_InitFunction(E_FUNC_UART0);	// Set UART pins
	InitTIMER0();
	
// UART initialization for HC-05
	DrvGPIO_InitFunction(E_FUNC_UART0);	// Set UART pins (GPB0/GPB1) for alternate function

	/* UART Configuration */
		sParam.u32BaudRate = 9600;
		sParam.u8cDataBits = DRVUART_DATABITS_8;
		sParam.u8cStopBits = DRVUART_STOPBITS_1;
		sParam.u8cParity = DRVUART_PARITY_NONE;
		sParam.u8cRxTriggerLevel = DRVUART_FIFO_1BYTES;
		
	/* Set UART Configuration and Enable Interrupt */
	if(DrvUART_Open(UART_PORT0,&sParam) != E_SUCCESS);
	DrvUART_EnableInt(UART_PORT0, DRVUART_RDAINT, UART_INT_HANDLE);


	DrvADC_Open(ADC_SINGLE_END, ADC_SINGLE_CYCLE_OP, 0xF0, INTERNAL_HCLK, 1);// initialize ADC (channels 4, 5, 6, 7 used)
	Init_GPIO();
							 					 
  DrvGPIO_Open(E_GPB, 9, E_IO_INPUT); // SW		
	
	// I2C initialization for LCD and MPU6050
	DrvGPIO_InitFunction(E_FUNC_I2C0);
	I2C_Open(100000);// 100kHz bus frequency
	LCD_I2C_Init();
	Init_MPU6050();// Initialize MPU6050 Accelerometer
	
	while(1){
		// 1. ADC Readings and Processing and GPIO
		DrvADC_StartConvert();// start A/D conversion
		while(DrvADC_IsConversionDone()==FALSE); // wait till conversion is done
		
		Vx = ADC->ADDR[5].RSLT & 0xFFF;// Read Joystick X (Channel 5)
		Vy = ADC->ADDR[4].RSLT & 0xFFF;// Read Joystick Y (Channel 4)
		heart_value = ADC->ADDR[6].RSLT & 0xFFF;// Read heart Sensor (Channel 6)
		temp_value=ADC->ADDR[7].RSLT & 0xFFF;// Read Temp Sensor (Channel 7)
		temp_value=temp_value*4.7;// 4.7V VCC
		temp_value=temp_value/4096;//12 bit ADC
		temp_value-=0.4;//400mV offset
		temp_value=temp_value*51;//1/19.6m[V/C]		
		SW = DrvGPIO_GetBit(E_GPB,9);
		
		// 2. Time Update Logic
		loop_counter++; // Increment the main loop counter for time tracking
		if (loop_counter >= 10000) loop_counter = 0; // Reset to prevent overflow
		
		// Update clock time every ~10 loops (~1 second)
		if (rState >= 1) { // Only increment time after setup (rState > 0)
			
			// Increment Seconds every 10 iterations (~1 second)
			if ((loop_counter % 10) == 0) {
			    sec++;
			    if (sec >= 60) {
			        sec = 0;
			        min++;
			        if (min >= 60) {
			            min = 0;
			            hour++;
			            if (hour >= 24) {
			                hour = 0;
			            }
			        }
			    }
			}
		}
		
		// 3. Heart Rate Conversion
		convert_raw_to_bpm(heart_value);
		
		// 4. Fall Detection Check (Highest Priority)
		if (fall_chk() == 1)
			{
				rState = 5;// Change state to Fall Alert Screen (Highest Priority)
			}
		// 5. Joystick Navigation Logic (Only if no fall is detected)
		else
			{
				if(rState>0)
					{
						if(Vy>3500&&rFlag==0)// Joystick Right Movement
							{
								if(rState>=4)// Wrap around from last state (4) to first state (1)
									{
										rState=1;
										rFlag=1;// Set debounce flag (rFlag=1)
										LCD_I2C_Clear();
									}
								else
									{
										rState++;// Move to next state
										rFlag=1;
										LCD_I2C_Clear();
									}
							}
						
						if(Vy<1800&&Vy>0&&rFlag==0)// Joystick Left Movement
							{
								if(rState<=1)// Wrap around from first state (1) to last state (4)
									{
										rState=4;
										rFlag=1;// Set debounce flag (rFlag=1)
										LCD_I2C_Clear();
									}
								else
									{
										rState--;// Move to previous state
										rFlag=1;
										LCD_I2C_Clear();
									}	
							}						
						
						if (rFlag)// Debounce Reset Logic: Only reset rFlag (rFlag=0) when joystick is centered (Vy is idle)
								{
									if(Vy<3000&&Vy>2300)
										rFlag=0;// Reset debounce flag, allowing new input
								}
						}
				}
			
	// 6. Main State Machine (Switch-Case)	
		switch (rState)
			{
				case 0:// Hour and Minute set
					if (set_state == 0) { // Setting Hour
						LCD_I2C_PrintAt(0,0,"Set Hour (0-23)");
						if(Vx>3500 && hour<23){
							hour++;
							DrvSYS_Delay(80000); // Debounce
						}
						if(Vx<1800&&Vx>0&&hour>0){
							hour--;
							DrvSYS_Delay(80000); // Debounce
						}
						sprintf(TEXT,"Hour: %02d",hour);
						LCD_I2C_PrintAt(1,0,TEXT);
						
						if(SW) {
							set_state = 1; // Move to set minute
							LCD_I2C_Clear();
							DrvSYS_Delay(100000); // Debounce
						}
					} else { // Setting Minute
						LCD_I2C_PrintAt(0,0,"Set Minute (0-59)");
						if(Vx>3500 && min<59){
							min++;
							DrvSYS_Delay(100000); // Debounce
						}
						if(Vx<1800&&Vx>0&&min>0){
							min--;
							DrvSYS_Delay(100000); // Debounce
						}
						sprintf(TEXT,"Min: %02d",min);
						LCD_I2C_PrintAt(1,0,TEXT);
						
						if(SW) {
							sec = 0; // Set seconds to zero before starting the clock
							rState = 1; // Transition to Home Screen
							LCD_I2C_Clear();
							DrvSYS_Delay(100000); // Debounce
						}
					}
					break;

						
				
				case 1:// Home Screen
					home(hour, min, sec); // Passed current time for display
					break;

				case 2:// Temperature Screen
						tmp_chk(temp_value);
						break;

				case 3:// Heart Rate Screen
					heart_chk(BPM); // Passed the converted BPM value
					break;
				case 4:// Emergency Message Screen
					emergency_text();
					break;
				case 5:// Fall Alert Screen (Entered only if fall_chk() returns 1)
					LCD_I2C_Clear();
					LCD_I2C_PrintAt(0,0,"Fall Detected!");
					sprintf(TEXT,"Fall Detected!");
					DrvUART_Write(UART_PORT0,TEXT,14);
					for(i=0;i<3;i++)//delay for 1 second
						DrvSYS_Delay(335000);
					rState = 1; // Transition back to home screen after alert display
					break;
				
		}
			DrvSYS_Delay(35000); // Approximately 35ms delay for loop stability
	}
}
	
// =============================================================================================
// Previous versions
// =============================================================================================	
	
////heart
//// pin1 Vss:						(to Gnd)
//// pin2 Vdd: to +5V					(to +5V)
//// pin3 signal: 77
//
////joystick
//// x : 76
//// y : 75
//// B : 50
//
////temp
//// signal : 78
//
//// HC05 Bluetooth module
//// pin1 : KEY   N.C
//// pin2 : VCC   to Vcc +5V
//// pin3 : GND   to GND
//// pin4 : TXD   to NUC140 UART0-RX (GPB0) 32
//// pin5 : RXD   to NUC140 UART0-TX (GPB1) 33
//
//#include <stdio.h>
//#include <stdint.h>
//#include "NUC1xx.h"
//#include "Driver\DrvGPIO.h"
//#include "Driver\DrvSYS.h"
//#include "Driver\DrvUART.h"
//#include <math.h>
//#include "DrvADC.h"
//#include "Driver_I2C.h"
//#include "LCM1602_I2C.h" 
//#include "MPU6050.h"
//volatile uint8_t comRbuf[16];
//volatile uint16_t comRbytes = 0;
//volatile uint16_t comRhead 	= 0;
//volatile uint16_t comRtail 	= 0;
//char cmd[3];
//
//// Define thresholds for the two-stage fall detection algorithm
//// Low-G Phase (Freefall): Sensor measures close to 0g
//#define FREEFALL_THRESHOLD_G (0.3) 
//// High-G Phase (Impact): Sensor measures a sharp spike in deceleration
//#define IMPACT_THRESHOLD_G (1.5) 
//// Normal Reset Threshold: If magnitude returns to near 1g without impact, reset detection
//#define NORMAL_G_HIGH (1.2)
//#define NORMAL_G_LOW (0.8)
//
//void Init_MPU6050()
//{
//	I2C_Write(MPU6050_PWR_MGMT_1, 0x00);	// CLL_SEL=0: internal 8MHz, TEMP_DIS=0, SLEEP=0 
//	I2C_Write(MPU6050_SMPLRT_DIV, 0x07);  // Gyro output sample rate = Gyro Output Rate/(1+SMPLRT_DIV)
//	I2C_Write(MPU6050_CONFIG, 0x06);      // set TEMP_OUT_L, DLPF=2 (Fs=1KHz)
//	I2C_Write(MPU6050_GYRO_CONFIG, 0x18); // bit[4:3] 0=+-250d/s,1=+-500d/s,2=+-1000d/s,3=+-2000d/s
//	I2C_Write(MPU6050_ACCEL_CONFIG, 0x01);// bit[4:3] 0=+-2g,1=+-4g,2=+-8g,3=+-16g, ACC_HPF=On (5Hz)
//}
//
//void Init_GPIO(void)
//{
//  DrvGPIO_Open(E_GPA, 0, E_IO_OUTPUT);
//  DrvGPIO_Open(E_GPA, 1, E_IO_OUTPUT);
//  DrvGPIO_Open(E_GPA, 2, E_IO_OUTPUT);
//  DrvGPIO_ClrBit(E_GPA, 0);
//  DrvGPIO_ClrBit(E_GPA, 1);
//  DrvGPIO_ClrBit(E_GPA, 2);
//}
//
//void UART_INT_HANDLE(void)
//{
//	while(UART0->ISR.RDA_IF==1) 
//	{
//		comRbuf[comRbytes]=UART0->DATA;
//		comRbytes++;		
//		if (comRbytes==3) {	
//			sprintf(cmd,"%s",comRbuf);
//		  comRbytes=0;
//		}
//	}
//}
//
//int fall_chk(){
//	float accX, accY, accZ;
//	uint8_t tmpL, tmpH;
//	int16_t tmp;
//	float magnitude;
//    
//    // State machine tracker: 0=Normal, 1=Low-G Detected (awaiting impact)
//    static uint8_t fall_phase = 0; 
//
//	// Read X-Axis
//	tmpL = I2C_Read(MPU6050_ACCEL_XOUT_L); 
//    tmpH = I2C_Read(MPU6050_ACCEL_XOUT_H); 
//	tmp = (tmpH<<8)+tmpL;
//	accX = (float) tmp/32768 * 2.0; 
//
//	// Read Y-Axis
//	tmpL = I2C_Read(MPU6050_ACCEL_YOUT_L); 
//    tmpH = I2C_Read(MPU6050_ACCEL_YOUT_H); 
//	tmp = (tmpH<<8)+tmpL;
//	accY = (float) tmp/32768 * 2.0; 
//
//	// Read Z-Axis
//	tmpL = I2C_Read(MPU6050_ACCEL_ZOUT_L); 
//    tmpH = I2C_Read(MPU6050_ACCEL_ZOUT_H); 
//	tmp = (tmpH<<8)+tmpL;
//	accZ = (float) tmp/32768 * 2.0;
//	
//	// Calculate the magnitude (vector sum of accelerations)
//	magnitude = sqrt(accX*accX + accY*accY + accZ*accZ);
//
//    if (fall_phase == 0) {
//        // Phase 0: Normal operation - Looking for Low-G event
//        if (magnitude < FREEFALL_THRESHOLD_G) {
//            fall_phase = 1; // Low-G detected. Move to Phase 1.
//        }
//    } else if (fall_phase == 1) {
//        // Phase 1: Low-G detected - Looking for High-G Impact
//        
//        if (magnitude > IMPACT_THRESHOLD_G) {
//            // High-G Impact Confirmed! Fall Detected.
//            fall_phase = 0; // Reset state machine
//            return 1;       // Return 1 to signal confirmed fall
//            
//        } else if (magnitude > NORMAL_G_LOW && magnitude < NORMAL_G_HIGH) { 
//            // The magnitude returned to near 1g without a high-G impact. 
//            // This was likely a false alarm (e.g., jumping up/down). Reset state.
//            fall_phase = 0; 
//        }
//        // If it's still outside the normal and impact range, stay in Phase 1 (waiting for impact).
//    }
//	
//	return 0; // No confirmed fall
//}
//
//void tmp_chk(int32_t value){
//	char  Menu[11]= "Temperature";
//	char val[2];
//	sprintf(val, "%d", value);
//	LCD_I2C_PrintAt(0,2,Menu);
//	LCD_I2C_PrintAt(1,0,val);
//}
//
//void heart_chk(){
//	char  Menu[16]= "Heart rate [BPM]";
//	LCD_I2C_PrintAt(0,0,Menu);
//}
//
//void home(){
//	char  Menu[16]= "Welcome   [Hour]";
//	char options[16]= "Move left/right";
//	LCD_I2C_PrintAt(0,0,Menu);
//	LCD_I2C_PrintAt(1,0,options);
//}
//	
//
//int32_t main (void)
//{
//	int32_t temp_value;
//	uint8_t rState=1;
//	uint8_t rFlag=0;
//  uint16_t Vx, Vy;
//  uint8_t  SW;
//	char val[4];
//  char TEXT[16];	
//	STR_UART_T sParam;
// 	UNLOCKREG();
//	SYSCLK->PWRCON.XTL12M_EN = 1; // enable external clock (12MHz)
//	SYSCLK->CLKSEL0.HCLK_S = 0;	  // select external clock (12MHz)
//	LOCKREG();
//	
//	DrvGPIO_InitFunction(E_FUNC_UART0);	// Set UART pins
//
//	
///*UART Setting */
//  sParam.u32BaudRate 		  = 9600;                                                                                                                                     
//  sParam.u8cDataBits 		  = DRVUART_DATABITS_8;
//  sParam.u8cStopBits 		  = DRVUART_STOPBITS_1;
//  sParam.u8cParity 		    = DRVUART_PARITY_NONE;
//  sParam.u8cRxTriggerLevel= DRVUART_FIFO_1BYTES;
//		
//	/* Set UART Configuration */
// 	if(DrvUART_Open(UART_PORT0,&sParam) != E_SUCCESS);
//	DrvUART_EnableInt(UART_PORT0, DRVUART_RDAINT, UART_INT_HANDLE);
//
//
//	DrvADC_Open(ADC_SINGLE_END, ADC_SINGLE_CYCLE_OP, 0xF0, INTERNAL_HCLK, 1);// initialize ADC
//	Init_GPIO();
//							 					 
//  DrvGPIO_Open(E_GPB, 9, E_IO_INPUT); // SW		
//	
//	DrvGPIO_InitFunction(E_FUNC_I2C0);  // Set I2C pins (same as before). :contentReference[oaicite:8]{index=8}
//	I2C_Open(100000);                   // 100kHz bus. :contentReference[oaicite:9]{index=9}
//  LCD_I2C_Init();
//	Init_MPU6050();                     // Initialize MPU6050
//	
//	while(1){
//		DrvADC_StartConvert();// start A/D conversion
//		while(DrvADC_IsConversionDone()==FALSE); // wait till conversion is done
//		
//		Vx = ADC->ADDR[5].RSLT & 0xFFF;
//		Vy = ADC->ADDR[4].RSLT & 0xFFF;
//		temp_value=ADC->ADDR[7].RSLT & 0xFFF; 	// input 12-bit ADC value
//		temp_value=(temp_value)*460/4096;
//		if (fall_chk() == 1)
//			{
//				rState = 5;
//			}
//		else
//			{
//				if(Vy>3500&&rFlag==0)
//					{
//						if(rState>=4)
//							{
//								rState=1;
//								rFlag=1;
//								LCD_I2C_Clear();
//							}
//						else
//							{
//								rState++;
//								rFlag=1;
//								LCD_I2C_Clear();
//							}
//					}
//				
//				if(Vy<1800&&Vy>0&&rFlag==0)
//					{
//						if(rState<=1)
//							{
//								rState=4;
//								rFlag=1;
//								LCD_I2C_Clear();
//							}
//						else
//							{
//								rState--;
//								rFlag=1;
//								LCD_I2C_Clear();
//							}
//					}
//					
//				if (rFlag)
//						{
//							if(Vy<3000&&Vy>2300)
//								rFlag=0;
//						}
//				}
//		switch (rState) {
//        case 1:
//            home();
//            break;
//            
//        case 2:
//						tmp_chk(temp_value);
//            break;
//            
//        case 3:
//					heart_chk();
//            break;
//				case 4:
//					break;
//				case 5:
//					LCD_I2C_Clear();
//					
//					LCD_I2C_PrintAt(0,0,"Fall");
//					DrvSYS_Delay(5000000);
//					break;
//				
//				
//			}
//		}
//	}
	
	
	
//	while(1)
//	{
//	  DrvADC_StartConvert();// start A/D conversion
//	  Vx = ADC->ADDR[5].RSLT;
//		Vy = ADC->ADDR[4].RSLT;
//		SW = DrvGPIO_GetBit(E_GPB,9);
//		LCD_I2C_PrintAt(0,0,Menu);
//		if((Vy>3500)){
//			while(1){
//				heart_chk();
//				SW = DrvGPIO_GetBit(E_GPB,9);
//				if(SW)
//					break;
//				}
//			}
//		if(Vy<1800&&Vy>0)
//			{
//				while(1)
//					{
//						tmp_chk();
//						SW = DrvGPIO_GetBit(E_GPB,9);
//						if(SW)
//							break;
//					}
//			}
//	}
//}



