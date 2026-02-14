typedef enum 
{
    I2C_PORT0 = 0,
    I2C_PORT1 = 1
} E_I2C_PORT;

extern void I2C_Open(uint32_t u32BusClock);

extern void I2C_Close(void);

extern void I2C_Write(uint8_t INDEX,uint8_t DATA);

extern uint8_t I2C_Read(uint8_t INDEX);

void I2C_WriteRawByte(uint8_t addr7, uint8_t data);
