// I2C functions for DCT HSK go here.
// 
// The following are I2C ports used on the TM4C123GXL launchpad for the DCT HSK:
// Pins    |    (5,6)      |      (9,10)     |    (23,24)
// I2C     | (SCL2,SDA2)   |    (SCL1,SDA1)  |  (SCL3,SDA3)
// Hardware| ADC_pressure  |    Cathode HV   |  Potential HV
#define PART_TM4C123GH6PM // This picks out the register values we need from the header files in the .cpp file.
// Slave address for I2C writes go here
#define DAC_slave_CATHODE 32
#include <stdint.h>
// ******************************
// I2C specific functions
// ******************************
// Opens I2C port 0 which are pins PB2 and PB3
void InitI2C0(void);
// Opens port of choice (0,3)
void InitI2C(const int port);
// Sends I2C N bytes to slave address on port of choice
uint32_t I2CSendNbytes(const int base, uint8_t slave_addr, uint8_t num_of_args, ...);
// Reads slave address on port of choice
uint32_t I2CReceive(uint8_t base, uint8_t slave_addr); 
