#ifndef ACCELEROMETER_H_INCLUDED
#define ACCELEROMETER_H_INCLUDED

#include <SPI.h>
#include <Arduino.h>

//Add the SPI library so we can communicate with the ADXL345 sensor

//Assign the Chip Select signal to pin 52.
#define accelCS  13
#define accelSDO  12
#define accelSCL  10
#define accelInt1  8
//This is a list of some of the registers available on the ADXL345.
//To learn more about these and the rest of the registers on the ADXL345, read the datasheet!
#define POWER_CTL  0x2D	//Power Control Register
#define DATA_FORMAT  0x31
#define DATAX0  0x32	//X-Axis Data 0
#define DATAX1  0x33	//X-Axis Data 1
#define DATAY0  0x34	//Y-Axis Data 0
#define DATAY1  0x35	//Y-Axis Data 1
#define DATAZ0  0x36	//Z-Axis Data 0
#define DATAZ1  0x37	//Z-Axis Data 1


//This buffer will hold values read from the ADXL345 registers.
char values[10];
//These variables will be used to hold the x,y and z axis accelerometer values.
int16_t x,y,z;
byte buff;
void accelSetup(){}
void readAccel(){}
void writeRegister(byte thisRegister, byte thisValue){}
unsigned int readRegister(byte thisRegister, int bytesToRead ){}

#endif // ACCELEROMETER_H_INCLUDED
