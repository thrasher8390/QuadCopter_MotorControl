#ifndef accel_c
#define accel_c
//Add the SPI library so we can communicate with the ADXL345 sensor
#include <SPI.h>
#include <Arduino.h>

#define DEBUG 3

//Assign the Chip Select signal to pin 52.
#define accelCS  10
#define accelInt1  8
//This is a list of some of the registers available on the ADXL345.
//To learn more about these and the rest of the registers on the ADXL345, read the datasheet!
#define POWER_CTL  0x2D	//Power Control Register
#define DATA_FORMAT 0x31
#define INT_ENABLE 0x2E
#define FIFO_CTL 0x38
#define INT_MAP 0x2F
#define BW_RATE 0x2C
#define DATAX0  0x32	//X-Axis Data 0
#define DATAX1  0x33	//X-Axis Data 1
#define DATAY0  0x34	//Y-Axis Data 0
#define DATAY1  0x35	//Y-Axis Data 1
#define DATAZ0  0x36	//Z-Axis Data 0
#define DATAZ1  0x37	//Z-Axis Data 1

double sensitivity = 0;

//This buffer will hold values read from the ADXL345 registers.
char values[6];
char uid[1];

int16_t xyz[3];
int16_t xyzCal[3];
//These variables will be used to hold the x,y and z axis accelerometer values.

byte buff;

void readRegister(char registerAddress, int numBytes, char * start){
  //Since we're performing a read operation, the most significant bit of the register address should be set.
  char address = 0x80 | registerAddress;
  //If we're doing a multi-byte read, bit 6 needs to be set as well.
  if(numBytes > 1)address = address | 0x40;

  //Set the Chip select pin low to start an SPI packet.
  digitalWrite(accelCS, LOW);
  //Transfer the starting register address that needs to be read.
  SPI.transfer(address);
  //Continue to read registers until we've read the number specified, storing the results to the input buffer.
  for(int i = 0; i<numBytes ; i++)
  {
      start[i] = SPI.transfer(0x00);
  }
  //Set the Chips Select pin high to end the SPI packet.
  digitalWrite(accelCS, HIGH);
}

void writeRegister(byte thisRegister, byte thisValue)
{
    //Delays will need to be changed.
  //delay(10);
  digitalWrite(accelCS, LOW);
  SPI.transfer(thisRegister);
  SPI.transfer(thisValue);
  digitalWrite(accelCS, HIGH);
  //delay(10);

}
void accelSetup(){
    //Initiate an SPI communication instance.
    SPI.begin();
    SPI.setClockDivider(SPI_CLOCK_DIV64);
    //Configure the SPI connection for the ADXL345.
    SPI.setDataMode(SPI_MODE3);

    // initalize the  data ready and chip select pins:
    pinMode(accelInt1, INPUT);
    pinMode(accelCS, OUTPUT);
    digitalWrite(accelCS, HIGH);
    //Before communication starts, the Chip Select pin needs to be set high.
    delay(5);
    writeRegister(DATA_FORMAT, 0X01);// +/-4g
    sensitivity = 4;
    delay(5);
    writeRegister(POWER_CTL, 0X08);
    delay(5);
	writeRegister(INT_ENABLE, 0x80);	//Activate the 'Data Ready' Interrupt
	delay(5);
	writeRegister(INT_MAP, 0x7F);			//Sent the Data Ready Interrupt to the INT1 pin
	delay(5);
	writeRegister(BW_RATE, 0x0F);			//Set Output Rate to 3200 Hz
    delay(5);
    writeRegister(FIFO_CTL, 0X00);			//BYPASS
    delay(5);
}

void readAccel(){
    //while(!accelInt1){};
    int16_t x,y,z;
    readRegister(DATAX0, 6, &values[0]);


    //The ADXL345 gives 10-bit acceleration values, but they are stored as bytes (8-bits). To get the full value, two bytes must be combined for each axis.
    //The X value is stored in values[0] and values[1].
    xyz[0] = ((((int)values[1]) & 0x03) <<8) | values[0];
    //The Y value is stored in values[2] and values[3].
    xyz[1] = ((((int)values[3]) & 0x03) <<8) | values[2];
    //The Z value is stored in values[4] and values[5].
    xyz[2] = ((((int)values[5]) & 0x03) <<8) | values[4];
    //Print the results to the terminal.

    if(DEBUG > 2)
    {
        readRegister(0x00, 1, &uid[0]);
        Serial.print(uid[0] & 0xFF,HEX);
        Serial.println();
        Serial.print(xyz[0], DEC);
        Serial.print(',');
        Serial.print(xyz[1], DEC);
        Serial.print(',');
        Serial.println(xyz[2], DEC);
    }
}
void accelCalibration()
{
    //A whole second of sampling at 3200Hz
    int numSamples = 720;//1 second

    int x1 =0, y1=0, z1=0;
    for(int i = 0 ; i < numSamples  ; i++)
    {
        readAccel();
        //The ADXL345 gives 10-bit acceleration values, but they are stored as bytes (8-bits). To get the full value, two bytes must be combined for each axis.
        //The X value is stored in values[0] and values[1].
        x1 += xyz[0];
        //The Y value is stored in values[2] and values[3].
        y1 += xyz[1];
        //The Z value is stored in values[4] and values[5].
        z1 += xyz[2];
    }

    xyzCal[0] = x1/numSamples;
    xyzCal[1] = y1/numSamples;
    xyzCal[2] = z1/numSamples;
    //Might be able to set offset register on ADXL345 instead of holding the Cal values
    if(DEBUG > 0)
    {
        Serial.println();
        Serial.print(xyzCal[0], DEC);
        Serial.print(',');
        Serial.print(xyzCal[1], DEC);
        Serial.print(',');
        Serial.println(xyzCal[2], DEC);
    }
    /*
    //Cal x
    if(xyz[0] <= 0)
    {
        writeRegister(0x1E, (xyzCal[0])&0xFF);
    }
    else
    {
        writeRegister(0x1E, ((xyzCal[0])&0xFF)|0x80);
    }
    //Call y
    if(xyz[1] <= 0)
    {
        writeRegister(0x1F, xyzCal[1]&0xFF);
    }
    else
    {
        writeRegister(0x1F, (xyzCal[1]&0xFF)|0x80);
    }
    if(xyz[1] <= 0)
    {
        writeRegister(0x20, xyzCal[1]&0xFF);
    }
    else
    {
        writeRegister(0x20, ((xyzCal[1]&0xFF)|0x80)-z0g);
    }
    int z0g = 1024/sensitivity;
    writeRegister(0x20, xyzCal[2]-z0g);
    */
}

#endif
