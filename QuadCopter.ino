#include <Arduino.h>
#include <Accelerometer.c>
#include <MotorControl.c>
#include <BlueTooth.c>





void setup()
{
    //Create a serial connection to display the data on the terminal.
    Serial.begin(9600);

    motorSetup();
    //setup SPI 4 wire
    accelSetup();
    accelCalibration(); //Calibration
    //setup UART tx/rx
    bluetoothSetup();


}

void loop()
{
  //pulse_LED();
  //fade_LED(motorArray[0]);
  motorControl();
  bluetoothReceiveLoop();

}
