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
  //Listen to command and then instantly check what state we are in
  bluetoothReceiveLoop();
  //Here is where we check weather or not we should change out motor state (OFF,IDLE,ON)
  MotorStateMachine();
  //Next we need to decide how to manuever the quad copter
  ForegroundMotorDriver();

  //Next we go through the pid loop to set the speeds we want the motors at
  BackgroundMotorDriver();

  //Adjust the speed of the motors
  MotorControl();
}
