#include <Arduino.h>
#include <Accelerometer.c>
#include <MotorControl.c>
#include <BlueTooth.c>



int beginTime;
int runTimeCounter;

void setup()
{

    //Create a serial connection to display the data on the terminal.
    Serial.begin(9600);
    //setup UART tx/rx
    bluetoothSetup();

    digitalWrite(ledpin, HIGH);

    motorSetup();

    //setup SPI 4 wire
    accelSetup();
    accelCalibration(); //Calibration

    digitalWrite(ledpin, LOW);
    beginTime = millis();
}

void loop()
{
    runTimeCounter++;
    if(runTimeCounter == 10000)
    {
        runTimeCounter = 0;
        int time = millis();
        Serial.println(time - beginTime, DEC);
        Serial.println((time - beginTime)/10000, DEC);
        beginTime = time;
    }
  //Listen to command and then instantly check what state we are in
  bluetoothReceiveLoop();
  //Here is where we check weather or not we should change out motor state (OFF,IDLE,ON)
  MotorStateMachine();

  if(GetCurrentPowerState() == ON)
  {
      digitalWrite(ledpin,HIGH);
      controller();
      //Next we need to decide how to manuever the quad copter
      ForegroundMotorDriver();
      //Next we go through the pid loop to set the speeds we want the motors at
      BackgroundMotorDriver();
      digitalWrite(ledpin,LOW);
  }

  //Adjust the speed of the motors
  MotorControl();
}
