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




void fade_LED(int ledPin)
{
  for(int fadeValue = 0 ; fadeValue <= max_speed; fadeValue +=5) {
    // sets the value (range from 0 to 255):
    analogWrite(ledPin, fadeValue);
    // wait for 30 milliseconds to see the dimming effect
    delay(30);
  }

  // fade out from max to min in increments of 5 points:
  for(int fadeValue = max_speed ; fadeValue >= 0; fadeValue -=5) {
    // sets the value (range from 0 to 255):
    analogWrite(ledPin, fadeValue);
    // wait for 30 milliseconds to see the dimming effect
    delay(30);
  }
}

