#ifndef BLUE_TOOTH_H
#define BLUE_TOOTH_H

#include <Arduino.h>
/********************************************//**
 * \brief imported from MotorControl.c for use when communicating via bluetooth
 ***********************************************/
extern POWER_STATES currentPowerState;

int ledpin = 2;
int RxD = 0;
int TxD = 1;
char val[10];
int valIndex = 0;
int initialize = 0;
char x = 0;
char y = 0;
char z = 0;
char rotation = 0;

void bluetoothSetup()
{
  pinMode(ledpin, OUTPUT);  // pin 48 (on-board LED) as OUTPUT
  pinMode(RxD, INPUT);
  pinMode(TxD, OUTPUT);
  Serial.begin(9600);       // start serial communication at 9600bps
}

void bluetoothReceiveLoop() {

  if(Serial.available() )       // if data is available to read
  {
    val[valIndex] = Serial.read();         // read it and store it in 'val'

    valIndex++;
    if(valIndex == 1){
        if(val[0] != 0x10 && val[0] != 0x11 && val[0] != 0x12 && val[0] != 0x20 && val[0] != 0x21 && val[0] != 0x22 && val[0] != 0x44)
        {
            valIndex = 0;
        }
    }
  }

  if( valIndex == 0x3 && (val[0] == 0x10 || val[0] == 0x11 || val[0] == 0x12 || val[0] == 0x20 || val[0] == 0x21 || val[0] == 0x22))               // if 'H' was received
  {
    valIndex = 0;
    if(currentPowerState == OFF && val[0] == 0x10)
    {
        currentPowerState = IDLE;
    }else if(currentPowerState == IDLE && val[0] == 0x20)
    {
        currentPowerState = ON;
    }else if(val[0] == 0x10 || val[0] == 0x11)
    {
        x = val[1];
        y = val[2];
        //digitalWrite(ledpin, LOW);   // otherwise turn it OFF
    }else if(val[0] == 0x20 || val[0] == 0x21)
    {
        rotation = val[1];
        z = val[2];
        //digitalWrite(ledpin, HIGH);  // turn ON the LED
    }else if(val[0] == 0x12)//Finger lifted off xy
    {
        x = 0;
        y = 0;

    }else if( val[0] == 0x22)//finger lifted off z rotate
    {
        z = 0;
        rotation = 0;
    }

    //digitalWrite(ledpin, HIGH);  // turn ON the LED
  }
  else if(valIndex == 0x03 )
  {
      valIndex = 0;
      currentPowerState = OFF;
  }
  else
  {

  }
}



#endif // Blue_Tooth_H
