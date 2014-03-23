#ifndef BLUE_TOOTH_H
#define BLUE_TOOTH_H

#include <Arduino.h>
/********************************************//**
 * \brief imported from MotorControl.c for use when communicating via bluetooth
 ***********************************************/
extern POWER_STATES currentPowerState;
extern POWER_STATES requestedPowerState;

int ledpin = 2;
int RxD = 0;
int TxD = 1;
char val[10];
int valIndex = 0;
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
        if(valIndex == 1)
        {
            if(val[0] != 0x10 && val[0] != 0x11 && val[0] != 0x12 && val[0] != 0x20 && val[0] != 0x21 && val[0] != 0x22 && val[0] != 0x44)
            {
                valIndex = 0;
            }
        }
        else if( valIndex == 0x3 && (val[0] == 0x10 || val[0] == 0x11 || val[0] == 0x12 || val[0] == 0x20 || val[0] == 0x21 || val[0] == 0x22))               // if 'H' was received
        {
            digitalWrite(ledpin, HIGH);
            valIndex = 0;
            //Should take these currentpowerstates out of the conditional. All of the transitions should be handled in motor state controller
            if(currentPowerState == OFF && val[0] == 0x10)
            {
                requestedPowerState = IDLE;
            }
            else if(currentPowerState == IDLE && val[0] == 0x20)
            {
                requestedPowerState = ON;
            }
            //Action down or action Move fo x,y
            else if(val[0] == 0x10 || val[0] == 0x11)
            {
                x = val[1];
                y = val[2];
            //digitalWrite(ledpin, LOW);   // otherwise turn it OFF
            }
            //Action down or Action move for rotation,z
            else if(val[0] == 0x20 || val[0] == 0x21)
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
            digitalWrite(ledpin, LOW);  // turn ON the LED
        }
        // we got a valid message but an invalid command so we turn off!
        //TODO need to be carefull for this, what if it happens when we're in mid flight? BOOOOOOOOOOOOOM!!!!!!!! CRASHHHH BURN!!!!!!!!!
        else if(valIndex == 0x03 )
        {
          valIndex = 0;
          requestedPowerState = OFF;
        }
        else
        {

        }

    }

}



#endif // Blue_Tooth_H
