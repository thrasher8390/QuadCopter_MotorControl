#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H



#include <Arduino.h>
#include <Accelerometer.c>
#include <BlueTooth.c>
#include <Servo.h>

#define max_speed 180
#define min_speed 68
double hoverMaxDif = 32; //max_speed - max_speed_hover
#define max_speed_hover 140
#define min_speed_hover 75


void motorX(double);
void motorY(double);
void motorZ(double);
void controller();
//Motor arrays
//0 = -x/left (black arm)
//1 = x/right (red arm)
//2 = -y/backwards (black arm)
//3 = y/forward (red arm)
Servo motorArray[4];
double motorSpeedCorrection[4];
double motorSpeedPrevious[4];
double motorSpeedHover[4];//The 5th value is an equlibrium value 0-4 get adjusted in motor x and motor y
double x_hover_adjust = 0;
double y_hover_adjust = 0;

int accelAvg[3];
int numAvg = 2;
//increase is the proportional term
double xChange_d = 0;
double yChange_d = 0;
double xChange_i = 0;
double yChange_i = 0;

/*
double ku = .5;
double pu = 16;
//Coeficients
double kp = ku*.6;  //proportional
double ki = 2*kp/pu;//1/64;//.01;  //integral
double kd = kp*pu/8;//1/64;//.05;  //derivative
*/
double kp = 1/5;  //proportional
double ki = kp/4;//1/64;//.01;  //integral
double kd = kp*.9;//1/64;//.05;  //derivative
//determines how quickly motorSpeedCorrection returns to motorSpeedHover
double stabalizeFactor = (max_speed- max_speed_hover);//20;
double correction = 1.41732;
//Calibration update
//int updateCT;
//int updateInterval;

void motorSetup()
{
    motorArray[0].attach(3);
    motorArray[1].attach(5);
    motorArray[2].attach(6);
    motorArray[3].attach(9);
    //pinMode(4, OUTPUT);      // sets the digital pin as output



    for(int i = 0; i<4;i++)
    {
        motorSpeedHover[i] = min_speed - 30 ;
        motorSpeedCorrection[i] = motorSpeedHover[i];
        motorSpeedPrevious[i] = 0xFFFF;
    }
    for(int i = 0 ; i < 4; i++) {
      // sets the value (range from 0 to 255):
        motorArray[i].write(motorSpeedHover[i]);
    }
}
void motorControl()
{
    //delay(10);
/*
readAccel();
int xChange = (xyz[0]- xyzCal[0])&0xFFFFFFFE;
int xDiff = xChange -xChange_prev;
xChange_prev = xChange;
delay(100);
if(xDiff>0)
digitalWrite(ledpin, HIGH);   // ready to fly!
else
    digitalWrite(ledpin, LOW);   // ready to fly!
*/
    if(initialize == 0 )
    {
//reset STATE
        for(int i = 0; i < 4; i++)
        {
            motorSpeedHover[i] = min_speed - 30 ;
            motorSpeedCorrection[i] = motorSpeedHover[i];
            motorSpeedPrevious[i] = 0xFFFF;
        }
        //digitalWrite(ledpin, HIGH);   // ready to fly!
    }else if(initialize == 1)
    {


        //Tests the motors by turning on -x then x then -y the y
        /*
        motorArray[0].write((int)80);
        delay(3000);
        motorArray[0].write((int)0);
        motorArray[1].write((int)80);
        delay(3000);
        motorArray[1].write((int)0);
        motorArray[2].write((int)80);
        delay(3000);
        motorArray[2].write((int)0);
        motorArray[3].write((int)80);
        delay(3000);
        motorArray[4].write((int)0);
*/

        //Motors initialized (they start spinning)
        digitalWrite(ledpin, LOW);   // otherwise turn it OFF
     for(int i = 0; i < 4; i++)
        {
            motorSpeedHover[i] = min_speed;
            motorSpeedCorrection[i] = motorSpeedHover[i];
        }
        accelCalibration(); //Calibration

    }else
    {
        controller();

        //todo try averaging 3 accell readings???
/*
        for(int i = 0 ; i <numAvg; i++)
        {
            readAccel();
            for(int j = 0 ; j<3; j++)
            {
                accelAvg[j] += xyz[j];
            }
        }

        //set xyz[] to the accelAvg and then reset accelAvg back to zero
        for(int i = 0; i <3; i++)
        {
            xyz[i] = accelAvg[i]/numAvg;
            accelAvg[i] = 0;
        }

*/
        readAccel();


        //if(x == 0)
        //{
            double xChange = xyz[0]- xyzCal[0];
            motorX(xChange);
        //}
        //if(y == 0)
        //{
            double yChange = xyz[1] - xyzCal[1];
            motorY(yChange);
        //}
        //if (z == 0)
        //{
            double zChange = xyz[2] - xyzCal[2];
            //motorZ(zChange);
        //}
    }

    for(int i = 0 ; i < 4; i++) {
      // sets the value (range from 0 to 180):

      if((int)motorSpeedCorrection[i] != (int)motorSpeedPrevious[i])
      {
          motorArray[i].write((int)motorSpeedCorrection[i]);
          motorSpeedPrevious[i] = motorSpeedCorrection[i];
          //digitalWrite(2,HIGH);
      }else
      {
          //digitalWrite(2, LOW);
      }
    }



    for(int i = 0 ; i < 4 && initialize > 1 ; i++ )
    {
        //motorSpeedCorrection can not be greater than hoverMaxDif away from hoverSpeedHover
        //TODO

        if(motorSpeedCorrection[i] - motorSpeedHover[i] > hoverMaxDif)
            motorSpeedCorrection[i] = motorSpeedHover[i] + hoverMaxDif ;

        if(motorSpeedCorrection[i] - motorSpeedHover[i] < -hoverMaxDif)
            motorSpeedCorrection[i] = motorSpeedHover[i] - hoverMaxDif ;


        // try to get motor speed back to hover speed

        //todo Should below be commented out?
        double dif = motorSpeedCorrection[i] - motorSpeedHover[i];
        motorSpeedCorrection[i] -= (dif/stabalizeFactor);
        //motorSpeedCorrection[i] = motorSpeedHover[i];
    }

    //updateCT++;

}

#define motorxy_hoveradjust_thresh 10
double motorxy_hover_increment = numAvg/200*motorxy_hoveradjust_thresh;

void motorX(double xAccel)
{
    //zeroing out hover speed when quad copter is stable
    if((int)xAccel == 0)
    {

        //TODO do I reset integral portion to 0

        if(x_hover_adjust > 0)
            x_hover_adjust--;
        else if(x_hover_adjust<0)
            x_hover_adjust++;
    }else if(xAccel>0)
    {
       x_hover_adjust++;
    }
    else if (xAccel<0)
    {
        x_hover_adjust--;
    }

    //Calculating derivative Current acc-last
    int xDiff = xAccel - xChange_d;
    xChange_d = xAccel;   //remember this accel value for next iteration

    xChange_i += xAccel;  //increase integral portion

    double propTemp = kp*xAccel;
    double intTemp = ki* xChange_i;
    double dervTemp = kd*xDiff;
    double totalAdjustment = (propTemp+intTemp+dervTemp);

    //Determines different motor power distrubution. Some motors are weaker than others.
    double offset = (motorSpeedHover[0]-motorSpeedHover[1])/2;
//At the end of every control/stabalize loop motorSpeedCorrection is set to motorSpeedHover. This way when control is called. motorspeedcorrection is changed and then stabalize makes necessary adjustments prior to turning copter.
    motorSpeedCorrection[0] = motorSpeedCorrection[0] + (totalAdjustment + offset);
    motorSpeedCorrection[1] = motorSpeedCorrection[1] - (totalAdjustment + offset);

    //Make sure all motor values are between min_speed and max_speed_hover
    if(motorSpeedCorrection[1]<min_speed)
        motorSpeedCorrection[1] = min_speed;
    if(motorSpeedCorrection[0]> max_speed)
        motorSpeedCorrection[0] = max_speed;
    if(motorSpeedCorrection[0]<min_speed)
        motorSpeedCorrection[0] = min_speed;
    if(motorSpeedCorrection[1]> max_speed)
        motorSpeedCorrection[1] = max_speed;

    //x_hover_adjust++;


    if(x_hover_adjust > motorxy_hoveradjust_thresh)
    {
        motorSpeedHover[0]+= motorxy_hover_increment;
        motorSpeedHover[1]-= motorxy_hover_increment;
        digitalWrite(ledpin, HIGH);  // turn ON the LED
        x_hover_adjust = 0;
    }else if(x_hover_adjust < -motorxy_hoveradjust_thresh)
    {
        motorSpeedHover[0]-= motorxy_hover_increment;
        motorSpeedHover[1]+= motorxy_hover_increment;
        digitalWrite(ledpin, LOW);  // turn ON the LED
        x_hover_adjust = 0;
    }

    return;
}

void motorY(double yAccel)
{
    //TODO should ignore hover adjust depending on controler setting
    if((int)yAccel == 0)
    {

        if(y_hover_adjust > 0)
            y_hover_adjust--;
        else if(y_hover_adjust<0)
            y_hover_adjust++;
    }else if(yAccel>0)
    {
       y_hover_adjust++;
    }
    else if (yAccel<0)
    {
        y_hover_adjust--;
    }
     //Calculating derivative Current acc-last
    int yDiff = yAccel - yChange_d;
    yChange_d = yAccel;   //remember this accel value for next iteration

    yChange_i += yAccel;  //increase integral portion

    double propTemp = kp*yAccel;
    double intTemp = ki* yChange_i;
    double dervTemp = kd*yDiff;
    double totalAdjustment = (propTemp+intTemp+dervTemp);

    //Determines different motor power distrubution. Some motors are weaker than others.
    double offset = (motorSpeedHover[2]-motorSpeedHover[3])/2;

    motorSpeedCorrection[2] = motorSpeedCorrection[2] + (totalAdjustment + offset);
    motorSpeedCorrection[3] = motorSpeedCorrection[3] - (totalAdjustment + offset);

//Keep motor speeds in bounds
    if(motorSpeedCorrection[3]<min_speed)
        motorSpeedCorrection[3] = min_speed;
    if(motorSpeedCorrection[2]> max_speed)
        motorSpeedCorrection[2] = max_speed;
    if(motorSpeedCorrection[2]<min_speed)
        motorSpeedCorrection[2] = min_speed;
    if(motorSpeedCorrection[3]> max_speed)
        motorSpeedCorrection[3] = max_speed;


//Hover speed adjust
    if(y_hover_adjust > motorxy_hoveradjust_thresh)
    {
        motorSpeedHover[2]+= motorxy_hover_increment;
        motorSpeedHover[3]-= motorxy_hover_increment;
        digitalWrite(ledpin, HIGH);  // turn ON the LED
        y_hover_adjust = 0;
    }else if(y_hover_adjust < -motorxy_hoveradjust_thresh)
    {
        motorSpeedHover[2]-= motorxy_hover_increment;
        motorSpeedHover[3]+= motorxy_hover_increment;
        digitalWrite(ledpin, LOW);  // turn ON the LED
        y_hover_adjust = 0;
    }
    return;
}

void motorZ(double increase)
{
    //int speed = 0;
    for(int i = 0 ; i < 4 ;  i++)
    {
        //motorSpeedCorrection[i] += increase;//stabalizeFactor*increase;

    }
    //TODO: adjust hover settings
    if(increase > 0)
    {

    }else
    {
    }

    return;
}



void forwards()
{
    //increase y
    motorSpeedCorrection[3] -= y;
    motorSpeedCorrection[2] += y;
    return;
}
void backwards()
{
    //increase -y
    motorSpeedCorrection[3] -= y;
    motorSpeedCorrection[2] += y;
    return;
}
void right()
{
    //increase x
    motorSpeedCorrection[1] -= x;
    motorSpeedCorrection[0] += x;
    return;
}
void left()
{
    //increase -x
    motorSpeedCorrection[1] -= x;
    motorSpeedCorrection[0] += x;
    return;
}
void up()
{
    int z_corrected = ((z*correction)*(max_speed - min_speed)/180 )+min_speed; //maximum input is 127 but max speed is 180

    //digitalWrite(ledpin, HIGH);  // turn ON the LED
     for(int i = 0; i<4 ; i++)
    {
        motorSpeedCorrection[i] = z_corrected;

        if(motorSpeedHover[i] < z_corrected && (motorSpeedHover[i])<max_speed_hover)
        {
            motorSpeedHover[i]++;
            //z = 0;
        }
        else
            motorSpeedHover[i]--;

        //if(motorSpeedHover[i] < min_speed_hover)
            //initialize = 0;
    }
    return;
}
void down()
{
    //decrease all
    //digitalWrite(ledpin, LOW);  // turn ON the LED
    for(int i = 0; i<4 ; i++)
    {
        motorSpeedCorrection[i] = 0;
        motorSpeedHover[i]--;
        //z=0;
    }
}
void rotateLeft()
{
    //CounterClockWise
    //increase y / -y
    return;
}
void rotateRight()
{
    //ClockWise
    //increase x / - x
    return;

}

void controller()
{
    if(x > 0)
    {
        right();
        //digitalWrite(ledpin, HIGH);  // turn ON the LED
    }else if (x < 0)
    {
        left();
        //digitalWrite(ledpin, LOW);  // turn ON the LED
    }
    if(y > 0)
    {
        forwards();
        //digitalWrite(ledpin, HIGH);  // turn ON the LED
    }else if(y < 0)
    {
        backwards();
        //digitalWrite(ledpin, LOW);  // turn ON the LED
    }
    if(z > 0)
    {
        up();
        //digitalWrite(ledpin, HIGH);  // turn ON the LED
    }else if(z < 0){
        down();
        //digitalWrite(ledpin, LOW);  // turn ON the LED
    }
    if(rotation > 0)
    {
        rotateLeft();
        //digitalWrite(ledpin, HIGH);  // turn ON the LED
    }else if(rotation < 0)
    {
        rotateRight();
        //digitalWrite(ledpin, LOW);  // turn ON the LED
    }

    //Used to make sure speed controls are accepted values
    for(int i = 0; i<4 ; i++)
    {

        if(motorSpeedCorrection[i] < min_speed)
            motorSpeedCorrection[i] = min_speed;
        else if( motorSpeedCorrection[i] > max_speed)
            motorSpeedCorrection[i] = max_speed;

        //Hover is changed in order to stabalize the quad copter
        if(motorSpeedHover[i] < min_speed_hover)
            motorSpeedHover[i] = min_speed_hover;
        else if( motorSpeedHover[i] > max_speed_hover)
            motorSpeedHover[i] = max_speed_hover;
    }
}
#endif // MOTOR_CONTROL_H
