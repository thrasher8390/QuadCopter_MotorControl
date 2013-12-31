#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H



#include <Arduino.h>
#include <Accelerometer.c>
//#include <BlueTooth.c>
#include <Servo.h>

#define max_speed 150
#define min_speed 68
#define max_speed_hover 120
#define min_speed_hover 75
#define ACCEL_SENSITIVITY_BUFFER 0x100

extern int ledpin;
extern int initialize;
extern char x;
extern char y;
extern char z;
double hoverMaxDif = (max_speed - max_speed_hover);

static void motorX(double);//stabilization procedure for x-axis
static void motorY(double);//stabilization procedure for y-axis
static void motorZ(double);//stabilization procedure for z-axis
static void controller(void);//takes input from bluetooth and decides how to change motor speeds
static void motorBounds(void);//keeps the speed of the motors within their limits
static void oscilateMotors(void); //used for debugging
static double absoluteValue(double);

//Motor arrays
//0 = -x/left (black arm)
//1 = x/right (red arm)
//2 = -y/backwards (black arm)
//3 = y/forward (red arm)
Servo motorArray[4];
double motorSpeedCorrection[4];
double motorSpeedPrevious[4];
double motorSpeedHover[4];


double x_hover_adjust = 0;
double y_hover_adjust = 0;

//int accelAvg[3];
//int numAvg = 2;
double motorxy_hoveradjust_thresh = 1024;
double motorxy_hover_increment = .125;

//increase is the proportional term
double xChange_d = 0;
double yChange_d = 0;
double xChange_i = 0;
double yChange_i = 0;

double kp = 1/256;  //proportional
double ki = kp/64;  //integral
double kd = 0;//kp*.90; //derivative

//determines how quickly motorSpeedCorrection returns to motorSpeedHover
double stabalizeFactor = (max_speed - max_speed_hover);

#define CONTROLLER_DUTY 3
int controllerDutyCt = 0;
//used to correct the speed of up and down for z
double  correction  =   1.41732;
/** An enum type.
*  The documentation block cannot be put after the enum!
*/
enum POWER_STATES
        {  OFF,      /**< \brief details the differen states the motors can be in */
           IDLE,     /**< \brief details the differen states the motors can be in */
           ON        /**< \brief details the differen states the motors can be in */
        };
/********************************************//**
 *  \brief Used to keep the state of which the motors are in
 ***********************************************/
POWER_STATES currentPowerState = OFF;

void motorSetup()
{
    motorArray[0].attach(3);
    motorArray[1].attach(5);
    motorArray[2].attach(6);
    motorArray[3].attach(9);

    for(int i = 0; i<4;i++)
    {
        motorSpeedHover[i] = min_speed - 30 ;
        motorSpeedCorrection[i] = motorSpeedHover[i];
        motorSpeedPrevious[i] = 0xFFFF;
        motorArray[i].write(motorSpeedHover[i]);
    }
}

void motorControl()
{
    if(currentPowerState == OFF )
    {
        //reset STATE
        for(int i = 0; i < 4; i++)
        {
            motorSpeedHover[i] = min_speed - 30 ;
            motorSpeedCorrection[i] = motorSpeedHover[i];
            motorSpeedPrevious[i] = 0xFFFF;
        }
        //digitalWrite(ledpin, HIGH);   // ready to fly!
    }
    else if(currentPowerState == IDLE)
    {
        //oscilateMotors();


        //Motors initialized (they start spinning)
        digitalWrite(ledpin, LOW);   // otherwise turn it OFF
     for(int i = 0; i < 4; i++)
        {
            motorSpeedHover[i] = min_speed;
            motorSpeedCorrection[i] = motorSpeedHover[i];
        }
        //accelCalibration(); //Calibration

    }
    else if(currentPowerState == ON)
    {


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
        //controller every 10th time
        if(controllerDutyCt >= CONTROLLER_DUTY)
        {
            controller();
            controllerDutyCt = 0;
        }
        controllerDutyCt++;
        readAccel();
        double xChange = xyz[0] - xyzCal[0];
        motorX(xChange);

        readAccel();
        double yChange = xyz[1] - xyzCal[1];
        motorY(yChange);
        //readAccel()();
        //double zChange = xyz[2] - xyzCal[2];
            //motorZ(zChange);
        //}

        motorBounds();
    }


    for(int i = 0 ; i < 4; i++) {
      // sets the value (range from 0 to 180):

      if(motorSpeedCorrection[i] != motorSpeedPrevious[i])
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

    }
}



void motorX(double xAccel)
{
    if(absoluteValue(xAccel) < ACCEL_SENSITIVITY_BUFFER)
    {
        return;
    }
    //zeroing out hover speed when quad copter is stable
    else if((int)xAccel == 0)
    {
        //xChange_i = 0;
        if(x_hover_adjust > 0)
            x_hover_adjust--;
        else if(x_hover_adjust < 0)
            x_hover_adjust++;
    }
    else if(xAccel > 0)
    {
       x_hover_adjust++;
    }
    else if (xAccel < 0)
    {
        x_hover_adjust--;
    }
    //Hover speed adjust
    //Determines different motor power distrubution. Some motors are weaker than others.
    if(x_hover_adjust > motorxy_hoveradjust_thresh)
    {
        motorSpeedHover[0]+= motorxy_hover_increment;
        motorSpeedHover[1]-= motorxy_hover_increment;
        digitalWrite(ledpin, HIGH);  // turn ON the LED
        x_hover_adjust = 0;
    }else if(x_hover_adjust < (-motorxy_hoveradjust_thresh))
    {
        motorSpeedHover[0]-= motorxy_hover_increment;
        motorSpeedHover[1]+= motorxy_hover_increment;
        digitalWrite(ledpin, HIGH);  // turn ON the LED
        x_hover_adjust = 0;
    }

    xChange_i += xAccel;  //increase integral portion
    //xChange_i = xChange_d+xAccel;

    //Calculating derivative Current acc-last
    double xDiff = xAccel - xChange_d;
    xChange_d = xAccel;   //remember this accel value for next iteration

    double propTemp = kp*xAccel;
    double intTemp = ki* xChange_i;
    double dervTemp = kd*xDiff;
    double totalAdjustment = (propTemp+intTemp+dervTemp);

    double offset = (motorSpeedHover[0]-motorSpeedHover[1])/2;
    //At the end of every control/stabalize loop motorSpeedCorrection is set to motorSpeedHover. This way when control is called. motorspeedcorrection is changed and then stabalize makes necessary adjustments prior to turning copter.
    motorSpeedCorrection[0] = motorSpeedCorrection[0] + (totalAdjustment + offset);
    motorSpeedCorrection[1] = motorSpeedCorrection[1] - (totalAdjustment + offset);
    return;
}

void motorY(double yAccel)
{
    if(absoluteValue(yAccel) < ACCEL_SENSITIVITY_BUFFER)
    {
        return;
    }
    //zeroing out hover speed when quad copter is stable
    else if((int)yAccel == 0)
    {
        //yChange_i = 0;
        if(y_hover_adjust > 0)
            y_hover_adjust--;
        else if(y_hover_adjust<0)
            y_hover_adjust++;
    }
    else if(yAccel>0)
    {
       y_hover_adjust++;
    }
    else if (yAccel<0)
    {
        y_hover_adjust--;
    }
    //Hover speed adjust
    if(y_hover_adjust > motorxy_hoveradjust_thresh)
    {
        motorSpeedHover[2]+= motorxy_hover_increment;
        motorSpeedHover[3]-= motorxy_hover_increment;
        digitalWrite(ledpin, HIGH);  // turn ON the LED
        y_hover_adjust = 0;
    }else if(y_hover_adjust < (-motorxy_hoveradjust_thresh))
    {
        motorSpeedHover[2]-= motorxy_hover_increment;
        motorSpeedHover[3]+= motorxy_hover_increment;
        digitalWrite(ledpin, LOW);  // turn ON the LED
        y_hover_adjust = 0;
    }

    yChange_i += yAccel;  //increase integral portion
    //yChange_i = yChange_d + yAccel;
    //Calculating derivative Current acc-last
    double yDiff = yAccel - yChange_d;
    yChange_d = yAccel;   //remember this accel value for next iteration

    double propTemp = kp*yAccel;
    double intTemp = ki* yChange_i;
    double dervTemp = kd*yDiff;
    double totalAdjustment = (propTemp+intTemp+dervTemp);

    //Determines different motor power distrubution. Some motors are weaker than others.
    double offset = (motorSpeedHover[2]-motorSpeedHover[3])/2;

    motorSpeedCorrection[2] = motorSpeedCorrection[2] + (totalAdjustment + offset);
    motorSpeedCorrection[3] = motorSpeedCorrection[3] - (totalAdjustment + offset);
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
    motorSpeedCorrection[3] -= y/127;
    motorSpeedCorrection[2] += y/127;
    return;
}
void backwards()
{
    //increase -y
    motorSpeedCorrection[3] -= y/127;
    motorSpeedCorrection[2] += y/127;
    return;
}
void right()
{
    //increase x
    motorSpeedCorrection[1] -= x/127;
    motorSpeedCorrection[0] += x/127;
    return;
}
void left()
{
    //increase -x
    motorSpeedCorrection[1] -= x/127;
    motorSpeedCorrection[0] += x/127;
    return;
}
void up()
{
    int z_corrected = ((z*correction)*(max_speed - min_speed)/180 )+min_speed; //maximum input is 127 but max speed is 180

    //digitalWrite(ledpin, HIGH);  // turn ON the LED
    double motorSpeedHoverAvg = 0;
     for(int i = 0; i<4 ; i++)
    {
        motorSpeedCorrection[i] = z_corrected;
        motorSpeedHoverAvg += motorSpeedHover[i];
    }
    motorSpeedHoverAvg /= 4;
    //We always need to either increment or decrement the Hover speed adjust of each motor individually in order to retain stabilization adjustments.
    if(motorSpeedHoverAvg < z_corrected && motorSpeedHoverAvg < max_speed_hover)
    {
        for(int i = 0; i<4 ; i++)
        {
            motorSpeedHover[i]++;
        }
    }
    else
    {
        for(int i = 0; i<4 ; i++)
        {
            motorSpeedHover[i]--;
        }
    }

        //if(motorSpeedHover[i] < min_speed_hover)
            //initialize = 0;

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
}

void motorBounds()
{
    //Used to make sure speed controls are accepted values
    double motorSpeedHoverAvg = 0;
    for(int i = 0; i<4 ; i++)
    {
        if(motorSpeedCorrection[i] < min_speed)
        {
            motorSpeedCorrection[i] = min_speed;
        }
        else if( motorSpeedCorrection[i] > max_speed)
        {
            motorSpeedCorrection[i] = max_speed;
        }

       motorSpeedHoverAvg += motorSpeedHover[i];
    }
    motorSpeedHoverAvg /= 4;
    for(int i = 0 ; i<4; i++)
    {
        if(motorSpeedHoverAvg < min_speed_hover)
        {
            motorSpeedHover[i] += (min_speed_hover - motorSpeedHoverAvg);
        }
        else if(motorSpeedHoverAvg > max_speed_hover)
        {
            motorSpeedHover[i] += (max_speed_hover - motorSpeedHoverAvg);
        }
    }
}
/**
*@brief turns motors on one by one (-x,x,-y,y)
*/
void oscilateMotors()
{

    //Tests the motors by turning on -x then x then -y the y
        motorArray[0].write(min_speed);
        delay(3000);
        motorArray[0].write((int)0);
        motorArray[1].write(min_speed);
        delay(3000);
        motorArray[1].write((int)0);
        motorArray[2].write(min_speed);
        delay(3000);
        motorArray[2].write((int)0);
        motorArray[3].write(min_speed);
        delay(3000);
        motorArray[4].write((int)0);

}

double absoluteValue(double num)
{
    if(num >= 0)
    {
        return (num);
    }
    else
    {
        return (-num);
    }
}
#endif // MOTOR_CONTROL_H
