//Library for dabble and it's gamepad
#define CUSTOM_SETTINGS
#define INCLUDE_GAMEPAD_MODULE
#include <Dabble.h>

//Default librarys for servo control
#include "Arduino.h"
#include <Servo.h>

//Wheels
Servo rfWheel;
Servo lfWheel;
Servo rbWheel;
Servo lbWheel;

//Firing servo
Servo arm;

//X and y diriction servo
Servo turret;
Servo bipod;


float pwm = 90;     //Servo angle
bool autoAim = 0;   //Auto aim mode
bool pod = 0;       //Aiming pod mode, must be on when auto aim mode, also adjusts the aiming e
bool flush = 0;
int temp = 90;

void setup() {
  //Servos pin assigning
  rfWheel.attach(13);
  lfWheel.attach(12);
  rbWheel.attach(11); 
  lbWheel.attach(10);  
  arm.attach(8);
  turret.attach(5);
  bipod.attach(4);

  //set serial and bluetooth port baud rate
  Serial.begin(9600);      
  Dabble.begin(9600, 6, 7);
}

void loop() {
  Dabble.processInput();
  exe();
}

void exe()
{
  //Dabble gamepad get inputs
  if (GamePad.isLeftPressed())
  {
    moveTurret(+0.05);
  }
  else if (GamePad.isRightPressed())
  {
    moveTurret(-0.05);
  }
  else
  {
    
    if(autoAim)
    {
      if(flush) //Flush the serial port when auto aim is activated, so the turret doesn't shake
      {
        temp = pwm;
        if (Serial.available() > 0) 
        {
          pwm = pwm + Serial.parseInt();      
          pwm = temp;
          turret.write(pwm);
          flush = 0;
        }
      }
      else if (Serial.available() > 0) 
      {
        pwm = pwm + Serial.parseInt();      
        if (pwm < 0) 
        {
          pwm = 0;
        }
        if(pwm > 180)
        {
          pwm = 180;
        }
          turret.write(pwm);
      }
    }
    else //If auto aim not on, tank is free to move, and put the pod down
      {
        if (GamePad.isUpPressed())
        {
          forward();
          pod = 0;
        }
        else if (GamePad.isDownPressed())
        { 
          back();
          pod = 0;  
        }
        else if (GamePad.isSquarePressed())
        {
          
          left();
          pod = 0;  
        }
        else if (GamePad.isCirclePressed())
        {
          
          right();
          pod = 0;  
        }
        else
        {
          stop();
        }
      }
  }
  
  if(pod) //Put the pod up
  {
    bipod.attach(4); 
    bipod.write(180);
  }
  else  //Put the pod down
  {
    bipod.write(80);
  }

  //Fire
  if (GamePad.isCrossPressed())
  {
    fire();
  }

  //Auto aim mode
  if (GamePad.isTrianglePressed())
  {
    autoAim = 1;
    pod = 1;
    flush = 1;
  }

  //Turnoff auto and pod mode
  if (GamePad.isStartPressed())
  {
    autoAim = 0;
    pod = 0;
  }

  //Turn off auto mode but stays/turn-on the pod
  if (GamePad.isSelectPressed())
  {  
    autoAim = 0;
    pod = 1;
  }  
}

//Turn the turret by adding or subtracting current angle with parameter
void moveTurret(float angle)
{
  if(pwm + angle > 180)
  {
    pwm = 180;
  }
  else if(pwm + angle < 0)
  {
    pwm = 0;
  }
  else
  {
    pwm = pwm + angle;
  }
  turret.write(pwm);
}

//Stop everything to save maximum power for firing
void fire()
{
  stop();
  arm.write(0);
  delay(2100);
  arm.write(90);
}

void recenter()
{
  turret.write(90);
}

void stop()
{
  rfWheel.write(90);
  lfWheel.write(90);
  rbWheel.write(90);
  lbWheel.write(90);
}

void start()
{
  rfWheel.attach(13);
  lfWheel.attach(12);
  rbWheel.attach(11); 
  lbWheel.attach(10);  
  arm.attach(8);
  turret.attach(5);
}
void right()
{
  rfWheel.write(180);
  lfWheel.write(180);
  rbWheel.write(180);
  lbWheel.write(180);
}

void left()
{
  rfWheel.write(0);
  lfWheel.write(0);
  rbWheel.write(0);
  lbWheel.write(0);
}

void back()
{
  rfWheel.write(180);
  rbWheel.write(180);
  lfWheel.write(0);
  lbWheel.write(0);
}

void forward()
{

  rfWheel.write(0);
  rbWheel.write(0);
  lfWheel.write(180);
  lbWheel.write(180);
 
}
