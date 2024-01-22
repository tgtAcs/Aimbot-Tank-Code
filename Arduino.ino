#define CUSTOM_SETTINGS
#define INCLUDE_GAMEPAD_MODULE
#include <Dabble.h>

#include "Arduino.h"
#include <SoftwareSerial.h>
#include <Servo.h>

Servo rfWheel;  // Create a servo object
Servo lfWheel;
Servo rbWheel;
Servo lbWheel;
Servo arm;
Servo turret;
Servo bipod;

float pwm = 90;
bool autoAim = 0;
bool pod = 0;
bool flush = 0;
int temp = 90;
bool doneMove = 0;

void setup() {
  rfWheel.attach(13);
  lfWheel.attach(12);
  rbWheel.attach(11); 
  lbWheel.attach(10);  
  arm.attach(8);
  turret.attach(5);
  bipod.attach(4);
  
  Serial.begin(9600);      // make sure your Serial Monitor is also set at this baud rate.
  Dabble.begin(9600, 6, 7);      //Enter baudrate of your bluetooth.Connect bluetooth on Bluetooth port present on evive.
}

void loop() {
  Dabble.processInput();             //this function is used to refresh data obtained from smartphone.Hence calling this function is mandatory in order to get data properly from your mobile.
  exe();
}

void exe()
{
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
      if(flush)
      {
        temp = pwm;
        if (Serial.available() > 0) 
        {
          doneMove = 0;
          pwm = pwm + Serial.parseInt();      
          pwm = temp;
          turret.write(pwm);
          flush = 0;
        }
      }
      else if (Serial.available() > 0) 
      {
        doneMove = 0;
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
    else
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
  
  
  
  
  if(pod)
  {
    bipod.attach(4); 
    bipod.write(180);
  }
  else
  {
    bipod.write(80);
  }

  
  

  if (GamePad.isCrossPressed())
  {
    fire();
  }

  if (GamePad.isTrianglePressed())
  {
    autoAim = 1;
    pod = 1;
    flush = 1;
  }

  if (GamePad.isStartPressed())
  {
    autoAim = 0;
    pod = 0;
  }

  if (GamePad.isSelectPressed())
  {  
    autoAim = 0;
    pod = 1;
  }  
}

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
