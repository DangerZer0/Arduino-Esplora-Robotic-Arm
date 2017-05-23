#include <Servo.h>  //Import Servo.h library
#include <Esplora.h>  //Import Esplora.h library

Servo myservoX, myservoY, myservoZ, myservoH;  //Create 4 servo objects to control the servos

int xDefServ,  yDefServ;   //Default servo location, in degrees (value from 0 to 180)
int xCurrServ, yCurrServ;  //Current servo location, in degrees (value from 0 to 180)
int xNewServ,  yNewServ;   //New servo location to reach, in degrees (value from 0 to 180)

int xCurrJoy, yCurrJoy;    //Current joystick position, in range (from -512 to +512)
int xCalJoy, yCalJoy;      //Calibrate offset of the joystick position, in range (from -512 to +512)

int calibBtnState = 0, calibBtnDuration = 0;  //Variables used in statemachine for the calibration button status
float stepUp, sysSpeed;                       //The update speed of the current X,Y Coordinates
int calibSpeed;                               //The speed of calibration process
int pulseCount, pulseComp;                    //Used to emulate servo PWM signal on output digital pin no. 1
int switch_up=0;

void setup() {
  xDefServ = 90;              //Default servo X angle
  yDefServ = 180;              //Default servo Y angle

  myservoX.attach(11);        //Initialize first servo on D11
  myservoX.write(xDefServ);   //Set servo to default X position
  myservoY.attach(3);         //Initialize second servo on D3
  myservoY.write(yDefServ);   //Set servo to default Y position
  myservoH.attach(0);         //Initialize third servo on D1
  xCurrServ = xDefServ;       //Set the current X Position
  yCurrServ = yDefServ;       //Set the current Y Position
  xNewServ = xDefServ;        //Set the new X Position
  yNewServ = yDefServ;        //Set the new Y Position

  xCalJoy = Esplora.readJoystickX();    //Set the calibration for the X joystick offet
  yCalJoy = Esplora.readJoystickY();    //Set the calibration for the Y joystick offet
  xCurrJoy = map(xCurrServ, 0, 180, -512, 512) - xCalJoy;   //Set the current joystick X reference position
  yCurrJoy = map(yCurrServ, 0, 180, -512, 512) - xCalJoy;   //Set the current joystick Y reference position

  stepUp = 0;               //Initialize the update speed
  pinMode(1, OUTPUT);       //Define the difital pin D1 as output pin, used to control the head tilt servo
  pinMode(0, OUTPUT);       //Define the digital pin D0 as output pin, used to control the head grab servo
  pulseCount = 0;           //Initialize the variable representing the pulse width for the shutter servo
  pulseComp = 0;            //Initialize the variable representing the pulse duration for the shutter servo
  sysSpeed = map(Esplora.readSlider(), 0, 1023, 1, 8) / 4.0;     //The speed of updating position
}



void reCalibrate() {
  //Set the servos' default position to the current position they hold.
  //This function works on the values in degrees
  xDefServ = map(xCurrJoy, -512, 512, 0, 180) ;        //Set the default X position, in degree
  yDefServ = map(yCurrJoy, -512, 512, 0, 180) ;        //Set the default Y position, in degree
}


void backToDef() {
  //Return to the default position defined by xDefServ and yDefServ angles
  //This reset will be executed with gradual speed depending on the Esplora slider value
  xNewServ = xDefServ;
  yNewServ = yDefServ;
  stepUp = 1;
  
  //Loop while the current position is not equal to the default position of the servo
  while ((xNewServ != xCurrServ) || (yNewServ != yCurrServ)) {
    calibSpeed = map(Esplora.readSlider(), 0, 1023, 40, 4);  //Set the calibration speed
    
    //Move the X servo
    if (xNewServ > xCurrServ)
      xCurrServ = xCurrServ + stepUp;
    else if (xNewServ < xCurrServ)
      xCurrServ = xCurrServ - stepUp;
    myservoX.write(xCurrServ);
    
    //Move the Y servo
    if (yNewServ > yCurrServ)
      yCurrServ = yCurrServ + stepUp;
    else if (yNewServ < yCurrServ)
      yCurrServ = yCurrServ - stepUp;
    myservoY.write(yCurrServ);
    delay(calibSpeed);
   }
   //Calucalte the X,Y current joystick positions
   xCurrJoy = map(xCurrServ, 0, 180, -512, 512);
   yCurrJoy = map(yCurrServ, 0, 180, -512, 512);
}

void takeShot() {
  //Movement in pulses
  while (pulseComp <= 500    ) {
    if (pulseComp < 250) {
      if (pulseCount < 1)
        digitalWrite(1, HIGH);
      else
        digitalWrite(1, LOW);
    }

    else {
        if (pulseCount < 2)
          digitalWrite(1, HIGH);
        else
          digitalWrite(1, LOW);
    }
    pulseCount = (pulseCount + 1) % 20;
    pulseComp = (pulseComp + 1);
    delay(1);
  }
}

//Main Loop
void loop() {
  sysSpeed = map(Esplora.readSlider(), 0, 1023, 1, 8) / 4.0;    //Calculate the system speed based on the Esplora slider

  stepUp = (-1.0 * map(Esplora.readJoystickX() - xCalJoy, -512, 512, -5, 5)) * sysSpeed;  //Calculate the step for the X joystick coordinate
  if (stepUp > 0)
    xCurrJoy = min(xCurrJoy + stepUp, 512);  //To avoid X position overflow
  else
    xCurrJoy = max(xCurrJoy + stepUp, -512); //To avoid X position downflow
    
  stepUp = (-1.0 * map(Esplora.readJoystickY() - yCalJoy, -512, 512, -5, 5)) * sysSpeed; //Calculate the step for the Y joystick coordinate
  if (stepUp > 0)
    yCurrJoy = min(yCurrJoy + stepUp, 512);  //To avoid Y position overflow
  else
    yCurrJoy = max(yCurrJoy + stepUp, -512); //To avoid Y position downflow
    
  
  myservoX.write(map(xCurrJoy, -512, 512, 0, 180));   //Write the X position to the servo
  myservoY.write(map(yCurrJoy, -512, 512, 0, 180));   //Write the Y position to the servo
  
  switch (calibBtnState)
  {
    case 0:
      if (Esplora.readButton(1) == LOW)
      {
        calibBtnState = 1;
        calibBtnDuration = 0;
      }
    break;
    case 1:
      calibBtnDuration = calibBtnDuration + 1;
      if (calibBtnDuration > 250)
      {
        reCalibrate();
        calibBtnState = 2;
        calibBtnDuration = 0;
      }
        
      if (Esplora.readButton(1) == HIGH)
      {
        backToDef();
        calibBtnState = 0;
        calibBtnDuration = 0;
      }
    break;
    case 2:
      if (Esplora.readButton(1) == HIGH)
      {
        calibBtnState = 0;
        calibBtnDuration = 0;
      }
   }
  
  //Read the current values from servos
  xCurrServ = myservoX.read();
  yCurrServ = myservoY.read();

  delay(4);
  int button = Esplora.readButton(SWITCH_UP); //Read UP Button
  
  //If the UP button is pressed rotate servo 4 and switch_up++
  //Thus the claw closes up
  if(button == HIGH) {
    myservoH.write(0);
    switch_up++;
  }
  
  //If the UP button is pressed twice switch-- and rotate servo 4 to a basic positin
  //Thus the claw opens up
  if(button == HIGH)
    if(switch_up==1) {
      switch_up--;
      myservoH.write(145);
    }
}
