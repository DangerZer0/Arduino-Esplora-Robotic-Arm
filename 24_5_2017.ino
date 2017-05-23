#include <Servo.h>
#include <Esplora.h>

Servo myservoX, myservoY, myservoZ, myservoH;  // create two servo objects to control the servos connected to the Tinkerkit connectors

int xDefServ,  yDefServ;   // default servo location, in degrees (value from 0 to 180)
int xCurrServ, yCurrServ;  // current servo location, in degrees (value from 0 to 180)
int xNewServ,  yNewServ;   // new servo location to reach, in degrees (value from 0 to 180)

int xCurrJoy, yCurrJoy;    // current joystick position, in range (from -512 to +512)
int xCalJoy, yCalJoy;      // calibrate offset of the joystick position, in range (from -512 to +512)

int calibBtnState = 0, calibBtnDuration = 0;  // variables used in statemachine for the calibration button status
float stepUp, sysSpeed;                       // the update speed of the current X,Y Coordinates
int calibSpeed;                               // the speed of calibration process
int pulseCount, pulseComp;                    // used to emulate servo PWM signal on output digital pin no. 1
int switch_up=0;

void setup()
{
  xDefServ = 90;              // default servo X angle
  yDefServ = 180;              // default servo Y angle

  myservoX.attach(11);        // initialize first servo on D11
  myservoX.write(xDefServ);   // set servo to default X position
  myservoY.attach(3);         // initialize first servo on D3
  myservoY.write(yDefServ);   // set servo to default Y position
  myservoH.attach(0);
  xCurrServ = xDefServ;       // Set the current X Position
  yCurrServ = yDefServ;       // Set the current Y Position
  xNewServ = xDefServ;        // Set the new X Position
  yNewServ = yDefServ;        // Set the new Y Position

  xCalJoy = Esplora.readJoystickX();    // set the calibration for the X joystick offet
  yCalJoy = Esplora.readJoystickY();    // set the calibration for the Y joystick offet
  xCurrJoy = map(xCurrServ, 0, 180, -512, 512) - xCalJoy;   // set the current joystick X reference position
  yCurrJoy = map(yCurrServ, 0, 180, -512, 512) - xCalJoy;   // set the current joystick Y reference position

  stepUp = 0;               // initialize the update speed
  pinMode(1, OUTPUT);       // define the difital pin D1 as output pin, used to control the head tilt servo
  pinMode(0, OUTPUT);       // define the digital pin D0 as output pin, used to control the head grab servo
  pulseCount = 0;           // initialize the variable representing the pulse width for the shutter servo
  pulseComp = 0;            // initialize the variable representing the pulse duration for the shutter servo
  sysSpeed = map(Esplora.readSlider(), 0, 1023, 1, 8) / 4.0;     // the speed of updating position
}



void reCalibrate(){
  // set the servos' default position to the current position they hold.
  // this function works on the values in degrees
  xDefServ = map(xCurrJoy, -512, 512, 0, 180) ;        // set the default X position, in degree
  yDefServ = map(yCurrJoy, -512, 512, 0, 180) ;        // set the default X position, in degree
}


void backToDef() {
  // return to the default position defined by xDefServ and yDefServ angles
  // this reset will be executed with gradual speed depending on the Esplora slider value
  xNewServ = xDefServ;
  yNewServ = yDefServ;
  stepUp = 1;
  
  // loop while the current position is not equal to the default position of the servo
  while ((xNewServ != xCurrServ) || (yNewServ != yCurrServ)) 
  {
    calibSpeed = map(Esplora.readSlider(), 0, 1023, 40, 4); // set the calibration speed
    
    //move the X servo
    if (xNewServ > xCurrServ)
      xCurrServ = xCurrServ + stepUp;
    else if (xNewServ < xCurrServ)
      xCurrServ = xCurrServ - stepUp;
    myservoX.write(xCurrServ);
    
    //move the Y servo
    if (yNewServ > yCurrServ)
      yCurrServ = yCurrServ + stepUp;
    else if (yNewServ < yCurrServ)
      yCurrServ = yCurrServ - stepUp;
    myservoY.write(yCurrServ);
    delay(calibSpeed);
    }
   // calucalte the X,Y current joystick positions
   xCurrJoy = map(xCurrServ, 0, 180, -512, 512);
   yCurrJoy = map(yCurrServ, 0, 180, -512, 512);
}

void takeShot() {
/* 
this is the tricky part, we need to imulate a PWM signal for the servo with the following characteristics,
- to move 90 counter-clockwize: we need continuose 20 ms signal with 1 ms HIGH pulse and 19 LOW.
- to move 90 clockwize: we need continuose 20 ms signal with 2 ms HIGH pulse and 18 LOW.
therefore, just use counters to emulate this behaviour for half a second duration.
*/
  while (pulseComp <= 500    ) {
    if (pulseComp < 250) {
      if (pulseCount < 1) {
        digitalWrite(1, HIGH);
      }
      else
      {
        digitalWrite(1, LOW);
      };
    }

    else {
        if (pulseCount < 2) {
          digitalWrite(1, HIGH);
        }
        else
        {
          digitalWrite(1, LOW);
        };
      }
    pulseCount = (pulseCount + 1) % 20;
    pulseComp = (pulseComp + 1);
    delay(1);
  }
}


// Main Loop
void loop()
{
  sysSpeed = map(Esplora.readSlider(), 0, 1023, 1, 8) / 4.0;    //calculate the system speed based on the Esplora slider

  stepUp = (-1.0 * map(Esplora.readJoystickX() - xCalJoy, -512, 512, -5, 5)) * sysSpeed;  //  calculate the step for the X joystick coordinate
  if (stepUp > 0)
    xCurrJoy = min(xCurrJoy + stepUp, 512);  // to avoid X position overflow
  else
    xCurrJoy = max(xCurrJoy + stepUp, -512); // to avoid X position downflow
    
  stepUp = (-1.0 * map(Esplora.readJoystickY() - yCalJoy, -512, 512, -5, 5)) * sysSpeed; //  calculate the step for the Y joystick coordinate
  if (stepUp > 0)
    yCurrJoy = min(yCurrJoy + stepUp, 512);  // to avoid Y position overflow
  else
    yCurrJoy = max(yCurrJoy + stepUp, -512); // to avoid Y position downflow
    
  
  myservoX.write(map(xCurrJoy, -512, 512, 0, 180));   // write the X position to the servo
  myservoY.write(map(yCurrJoy, -512, 512, 0, 180));   // write the Y position to the servo
  /* 
  statemachine for the Esplora Button 1 status
  basically, if you press Button 1 for more than 1 second, it will re-calibrate the default position to the new position.
  otherwise, if you press it for less than a second, it will move back to the default position previously set
  */
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
   
  //tack a shot when the joystick button is pressed 
  if (Esplora.readJoystickSwitch() == LOW) {
    pulseComp = 0;
    takeShot();
  }
  
  // read the current values from servos
  xCurrServ = myservoX.read();
  yCurrServ = myservoY.read();

  delay(4);
  int button = Esplora.readButton(SWITCH_UP);

  if(button == HIGH)
    myservoH.write(0);
    switch_up++;
  if(button == HIGH)
    if(switch_up==1){
      switch_up--;
      myservoH.write(145);
      }
}
