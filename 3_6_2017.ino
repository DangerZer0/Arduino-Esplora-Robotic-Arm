/*
- Use the joystick to control the servo motors for the X and Y axis
- Use the right button to use the Z axis servo motor
- Use the up button to grab items
- Use the down button to reset all servo motors to a previously chosen location
- Use the linear potentiometer to control the movement speed of the servo motors
*/

#include <Servo.h>
#include <Esplora.h>

/*
Create 4 servo objects to control the servos connected to the Arduino Esplora
- myservoX: X axis servo motor
- myservoY: Y axis servo motor
- myservoZ: Z axis servo motor
- myservoH: Hand grab servo motor
*/
Servo myservoX, myservoY, myservoZ, myservoH;  

int xDefServ,  yDefServ, zDefServ;   // Default servo location, in degrees (value from 0 to 180)
int xCurrServ, yCurrServ, zCurrServ;  // Current servo location, in degrees (value from 0 to 180)
int xNewServ,  yNewServ, zNewServ;   // New servo location to reach, in degrees (value from 0 to 180)

int xCurrJoy, yCurrJoy, zCurrJoy;    // Current joystick position, in range (from -512 to +512)
int xCalJoy, yCalJoy, zCalJoy;      // Calibrate offset on the joystick position, in range (from -512 to +512)

int calibBtnState = 0, calibBtnDuration = 0;  // Calibration button status buttons
float stepUp, sysSpeed;                       // The update speed of the current X,Y Coordinates
int calibSpeed;                               // The speed of calibration process
int pulseCount, pulseComp;                    // Used to emulate servo PWM signal on output digital pin no. 1

long debounce = 200;   // The debounce time, increase if the output flickers
int state = HIGH;      // The current state of the output pin
int previous = LOW;    // The previous reading from the input pin
long time = 0;         // The last time the output pin was toggled

long debounce1 = 200;   // The debounce time, increase if the output flickers
int state1 = HIGH;      // The current state of the output pin
int previous1 = LOW;    // The previous reading from the input pin
int time1 = 0;


void setup(){
  xDefServ = 90;              // Default servo X angle
  yDefServ = 180;              // Default servo Y angle
  zDefServ = 70;              // Default servo Z angle
  
  myservoX.attach(11);        // Initialize first servo on D11
  myservoX.write(xDefServ);   // Set servo to default X position
  myservoY.attach(3);         // Initialize second servo on D3
  myservoY.write(yDefServ);   // Set servo to default Y position
  myservoZ.attach(1);         // Initialize third servo on D1
  myservoZ.write(zDefServ);   // Set servo to default Z position
  myservoH.attach(0);         // Initialize forth servo on D0
  
  xCurrServ = xDefServ;       // Set the current X Position
  yCurrServ = yDefServ;       // Set the current Y Position
  zCurrServ = zDefServ;       // Set the current Z Position
  xNewServ = xDefServ;        // Set the new X Position
  yNewServ = yDefServ;        // Set the new Y Position
  zNewServ = zDefServ;        // Set the new Z position

  xCalJoy = Esplora.readJoystickX();    // Set the calibration for the X joystick offet
  yCalJoy = Esplora.readJoystickY();    // Set the calibration for the Y joystick offet
  zCalJoy = Esplora.readJoystickY();    // Set the calibration for the Z joystick offet
  xCurrJoy = map(xCurrServ, 0, 180, -512, 512) - xCalJoy;   // Set the current joystick X reference position
  yCurrJoy = map(yCurrServ, 0, 180, -512, 512) - xCalJoy;   // Set the current joystick Y reference position
  zCurrJoy = map(zCurrServ, 0, 180, -512, 512) - xCalJoy;   // Set the current joystick Z reference position8 
  
  stepUp = 0;               // Initialize the update speed
  pinMode(0, OUTPUT);       // Define the digital pin D0 as output pin, used to control the head grab servo motor
  pinMode(1, OUTPUT);
  pinMode(3, OUTPUT);
  sysSpeed = map(Esplora.readSlider(), 0, 1023, 1, 8) / 4.0;     // The speed of updating position
}


void reCalibrate(){
  // Set the servos' default position to the current position they hold.
  // This function works on the values in degrees
  xDefServ = map(xCurrJoy, -512, 512, 0, 180);        // Set the default X position, in degree
  yDefServ = map(yCurrJoy, -512, 512, 0, 180);        // Set the default Y position, in degree
  zDefServ = map(zCurrJoy, -512, 512, 0, 180);        // Set the default Z position, in degree
}


void backToDef() {
  // Return to the default position defined by xDefServ and yDefServ angles
  // This reset will be executed with gradual speed depending on the Esplora slider value
  xNewServ = xDefServ;
  yNewServ = yDefServ;
  stepUp = 1;
  
  // Loop while the current position is not equal to the default position of the servo
  while ((xNewServ != xCurrServ) || (yNewServ != yCurrServ)){
    calibSpeed = map(Esplora.readSlider(), 0, 1023, 40, 4); // Set the calibration speed
    
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
  // Calucalte the X, Y, Z current joystick positions
  xCurrJoy = map(xCurrServ, 0, 180, -512, 512);
  yCurrJoy = map(yCurrServ, 0, 180, -512, 512);
}


// Main Loop
void loop(){
  sysSpeed = map(Esplora.readSlider(), 0, 1023, 1, 8) / 4.0;    // Calculate the system speed based on the Esplora slider
  
  stepUp = (-1.0 * map(Esplora.readJoystickX() - xCalJoy, -512, 512, -5, 5)) * sysSpeed;  // Calculate the step for the X joystick coordinate
  if (stepUp > 0)
    xCurrJoy = min(xCurrJoy + stepUp, 512);  // To avoid X position overflow
  else
    xCurrJoy = max(xCurrJoy + stepUp, -512); // To avoid X position downflow
    
  int reading1 = Esplora.readButton(SWITCH_RIGHT);    
    
  if (reading1 == LOW && previous1 == HIGH && millis() - time > debounce1) {
    
    if (state1 == HIGH){
      stepUp = (-1.0 * map(Esplora.readJoystickY() - yCalJoy, -512, 512, -5, 5)) * sysSpeed; // Calculate the step for the Y joystick coordinate
      if (stepUp > 0)
        yCurrJoy = min(yCurrJoy + stepUp, 512);  // To avoid Y position overflow
      else
        yCurrJoy = max(yCurrJoy + stepUp, -512); // To avoid Y position downflow
      state1 = LOW;
    }
    
    else{
      stepUp = (-1.0 * map(Esplora.readJoystickY() - zCalJoy, -512, 512, -5, 5)) * sysSpeed; // Calculate the step for the Z joystick coordinate
      if (stepUp > 0)
        zCurrJoy = min(zCurrJoy + stepUp, 512);  // To avoid Z position overflow
      else
        zCurrJoy = max(zCurrJoy + stepUp, -512); // To avoid Z position downflow
      state1 = HIGH;
    }
    time = millis();
  }
  
  previous1 = reading1;

  myservoX.write(map(xCurrJoy, -512, 512, 0, 180));   // Write the X position to the servo
  myservoY.write(map(yCurrJoy, -512, 512, 0, 180));   // Write the Y position to the servo
  myservoZ.write(map(zCurrJoy, -512, 512, 0, 180));   // Write the Z position to the servo
 
  switch (calibBtnState){
    case 0:
      if (Esplora.readButton(1) == LOW)
      {
        calibBtnState = 1;
        calibBtnDuration = 0;
      }
    break;
    case 1:
      calibBtnDuration = calibBtnDuration + 1;
      if (calibBtnDuration > 250){
        reCalibrate();
        calibBtnState = 2;
        calibBtnDuration = 0;
      }
        
      if (Esplora.readButton(1) == HIGH){
        backToDef();
        calibBtnState = 0;
        calibBtnDuration = 0;
      }
    break;
    case 2:
      if (Esplora.readButton(1) == HIGH){
        calibBtnState = 0;
        calibBtnDuration = 0;
      }
   }
   
  // Read the current values from servos
  xCurrServ = myservoX.read();
  yCurrServ = myservoY.read();
  zCurrServ = myservoZ.read();
  
  delay(4);
  
  int reading = Esplora.readButton(SWITCH_UP);
  
  if (reading == LOW && previous == HIGH && millis() - time > debounce) {
    if (state == HIGH){
      myservoH.write(40);
      state = LOW;
    }
      
    else{
      state = HIGH;
      myservoH.write(180);
    }
    time = millis();    
  }
  previous = reading;
}
