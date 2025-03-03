//--------------------------------------------------------------------------
// Code for Haptics Laboratory - Template 1
// Code updated by Leonam Pecly - 2022.09.26
// Based on Professor Allison Akamura's team code - 05.22.20
//--------------------------------------------------------------------------

// Define the Hapkit version
//#define HAPKIT_A
#define HAPKIT_B
#define PI 3.1415926535897932384626433832795
// Includes
#include <math.h>

// Pin declares
#ifdef HAPKIT_A
  int pwmPin = 3; // PWM output pin for motor 1 - Hapkit A
  int dirPin = 12; // direction output pin for motor 1 - Hapkit A
#endif
#ifdef HAPKIT_B
  int pwmPin = 5; // PWM output pin for motor 1 - Hapkit B
  int dirPin = 8; // direction output pin for motor 1 - Hapkit B
#endif
int sensorPosPin = A2; // input pin for MR sensor
int fsrPin = A3; // input pin for FSR sensor

// Position tracking variables
int updatedPos = 0;     // keeps track of the latest updated value of the MR sensor reading
int rawPos = 0;         // current raw reading from MR sensor
int lastRawPos = 0;     // last raw reading from MR sensor
int lastLastRawPos = 0; // last last raw reading from MR sensor
int flipNumber = 0;     // keeps track of the number of flips over the 180deg mark
int tempOffset = 0;
int rawDiff = 0;
int lastRawDiff = 0;
int rawOffset = 0;
int lastRawOffset = 0;
int serial_prints = 0;
const int flipThresh = 700;  // threshold to determine whether or not a flip over the 180 degree mark occurred
boolean flipped = false;
double OFFSET = 980;
double OFFSET_NEG = 15;
double ts = 0;

// Kinematics variables
double xh = 0;           // position of the handle [m]

// Force output variables
double force = 0;           // force at the handle
double Tp = 0;              // torque of the motor pulley
double duty = 0;            // duty cylce (between 0 and 255)
unsigned int output = 0;    // output command to the motor


// --------------------------------------------------------------
// Setup function -- NO NEED TO EDIT
// --------------------------------------------------------------
void setup() 
{
  // Set up serial communication
  Serial.begin(9600);
  
  // Set PWM frequency 
  setPwmFrequency(pwmPin,1); 
  
  // Input pins
  pinMode(sensorPosPin, INPUT); // set MR sensor pin to be an input
  pinMode(fsrPin, INPUT);       // set FSR sensor pin to be an input

  // Output pins
  pinMode(pwmPin, OUTPUT);  // PWM pin for motor A
  pinMode(dirPin, OUTPUT);  // dir pin for motor A
  
  // Initialize motor 
  analogWrite(pwmPin, 0);     // set to not be spinning (0/255)
  digitalWrite(dirPin, LOW);  // set direction
  
  // Initialize position valiables
  lastLastRawPos = analogRead(sensorPosPin);
  lastRawPos = analogRead(sensorPosPin);
  flipNumber = 0;
  serial_prints = 0;
  ts = 0.00;
}


// --------------------------------------------------------------
// Main Loop
// --------------------------------------------------------------
void loop()
{
  
  //*************************************************************
  //*** Section 1. Compute position in counts (do not change) ***  
  //*************************************************************

  // Get voltage output by MR sensor
  rawPos = analogRead(sensorPosPin);  //current raw position from MR sensor

  // Calculate differences between subsequent MR sensor readings
  rawDiff = rawPos - lastRawPos;          //difference btwn current raw position and last raw position
  lastRawDiff = rawPos - lastLastRawPos;  //difference btwn current raw position and last last raw position
  rawOffset = abs(rawDiff);
  lastRawOffset = abs(lastRawDiff);
  
  // Update position record-keeping vairables
  lastLastRawPos = lastRawPos;
  lastRawPos = rawPos;
  
  // Keep track of flips over 180 degrees
  if((lastRawOffset > flipThresh) && (!flipped)) { // enter this anytime the last offset is greater than the flip threshold AND it has not just flipped
    if(lastRawDiff > 0) {        // check to see which direction the drive wheel was turning
      flipNumber--;              // cw rotation 
    } else {                     // if(rawDiff < 0)
      flipNumber++;              // ccw rotation
    }
    flipped = true;            // set boolean so that the next time through the loop won't trigger a flip
  } else {                        // anytime no flip has occurred
    flipped = false;
  }
  
  //*************************************************************
  //*** Section 2. Compute position in meters *******************
  //*************************************************************

   updatedPos = rawPos + flipNumber*OFFSET; // need to update pos based on what most recent offset is 
  
  // Step B.6: double ts = ?; // Compute the angle of the sector pulley (ts) in degrees based on updatedPos


  ts = (-0.0128*updatedPos) +1.3253;

  // Step B.7: xh = ?;       // Compute the position of the handle (in meters) based on ts (in radians)

  xh = 0.00579 * ts * PI/180;

    // Min Position (To the Left) = 0.00472
    // 0 position = -0.00005
    // Max Position (To the right)= -0.00425

  // Step B.8: print xh via serial monitor

  //*************************************************************
  //*** Section 3. Assign a motor output force in Newtons *******  
  //*************************************************************
 
  // Define kinematic parameters you may need
  double rp = 0.00475;   //[m]
  double rs = 0.073;   //[m] 
  // Step C.1: force = ?; // You can  generate a force by assigning this to a constant number (in Newtons) or use a haptic rendering / virtual environment
  //double force = 2.4; 

  // IMPORTANT Force Ranges = 0.12 - 2.5
  // Step C.2: Tp = ?;    // Compute the require motor pulley torque (Tp) to generate that force using kinematics
  //double Tp_test = force/172.7;

  // Spring

  // Max torque applied = 0.013896
    // min K constant = 0.12/ = 25.42372881
    // max K constant = 2.5/0.00472 = 529.6610169
    // max K constant = 2.5/0.00472 = 529.6610169
  
  double K_spring = 529.66101;
  double force = K_spring * xh;
  double T_spring = force/172.7;
  Tp = T_spring;
  if (serial_prints > 2000)
  {
    Serial.print("ts: ");
    Serial.println(ts);
    Serial.print("xh: ");
    Serial.println(xh, 5);
    //Serial.print("Tp: ");
    //Serial.println(Tp_test, 5);
    Serial.print("T_spring: ");
    Serial.println(T_spring, 5);
    serial_prints = 0;
  }
  serial_prints++;
  //*************************************************************
  //*** Section 4. Force output (do not change) *****************
  //*************************************************************
  
  // Determine correct direction for motor torque
  if(force > 0) { 
    digitalWrite(dirPin, HIGH);
  } else {
    digitalWrite(dirPin, LOW);
  }

  // Compute the duty cycle required to generate Tp (torque at the motor pulley)
  #ifdef HAPKIT_A
    duty = sqrt(430718*abs(Tp)*abs(Tp)*abs(Tp) - 10933*abs(Tp)*abs(Tp) + 126.5*abs(Tp));
  #endif
  #ifdef HAPKIT_B
    duty = sqrt(abs(Tp)/0.0183);
  #endif

  // Make sure the duty cycle is between 0 and 100%
  if (duty > 1) {            
    duty = 1;
  } else if (duty < 0) { 
    duty = 0;
  }  
  output = (int)(duty* 255);   // convert duty cycle to output signal
  analogWrite(pwmPin,output);  // output the signal
}

// --------------------------------------------------------------
// Function to set PWM Freq -- DO NOT EDIT
// --------------------------------------------------------------
void setPwmFrequency(int pin, int divisor) {
  byte mode;
  if(pin == 5 || pin == 6 || pin == 9 || pin == 10) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 64: mode = 0x03; break;
      case 256: mode = 0x04; break;
      case 1024: mode = 0x05; break;
      default: return;
    }
    if(pin == 5 || pin == 6) {
      TCCR0B = TCCR0B & 0b11111000 | mode;
    } else {
      TCCR1B = TCCR1B & 0b11111000 | mode;
    }
  } else if(pin == 3 || pin == 11) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 32: mode = 0x03; break;
      case 64: mode = 0x04; break;
      case 128: mode = 0x05; break;
      case 256: mode = 0x06; break;
      case 1024: mode = 0x7; break;
      default: return;
    }
    TCCR2B = TCCR2B & 0b11111000 | mode;
  }
}
