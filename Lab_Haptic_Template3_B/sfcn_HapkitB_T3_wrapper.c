
/*
 * Include Files
 *
 */
#if defined(MATLAB_MEX_FILE)
#include "tmwtypes.h"
#include "simstruc_types.h"
#else
#include "rtwtypes.h"
#endif



/* %%%-SFUNWIZ_wrapper_includes_Changes_BEGIN --- EDIT HERE TO _END */
#ifndef MATLAB_MEX_FILE

#include <math.h>
#include <Arduino.h>

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
const int flipThresh = 700;  // threshold to determine whether or not a flip over the 180 degree mark occurred
boolean flipped = false;

// Force output variables
double Tp = 0;              // torque of the motor pulley
double duty = 0;            // duty cylce (between 0 and 255)

double Xh = 0;
double var1 = 0;
double var2 = 0;

int get_updatedPos() {
	return updatedPos;
}

double get_Tp() {
	return Tp;
}

double get_Xh() {
	return Xh;
}

double get_var1() {
	return var1;
}

double get_var2() {
	return var2;
}

#endif
/* %%%-SFUNWIZ_wrapper_includes_Changes_END --- EDIT HERE TO _BEGIN */
#define u_width 1
#define y_width 1

/*
 * Create external references here.  
 *
 */
/* %%%-SFUNWIZ_wrapper_externs_Changes_BEGIN --- EDIT HERE TO _END */
/* extern double func(double a); */
/* %%%-SFUNWIZ_wrapper_externs_Changes_END --- EDIT HERE TO _BEGIN */

/*
 * Start function
 *
 */
void sfcn_HapkitB_T3_Start_wrapper(real_T *xD,
			const uint8_T *sensorPosPin, const int_T p_width0,
			const uint8_T *dirPin, const int_T p_width1,
			const uint8_T *pwmPin, const int_T p_width2)
{
/* %%%-SFUNWIZ_wrapper_Start_Changes_BEGIN --- EDIT HERE TO _END */
/*
 * Custom Start code goes here.
 */
/*
/* %%%-SFUNWIZ_wrapper_Start_Changes_END --- EDIT HERE TO _BEGIN */
}
/*
 * Output function
 *
 */
void sfcn_HapkitB_T3_Outputs_wrapper(const real32_T *Tp_in,
			const real32_T *Xh_in,
			const real32_T *var1_in,
			const real32_T *var2_in,
			real32_T *updatedPos_out,
			const real_T *xD,
			const uint8_T *sensorPosPin, const int_T p_width0,
			const uint8_T *dirPin, const int_T p_width1,
			const uint8_T *pwmPin, const int_T p_width2)
{
/* %%%-SFUNWIZ_wrapper_Outputs_Changes_BEGIN --- EDIT HERE TO _END */
/* This sample sets the output equal to the input
    y0[0] = u0[0]; 
    For complex signals use: y0[0].re = u0[0].re; 
    y0[0].im = u0[0].im;
    y1[0].re = u1[0].re;
    y1[0].im = u1[0].im;
*/

if (xD[0]==1) {
    # ifndef MATLAB_MEX_FILE
        rawPos = analogRead(sensorPosPin[0]);  //current raw position from MR sensor

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
        if(rawOffset > flipThresh) { // check to see if the data was good and the most current offset is above the threshold
        updatedPos = rawPos + flipNumber*rawOffset; // update the pos value to account for flips over 180deg using the most current offset 
        tempOffset = rawOffset;
        } else {                     // in this case there was a blip in the data and we want to use lastactualOffset instead
        updatedPos = rawPos + flipNumber*lastRawOffset;  // update the pos value to account for any flips over 180deg using the LAST offset
        tempOffset = lastRawOffset;
        }
        flipped = true;            // set boolean so that the next time through the loop won't trigger a flip
        } else {                        // anytime no flip has occurred
        updatedPos = rawPos + flipNumber*tempOffset; // need to update pos based on what most recent offset is 
        flipped = false;
        }
        
        updatedPos_out[0] = updatedPos;
        
        Tp = (double)Tp_in[0];
        Xh = (double)Xh_in[0];
        var1 = (double)var1_in[0];
        var2 = (double)var2_in[0];
        
        //Tp_out[0] = Tp;
        duty = sqrt(abs(Tp)/0.0183);
        
        if (duty > 1) 
        {            
            duty = 1;
        }
        else if (duty < 0)
        { 
            duty = 0;
        }  
        
        if(Tp > 0)
        {
            digitalWrite(dirPin[0], HIGH);
        }
        else
        {
            digitalWrite(dirPin[0], LOW);
        }
        
        
        //duty_out[0] = (int)((duty * 255));
        //OCR2B = (int)(duty* 255);  // Set duty cycle
        analogWrite(pwmPin[0],(int)(duty* 255));  // output the signal
        //OCR2B = 0;  // Set duty cycle
        
        //digitalWrite(5, !digitalRead(5));
    # endif
}
/* %%%-SFUNWIZ_wrapper_Outputs_Changes_END --- EDIT HERE TO _BEGIN */
}

/*
 * Updates function
 *
 */
void sfcn_HapkitB_T3_Update_wrapper(const real32_T *Tp_in,
			const real32_T *Xh_in,
			const real32_T *var1_in,
			const real32_T *var2_in,
			real32_T *updatedPos_out,
			real_T *xD,
			const uint8_T *sensorPosPin, const int_T p_width0,
			const uint8_T *dirPin, const int_T p_width1,
			const uint8_T *pwmPin, const int_T p_width2)
{
/* %%%-SFUNWIZ_wrapper_Update_Changes_BEGIN --- EDIT HERE TO _END */
/*
 * Code example
 *   xD[0] = u0[0];
 */

if (xD[0]!=1) {
    # ifndef MATLAB_MEX_FILE
        
        unsigned int pin = pwmPin[0];
        int divisor = 1;
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
    
        pinMode(pwmPin[0], OUTPUT);  // PWM pin for motor A
        pinMode(dirPin[0], OUTPUT);  // dir pin for motor A
        
        analogWrite(pwmPin[0], 0);     // set to not be spinning (0/255)
        digitalWrite(dirPin[0], LOW);  // set direction
        
        pinMode(5,OUTPUT);

        //updatedPos_out[0] = 0;
        
        
    # endif
        /* initialization done */ 
    xD[0]=1;
}
/* %%%-SFUNWIZ_wrapper_Update_Changes_END --- EDIT HERE TO _BEGIN */
}
/*
 * Terminate function
 *
 */
void sfcn_HapkitB_T3_Terminate_wrapper(real_T *xD,
			const uint8_T *sensorPosPin, const int_T p_width0,
			const uint8_T *dirPin, const int_T p_width1,
			const uint8_T *pwmPin, const int_T p_width2)
{
/* %%%-SFUNWIZ_wrapper_Terminate_Changes_BEGIN --- EDIT HERE TO _END */
/*
 * Custom Terminate code goes here.
 */
/*
    #ifndef MATLAB_MEX_FILE
    
    //Input[0] = 0;
    OCR2B = 240;
    
    #endif
*/
/* %%%-SFUNWIZ_wrapper_Terminate_Changes_END --- EDIT HERE TO _BEGIN */
}

