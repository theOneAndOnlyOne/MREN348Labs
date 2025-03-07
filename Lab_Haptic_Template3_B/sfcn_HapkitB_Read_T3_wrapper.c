
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

#include <Arduino.h>

#endif
/* %%%-SFUNWIZ_wrapper_includes_Changes_END --- EDIT HERE TO _BEGIN */
#define y_width 1

/*
 * Create external references here.  
 *
 */
/* %%%-SFUNWIZ_wrapper_externs_Changes_BEGIN --- EDIT HERE TO _END */
/* extern double func(double a); */

extern double get_Tp();
extern double get_Xh();
extern double get_var1();
extern double get_var2();
/* %%%-SFUNWIZ_wrapper_externs_Changes_END --- EDIT HERE TO _BEGIN */

/*
 * Start function
 *
 */
void sfcn_HapkitB_Read_T3_Start_wrapper(real_T *xD)
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
void sfcn_HapkitB_Read_T3_Outputs_wrapper(real32_T *Tp,
			real32_T *Xh,
			real32_T *var1,
			real32_T *var2,
			const real_T *xD)
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
              
      //updatedPos[0] = get_updatedPos();
      Tp[0] = get_Tp();
      Xh[0] = get_Xh();
      var1[0] = get_var1();
      var2[0] = get_var2();
      
    # endif

}
/* %%%-SFUNWIZ_wrapper_Outputs_Changes_END --- EDIT HERE TO _BEGIN */
}

/*
 * Updates function
 *
 */
void sfcn_HapkitB_Read_T3_Update_wrapper(real32_T *Tp,
			real32_T *Xh,
			real32_T *var1,
			real32_T *var2,
			real_T *xD)
{
/* %%%-SFUNWIZ_wrapper_Update_Changes_BEGIN --- EDIT HERE TO _END */
/*
 * Code example
 *   xD[0] = u0[0];
 */

if (xD[0]!=1) {
    # ifndef MATLAB_MEX_FILE
        
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
void sfcn_HapkitB_Read_T3_Terminate_wrapper(real_T *xD)
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

