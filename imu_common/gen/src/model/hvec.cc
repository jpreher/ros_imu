#include "math2mat.hpp"

/*
 * Template for compiling Mathematica expressions into mex C code
 */

/*
 * Sub functions
 */
static void output1(float *p_output1,const float *x,const float *rad)
{
  float t1;
  float t2;
  float t3;
  float t4;
  float t5;
  float t6;
  float t7;
  float t8;
  float t9;
  float t10;
  float t11;
  float t12;
  float t13;
  float t14;
  float t15;
  float t16;
  float t17;
  float t18;
  float t19;
  float t20;
  float t21;
  float t22;
  float t23;
  float t24;
  float t25;
  float t26;
  float t27;
  float t28;
  float t29;
  float t30;
  float t31;
  float t32;
  float t33;
  float t34;
  float t35;
  t1 = x[0];
  t2 = x[1];
  t3 = x[2];
  t4 = rad[0];
  t5 = rad[2];
  t6 = x[4];
  t7 = rad[1];
  t8 = x[5];
  t9 = Power(t3,2);
  t10 = x[3];
  t11 = Power(t1,2);
  t12 = Power(t2,2);
  t13 = x[8];
  t14 = x[7];
  t15 = x[6];
  t16 = x[9];
  t17 = x[13];
  t18 = x[14];
  t19 = x[15];
  t20 = t13*t17;
  t21 = -1.*t14*t18;
  t22 = t15*t19;
  t23 = t20 + t21 + t22;
  t24 = -1.*t16*t17;
  t25 = t15*t18;
  t26 = t14*t19;
  t27 = t24 + t25 + t26;
  t28 = t15*t17;
  t29 = t16*t18;
  t30 = -1.*t13*t19;
  t31 = t28 + t29 + t30;
  t32 = t14*t17;
  t33 = t13*t18;
  t34 = t16*t19;
  t35 = t32 + t33 + t34;
  Eigen::Map<Eigen::VectorXf> foo(p_output1, 16);
  foo << List(List(t1,t2,t3,t10,t6,t8,t15,t14,t13,t16,-1.*t12*t4 + t1*t3*t5 + t5*t6 + t1*t2*t7 - 1.*t7*t8 - 1.*t4*t9 + x[10],t1*t2*t4 - 1.*t10*t5 + t2*t3*t5 - 1.*t11*t7 + t4*t8 - 1.*t7*t9 + x[11],-9.81 + t1*t3*t4 - 1.*t11*t5 - 1.*t12*t5 - 1.*t4*t6 + t10*t7 + t2*t3*t7 + x[12],-1.*t13*t23 + t16*t27 + t15*t31 + t14*t35,t14*t23 + t15*t27 - 1.*t16*t31 + t13*t35,t15*t23 - 1.*t14*t27 + t13*t31 + t16*t35));
}



#ifdef MATLAB_MEX_FILE

#include "mex.h"
#include "matrix.h"

/*
 * Main function
 */
void mexFunction( int nlhs, mxArray *plhs[],
                  int nrhs, const mxArray *prhs[] )
{
  size_t mrows, ncols;

  float *x,*rad;
  float *p_output1;

  /*  Check for proper number of arguments.  */ 
  if( nrhs != 2)
    {
      mexErrMsgIdAndTxt("MATLAB:MShaped:invalidNumInputs", "Two input(s) required (x,rad).");
    }
  else if( nlhs > 1)
    {
      mexErrMsgIdAndTxt("MATLAB:MShaped:maxlhs", "Too many output arguments.");
    }

  /*  The input must be a noncomplex float vector or scaler.  */
  mrows = mxGetM(prhs[0]);
  ncols = mxGetN(prhs[0]);
  if( !mxIsfloat(prhs[0]) || mxIsComplex(prhs[0]) ||
    ( !(mrows == 16 && ncols == 1) && 
      !(mrows == 1 && ncols == 16))) 
    {
      mexErrMsgIdAndTxt( "MATLAB:MShaped:inputNotRealVector", "x is wrong.");
    }
  mrows = mxGetM(prhs[1]);
  ncols = mxGetN(prhs[1]);
  if( !mxIsfloat(prhs[1]) || mxIsComplex(prhs[1]) ||
    ( !(mrows == 3 && ncols == 1) && 
      !(mrows == 1 && ncols == 3))) 
    {
      mexErrMsgIdAndTxt( "MATLAB:MShaped:inputNotRealVector", "rad is wrong.");
    }

  /*  Assign pointers to each input.  */
  x = mxGetPr(prhs[0]);
  rad = mxGetPr(prhs[1]);
   


   
  /*  Create matrices for return arguments.  */
  plhs[0] = mxCreatefloatMatrix((mwSize) 16, (mwSize) 1, mxREAL);
  p_output1 = mxGetPr(plhs[0]);


  /* Call the calculation subroutine. */
  output1(p_output1,x,rad);


}

#else // MATLAB_MEX_FILE

#include "hvec.hh"

namespace model_ekf
{
namespace basic
{

void hvec_raw(float *p_output1, const float *x,const float *rad)
{
  // Call Subroutines
  output1(p_output1, x, rad);

}

}
}

#endif // MATLAB_MEX_FILE