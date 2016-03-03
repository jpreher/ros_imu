#include "math2mat.hpp"

/*
 * Template for compiling Mathematica expressions into mex C code
 */

/*
 * Sub functions
 */
static void output1(float *p_output1,const float *x,const float *a,const float *rad)
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
  t1 = rad[1];
  t2 = x[1];
  t3 = rad[2];
  t4 = x[0];
  t5 = rad[0];
  t6 = x[2];
  t7 = t3*t6;
  t8 = t5*t4;
  t9 = t1*t2;
  t10 = x[9];
  t11 = x[13];
  t12 = x[6];
  t13 = x[14];
  t14 = x[15];
  t15 = x[8];
  t16 = x[7];
  t17 = 2.*t15*t11;
  t18 = -2.*t16*t13;
  t19 = 2.*t12*t14;
  t20 = t17 + t18 + t19;
  t21 = 2.*t16*t11;
  t22 = 2.*t15*t13;
  t23 = 2.*t10*t14;
  t24 = t21 + t22 + t23;
  t25 = 2.*t12*t11;
  t26 = 2.*t10*t13;
  t27 = -2.*t15*t14;
  t28 = t25 + t26 + t27;
  t29 = -2.*t10*t11;
  t30 = 2.*t12*t13;
  t31 = 2.*t16*t14;
  t32 = t29 + t30 + t31;
  Eigen::Map<Eigen::VectorXf> foo(p_output1, 288);
  foo << List(List(1.,0,0,0,0,0,0,0,0,0,t7 + t9,-2.*t1*t4 + t2*t5,-2.*t3*t4 + t5*t6,0,0,0,0,1.,0,0,0,0,0,0,0,0,t1*t4 - 2.*t2*t5,t7 + t8,-2.*t2*t3 + t1*t6,0,0,0,0,0,1.,0,0,0,0,0,0,0,t3*t4 - 2.*t5*t6,t2*t3 - 2.*t1*t6,t8 + t9,0,0,0,0,0,0,1.,0,0,0,0,0,0,0,-1.*t3,t1,0,0,0,0,0,0,0,1.,0,0,0,0,0,t3,0,-1.*t5,0,0,0,0,0,0,0,0,1.,0,0,0,0,-1.*t1,t5,0,0,0,0,0,0,0,0,0,0,1.,0,0,0,0,0,0,t28,t32,t20,0,0,0,0,0,0,0,1.,0,0,0,0,0,t24,t20,2.*t10*t11 - 2.*t12*t13 - 2.*t14*t16,0,0,0,0,0,0,0,0,1.,0,0,0,0,-2.*t12*t14 - 2.*t11*t15 + 2.*t13*t16,t24,t28,0,0,0,0,0,0,0,0,0,1.,0,0,0,t32,-2.*t11*t12 - 2.*t10*t13 + 2.*t14*t15,t24,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0));
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

  float *x,*a,*rad;
  float *p_output1;

  /*  Check for proper number of arguments.  */ 
  if( nrhs != 3)
    {
      mexErrMsgIdAndTxt("MATLAB:MShaped:invalidNumInputs", "Three input(s) required (x,a,rad).");
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
      mexErrMsgIdAndTxt( "MATLAB:MShaped:inputNotRealVector", "a is wrong.");
    }
  mrows = mxGetM(prhs[2]);
  ncols = mxGetN(prhs[2]);
  if( !mxIsfloat(prhs[2]) || mxIsComplex(prhs[2]) ||
    ( !(mrows == 3 && ncols == 1) && 
      !(mrows == 1 && ncols == 3))) 
    {
      mexErrMsgIdAndTxt( "MATLAB:MShaped:inputNotRealVector", "rad is wrong.");
    }

  /*  Assign pointers to each input.  */
  x = mxGetPr(prhs[0]);
  a = mxGetPr(prhs[1]);
  rad = mxGetPr(prhs[2]);
   


   
  /*  Create matrices for return arguments.  */
  plhs[0] = mxCreatefloatMatrix((mwSize) 16, (mwSize) 18, mxREAL);
  p_output1 = mxGetPr(plhs[0]);


  /* Call the calculation subroutine. */
  output1(p_output1,x,a,rad);


}

#else // MATLAB_MEX_FILE

#include "Hmat.hh"

namespace model_ekf
{
namespace basic
{

void Hmat_raw(float *p_output1, const float *x,const float *a,const float *rad)
{
  // Call Subroutines
  output1(p_output1, x, a, rad);

}

}
}

#endif // MATLAB_MEX_FILE