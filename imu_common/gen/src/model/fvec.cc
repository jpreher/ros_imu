#include "math2mat.hpp"

/*
 * Template for compiling Mathematica expressions into mex C code
 */

/*
 * Sub functions
 */
static void output1(float *p_output1,const float *x,const float *dt)
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
  t1 = x[3];
  t2 = x[4];
  t3 = x[5];
  t4 = x[7];
  t5 = x[0];
  t6 = x[8];
  t7 = x[1];
  t8 = x[9];
  t9 = x[2];
  t10 = x[6];
  t11 = -1.*t4*t5;
  t12 = -1.*t6*t7;
  t13 = -1.*t8*t9;
  t14 = t11 + t12 + t13;
  t15 = 0.5*t14;
  t16 = t10*t5;
  t17 = -1.*t8*t7;
  t18 = t6*t9;
  t19 = t16 + t17 + t18;
  t20 = 0.5*t19;
  t21 = t8*t5;
  t22 = t10*t7;
  t23 = -1.*t4*t9;
  t24 = t21 + t22 + t23;
  t25 = 0.5*t24;
  t26 = -1.*t6*t5;
  t27 = t4*t7;
  t28 = t10*t9;
  t29 = t26 + t27 + t28;
  t30 = 0.5*t29;
  Eigen::Map<Eigen::VectorXf> foo(p_output1, 18);
  foo << List(List(t5 + t1*dt[0],t7 + t2*dt[0],t9 + t3*dt[0],t1,t2,t3,t10 + dt[0]*x[10],t4 + dt[0]*x[11],t6 + dt[0]*x[12],t8 + dt[0]*x[13],t15,t20,t25,t30,t15 + 0.5*(-1.*t1*t4 - 1.*t2*t6 - 1.*t3*t8),t20 + 0.5*(t1*t10 + t3*t6 - 1.*t2*t8),t25 + 0.5*(t10*t2 - 1.*t3*t4 + t1*t8),t30 + 0.5*(t10*t3 + t2*t4 - 1.*t1*t6)));
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

  float *x,*dt;
  float *p_output1;

  /*  Check for proper number of arguments.  */ 
  if( nrhs != 2)
    {
      mexErrMsgIdAndTxt("MATLAB:MShaped:invalidNumInputs", "Two input(s) required (x,dt).");
    }
  else if( nlhs > 1)
    {
      mexErrMsgIdAndTxt("MATLAB:MShaped:maxlhs", "Too many output arguments.");
    }

  /*  The input must be a noncomplex float vector or scaler.  */
  mrows = mxGetM(prhs[0]);
  ncols = mxGetN(prhs[0]);
  if( !mxIsfloat(prhs[0]) || mxIsComplex(prhs[0]) ||
    ( !(mrows == 18 && ncols == 1) && 
      !(mrows == 1 && ncols == 18))) 
    {
      mexErrMsgIdAndTxt( "MATLAB:MShaped:inputNotRealVector", "x is wrong.");
    }
  mrows = mxGetM(prhs[1]);
  ncols = mxGetN(prhs[1]);
  if( !mxIsfloat(prhs[1]) || mxIsComplex(prhs[1]) ||
    ( !(mrows == 1 && ncols == 1) && 
      !(mrows == 1 && ncols == 1))) 
    {
      mexErrMsgIdAndTxt( "MATLAB:MShaped:inputNotRealVector", "dt is wrong.");
    }

  /*  Assign pointers to each input.  */
  x = mxGetPr(prhs[0]);
  dt = mxGetPr(prhs[1]);
   


   
  /*  Create matrices for return arguments.  */
  plhs[0] = mxCreatefloatMatrix((mwSize) 18, (mwSize) 1, mxREAL);
  p_output1 = mxGetPr(plhs[0]);


  /* Call the calculation subroutine. */
  output1(p_output1,x,dt);


}

#else // MATLAB_MEX_FILE

#include "fvec.hh"

namespace model_ekf
{
namespace basic
{

void fvec_raw(float *p_output1, const float *x,const float *dt)
{
  // Call Subroutines
  output1(p_output1, x, dt);

}

}
}

#endif // MATLAB_MEX_FILE