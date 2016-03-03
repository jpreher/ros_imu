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
  t1 = x[1];
  t2 = x[0];
  t3 = rad[0];
  t4 = x[2];
  t5 = rad[2];
  t6 = rad[1];
  t7 = Power(t4,2);
  t8 = x[5];
  t9 = Power(t2,2);
  t10 = Power(t1,2);
  t11 = x[3];
  t12 = x[4];
  Eigen::Map<Eigen::VectorXf> foo(p_output1, 3);
  foo << List(List(-1.*t10*t3 + t12*t5 + t2*t4*t5 + t1*t2*t6 - 1.*t3*t7 - 1.*t6*t8 + x[10],t1*t2*t3 - 1.*t11*t5 + t1*t4*t5 - 1.*t6*t7 + t3*t8 - 1.*t6*t9 + x[11],-9.81 - 1.*t12*t3 + t2*t3*t4 - 1.*t10*t5 + t11*t6 + t1*t4*t6 - 1.*t5*t9 + x[12]));
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
  plhs[0] = mxCreatefloatMatrix((mwSize) 3, (mwSize) 1, mxREAL);
  p_output1 = mxGetPr(plhs[0]);


  /* Call the calculation subroutine. */
  output1(p_output1,x,a,rad);


}

#else // MATLAB_MEX_FILE

#include "acc_link.hh"

namespace model_ekf
{
namespace basic
{

void acc_link_raw(float *p_output1, const float *x,const float *a,const float *rad)
{
  // Call Subroutines
  output1(p_output1, x, a, rad);

}

}
}

#endif // MATLAB_MEX_FILE