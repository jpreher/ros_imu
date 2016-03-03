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
  float t31;
  float t32;
  float t33;
  float t34;
  float t35;
  t1 = x[7];
  t2 = -0.5*t1;
  t3 = x[6];
  t4 = 0.5*t3;
  t5 = x[9];
  t6 = 0.5*t5;
  t7 = x[8];
  t8 = -0.5*t7;
  t9 = -0.5*t5;
  t10 = 0.5*t1;
  t11 = 0.5*t7;
  t12 = x[0];
  t13 = 0.5*t12;
  t14 = x[1];
  t15 = 0.5*t14;
  t16 = x[2];
  t17 = 0.5*t16;
  t18 = -0.5*t12;
  t19 = x[3];
  t20 = -0.5*t16;
  t21 = x[5];
  t22 = x[4];
  t23 = 0.5*t22;
  t24 = t15 + t23;
  t25 = -0.5*t14;
  t26 = 0.5*t21;
  t27 = t17 + t26;
  t28 = -0.5*t19;
  t29 = t18 + t28;
  t30 = -0.5*t21;
  t31 = t20 + t30;
  t32 = -0.5*t22;
  t33 = t25 + t32;
  t34 = 0.5*t19;
  t35 = t13 + t34;
  Eigen::Map<Eigen::VectorXf> foo(p_output1, 324);
  foo << List(List(1.,0,0,0,0,0,0,0,0,0,t2,t4,t6,t8,t2,t4,t6,t8,0,1.,0,0,0,0,0,0,0,0,t8,t9,t4,t10,t8,t9,t4,t10,0,0,1.,0,0,0,0,0,0,0,t9,t11,t2,t4,t9,t11,t2,t4,dt[0],0,0,1.,0,0,0,0,0,0,0,0,0,0,t2,t4,t6,t8,0,dt[0],0,0,1.,0,0,0,0,0,0,0,0,0,t8,t9,t4,t10,0,0,dt[0],0,0,1.,0,0,0,0,0,0,0,0,t9,t11,t2,t4,0,0,0,0,0,0,1.,0,0,0,0,t13,t15,t17,0,t35,t24,t27,0,0,0,0,0,0,0,1.,0,0,t18,0,t20,t15,t29,0,t31,t24,0,0,0,0,0,0,0,0,1.,0,t25,t17,0,t18,t33,t27,0,t29,0,0,0,0,0,0,0,0,0,1.,t20,t25,t13,0,t31,t33,t35,0,0,0,0,0,0,0,dt[0],0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,dt[0],0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,dt[0],0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,dt[0],0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0));
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
  plhs[0] = mxCreatefloatMatrix((mwSize) 18, (mwSize) 18, mxREAL);
  p_output1 = mxGetPr(plhs[0]);


  /* Call the calculation subroutine. */
  output1(p_output1,x,dt);


}

#else // MATLAB_MEX_FILE

#include "Amat.hh"

namespace model_ekf
{
namespace basic
{

void Amat_raw(float *p_output1, const float *x,const float *dt)
{
  // Call Subroutines
  output1(p_output1, x, dt);

}

}
}

#endif // MATLAB_MEX_FILE