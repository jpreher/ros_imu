#include "math2mat.hpp"

/*
 * Template for compiling Mathematica expressions into mex C code
 */

/*
 * Sub functions
 */
static void output1(double *p_output1,const double *x)
{
  double t1;
  double t2;
  double t3;
  double t4;
  double t5;
  double t6;
  double t7;
  double t8;
  double t9;
  double t10;
  double t11;
  double t12;
  double t13;
  double t14;
  double t15;
  double t16;
  double t17;
  double t18;
  double t19;
  double t20;
  double t21;
  double t22;
  double t23;
  double t24;
  double t25;
  double t26;
  double t27;
  double t28;
  double t29;
  double t30;
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
  Eigen::Map<Eigen::VectorXd> foo(p_output1, 18);
  foo << List(List(dt*t1 + t5,dt*t2 + t7,dt*t3 + t9,t1,t2,t3,t10 + dt*x[10],t4 + dt*x[11],t6 + dt*x[12],t8 + dt*x[13],t15,t20,t25,t30,t15 + 0.5*(-1.*t1*t4 - 1.*t2*t6 - 1.*t3*t8),t20 + 0.5*(t1*t10 + t3*t6 - 1.*t2*t8),t25 + 0.5*(t10*t2 - 1.*t3*t4 + t1*t8),t30 + 0.5*(t10*t3 + t2*t4 - 1.*t1*t6)));
}

static void output2(double *p_output2,const double *x)
{
  double t1;
  double t2;
  double t3;
  double t4;
  double t5;
  double t6;
  double t7;
  double t8;
  double t9;
  double t10;
  double t11;
  double t12;
  double t13;
  double t14;
  double t15;
  double t16;
  double t17;
  double t18;
  double t19;
  double t20;
  double t21;
  double t22;
  double t23;
  double t24;
  double t25;
  double t26;
  double t27;
  double t28;
  double t29;
  double t30;
  double t31;
  double t32;
  t1 = x[0];
  t2 = x[1];
  t3 = x[2];
  t4 = x[3];
  t5 = x[4];
  t6 = x[5];
  t7 = Cross(List(t1,t2,t3,r0));
  t8 = Cross(List(t1,t2,t3,t7));
  t9 = Cross(List(t4,t5,t6,r0));
  t10 = x[8];
  t11 = x[7];
  t12 = x[6];
  t13 = x[9];
  t14 = Subscript(B,1(t));
  t15 = Subscript(B,2(t));
  t16 = Subscript(B,3(t));
  t17 = t10*t14;
  t18 = -1.*t11*t15;
  t19 = t12*t16;
  t20 = t17 + t18 + t19;
  t21 = -1.*t13*t14;
  t22 = t12*t15;
  t23 = t11*t16;
  t24 = t21 + t22 + t23;
  t25 = t12*t14;
  t26 = t13*t15;
  t27 = -1.*t10*t16;
  t28 = t25 + t26 + t27;
  t29 = t11*t14;
  t30 = t10*t15;
  t31 = t13*t16;
  t32 = t29 + t30 + t31;
  Eigen::Map<Eigen::VectorXd> foo(p_output2, 16);
  foo << List(List(t1,t2,t3,t4,t5,t6,t12,t11,t10,t13,t8 + t9 + Subscript(a,1)(t),t8 + t9 + Subscript(a,2)(t),-1.*g + t8 + t9 + Subscript(a,3)(t),-1.*t10*t20 + t13*t24 + t12*t28 + t11*t32,t11*t20 + t12*t24 - 1.*t13*t28 + t10*t32,t12*t20 - 1.*t11*t24 + t10*t28 + t13*t32));
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

  double *x;
  double *p_output1,*p_output2;

  /*  Check for proper number of arguments.  */ 
  if( nrhs != 1)
    {
      mexErrMsgIdAndTxt("MATLAB:MShaped:invalidNumInputs", "One input(s) required (x).");
    }
  else if( nlhs > 2)
    {
      mexErrMsgIdAndTxt("MATLAB:MShaped:maxlhs", "Too many output arguments.");
    }

  /*  The input must be a noncomplex double vector or scaler.  */
  mrows = mxGetM(prhs[0]);
  ncols = mxGetN(prhs[0]);
  if( !mxIsDouble(prhs[0]) || mxIsComplex(prhs[0]) ||
    ( !(mrows == 18 && ncols == 1) && 
      !(mrows == 1 && ncols == 18))) 
    {
      mexErrMsgIdAndTxt( "MATLAB:MShaped:inputNotRealVector", "x is wrong.");
    }

  /*  Assign pointers to each input.  */
  x = mxGetPr(prhs[0]);
   


   
  /*  Create matrices for return arguments.  */
  plhs[0] = mxCreateDoubleMatrix((mwSize) 18, (mwSize) 1, mxREAL);
  p_output1 = mxGetPr(plhs[0]);
  plhs[1] = mxCreateDoubleMatrix((mwSize) 16, (mwSize) 1, mxREAL);
  p_output2 = mxGetPr(plhs[1]);


  /* Call the calculation subroutine. */
  output1(p_output1,x);
  output2(p_output2,x);


}

#else // MATLAB_MEX_FILE

#include "fvec.hh"

namespace durus_3d_expr
{
namespace basic
{

void fvec_raw(double *p_output1, double *p_output2, const double *x)
{
  // Call Subroutines
  output1(p_output1, x);
  output2(p_output2, x);

}

}
}

#endif // MATLAB_MEX_FILE