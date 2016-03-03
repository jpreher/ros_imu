#include "math2mat.hpp"

/*
 * Template for compiling Mathematica expressions into mex C code
 */

/*
 * Sub functions
 */
<* StringJoin@@Table[
"static void "<>`argouts`[[i]]<>"(float *p_"<>`argouts`[[i]]<>","<>StringJoin[Riffle[StringJoin["const float *", ToString[#]] & /@ `argins`, ","]]<>")\n"<>
"{\n"<>
StringJoin[Riffle[StringJoin["  float ",ToString[#]]&/@`lvars`[[i]],";\n"],";\n"]<>
StringJoin@@{"  ", Riffle[`statements`[[i]], ";\n  "], ";\n"}<>
"  Eigen::Map<Eigen::VectorXf> foo(p_"<>`argouts`[[i]]<>", "<>ToString[`argoutDims`[[i,1]]*`argoutDims`[[i,2]]]<>");\n"<>
"  foo << "<>`final`[[i]]<>";\n"<>
"}\n\n"
,
{i,Length[`argouts`]}]
*>

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

  float <* StringJoin[Riffle[StringJoin["*", ToString[#]] & /@ `argins`, ","]] *>;
  float <* StringJoin[Riffle[StringJoin["*p_", #] & /@ `argouts`, ","]] *>;

  /*  Check for proper number of arguments.  */ 
  if( nrhs != <* Length[`argins`] *>)
    {
      mexErrMsgIdAndTxt("MATLAB:MShaped:invalidNumInputs", "<* Length[`argins`]/.{1 -> "One", 2 -> "Two", 3 -> "Three", 4 -> "Four", 5 -> "Five", 
 6 -> "Six", 7 -> "Seven", 8 -> "Eight", 9 -> "Nine"} *> input(s) required (<* StringJoin[ToString[#] & /@ Riffle[`argins`, ","]] *>).");
    }
  else if( nlhs > <* Length[`argouts`] *>)
    {
      mexErrMsgIdAndTxt("MATLAB:MShaped:maxlhs", "Too many output arguments.");
    }

  /*  The input must be a noncomplex float vector or scaler.  */
<* StringJoin@@Table[
  "  mrows = mxGetM(prhs["<>ToString[i-1]<>"]);\n"<>
  "  ncols = mxGetN(prhs["<>ToString[i-1]<>"]);\n"<>
  "  if( !mxIsfloat(prhs["<>ToString[i-1]<>"]) || mxIsComplex(prhs["<>ToString[i-1]<>"]) ||\n"<>
  "    ( !(mrows == "<>ToString[`arginDims`[[i,1]]]<>" && ncols == "<>ToString[`arginDims`[[i,2]]]<>") && \n"<>
  "      !(mrows == "<>ToString[`arginDims`[[i,2]]]<>" && ncols == "<>ToString[`arginDims`[[i,1]]]<>"))) \n"<>
  "    {\n"<>
  "      mexErrMsgIdAndTxt( \"MATLAB:MShaped:inputNotRealVector\", \""<>ToString[`argins`[[i]]]<>" is wrong.\");\n"<>
  "    }\n", {i, Length[`argins`]}]
*>
  /*  Assign pointers to each input.  */
<* StringJoin@@Table[
  "  "<>ToString[`argins`[[i]]]<>" = mxGetPr(prhs["<>ToString[i-1]<>"]);\n", {i, Length[`argins`]}]
*>   


   
  /*  Create matrices for return arguments.  */
<* StringJoin@@Table[
  "  plhs["<>ToString[i-1]<>"] = mxCreatefloatMatrix((mwSize) "<>ToString[`argoutDims`[[i,1]]]<>", (mwSize) "<>ToString[`argoutDims`[[i,2]]]<>", mxREAL);\n"<>
  "  p_"<>`argouts`[[i]]<>" = mxGetPr(plhs["<>ToString[i-1]<>"]);\n", {i, Length[`argouts`]}]
*>

  /* Call the calculation subroutine. */
<* StringJoin@@Table[
  "  "<>`argouts`[[i]]<>"(p_"<>`argouts`[[i]]<>","<> StringJoin[ToString[#] & /@ Riffle[`argins`, ","]]<>");\n", {i, Length[`argouts`]}]
*>

}

#else // MATLAB_MEX_FILE

#include "<*`name`*>.hh"

namespace <*`namespace`*>
{
namespace <*`behavior`*>
{

void <*`name`*>_raw(<*StringImplode[Table["float *p_" <> `argouts`[[i]], {i, Length[`argouts`]}], ", "]*>, <*StringImplode[Table["const float *"<>ToString[arg], {arg, `argins`}], ","]*>)
{
  // Call Subroutines
<*StringJoin@@Table[
"  "<>`argouts`[[i]]<>"(p_"<>`argouts`[[i]]<>", "<> StringImplode[Table[ToString[arg], {arg, `argins`}], ", "]<>");\n"
  , {i, Length[`argouts`]}
]*>
}

}
}

#endif // MATLAB_MEX_FILE
