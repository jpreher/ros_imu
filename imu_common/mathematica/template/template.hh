#include <Eigen/Dense>
#include <math2mat.hpp>

#ifdef MATLAB_MEX_FILE
// No need for external definitions
#else // MATLAB_MEX_FILE

namespace <*`namespace`*>
{
namespace <*`behavior`*>
{

void <*`name`*>_raw(<*StringImplode[Table["float *p_" <> `argouts`[[i]], {i, Length[`argouts`]}], ", "]*>, <*StringImplode[Table["const float *"<>ToString[arg], {arg, `argins`}], ","]*>);

inline void <*`name`*>(<*StringImplode[Table["Eigen::MatrixXf &p_" <> `argouts`[[i]], {i, Length[`argouts`]}], ", "]*>, <*StringImplode[Table["const Eigen::VectorXf &"<>ToString[`argins`[[i]]], {i, Length[`argins`]}], ","]*>)
{
  // Check
  // - Inputs
<*StringJoin@@Table[
"  assert_size_matrix(" <> ToString[`argins`[[i]]] <> ", " <> ToString[`arginDims`[[i,1]]] <> ", " <> ToString[`arginDims`[[i,2]]] <> ");\n"
  , {i, Length[`argins`]}
]*>
  // - Outputs
<*StringJoin@@Table[
"  assert_size_matrix(p_" <> ToString[`argouts`[[i]]] <> ", " <> ToString[`argoutDims`[[i,1]]] <> ", " <> ToString[`argoutDims`[[i,2]]] <> ");\n"
  , {i, Length[`argouts`]}
]*>
  // Call Subroutine with raw data
  <*`name`*>_raw(<*StringImplode[Table["p_" <> `argouts`[[i]] <> ".data()", {i, Length[`argouts`]}], ", "]*>, <*StringImplode[Table[ToString[arg] <> ".data()", {arg, `argins`}], ","]*>);
}

}
}

#endif // MATLAB_MEX_FILE
