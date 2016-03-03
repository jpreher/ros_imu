#include <Eigen/Dense>
#include <math2mat.hpp>

#ifdef MATLAB_MEX_FILE
// No need for external definitions
#else // MATLAB_MEX_FILE

namespace model_ekf
{
namespace basic
{

void hvec_raw(float *p_output1, const float *x,const float *rad);

inline void hvec(Eigen::MatrixXf &p_output1, const Eigen::VectorXf &x,const Eigen::VectorXf &rad)
{
  // Check
  // - Inputs
  assert_size_matrix(x, 16, 1);
  assert_size_matrix(rad, 3, 1);

  // - Outputs
  assert_size_matrix(p_output1, 16, 1);

  // Call Subroutine with raw data
  hvec_raw(p_output1.data(), x.data(),rad.data());
}

}
}

#endif // MATLAB_MEX_FILE