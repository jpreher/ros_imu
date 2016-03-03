#include <Eigen/Dense>
#include <math2mat.hpp>

#ifdef MATLAB_MEX_FILE
// No need for external definitions
#else // MATLAB_MEX_FILE

namespace model_ekf
{
namespace basic
{

void Hmat_raw(float *p_output1, const float *x,const float *a,const float *rad);

inline void Hmat(Eigen::MatrixXf &p_output1, const Eigen::VectorXf &x,const Eigen::VectorXf &a,const Eigen::VectorXf &rad)
{
  // Check
  // - Inputs
  assert_size_matrix(x, 16, 1);
  assert_size_matrix(a, 3, 1);
  assert_size_matrix(rad, 3, 1);

  // - Outputs
  assert_size_matrix(p_output1, 16, 18);

  // Call Subroutine with raw data
  Hmat_raw(p_output1.data(), x.data(),a.data(),rad.data());
}

}
}

#endif // MATLAB_MEX_FILE