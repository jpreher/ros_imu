#include <Eigen/Dense>
#include <math2mat.hpp>

#ifdef MATLAB_MEX_FILE
// No need for external definitions
#else // MATLAB_MEX_FILE

namespace model_ekf
{
namespace basic
{

void Amat_raw(float *p_output1, const float *x,const float *dt);

inline void Amat(Eigen::MatrixXf &p_output1, const Eigen::VectorXf &x,const Eigen::VectorXf &dt)
{
  // Check
  // - Inputs
  assert_size_matrix(x, 18, 1);
  assert_size_matrix(dt, 1, 1);

  // - Outputs
  assert_size_matrix(p_output1, 18, 18);

  // Call Subroutine with raw data
  Amat_raw(p_output1.data(), x.data(),dt.data());
}

}
}

#endif // MATLAB_MEX_FILE