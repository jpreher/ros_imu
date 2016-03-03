#include <Eigen/Dense>

#ifdef MATLAB_MEX_FILE
// No need for external definitions
#else // MATLAB_MEX_FILE

namespace durus_3d_expr
{
namespace basic
{

void fvec_raw(double *p_output1, double *p_output2, const double *x);

inline void fvec(Eigen::MatrixXd &p_output1, Eigen::MatrixXd &p_output2, const Eigen::VectorXd &x)
{
  // Check
  // - Inputs
  assert_size_matrix(x, 18, 1);

  // - Outputs
  assert_size_matrix(p_output1, 18, 1);
  assert_size_matrix(p_output2, 16, 1);

  // Call Subroutine with raw data
  fvec_raw(p_output1.data(), p_output2.data(), x.data());
}

}
}

#endif // MATLAB_MEX_FILE