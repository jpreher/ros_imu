#include <mex.h>
#include <matrix.h>
#include <Eigen/Dense>

#include <sstream>
#include <stdexcept>

#include "@EXPR_HEADER@"

#ifdef _WIN32
    #define PRETTY_FUNC_NAME __FUNCDNAME__
#else // __linux
    #define PRETTY_FUNC_NAME __PRETTY_FUNCTION__
#endif

/// @note From eigen_utilities
/// @todo Place in another package
#define common_assert_msg(expr, msg) \
    if (!(expr)) \
    { \
        std::ostringstream os; \
        os << "Assert Error: " << #expr << std::endl << \
            "File: " << __FILE__ ":" << __LINE__ << std::endl << \
            "Function: " << PRETTY_FUNC_NAME << std::endl << \
            "Message: " << msg << std::endl; \
        throw std::runtime_error(os.str()); \
    }

#define common_assert(expr) \
    common_assert_msg(expr, "[None]")


// Modified From: http://svn.anu.edu.au/AI/drwn/trunk/projects/matlab/drwnMatlabUtils.h
inline void mxArrayToEigen(const mxArray *m, Eigen::MatrixXd &A)
{
    const int nRows = mxGetM(m); // number of rows
    const int nCols = mxGetN(m); // number of columns
    A = Eigen::Map<Eigen::MatrixXd>(mxGetPr(m), nRows, nCols);
}
inline void mxArrayToEigen(const mxArray *m, Eigen::VectorXd &b)
{
    const int nRows = mxGetM(m); // number of rows
    const int nCols = mxGetN(m); // number of columns
    b = Eigen::Map<Eigen::VectorXd>(mxGetPr(m), nRows * nCols);
}

inline mxArray* mxEigenToArray(const Eigen::MatrixXd &A)
{
    const int nRows = A.rows(), nCols = A.cols();
    mxArray *m = mxCreateDoubleMatrix(nRows, nCols, mxREAL);
    double *p = mxGetPr(m);
    for (int i = 0; i < nRows; i++)
        for (int j = 0; j < nCols; j++)
            p[i + j * nRows] = A(i, j);
    return m;
}
inline mxArray* mxEigenToArray(const Eigen::VectorXd &b)
{
    const int nRows = b.rows();
    mxArray *m = mxCreateDoubleMatrix(nRows, 1, mxREAL);
    double *p = mxGetPr(m);
    for (int i = 0; i < nRows; i++)
        p[i] = b(i);
    return m;
}

inline mxArray *to_mex(double value)
{
    return mxCreateDoubleScalar(value);
}
inline void from_mex(const mxArray *m, double &value)
{
    value = mxGetScalar(m);
}

inline mxArray *to_mex(const Eigen::VectorXd &X)
{
    return mxEigenToArray(X);
}
inline void from_mex(const mxArray *m, Eigen::VectorXd &X)
{
    mxArrayToEigen(m, X);
}

inline mxArray *to_mex(const Eigen::MatrixXd &X)
{
    return mxEigenToArray(X);
}
inline void from_mex(const mxArray *m, Eigen::MatrixXd &X)
{
    mxArrayToEigen(m, X);
}

void mexFunction(int nargout, mxArray *argout[], int nargin, const mxArray *argin[])
{
    try
    {
        common_assert(nargin == @EXPR_NARGIN@);
        common_assert(nargout == 1);

@EXPR_IN_DECL@

@EXPR_OUT_DECL@
        @EXPR_NAMESPACE@::@EXPR_NAME@(@EXPR_ARGS_PASS@);
        argout[0] = to_mex(output_value);
    }
    catch (const std::exception &e)
    {
        std::string str = "[ C++ Exception ]\n" + std::string(e.what());
        mexErrMsgTxt(str.c_str());
    }
}
