#ifndef EKF_H
#define EKF_H

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Geometry>

using namespace Eigen;

class EKF {
public:
    Matrix<float, 18, 1>    x_hat;

    EKF();
    ~EKF();

    void initialize(Matrix<float, 18, 1> &x_init, Matrix<float, 18, 18> &P_init, Matrix<float, 18, 18> &Q_init, Matrix<float, 16, 16> &R_init, Matrix<float, 3, 1> &r_init);
    void reset();
    int  update(float dt, Matrix<float, 16, 1> &acc, Matrix<float, 16,1> &measurement);

private:
    Matrix<float, 18, 1>    x_minus;
    Matrix<float, 16, 1>    h;
    Matrix<float, 3, 1>     r;

    Matrix<float, 18, 16>    K_;
    Matrix<float, 18, 18>   Q_;
    Matrix<float, 16, 16>   R_;
    Matrix<float, 18, 18>   A_;
    Matrix<float, 16, 18>   H_;
    Matrix<float, 18, 18>   P_;
    Matrix<float, 18, 18>   P_minus_;
    Matrix<float, 18, 18>   I_;

    bool isInit;
};


#endif //EKF_H
