#ifndef EKF_H
#define EKF_H

#include <ros/ros.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Geometry>

using namespace Eigen;

class EKF {
public:
    Matrix<float, 14, 1> x_hat;
    Matrix<float, 14, 1> x_minus;
    Matrix<float, 9, 1> h;
    Matrix<float, 3, 1> r;
    Matrix<float, 9, 1> z;

    EKF();
    void initialize(Matrix<float, 14, 1> &x_init, Matrix<float, 14, 14> &P_init, Matrix<float, 14, 14> &Q_init, Matrix<float, 9, 9> &R_init, Matrix<float, 3, 1> &r_init);
    void update(float dt, Matrix<float, 3, 1> &acc, Matrix<float, 9,1> &measurement);

//private:
    Matrix<float, 14, 9>    K_;
    Matrix<float, 14, 14>   Q_;
    Matrix<float, 9, 9>     R_;
    Matrix<float, 14, 14>   A_;
    Matrix<float, 9, 14>    H_;
    Matrix<float, 14, 14>   P_;
    Matrix<float, 14, 14>   P_minus_;
    Matrix<float, 14, 14>   I_;

    float g;

    static float invSqrt(float x);
};





#endif //EKF_H

