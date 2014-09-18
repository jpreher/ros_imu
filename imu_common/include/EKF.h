#ifndef EKF_H
#define EKF_H

#include <ros/ros.h>
#include "MPU9150.h"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Geometry>

using namespace Eigen;

class EKF {
public:
	VectorXd x_hat;
	VectorXd x_minus;
	VectorXd h;
	VectorXd r;
	VectorXd z;

    EKF();
    void initialize(VectorXd &x_init, MatrixXd &P_init, MatrixXd &Q_init, MatrixXd &R_init, VectorXd &r_init);
    void update(double dt, VectorXd &acc, VectorXd &measurement);

//private:
	MatrixXd K_;
	MatrixXd Q_;
	MatrixXd R_;
	MatrixXd A_;
	MatrixXd H_;
    MatrixXd P_;
    MatrixXd P_minus_;
	MatrixXd I_;
    double g;

    static float invSqrt(float x);
};





#endif //EKF_H
