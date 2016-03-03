/*=====================================================================================================
// Model Based EKF.
//=====================================================================================================
//
// Class which facilitates the use of a model-based EKF to get orientation of leg segments.
//
// Author: Jake Reher
//=====================================================================================================*/
//---------------------------------------------------------------------------------------------------

#include <imu_common/EKF.hpp>

EKF::EKF() {
    x_hat.setZero();
    x_minus.setZero();
    h.setZero();
    r.setZero();
    K_.setZero();
    Q_.setZero();
    R_.setZero();
    A_.setZero();
    H_.setZero();
    P_.setZero();
    P_minus_.setZero();
    I_.setZero();

    isInit = false;
}

EKF::~EKF(){}

void EKF::initialize(Matrix<float, 18, 1> &x_init, Matrix<float, 18, 18> &P_init, Matrix<float, 18, 18> &Q_init, Matrix<float, 16, 16> &R_init, Matrix<float, 3, 1> &r_init){
    x_minus     << x_init;
    P_          << P_init;
    Q_          << Q_init;
    R_          << R_init;
    r           << r_init;

    isInit = true;
}

void EKF::reset(){
    x_hat.setZero();
    x_minus.setZero();
    h.setZero();
    r.setZero();
    K_.setZero();
    Q_.setZero();
    R_.setZero();
    A_.setZero();
    H_.setZero();
    P_.setZero();
    P_minus_.setZero();
    I_.setZero();

    isInit = false;
}

int EKF::update(float dt, Matrix<float, 3, 1> &acc, Matrix<float, 16,1> &measurement) {
    if (isInit){
        // Update the model
        model_ekf::basic::Amat_raw(A_.data(), x_hat.data(), &dt);
        model_ekf::basic::fvec_raw(x_minus.data(), x_hat.data(), &dt);

        // Project the error covariance ahead
        P_minus_ = A_ * P_ * A_.transpose() + Q_;

        // Calculate H
        model_ekf::basic::Hmat_raw(H_.data(), measurement.data(), acc.data(), r.data());
        model_ekf::basic::hvec_raw(h.data(), measurement.data(), acc.data(), r.data());

        // Calculate the Kalman gain
        K_ = (P_minus_ * H_.transpose()) * (H_ * P_minus_ * H_.transpose() + R_).inverse();

        x_hat = x_minus + K_ * (measurement - h);

        // Update the error covariance
        P_ = (I_ - K_ * H_) * P_minus_;

        return 1;
    } else {
        // Load in values from YAML (TODO)
        return 0;
    }
}
