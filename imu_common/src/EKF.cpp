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
using namespace model_ekf::basic;
//using namespace basic;

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

    x_hat = x_minus;

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

bool EKF::update(float dt, Matrix<float, 3, 1> &acc, Matrix<float, 16,1> &measurement) {
    if (isInit){
        // Update the model
        Amat_raw(A_.data(), x_hat.data(), &dt);
        fvec_raw(x_minus.data(), x_hat.data(), &dt);

        // Project the error covariance ahead
        P_minus_ = A_ * P_ * A_.transpose() + Q_;

        // Calculate H
        Hmat_raw(H_.data(), measurement.data(), acc.data(), r.data());
        hvec_raw(h.data(), measurement.data(), acc.data(), r.data());

        // Calculate the Kalman gain
        K_ = (P_minus_ * H_.transpose()) * (H_ * P_minus_ * H_.transpose() + R_).inverse();

        x_hat = x_minus + K_ * (measurement - h);

        // Update the error covariance
        P_ = (I_ - K_ * H_) * P_minus_;

        // Get the distal acceleration
        acc_link_raw(a_distal.data(), x_hat.data(), acc.data(), r.data());

        return true;
    } else {
        throw std::runtime_error("Attempted to update EKF without init!");
        return false;
    }
}
