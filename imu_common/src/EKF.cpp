/*=====================================================================================================
// Model Based EKF.
//=====================================================================================================
//
// Class which facilitates the use of a model-based EKF to get orientation of leg segments.
//
// Author: Jake Reher
//=====================================================================================================*/
//---------------------------------------------------------------------------------------------------

#include "EKF.h"

EKF::EKF() {
    h.setZero();
    x_minus.setZero();
    r.setZero();
    P_.setZero();
    Q_.setZero();
    R_.setZero();
    K_.setZero();
    A_.setZero();
    H_.setZero();
    P_minus_.setZero();
    I_ = Matrix<float, 14, 14>::Identity(14, 14);
    g = 9.81;
}

void EKF::initialize(Matrix<float, 14, 1> &x_init, Matrix<float, 14, 14> &P_init, Matrix<float, 14, 14> &Q_init, Matrix<float, 9, 9> &R_init, Matrix<float, 3, 1> &r_init) {
    r = r_init;
    P_ = P_init;
    Q_ = Q_init;
    R_ = R_init;
    x_hat = x_init;
}

void EKF::update(float dt, Matrix<float, 3, 1> &acc, Matrix<float, 9, 1> &measurement) {
    float rx = r(0);
    float ry = r(1);
    float rz = r(2);

    z = measurement;

    // Project the state ahead
    float w[4]; float wx; float wy; float wz;
    float wdotx; float wdoty; float wdotz;
    float q[4]; float q0; float q1; float q2; float q3;
    float nrm = 1.0;
    w[0] = 0.f;
    w[1] = x_hat(0);
    w[2] = x_hat(1);
    w[3] = x_hat(2);
    q[0] = x_hat(6);
    q[1] = x_hat(7);
    q[2] = x_hat(8);
    q[3] = x_hat(9);

    wx = x_hat(0); wy = x_hat(1); wz = x_hat(2);
    wdotx = x_hat(3); wdoty  = x_hat(4); wdotz = x_hat(5);
    q0 = x_hat(6); q1 = x_hat(7); q2 = x_hat(8); q3 = x_hat(9);

    quat::prod(w, q, q);

    // Calculate the A matrix at this time step
    A_.row(0) << 1., 0., 0., dt, 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.;
    A_.row(1) << 0., 1., 0., 0., dt, 0., 0., 0., 0., 0., 0., 0., 0., 0.;
    A_.row(2) << 0., 0., 1., 0., 0., dt, 0., 0., 0., 0., 0., 0., 0., 0.;
    A_.row(3) << 0., 0., 0., 1., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.;
    A_.row(4) << 0., 0., 0., 0., 1., 0., 0., 0., 0., 0., 0., 0., 0., 0.;
    A_.row(5) << 0., 0., 0., 0., 0., 1., 0., 0., 0., 0., 0., 0., 0., 0.;
    A_.row(6) << 0., 0., 0., 0., 0., 0., 1., 0., 0., 0., dt, 0., 0., 0.;
    A_.row(7) << 0., 0., 0., 0., 0., 0., 0., 1., 0., 0., 0., dt, 0., 0.;
    A_.row(8) << 0., 0., 0., 0., 0., 0., 0., 0., 1., 0., 0., 0., dt, 0.;
    A_.row(9) << 0., 0., 0., 0., 0., 0., 0., 0., 0., 1., 0., 0., 0., dt;
    A_.row(10) << -0.5*q1, -0.5*q2, -0.5*q3,  0., 0., 0.,    0, -0.5*wx, -0.5*wy, -0.5*wz,     0., 0., 0., 0.;
    A_.row(11) << 0.5*q0,  -0.5*q3, 0.5*q2,   0., 0., 0.,    0.5*wx, 0, 0.5*wz, -0.5*wy,       0., 0., 0., 0.;
    A_.row(12) << -0.5*q3, 0.5*q0,  -0.5*q1,  0., 0., 0.,    0.5*wy, -0.5*wz, 0, -0.5*wx,      0., 0., 0., 0.;
    A_.row(13) << -0.5*q2, 0.5*q1,  0.5*q0,   0., 0., 0.,    0.5*wz, 0.5*wy, -0.5*wx, 0,       0., 0., 0., 0.;

    x_minus << x_hat(0) + x_hat(3)*dt,  //angular velocity
               x_hat(1) + x_hat(4)*dt,
               x_hat(2) + x_hat(5)*dt,
               x_hat(3),                // angular acceleration
               x_hat(4),
               x_hat(5),
               x_hat(6) + x_hat(10)*dt, // quaternion
               x_hat(7) + x_hat(11)*dt,
               x_hat(8) + x_hat(12)*dt,
               x_hat(9) + x_hat(13)*dt,
               0.5*q[0], // time derivative of quaternion
               0.5*q[1],
               0.5*q[2],
               0.5*q[3];

    wx = x_minus(0); wy = x_minus(1); wz = x_minus(2);
    wdotx = x_minus(3); wdoty  = x_minus(4); wdotz = x_minus(5);
    q0 = x_minus(6); q1 = x_minus(7); q2 = x_minus(8); q3 = x_minus(9);
    nrm = invSqrt((q0*q0 + q1*q1 + q2*q2 + q3*q3));
    q0 = q0 * nrm;
    q1 = q1 * nrm;
    q2 = q2 * nrm;
    q3 = q3 * nrm;

    // Project the error covariance ahead
    P_minus_ = A_ * P_ * A_.transpose() + Q_;

    // Calculate H
    H_.row(0) << 1., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.;
    H_.row(1) << 0., 1., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.;
    H_.row(2) << 0., 0., 1., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.;
    H_.row(3) << 0., 0., 0., 1., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.;
    H_.row(4) << 0., 0., 0., 0., 1., 0., 0., 0., 0., 0., 0., 0., 0., 0.;
    H_.row(5) << 0., 0., 0., 0., 0., 1., 0., 0., 0., 0., 0., 0., 0., 0.;
    H_.row(6) << (-ry*wy + rz*wz),      (ry*wx + 2.*rx*wy),   (-2.*rx*wz + rz*wx),    0.,   -rz,   ry,        2.*g*q2,    2.*g*q3,    2.*g*q0,   2.*g*q1,       0., 0., 0., 0.;
    H_.row(7) << (-2.*ry*wx - rx*wy),   (rz*wz + rx*wx),      (-rz*wy - 2.*ry*wz),     rz,  0.,   -rx,         -2.*g*q1,  -2.*g*q0,    2.*g*q3,   2.*g*q2,       0., 0., 0., 0.;
    H_.row(8) << (rx*wz - 2.*rz*wx),    (2.*rz*wy + ry*wz),   (rx*wx - ry*wy),        -ry,   rx,  0.,         2.*g*q0,   -2.*g*q1,   -2.*g*q2,   2.*g*q3,       0., 0., 0., 0.;

    // Calculate the Kalman gain
    Matrix<float, 9, 9> denom(9,9);
    Matrix<float, 14, 9> H_transpose = H_.transpose();
    denom = H_ * P_minus_ * H_transpose + R_;
    K_ = (P_minus_ * H_transpose) * denom.inverse();

    // Calculate the a posteriori state
    h <<    x_minus(0),
            x_minus(1),
            x_minus(2),
            x_minus(3),
            x_minus(4),
            x_minus(5),
            acc(0) + (-rz*wdoty - ry*wdotz) + (-ry*wx*wy + rx*wy*wy - rx*wz*wy + rz*wx*wz) + (2.*g*(q1*q3 + q0*q2)),
            acc(1) + (rx*wdotz - rz*wdotx) + (rz*wy*wz - ry*wz*wz - ry*wx*wx + rx*wx*wy) + (2.*g*(q2*q3 - q0*q1)),
            acc(2) + (ry*wdotx + rx*wdoty) + (rx*wx*wz - rz*wx*wx + rz*wy*wy + ry*wy*wz) + (g*(q0*q0 - q1*q1 - q2*q2 + q3*q3));

    x_hat = x_minus + K_ * (z - h);

    nrm = invSqrt((x_hat(6)*x_hat(6) + x_hat(7)*x_hat(7) + x_hat(8)*x_hat(8) + x_hat(9)*x_hat(9)));
    float temp[4];
    temp[0] = x_hat(6)*x_hat(6);
    temp[1] = x_hat(7)*x_hat(7);
    temp[2] = x_hat(8)*x_hat(8);
    temp[3] = x_hat(9)*x_hat(9);

    x_hat(6) = x_hat(6) * nrm;
    x_hat(7) = x_hat(7) * nrm;
    x_hat(8) = x_hat(8) * nrm;
    x_hat(9) = x_hat(9) * nrm;

    // Update the error covariance
    P_ = (I_ - K_ * H_) * P_minus_;
}

//---------------------------------------------------------------------------------------------------
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root
float EKF::invSqrt(float x) {
    float halfx = 0.5f * x;
    float y = x;
    long i = * ( long * ) &y;
    i = 0x5f3759df - (i >> 1);
    y = * ( float * ) &i;
    y = y * (1.5f - ( halfx * y * y));
    return abs(y);
}
