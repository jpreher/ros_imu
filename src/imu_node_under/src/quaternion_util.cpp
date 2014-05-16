/*=====================================================================================================
// Quaternion Utilities Class for IMU
//=====================================================================================================
//
// Set of tools for easily manipulating quaternions
//
// Date			Author			Notes
// 04/04/2014 	Jake Reher		Initial Release
//
//=====================================================================================================*/
// For more details on how the math behind these functions work read:
// http://www.swarthmore.edu/NatSci/mzucker1/e27/diebel2006attitude.pdf
//---------------------------------------------------------------------------------------------------

#include "quaternion_util.h"

quat::quat(){}

void quat::conj(float q[4], float conjugate[4]){
    conjugate[0] = q[0];
    conjugate[1] = -q[1];
    conjugate[2] = -q[2];
    conjugate[3] = -q[3];
}

float quat::norm(float q[4]){
    float q0,q1,q2,q3;
    q0 = q[0];
    q1 = q[1];
    q2 = q[2];
    q3 = q[3];

    return sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
}

void quat::inv(float q[4], float qinv[4]){
    float conj[4];
    float norm;
    quat::conj(q, conj);
    norm = quat::norm(q);

    qinv[0] = conj[0] / norm;
    qinv[1] = conj[1] / norm;
    qinv[2] = conj[2] / norm;
    qinv[3] = conj[3] / norm;
}

void quat::prod(float q1[4], float q2[4], float q1q2[4]){
    //For more information see:
    //http://mathworld.wolfram.com/Quaternion.html
    float a0, a1, a2, a3;
    float b0, b1, b2, b3;

    a0 = q1[0];
    a1 = q1[1];
    a2 = q1[2];
    a3 = q1[3];
    b0 = q2[0];
    b1 = q2[1];
    b2 = q2[2];
    b3 = q2[3];

    q1q2[0] = a0*b0 - a1*b1 - a2*b2 - a3*b3;
    q1q2[1] = a0*b1 + a1*b0 + a2*b3 - a3*b2;
    q1q2[2] = a0*b2 - a1*b3 + a2*b0 + a3*b1;
    q1q2[3] = a0*b3 + a1*b2 - a2*b1 + a3*b0;
}

void quat::eulerXYZ(float q[4], float euler[3]){
    float q0, q1, q2, q3;

    q0 = q[0];
    q1 = q[1];
    q2 = q[2];
    q3 = q[3];

    euler[0] = atan2(2.f*q2*q3 + 2.f*q0*q1, q3*q3 - q2*q2 - q1*q1 + q0*q0);
    euler[1] = -asin(2.f*q1*q3 - 2.f*q0*q2);
    euler[2] = atan2(2.f*q1*q2 + 2.f*q0*q3, q1*q1 + q0*q0 - q3*q3 - q2*q2);
}

void quat::eulerXZY(float q[4], float euler[3]){
    float q0, q1, q2, q3;

    q0 = q[0];
    q1 = q[1];
    q2 = q[2];
    q3 = q[3];

    euler[0] = atan2(-2.f*q2*q3 + 2.f*q0*q1, q2*q2 - q3*q3 + q0*q0 - q1*q1);
    euler[1] = asin(2.f*q1*q2 + 2.f*q0*q3);
    euler[2] = atan2(-2.f*q1*q3 + 2.f*q0*q2, q1*q1 + q0*q0 - q3*q3 - q2*q2);
}

void quat::eulerZYX(float q[4], float euler[3]){
    float q0, q1, q2, q3;

    q0 = q[0];
    q1 = q[1];
    q2 = q[2];
    q3 = q[3];

    euler[0] = atan2(-2.f*q1*q2 + 2*q0*q3, q1*q1 + q0*q0 - q3*q3 - q2*q2);
    euler[1] = asin(2.f*q1*q3 + 2.f*q0*q2);
    euler[2] = atan2(-2.f*q2*q3 + 2.f*q0*q1, q3*q3 - q2*q2 - q1*q1 + q0*q0);
}

void quat::euler2quatXYZ(float euler[3], float qout[4]){
    float phi, theta, psi;

    phi = euler[0];
    theta = euler[1];
    psi = euler[2];

    qout[0] = cos(phi/2.f)*cos(theta/2.f)*cos(psi/2.f) + sin(phi/2.f)*sin(theta/2.f)*sin(psi/2.f);
    qout[1] = sin(phi/2.f)*cos(theta/2.f)*cos(psi/2.f) - cos(phi/2.f)*sin(psi/2.f)*sin(theta/2.f);
    qout[2] = cos(phi/2.f)*cos(psi/2.f)*sin(theta/2.f) + sin(phi/2.f)*cos(theta/2.f)*sin(psi/2.f);
    qout[3] = cos(phi/2.f)*cos(theta/2.f)*sin(psi/2.f) - sin(theta/2.f)*cos(psi/2.f)*sin(phi/2.f);
}

void quat::euler2quatZYX(float euler[3], float qout[4]){
    float phi, theta, psi;

    phi = euler[0];
    theta = euler[1];
    psi = euler[2];

    qout[0] = cos(phi/2.f)*cos(theta/2.f)*cos(psi/2.f) - sin(phi/2.f)*sin(theta/2.f)*sin(psi/2.f);
    qout[1] = cos(phi/2.f)*cos(theta/2.f)*sin(psi/2.f) + sin(phi/2.f)*cos(psi/2.f)*sin(theta/2.f);
    qout[2]= cos(phi/2.f)*cos(psi/2.f)*sin(theta/2.f) - sin(phi/2.f)*cos(theta/2.f)*sin(psi/2.f);
    qout[3] = cos(phi/2.f)*sin(theta/2.f)*sin(psi/2.f) + cos(theta/2.f)*cos(psi/2.f)*sin(phi/2.f);
}

//---------------------------------------------------------------------------------------------------
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root
float quat::invSqrt(float x) {
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i>>1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}


