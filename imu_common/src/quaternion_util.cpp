//---------------------------------------------------------------------------------------------------
/*=====================================================================================================
// Quaternion Utilities Class for IMU
//=====================================================================================================
//
// Set of tools for easily manipulating quaternions
//
// Date			Author			Notes
// 04/04/2014 	Jake Reher		Initial Release.
// 06/11/2014   Jake Reher      Added rotation of vector by quaternion.
// 10/28/2014   Jake Reher      Cleaned up math and added documentation.
// 04/05/2015   Jake Reher      Quaternion between two vecors added.
// 06/03/2015   Jake Reher      Added quat2eulerXYZ
//
//=====================================================================================================*/
// For more details on how the math behind these functions works read:
// http://www.swarthmore.edu/NatSci/mzucker1/e27/diebel2006attitude.pdf
//---------------------------------------------------------------------------------------------------

#include "quaternion_util.h"

quat::quat(){}

/* FUNCTION conj(float q[4], float conjugate[4])
 * Performs a quaternion conjugate operation on a four value float vector.
 */
void quat::conj(float q[4], float conjugate[4]){
    conjugate[0] = q[0];
    conjugate[1] = -q[1];
    conjugate[2] = -q[2];
    conjugate[3] = -q[3];
}

/* FUNCTION norm(float q[4])
 * Calculates the norm of a four element float array (quaternion sized)
 * RETURN: float value of norm.
 */
float quat::norm(float q[4]){
    float q0,q1,q2,q3;
    q0 = q[0];
    q1 = q[1];
    q2 = q[2];
    q3 = q[3];

    return sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
}

/* FUNCTION norm(float q[3])
 * Calculates the norm of a 3 element float array
 * RETURN: float value of norm.
 */
float quat::norm3(float v[3]) {
    float v1, v2, v3;
    v1 = v[0];
    v2 = v[1];
    v3 = v[2];

    return sqrt(v1*v1 + v2*v2 + v3*v3);
}

/* FUNCTION inv(float q[4], float qinv[4])
 * Calculates inverse of a quaternion.
 * Also normalizes the result for unit quaternion size maintenance.
 */
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

/* FUNCTION prod(float q1[4], float q2[4], float q1q2[4])
 * Performs the quaternion product of two quaternion arrays in the product order q1*q2.
 *
 * For more information see:
 * http://mathworld.wolfram.com/Quaternion.html
 */
void quat::prod(float q1[4], float q2[4], float q1q2[4]){
    float a0, a1, a2, a3;
    float b0, b1, b2, b3;
    float nrm;

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
    q1q2[2] = a0*b2 - a1*b3 + a2*b0 - a3*b1;
    q1q2[3] = a0*b3 + a1*b2 - a2*b1 + a3*b0;

    nrm = norm(q1q2);
    q1q2[0] = q1q2[0] / nrm;
    q1q2[1] = q1q2[1] / nrm;
    q1q2[2] = q1q2[2] / nrm;
    q1q2[3] = q1q2[3] / nrm;
}

/* FUNCTION rotateVec(float vec[3], float q[4], float rotated[3])
 * Takes in a three dimensional vector and a quaternion array.
 * "rotated" is a three dimensional vector which is the result of the quaternion rotation
 *      applied to the original vector.
 */
void quat::rotateVec(float vec[3], float q[4], float rotated[3]) {
    float q0, q1, q2, q3;
    q0 = q[0];
    q1 = q[1];
    q2 = q[2];
    q3 = q[3];

    float v1, v2, v3;
    v1 = vec[0];
    v2 = vec[1];
    v3 = vec[2];

    rotated[0] = v1*(1.f - 2.f*q2*q2 - 2.f*q3*q3)   + 2.f*v2*(q1*q2 + q0*q3)                + 2.f*v3*(q1*q3 - q0*q2);
    rotated[1] = 2.f*v1*(q1*q2 - q0*q3)             + v2*(1.f - 2.f*q1*q1 - 2.f*q3*q3)      + 2.f*v3*(q2*q3 + q0*q1);
    rotated[2] = 2.f*v1*(q1*q3 + q0*q2)             + 2.f*v2*(q2*q3 - q0*q1)                + v3*(1.f - 2.f*q1*q1 - 2.f*q2*q2);
}

/* FUNCTION eulerXYZ(float q[4], float euler[3])
 * Convert the quaternion to euler angles in XYZ rotation order.
 */
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

/* FUNCTION eulerXZY(float q[4], float euler[3])
 * Convert the quaternion to euler angles in XZY rotation order.
 */
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

/* FUNCTION eulerZYX(float q[4], float euler[3])
 * Convert the quaternion to euler angles in ZYX rotation order.
 */
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

/* FUNCTION euler2quatXYZ(float euler[3], float qout[4])
 * Convert the XYZ rotation order euler angle set to a quaternion rotation.
 */
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

/* FUNCTION euler2quatZYX(float euler[3], float qout[4])
 * Convert the ZYX rotation order euler angle set to a quaternion rotation.
 */
void quat::euler2quatZYX(float euler[3], float qout[4]){
    float phi, theta, psi;

    phi = euler[0];
    theta = euler[1];
    psi = euler[2];

    qout[0] = cos(phi/2.f)*cos(theta/2.f)*cos(psi/2.f) - sin(phi/2.f)*sin(theta/2.f)*sin(psi/2.f);
    qout[1] = cos(phi/2.f)*cos(theta/2.f)*sin(psi/2.f) + sin(phi/2.f)*cos(psi/2.f)*sin(theta/2.f);
    qout[2] = cos(phi/2.f)*cos(psi/2.f)*sin(theta/2.f) - sin(phi/2.f)*cos(theta/2.f)*sin(psi/2.f);
    qout[3] = cos(phi/2.f)*sin(theta/2.f)*sin(psi/2.f) + cos(theta/2.f)*cos(psi/2.f)*sin(phi/2.f);
}

void quat::two_vec_q(float v1[3], float v2[3], float qout[4]) {
    float t1[3], t2[3], c[3], a, l1, l2, tempq[4];
    // Normalize
    t1[0] = v1[0] / norm3(v1);
    t1[1] = v1[1] / norm3(v1);
    t1[2] = v1[2] / norm3(v1);
    t2[0] = v2[0] / norm3(v2);
    t2[1] = v2[1] / norm3(v2);
    t2[2] = v2[2] / norm3(v2);

    // Calculate Cross Product
    c[0] = t1[1]*t2[2] - t1[2]*t2[1];
    c[1] = t1[2]*t2[0] - t1[0]*t2[2];
    c[2] = t1[0]*t2[1] - t1[1]*t2[0];

    // Quaternion sub-elements
    a = t1[0]*t2[0] + t1[1]*t2[1] + t1[2]*t2[2];
    l1 = t1[0]*t1[0] + t1[1]*t1[1] + t1[2]*t1[2];
    l2 = t2[0]*t2[0] + t2[1]*t2[1] + t2[2]*t2[2];

    // Quaternion elements
    tempq[0] = sqrt(l1*l2) + a;
    tempq[1] = c[0];
    tempq[2] = c[1];
    tempq[3] = c[2];

    // Return
    qout[0] = tempq[0] / norm(tempq);
    qout[1] = tempq[1] / norm(tempq);
    qout[2] = tempq[2] / norm(tempq);
    qout[3] = tempq[3] / norm(tempq);
}

void quat::quat2eulerXYZ(float q[4], float euler[3]) {
    float q0, q1, q2, q3;
    q0 = q[0];
    q1 = q[1];
    q2 = q[2];
    q3 = q[3];

    euler[0] = atan2(2.*q2*q3 + 2.*q0*q1, q3*q3 - q2*q2 - q1*q1 + q0*q0);
    euler[1] = -asin(2.*q1*q3 - 2.*q0*q2);
    euler[2] = atan2(2.*q1*q2 + 2.*q0*q3, q1*q1 + q0*q0 - q3*q3 - q2*q2);
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


