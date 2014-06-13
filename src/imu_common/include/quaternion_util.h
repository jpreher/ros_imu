/*=====================================================================================================
// Quaternion Utilities Class for IMU
//=====================================================================================================
//
// Set of tools for easily manipulating quaternions
//
// Date			Author			Notes
// 04/04/2014 	Jake Reher		Initial Release
// 06/11/2014   Jake Reher      Added rotation of vector by quaternion
//
//=====================================================================================================*/
//---------------------------------------------------------------------------------------------------

#ifndef QUATERNION_UTIL_H
#define QUATERNION_UTIL_H

#include <math.h>

class quat{
private:
    static float invSqrt(float x);

public:
    quat();
    static void conj(float q[4], float conjugate[4]);
    static float norm(float q[4]);
    static void inv(float q[4], float qout[4]);
    static void prod(float q1[4], float q2[4], float q1q2[4]);
    static void rotateVec(float vec[3], float q[4], float rotated[3]);

    static void eulerXYZ(float q[4], float euler[3]);
    static void eulerXZY(float q[4], float euler[3]);
    static void eulerZYX(float q[4], float euler[3]);

    static void euler2quatXYZ(float euler[3], float qout[4]);
    static void euler2quatZYX(float euler[3], float qout[4]);
};

#endif // QUATERNION_UTIL_H
