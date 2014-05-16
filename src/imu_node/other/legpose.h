/*=====================================================================================================
// Prosthetic and Human Leg Angle Tools For IMU
//=====================================================================================================
//
// Set of tools for to get angles between links of prosthetic and human legs.
//
// Date			Author			Notes
// 04/09/2014 	Jake Reher		Initial Release
//
//=====================================================================================================*/
//---------------------------------------------------------------------------------------------------

#ifndef LEGPOSE_H
#define LEGPOSE_H

#include "quaternion_util.h"
#include <cmath>

class legPose
{
public:
    float qf_e_ref[4], qs_e_ref[4], qt_e_ref[4], qp_e_ref[4];
    float qf_s[4], qs_s[4], qt_s[4], qp_s[4];


    float qf_e[4], qs_e[4], qt_e[4], qp_e[4];
    float qfs[4], qst[4];
    float hfs[3], hst[3];
    float hf_e[3], pf_e[3];

    legPose();
    void initPose(float foot[4], float shank[4], float thigh[3], float prosFoot[4]);
    void updateHumanLeg(float foot[4], float shank[4], float thigh[4]);
    void updateProsLeg(float foot[4]);
};

#endif // LEGPOSE_H
