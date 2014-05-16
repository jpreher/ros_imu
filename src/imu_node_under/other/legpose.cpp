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

#include "legpose.h"

const float PI  =3.141592653589793238463;

legPose::legPose()
{
    qf_e_ref[0] = qs_e_ref[0] = qt_e_ref[0] = qp_e_ref[0] = 1.0f;
    qf_e_ref[1] = qs_e_ref[1] = qt_e_ref[1] = qp_e_ref[1] = 0.0f;
    qf_e_ref[2] = qs_e_ref[2] = qt_e_ref[2] = qp_e_ref[2] = 0.0f;
    qf_e_ref[3] = qs_e_ref[3] = qt_e_ref[3] = qp_e_ref[3] = 0.0f;
}

void legPose::initPose(float foot[4], float shank[4], float thigh[4], float prosFoot[4]){
    float qfs_e[4], qss_e[4], qts_e[4], qps_e[4];
    float qfs_e_inv[4], qss_e_inv[4], qts_e_inv[4], qps_e_inv[4];
    qfs_e[0] = foot[0];
    qfs_e[1] = foot[1];
    qfs_e[2] = foot[2];
    qfs_e[3] = foot[3];
    qss_e[0] = shank[0];
    qss_e[1] = shank[1];
    qss_e[2] = shank[2];
    qss_e[3] = shank[3];
    qts_e[0] = thigh[0];
    qts_e[1] = thigh[1];
    qts_e[2] = thigh[2];
    qts_e[3] = thigh[3];
    qps_e[0] = prosFoot[0];
    qps_e[1] = prosFoot[1];
    qps_e[2] = prosFoot[2];
    qps_e[3] = prosFoot[3];

    // Invert the measured reference pose measurement.
    quat::inv(qfs_e, qfs_e_inv);
    quat::inv(qss_e, qss_e_inv);
    quat::inv(qts_e, qts_e_inv);
    quat::inv(qps_e, qps_e_inv);

    // Create composite quaternion for reference pose.
    quat::prod(qfs_e_inv, qf_e_ref, qf_s);
    quat::prod(qss_e_inv, qs_e_ref, qs_s);
    quat::prod(qts_e_inv, qt_e_ref, qt_s);
    quat::prod(qps_e_inv, qp_e_ref, qp_s);
}

void legPose::updateHumanLeg(float foot[4], float shank[4], float thigh[4]){
    float qfs_e[4], qss_e[4], qts_e[4];
    qfs_e[0] = foot[0];
    qfs_e[1] = foot[1];
    qfs_e[2] = foot[2];
    qfs_e[3] = foot[3];
    qss_e[0] = shank[0];
    qss_e[1] = shank[1];
    qss_e[2] = shank[2];
    qss_e[3] = shank[3];
    qts_e[0] = thigh[0];
    qts_e[1] = thigh[1];
    qts_e[2] = thigh[2];
    qts_e[3] = thigh[3];

    // Quaternion in Earth Fixed Frame.
    quat::prod(qfs_e, qf_s,qf_e); //Foot to earth
    quat::prod(qss_e, qs_s,qs_e); //Shank to earth
    quat::prod(qts_e, qt_s,qt_e); //Thigh to earth

    // Invert the joint quaternion.
    float qs_e_inv[4], qt_e_inv[4];
    quat::inv(qs_e, qs_e_inv);
    quat::inv(qt_e, qt_e_inv);

    // Relative rotation between joints.
    quat::prod(qs_e_inv, qf_e, qfs); //Foot to shank
    quat::prod(qt_e_inv, qs_e, qst); //Shank to thigh

    // Convert to fixed angles.
    quat::eulerXZY(qfs, hfs);
    quat::eulerXZY(qst, hst);
    quat::eulerXZY(qf_e, hf_e);

    // Change angles to meaningful format.
    hfs[3] += PI / 2.0f;
    hst[3] = PI - std::abs(hst[3]);
}

void legPose::updateProsLeg(float prosFoot[4]){
    float qps_e[4];
    qps_e[0] = prosFoot[0];
    qps_e[1] = prosFoot[1];
    qps_e[2] = prosFoot[2];
    qps_e[3] = prosFoot[3];

    // Quaternion in Earth Fixed Frame.
    quat::prod(qps_e, qf_s,qf_e); //Foot to earth

    // Convert to fixed angle.
    quat::eulerXZY(qp_e, pf_e);
}

