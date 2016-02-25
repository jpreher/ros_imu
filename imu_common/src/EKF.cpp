/*=====================================================================================================
// Model Based EKF.
//=====================================================================================================
//
// Class which facilitates the use of a model-based EKF to get orientation of leg segments.
//
// Author: Jake Reher
//=====================================================================================================*/
//---------------------------------------------------------------------------------------------------

#include "EKF.hpp"

EKF::EKF() {

}


void initialize(Matrix<float, 14, 1> &x_init, Matrix<float, 14, 14> &P_init, Matrix<float, 14, 14> &Q_init, Matrix<float, 9, 9> &R_init, Matrix<float, 3, 1> &r_init){

}

void update(float dt, Matrix<float, 3, 1> &acc, Matrix<float, 9,1> &measurement) {

}
