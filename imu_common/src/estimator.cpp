#include "estimator.hpp"
#include <string.h>

chain_estimator::chain_estimator(ros::NodeHandle n){

}

chain_estimator::~chain_estimator(){

}

void chain_estimator::reset(const YAML::Node &node){
    std::vector<char*> addresses;
    std::vector<int> sn;

    // Extract the length of the chain;
    //N = node["IMU"]["N"].as<int>();

    // Get the addresses, connect, get serial numbers



    // Extract the IMU data for each IMU
    Matrix<float, 18, 1>  x_init;
    Matrix<float, 18, 18> P_init;
    Matrix<float, 18, 18> Q_init;
    Matrix<float, 16, 16> R_init;
    Matrix<float, 3, 1>   r_init;

    for (int i = 0; i<N; i++) {
        x_init.setZero();
        P_init.setZero();
        Q_init.setZero();
        R_init.setZero();
        r_init.setZero();

        node["xi"]  >> x_init;
        node["P"]   >> P_init;
        node["Q"]   >> Q_init;
        node["R"]   >> R_init;
        node["r"]   >> r_init;

        ekf[i].initialize(x_init, P_init, Q_init, R_init, r_init);
    }
}

int chain_estimator::update(){

}
