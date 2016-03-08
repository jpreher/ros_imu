#include <imu_common/estimator.hpp>
#include <string.h>

using namespace std;

chain_estimator::chain_estimator(){
    basePath = "/ampro/sensors/imu/";
    ros::param::get(basePath + "N", N);
    isInit = false;

    acc.resize(N+1);
    ekf.resize(N);
}

chain_estimator::~chain_estimator(){

}

void chain_estimator::reset(const YAML::Node &node){
    // Extract the EKF data for each segment
    Matrix<float, 18, 1>  x_init;
    Matrix<float, 18, 18> P_init;
    Matrix<float, 18, 18> Q_init;
    Matrix<float, 16, 16> R_init;
    Matrix<float, 3, 1>   r_init;

    x_init.setZero();
    P_init.setZero();
    Q_init.setZero();
    R_init.setZero();
    r_init.setZero();

    node["P"] >> P_init;
    node["Q"] >> Q_init;
    node["R"] >> R_init;

    for (int i = 0; i<N; i++) {
        node[to_string(i) + "x_init"] >> x_init;
        node[to_string(i) + "r"] >> r_init;

        ekf[i].initialize(x_init, P_init, Q_init, R_init, r_init);
    }

    // Setup the IMUs
    try {
        // Iterate and assign based on serial #


    } catch(const std::runtime_error &e) {
        std::cout << "Error setting up IMUs: " << e.what() << endl;
        isInit = false;
        return;
    }

    isInit = true;
}

int chain_estimator::update(){
    if (isInit) {
        // Update the IMUS then the EKF
        for (int i = 0; i<N; i++) {
            float dt;
            float meas[13];
            IMU[i].getStream(meas, &dt);
            last_measurement[i] << meas[0], meas[1], meas[2], meas[3],  // quat
                                   meas[4], meas[5], meas[6],           // gyr
                                   meas[4]/dt, meas[5]/dt, meas[6]/dt,  // gyrDot
                                   meas[7], meas[8], meas[9],           // accelerometer
                                   meas[10], meas[11], meas[12];        // mag

            if (i==0) { // Base joint
                acc[i] << 0., 0., 0.;
                ekf[i].update(dt, acc[i], last_measurement[i]);
                acc[i+1] = ekf[i].a_distal;
            } else { // All the distals
                ekf[i].update(dt, acc[i], last_measurement[i]);
                acc[i+1] = ekf[i].a_distal;
            }
        }
    } else {
        throw std::runtime_error("EKF attempted to run without loading parameters.");
        return false;
    }
}
