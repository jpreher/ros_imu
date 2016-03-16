#include <imu_common/estimator.hpp>
#include <string.h>

using namespace std;


chain_estimator::chain_estimator(){
    basePath = "/ampro/sensors/imu/";
    ros::param::get(basePath + "N", N);
    isInit = false;
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
    unsigned int sn;

    x_init.setZero();
    P_init.setZero();
    Q_init.setZero();
    R_init.setZero();
    r_init.setZero();

    node["P"] >> P_init;
    node["Q"] >> Q_init;
    node["R"] >> R_init;

    for (int i = 0; i<N; i++) {
        sensorptr tempSens;
        tempSens.reset(new sensor);

        node[to_string(i) + "x_init"]   >> x_init;
        node[to_string(i) + "r"]        >> r_init;
        node[to_string(i) + "SN"]       >> sn;

        tempSens->serial_id = sn;
        tempSens->ekf.initialize(x_init, P_init, Q_init, R_init, r_init);

        for (int j = 0; j<N; j++) {
            string port = "ttyACM" + std::to_string(j);
            if (!tempSens->IMU.openAndSetupComPort(port.c_str())) {
                tempSens->IMU.closeComPort();
                continue;
            }
            tempSens->IMU.getSerialNumber();
            if (tempSens->IMU.SerialNumber != sn) {
                tempSens->IMU.closeComPort();
                continue;
            }
            imu_vec.push_back(tempSens);
        }
    }

    isInit = true;
}

int chain_estimator::update(){
    if (isInit) {
        // Update the IMUS then the EKF
        for (int i = 0; i<N; i++) {
            float dt;
            float meas[13];
            imu_vec[i]->IMU.getStream(meas, &dt);
            imu_vec[i]->last_measurement <<  meas[0], meas[1], meas[2], meas[3],  // quat
                                             meas[4], meas[5], meas[6],           // gyr
                                             meas[4]/dt, meas[5]/dt, meas[6]/dt,  // gyrDot
                                             meas[7], meas[8], meas[9],           // accelerometer
                                             meas[10], meas[11], meas[12];        // mag

            if (i==0) { // Base joint
                imu_vec[i]->acc << 0., 0., 0.;
                imu_vec[i]->ekf.update(dt, imu_vec[i]->acc, imu_vec[i]->last_measurement);
                imu_vec[i+1]->acc = imu_vec[i]->ekf.a_distal;
            } else { // All the distals
                imu_vec[i]->ekf.update(dt, imu_vec[i]->acc, imu_vec[i]->last_measurement);
                imu_vec[i+1]->acc = imu_vec[i]->ekf.a_distal;
            }
        }
    } else {
        throw std::runtime_error("EKF attempted to run without loading parameters.");
        return false;
    }
}
