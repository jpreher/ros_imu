#include <imu_common/estimator.hpp>
#include <string.h>

using namespace std;
using namespace imu_common;

chain_estimator::chain_estimator(){
    basePath = "/ampro/sensors/imu/";
    ros::param::get(basePath + "N", N);
    isInit = false;
    streamRate = 400;
}

chain_estimator::~chain_estimator(){
    // Since shared_ptr we need to explicity tell it to call sub-deconstructors
    imu_vec.clear();
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

    imu_vec.resize(N);
    int cnt =0;

    // Initialize the vector of devices
    for (int i = 0; i<N; i++) {
        imu_vec[i].reset(new sensor);

        node[to_string(i)]["x_init"]   >> x_init;
        node[to_string(i)]["r"]        >> r_init;
        node[to_string(i)]["SN"]       >> sn;

        imu_vec[i]->serial_id = sn;
        imu_vec[i]->ekf.initialize(x_init, P_init, Q_init, R_init, r_init);
    }
    // Sort the vector
    for (int i=0; i<N; i++) {
        printf("Sensor %d:\n", i);
        for (int j = 0; j<N; j++) {
            string port = "ttyACM" + std::to_string(j);
            //printf("Opening com port %s\n", port.c_str());
            if (!imu_vec[i]->IMU.openAndSetupComPort(port.c_str())) {
                imu_vec[i]->IMU.closeComPort();
                printf("Failed to open com port ttyACM%d\n", j);
                continue;
            }
            imu_vec[i]->IMU.getSerialNumber();
            if (imu_vec[i]->IMU.SerialNumber != imu_vec[i]->serial_id) {
                imu_vec[i]->IMU.closeComPort();
                continue;
            }
            cnt ++;
            break;
        }
    }
    if (cnt != N)
        throw std::runtime_error("Not all of the EKF chains were able to load");

    // Start streaming on the vector
    for (int k=0; k<N; k++) {
        imu_vec[k]->IMU.setupStreamSlots(streamRate);
        imu_vec[k]->IMU.startStreaming();
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

            if ( N ==1 ) { // There is only one joint
                imu_vec[i]->acc << 0., 0., 0.;
                imu_vec[i]->ekf.update(dt, imu_vec[i]->acc, imu_vec[i]->last_measurement);
            } else if (i==0 && N > 1) { // Base joint in a chain
                imu_vec[i]->acc << 0., 0., 0.;
                imu_vec[i]->ekf.update(dt, imu_vec[i]->acc, imu_vec[i]->last_measurement);
                imu_vec[i+1]->acc = imu_vec[i]->ekf.a_distal;
            } else if (i==N-1 && N>1) { // Last joint in the chain
                imu_vec[i]->ekf.update(dt, imu_vec[i]->acc, imu_vec[i]->last_measurement);
            } else { // Intermediate joints
                imu_vec[i]->ekf.update(dt, imu_vec[i]->acc, imu_vec[i]->last_measurement);
                imu_vec[i+1]->acc = imu_vec[i]->ekf.a_distal;
            }
        }
    } else {
        throw std::runtime_error("EKF attempted to run without loading parameters.");
        return false;
    }
    return 1;
}
