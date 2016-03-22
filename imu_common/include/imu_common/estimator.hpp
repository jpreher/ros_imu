#ifndef ESTIMATOR
#define ESTIMATOR

#include <vector>
#include <yaml_eigen_utilities/yaml_eigen_utilities.hpp>

#include <imu_common/EKF.hpp>
#include <yei/yei_threespace.hpp>
#include <ros/ros.h>
#include <memory>
#include <boost/interprocess/shared_memory_object.hpp>

using std::shared_ptr;

namespace imu_common {

class chain_estimator {
public:
private:
    bool isInit;
    int streamRate;

    std::string                         basePath;
    struct sensor {
        YEI3Space              IMU;
        EKF                    ekf;
        Matrix<float, 3, 1>    acc;
        Matrix<float, 16,1>    last_measurement;
        int                    serial_id;
    };

public:
    bool isSHM;
    int N;

    chain_estimator();
    chain_estimator(bool useSHM);
    ~chain_estimator();

    void reset(const YAML::Node &node);
    int update();
    int getState(int i, float * state, float * meas, float * proximal_acc);
    std::vector<std::shared_ptr<sensor>>  imu_vec;
};

}


#endif // ESTIMATOR
