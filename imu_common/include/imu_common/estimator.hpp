#ifndef ESTIMATOR
#define ESTIMATOR

#include <vector>
#include <yaml_eigen_utilities/yaml_eigen_utilities.hpp>

#include <imu_common/EKF.hpp>
#include <yei/yei_threespace.hpp>
#include <ros/ros.h>
#include <memory>

using std::shared_ptr;

namespace imu_common {

class chain_estimator {
public:
private:
    bool isInit;
    int N;
    int streamRate;

    std::string                         basePath;
    struct sensor {
        YEI3Space              IMU;
        EKF                    ekf;
        Matrix<float, 3, 1>    acc;
        Matrix<float, 16,1>    last_measurement;
        int                    serial_id;
    };

    //typedef std::shared_ptr<sensor> sensorptr;

public:
    std::vector<float[3]> segment;

    chain_estimator();
    ~chain_estimator();

    void reset(const YAML::Node &node);
    int update();
    std::vector<std::shared_ptr<sensor>>  imu_vec;
};

}


#endif // ESTIMATOR
