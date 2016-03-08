#ifndef ESTIMATOR
#define ESTIMATOR

#include <vector>
#include <yaml_eigen_utilities/yaml_eigen_utilities.hpp>

#include <imu_common/EKF.hpp>
#include <yei/yei_threespace.hpp>
#include <ros/ros.h>

class chain_estimator {
public:
    std::vector<float[3]> segment;

    chain_estimator();
    ~chain_estimator();

    void reset(const YAML::Node &node);
    int update();

private:
    bool isInit;
    int N;

    std::string                         basePath;
    std::vector<YEI3Space>              IMU;
    std::vector<EKF>                    ekf;
    std::vector<Matrix<float, 3, 1>>    acc;
    std::vector<Matrix<float, 16,1>>    last_measurement;

};


#endif // ESTIMATOR
