#ifndef ESTIMATOR
#define ESTIMATOR

#include <vector>
#include <yaml_eigen_utilities/yaml_eigen_utilities.hpp>

#include <imu_common/EKF.hpp>
#include <yei/yei_threespace.hpp>


class chain_estimator {
public:
    std::vector<float[3]> segment;

    chain_estimator();
    ~chain_estimator();

    void reset();
    int update();

private:
    bool isInit;
    float * acc;
    int N;

    std::vector<YEI3Space> IMU;
    std::vector<EKF> ekf;

};










#endif // ESTIMATOR
