#ifndef ESTIMATOR
#define ESTIMATOR

#include <EKF.hpp>
#include <yei_threespace.hpp>
#include <vector>
#include <ros/ros.h>
#include <yaml_eigen_utilities/yaml_eigen_utilities.hpp>

class chain_estimator {
public:
    std::vector<float[3]> segment;

    chain_estimator(ros::NodeHandle n);
    ~chain_estimator();

    void reset(const YAML::Node &node);
    int update();

private:
    bool isInit;
    float * acc;
    int N;
    ros::NodeHandle node_handle_;

    std::vector<YEI3Space> IMU;
    std::vector<EKF> ekf;


};














#endif // ESTIMATOR
