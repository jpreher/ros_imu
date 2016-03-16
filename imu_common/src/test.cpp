#include <imu_common/estimator.hpp>
#include <roslib_utilities/ros_package_utilities.hpp>

using namespace YAML;

int main(int argc, char **argv) {
    ros::init(argc, argv, "ekf_chain");
    ros::NodeHandle n;
    ros::Rate rate(200);

    // Normally pull from roslaunch
    ros::param::set("/ampro/sensors/imu/", 1);

    // Setup the chain class
    chain_estimator chainEKF;
    Node doc;
    yaml_utilities::yaml_read_file(roslib_utilities::resolve_local_url("package://imu_common/share/sensor_config.yaml").string(), doc);
    chainEKF.reset(doc["test"]);

    /*
    while (ros::ok()) {
        sleep(1);
    }
    */

    return 0;
}
