#include <imu_common/estimator.hpp>
#include <roslib_utilities/ros_package_utilities.hpp>
#include <yei/yei_msg.h>

using namespace YAML;
using namespace imu_common;

int main(int argc, char **argv) {
    ros::init(argc, argv, "ekf_chain");
    ros::NodeHandle n;
    ros::Rate rate(200);
    yei::yei_msg data;
    ros::Publisher pub = n.advertise<yei::yei_msg>("chain_tester", 1000);
    int N;
    float streamPacket[13];
    float dt;

    // Load in the YAML config
    Node doc;
    yaml_utilities::yaml_read_file(roslib_utilities::resolve_local_url("package://imu_common/share/sensor_config.yaml").string(), doc);
    doc["N"] >> N;
    ros::param::set("/ampro/sensors/imu/N", N);

    // Setup chain ekf
    chain_estimator chainEKF;
    printf("Setting up EKF\n");
    chainEKF.reset(doc);

    rate.sleep();

    //streamPacket = (float*)malloc(chainEKF.imu_vec[0]->IMU.stream_byte_len * sizeof(float));

    chainEKF.update();

    while (ros::ok()) {
        //chainEKF.imu_vec[0]->IMU.getStream(streamPacket, &dt);
        data.header.stamp = ros::Time::now();

        data.quat.w = streamPacket[0];
        data.quat.x = streamPacket[1];
        data.quat.y = streamPacket[2];
        data.quat.z = streamPacket[3];

        data.gyr.x = streamPacket[4];
        data.gyr.y = streamPacket[5];
        data.gyr.z = streamPacket[6];

        data.acc.x = streamPacket[7];
        data.acc.y = streamPacket[8];
        data.acc.z = streamPacket[9];

        data.mag.x = streamPacket[10];
        data.mag.y = streamPacket[11];
        data.mag.z = streamPacket[12];

        pub.publish(data);

        ros::spinOnce();
        rate.sleep();
    }



    return 0;
}
