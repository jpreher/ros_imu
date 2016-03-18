#include <imu_common/estimator.hpp>
#include <roslib_utilities/ros_package_utilities.hpp>
#include <yei/yei_msg.h>

using namespace YAML;
using namespace imu_common;

int main(int argc, char **argv) {
    ros::init(argc, argv, "ekf_chain");
    ros::NodeHandle n;
    ros::Rate rate(200);
    yei::yei_msg dataThigh;
    yei::yei_msg dataShank;
    ros::Publisher pub1 = n.advertise<yei::yei_msg>("thigh", 1000);
    ros::Publisher pub2 = n.advertise<yei::yei_msg>("shank", 1000);
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

    while (ros::ok()) {
        chainEKF.imu_vec[0]->IMU.getStream(streamPacket, &dt);
        dataThigh.header.stamp = ros::Time::now();

        dataThigh.quat.w = streamPacket[0];
        dataThigh.quat.x = streamPacket[1];
        dataThigh.quat.y = streamPacket[2];
        dataThigh.quat.z = streamPacket[3];

        dataThigh.gyr.x = streamPacket[4];
        dataThigh.gyr.y = streamPacket[5];
        dataThigh.gyr.z = streamPacket[6];

        dataThigh.acc.x = streamPacket[7];
        dataThigh.acc.y = streamPacket[8];
        dataThigh.acc.z = streamPacket[9];

        dataThigh.mag.x = streamPacket[10];
        dataThigh.mag.y = streamPacket[11];
        dataThigh.mag.z = streamPacket[12];

        chainEKF.imu_vec[1]->IMU.getStream(streamPacket, &dt);
        dataShank.header.stamp = dataThigh.header.stamp;

        dataShank.quat.w = streamPacket[0];
        dataShank.quat.x = streamPacket[1];
        dataShank.quat.y = streamPacket[2];
        dataShank.quat.z = streamPacket[3];

        dataShank.gyr.x = streamPacket[4];
        dataShank.gyr.y = streamPacket[5];
        dataShank.gyr.z = streamPacket[6];

        dataShank.acc.x = streamPacket[7];
        dataShank.acc.y = streamPacket[8];
        dataShank.acc.z = streamPacket[9];

        dataShank.mag.x = streamPacket[10];
        dataShank.mag.y = streamPacket[11];
        dataShank.mag.z = streamPacket[12];


        pub1.publish(dataThigh);
        pub2.publish(dataShank);

        ros::spinOnce();
        rate.sleep();
    }

    //free(streamPacket);

    return 0;
}
