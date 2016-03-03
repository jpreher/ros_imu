#include "yei_threespace.hpp"
#include "yei/yei_msg.h"
#include "ros/ros.h"

int main(int argc, char **argv) {
    YEI3Space sensor;
    const char* port = "ttyACM0";
    float gyro_rate3[3];
    float accelerometer3[3];
    float compass3[3];
    float * streamPacket;
    yei::yei_msg data;
    int streamRate = 750;

    ros::init(argc, argv, "yei");
    ros::NodeHandle n;

    ros::Publisher pub = n.advertise<yei::yei_msg>("yei_msg", 1000);

    ros::Rate rate(2000);

	if ( !sensor.openAndSetupComPort(port) )
		return 0;

    sensor.getSerialNumber();
    printf("Serial Number: %x\n", sensor.SerialNumber);


    sensor.getAllCorrectedComponentSensorData(gyro_rate3, accelerometer3, compass3);

    printf(" Gyro: %f %f %f\n Accelerometer: %f %f %f\n Compass: %f %f %f\n\n", gyro_rate3[0], gyro_rate3[1], gyro_rate3[2],
                                                                              accelerometer3[0], accelerometer3[1], accelerometer3[2],
                                                                              compass3[0], compass3[1], compass3[2]);

    sensor.resetBaseOffset();
    sensor.offsetWithCurrentOrientation();
    sensor.getOffsetOrientationAsQuaternion();

    sensor.getAllCorrectedComponentSensorData(gyro_rate3, accelerometer3, compass3);
    printf(" Offset: %f %f %f %f\n\n\n", sensor.offsetQ[0], sensor.offsetQ[1], sensor.offsetQ[2], sensor.offsetQ[3]);

    sensor.setupStreamSlots(streamRate);

    streamPacket = (float*)malloc(sensor.stream_byte_len * sizeof(float));

    sensor.startStreaming();
    printf("Streaming worker thread at %d Hz\n", streamRate);

    while (ros::ok())
    {
        if (sensor.getStream(streamPacket)) {
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
        }
        ros::spinOnce();
        rate.sleep();
    }

    free(streamPacket);
    printf("Main is sending close signal!\n");
    sensor.stopStreaming();

    return 0;
}
