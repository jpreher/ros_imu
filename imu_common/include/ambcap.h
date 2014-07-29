/*=====================================================================================================
// AMBCAP tools to enable easy configurability of reading from and number of imu devices in ROS.
//=====================================================================================================
//
// ROS class with objects and functionality to easily collect data from IMU's on the BBB.
//
// Date         Author          Notes
// 07/15/2014   Jake Reher      Initial Release
//
//=====================================================================================================*/
//---------------------------------------------------------------------------------------------------

#ifndef AMBCAP_H
#define AMBCAP_H

#include <cstring>
#include <iostream>
#include "ros/ros.h"
#include "ros/time.h"
#include "MPU9150.h"
#include "imu_common/imu.h"
#include "std_srvs/Empty.h"
#include "std_msgs/Bool.h"
#include "quaternion_util.h"

class ambcap {

public:

    ambcap(ros::NodeHandle nh, int frequency);

    enum imu_select {
        l_foot,
        l_shank,
        l_thigh,
        r_foot,
        r_shank,
        r_thigh,
        torso,
        single
    };

    struct imu {
        enum    imu_select imu_location;

        MPU9150 MPU;

        int     bus,
                addr,
                chan,
                frequency;

        bool    isvertical,
                isinitalized,
                iscalibrated,
                doPR,
                doYaw,
                running;

        double  time_last_run;

        float   q_sensor_cal[4];

        std::string bias_path;

        ros::Publisher      data_pub_;
        ros::ServiceServer  calibrate_serv_;
        ros::ServiceServer  pitch_roll_ref_;
        ros::ServiceServer  yaw_ref_;
        imu_common::imu     data;

        bool initialize(ros::NodeHandle& nh);
        void check_cal();
        bool calibrate_gyro();
        void pitch_roll_ref();
        void yaw_ref();
        bool cal_gyr(std_srvs::Empty::Request  &req,
                     std_srvs::Empty::Response &resp);
        bool pitch_roll_serv(std_srvs::Empty::Request  &req,
                             std_srvs::Empty::Response &resp);
        bool yaw_serv(std_srvs::Empty::Request  &req,
                      std_srvs::Empty::Response &resp);
    };

    bool setting_select();
    bool spin();
    bool publishRunning();
    bool checkCalibration();
    static bool update(imu& device);
    static bool publish(imu& device);


private:
    imu L_foot;
    imu L_shank;
    imu L_thigh;
    imu R_foot;
    imu R_shank;
    imu R_thigh;
    imu Torso;
    imu Single;

    int frequency;
    int setting;
    ros::NodeHandle node_handle_;
    ros::Rate rate;

};


#endif //AMBCAP_H
