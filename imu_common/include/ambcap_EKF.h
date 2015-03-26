/*=====================================================================================================
// AMBCAP tools to enable easy configurability of reading from and number of imu devices in ROS.
//=====================================================================================================
//
// ROS class with objects and functionality to easily collect data from IMU's on the BBB.
//
// Date         Author          Notes
// 09/12/2014   Jake Reher      Initial Release
//
//=====================================================================================================*/
//---------------------------------------------------------------------------------------------------

#ifndef AMBCAP_EKF_H
#define AMBCAP_EKF_H

#include <cstring>
#include <iostream>
#include "ros/ros.h"
#include "ros/time.h"
#include "MPU9150.h"
#include "EKF.h"
#include "imu_common/imu.h"
#include "std_srvs/Empty.h"
#include "std_msgs/Bool.h"
#include "quaternion_util.h"

class ambcap_EKF {

public:

    ambcap_EKF(ros::NodeHandle nh, int frequency);

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
        EKF     Filter;

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

        double  time_last_run,
                length,
                radius,
                dt;

        float   q_sensor_cal[4];

        VectorXd x_init, r_init;
        MatrixXd P_init, Q_init, R_init;
        VectorXd processVar, measureVar;
        VectorXd velocity, Dvelocity, acc, a0;

        std::string bias_path;

        ros::Publisher      data_pub_;
        ros::ServiceServer  calibrate_serv_;
        ros::ServiceServer  pitch_roll_ref_;
        ros::ServiceServer  yaw_ref_;
        imu_common::imu     data;

        bool initialize(ros::NodeHandle &nh, Vector3d &rad);
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
    bool ambcap_EKF::updateEKF();
    bool publishRunning();
    bool checkCalibration();
    static bool update(imu& device);
    void filter(imu& device);
    static bool publish(imu& device);
    bool publishall;

    imu L_foot;
    imu L_shank;
    imu L_thigh;
    imu R_foot;
    imu R_shank;
    imu R_thigh;
    imu Torso;
    imu Single;

private:
    Vector3d Len_shank, Len_thigh, Rad_foot, Rad_shank, Rad_thigh, Rad_torso, Rad_single;

    int frequency;
    int setting;
    ros::NodeHandle node_handle_;
    ros::Rate rate;

};


#endif //AMBCAP_EKF_H
