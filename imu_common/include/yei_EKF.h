#ifndef yei_EKF_H
#define yei_EKF_H

#include <cstring>
#include <iostream>
#include "ros/ros.h"
#include "ros/time.h"
#include "EKF.h"
#include "imu_common/imu.h"
#include "std_srvs/Empty.h"
#include "std_msgs/Bool.h"
#include "yei_sensor/yei_msg.h"
#include "quaternion_util.h"
#include "yei_wrapper.h"

class yei_EKF {

public:

    yei_EKF(ros::NodeHandle nh, int freq);
    yei_wrapper yei_python;

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

        EKF     Filter;

        bool    running,
                isinitalized,
                iscalibrated,
                doPR,
                doYaw;

        double  time_last_run,
                length,
                radius,
                frequency;

        float   q_sensor_cal[4],
                dt;

        Matrix<float, 14, 1>     x_init;
        Matrix<float, 3, 1>     r_init;
        Matrix<float, 14, 14>   P_init, Q_init;
        Matrix<float, 9, 9>     R_init;
        Matrix<float, 14, 1>    processVar;
        Matrix<float, 9, 1>     measureVar;
        Matrix<float, 3, 1>     velocity, Dvelocity, acc, a0;

        std::string bias_path;

        ros::ServiceServer  calibrate_serv_;
        ros::ServiceServer  pitch_roll_ref_;
        ros::ServiceServer  yaw_ref_;
        ros::Publisher      data_pub_;
        imu_common::imu     data;
        imu_common::imu     rawdata;

        bool initialize(ros::NodeHandle &nh, Matrix<float, 3, 1> &rad, int freq);
        void check_cal();
        bool calibrate_gyro();
        void pitch_roll_ref();
        void yaw_ref();
        bool pitch_roll_serv(std_srvs::Empty::Request  &req,
                             std_srvs::Empty::Response &resp);
        bool yaw_serv(std_srvs::Empty::Request  &req,
                      std_srvs::Empty::Response &resp);
    };

    bool setting_select();
    bool spin();
    bool updateEKF();
    bool publishRunning();
    bool filterRunning();
    bool checkCalibration();
    void filter(imu& device);
    static bool publish(imu& device);
    bool publishall;
    void getYEIreading();

    imu R_shank;
    imu R_thigh;
    imu L_foot;

private:
    Matrix<float, 3, 1> Len_shank, Len_thigh, Rad_foot, Rad_shank, Rad_thigh, Rad_torso, Rad_single;

    int frequency;
    int setting;
    void Callback();
    ros::NodeHandle node_handle_;
    ros::Rate rate;

};


#endif //yei_EKF_H
