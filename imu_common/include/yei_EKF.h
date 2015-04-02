#ifndef yei_EKF_H
#define yei_EKF_H

#include <cstring>
#include <iostream>
#include "ros/ros.h"
#include "ros/time.h"
#include "EKF.h"
#include "imu_common/imu.h"
#include "imu_common/yei_msg.h"
#include "std_srvs/Empty.h"
#include "std_msgs/Bool.h"
#include "quaternion_util.h"

class yei_EKF {

public:

    yei_EKF(ros::NodeHandle nh, int freq);

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
                dt;

        float   q_sensor_cal[4];

        VectorXd x_init, r_init;
        MatrixXd P_init, Q_init, R_init;
        VectorXd processVar, measureVar;
        VectorXd velocity, Dvelocity, acc, a0;

        std::string bias_path;

        ros::ServiceServer  calibrate_serv_;
        ros::ServiceServer  pitch_roll_ref_;
        ros::ServiceServer  yaw_ref_;
        imu_common::imu     data;

        bool initialize(ros::NodeHandle &nh, Vector3d &rad);
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
    bool checkCalibration();
    static bool update(imu& device);
    void filter(imu& device);
    static bool publish(imu& device);
    bool publishall;

    imu R_shank;
    imu R_thigh;
    imu L_foot;

private:
    Vector3d Len_shank, Len_thigh, Rad_foot, Rad_shank, Rad_thigh, Rad_torso, Rad_single;

    int frequency;
    int setting;
    ros::NodeHandle node_handle_;
    ros::Rate rate;

};


#endif //yei_EKF_H
