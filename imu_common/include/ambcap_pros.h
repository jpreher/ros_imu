#ifndef AMBPROS_COMMON_H
#define AMBPROS_COMMON_H

#include "ros/ros.h"
#include "imu_common/imu.h"
#include "quaternion_util.h"
#include "butterworth_util.h"
#include "limits.hpp"
#include "std_srvs/Empty.h"

typedef std::shared_ptr<Butter> ButterPtr;
typedef std::shared_ptr<control_utilities::RateLimiter> LimitPtr;

class ambcap_pros {
public:
  ButterPtr left_butter_;
  ButterPtr right_butter_;

  LimitPtr limiter_;

  float right_impact_checker;
  float left_impact_checker;

  ros::Subscriber Rshank_sub_;
  ros::Subscriber Rthigh_sub_;  
  ros::Subscriber Rfoot_sub_;
  ros::Subscriber Lthigh_sub_;
  bool first;
  bool newDataRShank, newDataRThigh, newDataRFoot, newDataLThigh;
  bool RightIsStance;

  bool reset_pose_;

  ros::ServiceServer reset_pose_serv_;

  float Vdesired;
  float phip, phip_o;
  float tau;
  float switch_threshold;

  // Right leg variables
  float qRf_e_ref[4], qRs_e_ref[4], qRt_e_ref[4]; // Link to earth reference frame.
  float qRf_s[4], qRs_s[4], qRt_s[4];             // Link body frame to sensor frame rotation.
  float qRf_e[4], qRs_e[4], qRt_e[4];             // Link body frame to earth frame rotation.
  float qRfs[4];                    // Foot to shank rotation.
  float qRst[4];                    // Shank to thigh rotation.
  float hRst[3];                    // Euler angle shank to thigh.
  float hRfs[3];                    // Euler angle foot to shank.
  float hRfe[3];                   // Euler angle shank to earth.
  float qR_s_meas[4], qR_t_meas[4]; // Measured (sensor frame).
  float qR_f_meas[4];

  float Rf_vel[3], Rs_vel[3], Rt_vel[3], Lt_vel[3];

  // Left leg variables
  float qLt_e_ref[4]; // Link to earth reference frame.
  float qLs_s[4], qLt_s[4];         // Link body frame to sensor frame rotation.
  float qLs_e[4], qLt_e[4];         // Link body frame to earth frame rotation.
  float hLte[3];                   // Euler angle shank to earth.
  float qL_s_meas[4], qL_t_meas[4]; // Measured (sensor frame).

  int desired_freq_;

  ros::NodeHandle node_handle_;
  ros::Rate rate;

  ambcap_pros(ros::NodeHandle h);
  void initPose();
  void updatePose();
  void RshankCall(const imu_common::imu& reading);
  void RthighCall(const imu_common::imu& reading);
  void RfootCall(const imu_common::imu& reading);
  void LthighCall(const imu_common::imu& reading);
  void check_srvs();
  bool reset_pose(std_srvs::Empty::Request  &req,
                  std_srvs::Empty::Response &resp);
  void spinOnce();
};


#endif
