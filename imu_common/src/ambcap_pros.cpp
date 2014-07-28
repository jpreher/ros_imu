#include "ambcap_pros.h"

  ambcap_pros::ambcap_pros(ros::NodeHandle h) : node_handle_(h), rate(desired_freq_), reset_pose_(true)
  {    
    // Set up butterworth filter for step detection.
    const vector<float> a = {1.0, -1.561018075800718, 0.641351538057563};
    const vector<float> b = {0.800592403464570, -1.601184806929141, 0.800592403464570};
    left_butter_.reset(new Butter(b, a));
    right_butter_.reset(new Butter(b, a));

    first = true;
    newDataRFoot  = false;
    newDataRShank = false;
    newDataRThigh = false;
    newDataLThigh = false;
    qRf_e_ref[0] = qRs_e_ref[0] = qRt_e_ref[0] = 1.0f;
    qRf_e_ref[1] = qRs_e_ref[1] = qRt_e_ref[1] = 0.0f;
    qRf_e_ref[2] = qRs_e_ref[2] = qRt_e_ref[2] = 0.0f;
    qRf_e_ref[3] = qRs_e_ref[3] = qRt_e_ref[3] = 0.0f;

    qLt_e_ref[0] = 1.0f;
    qLt_e_ref[1] = 0.0f;
    qLt_e_ref[2] = 0.0f;
    qLt_e_ref[3] = 0.0f;

    reset_pose_serv_ = node_handle_.advertiseService("reset_pose", &ambcap_pros::reset_pose, this);

    Rshank_sub_ = node_handle_.subscribe("r_shank", 1, &ambcap_pros::RshankCall, this);
    Rthigh_sub_ = node_handle_.subscribe("r_thigh", 1, &ambcap_pros::RthighCall, this);
    Rfoot_sub_ = node_handle_.subscribe("r_foot", 1, &ambcap_pros::RfootCall, this);
    Lthigh_sub_ = node_handle_.subscribe("l_thigh", 1, &ambcap_pros::LthighCall, this);
  }

  void ambcap_pros::initPose() { //DONE
    float qRfs_e_inv[4];
    float qRss_e_inv[4], qRts_e_inv[4];
    float qLts_e_inv[4];

    ROS_INFO("Setting up initial pose.");
    // Invert the measured reference pose measurement.
    quat::inv(qR_f_meas, qRfs_e_inv);
    quat::inv(qR_s_meas, qRss_e_inv);
    quat::inv(qR_t_meas, qRts_e_inv);
    quat::inv(qL_t_meas, qLts_e_inv);

    // Create composite quaternion for reference pose.
    quat::prod(qRfs_e_inv, qRf_e_ref, qRf_s);
    quat::prod(qRss_e_inv, qRs_e_ref, qRs_s);
    quat::prod(qRts_e_inv, qRt_e_ref, qRt_s);
    quat::prod(qLts_e_inv, qLt_e_ref, qLt_s);

    ROS_INFO("Foot Initialized at %f, %f, %f, %f", qRf_s[0], qRf_s[1], qRf_s[2], qRf_s[3]);
    ROS_INFO("Shank Initialized at %f, %f, %f, %f", qRs_s[0], qRs_s[1], qRs_s[2], qRs_s[3]);
    ROS_INFO("RThigh Initialized at %f, %f, %f, %f", qRt_s[0], qRt_s[1], qRt_s[2], qRt_s[3]);
    ROS_INFO("LThigh Initialized at %f, %f, %f, %f", qLt_s[0], qLt_s[1], qLt_s[2], qLt_s[3]);

 }

  void ambcap_pros::updatePose() { //
    // Left: Quaternion in Earth Fixed Frame.
    quat::prod(qL_t_meas, qLt_s, qLt_e); // Thigh to earth

    // Left: Invert the joint quaternion.
    float qLt_e_inv[4];

    // Left: Convert to fixed angles.
    quat::eulerXZY(qLt_e, hLte);

    // Right: Quaternion in Earth Fixed Frame.
    quat::prod(qR_f_meas, qRf_s, qRf_e); // Foot to earth
    quat::prod(qR_s_meas, qRs_s, qRs_e); // Shank to earth
    quat::prod(qR_t_meas, qRt_s, qRt_e); // Thigh to earth

    // Right: Invert the joint quaternion.
    float qRs_e_inv[4], qRt_e_inv[4], qRf_e_inv[4];
    quat::inv(qRf_e, qRf_e_inv);
    quat::inv(qRs_e, qRs_e_inv);
    quat::inv(qRt_e, qRt_e_inv);

    // Right: Relative rotation between joints.
    quat::prod(qRs_e_inv, qRf_e, qRfs); // Foot to shank
    quat::prod(qRt_e_inv, qRs_e, qRst); // Shank to thigh

    // Right: Convert to fixed angles.
    quat::eulerXZY(qRst, hRst);
    quat::eulerXZY(qRfs, hRfs);
    quat::eulerXZY(qRf_e, hRfe);

    newDataRShank = false;
    newDataRThigh = false;
    newDataRFoot = false;
    newDataLThigh = false;
  }

  void ambcap_pros::RshankCall(const imu_common::imu& reading) {
    qR_s_meas[0] = reading.orientation.w;
    qR_s_meas[1] = reading.orientation.x;
    qR_s_meas[2] = reading.orientation.y;
    qR_s_meas[3] = reading.orientation.z;

    Rs_vel[0] = reading.angular_velocity.x;
    Rs_vel[1] = reading.angular_velocity.y;
    Rs_vel[2] = reading.angular_velocity.z;

    newDataRShank = true;

  }

  void ambcap_pros::RthighCall(const imu_common::imu& reading) {
    qR_t_meas[0] = reading.orientation.w;
    qR_t_meas[1] = reading.orientation.x;
    qR_t_meas[2] = reading.orientation.y;
    qR_t_meas[3] = reading.orientation.z;

    Rt_vel[0] = reading.angular_velocity.x;
    Rt_vel[1] = reading.angular_velocity.y;
    Rt_vel[2] = reading.angular_velocity.z;

    newDataRThigh = true;
  }

  void ambcap_pros::RfootCall(const imu_common::imu& reading) {
    qR_f_meas[0] = reading.orientation.w;
    qR_f_meas[1] = reading.orientation.x;
    qR_f_meas[2] = reading.orientation.y;
    qR_f_meas[3] = reading.orientation.z;

    Rf_vel[0] = reading.angular_velocity.x;
    Rf_vel[1] = reading.angular_velocity.y;
    Rf_vel[2] = reading.angular_velocity.z;

    newDataRFoot = true;
  }

  void ambcap_pros::LthighCall(const imu_common::imu& reading) {
    qL_t_meas[0] = reading.orientation.w;
    qL_t_meas[1] = reading.orientation.x;
    qL_t_meas[2] = reading.orientation.y;
    qL_t_meas[3] = reading.orientation.z;   

    Lt_vel[0] = reading.angular_velocity.x;
    Lt_vel[1] = reading.angular_velocity.y;
    Lt_vel[2] = reading.angular_velocity.z;

    newDataLThigh = true;
  }

  void ambcap_pros::check_srvs() {
    if ( reset_pose_ ) {
      ROS_INFO("Re-initializing pose.");
      initPose();
      reset_pose_ = false;
    }
  }

  bool ambcap_pros::reset_pose(std_srvs::Empty::Request  &req,
                  std_srvs::Empty::Response &resp) {
    reset_pose_ = true;
    return true;
  }

  void ambcap_pros::spinOnce() { 
    if ( newDataRShank && newDataRThigh && newDataRFoot && newDataLThigh ) {
        check_srvs();
        updatePose();
    }
  }
