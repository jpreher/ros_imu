/*=====================================================================================================
// AMBCAP tools to enable easy configurability of reading from and number of imu devices in ROS.
//=====================================================================================================
//
// ROS class with objects and functionality to easily collect data from MPU 6050-9150 model IMU's
// and filter with a complimentary filer on the BBB.
//
// Date         Author          Notes
// 09/12/2014   Jake Reher      Initial Release
// - 2015       Jake Reher      Additional Changes for prosthetic - calibration, tuning, functionality
//
//=====================================================================================================*/
//---------------------------------------------------------------------------------------------------

#include "ambcap_pros.h"

  ambcap_pros::ambcap_pros(ros::NodeHandle h) : node_handle_(h), rate(desired_freq_), reset_pose_(true)
  {    
    // Set up butterworth filter for step detection.
    const vector<float> a = {1.0, -1.561018075800718, 0.641351538057563};
    const vector<float> b = {0.800592403464570, -1.601184806929141, 0.800592403464570};
    left_butter_.reset(new Butter(b, a));
    right_butter_.reset(new Butter(b, a));

    first_run = true;
    newDataRFoot  = false;
    newDataRShank = false;
    newDataRThigh = false;
    newDataLThigh = false;

    Rshank_sub_ = node_handle_.subscribe("r_shank", 1, &ambcap_pros::RshankCall, this);
    Rthigh_sub_ = node_handle_.subscribe("r_thigh", 1, &ambcap_pros::RthighCall, this);
    //Rfoot_sub_ = node_handle_.subscribe("r_foot", 1, &ambcap_pros::RfootCall, this);
    //Lthigh_sub_ = node_handle_.subscribe("l_thigh", 1, &ambcap_pros::LthighCall, this);

    Joints_pos = h.advertise<geometry_msgs::Vector3>("joints_pos", desired_freq_);
    Joints_vel = h.advertise<geometry_msgs::Vector3>("joints_vel", desired_freq_);

    // REMOVE THE FOOT AND PROS THIGH FROM FEEDBACK
    qR_f_meas[0] = 1.f;
    qR_f_meas[1] = 0.f;
    qR_f_meas[2] = 0.f;
    qR_f_meas[3] = 0.f;
    qRf_s[0] = 1.f;
    qRf_s[1] = 0.f;
    qRf_s[2] = 0.f;
    qRf_s[3] = 0.f;
    Rf_vel[0] = 0.f;
    Rf_vel[1] = 0.f;
    Rf_vel[2] = 0.f;

    qL_t_meas[0] = 1.f;
    qL_t_meas[1] = 0.f;
    qL_t_meas[2] = 0.f;
    qL_t_meas[3] = 0.f;
    qLt_s[0] = 1.f;
    qLt_s[1] = 0.f;
    qLt_s[2] = 0.f;
    qLt_s[3] = 0.f;
    Lt_vel[0] = 0.f;
    Lt_vel[1] = 0.f;
    Lt_vel[2] = 0.f;
  }

  // Class implementation of a spinOnce() type function.
  // Checks for new data, if both are new then update pose.
  void ambcap_pros::spinOnce() {
    if ( newDataRShank && newDataRThigh ) {
        updatePose();
    }
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
    quat::inv(qR_f_meas, qRf_e_inv);
    quat::inv(qR_s_meas, qRs_e_inv);
    quat::inv(qR_t_meas, qRt_e_inv);

    // Right: Relative rotation between joints.
    quat::prod(qRf_e_inv, qR_s_meas, qRfs); // Foot to shank
    quat::prod(qRs_e_inv, qR_t_meas, qRst); // Shank to thigh

    // Right: Convert to fixed angles.
    quat::eulerXZY(qRst, hRst);
    quat::eulerXZY(qRs_e, hRfs);
    quat::eulerXZY(qRf_e, hRfe);

    newDataRShank = false;
    newDataRThigh = false;
    newDataRFoot = false;
    newDataLThigh = false;

    // Publish the messages
    Joints_msg.x = -hRst[2]; // right knee joint
    Joints_msg.y = -hRfs[2]; // right ankle joint
    Joints_msg.z = hRfe[2]; // right foot link
    Joints_pos.publish(Joints_msg);

    Joints_msg.x = -(Rt_vel[1] - Rs_vel[1]); // right knee joint velocity = thigh - shank
    Joints_msg.y = -(Rs_vel[1] - 0.f); // right ankle joint velocity  = shank - foot*0.0
    Joints_msg.z = 0.f; //Same convention as huihua = 0.f
    Joints_vel.publish(Joints_msg);
  }

  /* FUNCTION stepHeight()
   * Function which calculates current height of human foot based on:
   *    -Current human pose.
   *    -Current prosthetic pose.
   *    -An array of body dimensions.
   * INPUT imu_data: vector of imu angles - (0) = ankle, (1) = knee.
   * INPUT pros_data: vector of prosthetic angles - (0) = ankle, (1) = knee.
   * INPUT links: vector of link lengths: (0) = shank, (1) = thigh.
   * RETURN double value of height difference between prosthetic foot and human foot.
   */
  double ambcap_pros::stepHeight(VectorXd &imu_data, VectorXd &pros_data, VectorXd &links) {
    double pros_h, human_h;
    human_h = links(0)*cos(imu_data(0)) + links(1)*cos(imu_data(1) - imu_data(0));
    pros_h  = links(0)*cos(pros_data(0)) + links(1)*cos(pros_data(1) - pros_data(0));

    return (pros_h - human_h);
  }

  // CALLBACK FUNCTIONS.
  //-------------------------------------------------------------------------------------------------------
  void ambcap_pros::RshankCall(const imu_common::imu& reading) {
    qR_s_meas[0] = reading.orientation.w;
    qR_s_meas[1] = reading.orientation.x;
    qR_s_meas[2] = reading.orientation.y;
    qR_s_meas[3] = reading.orientation.z;

    qRs_s[0] = reading.orientation_REF.w;
    qRs_s[1] = reading.orientation_REF.x;
    qRs_s[2] = reading.orientation_REF.y;
    qRs_s[3] = reading.orientation_REF.z;

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

    qRt_s[0] = reading.orientation_REF.w;
    qRt_s[1] = reading.orientation_REF.x;
    qRt_s[2] = reading.orientation_REF.y;
    qRt_s[3] = reading.orientation_REF.z;

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

    qRf_s[0] = reading.orientation_REF.w;
    qRf_s[1] = reading.orientation_REF.x;
    qRf_s[2] = reading.orientation_REF.y;
    qRf_s[3] = reading.orientation_REF.z;

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

    qLt_s[0] = reading.orientation_REF.w;
    qLt_s[1] = reading.orientation_REF.x;
    qLt_s[2] = reading.orientation_REF.y;
    qLt_s[3] = reading.orientation_REF.z;

    Lt_vel[0] = reading.angular_velocity.x;
    Lt_vel[1] = reading.angular_velocity.y;
    Lt_vel[2] = reading.angular_velocity.z;

    newDataLThigh = true;
  }
