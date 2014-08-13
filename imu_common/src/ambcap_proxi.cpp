#include "ambcap_proxi.h"

  ambcap_proxi::ambcap_proxi(ros::NodeHandle h, int desired_freq) : node_handle_(h), rate(desired_freq), reset_pose_(true),
      desired_freq_(desired_freq)
  {    
    // Set up butterworth filter for step detection.
    const vector<float> a = {1.0, -1.561018075800718, 0.641351538057563};
    const vector<float> b = {0.800592403464570, -1.601184806929141, 0.800592403464570};
    left_butter_.reset(new Butter(b, a));
    right_butter_.reset(new Butter(b, a));

    first_run = true;
    newDataRShank  = false;
    newDataRThigh = false;
    newDataLShank = false;
    newDataLThigh = false;

    Rshank_sub_ = node_handle_.subscribe("r_shank", 1, &ambcap_proxi::RshankCall, this);
    Rthigh_sub_ = node_handle_.subscribe("r_thigh", 1, &ambcap_proxi::RthighCall, this);
    Lshank_sub_ = node_handle_.subscribe("r_shank", 1, &ambcap_proxi::LshankCall, this);
    Lthigh_sub_ = node_handle_.subscribe("l_thigh", 1, &ambcap_proxi::LthighCall, this);

    R_Joints_pos = h.advertise<geometry_msgs::Vector3>("R_joints_pos", desired_freq_);
    R_Joints_vel = h.advertise<geometry_msgs::Vector3>("R_joints_vel", desired_freq_);

    L_Joints_pos = h.advertise<geometry_msgs::Vector3>("L_joints_pos", desired_freq_);
    L_Joints_vel = h.advertise<geometry_msgs::Vector3>("L_joints_vel", desired_freq_);
  }

  void ambcap_proxi::updatePose() { //
    // Left: Quaternion in Earth Fixed Frame.
    quat::prod(qL_f_meas, qLf_s, qLf_e); // Foot to earth
    quat::prod(qL_s_meas, qLs_s, qLs_e); // Shank to earth
    quat::prod(qL_t_meas, qLt_s, qLt_e); // Thigh to earth

    // Left: Invert the joint quaternion.
    float qLs_e_inv[4], qLt_e_inv[4], qLf_e_inv[4];
    quat::inv(qLf_e, qLf_e_inv);
    quat::inv(qLs_e, qLs_e_inv);
    quat::inv(qLt_e, qLt_e_inv);

    // Left: Relative rotation between joints.
    quat::prod(qLf_e_inv, qLs_e, qLfs); // Foot to shank
    quat::prod(qLs_e_inv, qLt_e, qLst); // Shank to thigh

    // Left: Convert to fixed angles.
    quat::eulerXZY(qLst, hLst);
    quat::eulerXZY(qLfs, hLfs);
    quat::eulerXZY(qLf_e, hLfe);

    // Publish the messages
    Joints_msg.x = hLst[2]; // right knee joint
    Joints_msg.y = hLfs[2]; // right ankle joint
    Joints_msg.z = hLfe[2]; // right foot link
    L_Joints_pos.publish(Joints_msg);

    Joints_msg.x = -(Lt_vel[1] - Ls_vel[1]); // right knee joint velocity = thigh - shank
    Joints_msg.y = -(Ls_vel[1] - 0.f); // right ankle joint velocity  = shank - foot*0.0
    Joints_msg.z = 0.f; //Same convention as huihua = 0.f
    L_Joints_vel.publish(Joints_msg);

    //* -------------------- *//

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
    quat::prod(qRf_e_inv, qRs_e, qRfs); // Foot to shank
    quat::prod(qRs_e_inv, qRt_e, qRst); // Shank to thigh

    // Right: Convert to fixed angles.
    quat::eulerXZY(qRst, hRst);
    quat::eulerXZY(qRfs, hRfs);
    quat::eulerXZY(qRf_e, hRfe);

    newDataRShank = false;
    newDataRThigh = false;
    newDataLShank = false;
    newDataLThigh = false;

    // Publish the messages
    Joints_msg.x = hRst[2]; // right knee joint
    Joints_msg.y = hRfs[2]; // right ankle joint
    Joints_msg.z = hRfe[2]; // right foot link
    R_Joints_pos.publish(Joints_msg);

    Joints_msg.x = -(Rt_vel[1] - Rs_vel[1]); // right knee joint velocity = thigh - shank
    Joints_msg.y = -(Rs_vel[1] - 0.f); // right ankle joint velocity  = shank - foot*0.0
    Joints_msg.z = 0.f; //Same convention as huihua = 0.f
    R_Joints_vel.publish(Joints_msg);

  }

  void ambcap_proxi::RshankCall(const imu_common::imu& reading) {
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

  void ambcap_proxi::RthighCall(const imu_common::imu& reading) {
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

  void ambcap_proxi::LshankCall(const imu_common::imu& reading) {
    qL_s_meas[0] = reading.orientation.w;
    qL_s_meas[1] = reading.orientation.x;
    qL_s_meas[2] = reading.orientation.y;
    qL_s_meas[3] = reading.orientation.z;

    qLs_s[0] = reading.orientation_REF.w;
    qLs_s[1] = reading.orientation_REF.x;
    qLs_s[2] = reading.orientation_REF.y;
    qLs_s[3] = reading.orientation_REF.z;

    Ls_vel[0] = reading.angular_velocity.x;
    Ls_vel[1] = reading.angular_velocity.y;
    Ls_vel[2] = reading.angular_velocity.z;

    newDataLShank = true;
  }

  void ambcap_proxi::LthighCall(const imu_common::imu& reading) {
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


  void ambcap_proxi::spinOnce() { 
    if ( newDataRShank && newDataRThigh && newDataLShank && newDataLThigh ) {
        updatePose();
    }
  }
