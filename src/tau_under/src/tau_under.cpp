#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include <leg_pose/legPose.h>
#include "quaternion_util.h"


class tau_under {
public:
  ros::Subscriber shank_sub_;
  ros::Subscriber thigh_sub_;  
  bool first;
  bool newDataShank, newDataThigh;
  float qs_e_ref[4], qt_e_ref[4]; // Link to earth reference frame.
  float qs_s[4], qt_s[4];         // Link body frame to sensor frame rotation.
  float qs_e[4], qt_e[4];         // Link body frame to earth frame rotation.
  float qst[4];                   // Shank to thigh rotation.
  float hst[3];                   // Euler angle shank to thigh.
  float hs_e[3];                  // Euler angle shank to earth.
  float q_s_meas[4], q_t_meas[4]; // Measured (sensor frame).
  int desired_freq_;

  ros::NodeHandle node_handle_;
  leg_pose::legPose pose_data;
  ros::Rate rate;

  ros::Publisher pose_pub_;

  legPose(ros::NodeHandle h) : node_handle_(h), desired_freq_(400), rate(400) 
  {
    //rate = node_handle_.Rate(desired_freq_); // Run rate twice as fast as publishers (400 Hz)
    pose_pub_ = node_handle_.advertise<leg_pose::legPose>("pose", desired_freq_);

    first = true;
    newDataShank = false;
    newDataThigh = false;
    qs_e_ref[0] = qt_e_ref[0] = 1.0f;
    qs_e_ref[1] = qt_e_ref[1] = 0.0f;
    qs_e_ref[2] = qt_e_ref[2] = 0.0f;
    qs_e_ref[3] = qt_e_ref[3] = 0.0f;
  }

  void initPose() {
    float qss_e_inv[4], qts_e_inv[4];

    ROS_INFO("Setting up initial pose.");
    // Invert the measured reference pose measurement.
    quat::inv(q_s_meas, qss_e_inv);
    quat::inv(q_t_meas, qts_e_inv);

    // Create composite quaternion for reference pose.
    quat::prod(qss_e_inv, qs_e_ref, qs_s);
    quat::prod(qts_e_inv, qt_e_ref, qt_s);
  }

  void updateLeg() {
    // Quaternion in Earth Fixed Frame.
    quat::prod(q_s_meas, qs_s, qs_e); //Shank to earth
    quat::prod(q_t_meas, qt_s, qt_e); //Thigh to earth

    // Invert the joint quaternion.
    float qs_e_inv[4], qt_e_inv[4];
    quat::inv(qs_e, qs_e_inv);
    quat::inv(qt_e, qt_e_inv);

    // Relative rotation between joints.
    quat::prod(qt_e_inv, qs_e, qst); //Shank to thigh

    // Convert to fixed angles.
    quat::eulerXZY(qst, hst);
    quat::eulerXZY(qs_e, hs_e);
  }

  void shankCall(const sensor_msgs::Imu& reading) {
    q_s_meas[0] = reading.orientation.w;
    q_s_meas[1] = reading.orientation.x;
    q_s_meas[2] = reading.orientation.y;
    q_s_meas[3] = reading.orientation.z;
    
    newDataShank = true;
  }

  void thighCall(const sensor_msgs::Imu& reading) {
    q_t_meas[0] = reading.orientation.w;
    q_t_meas[1] = reading.orientation.x;
    q_t_meas[2] = reading.orientation.y;
    q_t_meas[3] = reading.orientation.z;   

    newDataThigh = true;
  }

  void publish_data() {
    if ( first ) {
      initPose();
      first = false;
    } else {
      updateLeg();
    }

    pose_data.shank_earth = hs_e[2];
    pose_data.thigh_shank = hst[2];

    pose_pub_.publish(pose_data);

    newDataThigh = false;
    newDataShank = false;
  }

  void spin() {
    while (ros::ok()){
      if ( newDataThigh && newDataShank )
        publish_data();
      
      ros::spinOnce();
      rate.sleep();
    } 
  }
};

int main(int argc, char **argv)
{
  int desired_freq_ = 400;
  ros::init(argc, argv, "legPose_right_no_foot");
  ros::NodeHandle n;
  legPose lp(n);
  lp.shank_sub_ = n.subscribe("right_leg/data1", 100, &legPose::shankCall, &lp);
  lp.thigh_sub_ = n.subscribe("right_leg/data2", 100, &legPose::thighCall, &lp);

  lp.spin();

  return 0;
}

/* TASKS
- Subscribers for each of the joints
- On all flag calculate angles and publish tau_under
- dtau? <- investigate further


*/