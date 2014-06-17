#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include <tau_under/tau_under_msg.h>
#include "quaternion_util.h"
#include "butterworth_util.h"
#include "std_srvs/Empty.h"


typedef std::shared_ptr<Butter> ButterPtr;

class tau_func {
public:
  ButterPtr left_butter_;
  ButterPtr right_butter_;

  float right_impact_checker;
  float left_impact_checker;

  ros::Subscriber Rshank_sub_;
  ros::Subscriber Rthigh_sub_;  
  ros::Subscriber Lshank_sub_;
  ros::Subscriber Lthigh_sub_;
  bool first;
  bool newDataRShank, newDataRThigh, newDataLShank, newDataLThigh;
  bool RightIsStance;

  bool reset_pose_;
  bool start_tau_;
  bool stop_tau_;
  bool tau_running_;

  ros::ServiceServer reset_pose_serv_;
  ros::ServiceServer start_tau_serv_;
  ros::ServiceServer stop_tau_serv_;

  float Vdesired;
  float phip, phip_o;
  float tau;
  float switch_threshold;

  // Right leg variables
  float qRs_e_ref[4], qRt_e_ref[4]; // Link to earth reference frame.
  float qRs_s[4], qRt_s[4];         // Link body frame to sensor frame rotation.
  float qRs_e[4], qRt_e[4];         // Link body frame to earth frame rotation.
  float qRst[4];                    // Shank to thigh rotation.
  float hRst[3];                    // Euler angle shank to thigh.
  float hRs_e[3];                   // Euler angle shank to earth.
  float qR_s_meas[4], qR_t_meas[4]; // Measured (sensor frame).

  // Left leg variables
  float qLs_e_ref[4], qLt_e_ref[4]; // Link to earth reference frame.
  float qLs_s[4], qLt_s[4];         // Link body frame to sensor frame rotation.
  float qLs_e[4], qLt_e[4];         // Link body frame to earth frame rotation.
  float qLst[4];                    // Shank to thigh rotation.
  float hLst[3];                    // Euler angle shank to thigh.
  float hLs_e[3];                   // Euler angle shank to earth.
  float qL_s_meas[4], qL_t_meas[4]; // Measured (sensor frame).

  float Lt, Lc; // LENGTH OF THIGH AND CALF

  int desired_freq_;

  ros::NodeHandle node_handle_;
  tau_under::tau_under_msg pose_data;
  ros::Rate rate;

  ros::Publisher tau_pub_;

  tau_func(ros::NodeHandle h) : node_handle_(h), desired_freq_(600), rate(desired_freq_),
           tau_running_(false), reset_pose_(false), start_tau_(false), stop_tau_(false)
  {
    ros::NodeHandle tau_node_handle(node_handle_, "pose");
    tau_pub_ = node_handle_.advertise<tau_under::tau_under_msg>("tau_under", desired_freq_);
    
    // Set up butterworth filter for step detection.
    const vector<float> a = {1.0, -1.561018075800718, 0.641351538057563};
    const vector<float> b = {0.800592403464570, -1.601184806929141, 0.800592403464570};
    left_butter_.reset(new Butter(b, a));
    right_butter_.reset(new Butter(b, a));

    first = true;
    newDataRShank = false;
    newDataRThigh = false;
    newDataLShank = false;
    newDataLShank = false;
    qRs_e_ref[0] = qRt_e_ref[0] = 1.0f;
    qRs_e_ref[1] = qRt_e_ref[1] = 0.0f;
    qRs_e_ref[2] = qRt_e_ref[2] = 0.0f;
    qRs_e_ref[3] = qRt_e_ref[3] = 0.0f;

    qLs_e_ref[0] = qLt_e_ref[0] = 1.0f;
    qLs_e_ref[1] = qLt_e_ref[1] = 0.0f;
    qLs_e_ref[2] = qLt_e_ref[2] = 0.0f;
    qLs_e_ref[3] = qLt_e_ref[3] = 0.0f;

    ros::param::get("tauParam/v_desired", Vdesired);
    ros::param::get("tauParam/length_calf", Lc);
    ros::param::get("tauParam/length_thigh", Lt);
    ros::param::get("tauParam/right_start", RightIsStance);
    ros::param::get("tauParam/switch_threshold", switch_threshold);

    reset_pose_serv_ = tau_node_handle.advertiseService("reset_pose", &tau_func::reset_pose, this);
    start_tau_serv_ =  tau_node_handle.advertiseService("start_tau", &tau_func::start_tau, this);
    stop_tau_serv_ = tau_node_handle.advertiseService("stop_tau", &tau_func::stop_tau, this);
  }

  void initPose() { //DONE
    float qRss_e_inv[4], qRts_e_inv[4];
    float qLss_e_inv[4], qLts_e_inv[4];
    float qhs_e_inv[4];

    ROS_INFO("Setting up initial pose.");
    // Invert the measured reference pose measurement.
    quat::inv(qR_s_meas, qRss_e_inv);
    quat::inv(qR_t_meas, qRts_e_inv);
    quat::inv(qL_s_meas, qLss_e_inv);
    quat::inv(qL_t_meas, qLts_e_inv);

    // Create composite quaternion for reference pose.
    quat::prod(qRss_e_inv, qRs_e_ref, qRs_s);
    quat::prod(qRts_e_inv, qRt_e_ref, qRt_s);
    quat::prod(qLss_e_inv, qLs_e_ref, qLs_s);
    quat::prod(qLts_e_inv, qLt_e_ref, qLt_s);
  }

  void updatePose() { //
    // Left: Quaternion in Earth Fixed Frame.
    quat::prod(qL_s_meas, qLs_s, qLs_e); // Shank to earth
    quat::prod(qL_t_meas, qLt_s, qLt_e); // Thigh to earth

    // Left: Invert the joint quaternion.
    float qLs_e_inv[4], qLt_e_inv[4];
    quat::inv(qLs_e, qLs_e_inv);
    quat::inv(qLt_e, qLt_e_inv);

    // Left: Relative rotation between joints.
    quat::prod(qLt_e_inv, qLs_e, qLst); // Shank to thigh

    // Left: Convert to fixed angles.
    quat::eulerXZY(qLst, hLst);
    quat::eulerXZY(qLs_e, hLs_e);

    hLst[2] = -hLst[2];
    hLs_e[2] = -hLs_e[2];

    // Right: Quaternion in Earth Fixed Frame.
    quat::prod(qR_s_meas, qRs_s, qRs_e); // Shank to earth
    quat::prod(qR_t_meas, qRt_s, qRt_e); // Thigh to earth

    // Right: Invert the joint quaternion.
    float qRs_e_inv[4], qRt_e_inv[4];
    quat::inv(qRs_e, qRs_e_inv);
    quat::inv(qRt_e, qRt_e_inv);

    // Right: Relative rotation between joints.
    quat::prod(qRt_e_inv, qRs_e, qRst); // Shank to thigh

    // Right: Convert to fixed angles.
    quat::eulerXZY(qRst, hRst);
    quat::eulerXZY(qRs_e, hRs_e);

    hRst[2] = -hRst[2];
    hRs_e[2] = -hRs_e[2];
  }

  void RshankCall(const sensor_msgs::Imu& reading) {
    qR_s_meas[0] = reading.orientation.w;
    qR_s_meas[1] = reading.orientation.x;
    qR_s_meas[2] = reading.orientation.y;
    qR_s_meas[3] = reading.orientation.z;

    float tempacc[3];
    tempacc[0] = reading.linear_acceleration.x;
    tempacc[1] = reading.linear_acceleration.y;
    tempacc[2] = reading.linear_acceleration.z;
    tempacc[0] = sqrt(tempacc[0] * tempacc[0] + tempacc[1] * tempacc[1] + tempacc[2] * tempacc[2]);

    right_impact_checker = right_butter_->update(tempacc[0]);
    
    newDataRShank = true;
  }

  void RthighCall(const sensor_msgs::Imu& reading) {
    qR_t_meas[0] = reading.orientation.w;
    qR_t_meas[1] = reading.orientation.x;
    qR_t_meas[2] = reading.orientation.y;
    qR_t_meas[3] = reading.orientation.z;   

    newDataRThigh = true;
  }

  void LshankCall(const sensor_msgs::Imu& reading) {
    qL_s_meas[0] = reading.orientation.w;
    qL_s_meas[1] = reading.orientation.x;
    qL_s_meas[2] = reading.orientation.y;
    qL_s_meas[3] = reading.orientation.z;

    float tempacc[3];
    tempacc[0] = reading.linear_acceleration.x;
    tempacc[1] = reading.linear_acceleration.y;
    tempacc[2] = reading.linear_acceleration.z;
    tempacc[0] = sqrt(tempacc[0] * tempacc[0] + tempacc[1] * tempacc[1] + tempacc[2] * tempacc[2]);

    left_impact_checker = left_butter_->update(tempacc[0]);
    
    newDataLShank = true;
  }

  void LthighCall(const sensor_msgs::Imu& reading) {
    qL_t_meas[0] = reading.orientation.w;
    qL_t_meas[1] = reading.orientation.x;
    qL_t_meas[2] = reading.orientation.y;
    qL_t_meas[3] = reading.orientation.z;   

    newDataLThigh = true;
  }

  void resetTau() {
    if ( RightIsStance ) {
      // Calculate phip_o with right
      // nonlinear phip
      //phip_o = - Lc*sin(hRs_e[2]) - Lt*sin(hRs_e[2] + hRst[2]);
      // linear phip
      phip_o = Lc*hRs_e[2] + Lt*(hRs_e[2] + hRst[2]);
    } else {
      // Calculate phip_o with left
      //phip_o = - Lc*sin(hLs_e[2]) - Lt*sin(hLs_e[2] + hLst[2]);
      phip_o = Lc*hLs_e[2] + Lt*(hLs_e[2] + hLst[2]);
    }
    calcTau();
  }

  void calcTau(){
    if ( RightIsStance ) {
      // nonlinear tau
      // tau = (-Lc*sin(hRs_e[2]) - Lt*sin(hRs_e[2] + hRst[2]) - phip_o) / Vdesired;
      // linear tau
      tau = (Lc*(hRs_e[2]) + Lt*(hRs_e[2] + hRst[2]) - phip_o) / Vdesired;
    } else {
      // nonlinear tau
      //tau = (-Lc*sin(hLs_e[2]) - Lt*sin(hLs_e[2] + hLst[2]) - phip_o) / Vdesired;
      // linear tau
      tau = (Lc*(hLs_e[2]) + Lt*(hLs_e[2] + hLst[2]) - phip_o) / Vdesired;
    }
  }

  bool checkReset() {
    // Check the butterworth filter magnitudes and publish true if reset is to be applied
    if ( RightIsStance ) {
      if ( right_impact_checker > switch_threshold ) {
        return true;
      } else { return false; }
    } else {
      if ( left_impact_checker > switch_threshold ) {
        return true;
      } else { return false; }
    }
  }

  void publish_data() { //DONE
    if ( first ) {
      initPose();
      resetTau();
      first = false;
    } else {
      updatePose();
    }

    if ( tau_running_ ) { // Dont run the tau calculation until the user starts it.
      if ( checkReset() ) {
        if ( RightIsStance ) {
          RightIsStance = false;
          resetTau();
        } else {
          RightIsStance = true;
          resetTau();
        }
      }

      calcTau();
    }

    pose_data.right_shank_earth = hRs_e[2];
    pose_data.right_thigh_shank = hRst[2];
    pose_data.left_shank_earth = hLs_e[2];
    pose_data.left_thigh_shank = hLst[2];
    pose_data.tau = tau;

    tau_pub_.publish(pose_data);

    newDataRThigh = false;
    newDataRShank = false;
    newDataLThigh = false;
    newDataLShank = false;
  }

  void spin() { //DONE
    while (ros::ok()){
      if ( newDataRThigh && newDataRShank && newDataLThigh && newDataLShank )
        publish_data();
        check_srvs();
        ros::spinOnce();
        rate.sleep();
    } 
  }

  void check_srvs() {
    if ( reset_pose_ ) {
      ROS_INFO("Re-initializing pose.");
      initPose();
      reset_pose_ = false;
    }
    if ( start_tau_ ) {
      ROS_INFO("Starting tau calculation.");
      tau_running_ = true;
      start_tau_ = false;
      resetTau();
    }
    if ( stop_tau_ ) {
      ROS_INFO("Stopping tau calculation.");
      stop_tau_ = false;
      tau_running_ = false;
    }
  }

  bool reset_pose(std_srvs::Empty::Request  &req,
                  std_srvs::Empty::Response &resp) {
    reset_pose_ = true;
    return true;
  }

  bool start_tau(std_srvs::Empty::Request  &req,
                std_srvs::Empty::Response &resp) {
    start_tau_ = true;
    return true;
  }

  bool stop_tau(std_srvs::Empty::Request  &req,
                std_srvs::Empty::Response &resp) {
    stop_tau_ = true;
    return true;
  }
};

int main(int argc, char **argv) { //DONE
  ros::init(argc, argv, "legPose_right_no_foot");
  ros::NodeHandle n;
  tau_func tu(n);
  tu.Rshank_sub_ = n.subscribe("human_under/data3", 100, &tau_func::RshankCall, &tu);
  tu.Rthigh_sub_ = n.subscribe("human_under/data4", 100, &tau_func::RthighCall, &tu);
  tu.Lshank_sub_ = n.subscribe("human_under/data1", 100, &tau_func::LshankCall, &tu);
  tu.Lthigh_sub_ = n.subscribe("human_under/data2", 100, &tau_func::LthighCall, &tu);

  tu.spin();

  return 0;
}

/* TASKS
DONE- Subscribers for each of the joints
- On all flag calculate angles and publish tau_under
- dtau? <- investigate further
- Calculate each joint

*/