/* IMU_Node for Beaglebone Black and MPU-9150 IMU
 * Written for two leg cases:
 * 		Left Leg: 4 IMU (foot, shank, thigh, hip)
 *		Right Leg: 3 IMU (foot, shank, thigh)
 * Author: Jake Reher
 */

#include "MPU9150.h"
#include "quaternion_util.h"

#include <assert.h>
#include <math.h>
#include <iostream>
#include <cstring>

#include <boost/format.hpp>

#include "ros/ros.h"
#include "ros/time.h"

#include <imu_common/imu.h>
#include "std_srvs/Empty.h"

#include "std_msgs/Bool.h"

//Sample frequency
float sampleFreq = 200.0f;

typedef std::shared_ptr<MPU9150> MPUptr;

class imuNode {
public:
	// Placeholders for the IMU classes
	MPUptr IMU1;

	// Keep track of number of IMU
	int numIMU;
	int imu1_bus;
	int imu1_addr;
	bool imu1_isvert;
	std::string imu1_name;

  	ros::Rate rate;

	// IMU messages are published with standard IMU sensor_msgs.
	imu_common::imu reading1;

	std::string was_slow_;

  	ros::NodeHandle node_handle_;
  	ros::NodeHandle private_node_handle_;
  	ros::Publisher imu1_data_pub_;
  	ros::ServiceServer calibrate_serv1_;
  	ros::Publisher is_calibrated1_pub_;

  	bool running;

  	bool autocalibrate_;
  	bool calibrate_requested1_;
  	bool calibrated1_;

	double desired_freq_;
  
  	imuNode(ros::NodeHandle h) : private_node_handle_("~"),
  		node_handle_(h), calibrate_requested1_(false),
		desired_freq_(sampleFreq),
  		rate(sampleFreq) 
  	{

  		ros::NodeHandle imu_node_handle(node_handle_, "human_under");

  		// Set up params for calibration servers
    	private_node_handle_.param("autocalibrate", autocalibrate_, true);
    	private_node_handle_.param("assume_calibrated1", calibrated1_, false);

    	// Set up IMU publishers
    	imu1_data_pub_ = imu_node_handle.advertise<imu_common::imu>("data1", desired_freq_);

    	// Calibration server for each IMU
    	calibrate_serv1_ = imu_node_handle.advertiseService("calibrate_gyro1", &imuNode::calibrate1, this);

    	// Publishers to release info on which IMU's are calibrated.
    	is_calibrated1_pub_ = imu_node_handle.advertise<std_msgs::Bool>("is_calibrated1", 1, false);

    	// Loop is not to run yet.
    	running = false;

    	//Load in values from the parameter server.
		std::string bias_path1;

		// IMU 1
		ros::param::get("/imu/imu1/name", imu1_name);
		ros::param::get("/imu/imu1/bus", imu1_bus);
		ros::param::get("/imu/imu1/addr", imu1_addr);
		ros::param::get("/imu/imu1/bias_path", bias_path1);
		ros::param::get("/imu/imu1/isvertical", imu1_isvert);

		ros::param::get("/imu/num_imu", numIMU);

		const char* imu1_biasPath = bias_path1.c_str();

		// Set up the proper classes for the IMUs
		ROS_INFO("Loading left IMU1 bias and setting up class.");
		IMU1.reset(new MPU9150(imu1_bus, imu1_addr, imu1_biasPath, sampleFreq, imu1_isvert, true));
  	}

	~imuNode()
	{
		stop();
	}

	int start() {
		std_msgs::Bool calibrate_msg;

		if ( running == false ) {
	  		// Turn on IMU1
	  		ROS_INFO("Initializing IMU1");
            IMU1->initialize();

	  		if (autocalibrate_){
	  			calibrate_requested1_ = true;
	  		}

	  		if ( autocalibrate_ || calibrate_requested1_ ){
	  			ROS_INFO("Calibrating IMU 1.");
	  			calibrateIMU(IMU1);
	  			calibrate_msg.data = true;
	  			is_calibrated1_pub_.publish(calibrate_msg);
	  			calibrate_requested1_ = false;
	  			autocalibrate_ = false;
	  		} else {
	  			ROS_INFO("Not calibrating IMU 1, use service to calibrate.");
	  		}


	  		ROS_INFO("All IMU sensors initialized.");

	  		running = true;
	  	}
  		return 0;
  	}

	int stop() {
		if ( running )
			running = false;

		return 0;
	}

	int publish_data() {
	    static double prevtime = 0;
	    double starttime = ros::Time::now().toSec();

	    getData(reading1);
	    
	    double endtime = ros::Time::now().toSec();
	    if (endtime - starttime > 0.01)
	    {
	        ROS_WARN("Gathering data took %f ms. Nominal is <5 ms.", 1000 * (endtime - starttime));
	        was_slow_ = "Full IMU loop was slow.";
	    }

	    imu1_data_pub_.publish(reading1);
	        
	    return(0);
	}

	bool spin() {
		while (!ros::isShuttingDown()) // Using ros::isShuttingDown to avoid restarting the node during a shutdown.
	    {
		    if (start() == 0)
		    {
		        while(node_handle_.ok()) {
		          	publish_data();
		          	check_srvs();
		          	ros::spinOnce();
		          	rate.sleep();
		        } 
		    } else {
		       	// No need for diagnostic here since a broadcast occurs in start
		       	// when there is an error.
		       	usleep(1000000);
		       	ros::spinOnce();
		    }
	    	
	    stop();

	    return true;
		}
	}

	void getData(imu_common::imu& data1) {
        IMU1->MahonyAHRSupdateIMU();

        data1.linear_acceleration.x = IMU1->v_acc[0];
        data1.linear_acceleration.y = IMU1->v_acc[1];
        data1.linear_acceleration.z = IMU1->v_acc[2];
 
        data1.angular_velocity.x = IMU1->v_gyr[0];
        data1.angular_velocity.y = IMU1->v_gyr[1];
        data1.angular_velocity.z = IMU1->v_gyr[2];

        data1.magnetometer.x = IMU1->v_mag[0];
        data1.magnetometer.y = IMU1->v_mag[1];
        data1.magnetometer.z = IMU1->v_mag[2];

        data1.orientation.w = IMU1->v_quat[0];
        data1.orientation.x = IMU1->v_quat[1];
        data1.orientation.y = IMU1->v_quat[2];
        data1.orientation.z = IMU1->v_quat[3];

        data1.header.stamp = ros::Time::now();
	}

	bool calibrate1(std_srvs::Empty::Request  &req,
   		        	std_srvs::Empty::Response &resp) {
		std_msgs::Bool calibrate_msg;
		calibrate_msg.data = true;
		calibrate_requested1_ = true;
		is_calibrated1_pub_.publish(calibrate_msg);
		return true;
	}

	void calibrateIMU(MPUptr& IMU){
		float tempx = 0.f;
		float tempy = 0.f;
		float tempz = 0.f;

		//Take 500 readings and average the values.
		for ( int i=0; i<500; i++ ) {
			IMU->read6DOF();
			tempx += IMU->v_gyr[0];
			tempy += IMU->v_gyr[1];
			tempz += IMU->v_gyr[2];
		}

		if ( IMU->vert_orient ) {
			//If the imu is vertical exchange x and z
			IMU->GYRbias[0] += tempz / 500.0f;
			IMU->GYRbias[2] -= tempx / 500.0f;
		} else {
			//Otherwise just average
			IMU->GYRbias[0] += tempx / 500.0f;
			IMU->GYRbias[2] += tempz / 500.0f;
		}
		//Y axis does not change.
		IMU->GYRbias[1] += tempy / 500.0f;

		ROS_INFO("Finished Calibration with offsets %f, %f", tempx/500.f, tempz/500.f);
	}

	void check_srvs() {
		std_msgs::Bool calibrate_msg;

		if ( calibrate_requested1_ ) {
			calibrateIMU(IMU1);
			calibrate_msg.data = true;
			is_calibrated1_pub_.publish(calibrate_msg);
			calibrate_requested1_ = false;
		}
	}

};

main(int argc, char** argv)
{
	// Init the ros node.
	ros::init(argc, argv, "mpu9150_node");
	ros::NodeHandle nh;

	imuNode in(nh);
	in.spin();

	return(0);
}