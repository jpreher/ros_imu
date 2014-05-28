/* IMU_Node for Beaglebone Black and MPU-9150 IMU
 * Written for two leg cases:
 * 		Left Leg: 4 IMU (foot, shank, thigh, hip)
 *		Right Leg: 3 IMU (foot, shank, thigh)
 * Author: Jake Reher
 */

#include "MPU9150.h"

#include <assert.h>
#include <math.h>
#include <iostream>
#include <cstring>

#include <boost/format.hpp>

#include "ros/ros.h"
#include "ros/time.h"

#include "sensor_msgs/Imu.h"
#include "std_srvs/Empty.h"

#include "std_msgs/Bool.h"

// Define which leg you want this node to handle
#define left_leg false
#define right_leg true

//Sample frequency
float sampleFreq = 150.0f;

typedef std::shared_ptr<MPU9150> MPUptr;

class imuNode {
public:
	// Placeholders for the IMU classes
	MPUptr IMU1;
	MPUptr IMU2;
	MPUptr IMU3;

	// Keep track of number of IMU
	int numIMU;
	int imu1_bus, imu2_bus, imu3_bus;
	int imu1_addr, imu2_addr, imu3_addr;
	bool imu1_isvert, imu2_isvert, imu3_isvert;
	std::string imu1_name, imu2_name, imu3_name;

  	ros::Rate rate;

	// IMU messages are published with standard IMU sensor_msgs.
	sensor_msgs::Imu reading1;
	sensor_msgs::Imu reading2;
	sensor_msgs::Imu reading3;

	std::string was_slow_;

  	ros::NodeHandle node_handle_;
  	ros::NodeHandle private_node_handle_;
  	ros::Publisher imu1_data_pub_;
  	ros::Publisher imu2_data_pub_;
  	ros::Publisher imu3_data_pub_;
  	ros::ServiceServer calibrate1_serv_;
  	ros::ServiceServer calibrate2_serv_;
  	ros::ServiceServer calibrate3_serv_;
  	ros::Publisher is_calibrated1_pub_;
  	ros::Publisher is_calibrated2_pub_;
  	ros::Publisher is_calibrated3_pub_;

  	bool running;

  	bool autocalibrate_;
  	bool calibrate_requested1_;
  	bool calibrate_requested2_;
  	bool calibrate_requested3_;
  	bool calibrated1_;
	bool calibrated2_;
	bool calibrated3_;


	double desired_freq_;
  
  	imuNode(ros::NodeHandle h) : private_node_handle_("~"),
  		node_handle_(h), calibrate_requested1_(false), calibrate_requested2_(false),
  		calibrate_requested3_(false), desired_freq_(sampleFreq),
  		rate(sampleFreq) 
  	{
	  	std::string leg;
  		if ( left_leg ) {
  			ROS_INFO("Leg is left");
  			leg = "left_leg";
  		} else {
  			ROS_INFO("Leg is right");
  			leg = "right_leg";
  		}
  		ros::NodeHandle imu_node_handle(node_handle_, leg);

  		// Set up params for calibration servers
    	private_node_handle_.param("autocalibrate", autocalibrate_, true);
    	private_node_handle_.param("assume_calibrated1", calibrated1_, false);
    	private_node_handle_.param("assume_calibrated2", calibrated2_, false);
       	private_node_handle_.param("assume_calibrated3", calibrated3_, false);

    	// Set up IMU publishers
    	imu1_data_pub_ = imu_node_handle.advertise<sensor_msgs::Imu>("data1", desired_freq_);
    	imu2_data_pub_ = imu_node_handle.advertise<sensor_msgs::Imu>("data2", desired_freq_);
    	imu3_data_pub_ = imu_node_handle.advertise<sensor_msgs::Imu>("data3", desired_freq_);

    	// Calibration server for each IMU
    	//calibrate1_serv_ = imu_node_handle.advertiseService("calibrate", &imuNode::calibrate, this);
    	//calibrate2_serv_ = imu_node_handle.advertiseService("calibrate", &imuNode::calibrate, this);
    	//calibrate3_serv_ = imu_node_handle.advertiseService("calibrate", &imuNode::calibrate, this);

    	// Publishers to release info on which IMU's are calibrated.
    	is_calibrated1_pub_ = imu_node_handle.advertise<std_msgs::Bool>("is_calibrated", 1, true);
    	is_calibrated2_pub_ = imu_node_handle.advertise<std_msgs::Bool>("is_calibrated", 1, true);
    	is_calibrated3_pub_ = imu_node_handle.advertise<std_msgs::Bool>("is_calibrated", 1, true);

    	// Loop is not to run yet.
    	running = false;

		// Pull values from the ROS parameter server and set up IMUs.
		////////// TODO: change this to the node handle and automate the loading of parameters
		if ( left_leg == right_leg ){
			ROS_WARN("Both legs are assigned (or unassigned), IMU parameters will not be correct!");

		} else if ( left_leg == true ){
			std::string bias_path1;
			std::string bias_path2;
			std::string bias_path3;
			// IMU 1
			ros::param::get("/imu/left_leg/imu1/name", imu1_name);
			ros::param::get("/imu/left_leg/imu1/bus", imu1_bus);
			ros::param::get("/imu/left_leg/imu1/addr", imu1_addr);
			ros::param::get("/imu/left_leg/imu1/bias_path", bias_path1);
			ros::param::get("/imu/left_leg/imu1/isvertical", imu1_isvert);
			// IMU 2
			ros::param::get("/imu/left_leg/imu2/name", imu2_name);
			ros::param::get("/imu/left_leg/imu2/bus", imu2_bus);
			ros::param::get("/imu/left_leg/imu2/addr", imu2_addr);
			ros::param::get("/imu/left_leg/imu2/bias_path", bias_path2);
			ros::param::get("/imu/left_leg/imu2/isvertical", imu2_isvert);
			// IMU 3
			ros::param::get("/imu/left_leg/imu3/name", imu3_name);
			ros::param::get("/imu/left_leg/imu3/bus", imu3_bus);
			ros::param::get("/imu/left_leg/imu3/addr", imu3_addr);
			ros::param::get("/imu/left_leg/imu3/bias_path", bias_path3);
			ros::param::get("/imu/left_leg/imu3/isvertical", imu3_isvert);

			ros::param::get("/imu/left_leg/num_imu", numIMU);

			const char* imu1_biasPath = bias_path1.c_str();
			const char* imu2_biasPath = bias_path2.c_str();
			const char* imu3_biasPath = bias_path3.c_str();

			// Set up the proper classes for the IMUs
			ROS_INFO("Loading left IMU1 bias and setting up class.");
			IMU1.reset(new MPU9150(imu1_bus, imu1_addr, imu1_biasPath, sampleFreq, imu1_isvert));
			ROS_INFO("Loading left IMU2 bias and setting up class.");
			IMU2.reset(new MPU9150(imu2_bus, imu2_addr, imu2_biasPath, sampleFreq, imu2_isvert));
			ROS_INFO("Loading left IMU3 bias and setting up class.");
			IMU3.reset(new MPU9150(imu3_bus, imu3_addr, imu3_biasPath, sampleFreq, imu3_isvert));

		} else if ( right_leg == true ){
			int imu1_bus, imu2_bus;
			int imt1_addr, imu2_addr;
			bool imu1_isvert, imu2_isvert;
			std::string bias_path1;
			std::string bias_path2;
			std::string bias_path3;
			// IMU 1
			ros::param::get("/imu/right_leg/imu1/name", imu1_name);
			ros::param::get("/imu/right_leg/imu1/bus", imu1_bus);
			ros::param::get("/imu/right_leg/imu1/addr", imu1_addr);
			ros::param::get("/imu/right_leg/imu1/bias_path", bias_path1);
			ros::param::get("/imu/right_leg/imu1/isvertical", imu1_isvert);
			// IMU 2
			ros::param::get("/imu/right_leg/imu2/name", imu2_name);
			ros::param::get("/imu/right_leg/imu2/bus", imu2_bus);
			ros::param::get("/imu/right_leg/imu2/addr", imu2_addr);
			ros::param::get("/imu/right_leg/imu2/bias_path", bias_path2);
			ros::param::get("/imu/right_leg/imu2/isvertical", imu2_isvert);

			const char* imu1_biasPath = bias_path1.c_str();
			const char* imu2_biasPath = bias_path2.c_str();

			if (imu1_isvert)
				ROS_INFO("IMU1 is vertical");

			ROS_INFO("Loading right IMU1 bias and setting up class.");
			IMU1.reset(new MPU9150(imu1_bus, imu1_addr, imu1_biasPath, sampleFreq, imu1_isvert));
			ROS_INFO("Loading right IMU2 bias and setting up class.");
			IMU2.reset(new MPU9150(imu2_bus, imu2_addr, imu2_biasPath, sampleFreq, imu2_isvert));

		} else {
			ROS_WARN("Node leg is improperly specified! Didn't do anything. Fix me up!");
		}
  	}

	~imuNode()
	{
		stop();
	}

	int start() {
		if ( running == false ) {
	  		// Turn on IMU1
	  		ROS_INFO("Initializing IMU1");
            IMU1->initialize();
	  		ROS_INFO("Initializing IMU2");
            IMU2->initialize();
            if ( numIMU > 2 ) {
            	ROS_INFO("Initializing IMU3");
            	IMU3->initialize();
            }

	  		if (autocalibrate_){
	  			calibrate_requested1_ = true;
	  			calibrate_requested2_ = true;
	  			calibrate_requested3_ = true;
	  		}

	  		if ( autocalibrate_ || calibrate_requested1_ ){
	  			ROS_INFO("Calibrating IMU 1.");
	  			calibrateIMU(IMU1);
	  			calibrate_requested1_ = false;
	  			autocalibrate_ = false;
	  		} else {
	  			ROS_INFO("Not calibrating IMU 1, use service to calibrate.");
	  		}
	  		if ( autocalibrate_ || calibrate_requested2_ ){
	  			ROS_INFO("Calibrating IMU 2.");
	  			calibrateIMU(IMU2);
	  			calibrate_requested2_ = false;
	  			autocalibrate_ = false;
	  		} else {
	  			ROS_INFO("Not calibrating IMU 2, use service to calibrate.");
	  		}
	  		if (numIMU > 2) {
	  			if ( autocalibrate_ || calibrate_requested3_ ){
	  				ROS_INFO("Calibrating IMU 3.");
	  				calibrateIMU(IMU3);
	  				calibrate_requested3_ = false;
	  				autocalibrate_ = false;
	  			} else {
	  				ROS_INFO("Not calibrating IMU 3, use service to calibrate.");
	  			}
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

	    if (prevtime && prevtime - starttime > 0.006)
	    {
	       	ROS_WARN("Full IMU loop took %f ms. Nominal is 5 ms.", 1000 * (prevtime - starttime));
	        was_slow_ = "Full IMU loop was slow.";
	    }

	    getData(reading1, reading2, reading3);
	    double endtime = ros::Time::now().toSec();
	    if (endtime - starttime > 0.006)
	    {
	        ROS_WARN("Gathering data took %f ms. Nominal is <5 ms.", 1000 * (endtime - starttime));
	        was_slow_ = "Full IMU loop was slow.";
	    }

	    prevtime = starttime;
	    starttime = ros::Time::now().toSec();

	    imu1_data_pub_.publish(reading1);
	    imu2_data_pub_.publish(reading2);
	    if ( numIMU > 2 )
	    	imu3_data_pub_.publish(reading3);

	    endtime = ros::Time::now().toSec();
	    if (endtime - starttime > 0.006)
	    {
	        ROS_WARN("Publishing took %f ms. Nominal is <5 ms.", 1000 * (endtime - starttime));
	        was_slow_ = "Full IMU loop was slow.";
	    }
	        
	    return(0);
	}

	bool spin() {
		while (!ros::isShuttingDown()) // Using ros::isShuttingDown to avoid restarting the node during a shutdown.
	    {
		    if (start() == 0)
		    {
		        while(node_handle_.ok()) {
		          	publish_data();
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

	void getData(sensor_msgs::Imu& data1, sensor_msgs::Imu& data2, sensor_msgs::Imu& data3){
        IMU1->MahonyAHRSupdateIMU();
        IMU2->MahonyAHRSupdateIMU();
        if ( numIMU > 2 )
        	IMU3->MahonyAHRSupdateIMU();

        data1.linear_acceleration.x = IMU1->v_acc[0];
        data1.linear_acceleration.y = IMU1->v_acc[1];
        data1.linear_acceleration.z = IMU1->v_acc[2];
 
        data1.angular_velocity.x = IMU1->v_gyr[0];
        data1.angular_velocity.y = IMU1->v_gyr[1];
        data1.angular_velocity.z = IMU1->v_gyr[2];

        data1.orientation.w = IMU1->v_quat[0];
        data1.orientation.x = IMU1->v_quat[1];
        data1.orientation.y = IMU1->v_quat[2];
        data1.orientation.z = IMU1->v_quat[3];

        data1.header.stamp = ros::Time::now();

        data2.linear_acceleration.x = IMU2->v_acc[0];
        data2.linear_acceleration.y = IMU2->v_acc[1];
        data2.linear_acceleration.z = IMU2->v_acc[2];
 
        data2.angular_velocity.x = IMU2->v_gyr[0];
        data2.angular_velocity.y = IMU2->v_gyr[1];
        data2.angular_velocity.z = IMU2->v_gyr[2];

        data2.orientation.w = IMU2->v_quat[0];
        data2.orientation.x = IMU2->v_quat[1];
        data2.orientation.y = IMU2->v_quat[2];
        data2.orientation.z = IMU2->v_quat[3];

        data2.header.stamp = ros::Time::now();
	
        if ( numIMU > 2 ) {
        	data3.linear_acceleration.x = IMU3->v_acc[0];
	        data3.linear_acceleration.y = IMU3->v_acc[1];
	        data3.linear_acceleration.z = IMU3->v_acc[2];
	 
	        data3.angular_velocity.x = IMU3->v_gyr[0];
	        data3.angular_velocity.y = IMU3->v_gyr[1];
	        data3.angular_velocity.z = IMU3->v_gyr[2];

	        data3.orientation.w = IMU3->v_quat[0];
	        data3.orientation.x = IMU3->v_quat[1];
	        data3.orientation.y = IMU3->v_quat[2];
	        data3.orientation.z = IMU3->v_quat[3];

	        data3.header.stamp = ros::Time::now();
        }
	}

	void calibrateIMU(MPUptr& IMU){
		IMU->read6DOF();
		float tempx = 0.;
		float tempy = 0.;
		float tempz = 0.;
		for ( int i=0; i<500; i++ ) {
			tempx += IMU->v_gyr[0];
			tempy += IMU->v_gyr[1];
			tempz += IMU->v_gyr[2];
		}
		IMU->GYRbias[0] += tempx / 500.0;
		IMU->GYRbias[1] += tempy / 500.0;
		IMU->GYRbias[2] += tempz / 500.0;
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
