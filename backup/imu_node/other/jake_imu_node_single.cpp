/* IMU_Node for Beaglebone Black and MPU-9150 IMU
 * Written for one IMU. Load yaml for case IMU1 before use.
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

//Sample frequency
float sampleFreq = 200.0f;

typedef std::shared_ptr<MPU9150> MPUptr;

class imuNode {
public:
	// Placeholders for the IMU classes
	MPUptr IMU;

	// Keep track of number of IMU
	int imu_bus;
	int imu_addr;
	bool imu_isvert;
    std::string imu_name;
    std::string imu_biasPath;

	// IMU messages are published with standard IMU sensor_msgs.
	sensor_msgs::Imu reading;

	std::string was_slow_;

  	ros::NodeHandle node_handle_;
  	ros::NodeHandle private_node_handle_;
  	ros::Publisher imu_data_pub_;
  	ros::ServiceServer calibrate_serv_;
  	ros::Publisher is_calibrated_pub_;

  	bool running;
  	float Gb[3];

  	bool autocalibrate_;
  	bool calibrate_requested_;
  	bool calibrated_;

	double desired_freq_;
  
    imuNode(ros::NodeHandle h) : private_node_handle_("~"),
        node_handle_(h), calibrate_requested_(false), desired_freq_(200)
  	{
  		//Constructorey things happen here!
  		ros::NodeHandle imu_node_handle(node_handle_, "imu");

  		// Set up params for calibration servers
    	private_node_handle_.param("autocalibrate", autocalibrate_, true);
    	private_node_handle_.param("assume_calibrated", calibrated_, false);
        //private_node_handle_.param("max_drift_rate", max_drift_rate_, 0.0002);

        // Set up IMU publishers
    	imu_data_pub_ = imu_node_handle.advertise<sensor_msgs::Imu>("data", desired_freq_);

    	// Calibration server for each IMU
        //calibrate_serv_ = imu_node_handle.advertiseService("calibrate", &imuNode::calibrate, this);

    	// Publishers to release info on which IMU's are calibrated.
    	is_calibrated_pub_ = imu_node_handle.advertise<std_msgs::Bool>("is_calibrated", 1, true);

    	// Loop is not to run yet.
    	running = false;

		// Pull values from the ROS parameter server and set up IMUs.
		////////// TODO: change this to the node handle and automate the loading of parameters
            ros::param::get("/imu/name", imu_name);
            ros::param::get("/imu/bus", imu_bus);
			ros::param::get("/imu/addr", imu_addr);
            ros::param::get("/imu/bias_path", imu_biasPath);
			ros::param::get("/imu/is_vertical", imu_isvert);

			// Set up the proper classes for the IMUs
            const char *path = imu_biasPath.c_str();
            IMU.reset(new MPU9150(imu_bus, imu_addr, path, sampleFreq, imu_isvert));
  	}

    ~imuNode()
	{
		stop();
	}

	int start() {
		if ( running == false ) {
	  		// Turn on IMU1
            IMU->initialize();

	  		if (autocalibrate_){
	  			calibrate_requested_ = true;
	  		}

	  		if ( autocalibrate_ || calibrate_requested_ ){
	  			ROS_INFO("Calibrating IMU 1.");
	  			calibrateIMU();
	  			calibrate_requested_ = false;
	  			autocalibrate_ = false;
	  		} else {
                ROS_INFO("Not calibrating IMU 1, use service to calibrate.");
	  		}
	  	}
	  	ROS_INFO("All IMU sensors initialized.");
	  	running = true;
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

	    if (prevtime && prevtime - starttime > 0.01)
	    {
	       	ROS_WARN("Full IMU loop took %f ms. Nominal is 5 ms.", 1000 * (prevtime - starttime));
	        was_slow_ = "Full IMU loop was slow.";
	    }

        getData(reading);
	    double endtime = ros::Time::now().toSec();
	    if (endtime - starttime > 0.01)
	    {
	        ROS_WARN("Gathering data took %f ms. Nominal is <5 ms.", 1000 * (endtime - starttime));
	        was_slow_ = "Full IMU loop was slow.";
	    }

	    prevtime = starttime;
	    starttime = ros::Time::now().toSec();

        imu_data_pub_.publish(reading);

	    endtime = ros::Time::now().toSec();
	    if (endtime - starttime > 0.01)
	    {
	        ROS_WARN("Publishing took %f ms. Nominal is <5 ms.", 1000 * (endtime - starttime));
	        was_slow_ = "Full IMU loop was slow.";
	    }
	        
	    return(0);
	}

	bool spin() {
		double endtime = ros::Time::now().toSec();
		double starttime = ros::Time::now().toSec();
		while (!ros::isShuttingDown()) // Using ros::isShuttingDown to avoid restarting the node during a shutdown.
	    {
	    	if (endtime - starttime >= 0.005) {
		      if (start() == 0)
		      {
		        while(node_handle_.ok()) {
		        	starttime = ros::Time::now().toSec();
		          	publish_data();
		          	ros::spinOnce();
		          	endtime = ros::Time::now().toSec();
		        }
		      } else {
		      	starttime = ros::Time::now().toSec();
		        // No need for diagnostic here since a broadcast occurs in start
		        // when there is an error.
		        usleep(1000000);
		        ros::spinOnce();
		        endtime = ros::Time::now().toSec();
		      }
		  } else {
		  	endtime = ros::Time::now().toSec();
		  }
	    }

    stop();

    return true;
	}

    void getData(sensor_msgs::Imu& data){
        IMU->MahonyAHRSupdateIMU();
        //IMU->read6DOF();

        data.linear_acceleration.x = IMU->v_acc[0];
        data.linear_acceleration.y = IMU->v_acc[1];
        data.linear_acceleration.z = IMU->v_acc[2];
 
        data.angular_velocity.x = IMU->v_gyr[0] - Gb[0];
        data.angular_velocity.y = IMU->v_gyr[1] - Gb[1];
        data.angular_velocity.z = IMU->v_gyr[2] - Gb[2];

        data.orientation.w = IMU->v_quat[0];
        data.orientation.x = IMU->v_quat[1];
        data.orientation.y = IMU->v_quat[2];
        data.orientation.z = IMU->v_quat[3];

        data.header.stamp = ros::Time::now();
	}

	void calibrateIMU(){
		IMU->read6DOF();
		float tempx = 0.;
		float tempy = 0.;
		float tempz = 0.;
		for ( int i=0; i<500; i++ ) {
			tempx += IMU->v_gyr[0];
			tempy += IMU->v_gyr[1];
			tempz += IMU->v_gyr[2];
		}
		Gb[0] = tempx / 500.0;
		Gb[1] = tempy / 500.0;
		Gb[2] = tempz / 500.0;
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
