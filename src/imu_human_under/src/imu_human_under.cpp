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

#include "sensor_msgs/Imu.h"
#include "std_srvs/Empty.h"

#include "std_msgs/Bool.h"

//Sample frequency
float sampleFreq = 200.0f;

typedef std::shared_ptr<MPU9150> MPUptr;

class imuNode {
public:
	// Placeholders for the IMU classes
	MPUptr IMU1;
	MPUptr IMU2;
	MPUptr IMU3;
	MPUptr IMU4;

	// Keep track of number of IMU
	int numIMU;
	int imu1_bus, imu2_bus, imu3_bus, imu4_bus;
	int imu1_addr, imu2_addr, imu3_addr, imu4_addr;
	bool imu1_isvert, imu2_isvert, imu3_isvert, imu4_isvert;
	std::string imu1_name, imu2_name, imu3_name, imu4_name;

  	ros::Rate rate;

	// IMU messages are published with standard IMU sensor_msgs.
	sensor_msgs::Imu reading1;
	sensor_msgs::Imu reading2;
	sensor_msgs::Imu reading3;
	sensor_msgs::Imu reading4;

	std::string was_slow_;

  	ros::NodeHandle node_handle_;
  	ros::NodeHandle private_node_handle_;
  	ros::Publisher imu1_data_pub_;
  	ros::Publisher imu2_data_pub_;
  	ros::Publisher imu3_data_pub_;
  	ros::Publisher imu4_data_pub_;
  	ros::ServiceServer calibrate_serv1_;
  	ros::ServiceServer calibrate_serv2_;
  	ros::ServiceServer calibrate_serv3_;
  	ros::ServiceServer calibrate_serv4_;
  	ros::Publisher is_calibrated1_pub_;
  	ros::Publisher is_calibrated2_pub_;
  	ros::Publisher is_calibrated3_pub_;
  	ros::Publisher is_calibrated4_pub_;
  	ros::ServiceServer left_yaw_cal_serv_;
  	ros::ServiceServer right_yaw_cal_serv_;

  	bool reset_pose_;
  	bool start_tau_;
  	bool stop_tau_;
  	bool tau_running_;
  	bool left_yaw_calibrate_;
  	bool right_yaw_calibrate_;

  	bool running;

  	bool autocalibrate_;
  	bool calibrate_requested1_;
  	bool calibrate_requested2_;
  	bool calibrate_requested3_;
  	bool calibrate_requested4_;
  	bool calibrated1_;
	bool calibrated2_;
	bool calibrated3_;
	bool calibrated4_;

	float qRs_s[4], qRt_s[4];
	float l_y_quat[4], r_y_quat[4];

	double desired_freq_;
  
  	imuNode(ros::NodeHandle h) : private_node_handle_("~"),
  		node_handle_(h), calibrate_requested1_(false), calibrate_requested2_(false),
  		calibrate_requested3_(false), calibrate_requested4_(false),
  		right_yaw_calibrate_(false), left_yaw_calibrate_(false),
		desired_freq_(sampleFreq),
  		rate(sampleFreq) 
  	{

  		ros::NodeHandle imu_node_handle(node_handle_, "human_under");

  		// Set up params for calibration servers
    	private_node_handle_.param("autocalibrate", autocalibrate_, true);
    	private_node_handle_.param("assume_calibrated1", calibrated1_, false);
    	private_node_handle_.param("assume_calibrated2", calibrated2_, false);
    	private_node_handle_.param("assume_calibrated3", calibrated3_, false);
    	private_node_handle_.param("assume_calibrated4", calibrated4_, false);

    	// Set up IMU publishers
    	imu1_data_pub_ = imu_node_handle.advertise<sensor_msgs::Imu>("data1", desired_freq_);
    	imu2_data_pub_ = imu_node_handle.advertise<sensor_msgs::Imu>("data2", desired_freq_);
    	imu3_data_pub_ = imu_node_handle.advertise<sensor_msgs::Imu>("data3", desired_freq_);
    	imu4_data_pub_ = imu_node_handle.advertise<sensor_msgs::Imu>("data4", desired_freq_);

    	// Calibration server for each IMU
    	calibrate_serv1_ = imu_node_handle.advertiseService("calibrate_gyro1", &imuNode::calibrate1, this);
    	calibrate_serv2_ = imu_node_handle.advertiseService("calibrate_gyro2", &imuNode::calibrate2, this);
    	calibrate_serv3_ = imu_node_handle.advertiseService("calibrate_gyro3", &imuNode::calibrate3, this);
    	calibrate_serv4_ = imu_node_handle.advertiseService("calibrate_gyro4", &imuNode::calibrate4, this);

    	left_yaw_cal_serv_ = imu_node_handle.advertiseService("calibrate_left_yaw", &imuNode::left_yaw, this);
    	right_yaw_cal_serv_ = imu_node_handle.advertiseService("calibrate_right_yaw", &imuNode::right_yaw, this);

    	// Publishers to release info on which IMU's are calibrated.
    	is_calibrated1_pub_ = imu_node_handle.advertise<std_msgs::Bool>("is_calibrated1", 1, false);
    	is_calibrated2_pub_ = imu_node_handle.advertise<std_msgs::Bool>("is_calibrated2", 1, false);
    	is_calibrated3_pub_ = imu_node_handle.advertise<std_msgs::Bool>("is_calibrated3", 1, false);
    	is_calibrated4_pub_ = imu_node_handle.advertise<std_msgs::Bool>("is_calibrated4", 1, false);

    	// Loop is not to run yet.
    	running = false;

    	//Load in values from the parameter server.
		std::string bias_path1;
		std::string bias_path2;
		std::string bias_path3;
		std::string bias_path4;
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

		ros::param::get("/imu/left_leg/num_imu", numIMU);

		const char* imu1_biasPath = bias_path1.c_str();
		const char* imu2_biasPath = bias_path2.c_str();

		// Set up the proper classes for the IMUs
		ROS_INFO("Loading left IMU1 bias and setting up class.");
		IMU1.reset(new MPU9150(imu1_bus, imu1_addr, imu1_biasPath, sampleFreq, imu1_isvert));
		ROS_INFO("Loading left IMU2 bias and setting up class.");
		IMU2.reset(new MPU9150(imu2_bus, imu2_addr, imu2_biasPath, sampleFreq, imu2_isvert));

		// IMU 1
		ros::param::get("/imu/right_leg/imu1/name", imu3_name);
		ros::param::get("/imu/right_leg/imu1/bus", imu3_bus);
		ros::param::get("/imu/right_leg/imu1/addr", imu3_addr);
		ros::param::get("/imu/right_leg/imu1/bias_path", bias_path3);
		ros::param::get("/imu/right_leg/imu1/isvertical", imu3_isvert);
		// IMU 2
		ros::param::get("/imu/right_leg/imu2/name", imu4_name);
		ros::param::get("/imu/right_leg/imu2/bus", imu4_bus);
		ros::param::get("/imu/right_leg/imu2/addr", imu4_addr);
		ros::param::get("/imu/right_leg/imu2/bias_path", bias_path4);
		ros::param::get("/imu/right_leg/imu2/isvertical", imu4_isvert);

		const char* imu3_biasPath = bias_path3.c_str();
		const char* imu4_biasPath = bias_path4.c_str();

		if (imu3_isvert)
			ROS_INFO("IMU3 is vertical");
		if (imu4_isvert)
			ROS_INFO("IMU4 is vertical");

		ROS_INFO("Loading right IMU3 bias and setting up class.");
		IMU3.reset(new MPU9150(imu3_bus, imu3_addr, imu3_biasPath, sampleFreq, imu3_isvert));
		ROS_INFO("Loading right IMU4 bias and setting up class.");
		IMU4.reset(new MPU9150(imu4_bus, imu4_addr, imu4_biasPath, sampleFreq, imu4_isvert));

		// Initialize the yaw offset as zero, user must calibrate to get later
		l_y_quat[0] = 1.f;
		l_y_quat[1] = l_y_quat[2] = l_y_quat[3] = 0.f;
		r_y_quat[0] = 1.f;
		r_y_quat[1] = r_y_quat[2] = r_y_quat[3] = 0.f;
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
	  		ROS_INFO("Initializing IMU2");
            IMU2->initialize();
            ROS_INFO("Initializing IMU3");
            IMU3->initialize();
            ROS_INFO("Initializing IMU4");
            IMU4->initialize();

	  		if (autocalibrate_){
	  			calibrate_requested1_ = true;
	  			calibrate_requested2_ = true;
	  			calibrate_requested3_ = true;
	  			calibrate_requested4_ = true;
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
	  		if ( autocalibrate_ || calibrate_requested2_ ){
	  			ROS_INFO("Calibrating IMU 2.");
	  			calibrateIMU(IMU2);
	  			calibrate_msg.data = true;
	  			is_calibrated2_pub_.publish(calibrate_msg);
	  			calibrate_requested2_ = false;
	  			autocalibrate_ = false;
	  		} else {
	  			ROS_INFO("Not calibrating IMU 2, use service to calibrate.");
	  		}
	  		if ( autocalibrate_ || calibrate_requested3_ ){
	  			ROS_INFO("Calibrating IMU 3.");
	  			calibrateIMU(IMU3);
	  			calibrate_msg.data = true;
	  			is_calibrated3_pub_.publish(calibrate_msg);
	  			calibrate_requested3_ = false;
	  			autocalibrate_ = false;
	  		} else {
	  			ROS_INFO("Not calibrating IMU 3, use service to calibrate.");
	  		}
	  		if ( autocalibrate_ || calibrate_requested4_ ){
	  			ROS_INFO("Calibrating IMU 4.");
	  			calibrateIMU(IMU4);
	  			calibrate_msg.data = true;
	  			is_calibrated4_pub_.publish(calibrate_msg);
	  			calibrate_requested4_ = false;
	  			autocalibrate_ = false;
	  		} else {
	  			ROS_INFO("Not calibrating IMU 4, use service to calibrate.");
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

	    if (prevtime && prevtime - starttime > 0.01)
	    {
	       	ROS_WARN("Full IMU loop took %f ms. Nominal is 5 ms.", 1000 * (prevtime - starttime));
	        was_slow_ = "Full IMU loop was slow.";
	    }

	    getData(reading1, reading2, reading3, reading4);
	    double endtime = ros::Time::now().toSec();
	    if (endtime - starttime > 0.01)
	    {
	        ROS_WARN("Gathering data took %f ms. Nominal is <5 ms.", 1000 * (endtime - starttime));
	        was_slow_ = "Full IMU loop was slow.";
	    }

	    prevtime = starttime;
	    starttime = ros::Time::now().toSec();

	    imu1_data_pub_.publish(reading1);
	    imu2_data_pub_.publish(reading2);
	    imu3_data_pub_.publish(reading3);
	    imu4_data_pub_.publish(reading4);

	    endtime = ros::Time::now().toSec();
	    if (endtime - starttime > 0.01)
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

	void getData(sensor_msgs::Imu& data1, sensor_msgs::Imu& data2, sensor_msgs::Imu& data3, sensor_msgs::Imu& data4){
        IMU1->MahonyAHRSupdateIMU();
        IMU2->MahonyAHRSupdateIMU();

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

        data1.header.stamp = ros::Time::now();
        data2.header.stamp = data1.header.stamp;

        IMU3->MahonyAHRSupdateIMU();
        IMU4->MahonyAHRSupdateIMU();

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

        data4.linear_acceleration.x = IMU4->v_acc[0];
        data4.linear_acceleration.y = IMU4->v_acc[1];
        data4.linear_acceleration.z = IMU4->v_acc[2];
 
        data4.angular_velocity.x = IMU4->v_gyr[0];
        data4.angular_velocity.y = IMU4->v_gyr[1];
        data4.angular_velocity.z = IMU4->v_gyr[2];

        data4.orientation.w = IMU4->v_quat[0];
        data4.orientation.x = IMU4->v_quat[1];
        data4.orientation.y = IMU4->v_quat[2];
        data4.orientation.z = IMU4->v_quat[3];

        data3.header.stamp = ros::Time::now();
        data4.header.stamp = data3.header.stamp;

	}

	bool calibrate1(std_srvs::Empty::Request  &req,
   		        	std_srvs::Empty::Response &resp) {
		std_msgs::Bool calibrate_msg;
		calibrate_msg.data = true;
		calibrate_requested1_ = true;
		is_calibrated1_pub_.publish(calibrate_msg);
		return true;
	}

	bool calibrate2(std_srvs::Empty::Request  &req,
   		        	std_srvs::Empty::Response &resp) {
		std_msgs::Bool calibrate_msg;
		calibrate_msg.data = true;
		calibrate_requested2_ = true;
		is_calibrated2_pub_.publish(calibrate_msg);
		return true;
	}

	bool calibrate3(std_srvs::Empty::Request  &req,
   		        	std_srvs::Empty::Response &resp) {
		std_msgs::Bool calibrate_msg;
		calibrate_msg.data = true;
		calibrate_requested3_ = true;
		is_calibrated3_pub_.publish(calibrate_msg);
		return true;
	}

	bool calibrate4(std_srvs::Empty::Request  &req,
   		        	std_srvs::Empty::Response &resp) {
		std_msgs::Bool calibrate_msg;
		calibrate_msg.data = true;
		calibrate_requested4_ = true;
		is_calibrated4_pub_.publish(calibrate_msg);
		return true;
	}	

	bool right_yaw(std_srvs::Empty::Request  &req,
   		           std_srvs::Empty::Response &resp) {
		right_yaw_calibrate_ = true;
		return true;

	}

	bool left_yaw(std_srvs::Empty::Request  &req,
   		          std_srvs::Empty::Response &resp) {
		left_yaw_calibrate_ = true;
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
		if ( calibrate_requested2_ ) {
			calibrateIMU(IMU2);
			calibrate_msg.data = true;
			is_calibrated2_pub_.publish(calibrate_msg);
			calibrate_requested2_ = false;
		}
		if ( calibrate_requested3_ ) {
			calibrateIMU(IMU3);
			calibrate_msg.data = true;
			is_calibrated3_pub_.publish(calibrate_msg);
			calibrate_requested3_ = false;
		}
		if ( calibrate_requested4_ ) {
			calibrateIMU(IMU4);
			calibrate_msg.data = true;
			is_calibrated4_pub_.publish(calibrate_msg);
			calibrate_requested4_ = false;
		}
		if ( right_yaw_calibrate_ ) {
      		ROS_INFO("Got command to start right leg yaw calibration.");
      		calibrateYaw();
    	}
    	if ( left_yaw_calibrate_ ) {
      		ROS_INFO("Got command to start left leg yaw calibration.");
      		calibrateYaw();
    	}
	}

  	void calibrateYaw() {
    	float gx[5000], gy[5000], gX[5000], gY[5000];
    	float tempg;
    	float tempnorm;
    	float tempdot;
    	float theta_t[5000];
    	int N = 0;
    	float sign;
    	float den, num, eul[3];
    	float q_ref[4], qss_e_inv, qts_e_inv;

    	q_ref[0] = 1.f;
    	q_ref[1] = q_ref[2] = q_ref[3] = 0.f;

    	if ( right_yaw_calibrate_ ) {
    		initPose(IMU3, IMU4);

       		// Invert the measured reference pose measurement.
    		//quat::inv(IMU3->v_quat, qss_e_inv);
    		//quat::inv(IMU4->v_quat, qts_e_inv);

   			ROS_INFO("Start right leg movement");
      		for ( int i=0; i<500; i++ ){
      			// Read the IMU
      			IMU3->read6DOF();
      			IMU4->read6DOF();
      			gx[i] = IMU3->v_gyr[0];
      			gy[i] = IMU3->v_gyr[1];
     			gX[i] = IMU4->v_gyr[0];
     			gY[i] = IMU4->v_gyr[1];

     			// Perform the calculation
        		sign = copysignf(1.f, gx[i]*gY[i] - gy[i]*gX[i]);
        		tempdot = gx[i]*gX[i] + gy[i]*gY[i];
        		tempnorm = (gx[i]*gx[i] + gy[i]*gy[i]) * (gX[i]*gX[i] + gY[i]*gY[i]);

        		if (tempdot < 0.001) {
        			theta_t[i] = sign * 0.f;
        		} else if (tempnorm < 0.001) {
        			theta_t[i] = sign * 0.f;
        		} else {
        			theta_t[i] = sign * acos(tempdot / tempnorm);
        		}

        		std::cout << theta_t[i] << ", " << i << ", ";

        		// Tick up the indexer
        		N += 1;

        		// Wait
        		usleep(5000);
      		}
      		num = 0.f;
      		den = 0.f;

      		// Calculate the yaw angle
      		for (int i=0; i<N; i++) {
      			num += sqrt(gx[i]*gx[i] + gy[i]*gy[i]) * theta_t[i];
      			den += sqrt(gx[i]*gx[i] + gy[i]*gy[i]);
      		}

      		eul[0] = 0.f;
      		eul[1] = 0.f;
      		eul[2] = num/den;

      		quat::euler2quatXYZ(eul, l_y_quat);

      		ROS_INFO("Finished right yaw calibration!");
      		right_yaw_calibrate_ = false;
    	}
    	if (left_yaw_calibrate_ ) {
    		initPose(IMU1, IMU2);

      		ROS_INFO("Start left leg movement");
      		for ( int i=0; i<500; i++ ){
      			// Read the IMU
      			IMU1->MahonyAHRSupdateIMU();
      			IMU2->MahonyAHRSupdateIMU();
      			gx[i] = IMU1->v_gyr[0];
      			gy[i] = IMU1->v_gyr[1];
     			gX[i] = IMU2->v_gyr[0];
     			gY[i] = IMU2->v_gyr[1];

     			// Perform the calculation
        		sign = copysignf(1.f, gx[i]*gY[i] - gy[i]*gX[i]);
        		tempdot = gx[i]*gX[i] + gy[i]*gY[i];
        		tempnorm = (gx[i]*gx[i] + gy[i]*gy[i]) * (gX[i]*gX[i] + gY[i]*gY[i]);
        		theta_t[i] = sign * acos(tempdot / tempnorm);

        		// Tick up the indexer
        		N += 1;

        		// Wait
        		usleep(5000);
      		}

      		ROS_INFO("Finished left yaw calibration!");
      		left_yaw_calibrate_ = false;
    	}
  	}

  	void initPose(MPUptr &imu1, MPUptr &imu2) {
    	float qRss_e_inv[4], qRts_e_inv[4];
    	float qRs_e_ref[4], qRt_e_ref[4];
    	float qR_s_meas[4], qR_t_meas[4];
    	imu1->MahonyAHRSupdateIMU();
    	imu2->MahonyAHRSupdateIMU();

    	qRs_e_ref[0] = qRt_e_ref[0] = 1.f;
    	qRs_e_ref[1] = qRt_e_ref[1] = 0.f;
    	qRs_e_ref[2] = qRt_e_ref[2] = 0.f;
    	qRs_e_ref[3] = qRt_e_ref[3] = 0.f;

    	qR_s_meas[0] = imu1->v_quat[0];
    	qR_s_meas[1] = imu1->v_quat[1];
    	qR_s_meas[2] = imu1->v_quat[2];
    	qR_s_meas[3] = imu1->v_quat[3];

    	qR_t_meas[0] = imu2->v_quat[0];
    	qR_t_meas[1] = imu2->v_quat[1];
    	qR_t_meas[2] = imu2->v_quat[2];
    	qR_t_meas[3] = imu2->v_quat[3];

    	ROS_INFO("Setting up initial pose.");
    	// Invert the measured reference pose measurement.
    	quat::inv(qR_s_meas, qRss_e_inv);
    	quat::inv(qR_t_meas, qRts_e_inv);

    	// Create composite quaternion for reference pose.
    	quat::prod(qRss_e_inv, qRs_e_ref, qRs_s);
    	quat::prod(qRts_e_inv, qRt_e_ref, qRt_s);
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