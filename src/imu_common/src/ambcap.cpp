#include "ambcap.h"

ambcap::ambcap(ros::NodeHandle nh, int freq): rate((float)freq) {
    frequency = freq;
    node_handle_ = nh;

    //Settings options.
    // 0: Full body
    // 1: Full legs
    // 2: Under legs
    // 3: Prosthetic
    // 4: Single
    ros::param::get("/capture_setting", setting);
    setting_select();
}

bool ambcap::spin() {
    while(!ros::isShuttingDown()) {
        while(node_handle_.ok()) {
            checkCalibration();
            publishRunning();
            ros::spinOnce();
            rate.sleep();
        }
    }
}

bool ambcap::setting_select() {
    switch(setting) {
    case 0:
        ROS_INFO("Setting up objects for Full Body Capture");
        L_foot.imu_location = ambcap::l_foot;
        L_foot.frequency = frequency;
        initialize(node_handle_, L_foot);

        L_shank.imu_location = ambcap::l_shank;
        L_shank.frequency = frequency;
        initialize(node_handle_, L_shank);

        L_thigh.imu_location = ambcap::l_thigh;
        L_thigh.frequency = frequency;
        initialize(node_handle_, L_thigh);

        R_foot.imu_location = ambcap::r_foot;
        R_foot.frequency = frequency;
        initialize(node_handle_, R_foot);

        R_shank.imu_location = ambcap::r_shank;
        R_shank.frequency = frequency;
        initialize(node_handle_, R_shank);

        R_thigh.imu_location = ambcap::r_thigh;
        R_thigh.frequency = frequency;
        initialize(node_handle_, R_thigh);

        Torso.imu_location = ambcap::torso;
        Torso.frequency = frequency;
        initialize(node_handle_, Torso);
        break;
    case 1:
        ROS_INFO("Setting up objects for Full Legs Capture");
        L_foot.imu_location = ambcap::l_foot;
        L_foot.frequency = frequency;
        initialize(node_handle_, L_foot);

        L_shank.imu_location = ambcap::l_shank;
        L_shank.frequency = frequency;
        initialize(node_handle_, L_shank);

        L_thigh.imu_location = ambcap::l_thigh;
        L_thigh.frequency = frequency;
        initialize(node_handle_, L_thigh);

        R_foot.imu_location = ambcap::r_foot;
        R_foot.frequency = frequency;
        initialize(node_handle_, R_foot);

        R_shank.imu_location = ambcap::r_shank;
        R_shank.frequency = frequency;
        initialize(node_handle_, R_shank);

        R_thigh.imu_location = ambcap::r_thigh;
        R_thigh.frequency = frequency;
        initialize(node_handle_, R_thigh);
        break;
    case 2:
        ROS_INFO("Setting up objects for Under-Actuated Legs Capture");
        L_shank.imu_location = ambcap::l_shank;
        L_shank.frequency = frequency;
        initialize(node_handle_, L_shank);

        L_thigh.imu_location = ambcap::l_thigh;
        L_thigh.frequency = frequency;
        initialize(node_handle_, L_thigh);

        R_shank.imu_location = ambcap::r_shank;
        R_shank.frequency = frequency;
        initialize(node_handle_, R_shank);

        R_thigh.imu_location = ambcap::r_thigh;
        R_thigh.frequency = frequency;
        initialize(node_handle_, R_thigh);
        break;
    case 3:
        ROS_INFO("Setting up objects for Prosthetic IMUs");
        L_thigh.imu_location = ambcap::l_thigh;
        L_thigh.frequency = frequency;
        initialize(node_handle_, L_thigh);

        R_foot.imu_location = ambcap::r_foot;
        R_foot.frequency = frequency;
        initialize(node_handle_, R_foot);

        R_shank.imu_location = ambcap::r_shank;
        R_shank.frequency = frequency;
        initialize(node_handle_, R_shank);

        R_thigh.imu_location = ambcap::r_thigh;
        R_thigh.frequency = frequency;
        initialize(node_handle_, R_thigh);
        break;
    case 4:
        ROS_INFO("Setting up objects for single IMU");
        Single.imu_location = ambcap::single;
        Single.frequency = frequency;
        initialize(node_handle_, Single);
    default:
        ROS_INFO("NO VALID USER SETTINGS FOR CAPTURE TYPE!");
        exit(1);
    }
}

bool ambcap::initialize(ros::NodeHandle& nh, imu& device) {
    std::string param_base_path = "/imu/";
    std::string param_device;
    //Address the imu to the proper parameters on the parameter server.
    if ( device.imu_location == l_foot )
        param_device = "l_foot";
    if ( device.imu_location == l_shank )
        param_device = "l_shank";
    if ( device.imu_location == l_thigh )
        param_device = "l_thigh";
    if ( device.imu_location == r_foot )
        param_device = "r_foot";
    if ( device.imu_location == r_shank )
        param_device = "r_shank";
    if ( device.imu_location == r_thigh )
        param_device = "r_thigh";
    if ( device.imu_location == torso )
        param_device = "torso";
    if ( device.imu_location == single )
        param_device = "single";

    //Load in the parameters and assign them to the struct.
    param_base_path += param_device;
    ros::param::get(param_base_path + "/bus", device.bus);
    ros::param::get(param_base_path + "/addr", device.addr);
    ros::param::get(param_base_path + "/chan", device.chan);
    ros::param::get(param_base_path + "/bias_path", device.bias_path);
    ros::param::get(param_base_path + "/isvertical", device.isvertical);
    ros::param::get(param_base_path + "/freq", device.frequency);

    //Copy contents of newly established MPU9150 object to the device.
    const char* temppath = device.bias_path.c_str();
    MPU9150 tempMPU(device.bus, device.addr, device.chan, device.frequency, device.isvertical, false);
    device.MPU = tempMPU;
    device.MPU.initialize(temppath);
    device.iscalibrated = false;
    device.time_last_run = ros::Time::now().toSec();

    //Set up the publisher and servicer.
    device.data_pub_ = nh.advertise<imu_common::imu>(param_device, device.frequency);
    //device.calibrate_serv_ = nh.advertiseService("calibrate_" + param_device, device.calibrate_callback);

    calibrate_gyro(device);
    device.running = true;
    return true;
}

bool ambcap::calibrate_gyro(imu& device) {
    float tempx = 0.f;
    float tempy = 0.f;
    float tempz = 0.f;

    //Take 500 readings and average the values.
    for ( int i=0; i<500; i++ ) {
        device.MPU.read6DOF();
        tempx += device.MPU.v_gyr[0];
        tempy += device.MPU.v_gyr[1];
        tempz += device.MPU.v_gyr[2];
    }

    if ( device.isvertical ) {
        //If the imu is vertical exchange x and z
        device.MPU.GYRbias[0] += tempz / 500.0f;
        device.MPU.GYRbias[2] -= tempx / 500.0f;
    } else {
        //Otherwise just average
        device.MPU.GYRbias[0] += tempx / 500.0f;
        device.MPU.GYRbias[2] += tempz / 500.0f;
    }
    //Y axis does not change.
    device.MPU.GYRbias[1] += tempy / 500.0f;

    ROS_INFO("Finished Calibration with offsets %f, %f", tempx/500.f, tempz/500.f);
    device.iscalibrated = true;
    return true;
}

bool ambcap::update(imu& device) {
    double time_now = ros::Time::now().toSec();
    float dt = (float)(time_now - device.time_last_run);

    device.MPU.MahonyAHRSupdateIMU(dt);
    device.data.linear_acceleration.x = device.MPU.v_acc[0];
    device.data.linear_acceleration.y = device.MPU.v_acc[1];
    device.data.linear_acceleration.z = device.MPU.v_acc[2];

    device.data.angular_velocity.x = device.MPU.v_gyr[0];
    device.data.angular_velocity.y = device.MPU.v_gyr[1];
    device.data.angular_velocity.z = device.MPU.v_gyr[2];

    device.data.magnetometer.x = device.MPU.v_mag[0];
    device.data.magnetometer.y = device.MPU.v_mag[1];
    device.data.magnetometer.z = device.MPU.v_mag[2];

    device.data.orientation.w = device.MPU.v_quat[0];
    device.data.orientation.x = device.MPU.v_quat[1];
    device.data.orientation.y = device.MPU.v_quat[2];
    device.data.orientation.z = device.MPU.v_quat[3];

    device.data.integralE.x = device.MPU.integralFBx;
    device.data.integralE.y = device.MPU.integralFBy;
    device.data.integralE.z = device.MPU.integralFBz;

    device.data.header.stamp = ros::Time::now();
    device.time_last_run = time_now;
    return true;
}

bool ambcap::publish(imu& device) {
    device.data_pub_.publish(device.data);
}

bool ambcap::publishRunning() {
    //Update all running components first
    if ( L_foot.running )
        update(L_foot);
    if ( L_shank.running )
        update(L_shank);
    if ( L_thigh.running )
        update(L_thigh);
    if ( R_foot.running )
        update(R_foot);
    if ( R_shank.running )
        update(R_shank);
    if ( R_thigh.running )
        update(R_thigh);
    if ( Torso.running )
        update(Torso);
    if ( Single.running )
        update(Single);

    //Then publish (keeps timing tighter).
    if ( L_foot.running )
        publish(L_foot);
    if ( L_shank.running )
        publish(L_shank);
    if ( L_thigh.running )
        publish(L_thigh);
    if ( R_foot.running )
        publish(R_foot);
    if ( R_shank.running )
        publish(R_shank);
    if ( R_thigh.running )
        publish(R_thigh);
    if ( Torso.running )
        publish(Torso);
    if ( Single.running )
        publish(Single);

    return true;
}

bool ambcap::checkCalibration() {
    if ( !L_foot.iscalibrated && L_foot.running )
        calibrate_gyro(L_foot);
    if ( !L_shank.iscalibrated && L_shank.running )
        calibrate_gyro(L_shank);
    if ( !L_thigh.iscalibrated && L_thigh.running )
        calibrate_gyro(L_thigh);
    if ( !R_foot.iscalibrated && R_foot.running )
        calibrate_gyro(R_foot);
    if ( !R_shank.iscalibrated && R_shank.running )
        calibrate_gyro(R_shank);
    if ( !R_thigh.iscalibrated && R_thigh.running )
        calibrate_gyro(R_thigh);
    if ( !Torso.iscalibrated && Torso.running )
        calibrate_gyro(Torso);
    if ( !Single.iscalibrated && Single.running )
        calibrate_gyro(Single);
}
