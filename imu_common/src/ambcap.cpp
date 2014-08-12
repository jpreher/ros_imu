/*=====================================================================================================
// AMBCAP tools to enable easy configurability of reading from and number of imu devices in ROS.
//=====================================================================================================
//
// ROS class with objects and functionality to easily collect data from IMU's on the BBB.
//
// Date         Author          Notes
// 07/15/2014   Jake Reher      Initial Release
//
//=====================================================================================================*/
//---------------------------------------------------------------------------------------------------

#include "ambcap.h"

/* CONSTRUCTOR ambcap(ros::NodeHandle nh, int freq)
 * Assigns the node handle to the class. Initializes with provided frequency.
 *  First assigns all devices to uncalibrated. Loads in desired capture setting.
 *  Finally initializes all device objects corresponding to the capture setting.
 * @PARAM nh - Node handle of ROS node using AMBCAP.
 * @PARAM freq - User selected frequency, used in ros::rate timer.
 */
ambcap::ambcap(ros::NodeHandle nh, int freq): rate((float)freq) {
    frequency = freq;
    node_handle_ = nh;

    L_foot.iscalibrated = false;
    L_shank.iscalibrated = false;
    L_thigh.iscalibrated = false;
    R_foot.iscalibrated = false;
    R_shank.iscalibrated = false;
    R_thigh.iscalibrated = false;
    Torso.iscalibrated = false;
    Single.iscalibrated = false;

    //Settings options.
    // 0: Full body
    // 1: Full legs
    // 2: Under legs
    // 3: Prosthetic
    // 4: Single
    ros::param::get("/capture_setting", setting);
    setting_select();
}

/* FUNCTION spin()
 * Main while loop of AMBCAP.
 * Checks calibration, if OK it updates and publishes and enabled
 *  devices and then spins ROS.
 */
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

/* FUNCTION readBit()
 * Uses user defined setting to establish appropriate IMU objects
 */
bool ambcap::setting_select() {
    switch(setting) {
    case 0:
        ROS_INFO("Setting up objects for Full Body Capture");
        L_foot.imu_location = ambcap::l_foot;
        L_foot.frequency = frequency;
        L_foot.initialize(node_handle_);

        L_shank.imu_location = ambcap::l_shank;
        L_shank.frequency = frequency;
        L_shank.initialize(node_handle_);

        L_thigh.imu_location = ambcap::l_thigh;
        L_thigh.frequency = frequency;
        L_thigh.initialize(node_handle_);

        R_foot.imu_location = ambcap::r_foot;
        R_foot.frequency = frequency;
        R_foot.initialize(node_handle_);

        R_shank.imu_location = ambcap::r_shank;
        R_shank.frequency = frequency;
        R_shank.initialize(node_handle_);

        R_thigh.imu_location = ambcap::r_thigh;
        R_thigh.frequency = frequency;
        R_thigh.initialize(node_handle_);

        Torso.imu_location = ambcap::torso;
        Torso.frequency = frequency;
        Torso.initialize(node_handle_);

        Single.running = false;
        break;
    case 1:
        ROS_INFO("Setting up objects for Full Legs Capture");
        L_foot.imu_location = ambcap::l_foot;
        L_foot.frequency = frequency;
        L_foot.initialize(node_handle_);

        L_shank.imu_location = ambcap::l_shank;
        L_shank.frequency = frequency;
        L_shank.initialize(node_handle_);

        L_thigh.imu_location = ambcap::l_thigh;
        L_thigh.frequency = frequency;
        L_thigh.initialize(node_handle_);

        R_foot.imu_location = ambcap::r_foot;
        R_foot.frequency = frequency;
        R_foot.initialize(node_handle_);

        R_shank.imu_location = ambcap::r_shank;
        R_shank.frequency = frequency;
        R_shank.initialize(node_handle_);

        R_thigh.imu_location = ambcap::r_thigh;
        R_thigh.frequency = frequency;
        R_thigh.initialize(node_handle_);

        Torso.running = false;
        Single.running = false;
        break;
    case 2:
        ROS_INFO("Setting up objects for Under-Actuated Legs Capture");
        L_shank.imu_location = ambcap::l_shank;
        L_shank.frequency = frequency;
        L_shank.initialize(node_handle_);

        L_thigh.imu_location = ambcap::l_thigh;
        L_thigh.frequency = frequency;
        L_thigh.initialize(node_handle_);

        R_shank.imu_location = ambcap::r_shank;
        R_shank.frequency = frequency;
        R_shank.initialize(node_handle_);

        R_thigh.imu_location = ambcap::r_thigh;
        R_thigh.frequency = frequency;
        R_thigh.initialize(node_handle_);

        L_foot.running = false;
        R_foot.running = false;
        Torso.running = false;
        Single.running = false;
        break;
    case 3:
        ROS_INFO("Setting up objects for Prosthetic IMUs");
        /*
        L_thigh.imu_location = ambcap::l_thigh;
        L_thigh.frequency = frequency;
        L_thigh.initialize(node_handle_);

        R_foot.imu_location = ambcap::r_foot;
        R_foot.frequency = frequency;
        R_foot.initialize(node_handle_);
        */
        L_thigh.running = false;
        R_foot.running = false;

        R_shank.imu_location = ambcap::r_shank;
        R_shank.frequency = frequency;
        R_shank.initialize(node_handle_);

        R_thigh.imu_location = ambcap::r_thigh;
        R_thigh.frequency = frequency;
        R_thigh.initialize(node_handle_);

        L_foot.running = false;
        L_shank.running = false;
        Torso.running = false;
        Single.running = false;
        break;
    case 4:
        ROS_INFO("Setting up objects for single IMU");
        Single.imu_location = ambcap::single;
        Single.frequency = frequency;
        Single.initialize(node_handle_);

        L_foot.running = false;
        L_shank.running = false;
        L_thigh.running = false;
        R_foot.running = false;
        R_shank.running = false;
        R_thigh.running = false;
        Torso.running = false;
        break;
    default:
        ROS_INFO("NO VALID USER SETTINGS FOR CAPTURE TYPE!");
        exit(1);
    }
}

/* FUNCTION initialize(ros::NodeHandle& nh, imu& device)
 * Initializes provided imu object and starts the associated ros services on the node.
 * @PARAM nh - Node handle that the ROS services will be assosciated to.
 * @PARAM device - IMU device which will be initialized.
 */
bool ambcap::imu::initialize(ros::NodeHandle& nh) {
    std::string param_base_path = "/imu/";
    std::string param_device;
    //Address the imu to the proper parameters on the parameter server.
    if ( imu_location == l_foot ) {
        param_device = "l_foot";
        calibrate_serv_ = nh.advertiseService("l_foot_gyr_cal", &ambcap::imu::cal_gyr, this);
        pitch_roll_ref_ = nh.advertiseService("l_foot_PR", &ambcap::imu::pitch_roll_serv, this);
        yaw_ref_        = nh.advertiseService("l_foot_yaw", &ambcap::imu::yaw_serv, this);
    }
    if ( imu_location == l_shank ) {
        param_device = "l_shank";
        calibrate_serv_ = nh.advertiseService("l_shank_gyr_cal", &ambcap::imu::cal_gyr, this);
        pitch_roll_ref_ = nh.advertiseService("l_shank_PR", &ambcap::imu::pitch_roll_serv, this);
        yaw_ref_        = nh.advertiseService("l_shank_yaw", &ambcap::imu::yaw_serv, this);
    }
    if ( imu_location == l_thigh ) {
        param_device = "l_thigh";
        calibrate_serv_ = nh.advertiseService("l_thigh_gyr_cal", &ambcap::imu::cal_gyr, this);
        pitch_roll_ref_ = nh.advertiseService("l_thigh_PR", &ambcap::imu::pitch_roll_serv, this);
        yaw_ref_        = nh.advertiseService("l_thigh_yaw", &ambcap::imu::yaw_serv, this);
    }
    if ( imu_location == r_foot ) {
        param_device = "r_foot";
        calibrate_serv_ = nh.advertiseService("r_foot_gyr_cal", &ambcap::imu::cal_gyr, this);
        pitch_roll_ref_ = nh.advertiseService("r_foot_PR", &ambcap::imu::pitch_roll_serv, this);
        yaw_ref_        = nh.advertiseService("r_foot_yaw", &ambcap::imu::yaw_serv, this);
    }
    if ( imu_location == r_shank ) {
        param_device = "r_shank";
        calibrate_serv_ = nh.advertiseService("r_shank_gyr_cal", &ambcap::imu::cal_gyr, this);
        pitch_roll_ref_ = nh.advertiseService("r_shank_PR", &ambcap::imu::pitch_roll_serv, this);
        yaw_ref_        = nh.advertiseService("r_shakn_yaw", &ambcap::imu::yaw_serv, this);
    }
    if ( imu_location == r_thigh ) {
        param_device = "r_thigh";
        calibrate_serv_ = nh.advertiseService("r_thigh_gyr_cal", &ambcap::imu::cal_gyr, this);
        pitch_roll_ref_ = nh.advertiseService("r_thigh_PR", &ambcap::imu::pitch_roll_serv, this);
        yaw_ref_        = nh.advertiseService("r_thigh_yaw", &ambcap::imu::yaw_serv, this);
    }
    if ( imu_location == torso ) {
        param_device = "torso";
        calibrate_serv_ = nh.advertiseService("torso_gyr_cal", &ambcap::imu::cal_gyr, this);
        pitch_roll_ref_ = nh.advertiseService("torso_PR", &ambcap::imu::pitch_roll_serv, this);
        yaw_ref_        = nh.advertiseService("torso_yaw", &ambcap::imu::yaw_serv, this);
    }
    if ( imu_location == single ) {
        param_device = "single";
        calibrate_serv_ = nh.advertiseService("single_gyr_cal", &ambcap::imu::cal_gyr, this);
        pitch_roll_ref_ = nh.advertiseService("single_PR", &ambcap::imu::pitch_roll_serv, this);
        yaw_ref_        = nh.advertiseService("single_yaw", &ambcap::imu::yaw_serv, this);
    }

    q_sensor_cal[0] = 1.f;
    q_sensor_cal[1] = 0.f;
    q_sensor_cal[2] = 0.f;
    q_sensor_cal[3] = 0.f;

    //Load in the parameters and assign them to the struct.
    param_base_path += param_device;
    ros::param::get(param_base_path + "/bus", bus);
    ros::param::get(param_base_path + "/addr", addr);
    ros::param::get(param_base_path + "/chan", chan);
    ros::param::get(param_base_path + "/bias_path", bias_path);
    ros::param::get(param_base_path + "/isvertical", isvertical);
    ros::param::get(param_base_path + "/freq", frequency);

    //Copy contents of newly established MPU9150 object to the device.
    const char* temppath = bias_path.c_str();
    MPU9150 tempMPU(bus, addr, chan, frequency, isvertical, false);
    MPU = tempMPU;
    MPU.initialize(temppath);
    ROS_INFO("%s initialized on bus %d, chan %d, address %d", param_device.c_str(), bus, chan, addr);
    iscalibrated = false;
    doPR = true;
    doYaw = false;
    time_last_run = ros::Time::now().toSec();

    //Set up the publisher and servicer.
    data_pub_ = nh.advertise<imu_common::imu>(param_device, frequency);
    //device.calibrate_serv_ = nh.advertiseService("calibrate_" + param_device, device.calibrate_callback);

    calibrate_gyro();
    running = true;
    return true;
}

/* FUNCTION update(imu& device)
 * Read values from the imu device hardware.
 * @PARAM device - IMU object which will be populated with hardware readings.
 */
bool ambcap::update(imu& device) {
    double time_now = ros::Time::now().toSec();
    float dt = (time_now - device.time_last_run);

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

    device.data.orientation_REF.w = device.q_sensor_cal[0];
    device.data.orientation_REF.x = device.q_sensor_cal[1];
    device.data.orientation_REF.y = device.q_sensor_cal[2];
    device.data.orientation_REF.z = device.q_sensor_cal[3];

    device.data.header.stamp = ros::Time::now();
    device.time_last_run = time_now;
    return true;
}

/* FUNCTION publish(imu& device)
 * Publish current stored values in IMU object.
 * @PARAM device - IMU object which will publish values to its assigned node.
 */
bool ambcap::publish(imu& device) {
    device.data_pub_.publish(device.data);
}

/* FUNCTION publishRunning()
 * Checks all devices for running status, updates then publishes in that order.
 */
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

/* FUNCTION checkCalibration()
 * Checks all possible devices for running status and calibration status.
 * If running and not calibrated sends object to calibrate_gyro function.
 */
bool ambcap::checkCalibration() {
    if ( L_foot.running )
        L_foot.check_cal();
    if ( L_shank.running )
        L_shank.check_cal();
    if ( L_thigh.running )
        L_thigh.check_cal();
    if ( R_foot.running )
        R_foot.check_cal();
    if ( R_shank.running )
        R_shank.check_cal();
    if ( R_thigh.running )
        R_thigh.check_cal();
    if ( Torso.running )
        Torso.check_cal();
    if ( Single.running )
        Single.check_cal();
}

void ambcap::imu::check_cal() {
    if ( !iscalibrated )
        calibrate_gyro();
    if ( doPR )
        pitch_roll_ref();
    if ( doYaw )
        yaw_ref();
}

/* FUNCTION calibrate_gyro(imu& device)
 * Calibrates the gyroscope on the provided IMU device.
 * @PARAM device - IMU object which will be calibrated.
 */
bool ambcap::imu::calibrate_gyro() {
    float tempx = 0.f;
    float tempy = 0.f;
    float tempz = 0.f;

    //Take 500 readings and average the values.
    for ( int i=0; i<500; i++ ) {
        MPU.read6DOF();
        tempx += MPU.v_gyr[0];
        tempy += MPU.v_gyr[1];
        tempz += MPU.v_gyr[2];
        usleep(100);
    }

    if ( isvertical ) {
        //If the imu is vertical exchange x and z
        MPU.GYRbias[0] += tempz / 500.0f;
        MPU.GYRbias[2] -= tempx / 500.0f;
    } else {
        //Otherwise just average
        MPU.GYRbias[0] += tempx / 500.0f;
        MPU.GYRbias[2] += tempz / 500.0f;
    }
    //Y axis does not change.
    MPU.GYRbias[1] += tempy / 500.0f;

    ROS_INFO("Finished Calibration with offsets %f, %f", tempx/500.f, tempz/500.f);
    iscalibrated = true;
    return true;
}

void ambcap::imu::pitch_roll_ref() {
    float q_ref[4];
    q_ref[0] = 1.f;
    q_ref[1] = 0.f;
    q_ref[2] = 0.f;
    q_ref[3] = 0.f;

    MPU.MahonyAHRSupdateIMU();
    quat::inv(MPU.v_quat, q_sensor_cal);
    quat::prod(q_sensor_cal, q_ref, q_sensor_cal);
    ROS_INFO("Sensor Initialized at %f, %f, %f, %f", q_sensor_cal[0], q_sensor_cal[1], q_sensor_cal[2], q_sensor_cal[3]);
    doPR = false;
}

void ambcap::imu::yaw_ref() {
    float fixed_vel_x[1000], fixed_vel_y[1000], fixed_vel_z[1000];
    float norm_fixed[1000];
    float pin_velocity_y[1000];
    float theta[1000];
    float yaw;
    float numerator, denominator;
    float temp[3], sign, temp_q[4];
    ROS_INFO("Started capturing movement, swing leg foreward and back");

    for (int i=0; i<1000; i++) {
        MPU.MahonyAHRSupdateIMU();
        temp[0] = MPU.v_gyr[0];
        temp[1] = MPU.v_gyr[1];
        temp[2] = MPU.v_gyr[2];
        quat::rotateVec(temp, q_sensor_cal, temp);
        fixed_vel_x[i] = temp[0];
        fixed_vel_y[i] = temp[1];
        fixed_vel_z[i] = temp[2];

        norm_fixed[i] = sqrt(fixed_vel_x[i]*fixed_vel_x[i] + fixed_vel_y[i]*fixed_vel_y[i]);
        sign = copysignf(1.f, temp[1]);
        pin_velocity_y[i] = sign * sqrt(fixed_vel_x[i]*fixed_vel_x[i] + fixed_vel_y[i]*fixed_vel_y[i] + fixed_vel_z[i]*fixed_vel_z[i]);
        temp[0] = fixed_vel_x[i] * pin_velocity_y[i];
        sign = copysignf(1.0, temp[0]);
        theta[i] = sign * acos((fixed_vel_y[i] * pin_velocity_y[i]) / (norm_fixed[i] * fabs(pin_velocity_y[i])));
        usleep(5000);
    }

    numerator = 0.f;
    denominator = 0.f;
    for (int i=0; i<1000; i++) {
        numerator += norm_fixed[i] * theta[i];
        denominator += norm_fixed[i];
    }
    yaw = numerator / denominator;
    ROS_INFO("Yaw angle found as %f", yaw);

    temp[0] = 0.f;
    temp[1] = 0.f;
    temp[2] = yaw;

    quat::euler2quatXYZ(temp, temp_q);
    quat::inv(temp_q, temp_q);
    quat::prod(temp_q, q_sensor_cal, q_sensor_cal);
    ROS_INFO("Sensor Initialized at %f, %f, %f, %f", q_sensor_cal[0], q_sensor_cal[1], q_sensor_cal[2], q_sensor_cal[3]);

    doYaw = false;
}

bool ambcap::imu::cal_gyr(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp) {
    iscalibrated = false;
    return true;
}

bool ambcap::imu::pitch_roll_serv(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp) {
    doPR = true;
    return true;
}

bool ambcap::imu::yaw_serv(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp) {
    doYaw = true;
    return true;
}















