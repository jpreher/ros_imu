/*=====================================================================================================
// AMBCAP tools to enable easy configurability of reading from and number of imu devices in ROS.
//=====================================================================================================
//
// ROS class with objects and functionality to easily collect data from IMU's on the BBB.
//
// Date         Author          Notes
// 09/12/2014   Jake Reher      Initial Release
//
//=====================================================================================================*/
//---------------------------------------------------------------------------------------------------

#include "ambcap_EKF.h"

/* CONSTRUCTOR ambcap_EKF(ros::NodeHandle nh, int freq)
 * Assigns the node handle to the class. Initializes with provided frequency.
 *  First assigns all devices to uncalibrated. Loads in desired capture setting.
 *  Finally initializes all device objects corresponding to the capture setting.
 * @PARAM nh - Node handle of ROS node using AMBCAP.
 * @PARAM freq - User selected frequency, used in ros::rate timer.
 */
ambcap_EKF::ambcap_EKF(ros::NodeHandle nh, int freq, bool ampublish): rate((float)freq) {
    // TELL THE USER WHICH FILTER IS IN USE
    ROS_WARN("USING EKF FILTER - FOR ONLINE USE WITH PROSTHETIC");

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

    // Body lengths
    // Jake

    Len_shank << 0., 0., -0.4318;
    Len_thigh << 0., 0., -0.4318;
    Rad_foot << 0.1016, 0., -0.0254;
    Rad_shank << 0.0508, 0., -0.3556;
    Rad_thigh << 0.0762, 0., -0.3048;
    Rad_torso << 0.1270, 0., 0.3556;
    Rad_single << 0., 0., 0.;

    // Jon
    /*
    Len_shank << 0., 0., -0.413;
    Len_thigh << 0., 0., -0.419;
    Rad_foot << 0.1016, 0., -0.0254; // Dont use
    Rad_shank << 0.0635, 0., -0.3302; //
    Rad_thigh << 0.08255, 0., -0.381; //
    Rad_torso << 0.1270, 0., 0.3556; // Dont use
    Rad_single << 0., 0., 0.;
    */

    //Settings options.
    // 0: Full body
    // 1: Full legs
    // 2: Under legs
    // 3: Prosthetic
    // 4: Single
    ros::param::get("/capture_setting", setting);
    setting_select();

    publishall = ampublish;
}

/* FUNCTION spin()
 * Main while loop of AMBCAP.
 * Checks calibration, if OK it updates and publishes and enabled
 *  devices and then spins ROS.
 */
bool ambcap_EKF::spin() {
    while(!ros::isShuttingDown()) {
        while(node_handle_.ok()) {
            checkCalibration();
            publishRunning();
            ros::spinOnce();
            rate.sleep();
        }
    }
}

/* FUNCTION updateEKF()
 * Main while loop of AMBCAP.
 * Checks calibration, if OK it updates and publishes and enabled
 *  devices and then spins ROS.
 */
bool ambcap_EKF::updateEKF() {
    if(!ros::isShuttingDown()) {
        if(node_handle_.ok()) {
            checkCalibration();
            publishRunning();
        }
    }
}

/* FUNCTION readBit()
 * Uses user defined setting to establish appropriate IMU objects
 */
bool ambcap_EKF::setting_select() {
    switch(setting) {
    case 0:
        ROS_INFO("Setting up objects for Full Body Capture");
        L_foot.imu_location = ambcap_EKF::l_foot;
        L_foot.frequency = frequency;
        L_foot.initialize(node_handle_, Rad_foot);

        L_shank.imu_location = ambcap_EKF::l_shank;
        L_shank.frequency = frequency;
        L_shank.initialize(node_handle_, Rad_shank);

        L_thigh.imu_location = ambcap_EKF::l_thigh;
        L_thigh.frequency = frequency;
        L_thigh.initialize(node_handle_, Rad_thigh);

        R_foot.imu_location = ambcap_EKF::r_foot;
        R_foot.frequency = frequency;
        R_foot.initialize(node_handle_, Rad_foot);

        R_shank.imu_location = ambcap_EKF::r_shank;
        R_shank.frequency = frequency;
        R_shank.initialize(node_handle_, Rad_shank);

        R_thigh.imu_location = ambcap_EKF::r_thigh;
        R_thigh.frequency = frequency;
        R_thigh.initialize(node_handle_, Rad_thigh);

        Torso.imu_location = ambcap_EKF::torso;
        Torso.frequency = frequency;
        Torso.initialize(node_handle_, Rad_torso);

        Single.running = false;
        break;
    case 1:
        ROS_INFO("Setting up objects for Full Legs Capture");
        L_foot.imu_location = ambcap_EKF::l_foot;
        L_foot.frequency = frequency;
        L_foot.initialize(node_handle_, Rad_foot);

        L_shank.imu_location = ambcap_EKF::l_shank;
        L_shank.frequency = frequency;
        L_shank.initialize(node_handle_, Rad_shank);

        L_thigh.imu_location = ambcap_EKF::l_thigh;
        L_thigh.frequency = frequency;
        L_thigh.initialize(node_handle_, Rad_thigh);

        R_foot.imu_location = ambcap_EKF::r_foot;
        R_foot.frequency = frequency;
        R_foot.initialize(node_handle_, Rad_foot);

        R_shank.imu_location = ambcap_EKF::r_shank;
        R_shank.frequency = frequency;
        R_shank.initialize(node_handle_, Rad_shank);

        R_thigh.imu_location = ambcap_EKF::r_thigh;
        R_thigh.frequency = frequency;
        R_thigh.initialize(node_handle_, Rad_thigh);

        Torso.running = false;
        Single.running = false;
        break;
    case 2:
        ROS_INFO("Setting up objects for Under-Actuated Legs Capture");
        L_shank.imu_location = ambcap_EKF::l_shank;
        L_shank.frequency = frequency;
        L_shank.initialize(node_handle_, Rad_shank);

        L_thigh.imu_location = ambcap_EKF::l_thigh;
        L_thigh.frequency = frequency;
        L_thigh.initialize(node_handle_, Rad_thigh);

        R_shank.imu_location = ambcap_EKF::r_shank;
        R_shank.frequency = frequency;
        R_shank.initialize(node_handle_, Rad_shank);

        R_thigh.imu_location = ambcap_EKF::r_thigh;
        R_thigh.frequency = frequency;
        R_thigh.initialize(node_handle_, Rad_thigh);

        L_foot.running = false;
        R_foot.running = false;
        Torso.running = false;
        Single.running = false;
        break;
    case 3:
        ROS_INFO("Setting up objects for Prosthetic IMUs");
        /*
        L_thigh.imu_location = ambcap_EKF::l_thigh;
        L_thigh.frequency = frequency;
        L_thigh.initialize(node_handle_);

        R_foot.imu_location = ambcap_EKF::r_foot;
        R_foot.frequency = frequency;
        R_foot.initialize(node_handle_);
        */
        L_thigh.running = false;
        R_foot.running = false;

        R_shank.imu_location = ambcap_EKF::r_shank;
        R_shank.frequency = frequency;
        R_shank.initialize(node_handle_, Rad_shank);

        R_thigh.imu_location = ambcap_EKF::r_thigh;
        R_thigh.frequency = frequency;
        R_thigh.initialize(node_handle_, Rad_thigh);

        L_foot.running = false;
        L_shank.running = false;
        Torso.running = false;
        Single.running = false;
        break;
    case 4:
        ROS_INFO("Setting up objects for single IMU");
        Single.imu_location = ambcap_EKF::single;
        Single.frequency = frequency;
        Single.initialize(node_handle_, Rad_single);

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
bool ambcap_EKF::imu::initialize(ros::NodeHandle& nh, Vector3d &rad) {
    // Initialize all the things
    velocity.resize(3);
    Dvelocity.resize(3);
    acc.resize(3);
    a0.resize(3);
    r_init.resize(3);

    std::string param_base_path = "/imu/";
    std::string param_device;
    r_init << rad(0), rad(1), rad(2);

    //Address the imu to the proper parameters on the parameter server.
    if ( imu_location == l_foot ) {
        param_device = "l_foot";
        calibrate_serv_ = nh.advertiseService("l_foot_gyr_cal", &ambcap_EKF::imu::cal_gyr, this);
        pitch_roll_ref_ = nh.advertiseService("l_foot_PR", &ambcap_EKF::imu::pitch_roll_serv, this);
        yaw_ref_        = nh.advertiseService("l_foot_yaw", &ambcap_EKF::imu::yaw_serv, this);
    }
    if ( imu_location == l_shank ) {
        param_device = "l_shank";
        calibrate_serv_ = nh.advertiseService("l_shank_gyr_cal", &ambcap_EKF::imu::cal_gyr, this);
        pitch_roll_ref_ = nh.advertiseService("l_shank_PR", &ambcap_EKF::imu::pitch_roll_serv, this);
        yaw_ref_        = nh.advertiseService("l_shank_yaw", &ambcap_EKF::imu::yaw_serv, this);
    }
    if ( imu_location == l_thigh ) {
        param_device = "l_thigh";
        calibrate_serv_ = nh.advertiseService("l_thigh_gyr_cal", &ambcap_EKF::imu::cal_gyr, this);
        pitch_roll_ref_ = nh.advertiseService("l_thigh_PR", &ambcap_EKF::imu::pitch_roll_serv, this);
        yaw_ref_        = nh.advertiseService("l_thigh_yaw", &ambcap_EKF::imu::yaw_serv, this);
    }
    if ( imu_location == r_foot ) {
        param_device = "r_foot";
        calibrate_serv_ = nh.advertiseService("r_foot_gyr_cal", &ambcap_EKF::imu::cal_gyr, this);
        pitch_roll_ref_ = nh.advertiseService("r_foot_PR", &ambcap_EKF::imu::pitch_roll_serv, this);
        yaw_ref_        = nh.advertiseService("r_foot_yaw", &ambcap_EKF::imu::yaw_serv, this);
    }
    if ( imu_location == r_shank ) {
        param_device = "r_shank";
        calibrate_serv_ = nh.advertiseService("r_shank_gyr_cal", &ambcap_EKF::imu::cal_gyr, this);
        pitch_roll_ref_ = nh.advertiseService("r_shank_PR", &ambcap_EKF::imu::pitch_roll_serv, this);
        yaw_ref_        = nh.advertiseService("r_shank_yaw", &ambcap_EKF::imu::yaw_serv, this);
    }
    if ( imu_location == r_thigh ) {
        param_device = "r_thigh";
        calibrate_serv_ = nh.advertiseService("r_thigh_gyr_cal", &ambcap_EKF::imu::cal_gyr, this);
        pitch_roll_ref_ = nh.advertiseService("r_thigh_PR", &ambcap_EKF::imu::pitch_roll_serv, this);
        yaw_ref_        = nh.advertiseService("r_thigh_yaw", &ambcap_EKF::imu::yaw_serv, this);
    }
    if ( imu_location == torso ) {
        param_device = "torso";
        calibrate_serv_ = nh.advertiseService("torso_gyr_cal", &ambcap_EKF::imu::cal_gyr, this);
        pitch_roll_ref_ = nh.advertiseService("torso_PR", &ambcap_EKF::imu::pitch_roll_serv, this);
        yaw_ref_        = nh.advertiseService("torso_yaw", &ambcap_EKF::imu::yaw_serv, this);
    }
    if ( imu_location == single ) {
        param_device = "single";
        calibrate_serv_ = nh.advertiseService("single_gyr_cal", &ambcap_EKF::imu::cal_gyr, this);
        pitch_roll_ref_ = nh.advertiseService("single_PR", &ambcap_EKF::imu::pitch_roll_serv, this);
        yaw_ref_        = nh.advertiseService("single_yaw", &ambcap_EKF::imu::yaw_serv, this);
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

    // Setup ekf
    x_init.resize(14);
    x_init << 0.,
              0.0001,
              0.,
              0.,
              0.00001,
              0.,
              1.,
              0.,
              0.,
              0.,
              0.,
              0.,
              0.,
              0.;

    // Process Variance
    processVar.resize(14);
    processVar << 0.01, 0.01, 0.01,
                  10., 10., 10.,
                  0.00001, 0.00001, 0.00001, 0.00001,
                  0.001, 0.001, 0.001, 0.001;

    // Measurement Variance
    measureVar.resize(9);
    measureVar << 6., 6., 6.,
                  3000., 3000., 3000.,
                  900., 900., 900.;

    // Matrices
    P_init.resize(14,14); Q_init.resize(14,14); R_init.resize(9,9);
    P_init = MatrixXd::Identity(14,14);
    Q_init = MatrixXd(processVar.asDiagonal());
    R_init = MatrixXd(measureVar.asDiagonal());

    Filter.initialize(x_init, P_init, Q_init, R_init, r_init);

    velocity << 0., 0., 0.;
    Dvelocity << 0., 0., 0.;

    // Set up calibration steps
    iscalibrated = false;
    doPR = true;
    doYaw = false;
    time_last_run = ros::Time::now().toSec();

    //Set up the publisher and servicer.
    data_pub_ = nh.advertise<imu_common::imu>(param_device, frequency);
    //device.calibrate_serv_ = nh.advertiseService("calibrate_" + param_device, device.calibrate_callback);

    calibrate_gyro();
    running = true;
    usleep(50000);
    return true;
}

/* FUNCTION update(imu& device)
 * Read values from the imu device hardware.
 * @PARAM device - IMU object which will be populated with hardware readings.
 */
bool ambcap_EKF::update(imu& device) {
    double time_now = ros::Time::now().toSec();
    double dt = (time_now - device.time_last_run);
    VectorXd measurement;

    device.MPU.read6DOF();
    device.dt = 0.005;

    //device.data.linear_acceleration.x = device.MPU.v_acc[0];
    //device.data.linear_acceleration.y = device.MPU.v_acc[1];
    //device.data.linear_acceleration.z = device.MPU.v_acc[2];

    device.data.magnetometer.x = device.MPU.v_mag[0];
    device.data.magnetometer.y = device.MPU.v_mag[1];
    device.data.magnetometer.z = device.MPU.v_mag[2];

    //device.data.orientation.w = device.MPU.v_quat[0];
    //device.data.orientation.x = device.MPU.v_quat[1];
    //device.data.orientation.y = device.MPU.v_quat[2];
    //device.data.orientation.z = device.MPU.v_quat[3];

    device.data.orientation_REF.w = device.q_sensor_cal[0];
    device.data.orientation_REF.x = device.q_sensor_cal[1];
    device.data.orientation_REF.y = device.q_sensor_cal[2];
    device.data.orientation_REF.z = device.q_sensor_cal[3];

    device.data.header.stamp = ros::Time::now();
    device.time_last_run = time_now;

    return true;
}


void ambcap_EKF::filter(imu& device) {
    // Rotate the measurements
    float tempa[3];
    quat::rotateVec(device.MPU.v_acc, device.q_sensor_cal, tempa);
    device.acc << tempa[0],
                  0.,
                  tempa[2];

    quat::rotateVec(device.MPU.v_gyr, device.q_sensor_cal, tempa);

    device.Dvelocity = Vector3d( 0., (-tempa[1] - device.velocity(1)) / 0.005, 0.);
    device.velocity = Vector3d( 0., -tempa[1], 0.);

    // Get previous joint acceleration.
    device.a0.resize(3);
    if ( device.imu_location == l_foot ) {
        //TODO
        device.a0 << 0., 0., 0.;
    }
    if ( device.imu_location == l_shank ) {
        Vector3d tempThighW, tempThighWdot;
        float temp[3], tempq[4];
        tempThighW << -Len_thigh(0) * L_thigh.velocity(1) * L_thigh.velocity(1), 0., -Len_thigh(2) * L_thigh.velocity(1) * L_thigh.velocity(1);
        tempThighWdot << Len_thigh(2) * -L_thigh.Dvelocity(1), 0., -Len_thigh(0) * -L_thigh.Dvelocity(1);

        // Bad jake, bad, need to update utilities to use Eigen..
        temp[0] = tempThighW(0) + tempThighWdot(0);
        temp[1] = tempThighW(1) + tempThighWdot(1);
        temp[2] = tempThighW(2) + tempThighWdot(2);
        tempq[0] = device.Filter.x_hat(6);
        tempq[1] = device.Filter.x_hat(7);
        tempq[2] = device.Filter.x_hat(8);
        tempq[3] = device.Filter.x_hat(9);

        quat::rotateVec(temp, tempq, temp);
        device.a0 = Vector3d(temp[0], temp[1], temp[2]);
    }
    if ( device.imu_location == l_thigh ) {
        device.a0 << 0., 0., 0.;
    }
    if ( device.imu_location == r_foot ) {
        //TODO
    }
    if ( device.imu_location == r_shank ) {
        Vector3d tempThighW, tempThighWdot;
        float temp[3], tempq[4];


        tempThighW << -Len_thigh(0) * R_thigh.velocity(1) * R_thigh.velocity(1),
                      0.,
                      -Len_thigh(2) * R_thigh.velocity(1) * R_thigh.velocity(1);

        tempThighWdot << Len_thigh(2) * R_thigh.Dvelocity(1),
                         0.,
                         -Len_thigh(0) * R_thigh.Dvelocity(1);

        // Add the two components
        temp[0] = tempThighW(0) + tempThighWdot(0);
        temp[1] = tempThighW(1) + tempThighWdot(1);
        temp[2] = tempThighW(2) + tempThighWdot(2);
        tempq[0] = device.Filter.x_hat(6);
        tempq[1] = device.Filter.x_hat(7);
        tempq[2] = device.Filter.x_hat(8);
        tempq[3] = device.Filter.x_hat(9);

        quat::rotateVec(temp, tempq, temp);
        device.a0 = Vector3d(temp[0], temp[1], temp[2]);

        //device.a0 << 0., 0., 0.;
    }
    if ( device.imu_location == r_thigh ) {
        device.a0 << 0., 0., 0.;
    }
    if ( device.imu_location == torso ) {
        //TODO
        device.a0 << 0., 0., 0.;
    }
    if ( device.imu_location == single ) {
        device.a0 << 0., 0., 0.;
    }

    VectorXd measurement(9);
    measurement << device.velocity, device.Dvelocity, device.acc;

    device.Filter.update(device.dt, device.a0, measurement);

    device.data.linear_acceleration.x = device.acc(0);
    device.data.linear_acceleration.y = device.acc(1);
    device.data.linear_acceleration.z = device.acc(2);

    device.data.linear_acceleration_model.x = device.Filter.h(6);
    device.data.linear_acceleration_model.y = device.Filter.h(7);
    device.data.linear_acceleration_model.z = device.Filter.h(8);

    device.data.angular_acceleration_model.x = device.Filter.x_hat(3);
    device.data.angular_acceleration_model.y = device.Filter.x_hat(4);
    device.data.angular_acceleration_model.z = device.Filter.x_hat(5);

    device.data.angular_acceleration_measure.x = device.Dvelocity(0);
    device.data.angular_acceleration_measure.y = device.Dvelocity(1);
    device.data.angular_acceleration_measure.z = device.Dvelocity(2);

    device.data.angular_velocity.x = device.Filter.x_hat(0);
    device.data.angular_velocity.y = device.Filter.x_hat(1);
    device.data.angular_velocity.z = device.Filter.x_hat(2);

    device.data.angular_velocity_measure.x = device.velocity(0);
    device.data.angular_velocity_measure.y = device.velocity(1);
    device.data.angular_velocity_measure.z = device.velocity(2);

    device.data.orientation.w = device.Filter.x_hat(6);
    device.data.orientation.x = device.Filter.x_hat(7);
    device.data.orientation.y = device.Filter.x_hat(8);
    device.data.orientation.z = device.Filter.x_hat(9);

    device.data.quaternion_velocity_model.w = device.Filter.x_hat(10);
    device.data.quaternion_velocity_model.x = device.Filter.x_hat(11);
    device.data.quaternion_velocity_model.y = device.Filter.x_hat(12);
    device.data.quaternion_velocity_model.z = device.Filter.x_hat(13);
}

/* FUNCTION publish(imu& device)
 * Publish current stored values in IMU object.
 * @PARAM device - IMU object which will publish values to its assigned node.
 */
bool ambcap_EKF::publish(imu& device) {
    device.data_pub_.publish(device.data);
}

/* FUNCTION publishRunning()
 * Checks all devices for running status, updates then publishes in that order.
 */
bool ambcap_EKF::publishRunning() {
    //Update all running components first
    if ( L_foot.running ) {
        update(L_foot);
        filter(L_foot);
    }
    if ( L_shank.running ) {
        update(L_shank);
        filter(L_shank);
    }
    if ( L_thigh.running ) {
        update(L_thigh);
        filter(L_thigh);
    }
    if ( R_foot.running ) {
        update(R_foot);
        filter(R_foot);
    }
    if ( R_thigh.running ) {
        update(R_thigh);
        filter(R_thigh);
        //publish(R_thigh);
    }
    if ( R_shank.running ) {
        update(R_shank);
        filter(R_shank);
        //publish(R_shank);
    }
    if ( Torso.running ) {
        update(Torso);
        filter(Torso);
    }
    if ( Single.running ) {
        update(Single);
        filter(Single);
    }

    //Then filter and publish (keeps timing tighter).
    if ( publishall ) {
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
    }

    return true;
}

/* FUNCTION checkCalibration()
 * Checks all possible devices for running status and calibration status.
 * If running and not calibrated sends object to calibrate_gyro function.
 */
bool ambcap_EKF::checkCalibration() {
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

void ambcap_EKF::imu::check_cal() {
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
bool ambcap_EKF::imu::calibrate_gyro() {
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

void ambcap_EKF::imu::pitch_roll_ref() {
    float q_ref[4];
    q_ref[0] = 1.f;
    q_ref[1] = 0.f;
    q_ref[2] = 0.f;
    q_ref[3] = 0.f;

    for (int i=0; i<100; i++) {
        MPU.MahonyAHRSupdateIMU();
        usleep(5000);
    }
    quat::inv(MPU.v_quat, q_sensor_cal);
    quat::prod(q_sensor_cal, q_ref, q_sensor_cal);
    ROS_INFO("Sensor Initialized at %f, %f, %f, %f", q_sensor_cal[0], q_sensor_cal[1], q_sensor_cal[2], q_sensor_cal[3]);
    doPR = false;
}

void ambcap_EKF::imu::yaw_ref() {
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
    //quat::inv(temp_q, temp_q);
    quat::prod(q_sensor_cal, temp_q, q_sensor_cal);
    ROS_INFO("Sensor Initialized at %f, %f, %f, %f", q_sensor_cal[0], q_sensor_cal[1], q_sensor_cal[2], q_sensor_cal[3]);

    doYaw = false;
}

bool ambcap_EKF::imu::cal_gyr(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp) {
    iscalibrated = false;
    return true;
}

bool ambcap_EKF::imu::pitch_roll_serv(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp) {
    doPR = true;
    return true;
}

bool ambcap_EKF::imu::yaw_serv(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp) {
    doYaw = true;
    return true;
}
