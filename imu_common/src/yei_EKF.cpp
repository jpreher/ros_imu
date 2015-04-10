/*=====================================================================================================
// AMBCAP tools to enable easy configurability of reading from and number of imu devices in ROS.
//=====================================================================================================
//
// ROS class with objects and functionality to easily collect data from MPU 6050-9150 model IMU's
// and filter with an EKF on the BBB.
//
// Date         Author          Notes
// 09/12/2014   Jake Reher      Initial Release
// - 2015       Jake Reher      Additional Changes for prosthetic - calibration, tuning, functionality
//
//=====================================================================================================*/
//---------------------------------------------------------------------------------------------------

#include "yei_EKF.h"

/* CONSTRUCTOR yei_EKF(ros::NodeHandle nh, int freq)
 * Assigns the node handle to the class. Initializes with provided frequency.
 *  First assigns all devices to uncalibrated. Loads in desired capture setting.
 *  Finally initializes all device objects corresponding to the capture setting.
 * @PARAM nh - Node handle of ROS node using AMBCAP.
 * @PARAM freq - User selected frequency, used in ros::rate timer.
 */
yei_EKF::yei_EKF(ros::NodeHandle nh, int freq): rate((float)freq) {
    // TELL THE USER WHICH FILTER IS IN USE
    ROS_WARN("USING EKF FILTER - FOR ONLINE USE WITH PROSTHETIC");

    frequency = freq;
    node_handle_ = nh;

    R_shank.iscalibrated = false;
    R_thigh.iscalibrated = false;
    L_foot.iscalibrated = false;

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

    // Initialize the imus
    R_shank.imu_location = r_shank;
    R_thigh.imu_location = r_thigh;
    L_foot.imu_location = l_foot;
    R_shank.initialize(nh, Rad_shank, freq);
    R_thigh.initialize(nh, Rad_thigh, freq);
    L_foot.initialize(nh, Rad_foot, freq);

    // Setup and initialize the python wrapper for the sensor API
    char *argv[] = {"yei_sensor", "IMU_init", "IMU_callback"};
    int argc = sizeof(argv) / sizeof(char*);
    yei_python.initialize(argc, argv);

    // Let the sensor run for a bit before calibrating / etc
    for (int i=0; i<200; i++) {
        usleep(5000);
       getYEIreading();
    }

    // Clear the calibration by updating
    checkCalibration();
}

/* FUNCTION spin()
 * Main while loop of AMBCAP.
 * Checks calibration, if OK it updates and publishes all enabled
 *  devices and then spins ROS.
 */
bool yei_EKF::spin() {
    while(!ros::isShuttingDown()) {
        while(node_handle_.ok()) {
            ros::spinOnce();
            Callback();
            publishRunning();
            checkCalibration();
            rate.sleep();
        }
    }
}

/* FUNCTION updateEKF()
 * Main while loop of AMBCAP.
 * Checks calibration, if OK it updates and publishes and enabled
 *  devices and then spins ROS.
 */
bool yei_EKF::updateEKF() {
    if(!ros::isShuttingDown()) {
        if(node_handle_.ok()) {
            Callback();
            checkCalibration();
        }
    }
}

/* FUNCTION initialize(ros::NodeHandle& nh, imu& device)
 * Initializes provided imu object and starts the associated ros services on the node.
 * @PARAM nh - Node handle that the ROS services will be assosciated to.
 * @PARAM device - IMU device which will be initialized.
 */
bool yei_EKF::imu::initialize(ros::NodeHandle& nh, Vector3d &rad, int freq) {
    // Initialize all the things
    velocity.resize(3);
    Dvelocity.resize(3);
    acc.resize(3);
    a0.resize(3);
    r_init.resize(3);

    std::string param_base_path = "/imu/";
    std::string param_device;
    r_init << rad(0), rad(1), rad(2);
    this->frequency = (double)freq;

    //Address the imu to the proper parameters on the parameter server.
    if ( imu_location == l_foot ) {
        param_device = "l_foot";
        pitch_roll_ref_ = nh.advertiseService("l_foot_PR", &yei_EKF::imu::pitch_roll_serv, this);
        yaw_ref_        = nh.advertiseService("l_foot_yaw", &yei_EKF::imu::yaw_serv, this);
    }
    if ( imu_location == r_shank ) {
        param_device = "r_shank";
        pitch_roll_ref_ = nh.advertiseService("r_shank_PR", &yei_EKF::imu::pitch_roll_serv, this);
        yaw_ref_        = nh.advertiseService("r_shank_yaw", &yei_EKF::imu::yaw_serv, this);
    }
    if ( imu_location == r_thigh ) {
        param_device = "r_thigh";
        pitch_roll_ref_ = nh.advertiseService("r_thigh_PR", &yei_EKF::imu::pitch_roll_serv, this);
        yaw_ref_        = nh.advertiseService("r_thigh_yaw", &yei_EKF::imu::yaw_serv, this);
    }

    q_sensor_cal[0] = 1.f;
    q_sensor_cal[1] = 0.f;
    q_sensor_cal[2] = 0.f;
    q_sensor_cal[3] = 0.f;

    //Copy contents of newly established MPU9150 object to the device.
    ROS_INFO("Sensor Created");

    // Setup ekf
    x_init.resize(14);
    x_init << 0., 0.0001, 0.,
              0., 0.00001, 0.,
              1., 0., 0., 0.,
              0., 0., 0., 0.;

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
    dt = 0.005;

    // Set up calibration steps
    iscalibrated = false;
    doPR = true;
    doYaw = false;
    time_last_run = ros::Time::now().toSec();

    //Set up the publisher and servicer.
    data_pub_ = nh.advertise<imu_common::imu>(param_device, 1000);
    //device.calibrate_serv_ = nh.advertiseService("calibrate_" + param_device, device.calibrate_callback);

    running = true;
    usleep(50000);
    return true;
}

/* FUNCTION filter()
 * Rotates measurements according to reference and runs EKF.
 */
void yei_EKF::filter(imu& device) {
    // Rotate the measurements
    float tempa[3], tempg[3];
    tempa[0] = device.data.linear_acceleration.x;
    tempa[1] = device.data.linear_acceleration.y;
    tempa[2] = device.data.linear_acceleration.z;
    quat::rotateVec(tempa, device.q_sensor_cal, tempa);
    device.acc << tempa[0],
                  0.,
                  tempa[2];

    tempg[0] = device.data.angular_velocity_measure.x;
    tempg[1] = device.data.angular_velocity_measure.y;
    tempg[2] = device.data.angular_velocity_measure.z;
    quat::rotateVec(tempg, device.q_sensor_cal, tempg);

    device.Dvelocity = Vector3d( 0., (-tempg[1] - device.velocity(1)) / 0.005, 0.);
    device.velocity = Vector3d( 0., -tempg[1], 0.);

    // Get previous joint acceleration.
    device.a0.resize(3);
    if ( device.imu_location == l_foot ) {
        //TODO
        device.a0 << 0., 0., 0.;
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

    VectorXd measurement(9);
    measurement << device.velocity, device.Dvelocity, device.acc;

    device.Filter.update(device.dt, device.a0, measurement);

    device.data.header.stamp = ros::Time::now();

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

    device.data.orientation.w = device.Filter.x_hat(6);
    device.data.orientation.x = device.Filter.x_hat(7);
    device.data.orientation.y = device.Filter.x_hat(8);
    device.data.orientation.z = device.Filter.x_hat(9);

    device.data.quaternion_velocity_model.w = device.Filter.x_hat(10);
    device.data.quaternion_velocity_model.x = device.Filter.x_hat(11);
    device.data.quaternion_velocity_model.y = device.Filter.x_hat(12);
    device.data.quaternion_velocity_model.z = device.Filter.x_hat(13);
}

/* FUNCTION publishRunning()
 * Checks all devices for running status, updates then publishes in that order.
 */
bool yei_EKF::publishRunning() {
    //Update all running components first
    if ( L_foot.running ) {
        L_foot.data_pub_.publish(L_foot.data);
    }
    if ( R_thigh.running ) {
        R_thigh.data_pub_.publish(R_thigh.data);
    }
    if ( R_shank.running ) {
        R_shank.data_pub_.publish(R_shank.data);
    }

    return true;
}

/* FUNCTION filterRunning()
 * Checks all devices for running status, updates then publishes in that order.
 */
bool yei_EKF::filterRunning() {
    //Update all running components first
    if ( L_foot.running ) {
        filter(L_foot);
    }
    if ( R_thigh.running ) {
        filter(R_thigh);
    }
    if ( R_shank.running ) {
        filter(R_shank);
    }

    return true;
}

/* FUNCTION checkCalibration()
 * Checks all possible devices for running status and calibration status.
 * If running and not calibrated sends object to calibrate_gyro function.
 */
bool yei_EKF::checkCalibration() {
    if ( L_foot.running )
        L_foot.check_cal();
    if ( R_shank.running )
        R_shank.check_cal();
    if ( R_thigh.running )
        R_thigh.check_cal();
}

void yei_EKF::imu::check_cal() {
    if ( doPR )
        pitch_roll_ref();
    if ( doYaw )
        yaw_ref();
}


void yei_EKF::imu::pitch_roll_ref() {
    float q_ref[4], grav[3], acc[3];
    q_ref[0] = 1.f;
    q_ref[1] = 0.f;
    q_ref[2] = 0.f;
    q_ref[3] = 0.f;
    grav[0] = 0.f;
    grav[1] = 0.f;
    grav[2] = 1.f;

    acc[0] = data.linear_acceleration.x;
    acc[1] = data.linear_acceleration.y;
    acc[2] = data.linear_acceleration.z;
    quat::two_vec_q(acc, grav, q_sensor_cal);
    quat::inv(q_sensor_cal, q_sensor_cal);
    ROS_INFO("Sensor Initialized at %f, %f, %f, %f", q_sensor_cal[0], q_sensor_cal[1], q_sensor_cal[2], q_sensor_cal[3]);
    doPR = false;
}

void yei_EKF::imu::yaw_ref() {
    float fixed_vel_x[1000], fixed_vel_y[1000], fixed_vel_z[1000];
    float norm_fixed[1000];
    float pin_velocity_y[1000];
    float theta[1000];
    float yaw;
    float numerator, denominator;
    float temp[3], sign, temp_q[4];
    ROS_INFO("Started capturing movement, swing leg forward and back");
    // Not actually implemented right now in C++ -- Filler Function
    quat::prod(q_sensor_cal, temp_q, q_sensor_cal);
    ROS_INFO("Sensor Initialized at %f, %f, %f, %f", q_sensor_cal[0], q_sensor_cal[1], q_sensor_cal[2], q_sensor_cal[3]);

    doYaw = false;
}

/* FUNCTION pitch_roll_serv()
 * Callback if server is activated for PR calibration. Changes flag which will be caught in spin.
 */
bool yei_EKF::imu::pitch_roll_serv(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp) {
    doPR = true;
    return true;
}

/* FUNCTION checkCalibration()
 * Callback if server is activated for yaw calibration. NOTE: Will not actually calibrate at this time
 */
bool yei_EKF::imu::yaw_serv(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp) {
    doYaw = true;
    return true;
}

/* FUNCTION Callback()
 * Main callback. Calls python function to read sensor and populates the data.
 */
void yei_EKF::Callback() {
    // Get measurement here and populate the message
    getYEIreading();

    // Run the filter
    filterRunning();
}

void yei_EKF::getYEIreading() {
    float reading[27];

    yei_python.getLastStream(reading);

    R_shank.data.linear_acceleration.x = 9.81f * reading[0];
    R_shank.data.linear_acceleration.y = 9.81f * reading[1];
    R_shank.data.linear_acceleration.z = 9.81f * reading[2];
    R_shank.data.angular_velocity_measure.x = reading[3];
    R_shank.data.angular_velocity_measure.y = reading[4];
    R_shank.data.angular_velocity_measure.z = reading[5];
    R_shank.data.magnetometer.x = reading[6];
    R_shank.data.magnetometer.y = reading[7];
    R_shank.data.magnetometer.z = reading[8];

    R_thigh.data.linear_acceleration.x = 9.81f * reading[9];
    R_thigh.data.linear_acceleration.y = 9.81f * reading[10];
    R_thigh.data.linear_acceleration.z = 9.81f * reading[11];
    R_thigh.data.angular_velocity_measure.x = reading[12];
    R_thigh.data.angular_velocity_measure.y = reading[13];
    R_thigh.data.angular_velocity_measure.z = reading[14];
    R_thigh.data.magnetometer.x = reading[15];
    R_thigh.data.magnetometer.y = reading[16];
    R_thigh.data.magnetometer.z = reading[17];

    L_foot.data.linear_acceleration.x = 9.81f * reading[18];
    L_foot.data.linear_acceleration.y = 9.81f * reading[19];
    L_foot.data.linear_acceleration.z = 9.81f * reading[20];
    L_foot.data.angular_velocity_measure.x = reading[21];
    L_foot.data.angular_velocity_measure.y = reading[22];
    L_foot.data.angular_velocity_measure.z = reading[23];
    L_foot.data.magnetometer.x = reading[24];
    L_foot.data.magnetometer.y = reading[25];
    L_foot.data.magnetometer.z = reading[26];
}

