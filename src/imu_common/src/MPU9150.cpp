/*=====================================================================================================
// I2C Tools for the BeagleBone Black
//=====================================================================================================
//
// Set of tools for to utilize the I2C interface of BBB.
//
// Date			Author			Notes
// 02/10/2014 	Jake Reher		Initial Release
// 04/01/2014   Jake Reher      Finalized Deadreckoning (which was subsequently removed after proving
//                              unsatisfactory for online applications). Code polished and finalized.
// 04/14/2014   Jake Reher      Made several sign changes and zeroed z axis proportial gains to eliminate
//                              error buildup on z axis of quaternion in 6DOF measurement cases.
// 06/06/2014   Jake Reher      Changed accelerometer range to +/- 4 g's. 
// 06/17/2014   Jake Reher      Added magnetometer biasing and vertical orientation capabilities.
//
//=====================================================================================================*/
//---------------------------------------------------------------------------------------------------

#include "MPU9150.h"

#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <math.h>

/* CONSTRUCTOR
 *
 */
MPU9150::MPU9150(uint8_t bus, uint8_t address, const char *bias, float freq, bool vertical, bool magnetometer) {
    // Sets the bus, address, and bias of the IMU.
    this->bus = bus;
    devAddress = address;
    sampleFreq = freq;
    this->vert_orient = vertical;

	// Loads the config file for the sensor and assigns the bias.
    ifstream configfile;
    configfile.open(bias);
    if ( !configfile.is_open() ){
        cout << "Failed to open configuration file!" << endl;
        GYRbias[0] = GYRbias[1] = GYRbias[2] = 0.;
        ACCbias[0] = ACCbias[1] = ACCbias[2] = 0.;
        IMUscale[0] = IMUscale[4] = IMUscale[8] = 1.;
        IMUscale[1] = IMUscale[2] = IMUscale[3] = IMUscale[5] = IMUscale[6] = IMUscale[7] = 0.;
    } else {
        for (int i=0; i<15; i++) {
            if (i < 3){
                configfile >> GYRbias[i];
            } else if (i < 6) {
                configfile >> ACCbias[i-3];
            }
        	else {
                configfile >> IMUscale[i-6];
            }
        }
    }
    if ( magnetometer ) {
        if ( !configfile.is_open() ) {
            cout << "Failed to open configuration file!" << endl;
            MAGbias[0] = MAGbias[1] = MAGbias[2] = 0.f;
            MAGscale[0] = MAGscale[4] = MAGscale[8] = 1.f;
            MAGscale[1] = MAGscale[2] = MAGscale[3] = MAGscale[5] = MAGscale[6] = MAGscale[7] = 0.f;
        } else {
            for (int i=0; i<12; i++) {
                if (i<3) {
                    configfile >> MAGbias[i];
                } else {
                    configfile >> MAGscale[i-3];
                }
            }
        }
    }
    configfile.close();

    // Load butterworth filter constants.
    const vector<float> a = {1.0, -1.56101807580072, 0.641351538057563};
    const vector<float> b = {0.0200833655642113, 0.0401667311284225, 0.0200833655642113};

    //Setup butterworth filters.
    butterX.reset(new Butter(b, a));
    butterY.reset(new Butter(b, a));
    butterZ.reset(new Butter(b, a));

    twoKp = twoKpDef;	// 2 * proportional gain (Kp)
    twoKi = twoKiDef;	// 2 * integral gain (Ki)
    integralFBx = 0.0f;
    integralFBy = 0.0f;
    integralFBz = 0.0f;	// integral error terms scaled by Ki.
}

/* DESTRUCTOR
 *
 */
MPU9150::~MPU9150() {
    //TODO destructor
}

/* FUNCTION initialize()
 * Initialize the IMU.
 * Sets sleep mode off. Sets accel/gyro range.
 * Sets digital low pass filter settings.
 * Sets the clock to the x-gyro (this was found by others online to be more reliable).
 */
void MPU9150::initialize() {
    setSleep(false);

    //Initialize the compass
    initCompass();

    setClock(MPU6050_CLOCK_PLL_XGYRO);
    setAccelRange(MPU6050_ACCEL_FS_4);
    setGyroRange(MPU6050_GYRO_FS_1000);
    setDLPFMode(MPU6050_DLPF_BW_98);
    usleep(50000);
    initOrientation();

    //Initial convergence of butterworth filters. Takes 0.5 seconds @ 200Hz.
    for (int i=0; i<100; i++)
        read6DOF();
}

/* FUNCTION setSleep()
 * Sets the sleep mode of the IMU.
 * @PARAM enabled - If false, turn sleep mode off.
 */
void MPU9150::setSleep(bool enabled) {
    BBBI2C::writeBit(bus, devAddress, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, enabled);
}

/* FUNCTION setClock()
 * Sets the clock source of the IMU.
 * @PARAM source - Register id of clock source.
 */
void MPU9150::setClock(uint8_t source) {
    BBBI2C::writeBits(bus, devAddress, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH, source);
}

/* FUNCTION setGyroRange()
 * Sets the gyroscope readable range.
 * @PARAM range - Gyro range setting.
 */
void MPU9150::setGyroRange(uint8_t range) {
    BBBI2C::writeBits(bus, devAddress, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, range);
    this->GYRO_CURRENT_SETTING = range;
}

/* FUNCTION setAccelRange()
 * Sets the accelerometer readable range.
 * @PARAM range - Accel range setting.
 */
void MPU9150::setAccelRange(uint8_t range) {
    BBBI2C::writeBits(bus, devAddress, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, range);
    this->ACCEL_CURRENT_SETTING = range;
}

/* FUNCTION setDLPFMode(uint8_t mode)
 * Set the digital low pass filter configuration.
 * The DLPF_CFG parameter sets the digital low pass filter configuration. It
 * also determines the internal sampling rate used by the device as shown in
 * the table below.
 *
 * Note: The accelerometer output rate is 1kHz. This means that for a Sample
 * Rate greater than 1kHz, the same accelerometer sample may be output to the
 * FIFO, DMP, and sensor registers more than once.
 *
 * <pre>
 *          |   ACCELEROMETER    |           GYROSCOPE
 * DLPF_CFG | Bandwidth | Delay  | Bandwidth | Delay  | Sample Rate
 * ---------+-----------+--------+-----------+--------+-------------
 * 0        | 260Hz     | 0ms    | 256Hz     | 0.98ms | 8kHz
 * 1        | 184Hz     | 2.0ms  | 188Hz     | 1.9ms  | 1kHz
 * 2        | 94Hz      | 3.0ms  | 98Hz      | 2.8ms  | 1kHz
 * 3        | 44Hz      | 4.9ms  | 42Hz      | 4.8ms  | 1kHz
 * 4        | 21Hz      | 8.5ms  | 20Hz      | 8.3ms  | 1kHz
 * 5        | 10Hz      | 13.8ms | 10Hz      | 13.4ms | 1kHz
 * 6        | 5Hz       | 19.0ms | 5Hz       | 18.6ms | 1kHz
 * 7        |   -- Reserved --   |   -- Reserved --   | Reserved
 * </pre>
 *
 * @param mode New DLFP configuration setting
 * @see getDLPFBandwidth()
 * @see MPU6050_DLPF_BW_256
 * @see MPU6050_RA_CONFIG
 * @see MPU6050_CFG_DLPF_CFG_BIT
 * @see MPU6050_CFG_DLPF_CFG_LENGTH
 */
void MPU9150::setDLPFMode(uint8_t mode) {
    BBBI2C::writeBits(bus, devAddress, MPU6050_RA_CONFIG, MPU6050_CFG_DLPF_CFG_BIT, MPU6050_CFG_DLPF_CFG_LENGTH, mode);
}

/* FUNCTION initCompass()
 * Initializes the compass and allows the values to be read as slave to IMU.
 *
 */
void MPU9150::initCompass() {
    //Enable master I2C mode
    BBBI2C::writeBit(bus,devAddress, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_I2C_MST_EN_BIT, 1);

    //Slave0 is set as the magnetometer registers to be read from (0x02-0x08)
    //Slave1 is set as the magnetometer registers to be written from (used to power on/off and enable).
    //-------------------------------------------------------------------------------------------------
    //Sets the MPU9150 to wait for ext sensor data before data ready interrupt triggered.
    BBBI2C::writeBit(bus, devAddress, MPU6050_RA_I2C_MST_CTRL, MPU6050_WAIT_FOR_ES_BIT, 1);
    //Sets slave0 to read.
    BBBI2C::writeBit(bus,devAddress, MPU6050_RA_I2C_SLV0_ADDR, MPU6050_I2C_SLV_RW_BIT, 1);
    //Sets slave0 to the Magnetometer I2C address (default 0x0C).
    BBBI2C::writeBits(bus, devAddress, MPU6050_RA_I2C_SLV0_ADDR, MPU6050_I2C_SLV_ADDR_BIT, MPU6050_I2C_SLV_ADDR_LENGTH, 0x0C);
    //Sets the start point for slave0 read (0x03 is mag_X_L)
    BBBI2C::writeByte(bus, devAddress, MPU6050_RA_I2C_SLV0_REG, 0x02);
    //Sets the slave0 status to enable I2C data transaction.
    BBBI2C::writeBit(bus, devAddress, MPU6050_RA_I2C_SLV0_CTRL, MPU6050_I2C_SLV_EN_BIT, 1);
    //Specifies number of bytes to be read from magnetometer (6 - x,y,z H and L).
    BBBI2C::writeBits(bus, devAddress, MPU6050_RA_I2C_SLV0_CTRL, MPU6050_I2C_SLV_LEN_BIT, MPU6050_I2C_SLV_LEN_LENGTH, 8);

    //Sets slave1 to the Magnetometer I2C address (default 0x0C).
    BBBI2C::writeBits(bus, devAddress, MPU6050_RA_I2C_SLV1_ADDR, MPU6050_I2C_SLV_ADDR_BIT, MPU6050_I2C_SLV_ADDR_LENGTH, 0x0C);
    //Sets slave1 to write and which register the write is to start from.
    BBBI2C::writeBit(bus,devAddress, MPU6050_RA_I2C_SLV1_ADDR, MPU6050_I2C_SLV_RW_BIT, 0);
    BBBI2C::writeByte(bus, devAddress, MPU6050_RA_I2C_SLV1_REG, 0x0A);
    //Enable and set byte length to one.
    BBBI2C::writeBit(bus, devAddress, MPU6050_RA_I2C_SLV1_CTRL, MPU6050_I2C_SLV_EN_BIT, 1);
    BBBI2C::writeBits(bus, devAddress, MPU6050_RA_I2C_SLV1_CTRL, MPU6050_I2C_SLV_LEN_BIT, MPU6050_I2C_SLV_LEN_LENGTH, 1);
    //Populate the container register for slave write (0x64) with enable command.
    BBBI2C::writeByte(bus, devAddress, MPU6050_RA_I2C_SLV1_DO, 0x01);

    //Specify slave0 and slave1 as having a delay rate.
    BBBI2C::writeBits(bus, devAddress, MPU6050_RA_I2C_MST_DELAY_CTRL, MPU6050_DELAYCTRL_I2C_SLV1_DLY_EN_BIT, 2, 0x03);
    //Specify master I2C delay.
    BBBI2C::writeBits(bus, devAddress, MPU6050_RA_I2C_SLV4_CTRL, 4, 5, 0x04);
    //Clear write container.
    BBBI2C::writeByte(bus, devAddress, MPU6050_RA_I2C_SLV1_DO, 0x01);
    //Reset user settings
    BBBI2C::writeByte(bus,devAddress, MPU6050_RA_USER_CTRL, 0x00);
    //Populate write container.
    BBBI2C::writeByte(bus, devAddress, MPU6050_RA_I2C_SLV1_DO, 0x01);
    //Enable master I2C mode
    BBBI2C::writeBit(bus,devAddress, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_I2C_MST_EN_BIT, 1);

    ////////////TODO: add unittest code for checking if compass working properly
}

/* FUNCTION read6DOF()
 * Reads the three axis accel and gyro, converting to human readable format
 * as appropriate for the current settings.
 */
void MPU9150::read6DOF() {
    float tempf[3];
    int16_t tempi[6];

    BBBI2C::readBytes(bus, devAddress, MPU6050_RA_ACCEL_XOUT_H, 14, buffer);
    tempi[0] = ((int16_t)buffer[0]) << 8 | buffer[1];
    tempi[1] = ((int16_t)buffer[2]) << 8 | buffer[3];
    tempi[2] = ((int16_t)buffer[4]) << 8 | buffer[5];
    tempi[3] = ((int16_t)buffer[8]) << 8 | buffer[9];
    tempi[4] = ((int16_t)buffer[10]) << 8 | buffer[11];
    tempi[5] = ((int16_t)buffer[12]) << 8 | buffer[13];

    for (int i=0; i<6; i++) {
        tempi[i] = ~tempi[i] + 1;
    }

    if (ACCEL_CURRENT_SETTING == MPU6050_ACCEL_FS_2) {
        for (int i=0; i<3; i++){
            tempf[i] = (float)tempi[i] / 16384.0 * 9.80665;//Output in m/s/s.
        }
        tempf[1] *= -1.0f;
        tempf[2] *= -1.0f;

        tempf[0] -= ACCbias[0];
        tempf[1] -= ACCbias[1]; //Y and Z axes must be flipped signs as biasing is done in madgwick frame of reference.
        tempf[2] -= ACCbias[2];
        v_acc[0] = IMUscale[0]*tempf[0] + IMUscale[1]*tempf[1] + IMUscale[2]*tempf[2];
        v_acc[1] = IMUscale[3]*tempf[0] + IMUscale[4]*tempf[1] + IMUscale[5]*tempf[2];
        v_acc[2] = IMUscale[6]*tempf[0] + IMUscale[7]*tempf[1] + IMUscale[8]*tempf[2];
    }
    if (ACCEL_CURRENT_SETTING == MPU6050_ACCEL_FS_4) {
        for (int i=0; i<3; i++){
            tempf[i] = (float)tempi[i] / 8192.0 * 9.80665;
        }
        tempf[1] *= -1.0f;
        tempf[2] *= -1.0f;

        tempf[0] -= ACCbias[0];
        tempf[1] -= ACCbias[1]; //Y and Z axes must be flipped signs as biasing is done in madgwick frame of reference.
        tempf[2] -= ACCbias[2];
        v_acc[0] = IMUscale[0]*tempf[0] + IMUscale[1]*tempf[1] + IMUscale[2]*tempf[2];
        v_acc[1] = IMUscale[3]*tempf[0] + IMUscale[4]*tempf[1] + IMUscale[5]*tempf[2];
        v_acc[2] = IMUscale[6]*tempf[0] + IMUscale[7]*tempf[1] + IMUscale[8]*tempf[2];
    }
    if (ACCEL_CURRENT_SETTING == MPU6050_ACCEL_FS_8) {
        for (int i=0; i<3; i++){
            tempf[i] = (float)tempi[i] / 4096.0 * 9.80665;
        }
        tempf[1] *= -1.0f;
        tempf[2] *= -1.0f;

        tempf[0] -= ACCbias[0];
        tempf[1] -= ACCbias[1]; //Y and Z axes must be flipped signs as biasing is done in madgwick frame of reference.
        tempf[2] -= ACCbias[2];
        v_acc[0] = IMUscale[0]*tempf[0] + IMUscale[1]*tempf[1] + IMUscale[2]*tempf[2];
        v_acc[1] = IMUscale[3]*tempf[0] + IMUscale[4]*tempf[1] + IMUscale[5]*tempf[2];
        v_acc[2] = IMUscale[6]*tempf[0] + IMUscale[7]*tempf[1] + IMUscale[8]*tempf[2];
    }
    if (ACCEL_CURRENT_SETTING == MPU6050_ACCEL_FS_16) {
        for (int i=0; i<3; i++){
            tempf[i] = (float)tempi[i] / 2048.0 * 9.80665;
        }
        tempf[1] *= -1.0f;
        tempf[2] *= -1.0f;

        tempf[0] -= ACCbias[0];
        tempf[1] -= ACCbias[1]; //Y and Z axes must be flipped signs as biasing is done in madgwick frame of reference.
        tempf[2] -= ACCbias[2];
        v_acc[0] = IMUscale[0]*tempf[0] + IMUscale[1]*tempf[1] + IMUscale[2]*tempf[2];
        v_acc[1] = IMUscale[3]*tempf[0] + IMUscale[4]*tempf[1] + IMUscale[5]*tempf[2];
        v_acc[2] = IMUscale[6]*tempf[0] + IMUscale[7]*tempf[1] + IMUscale[8]*tempf[2];
    }
    if (GYRO_CURRENT_SETTING == MPU6050_GYRO_FS_250) {
        for (int i=0; i<3; i++){
            v_gyr[i] = (float)tempi[i+3] / 131.0 * 0.0174533; //Output is rad/sec.
        }
        v_gyr[0] *= -1.0f;

        v_gyr[0] -= GYRbias[0];
        v_gyr[1] -= GYRbias[1];
        v_gyr[2] -= GYRbias[2];
    }
    if (GYRO_CURRENT_SETTING == MPU6050_GYRO_FS_500) {
        for (int i=0; i<3; i++){
            v_gyr[i] = (float)tempi[i+3] / 65.5 * 0.0174533;
        }
        v_gyr[0] *= -1.0f;

        v_gyr[0] -= GYRbias[0];
        v_gyr[1] -= GYRbias[1];
        v_gyr[2] -= GYRbias[2];
    }
    if (GYRO_CURRENT_SETTING == MPU6050_GYRO_FS_1000) {
        for (int i=0; i<3; i++){
            v_gyr[i] = (float)tempi[i+3] / 32.8 * 0.0174533;
        }
        v_gyr[0] *= -1.0f;

        v_gyr[0] -= GYRbias[0];
        v_gyr[1] -= GYRbias[1];
        v_gyr[2] -= GYRbias[2];
    }
    if (GYRO_CURRENT_SETTING == MPU6050_GYRO_FS_2000) {
        for (int i=0; i<3; i++){
            v_gyr[i] = (float)tempi[i+3] / 16.4 * 0.0174533;
        }
        v_gyr[0] *= -1.0f;

        v_gyr[0] -= GYRbias[0];
        v_gyr[1] -= GYRbias[1];
        v_gyr[2] -= GYRbias[2];
    }

    //If the IMU has been defined as a vertical IMU switch the X and Z axes.
    if (vert_orient == true) {
        float temp = v_acc[0];
        v_acc[0] = -1.f * v_acc[2];
        v_acc[2] = temp;

        temp = v_gyr[0];
        v_gyr[0] = -1.f * v_gyr[2];
        v_gyr[2] = temp;
    }

    //Low pass filter the acceleration
    //Disabled as it causes ~5-10ms phase lag in angle reading
    filt_acc[0] = butterX->update(v_acc[0]);
    filt_acc[1] = butterY->update(v_acc[1]);
    filt_acc[2] = butterZ->update(v_acc[2]);
}

/* FUNCTION read9DOF()
 * Runs the 6DOF code and also retreives magnetometer readings.
 *
 */
void MPU9150::read9DOF() {
    float tempf[3];
    //Get accel and gyro readings.
    read6DOF();

    //Read 6 bytes from the magnetometer.
    BBBI2C::readBytes(bus, devAddress, MPU9150_RA_MAG_XOUT_L, 6, buffer);
    int16_t tempi[3];

    //Output the magnetometer readings.
    //Measurement range from -4096 to 4095 in decimal.
    //Conversion is 0.3*int16 = Magnetic flux density (uT)
    tempi[0] = ((int16_t)buffer[1]) << 8 | buffer[0];
    tempi[1] = ((int16_t)buffer[3]) << 8 | buffer[2];
    tempi[2] = ((int16_t)buffer[5]) << 8 | buffer[4];

    for (int i=0; i<3; i++) {
        tempi[i] = ~tempi[i] + 1;
        v_mag[i] = 0.3f * (float)tempi[i];
    }

    tempf[0] = v_mag[0];
    tempf[1] = v_mag[1];
    tempf[2] = v_mag[2];

    v_mag[0] = MAGscale[0] * (tempf[0] - MAGbias[0]) + MAGscale[1] * (tempf[1] - MAGbias[1]) + MAGscale[2] * (tempf[2] - MAGbias[2]);
    v_mag[1] = MAGscale[4] * (tempf[1] - MAGbias[1]) + MAGscale[5] * (tempf[2] - MAGbias[2]);
    v_mag[2] = MAGscale[8] * (tempf[2] - MAGbias[2]);

    //If the IMU has been defined as a vertical IMU switch the X and Z axes.
    if (vert_orient == true) {
        float temp = v_mag[0];
        v_mag[0] = -1.f * v_mag[2];
        v_mag[2] = temp;
    }
}

/* FUNCTION initOrientation()
 * Initializes the euler angles and quaternions for the IMU.
 * Reduces convergence time of filter.
 *
 */
void MPU9150::initOrientation() {
    read6DOF();
    //read9DOF();
    float sign = copysignf(1.0, v_acc[2]);
    float roll = atan2(v_acc[1], sign * sqrt(v_acc[2]*v_acc[2] + v_acc[0]*v_acc[0]));
    float pitch = atan2(-v_acc[0], sqrt(v_acc[2]*v_acc[2] + v_acc[1]*v_acc[1]));
    float yaw = 0.0f; //TODO INIT WITH MAGNETOMETER

    v_euler[0] = roll;
    v_euler[1] = pitch;
    v_euler[2] = yaw;

    quat::euler2quatXYZ(v_euler,v_quat);
}

/*=====================================================================================================
// The algorithms below were taken from MahonyAHRS.c and adapted for use in the AMBER Lab.
//=====================================================================================================
//
// Madgwick's implementation of Mayhony's AHRS algorithm.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date			Author			Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
// 02/05/2014 	Jake Reher		Edited functions for use with AMBER Lab Prosthetic and added euler angle calculations.
// 04/06/2014   Jake Reher      Additional changes for better use of quaternion class.
//
//=====================================================================================================*/
//---------------------------------------------------------------------------------------------------
// AHRS algorithm update

void MPU9150::MahonyAHRSupdate() {
    float recipNorm;
    float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
    float hx, hy, bx, bz;
    float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
    float halfex, halfey, halfez;
    float qa, qb, qc;
    float tempgx, tempgy, tempgz;

    read9DOF(); //Updates AHRS readings.

    float ax = v_acc[0];
    float ay = v_acc[1];
    float az = v_acc[2];
    float gx = v_gyr[0];
    float gy = v_gyr[1];
    float gz = v_gyr[2];
    float mx = v_mag[0];
    float my = v_mag[1];
    float mz = v_mag[2];
    float q0 = v_quat[0];
    float q1 = v_quat[1];
    float q2 = v_quat[2];
    float q3 = v_quat[3];

    // Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
    if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
        MahonyAHRSupdateIMU();
        return;
    }

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

        // Normalise accelerometer measurement
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Normalise magnetometer measurement
        recipNorm = invSqrt(mx * mx + my * my + mz * mz);
        mx *= recipNorm;
        my *= recipNorm;
        mz *= recipNorm;

        // Auxiliary variables to avoid repeated arithmetic
        q0q0 = q0 * q0;
        q0q1 = q0 * q1;
        q0q2 = q0 * q2;
        q0q3 = q0 * q3;
        q1q1 = q1 * q1;
        q1q2 = q1 * q2;
        q1q3 = q1 * q3;
        q2q2 = q2 * q2;
        q2q3 = q2 * q3;
        q3q3 = q3 * q3;

        // Reference direction of Earth's magnetic field
        hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
        hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
        bx = sqrt(hx * hx + hy * hy);
        bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

        // Estimated direction of gravity and magnetic field
        halfvx = q1q3 - q0q2;
        halfvy = q0q1 + q2q3;
        halfvz = q0q0 - q1q1 - q2q2 + q3q3;
        halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
        halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
        halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);

        // Error is sum of cross product between estimated direction and measured direction of field vectors
        halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
        halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
        halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);

        // Compute and apply integral feedback if enabled
        if(twoKi > 0.0f) {
            integralFBx += twoKi * halfex * (1.0f / sampleFreq);	// integral error scaled by Ki
            integralFBy += twoKi * halfey * (1.0f / sampleFreq);
            integralFBz += twoKi * halfez * (1.0f / sampleFreq);
            gx += integralFBx;	// apply integral feedback
            gy += integralFBy;
            gz += integralFBz;
        }
        else {
            integralFBx = 0.0f;	// prevent integral windup
            integralFBy = 0.0f;
            integralFBz = 0.0f;
        }

        // Apply proportional feedback
        gx += twoKp * halfex;
        gy += twoKp * halfey;
        gz += twoKp * halfez;
    }

    // Integrate rate of change of quaternion
    tempgx = gx * (0.5f * (1.0f / sampleFreq));		// pre-multiply common factors
    tempgy = gy * (0.5f * (1.0f / sampleFreq));
    tempgz = gz * (0.5f * (1.0f / sampleFreq));
    qa = q0;
    qb = q1;
    qc = q2;
    q0 += (-qb * tempgx - qc * tempgy - q3 * tempgz);
    q1 += (qa * tempgx + qc * tempgz - q3 * tempgy);
    q2 += (qa * tempgy - qb * tempgz + q3 * tempgx);
    q3 += (qa * tempgz + qb * tempgy - qc * tempgx);

    // Normalise quaternion
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;

    v_quat[0] = q0;
    v_quat[1] = q1;
    v_quat[2] = q2;
    v_quat[3] = q3;

    quat::eulerXZY(v_quat,v_euler);
}

//---------------------------------------------------------------------------------------------------
// IMU algorithm update

void MPU9150::MahonyAHRSupdateIMU(float dt) {
    float recipNorm;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    read6DOF(); //Updates IMU readings. Uses filtered accel data.

    float ax = v_acc[0];
    float ay = v_acc[1];
    float az = v_acc[2];
    float gx = v_gyr[0];
    float gy = v_gyr[1];
    float gz = 0.f; // Assume zero yaw.
    float q0 = v_quat[0];
    float q1 = v_quat[1];
    float q2 = v_quat[2];
    float q3 = v_quat[3];

    float tempgx;
    float tempgy;
    float tempgz;

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

        // Normalise accelerometer measurement
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Estimated direction of gravity and vector perpendicular to magnetic flux
        halfvx = q1 * q3 - q0 * q2;
        halfvy = q0 * q1 + q2 * q3;
        halfvz = q0 * q0 - 0.5f + q3 * q3;

        // Error is sum of cross product between estimated and measured direction of gravity
        halfex = (ay * halfvz - az * halfvy);
        halfey = (az * halfvx - ax * halfvz);
        halfez = (ax * halfvy - ay * halfvx);

        // Compute and apply integral feedback if enabled
        if(twoKi > 0.0f) {
            integralFBx += halfex;	// integral error scaled by Ki
            integralFBy += halfey;
            //integralFBz += twoKi * halfez * (1.0f / sampleFreq);
            integralFBz = 0.0; //Preventing integral windup of yaw. This is causing problems.
            gx += twoKi * integralFBx;	// apply integral feedback
            gy += twoKi * integralFBy;
            gz += twoKi * integralFBz;
        }
        else {
            integralFBx = 0.0f;	// prevent integral windup
            integralFBy = 0.0f;
            integralFBz = 0.0f;
        }

        // Apply proportional feedback
        gx += twoKp * halfex;
        gy += twoKp * halfey;
        //gz += twoKp * halfez;
        gz += 0.f; //Preventing linear error buildup of yaw.
    }

    // Integrate rate of change of quaternion
    tempgx = gx;		// pre-multiply common factors
    tempgy = gy;
    tempgz = gz;
    qa = q0;
    qb = q1;
    qc = q2;
    q0 += (-qb * tempgx - qc * tempgy - q3 * tempgz) * (0.5f * dt);
    q1 += (qa * tempgx + qc * tempgz - q3 * tempgy) * (0.5f * dt);
    q2 += (qa * tempgy - qb * tempgz + q3 * tempgx) * (0.5f * dt);
    q3 += (qa * tempgz + qb * tempgy - qc * tempgx) * (0.5f * dt);

    // Normalise quaternion
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;

    v_quat[0] = q0;
    v_quat[1] = q1;
    v_quat[2] = q2;
    v_quat[3] = 0.f;

    quat::eulerXZY(v_quat,v_euler);
}

void MPU9150::MahonyAHRSupdateIMU() {
    float recipNorm;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    float dt = sampleFreq;

    read6DOF(); //Updates IMU readings. Uses filtered accel data.

    float ax = v_acc[0];
    float ay = v_acc[1];
    float az = v_acc[2];
    float gx = v_gyr[0];
    float gy = v_gyr[1];
    float gz = 0.f; // Assume zero yaw.
    float q0 = v_quat[0];
    float q1 = v_quat[1];
    float q2 = v_quat[2];
    float q3 = v_quat[3];

    float tempgx;
    float tempgy;
    float tempgz;

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

        // Normalise accelerometer measurement
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Estimated direction of gravity and vector perpendicular to magnetic flux
        halfvx = q1 * q3 - q0 * q2;
        halfvy = q0 * q1 + q2 * q3;
        halfvz = q0 * q0 - 0.5f + q3 * q3;

        // Error is sum of cross product between estimated and measured direction of gravity
        halfex = (ay * halfvz - az * halfvy);
        halfey = (az * halfvx - ax * halfvz);
        halfez = (ax * halfvy - ay * halfvx);

        // Compute and apply integral feedback if enabled
        if(twoKi > 0.0f) {
            integralFBx += halfex;  // integral error scaled by Ki
            integralFBy += halfey;
            //integralFBz += twoKi * halfez * (1.0f / sampleFreq);
            integralFBz = 0.0; //Preventing integral windup of yaw. This is causing problems.
            gx += twoKi * integralFBx;  // apply integral feedback
            gy += twoKi * integralFBy;
            gz += twoKi * integralFBz;
        }
        else {
            integralFBx = 0.0f; // prevent integral windup
            integralFBy = 0.0f;
            integralFBz = 0.0f;
        }

        // Apply proportional feedback
        gx += twoKp * halfex;
        gy += twoKp * halfey;
        //gz += twoKp * halfez;
        gz += 0.f; //Preventing linear error buildup of yaw.
    }

    // Integrate rate of change of quaternion
    tempgx = gx;        // pre-multiply common factors
    tempgy = gy;
    tempgz = gz;
    qa = q0;
    qb = q1;
    qc = q2;
    q0 += (-qb * tempgx - qc * tempgy - q3 * tempgz) * (0.5f * dt);
    q1 += (qa * tempgx + qc * tempgz - q3 * tempgy) * (0.5f * dt);
    q2 += (qa * tempgy - qb * tempgz + q3 * tempgx) * (0.5f * dt);
    q3 += (qa * tempgz + qb * tempgy - qc * tempgx) * (0.5f * dt);

    // Normalise quaternion
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;

    v_quat[0] = q0;
    v_quat[1] = q1;
    v_quat[2] = q2;
    v_quat[3] = 0.f;

    quat::eulerXZY(v_quat,v_euler);
}

//---------------------------------------------------------------------------------------------------
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root
float MPU9150::invSqrt(float x) {
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i>>1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}






