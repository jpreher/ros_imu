
#ifndef MPU9150_H
#define MPU9150_H

#include "BBB_I2C.h"
#include "MPU_RegMap.h"
#include "butterworth_util.h"
#include "quaternion_util.h"

#include <memory>
#include <fstream>
#include <stdexcept>
#include <stdint.h>

//---------------------------------------------------------------------------------------------------
// Mahony Filter Definitions
#define twoKpDef	(2.0f * 2.0f)	// 2 * proportional gain
#define twoKiDef	(2.0f * 0.05f)	// 2 * integral gain

typedef std::shared_ptr<Butter> ButterPtr;

class MPU9150 {
private:
    ButterPtr butterX;
    ButterPtr butterY;
    ButterPtr butterZ;

    uint8_t bus, devAddress;
    uint8_t buffer[64];
    float ACCbias[3];
    float IMUscale[9];
    float integralFBx, integralFBy, integralFBz;
    float twoKp, twoKi;

    int GYRO_CURRENT_SETTING;
    int ACCEL_CURRENT_SETTING;

    float invSqrt(float x);
    void setSleep(bool enabled);
    void setClock(uint8_t source);
    void setGyroRange(uint8_t range);
    void setAccelRange(uint8_t range);
    void setDLPFMode(uint8_t mode);
    void initCompass();

public:
    bool vert_orient;
    float GYRbias[3];
    float filt_acc[3];    //Removed as butterworth not in use
    float sampleFreq;
    float v_acc[3];         //ax,ay,az
    float v_gyr[3];         //gx,gy,gz
    float v_mag[3];         //mx,my,mz
    float v_quat[4];        //q0, q1, q2, q3 (q0=w)
    float v_cquat[4];       //q0, q1, q2, q3 removing initital orientation
    float v_euler[3];       //roll, pitch, yaw
    float ref_quat[4];

    MPU9150(uint8_t bus, uint8_t address, const char *bias, float freq, bool vertical);
    virtual ~MPU9150();
    void initialize();
    void read6DOF();
    void read9DOF();
    void initOrientation();
    void MahonyAHRSupdate();
    void MahonyAHRSupdateIMU();
};

#endif // MPU9150_H
