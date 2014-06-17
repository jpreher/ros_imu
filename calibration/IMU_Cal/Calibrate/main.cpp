#include <iostream>
#include <stdint.h>
#include "MPU9150.h"

#include <sys/time.h>
#include <fstream>
#include <unistd.h>
#include <string>

using namespace std;

char dummy;

float sampleFreq = 200.0f;
float dt = 1.0f/sampleFreq;

struct timeval mytime;
int startTime;
int newTime;
float timediff;
float t[200];
float g = 9.80556;

struct calibrate {
    float tempX[200];
    float tempY[200];
    float tempZ[200];
    float tempGX[200];
    float tempGY[200];
    float tempGZ[200];

    float ax1, ay1, az1;
    float ax2, ay2, az2;
    float ax3, ay3, az3;
    float ax4, ay4, az4;
    float ax5, ay5, az5;
    float ax6, ay6, az6;

    float gbx, gby, gbz;
    float abx, aby, abz;

    float M[3][3];

    void init() {
        ax1 = 0.;
        ay1 = 0.;
        az1 = 0.;
        ax2 = 0.;
        ay2 = 0.;
        az2 = 0.;
        ax3 = 0.;
        ay3 = 0.;
        az3 = 0.;
        ax4 = 0.;
        ay4 = 0.;
        az4 = 0.;
        ax5 = 0.;
        ay5 = 0.;
        az5 = 0.;
        ax6 = 0.;
        ay6 = 0.;
        az6 = 0.;

        gbx = 0.;
        gby = 0.;
        gbz = 0.;
        abx = 0.;
        aby = 0.;
        abz = 0.;
    }

    void update(){
        MPU9150 hipIMU(2, 0x68, "dummyCal.txt", sampleFreq, false);
        hipIMU.initialize();
        usleep(100000);

        int i = 0;
        bool end = false;
        gettimeofday(&mytime, NULL);
        startTime = mytime.tv_usec;
        newTime = startTime;

        while (end == false) {

            timediff = newTime - startTime;

            if (timediff >= dt*1000000) {

                hipIMU.read6DOF();
                tempX[i] = hipIMU.v_acc[0];
                tempY[i] = hipIMU.v_acc[1];
                tempZ[i] = hipIMU.v_acc[2];
                tempGX[i] = hipIMU.v_gyr[0];
                tempGY[i] = hipIMU.v_gyr[1];
                tempGZ[i] = hipIMU.v_gyr[2];

                if(i==0){
                    t[i] = timediff/1000000.0f;
                } else {
                    t[i] = t[i-1] + timediff/1000000.0f;
                }
                i++;
                usleep(100);
                gettimeofday(&mytime, NULL);
                startTime = mytime.tv_usec;
                newTime = startTime;

            } else {
                gettimeofday(&mytime, NULL);
                newTime = mytime.tv_usec;
                if (newTime <= startTime) {
                    newTime += 1000000;
                }
            }
            if (i>=200) {
                end = true;
            }
        }
    }
};

int main()
{    
    ofstream dummyCal;
    dummyCal.open ("dummyCal.txt");
    dummyCal << 0.0 << "\n" << 0.0 << "\n" << 0.0 << "\n" << //Gyro
                0.0 << "\n" << 0.0 << "\n" << 0.0 << "\n" << //Acc bias
                1.0 << "\n" << 0.0 << "\n" << 0.0 << "\n" << //Acc scale x
                0.0 << "\n" << 1.0 << "\n" << 0.0 << "\n" << //Acc scale y
                0.0 << "\n" << 0.0 << "\n" << 1.0 << endl;    //Acc scale z
    dummyCal.close();

    calibrate IMUCal;
    IMUCal.init();

    cout << "\n Positive X, Press ENTER" << endl;
    cin >> dummy;
    IMUCal.update();
    for (int j=0; j<200; j++) {
        IMUCal.ax1 += IMUCal.tempX[j];
        IMUCal.ay1 += IMUCal.tempY[j];
        IMUCal.az1 += IMUCal.tempZ[j];
        IMUCal.gbx += IMUCal.tempGX[j];
        IMUCal.gby += IMUCal.tempGY[j];
        IMUCal.gbz += IMUCal.tempGZ[j];
    }
    IMUCal.ax1 /= 200.;
    IMUCal.ay1 /= 200.;
    IMUCal.az1 /= 200.;


    cout << "\n Negative X, Press ENTER" << endl;
    cin >> dummy;
    IMUCal.update();
    for (int j=0; j<200; j++) {
        IMUCal.ax2 += IMUCal.tempX[j];
        IMUCal.ay2 += IMUCal.tempY[j];
        IMUCal.az2 += IMUCal.tempZ[j];
    }
    IMUCal.ax2 /= 200.;
    IMUCal.ay2 /= 200.;
    IMUCal.az2 /= 200.;

    cout << "\n Positive Y, Press ENTER" << endl;
    cin >> dummy;
    IMUCal.update();
    for (int j=0; j<200; j++) {
        IMUCal.ax3 += IMUCal.tempX[j];
        IMUCal.ay3 += IMUCal.tempY[j];
        IMUCal.az3 += IMUCal.tempZ[j];
    }
    IMUCal.ax3 /= 200.;
    IMUCal.ay3 /= 200.;
    IMUCal.az3 /= 200.;

    cout << "\n Negative Y, Press ENTER" << endl;
    cin >> dummy;
    IMUCal.update();
    for (int j=0; j<200; j++) {
        IMUCal.ax4 += IMUCal.tempX[j];
        IMUCal.ay4 += IMUCal.tempY[j];
        IMUCal.az4 += IMUCal.tempZ[j];
    }
    IMUCal.ax4 /= 200.;
    IMUCal.ay4 /= 200.;
    IMUCal.az4 /= 200.;

    cout << "\n Positive Z, Press ENTER" << endl;
    cin >> dummy;
    IMUCal.update();
    for (int j=0; j<200; j++) {
        IMUCal.ax5 += IMUCal.tempX[j];
        IMUCal.ay5 += IMUCal.tempY[j];
        IMUCal.az5 += IMUCal.tempZ[j];
        IMUCal.gbx += IMUCal.tempGX[j];
        IMUCal.gby += IMUCal.tempGY[j];
        IMUCal.gbz += IMUCal.tempGZ[j];
    }
    IMUCal.ax5 /= 200.;
    IMUCal.ay5 /= 200.;
    IMUCal.az5 /= 200.;
    IMUCal.gbx /= 200.;
    IMUCal.gby /= 200.;
    IMUCal.gbz /= 200.;

    cout << "\n Negative Z, Press ENTER" << endl;
    cin >> dummy;
    IMUCal.update();
    for (int j=0; j<200; j++) {
        IMUCal.ax6 += IMUCal.tempX[j];
        IMUCal.ay6 += IMUCal.tempY[j];
        IMUCal.az6 += IMUCal.tempZ[j];
    }
    IMUCal.ax6 /= 200.;
    IMUCal.ay6 /= 200.;
    IMUCal.az6 /= 200.;

    IMUCal.abx = IMUCal.ax1 + IMUCal.ax2 + IMUCal.ax3 + IMUCal.ax4 + IMUCal.ax5 + IMUCal.ax6;
    IMUCal.abx /= 6.;
    IMUCal.aby = IMUCal.ay1 + IMUCal.ay2 + IMUCal.ay3 + IMUCal.ay4 + IMUCal.ay5 + IMUCal.ay6;
    IMUCal.aby /= 6.;
    IMUCal.abz = IMUCal.az1 + IMUCal.az2 + IMUCal.az3 + IMUCal.az4 + IMUCal.az5 + IMUCal.az6;
    IMUCal.abz /= 6.;

    IMUCal.M[0][0] = (IMUCal.ax1-IMUCal.ax2) / (2. * g);
    IMUCal.M[0][1] = (IMUCal.ax3-IMUCal.ax4) / (2. * g);
    IMUCal.M[0][2] = (IMUCal.ax5-IMUCal.ax6) / (2. * g);

    IMUCal.M[1][0] = (IMUCal.ay1-IMUCal.ay2) / (2. * g);
    IMUCal.M[1][1] = (IMUCal.ay3-IMUCal.ay4) / (2. * g);
    IMUCal.M[1][2] = (IMUCal.ay5-IMUCal.ay6) / (2. * g);

    IMUCal.M[2][0] = (IMUCal.az1-IMUCal.az2) / (2. * g);
    IMUCal.M[2][1] = (IMUCal.az3-IMUCal.az4) / (2. * g);
    IMUCal.M[2][2] = (IMUCal.az5-IMUCal.az6) / (2. * g);

    ofstream CalFile;
    CalFile.open ("CalFile.txt");
    CalFile  << IMUCal.gbx << "    " << IMUCal.gby << "   " << IMUCal.gbz << "   \n" << //Gyro
                IMUCal.abx << "    " << IMUCal.aby << "   " << IMUCal.abz << "   \n\n" << //Acc bias
                IMUCal.M[0][0] << "    " << IMUCal.M[0][1] << "   " << IMUCal.M[0][2] << "   \n" << //Acc scale x
                IMUCal.M[1][0] << "    " << IMUCal.M[1][1] << "   " << IMUCal.M[1][2] << "   \n" << //Acc scale y
                IMUCal.M[2][0] << "    " << IMUCal.M[2][1] << "   " << IMUCal.M[2][2] << endl;    //Acc scale z
    CalFile.close();



    return 0;
}




