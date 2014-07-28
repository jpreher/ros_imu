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
float t[2000];
float g = 9.80556;

struct calibrate {
    float tempX[2000];
    float tempY[2000];
    float tempZ[2000];

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

                hipIMU.read9DOF();
                tempX[i] = hipIMU.v_mag[0];
                tempY[i] = hipIMU.v_mag[1];
                tempZ[i] = hipIMU.v_mag[2];

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
            if (i>=2000) {
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

    cout << "\n Get ready to rotate about all the axes, Press ENTER" << endl;
    cin >> dummy;
    IMUCal.update();

    ofstream CalFile;
    CalFile.open ("CalFile.txt");
    for (int i=0; i<2000; i++){
        CalFile << IMUCal.tempX[i] << ", " << \
                   IMUCal.tempY[i] << ", " << \
                   IMUCal.tempZ[i] << endl;
    }
    CalFile.close();

    return 0;
}




