#ifndef YEI_THREESPACE
#define YEI_THREESPACE

#include <fcntl.h>		// File control definitions
#include <stdio.h> 		// Standard input/output definitions
#include <unistd.h> 	// UNIX standard function definitions
#include <termios.h> 	// POSIX terminal control definitions
#include <stdlib.h>     // malloc, free, rand
#include <string.h>
#include <sys/ioctl.h>
#include <thread>
#include <mutex>
#include <condition_variable>

#include "yei_commands.hpp"

class YEI3Space {
public:
    YEI3Space();
	~YEI3Space();

    // Sensor settings and data
    unsigned int SerialNumber;
    unsigned int streamingSlots[8];
    unsigned int stream_byte_len;
    char stream_parse_str[257];
    float offsetQ[4];

    // Configure and connect
	int openAndSetupComPort(const char* comport);

    // Streaming options
    int setupStreamSlots(int streamRate);
    int startStreaming();
    int stopStreaming();
    int getStream(float * data);

    // API commands
    int getSerialNumber();
    int getAllCorrectedComponentSensorData(float * gyro_rate3, float * accelerometer3, float * compass3);
    int offsetWithCurrentOrientation();
    int resetBaseOffset();
    int offsetWithQuaternion(const float * quat4);
    int setBaseOffsetWithCurrentOrientation();
    int tareWithCurrentOrientation();
    int tareWithQuaternion(const float * quat4);
    int tareWithRotationMatrix(const float * matrix9);
    int setAxisDirections(unsigned char axis_direction_byte);
    int resetKalmanFilter();
    int setAccelerometerRange(unsigned char accelerometer_range_setting);
    int setFilterMode(unsigned char mode);
    int setRunningAverageMode(unsigned char mode);
    int setGyroscopeRange(unsigned char gyroscope_range_setting);
    int setCompassRange(unsigned char compass_range_setting);
    int getEulerAngleDecompositionOrder(unsigned char * order);
    int getOffsetOrientationAsQuaternion();

private:
    // Multithreading
    std::mutex mu;              // thread mutex
    std::condition_variable cv; // Condition variable for passing thread locking order (prevent starvation)
    std::thread readThread;

    // Device specific options
	int BaudRate;
	int ByteSize;
	int StopBits;
	int Parity;
    int fd;                     // File handle for device
    struct termios old_options; // TERMIOS setting of device when first opened
    float * last_stream_data;
    bool streamON;
    bool threadON;
    bool newData;

    // Stream
    int checkStream();
    int streamThread();

    // Low level reading and writing
	int writeRead(const TSS_Command * cmd_info, const char * input_data, char * output_data);
    int readFile(unsigned int rtn_data_len, char *rtn_data_detail, char * output_data);

    // Data management
    unsigned char createChecksum(const char * command_bytes, const unsigned int num_bytes);
    int parseData(char* data, int len, const char* parser_str);
	void endian_swap_16(unsigned short * x);
	void endian_swap_32(unsigned int * x);
};


#endif // YEI_THREESPACE
