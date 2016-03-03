#include "yei_threespace.hpp"

// Constructor
YEI3Space::YEI3Space() {
    fd = -2;            // File descriptor is invalid to start
    BaudRate = B115200; // 115200 baud rate
    ByteSize = CS8;     // 8 bits
    StopBits = CSTOPB; 	// One stop bit
    Parity 	 = ~PARENB; // No parity

    // Default streaming slots (can be programmatically set on your own by changing class values)
    streamingSlots[0]= TSS_GET_TARED_ORIENTATION_AS_QUATERNION; // stream slot0
    streamingSlots[1]= TSS_GET_CORRECTED_GYRO_RATE; // stream slot1
    streamingSlots[2]= TSS_GET_CORRECTED_ACCELEROMETER_VECTOR; // stream slot2
    streamingSlots[3]= TSS_GET_CORRECTED_COMPASS_VECTOR; // stream slot3
    streamingSlots[4]= TSS_NULL; // stream slot4
    streamingSlots[5]= TSS_NULL; // stream slot5
    streamingSlots[6]= TSS_NULL; // stream slot6
    streamingSlots[7]= TSS_NULL; // stream slot7

    stream_byte_len = 0; // To be set by setupStreamSlots()
    streamON = false;
    threadON = false;
    newData  = false;
}

// Destructor
YEI3Space::~YEI3Space() {
    stopStreaming();
    tcflush(fd, TCIOFLUSH); // clear buffer
    tcsetattr(fd, TCSANOW, &old_options); // reset the terminal options
    close(fd);
}

// This creates the comport handle and does initial setup like baudrates.
int YEI3Space::openAndSetupComPort(const char* comport)
{
    struct termios options;  // Terminal options
    int rc, status;
    fd_set rfds;
    char str[12];

    fcntl(fd, F_SETFL, 0);

    strcpy(str, "/dev/");
    strcat(str, comport);
    fd = open("/dev/ttyACM0", O_RDWR | O_NOCTTY | O_NDELAY | O_NONBLOCK);
    if ( fd == -1 ) {
        printf("COMPORT %s not available!\n", str);
        fd = -2;
        return 0;
    }

    // Get last COM port attributes
    tcgetattr(fd, &old_options);
    options = old_options;

    // Flush the port's buffers (in and out) before we start using it
    tcflush(fd, TCIOFLUSH);

    // Set incoming baud rates
    cfsetispeed(&options, B115200);

    // Set outgoing baud rates
    cfsetospeed(&options, B115200);

    // Control Mode Parameters
    options.c_cflag &= CS8;
    options.c_cflag &= ~INPCK;
    //options.c_cflag |= CLOCAL | CREAD;
    options.c_cflag &= ~CSTOPB;
    //options.c_cflag &= ~CRTSCTS;
    options.c_cflag &= ~(PARENB | PARODD);

    // Input Mode Parameters
    options.c_iflag = IGNBRK;
    options.c_iflag &= ~(IXON | IXOFF | IXANY);

    // Local Flag Parameters
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

    // Timeout and return length
    options.c_cc[1] = 0; // or use 1 if you feel like perma-waiting
    options.c_cc[VTIME]= 10; // Timeout in 0.1s

    tcflush(fd, TCIOFLUSH); // Flush input and output data from device

    // Set the new attributes
    if((rc = tcsetattr(fd, TCSANOW, &options)) != 0){
        perror("Failed to set attr: ");
        return 0;
    }

    /*    int mcs=0;
    ioctl(fd, TIOCMGET, &mcs);
    mcs |= TIOCM_RTS;
    ioctl(fd, TIOCMSET, &mcs);

    if (tcgetattr(fd, &options)!=0)
    {
        perror("tcgetattr() 4 failed: ");
        return 0;
    }

    options.c_cflag &= ~CRTSCTS; */

    if (tcsetattr(fd, TCSANOW, &options)!=0)
    {
        printf("tcsetattr() 2 failed");
        return 0;
    }

    printf("Established connection to %s: %d\n", comport, fd);
    tcflush(fd, TCIOFLUSH); // clear buffer

    return 1;
}

// When called this requests the serial number from the device and assigns it to the SerialNumber class member.
int YEI3Space::getSerialNumber() {
    return writeRead(&simple_commands[TSS_GET_SERIAL_NUMBER], NULL, (char *)&SerialNumber);
}

// Get the current gyro / accel / mag data from the sensor
int YEI3Space::getAllCorrectedComponentSensorData(float * gyro_rate3, float * accelerometer3, float * compass3) {
    float sensor_data9[9];
    int ret = writeRead(&simple_commands[TSS_GET_ALL_CORRECTED_COMPONENT_SENSOR_DATA], NULL, (char*)&sensor_data9);
    if( ret == 1 ){
        memcpy(gyro_rate3,      &sensor_data9[0],   sizeof(float)*3);
        memcpy(accelerometer3,  &sensor_data9[3],   sizeof(float)*3);
        memcpy(compass3,        &sensor_data9[6],   sizeof(float)*3);
    }
    return 1;
}

// Assign the streaming slots as defined in the constructor.
int YEI3Space::setupStreamSlots(int streamRate) {
    // Setup
    int i;
    unsigned char command_bytes[8];
    unsigned int rtn_data_detail_len;

    // Setup header information
    TSS_Protocol_Header_Setup wired_setup;
    wired_setup.protocol_byte = 0;
    wired_setup.protocol_bits.success_failure = 1;
    wired_setup.protocol_bits.timestamp = 0;
    wired_setup.protocol_bits.command_echo = 0;
    wired_setup.protocol_bits.checksum = 0;
    wired_setup.protocol_bits.logical_id = 0;
    wired_setup.protocol_bits.serial_number = 0;
    wired_setup.protocol_bits.data_length = 1;
    wired_setup.protocol_bits.pad = 0;

    if (!writeRead(&simple_commands[TSS_SET_WIRED_RESPONSE_HEADER_BITFIELD],(char*)&wired_setup.protocol_byte, NULL)) {
        printf("Error with setting header bitfield");
        return 1;
    }

    // Step through the stream commands and allocate the stream information
    stream_byte_len = 0;
    for( i=0; i < 8; i++){
        TSS_Command command = simple_commands[streamingSlots[i]];
        command_bytes[i] = command.command;
        stream_byte_len += command.rtn_data_len;
        rtn_data_detail_len = strlen(command.rtn_data_detail);

        //printf("current command detail %s\n", command.rtn_data_detail);
        if (i ==0) {
            strcpy(stream_parse_str, command.rtn_data_detail);
        } else {
            if((strlen(stream_parse_str)+rtn_data_detail_len) <= 256){
                strncat(stream_parse_str, command.rtn_data_detail, strlen(command.rtn_data_detail));
            } else{
                stream_byte_len = 0;
                printf("Error: Slot Buffer is full!!!\n");
                return 0;
            }
        }
    }
    last_stream_data = (float*)malloc(stream_byte_len * sizeof(float));

    // Write settings
    if (!writeRead(&simple_commands[TSS_SET_STREAMING_SLOTS], (char*)command_bytes, NULL)) {
        printf("Error writing the streaming slots");
        return 1;
    }

    // Set streaming timing configuration
    unsigned int interval = 1./streamRate * 1000000.; // microseconds between streaming packets
    // A duration of 0xffffffff will have the streaming session run till the stop stream command is called
    unsigned int duration = 0xffffffff;
    unsigned int delay = 10000; // microseconds until start streaming
    unsigned int args[3]= {interval,duration, delay};
    if (!writeRead(&simple_commands[TSS_SET_STREAMING_TIMING], (char*)args, NULL)) {
        printf("Error writing the streaming timing.");
        return 1;
    }


    tcflush(fd, TCIOFLUSH); // clear buffer
    return 0;
}

// Write to the device to start streaming then start background serial monitor thread
int YEI3Space::startStreaming(){
    //std::lock_guard<std::mutex> guard(mu);
    int ret;
    tcflush(fd, TCIOFLUSH); // clear buffer
    ret = writeRead(&simple_commands[TSS_START_STREAMING], NULL, NULL);
    if (ret == 1){
        // Start thread
        streamON = true;
        readThread = std::thread(&YEI3Space::streamThread, this);
        printf("Started stream\n");
    }
    return 1;
}

// Write false to the streamON and send a serial command to stop streams
int YEI3Space::stopStreaming(){
    if (streamON) {
        {
            {
                std::lock_guard<std::mutex> guard(mu);
                streamON = false;
            }
            if (readThread.joinable()) readThread.join();
        }

        printf("Closing stream\n");

        return writeRead(&simple_commands[TSS_STOP_STREAMING], NULL, NULL);
        tcflush(fd, TCIOFLUSH); // clear buffer
    }
}

// Function which will as a background thread until streamON is set to false.
int YEI3Space::streamThread() {
    printf("Running the stream thread\n");
    bool ok = true;
    sleep(0.1);

    while (ok) {
        ok = checkStream();
    }

    mu.lock();
    threadON = 0;
    mu.unlock();
    return 1;
}

// Take a reading on the serial port, assume control of the shared memory for the stream data packet and write
int YEI3Space::checkStream() {
    // Guard takes on the mutex for the class and locks the memory for the functions herein.
    // This memory is released when it goes out of scope (i.e. the function returns or fails.
    //printf("Getting data from stream\n");
    float * dat;
    int on;
    int ret = 0;

    dat = (float*)malloc(stream_byte_len * sizeof(float));

    {
        ret = readFile(stream_byte_len, stream_parse_str, (char*)dat);
    }

    if (ret == 1) {
        std::lock_guard<std::mutex> guard(mu);
        memcpy(last_stream_data,      dat,   sizeof(float)*stream_byte_len);
        free(dat);
        newData = true;
    } else {
        std::lock_guard<std::mutex> guard(mu);
        //streamON = false;
        newData = false;
        //printf("Stream did not match\n");
        free(dat);
    }
    on = streamON;
    return on;
}

// Get current stream value for an external user
// Copies the stream shared memory data and exports it
int YEI3Space::getStream(float * data) {
    // Guard takes on the mutex for the class and locks the memory for the functions herein.
    // This memory is released when it goes out of scope (i.e. the function returns or fails.
    {
        std::lock_guard<std::mutex> guard(mu);
        if (newData) {
            memcpy(data,      last_stream_data,   sizeof(float)*stream_byte_len);
            newData = false;
        } else {
            return 0;
        }
    }
    return 1;
}

// Take a parser descriptor, data, and length and perform endian swaps to fix for proper reading of data.
// data - container for serial data in and proper data out
// len  - length of the data
// parser_str - description of destination datatype
int YEI3Space::parseData( char* data, int len, const char* parser_str){
    int parser_index=0;
    int data_index=0;
    while(parser_str[parser_index]!= 0 ){
        switch(parser_str[parser_index]){
        case 'i': //int
        case 'I': //unsigned int
        case 'f': //float
            endian_swap_32((unsigned int *)&data[data_index]);
            data_index+=4;
            break;
        case 'h': //short
        case 'H': //unsigned short
            endian_swap_16((unsigned short *)&data[data_index]);
            data_index+=2;
            break;
        case 'b': //signed char
        case 'B': //unsigned char
        case '?': //bool
        case 's': //char[]
        case 'x': //pad byte
            //No swaps needed on single bytes
            data_index+=1;
            break;
        default:
            //should not get here
            printf("should not get here: %c\n",parser_str[parser_index]);
            break;
        }
        parser_index+=1;
    }
    if(data_index+1 == len){
        return 1;
    }
    return 0;
}

// This is a convenience function to calculate the last byte in the packet
// Commands without parameters can use the same number as the command
// \param command_bytes The address of the array to sum, this does not include the start byte
// \param num_bytes The number of bytes in the array Data + 1 for wired. Data + 2 for wireless
// \return checksum
unsigned char YEI3Space::createChecksum(const char * command_bytes, const unsigned int num_bytes)
{
    unsigned int chkSum = 0;
    unsigned int i;
    for (i = 0; i < num_bytes; i++){
        chkSum += command_bytes[i];
    }
    return (unsigned char)(chkSum % 256);
}

// Write and read from a 3-space device
int YEI3Space::writeRead(const TSS_Command * cmd_info, const char * input_data, char * output_data) {
    // Check to make sure the class has a valid device
    if ( fd < 0 )
        return 0;

    // Setup variables
    unsigned int write_size = cmd_info->in_data_len+3; //3 ={Start byte, Command byte, Checksum byte}
    unsigned int read_size =  cmd_info->rtn_data_len;
    char * write_array;
    int rv, wv;

    // Allocate memory for command write and assign the start byte + command
    write_array = (char*) malloc(write_size);
    if( write_array == NULL ){
        printf("ERROR: Out of memory\n");
        return 0;
    }
    memset((void *)write_array, 0, sizeof(write_array)); // clear the buffer out.

    if (cmd_info->command == simple_commands[TSS_START_STREAMING].command)
       write_array[0] = TSS_RESPONSE_HEADER_START_BYTE;
    else
        write_array[0] = TSS_START_BYTE;

    write_array[1] = cmd_info->command;

    if( cmd_info->in_data_len ){
        memcpy(write_array+2, input_data, cmd_info->in_data_len);
        parseData(write_array+2, cmd_info->in_data_len, cmd_info->in_data_detail);
    }
    write_array[write_size-1] = createChecksum(write_array+1, write_size-2);

    // Write the command to the device

    wv = write(fd, write_array, write_size);  //Send data
    if (wv == -1) {
        perror("Write Error: \n");
    } else if ( wv == 0 ) {
        perror("Nothing was written: \n");
    }

    free(write_array);

    // Initialize file descriptor sets
    fd_set read_fds, write_fds, except_fds;
    FD_ZERO(&read_fds);
    FD_ZERO(&write_fds);
    FD_ZERO(&except_fds);
    FD_SET(fd, &read_fds);

    // Set timeout to 1.0 seconds
    struct timeval timeout;
    timeout.tv_sec = 1;
    timeout.tv_usec = 0;

    // Wait for input to become ready or until the time out; the first parameter is
    // 1 more than the largest file descriptor in any of the sets
    if (cmd_info->rtn_data_len) {
        if (select(fd + 1, &read_fds, &write_fds, &except_fds, &timeout) == 1)
        {
            rv = read( fd, output_data, read_size ); // there was data to read
            if (rv == read_size) {
                parseData( output_data, read_size, cmd_info->rtn_data_detail);
            } else {
                perror("Missed tail end of a packet\n");
                return 0;
            }
            tcflush(fd, TCIOFLUSH); // clear buffer
        } else
        {
            perror("Timed out while waiting for response.\n");
            return 0;
        }
    }

    return 1;
}

//  19(0x13)
int YEI3Space::offsetWithCurrentOrientation() {
    return writeRead(&simple_commands[TSS_OFFSET_WITH_CURRENT_ORIENTATION],NULL,NULL);
}
//  20(0x14)
int YEI3Space::resetBaseOffset() {
    return writeRead(&simple_commands[TSS_RESET_BASE_OFFSET],NULL,NULL);
}
//  21(0x15)
int YEI3Space::offsetWithQuaternion(const float * quat4) {
    return writeRead(&simple_commands[TSS_OFFSET_WITH_QUATERNION],(char*)quat4,NULL);
}
//  22(0x16)
int YEI3Space::setBaseOffsetWithCurrentOrientation() {
    return writeRead(&simple_commands[TSS_SET_BASE_OFFSET_WITH_CURRENT_ORIENTATION],NULL,NULL);
}
//96(0x60)
int YEI3Space::tareWithCurrentOrientation() {
    return writeRead(&simple_commands[TSS_TARE_WITH_CURRENT_ORIENTATION], NULL, NULL);
}
//97(0x61)
int YEI3Space::tareWithQuaternion(const float * quat4) {
    return writeRead(&simple_commands[TSS_TARE_WITH_QUATERNION],(char *)quat4,NULL);
}
//98(0x62)
int YEI3Space::tareWithRotationMatrix(const float * matrix9) {
    return writeRead(&simple_commands[TSS_TARE_WITH_ROTATION_MATRIX],(char *)matrix9, NULL);
}
//116(0x74)
int YEI3Space::setAxisDirections(unsigned char axis_direction_byte) {
    return writeRead(&simple_commands[TSS_SET_AXIS_DIRECTIONS],(char *)&axis_direction_byte,NULL);
}
//120(0x78)
int YEI3Space::resetKalmanFilter() {
    return writeRead(&simple_commands[TSS_RESET_KALMAN_FILTER],NULL, NULL);
}
//121(0x79)
int YEI3Space::setAccelerometerRange(unsigned char accelerometer_range_setting) {
    return writeRead(&simple_commands[TSS_SET_ACCELEROMETER_RANGE],(char *)&accelerometer_range_setting,NULL);
}
//123(0x7b)
int YEI3Space::setFilterMode(unsigned char mode) {
    return writeRead(&simple_commands[TSS_SET_FILTER_MODE], (char *)&mode, NULL);
}
//221(0xdd)
int YEI3Space::tss_setWiredResponseHeaderBitfield(unsigned int header_bitfield){
    return writeRead(&simple_commands[TSS_SET_WIRED_RESPONSE_HEADER_BITFIELD], (char *)&header_bitfield, NULL);
}
//124(0x7c)
int YEI3Space::setRunningAverageMode(unsigned char mode) {
    return writeRead(&simple_commands[TSS_SET_RUNNING_AVERAGE_MODE],(char *)&mode,NULL);
}
//125(0x7d)
int YEI3Space::setGyroscopeRange(unsigned char gyroscope_range_setting) {
    return writeRead(&simple_commands[TSS_SET_GYROSCOPE_RANGE],(char *)&gyroscope_range_setting,NULL);
}
//126(0x7e)
int YEI3Space::setCompassRange(unsigned char compass_range_setting) {
    return writeRead(&simple_commands[TSS_SET_COMPASS_RANGE],(char *)&compass_range_setting,NULL);
}
//156(0x9c)
int YEI3Space::getEulerAngleDecompositionOrder(unsigned char * order) {
    return writeRead(&simple_commands[TSS_GET_EULER_ANGLE_DECOMPOSITION_ORDER],NULL,(char *)order);
}
//159(0x9f)
int YEI3Space::getOffsetOrientationAsQuaternion() {
    int ret;

    ret =  writeRead(&simple_commands[TSS_GET_OFFSET_ORIENTATION_AS_QUATERNION],NULL,(char *)offsetQ);
}

// Read from a threespace device serial port
// Only called when streaming
// Equivalent to parseWiredStreamData in API
int YEI3Space::readFile(unsigned int rtn_data_len, char *rtn_data_detail, char * output_data) {
    // Check to make sure the class has a valid device
    if ( fd < 0 ) {
        tcflush(fd, TCIOFLUSH); // clear buffer
        return 0;
    }
    if (!rtn_data_len) {
        tcflush(fd, TCIOFLUSH); // clear buffer
        return 0;
    }

    // Setup variables
    int header_size;
    char header_data[2];
    int num_bytes_read;
    unsigned int read_size =  stream_byte_len;

    header_size = 2; // We have asked for a 2-byte header
    header_data[0] = 1; // Should set to 0 on read if success

    // Initialize file descriptor sets
    fd_set read_fds;
    FD_ZERO(&read_fds);
    FD_SET(fd, &read_fds);

    // Set timeout to 1.0 seconds
    struct timeval timeout;
    timeout.tv_sec = 2;
    timeout.tv_usec = 0;
    if (!select(fd + 1, &read_fds, NULL, NULL, &timeout)) {
        perror("Could not select device!\n");
        tcflush(fd, TCIOFLUSH); // clear buffer
        return 0;
    }

    // Wait for input to become ready or until the time out; the first parameter is
    // 1 more than the largest file descriptor in any of the sets
    num_bytes_read = read( fd, header_data, header_size ); // there was data to read

    //printf("header data %d, %d\n", header_data[0], header_data[1]);
    if (!num_bytes_read) {
        printf("Error in reading stream.\n");
        tcflush(fd, TCIOFLUSH); // clear buffer
        return 0;
    }
    if (header_data[0] != 0) {
        printf("Success not true on read\n");
        tcflush(fd, TCIOFLUSH); // clear buffer
        return 0;
    }
    if (header_data[1] != stream_byte_len) {
        printf("Stream byte length does not match header!\n");
        tcflush(fd, TCIOFLUSH); // clear buffer
        return 0;
    }

    num_bytes_read = 0;
    num_bytes_read = read( fd, output_data, stream_byte_len ); // there was data to read
    if (num_bytes_read) {
        parseData( output_data, read_size, rtn_data_detail);
    } else {
        perror("Error in reading stream.\n");
        tcflush(fd, TCIOFLUSH); // clear buffer
        return 0;
    }
    return 1;

}


// The 3-Space sensors are Big Endian and x86 is Little Endian
// So the bytes need be swapped around, this function can convert from and
// to big endian
void YEI3Space::endian_swap_16(unsigned short * x)
{
    *x = (*x>>8) |
            (*x<<8);
}

void YEI3Space::endian_swap_32(unsigned int * x)
{
    *x = (*x>>24) |
            ((*x<<8) & 0x00FF0000) |
            ((*x>>8) & 0x0000FF00) |
            (*x<<24);
}


