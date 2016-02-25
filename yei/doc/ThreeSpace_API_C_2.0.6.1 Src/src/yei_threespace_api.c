/********************************************//**
 * \file yei_threespace_api.c
 * \brief  ThreeSpace API 2.0
 * \author Chris George
 * \author Daniel Morrison
 * \copyright Copyright 1998-2014, YEI Corporation.
 *
 * The YEI 3-Space C API is released under the YEI 3-Space Open Source License, which allows for both
 * non-commercial use and commercial use with certain restrictions.
 *
 * For Non-Commercial Use, your use of Covered Works is governed by the GNU GPL v.3, subject to the YEI 3-Space Open
 * Source Licensing Overview and Definitions.
 *
 * For Commercial Use, a YEI Commercial/Redistribution License is required, pursuant to the YEI 3-Space Open Source
 * Licensing Overview and Definitions. Commercial Use, for the purposes of this License, means the use, reproduction
 * and/or Distribution, either directly or indirectly, of the Covered Works or any portion thereof, or a Compilation,
 * Improvement, or Modification, for Pecuniary Gain. A YEI Commercial/Redistribution License may or may not require
 * payment, depending upon the intended use.
 *
 * Full details of the YEI 3-Space Open Source License can be found in license.txt
 * License also available online at http://www.yeitechnology.com/yei-3-space-open-source-license
 ***********************************************/
#include "yei_threespace_api.h"
#include "yei_threespace_core.h"

TSS_Sensor * sensor_list[64] = {0}; //makes these back into dynamic arrays TODO
unsigned int sensor_list_len = 0;
unsigned int sensor_list_counter = 0;

TSS_Dongle * dongle_list[64] = {0}; //makes these back into dynamic arrays TODO
unsigned int dongle_list_len = 0;
unsigned int dongle_list_counter = 0;

unsigned int wireless_retries = TSS_DEFAULT_WIRLESSS_RETRIES;
int default_baud_rate=TSS_DEFAULT_BAUD_RATE;

#ifdef _HEXDUMP
FILE * f_hex_dump;
#endif

// DLL Cleanup code
#ifndef TSS_STATIC_LIB
BOOL WINAPI DllMain(HINSTANCE dll_handle,   // handle to DLL module
                    DWORD reason,           // reason for calling function
                    LPVOID reserved_ptr)    // reserved
{
// Perform actions based on the reason for calling.
// unsigned int i;
    switch(reason){
        case DLL_PROCESS_ATTACH:
        // // Initialize once for each new process.
        // // Return FALSE to fail DLL load.
            tss_initThreeSpaceAPI();
            break;

        //case DLL_THREAD_ATTACH:
        // // Do thread-specific initialization.
        // break;

        //case DLL_THREAD_DETACH:
        // // Do thread-specific cleanup.
        // break;

        case DLL_PROCESS_DETACH:
            // Perform any necessary cleanup.
            tss_delThreeSpaceAPI();
            break;
    }
    return TRUE;    // Successful DLL_PROCESS_ATTACH.
}
#endif // TSS_STATIC_LIB

//convenience functions
void tss_parseAxisDirections(unsigned char axis_byte, TSS_Axis_Direction * axis_order, char * neg_x, char * neg_y, char * neg_z){
    *axis_order = axis_byte & 7;
    *neg_x = 0;
    *neg_y = 0;
    *neg_z = 0;
    if(axis_byte & 32){
        *neg_x = 1;
    }
    if(axis_byte & 16){
        *neg_y = 1;
    }
    if(axis_byte & 8){
        *neg_z = 1;
    }
}

unsigned char tss_generateAxisDirections(TSS_Axis_Direction axis_order, char neg_x, char neg_y, char neg_z){
    unsigned char direction_byte = axis_order;
    if(neg_x){
        direction_byte = direction_byte | 32;
    }
    if(neg_y){
        direction_byte = direction_byte | 16;
    }
    if(neg_z){
        direction_byte = direction_byte | 8;
    }
    return direction_byte;
}

//API specific functions
void tss_setSoftwareWirelessRetries(unsigned int retries){
    wireless_retries = retries;
}

unsigned int tss_getSoftwareWirelessRetries(){
    return wireless_retries;
}

void tss_setDefaultCreateDeviceBaudRate( int baud_rate){
    default_baud_rate = baud_rate;
}

int tss_getDefaultCreateDeviceBaudRate(){
    return default_baud_rate;
}

//tss_getComPorts is in yei_enum_ports.c

TSS_EXPORT TSS_Error tss_getTSDeviceInfoFromComPort(const char* com_port, TSS_ComInfo * com_info){
    int com_num;
    int device_serial = 0;
//    char min_firmware_version[] = {"17Mar2013A00"}; //absolute min version
    char com_port_fixed[16] = {0};
    HANDLE serial_port = INVALID_HANDLE_VALUE;
    DCB dcb_serial_params = {0};
    COMMTIMEOUTS timeouts = {0};
    if(!memcmp(com_port, "COM", 3)){
        com_num = atoi(com_port + 3);
        if(com_num > 8){
            strcpy(com_port_fixed, "\\\\.\\");
            strncat(com_port_fixed, com_port, 6);
        }
        else{
            strcpy(com_port_fixed, com_port);
        }
    }
    #ifdef DEBUG_PRINT
    printf("%s\n", com_port_fixed);
    #endif
    serial_port = CreateFileA(com_port_fixed,
                             GENERIC_READ | GENERIC_WRITE,
                             0, 0, OPEN_EXISTING, FILE_FLAG_OVERLAPPED, 0);
    if(serial_port == INVALID_HANDLE_VALUE){
        /*#ifdef DEBUG_PRINT
        // Retrieve the system error message for the last-error code
        LPVOID msg_buf;
        DWORD err_code = GetLastError();

        FormatMessageA(

            FORMAT_MESSAGE_ALLOCATE_BUFFER |
            FORMAT_MESSAGE_FROM_SYSTEM |
            FORMAT_MESSAGE_IGNORE_INSERTS,
            NULL,

            err_code,
            MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT),
            (LPSTR) &msg_buf,
            0, NULL );

        // Display the error message
        printf("Failed to create port handle ERROR[%d]: %s", err_code, (LPCSTR)msg_buf);

        // Free memory
        LocalFree(msg_buf);
        #endif*/
        #ifdef DEBUG_PRINT
        printf("INVALID_HANDLE_VALUE \n");
        #endif

        return TSS_ERROR_READ;
    }
    // Configure Serial Port

    dcb_serial_params.DCBlength = sizeof(dcb_serial_params);
    if (!GetCommState(serial_port, &dcb_serial_params)){
        // error getting state
        printf("We failed to get the previous state of the serial port :(\n");
    }
    dcb_serial_params.BaudRate = default_baud_rate;
    dcb_serial_params.ByteSize = 8;
    dcb_serial_params.StopBits = ONESTOPBIT;
    dcb_serial_params.Parity = NOPARITY;
    if(!SetCommState(serial_port, &dcb_serial_params)){
        // error setting serial port state
        printf("!Failed to set the serial port's state\n");
    }

    timeouts.ReadIntervalTimeout = -1;
    timeouts.ReadTotalTimeoutConstant = 100;
    timeouts.ReadTotalTimeoutMultiplier = 10;
    timeouts.WriteTotalTimeoutConstant = 100;
    timeouts.WriteTotalTimeoutMultiplier = 10;
    if(!SetCommTimeouts(serial_port, &timeouts)){
        // error occureed. Inform user
        printf("Well, we failed to set the timeouts :(\n");
    }
    f7WriteRead(serial_port, &simple_commands[TSS_STOP_STREAMING],NULL, NULL);
    Sleep(60);
    PurgeComm(serial_port,PURGE_RXCLEAR|PURGE_TXCLEAR);
    device_serial = 0xffffffff;
    f7WriteRead(serial_port, &simple_commands[TSS_GET_SERIAL_NUMBER], NULL, (char*)&device_serial);
    if(device_serial == 0xffffffff){
        //printf("Failed to get serial1!\n"); //this can happen on older firmware that doesn't support 2.0 commands retry fixes it
        f7WriteRead(serial_port, &simple_commands[TSS_GET_SERIAL_NUMBER], NULL, (char*)&device_serial);
            if(device_serial == 0xffffffff){
            #ifdef DEBUG_PRINT
            printf("Failed to get serial!\n");
            #endif
            CloseHandle(serial_port);
            return TSS_ERROR_READ;
        }
    }
    com_info->serial_number = device_serial;
    #ifdef DEBUG_PRINT
    printf("device_serial: %X\n", device_serial);
    #endif

    f7WriteRead(serial_port, &simple_commands[TSS_GET_FIRMWARE_VERSION_STRING], NULL, com_info->firmware_version);
    #ifdef DEBUG_PRINT
    printf("Firmware version=%s\n", firmware_version);
    #endif
    com_info->firmware_version[12]=0; //adding a null terminator to the string

    com_info->fw_compatibility  = getFirmwareCompatibility(com_info->firmware_version);

    f7WriteRead(serial_port, &simple_commands[TSS_GET_HARDWARE_VERSION_STRING], NULL, com_info->hardware_version);
    CloseHandle(serial_port);
    #ifdef DEBUG_PRINT
    printf("TSS_NO_DEVICE_ID %s\n", hardware_version);
    #endif
    com_info->hardware_version[32]=0;

    if(!memcmp(com_info->hardware_version + 4, "USB", 3) || !memcmp(com_info->hardware_version + 4, "MUSB", 4) || !memcmp(com_info->hardware_version + 4, "USBWT", 5)){
        com_info->device_type = TSS_USB;
    }
    else if(!memcmp(com_info->hardware_version + 4, "DNG", 3)){
        com_info->device_type = TSS_DNG;
    }
    else if(!memcmp(com_info->hardware_version + 4, "WL ", 3) || !memcmp(com_info->hardware_version + 4, "WL-", 3)){
        com_info->device_type = TSS_WL;
    }
    else if(!memcmp(com_info->hardware_version + 4, "EM ", 3) || !memcmp(com_info->hardware_version + 4, "EM-", 3)){
        com_info->device_type = TSS_EM;
    }
    else if(!memcmp(com_info->hardware_version + 4, "DL ", 3) || !memcmp(com_info->hardware_version + 4, "DL-", 3)){
        com_info->device_type = TSS_DL;
    }
    else if(!memcmp(com_info->hardware_version + 4, "BT ", 3) || !memcmp(com_info->hardware_version + 4, "BT-", 3)){
        com_info->device_type = TSS_BT;
    }
    else if(!memcmp(com_info->hardware_version + 4, "BTL", 3)){
        com_info->device_type = TSS_BTL;
    }
    else{
        //printf("TSS_NO_DEVICE_ID %s\n", com_info->hardware_version);
        com_info->device_type = TSS_UNKNOWN;
    }
    return TSS_NO_ERROR;
}

TSS_EXPORT TSS_Error tss_getTSDeviceInfo(TSS_Device_Id device, TSS_ComInfo * com_info){
    TSS_Error error;
    unsigned int tss_idx;
    HANDLE serial_port;
    TSS_Firmware_Compatibility fw_compatibility;
    error= TSS_ERROR_COMMAND_FAIL;
    fw_compatibility = TSS_FW_NOT_COMPATIBLE;
    serial_port = INVALID_HANDLE_VALUE;
    tss_idx = ~(~device|TSS_NO_DONGLE_ID);
    if(tss_idx < sensor_list_len && sensor_list[tss_idx]){
        fw_compatibility = sensor_list[tss_idx]->fw_compatibility;
        com_info->fw_compatibility = fw_compatibility;
        com_info->device_type = sensor_list[tss_idx]->device_type;
        serial_port = sensor_list[tss_idx]->serial_port;
    }
    else if(device & TSS_DONGLE_ID){
        tss_idx = (device - TSS_DONGLE_ID);
        if(tss_idx < dongle_list_len && dongle_list[tss_idx]){
            fw_compatibility = dongle_list[tss_idx]->fw_compatibility;
            com_info->fw_compatibility = fw_compatibility;
            com_info->device_type = dongle_list[tss_idx]->device_type;
            serial_port = dongle_list[tss_idx]->serial_port;
        }
    }
    else{
        return TSS_INVALID_ID;
    }
    if(fw_compatibility == TSS_FW_NOT_COMPATIBLE){
        int device_serial = 0;
        f7WriteRead(serial_port, &simple_commands[TSS_GET_SERIAL_NUMBER], NULL, (char*)&device_serial);
        if(device_serial == 0xffffffff){
            return TSS_ERROR_READ;
        }
        com_info->serial_number = device_serial;

        f7WriteRead(serial_port, &simple_commands[TSS_GET_FIRMWARE_VERSION_STRING], NULL, com_info->firmware_version);
        com_info->firmware_version[12]=0; //adding a null terminator to the string

        f7WriteRead(serial_port, &simple_commands[TSS_GET_HARDWARE_VERSION_STRING], NULL, com_info->hardware_version);
        com_info->hardware_version[32]=0;
        return TSS_NO_ERROR;
    }
    else{
        error =tss_getSerialNumber(device,&com_info->serial_number,NULL);
        if(error){
            return error;
        }
        error =tss_getFirmwareVersionString(device,com_info->firmware_version,NULL);
        if(error){
            return error;
        }
        error =tss_getHardwareVersionString(device,com_info->hardware_version,NULL);
        if(error){
            return error;
        }
        error =tss_getTSDeviceType(device,&com_info->device_type);
        if(error){
            return error;
        }
    }

    return error;
}

TSS_Device_Id tss_createTSDeviceU16Str(const wchar_t * com_port, TSS_Timestamp_Mode mode){
    char corrected_com_port[32] = {0};
    size_t wcsChars;
    wcsChars = wcslen(com_port);
    WideCharToMultiByte(CP_UTF8, 0, com_port, wcsChars, corrected_com_port, sizeof(corrected_com_port), NULL, 0);
    return tss_createTSDeviceStr(corrected_com_port, mode);
}

TSS_Device_Id tss_createTSDeviceU16StrEX(const wchar_t * com_port, int baud_rate, TSS_Timestamp_Mode mode){
    char corrected_com_port[32] = {0};
    size_t wcsChars;
    wcsChars = wcslen(com_port);
    WideCharToMultiByte(CP_UTF8, 0, com_port, wcsChars, corrected_com_port, sizeof(corrected_com_port), NULL, 0);
    return tss_createTSDeviceStrEx(corrected_com_port, baud_rate, mode);
}

TSS_Device_Id tss_createTSDeviceStr(const char * com_port, TSS_Timestamp_Mode mode){
    return tss_createTSDeviceStrEx(com_port, 115200, mode);
}

TSS_Device_Id tss_createTSDeviceStrEx(const char* com_port, int baud_rate, TSS_Timestamp_Mode mode){
    int com_num;
    TSS_Sensor * new_sensor;
    int device_serial = 0;
    char hardware_version[33] = {0};
    char firmware_version[13] = {0};
//    char min_firmware_version[] = {"17Mar2013A00"}; //absolute min version
//    char min_firmware_version[] = {"25Apr2013A00"};
    TSS_Type device_type;
    char com_port_fixed[16] = {0};
    TSS_Protocol_Header_Setup header_setup;
    HANDLE serial_port = INVALID_HANDLE_VALUE;
    DCB dcb_serial_params = {0};
    COMMTIMEOUTS timeouts = {0};
    TSS_Protocol_Header_Setup wired_setup;
    TSS_Firmware_Compatibility fw_version;
    if(!memcmp(com_port, "COM", 3)){
        com_num = atoi(com_port + 3);
        if(com_num > 8){
            strcpy(com_port_fixed, "\\\\.\\");
            strncat(com_port_fixed, com_port, 6);
        }
        else{
            strcpy(com_port_fixed, com_port);
        }
    }
    #ifdef DEBUG_PRINT
    printf("%s\n", com_port_fixed);
    #endif
    serial_port = CreateFileA(com_port_fixed,
                             GENERIC_READ | GENERIC_WRITE,
                             0, 0, OPEN_EXISTING, FILE_FLAG_OVERLAPPED, 0);
    if(serial_port == INVALID_HANDLE_VALUE){
        /*#ifdef DEBUG_PRINT
        // Retrieve the system error message for the last-error code
        LPVOID msg_buf;
        DWORD err_code = GetLastError();

        FormatMessageA(

            FORMAT_MESSAGE_ALLOCATE_BUFFER |
            FORMAT_MESSAGE_FROM_SYSTEM |
            FORMAT_MESSAGE_IGNORE_INSERTS,
            NULL,

            err_code,
            MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT),
            (LPSTR) &msg_buf,
            0, NULL );

        // Display the error message
        printf("Failed to create port handle ERROR[%d]: %s", err_code, (LPCSTR)msg_buf);

        // Free memory
        LocalFree(msg_buf);
        #endif*/
        printf("INVALID_HANDLE_VALUE \n");

        return TSS_NO_DEVICE_ID;
    }
    // Configure Serial Port

    switch(baud_rate){
        case 1200:
        case 2400:
        case 4800:
        case 9600:
        case 19200:
        case 28800:
        case 38400:
        case 57600:
        case 115200:
        case 230400:
        case 460800:
        case 921600:
            break;
        default:
            baud_rate = default_baud_rate;
    };

    dcb_serial_params.DCBlength = sizeof(dcb_serial_params);
    if (!GetCommState(serial_port, &dcb_serial_params)){
        // error getting state
        printf("We failed to get the previous state of the serial port :(\n");
    }
    dcb_serial_params.BaudRate = baud_rate;
    dcb_serial_params.ByteSize = 8;
    dcb_serial_params.StopBits = ONESTOPBIT;
    dcb_serial_params.Parity = NOPARITY;
    dcb_serial_params.XonChar = 17;  //might fixes cases of bad configurations of ports because XonChar==XoffChar?
    dcb_serial_params.XoffChar = 19; //might fixes cases of bad configurations of ports because XonChar==XoffChar?
    if(!SetCommState(serial_port, &dcb_serial_params)){
        // error setting serial port state
        printf("!Failed to set the serial port's state\n");
    }

    timeouts.ReadIntervalTimeout = -1;
    timeouts.ReadTotalTimeoutConstant = 200;
    timeouts.ReadTotalTimeoutMultiplier = 10;
    timeouts.WriteTotalTimeoutConstant = 200;
    timeouts.WriteTotalTimeoutMultiplier = 10;
    if(!SetCommTimeouts(serial_port, &timeouts)){
        // error occureed. Inform user
        printf("Well, we failed to set the timeouts :(\n");
    }
    f7WriteRead(serial_port, &simple_commands[TSS_STOP_STREAMING],NULL, NULL);
    Sleep(60);
    PurgeComm(serial_port,PURGE_RXCLEAR|PURGE_TXCLEAR);
    device_serial = 0xffffffff;
    f7WriteRead(serial_port, &simple_commands[TSS_GET_SERIAL_NUMBER], NULL, (char*)&device_serial);
    if(device_serial == 0xffffffff){
        //printf("Failed to get serial1!\n"); //this can happen on older firmware that doesn't support 2.0 commands retry fixes it
        f7WriteRead(serial_port, &simple_commands[TSS_GET_SERIAL_NUMBER], NULL, (char*)&device_serial);
        if(device_serial == 0xffffffff){
            printf("Failed to get serial2!\n");
            CloseHandle(serial_port);
            return TSS_NO_DEVICE_ID;
        }
    }
    #ifdef DEBUG_PRINT
    printf("device_serial: %X\n", device_serial);
    #endif

    f7WriteRead(serial_port, &simple_commands[TSS_GET_FIRMWARE_VERSION_STRING], NULL, firmware_version);
    #ifdef DEBUG_PRINT
    printf("Firmware version=%s\n", firmware_version);
    #endif
//    if(compareFirmwareVersion(firmware_version, min_firmware_version) < 0){
//        printf("Incompatible firmware version detected!!\n");
//        CloseHandle(serial_port);
//        return TSS_NO_DEVICE_ID;
//    }

    fw_version = getFirmwareCompatibility(firmware_version);


    f7WriteRead(serial_port, &simple_commands[TSS_GET_HARDWARE_VERSION_STRING], NULL, hardware_version);
    #ifdef DEBUG_PRINT
    printf("TSS_NO_DEVICE_ID %s\n", hardware_version);
    #endif
    if(!memcmp(hardware_version + 4, "USB", 3) || !memcmp(hardware_version + 4, "MUSB", 4) || !memcmp(hardware_version + 4, "USBWT", 5)){
        device_type = TSS_USB;
    }
    else if(!memcmp(hardware_version + 4, "DNG", 3)){
        device_type = TSS_DNG;
    }
    else if(!memcmp(hardware_version + 4, "WL ", 3) || !memcmp(hardware_version + 4, "WL-", 3)){
        device_type = TSS_WL;
    }
    else if(!memcmp(hardware_version + 4, "EM ", 3) || !memcmp(hardware_version + 4, "EM-", 3)){
        device_type = TSS_EM;

    }
    else if(!memcmp(hardware_version + 4, "DL ", 3) || !memcmp(hardware_version + 4, "DL-", 3)){
        device_type = TSS_DL;
    }
    else if(!memcmp(hardware_version + 4, "BT ", 3) || !memcmp(hardware_version + 4, "BT-", 3)){
        device_type = TSS_BT;
    }
    else if(!memcmp(hardware_version + 4, "BTL", 3)){
        device_type = TSS_BTL;
    }
    else{
        printf("TSS_NO_DEVICE_ID %s\n", hardware_version);
        CloseHandle(serial_port);
        return TSS_NO_DEVICE_ID;
    }

    #ifdef DEBUG_PRINT
    printf("TSS_DEVICE_TYPE: %d\n", device_type);
    #endif

    if(device_type == TSS_DNG){
        unsigned short reception_bitfield=0;
        char i,j;
        TSS_Dongle * new_dongle;
        int autoflush;
        #ifdef DEBUG_PRINT
        COMSTAT com_stat;
        #endif
        f7WriteRead(serial_port,&simple_commands[TSS_GET_RECEPTION_BITFIELD],NULL, (char*)&reception_bitfield );
        #ifdef DEBUG_PRINT
        printf("reception_bitfield %X\n",reception_bitfield);
        #endif
        for( i = 0; i<15; i++){
            if( (reception_bitfield >> i) & 1){
                TSS_Error error;
                for( j=0; j < 15 ; j++){
                    error = f8WriteRead(serial_port, i, &simple_commands[TSS_STOP_STREAMING], NULL, NULL);
                    if(error == TSS_NO_ERROR){
                        #ifdef DEBUG_PRINT
                        printf("stopped sensor, %d\n",i);
                        #endif
                        break;
                    }
                }
            }
        }
        #ifdef DEBUG_PRINT
        f7WriteRead(serial_port,&simple_commands[TSS_GET_RECEPTION_BITFIELD],NULL, (char*)&reception_bitfield );
        ClearCommError(serial_port, NULL,  &com_stat);
        printf("cbInQue: %d\n",(int)com_stat.cbInQue);
        printf("Creating a Dongle\n");
        #endif
//        int i;
        new_dongle = (TSS_Dongle*)malloc(sizeof(TSS_Dongle));
        if(new_dongle == 0){printf("ERROR: Out of memory\n"); return 0;}
        new_dongle->device_type = device_type;
        new_dongle->fw_compatibility = fw_version;
        new_dongle->device_serial = device_serial;
        new_dongle->baudrate = 115200;
        //new_dongle->w_sensors = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
        memset(new_dongle->w_sensors, 0, sizeof(new_dongle->w_sensors));
        memset(new_dongle->protocol_header, 0, sizeof(new_dongle->protocol_header));
        new_dongle->last_stream_data = NULL;
        new_dongle->timestamp_mode = mode;
        //Thread stuff
        new_dongle->is_active = 1;
        new_dongle->reader_event = CreateEvent(NULL, FALSE, FALSE, NULL);
        new_dongle->writer_event = CreateEvent(NULL, FALSE, FALSE, NULL);
        InitializeCriticalSection(&new_dongle->reader_lock);
        InitializeCriticalSection(&new_dongle->stream_lock);
        new_dongle->serial_port = serial_port;
        new_dongle->reader_thread = INVALID_HANDLE_VALUE;

        wired_setup.protocol_byte = 0;
        if(fw_version > TSS_FW_NOT_COMPATIBLE){
            wired_setup.protocol_bits.success_failure = 1;
            wired_setup.protocol_bits.logical_id = 1;
            wired_setup.protocol_bits.command_echo = 1;
            wired_setup.protocol_bits.data_length = 1;
            if(mode == TSS_TIMESTAMP_SENSOR){
                 wired_setup.protocol_bits.timestamp = 1;
            }
            header_setup.protocol_byte = 0;
            f7WriteRead(serial_port, &simple_commands[TSS_GET_WIRED_RESPONSE_HEADER_BITFIELD], NULL, (char*)&header_setup.protocol_byte);
            #ifdef DEBUG_PRINT
            printf("Header Protocol: %u\n", header_setup.protocol_byte);
            #endif
            if(header_setup.protocol_byte != wired_setup.protocol_byte){
                f7WriteRead(serial_port, &simple_commands[TSS_SET_WIRED_RESPONSE_HEADER_BITFIELD], (char*)&wired_setup.protocol_byte, NULL);
                f7WriteRead(serial_port, &simple_commands[TSS_GET_WIRED_RESPONSE_HEADER_BITFIELD], NULL, (char*)&header_setup.protocol_byte);
                if(header_setup.protocol_byte != wired_setup.protocol_byte){
                    printf("Error header protocol didnt set!!!\n");
                    free(new_dongle);
                    return TSS_NO_DEVICE_ID;
                }
            }
            header_setup.protocol_byte = 0;
            f7WriteRead(serial_port, &simple_commands[TSS_GET_WIRELESS_RESPONSE_HEADER_BITFIELD], NULL, (char*)&header_setup.protocol_byte);
            #ifdef DEBUG_PRINT
            printf("Header Wireless Protocol: %u\n", header_setup.protocol_byte);
            #endif
            if(header_setup.protocol_byte != wired_setup.protocol_byte){
                f7WriteRead(serial_port, &simple_commands[TSS_SET_WIRELESS_RESPONSE_HEADER_BITFIELD], (char*)&wired_setup.protocol_byte, NULL);
                f7WriteRead(serial_port, &simple_commands[TSS_GET_WIRELESS_RESPONSE_HEADER_BITFIELD], NULL, (char*)&header_setup.protocol_byte);
                if(header_setup.protocol_byte != wired_setup.protocol_byte){
                    printf("Error header protocol didnt set!!!\n");
                    free(new_dongle);
                    return TSS_NO_DEVICE_ID;
                }
            }

            autoflush = 1;
            f7WriteRead(serial_port, &simple_commands[TSS_SET_WIRELESS_STREAMING_AUTO_FLUSH_MODE], (char*)&autoflush, NULL);
            f7WriteRead(serial_port, &simple_commands[TSS_START_STREAMING], NULL, NULL);

    //        TSS_Stream_Args reader = {serial_port, NULL, 0, NULL};
            new_dongle->reader_thread = (HANDLE)_beginthreadex(NULL, 0, _serialReadLoopDongle, new_dongle, 0, NULL);
        }
        if( dongle_list_len < dongle_list_counter + 1){
            //dongle_list = realloc(dongle_list, sizeof(TSS_Dongle *) *(dongle_list_counter + 1));
            dongle_list_len = dongle_list_counter + 1;
        }
        dongle_list[dongle_list_counter] = new_dongle;
        dongle_list_counter++;
        new_dongle->device_id = (TSS_DONGLE_ID + (dongle_list_counter - 1));
        return new_dongle->device_id;
    }
    #ifdef DEBUG_PRINT
    printf("Creating a Sensor\n");
    #endif
    new_sensor = (TSS_Sensor*)malloc(sizeof(TSS_Sensor));
    if(new_sensor == 0){printf("ERROR: Out of memory\n"); return 0;}
    new_sensor->device_type = device_type;
    new_sensor->fw_compatibility = fw_version;
    new_sensor->device_serial = device_serial;
    new_sensor->baudrate = default_baud_rate;
    new_sensor->logical_id = 0;
    new_sensor->dongle = TSS_NO_DEVICE_ID;
    memset(new_sensor->protocol_header, 0, sizeof(new_sensor->protocol_header));
    new_sensor->stream_slots[0] = TSS_NULL;
    new_sensor->stream_slots[1] = TSS_NULL;
    new_sensor->stream_slots[2] = TSS_NULL;
    new_sensor->stream_slots[3] = TSS_NULL;
    new_sensor->stream_slots[4] = TSS_NULL;
    new_sensor->stream_slots[5] = TSS_NULL;
    new_sensor->stream_slots[6] = TSS_NULL;
    new_sensor->stream_slots[7] = TSS_NULL;
    memset(new_sensor->stream_parse_str, 0, sizeof(new_sensor->stream_parse_str));
    new_sensor->stream_byte_len = 0;
    new_sensor->last_stream_data = NULL;
    new_sensor->record_count = 0;
    new_sensor->timestamp_mode = mode;
    new_sensor->stream_enabled = 0;
    //Thread stuff
    new_sensor->is_active = 1;
    new_sensor->new_data_event = CreateEvent(NULL, TRUE, FALSE, NULL);
    new_sensor->reader_event = CreateEvent(NULL, FALSE, FALSE, NULL);
    new_sensor->writer_event = CreateEvent(NULL, FALSE, FALSE, NULL);
    InitializeCriticalSection(&new_sensor->reader_lock);
    InitializeCriticalSection(&new_sensor->stream_lock);
    new_sensor->serial_port = serial_port;
    new_sensor->callback = NULL;
    new_sensor->reader_thread = INVALID_HANDLE_VALUE;

    wired_setup.protocol_byte = 0;
    if(fw_version > TSS_FW_NOT_COMPATIBLE){
        wired_setup.protocol_bits.success_failure = 1;
        wired_setup.protocol_bits.command_echo = 1;
        wired_setup.protocol_bits.data_length = 1;
        if(mode == TSS_TIMESTAMP_SENSOR){
             wired_setup.protocol_bits.timestamp = 1;
        }
        header_setup.protocol_byte = 0;
        f7WriteRead(serial_port, &simple_commands[TSS_GET_WIRED_RESPONSE_HEADER_BITFIELD], NULL, (char*)&header_setup.protocol_byte);
        #ifdef DEBUG_PRINT
        printf("Header Protocol: %u\n", header_setup.protocol_byte);
        #endif
        if(header_setup.protocol_byte != wired_setup.protocol_byte){
            f7WriteRead(serial_port, &simple_commands[TSS_SET_WIRED_RESPONSE_HEADER_BITFIELD], (char*)&wired_setup.protocol_byte, NULL);
            f7WriteRead(serial_port, &simple_commands[TSS_GET_WIRED_RESPONSE_HEADER_BITFIELD], NULL, (char*)&header_setup.protocol_byte);
            if(header_setup.protocol_byte != wired_setup.protocol_byte){
                printf("Error header protocol didnt set!!!\n");
                free(new_sensor);
                return TSS_NO_DEVICE_ID;
            }
        }
    //        TSS_Stream_Args reader = {serial_port, NULL, 0, NULL};
        new_sensor->reader_thread = (HANDLE)_beginthreadex(NULL, 0, _serialReadLoop, new_sensor, 0, NULL);
    }
    if( sensor_list_len < sensor_list_counter + 1){
        //sensor_list = realloc(sensor_list, sizeof(TSS_Sensor *) * (sensor_list_counter + 1));
        sensor_list_len = sensor_list_counter + 1;
    }
    sensor_list[sensor_list_counter] = new_sensor;
    sensor_list_counter++;
    //get serial data

    switch(device_type){
    case TSS_BTL:
        new_sensor->device_id = (TSS_BOOTLOADER_ID + (sensor_list_counter - 1));
        break;
    case TSS_USB:
        new_sensor->device_id = (TSS_USB_ID + (sensor_list_counter - 1));
        break;
    case TSS_DNG:
        new_sensor->device_id = (TSS_DONGLE_ID + (dongle_list_counter - 1));
        break;
    case TSS_WL:
        new_sensor->device_id = (TSS_WIRELESS_ID + (sensor_list_counter - 1));
        break;
    case TSS_WL_W:
        new_sensor->device_id = (TSS_WIRELESS_W_ID + (sensor_list_counter - 1));
        break;
    case TSS_EM:
        new_sensor->device_id = (TSS_EMBEDDED_ID + (sensor_list_counter - 1));
        break;
    case TSS_DL:
        new_sensor->device_id = (TSS_DATALOGGER_ID + (sensor_list_counter - 1));
        break;
    case TSS_BT:
        new_sensor->device_id = (TSS_BLUETOOTH_ID + (sensor_list_counter - 1));
        break;
    case TSS_UNKNOWN:
        printf("Somehow a device without a known type was made?");
        //free(new_sensor);
        CloseHandle(serial_port);
        return TSS_NO_DEVICE_ID;
    }
    return new_sensor->device_id;
}

TSS_Error tss_closeTSDevice(TSS_Device_Id device){
    unsigned int tss_idx = ~(~device|TSS_NO_DONGLE_ID);
    if(tss_idx < sensor_list_len && sensor_list[tss_idx]){
        if(sensor_list[tss_idx]->stream_enabled){
            int j;
            for(j=0 ; j<5; j++){
                if(!tss_stopStreaming(device, NULL)){
                    break;
                }
            }
        }
        sensor_list[tss_idx]->is_active= 0;
        if(sensor_list[tss_idx]->serial_port){
            f9Write(sensor_list[tss_idx]->serial_port,&simple_commands[TSS_STOP_STREAMING],NULL);
            SetEvent(sensor_list[tss_idx]->reader_event);
        }
        CloseHandle(sensor_list[tss_idx]->serial_port);
        WaitForSingleObject(sensor_list[tss_idx]->reader_thread, 510);
        free(sensor_list[tss_idx]->last_stream_data);
        #ifdef DEBUG_PRINT
        printf("closing handle\n");
        #endif
        CloseHandle(sensor_list[tss_idx]->new_data_event);
        if(sensor_list[tss_idx]->device_type != TSS_WL_W){
            CloseHandle(sensor_list[tss_idx]->reader_event);
            CloseHandle(sensor_list[tss_idx]->writer_event);
            DeleteCriticalSection(&sensor_list[tss_idx]->reader_lock);
        }
        else{ //clean up wireless and remove the sensor from the donglelist
            unsigned int tss_d_idx = (sensor_list[tss_idx]->dongle - TSS_DONGLE_ID);
            dongle_list[tss_d_idx]->w_sensors[(int)sensor_list[tss_idx]->logical_id] = 0;
        }
        DeleteCriticalSection(&sensor_list[tss_idx]->stream_lock);
        //Free memory
        free(sensor_list[tss_idx]);
        sensor_list[tss_idx] = NULL;
        return TSS_NO_ERROR;
    }
    if(device&TSS_DONGLE_ID){
        unsigned short reception_bitfield=0;
        char k,j;
        tss_idx = (device-TSS_DONGLE_ID);
        if(tss_idx < dongle_list_len && dongle_list[tss_idx]){
            for( k = 0; k<15; k++){
                tss_closeTSDevice(dongle_list[tss_idx]->w_sensors[(int)k]);
            }
            dongle_list[tss_idx]->is_active= 0;
            f9Write(dongle_list[tss_idx]->serial_port,&simple_commands[TSS_STOP_STREAMING],NULL);
            SetEvent(dongle_list[tss_idx]->reader_event);

            WaitForSingleObject(dongle_list[tss_idx]->reader_thread, 510);

            f7WriteRead(dongle_list[tss_idx]->serial_port, &simple_commands[TSS_STOP_STREAMING],NULL, NULL);
            Sleep(60);
            PurgeComm(dongle_list[tss_idx]->serial_port,PURGE_RXCLEAR|PURGE_TXCLEAR);

            f7WriteRead(dongle_list[tss_idx]->serial_port,&simple_commands[TSS_GET_RECEPTION_BITFIELD],NULL, (char*)&reception_bitfield );
            #ifdef DEBUG_PRINT
            printf("reception_bitfield %X\n",reception_bitfield);
            #endif
            for( k = 0; k<15; k++){
                if( (reception_bitfield >> k) & 1){
                    #ifdef DEBUG_PRINT
                    printf("Logical_id: %d\n", k);
                    #endif
                    TSS_Error error;
                    for( j=0; j < 5 ; j++){
                        #ifdef DEBUG_PRINT
                        printf("tries:%d\n", j);
                        #endif
                        error = f8WriteRead(dongle_list[tss_idx]->serial_port, k, &simple_commands[TSS_STOP_STREAMING], NULL, NULL);
                        if(error == TSS_NO_ERROR){
                            #ifdef DEBUG_PRINT
                            printf("stopped sensor, %d\n",i);
                            #endif
                            break;
                        }
                    }
                }
            }

            free(dongle_list[tss_idx]->last_stream_data);
            #ifdef DEBUG_PRINT
            printf("closing handle dongle\n");
            #endif
            CloseHandle(dongle_list[tss_idx]->serial_port);
            CloseHandle(dongle_list[tss_idx]->reader_event);
            CloseHandle(dongle_list[tss_idx]->writer_event);
            DeleteCriticalSection(&dongle_list[tss_idx]->reader_lock);
            DeleteCriticalSection(&dongle_list[tss_idx]->stream_lock);
            //Free memory
            free(dongle_list[tss_idx]);
            dongle_list[tss_idx] = NULL;
            return TSS_NO_ERROR;
        }
    }
    return TSS_INVALID_ID;
}

TSS_Error tss_getSensorFromDongle(TSS_Device_Id device, int logical_id, unsigned int * w_ts_device){
    unsigned int tss_idx = 0;
    tss_idx = (device - TSS_DONGLE_ID);
    if(tss_idx < dongle_list_len && dongle_list[tss_idx]){
        TSS_Dongle * dongle  = dongle_list[tss_idx];
        if( logical_id < 0 || logical_id > 14){
            return TSS_ERROR_PARAMETER;
        }
        if(dongle->w_sensors[logical_id] == 0 || dongle->w_sensors[logical_id] == TSS_NO_DEVICE_ID ){
            int w_serial_number=0;
            f9WriteReadDongle(dongle, &simple_commands[TSS_GET_SERIAL_NUMBER_AT_LOGICAL_ID], (char*)&logical_id, (char *)&w_serial_number, NULL);
            if(w_serial_number != 0){
                #ifdef DEBUG_PRINT
                printf("Device serial from dong: %X\n",w_serial_number);
                #endif
                *w_ts_device = _createTSSWirelessSensor(device, logical_id);
                dongle->w_sensors[logical_id] = *w_ts_device;
            }
            else{
                *w_ts_device = TSS_NO_DEVICE_ID;
                dongle->w_sensors[logical_id] = *w_ts_device;
            }
        }
        *w_ts_device = dongle->w_sensors[logical_id];
    }
    if(*w_ts_device == TSS_NO_DEVICE_ID){
        return TSS_ERROR_COMMAND_FAIL;
    }
    return TSS_NO_ERROR;
}

TSS_Error tss_setSensorToDongle(TSS_Device_Id device, int logical_id, unsigned int w_serial_number){
    unsigned int tss_idx = 0;
    tss_idx = (device - TSS_DONGLE_ID);
    if(tss_idx < dongle_list_len && dongle_list[tss_idx]){
        unsigned int w_ts_device;
        TSS_Dongle * dongle = dongle_list[tss_idx];
        char id_serial_number_list[5] = {0};
        memcpy(id_serial_number_list, (char*)&logical_id, 1);
        memcpy(id_serial_number_list + 1, (char*)&w_serial_number, 4);
        f9WriteReadDongle(dongle, &simple_commands[TSS_SET_SERIAL_NUMBER_AT_LOGICAL_ID], id_serial_number_list, NULL, NULL);
        w_ts_device = dongle->w_sensors[logical_id];
        if(w_ts_device == 0 || w_ts_device == TSS_NO_DEVICE_ID){
            w_ts_device = _createTSSWirelessSensor(device, logical_id);
            dongle->w_sensors[logical_id] = w_ts_device;
        }
        else{
            tss_idx = (w_ts_device - TSS_WIRELESS_W_ID);
            sensor_list[tss_idx]->device_serial = w_serial_number;
        }
    }
    return TSS_NO_ERROR;
}

TSS_EXPORT TSS_Error tss_setTimestampMode(TSS_Device_Id device, TSS_Timestamp_Mode mode){
    TSS_Error error;
    unsigned int tss_idx = ~(~device|TSS_NO_DONGLE_ID);
    if(device & TSS_WIRELESS_W_ID){
        return TSS_INVALID_COMMAND;
    }
    if(tss_idx < sensor_list_len && sensor_list[tss_idx]){
        sensor_list[tss_idx]->timestamp_mode = mode;
        return TSS_NO_ERROR;
    }
    else if(device & TSS_DONGLE_ID) {
        tss_idx = (device - TSS_DONGLE_ID);
        if(tss_idx < dongle_list_len && dongle_list[tss_idx]){
            TSS_Dongle * dongle = dongle_list[tss_idx];
            if (mode == TSS_TIMESTAMP_SENSOR && dongle->timestamp_mode != TSS_TIMESTAMP_SENSOR){
                TSS_Protocol_Header_Setup wired_setup;
                unsigned int timestamp;
                wired_setup.protocol_byte = 0;
                wired_setup.protocol_bits.success_failure = 1;
                wired_setup.protocol_bits.logical_id = 1;
                wired_setup.protocol_bits.command_echo = 1;
                wired_setup.protocol_bits.data_length = 1;
                wired_setup.protocol_bits.timestamp = 1;
                error = tss_setWirelessStreamingAutoFlushMode(device, 0 ,NULL); //turning off stream, if its streaming
                if(error){
                EnterCriticalSection(&dongle->reader_lock);
                    return error;
                }
                dongle->timestamp_mode = mode;
                f7WriteRead(dongle->serial_port, &simple_commands[TSS_SET_WIRED_RESPONSE_HEADER_BITFIELD], (char*)&wired_setup.protocol_byte, NULL);
                f7WriteRead(dongle->serial_port, &simple_commands[TSS_SET_WIRELESS_RESPONSE_HEADER_BITFIELD], (char*)&wired_setup.protocol_byte, NULL);
                Sleep(600); //until i figure another way
                LeaveCriticalSection(&dongle->reader_lock);
                error = tss_setWirelessStreamingAutoFlushMode(device, 1, &timestamp); //turning off stream, if its streaming
                if(error){
                    return error;
                }
                #ifdef DEBUG_PRINT
                printf("Timestamp reading %u\n", timestamp);
                #endif
            }
            else if((mode == TSS_TIMESTAMP_SYSTEM || mode == TSS_TIMESTAMP_NONE) && (dongle->timestamp_mode != TSS_TIMESTAMP_SYSTEM || dongle->timestamp_mode != TSS_TIMESTAMP_NONE)){
                TSS_Protocol_Header_Setup wired_setup;
                unsigned int timestamp;
                wired_setup.protocol_byte = 0;
                wired_setup.protocol_bits.success_failure = 1;
                wired_setup.protocol_bits.logical_id = 1;
                wired_setup.protocol_bits.command_echo = 1;
                wired_setup.protocol_bits.data_length = 1;
                wired_setup.protocol_bits.timestamp = 0;
                error = tss_setWirelessStreamingAutoFlushMode(device, 0, NULL); //turning off stream, if its streaming
                if(error){
                    return error;
                }
                EnterCriticalSection(&dongle->reader_lock);
                dongle->timestamp_mode = mode;
                f7WriteRead(dongle->serial_port, &simple_commands[TSS_SET_WIRED_RESPONSE_HEADER_BITFIELD], (char*)&wired_setup.protocol_byte, NULL);
                f7WriteRead(dongle->serial_port, &simple_commands[TSS_SET_WIRELESS_RESPONSE_HEADER_BITFIELD], (char*)&wired_setup.protocol_byte, NULL);
                Sleep(600); //until i figure another way
                LeaveCriticalSection(&dongle->reader_lock);
                error=tss_setWirelessStreamingAutoFlushMode(device, 1, &timestamp); //turning off stream, if its streaming
                if(error){
                    return error;
                }
            }
            return TSS_NO_ERROR;
        }
    }
    //TODO else its a dongle?
    return TSS_INVALID_ID;
}

TSS_Error tss_getLastStreamData(TSS_Device_Id device, char * output_data, unsigned int output_data_len, unsigned int * timestamp){
    unsigned int tss_idx = ~(~device|TSS_NO_DONGLE_ID);
    if(tss_idx < sensor_list_len && sensor_list[tss_idx]){
        DWORD result;
        TSS_Sensor * sensor = sensor_list[tss_idx];
        if(sensor->stream_byte_len != output_data_len){
            #ifdef DEBUG_PRINT
            printf("sensor->stream_byte_len != output_data_len\n");
            #endif
            return TSS_ERROR_PARAMETER;
        }
        result = WaitForSingleObject(sensor->new_data_event, 0);
        if(result != WAIT_OBJECT_0){
            return TSS_ERROR_READ;
        }
        EnterCriticalSection(&sensor->stream_lock);
        memcpy(output_data, sensor->last_stream_data, sensor->stream_byte_len);
        if(timestamp){
            *timestamp = sensor->last_stream_timestamp;
        }
        LeaveCriticalSection(&sensor->stream_lock);
        return TSS_NO_ERROR;
    }
    return TSS_INVALID_ID;
}

TSS_Error tss_getLastStreamData2(TSS_Device_Id device, char * output_data, unsigned int output_data_len, unsigned int * timestamp){ //created for udk bindings
    return tss_getLastStreamData(device, output_data, output_data_len, timestamp);
}

TSS_Error tss_getLatestStreamData(TSS_Device_Id device, char * output_data, unsigned int output_data_len, unsigned int timeout, unsigned int * timestamp){
    unsigned int tss_idx = ~(~device|TSS_NO_DONGLE_ID);
    if(tss_idx < sensor_list_len && sensor_list[tss_idx]){
        DWORD result = WaitForSingleObject(sensor_list[tss_idx]->new_data_event, timeout);
        if(result == WAIT_OBJECT_0){
            TSS_Error tss_error = tss_getLastStreamData(device, output_data, output_data_len, timestamp);
            ResetEvent(sensor_list[tss_idx]->new_data_event);
            return tss_error;
        }
        return TSS_ERROR_TIMEOUT;
    }
    return TSS_INVALID_ID;
}

TSS_EXPORT TSS_Error tss_sendRawCommandFormatted(   TSS_Device_Id device,
                                                    unsigned char command,
                                                    char * in_data,
                                                    char * in_data_detail,
                                                    char * rtn_data,
                                                    char * rtn_data_detail,
                                                    unsigned int * timestamp){
    TSS_Command custom_command;
    custom_command.command = command;
    custom_command.description_str = "customCommandFormatted";
    custom_command.rtn_data_len = calculateParseDataSize(rtn_data_detail);
    custom_command.rtn_data_detail = rtn_data_detail;
    custom_command.in_data_len = calculateParseDataSize(in_data_detail);
    custom_command.in_data_detail = in_data_detail;
    custom_command.compatibility_mask = 0xfe000000;
    custom_command.fw_compatibility = TSS_FW_20R7_COMPATIBLE;
    return writeRead(device, &custom_command, in_data, rtn_data, timestamp);
}

TSS_EXPORT TSS_Error tss_sendRawCommand(TSS_Device_Id device,
                                        unsigned char command,
                                        char * in_data,
                                        int in_data_size,
                                        char * rtn_data,
                                        int rtn_data_size,
                                        unsigned int * timestamp){
    TSS_Command custom_command;
    custom_command.command = command;
    custom_command.description_str = "customCommand";
    custom_command.rtn_data_len = rtn_data_size;
    custom_command.rtn_data_detail = "";
    custom_command.in_data_len = in_data_size;
    custom_command.in_data_detail = "";
    custom_command.compatibility_mask = 0xfe000000;
    custom_command.fw_compatibility = TSS_FW_20R7_COMPATIBLE;
    return writeRead(device, &custom_command, in_data, rtn_data, timestamp);
}

TSS_Error tss_setNewDataCallBack(TSS_Device_Id device, TSS_CallBack callback_func){
    unsigned int tss_idx;
    tss_idx = ~(~device|TSS_NO_DONGLE_ID);
    if(tss_idx < sensor_list_len && sensor_list[tss_idx]){
        sensor_list[tss_idx]->callback = callback_func;
    }
    return TSS_INVALID_ID;
}

TSS_Error tss_getRecordCount(TSS_Device_Id device){
    unsigned int tss_idx = ~(~device|TSS_NO_DONGLE_ID);
    if(tss_idx < sensor_list_len && sensor_list[tss_idx]){
        return sensor_list[tss_idx]->record_count;
    }
    return TSS_NO_ERROR;
}

int tss_isConnected(TSS_Device_Id device,int reconnect){ //TODO
    unsigned int serial;
    if(tss_getSerialNumber(device, &serial, NULL)== TSS_NO_ERROR){
        return 1;
    }
    else{
        return 0;
    }
}

TSS_Error tss_getTSDeviceType(TSS_Device_Id device, TSS_Type * device_type){
    unsigned int tss_idx = ~(~device|TSS_NO_DONGLE_ID);
    if(tss_idx < sensor_list_len && sensor_list[tss_idx]){
        *device_type = sensor_list[tss_idx]->device_type;
        return TSS_NO_ERROR;
    }
    if(device & TSS_DONGLE_ID){
        tss_idx = (device - TSS_DONGLE_ID);
        if(tss_idx < dongle_list_len && dongle_list[tss_idx]){
            *device_type = TSS_DNG;
            return TSS_NO_ERROR;
        }
    }
    return TSS_INVALID_ID;
}

TSS_EXPORT int tss_resetThreeSpaceAPI(){
    int error;
    error = tss_delThreeSpaceAPI();
    if(!error){
        error = tss_initThreeSpaceAPI();
    }
    return error;
}

TSS_EXPORT int tss_createTSDevicesBySerialNumber(   unsigned int * serial_list,
                                                    TSS_Device_Id * device_list,
                                                    int list_length,
                                                    int search_wireless,
                                                    int search_unknown_devices,
                                                    TSS_Timestamp_Mode mode){
    int i,j,k;
    TSS_ComPort comport_list[1];
    TSS_ComInfo com_info;
    int device_count;
    TSS_Error error;
    int devices_created = 0;
    int offset=0;
    int comport_list_len = sizeof(comport_list)/sizeof(TSS_ComPort);

    //checking through existing devices ids first to see if a sensor of that serial was already created
    for(j=0; j < list_length; j++){
        device_list[j]= TSS_NO_DEVICE_ID;
        for(i=0; (unsigned)i < dongle_list_len; i++){
            if( dongle_list[i] != NULL){
                if(dongle_list[i]->device_serial == serial_list[j]){
                    device_list[j] = dongle_list[i]->device_id;
                    devices_created++;
                    break;
                }
                for( k = 0; k<15; k++){
                    if(dongle_list[i]->w_sensors[k] != 0){
                        //int id_mask;
                        TSS_Device_Id w_device = dongle_list[i]->w_sensors[k];
                        //id_mask = w_device & TSS_ALL_SENSORS_ID;
                        if(w_device & TSS_WIRELESS_W_ID){
                            unsigned int tss_idx = (w_device-TSS_WIRELESS_W_ID);
                            if(tss_idx < sensor_list_len && sensor_list[tss_idx]){
                                if(sensor_list[i]->device_serial == serial_list[j]){
                                    device_list[j] = dongle_list[i]->device_id;
                                    devices_created++;
                                    break;
                                }
                            }

                        }
                    }
                }
                if(device_list[j] != TSS_NO_DEVICE_ID){
                    break;
                }
            }
        }
        if(device_list[j] != TSS_NO_DEVICE_ID){
            break;
        }
        for(i=0; (unsigned)i < sensor_list_len; i++){
            if( sensor_list[i] != NULL){
                if(sensor_list[i]->device_serial == serial_list[j]){
                    device_list[j] = sensor_list[i]->device_id;
                    devices_created++;
                    break;
                }
            }
        }
    }
    // look through comports for dongles
    offset=0;
    do{
        device_count =tss_getComPorts(comport_list,comport_list_len,offset,TSS_FIND_DNG);
        for( i=0; i< device_count; ++i){
            //if search wireless the dongle will be created
            if(search_wireless){
                TSS_Device_Id device = tss_createTSDeviceStr(comport_list[i].com_port, mode);
                if(device != TSS_NO_DEVICE_ID){
                    unsigned int serial_number;
                    tss_getSerialNumber(device, &serial_number, NULL);
                    for(j=0; j < list_length; j++){
                        if(device_list[j] != TSS_NO_DEVICE_ID){
                            continue;
                        }
                        if(com_info.serial_number == serial_list[j]){
                            device_list[j]= device;
                            devices_created++;
                            break;
                        }
                    }
                    for( k = 0; k<15; k++){
                        serial_number = TSS_NO_DEVICE_ID;
                        tss_getSerialNumberAtLogicalID(device, k, &serial_number, NULL);
                        if(serial_number != 0){
                            for(j=0; j < list_length; j++){
                                if(device_list[j] != TSS_NO_DEVICE_ID){
                                    continue;
                                }
                                if(serial_number == serial_list[j]){
                                    unsigned int w_device;
                                    tss_getSensorFromDongle(device, k, &w_device);
                                    if(w_device != TSS_NO_DEVICE_ID){
                                        device_list[j]= w_device;
                                        devices_created++;
                                        break;
                                    }
                                }
                            }
                        }
                    }
                }
            }
            else{
                error= tss_getTSDeviceInfoFromComPort(comport_list[i].com_port,&com_info);
                if( error == TSS_NO_ERROR){
                    for(j=0; j < list_length; j++){
                        if(device_list[j] != TSS_NO_DEVICE_ID){
                            continue;
                        }
                        if(com_info.serial_number == serial_list[j]){
                            TSS_Device_Id device = tss_createTSDeviceStr(comport_list[i].com_port, mode);
                            if(device != TSS_NO_DEVICE_ID){
                                device_list[j]= device;
                                devices_created++;
                                break;
                            }
                        }
                    }
                    //printf("============(%s)=============\n",comport_list[i].com_port);
                    //printf("DeviceType:%s\nSerial:%08X\nHardwareVersion:%s\nFirmwareVersion:%s\nCompatibility:%d\n",
                    //   TSS_Type_String[com_info.device_type],
                    //   com_info.serial_number,
                    //   com_info.hardware_version,
                    //   com_info.firmware_version,
                    //   com_info.fw_compatibility);
                    //printf("================================\n");
                }
                else{
                    printf("Failed to communicate on %s\n",comport_list[i].com_port);
                }
            }
        }
        if(device_count > comport_list_len){
            break;
        }
        offset+=device_count;
    }while(device_count);
    //All other connected devices
    offset=0;
    do{
        device_count =tss_getComPorts(comport_list,comport_list_len,offset,TSS_FIND_ALL_KNOWN^TSS_FIND_DNG);
        for( i=0; i< device_count; ++i){
            error= tss_getTSDeviceInfoFromComPort(comport_list[i].com_port,&com_info);
            if( error == TSS_NO_ERROR){
                for(j=0; j < list_length; j++){
                    if(device_list[j] != TSS_NO_DEVICE_ID){
                        continue;
                    }
                    if(com_info.serial_number == serial_list[j]){
                        TSS_Device_Id device = tss_createTSDeviceStr(comport_list[i].com_port, mode);
                        if(device != TSS_NO_DEVICE_ID){
                            device_list[j]= device;
                            devices_created++;
                            break;
                        }
                    }
                }
            }
            else{
                printf("Failed to communicate on %s\n",comport_list[i].com_port);
            }
        }
        if(device_count > comport_list_len){
            break;
        }
        offset+=device_count;
    }while(device_count);

    //search unknown if desired
    if(! search_unknown_devices){
        return devices_created;
    }
    offset=0;
    do{
        device_count =tss_getComPorts(comport_list,comport_list_len,offset,TSS_FIND_UNKNOWN);
        for( i=0; i< device_count; ++i){
            error= tss_getTSDeviceInfoFromComPort(comport_list[i].com_port,&com_info);
            if( error == TSS_NO_ERROR){
                for(j=0; j < list_length; j++){
                    if(device_list[j] != TSS_NO_DEVICE_ID){
                        continue;
                    }
                    if(com_info.serial_number == serial_list[j]){
                        TSS_Device_Id device = tss_createTSDeviceStr(comport_list[i].com_port, mode);
                        if(device != TSS_NO_DEVICE_ID){
                            device_list[j]= device;
                            devices_created++;
                            break;
                        }
                    }
                }
            }
            else{
                printf("Failed to communicate on %s\n",comport_list[i].com_port);
            }
        }
        if(device_count > comport_list_len){
            break;
        }
        offset+=device_count;
    }while(device_count);
    return devices_created;
}

TSS_EXPORT int tss_getDeviceIDs( TSS_Device_Id * device_list, int list_length, int offset, int filter){
    int device_idx = 0;
    unsigned int i;
    for(i=0; i < sensor_list_len; i++){
        if(device_idx == list_length){
            break;
        }
        if( sensor_list[i] != NULL){
            if(offset){
                offset--;
                continue;
            }
            if((filter & TSS_FIND_USB) && sensor_list[i]->device_type == TSS_USB){
                device_list[device_idx] = sensor_list[i]->device_id;
                device_idx++;
            }
            else if((filter & TSS_FIND_WL) && (sensor_list[i]->device_type == TSS_WL || sensor_list[i]->device_type == TSS_WL_W)){
                device_list[device_idx] = sensor_list[i]->device_id;
                device_idx++;
            }
            else if((filter & TSS_FIND_EM) && sensor_list[i]->device_type == TSS_EM){
                device_list[device_idx] = sensor_list[i]->device_id;
                device_idx++;
            }
            else if((filter & TSS_FIND_DL) && sensor_list[i]->device_type == TSS_DL){
                device_list[device_idx] = sensor_list[i]->device_id;
                device_idx++;
            }
            else if((filter & TSS_FIND_BT) && sensor_list[i]->device_type == TSS_BT){
                device_list[device_idx] = sensor_list[i]->device_id;
                device_idx++;
            }
        }
    }
    if(filter & TSS_FIND_DNG){
        for(i=0; i < dongle_list_len; i++){
            if(device_idx == list_length){
                break;
            }
            if( dongle_list[i] != NULL){
                if(offset){
                    offset--;
                    continue;
                }
                device_list[device_idx] = dongle_list[i]->device_id;
                device_idx++;
            }
        }
    }
    return device_idx;
}

//Orientation Commands
//0(0x00)
TSS_EXPORT TSS_Error tss_getTaredOrientationAsQuaternion(TSS_Device_Id device,float * quat4, unsigned int * timestamp){
    return writeRead(device, &simple_commands[TSS_GET_TARED_ORIENTATION_AS_QUATERNION], NULL, (char*)quat4, timestamp);
}
//1(0x01)
TSS_EXPORT TSS_Error tss_getTaredOrientationAsEulerAngles(TSS_Device_Id device, float * euler3, unsigned int * timestamp){
    return writeRead(device, &simple_commands[TSS_GET_TARED_ORIENTATION_AS_EULER_ANGLES], NULL, (char*)euler3, timestamp);
}
//2(0x02)
TSS_EXPORT TSS_Error tss_getTaredOrientationAsRotationMatrix(TSS_Device_Id device, float * matrix9, unsigned int * timestamp){
    return writeRead(device, &simple_commands[TSS_GET_TARED_ORIENTATION_AS_ROTATION_MATRIX], NULL, (char*)matrix9, timestamp);
}
//3(0x03)
TSS_EXPORT TSS_Error tss_getTaredOrientationAsAxisAngle(TSS_Device_Id device, float * axis_angle4, unsigned int * timestamp){
    return writeRead(device, &simple_commands[TSS_GET_TARED_ORIENTATION_AS_AXIS_ANGLE], NULL, (char*)axis_angle4, timestamp);
}
//4(0x04)
TSS_EXPORT TSS_Error tss_getTaredOrientationAsTwoVector(TSS_Device_Id device, float * forward3, float * down3, unsigned int * timestamp){
    float forward_down[6];
    TSS_Error error=writeRead(device, &simple_commands[TSS_GET_TARED_ORIENTATION_AS_TWO_VECTOR], NULL, (char*)&forward_down, timestamp);
    if( error == TSS_NO_ERROR){
        memcpy(forward3,&forward_down[0],sizeof(float)*3);
        memcpy(down3,&forward_down[3],sizeof(float)*3);
    }
    return error;
}
//5(0x05)
TSS_EXPORT TSS_Error tss_getDifferenceQuaternion(TSS_Device_Id device, float * quat4, unsigned int * timestamp){
    return writeRead(device, &simple_commands[TSS_GET_DIFFERENCE_QUATERNION], NULL, (char*)quat4, timestamp);
}
//6(0x06)
TSS_EXPORT TSS_Error tss_getUntaredOrientationAsQuaternion(TSS_Device_Id device, float * quat4, unsigned int * timestamp){
    return writeRead(device, &simple_commands[TSS_GET_UNTARED_ORIENTATION_AS_QUATERNION], NULL, (char*)quat4, timestamp);
}
//7(0x07)
TSS_EXPORT TSS_Error tss_getUntaredOrientationAsEulerAngles(TSS_Device_Id device, float * euler3, unsigned int * timestamp){
    return writeRead(device, &simple_commands[TSS_GET_UNTARED_ORIENTATION_AS_EULER_ANGLES], NULL, (char*)euler3, timestamp);
}
//8(0x08)
TSS_EXPORT TSS_Error tss_getUntaredOrientationAsRotationMatrix(TSS_Device_Id device, float * matrix9, unsigned int * timestamp){
    return writeRead(device, &simple_commands[TSS_GET_UNTARED_ORIENTATION_AS_ROTATION_MATRIX], NULL, (char*)matrix9, timestamp);
}
//9(0x09)
TSS_EXPORT TSS_Error tss_getUntaredOrientationAsAxisAngle(TSS_Device_Id device, float * axis_angle4, unsigned int * timestamp){
    return writeRead(device, &simple_commands[TSS_GET_UNTARED_ORIENTATION_AS_AXIS_ANGLE], NULL, (char*)axis_angle4, timestamp);
}
//10(0x0a)
TSS_EXPORT TSS_Error tss_getUntaredOrientationAsTwoVector(TSS_Device_Id device, float * north3, float * gravity3, unsigned int * timestamp){
    float north_gravity[6];
    TSS_Error error=writeRead(device, &simple_commands[TSS_GET_UNTARED_ORIENTATION_AS_TWO_VECTOR], NULL, (char*)&north_gravity, timestamp);
    if( error == TSS_NO_ERROR){
        memcpy(north3,&north_gravity[0],sizeof(float)*3);
        memcpy(gravity3,&north_gravity[3],sizeof(float)*3);
    }
    return error;
}
//11(0x0b)
TSS_EXPORT TSS_Error tss_getTaredTwoVectorInSensorFrame(TSS_Device_Id device, float * forward3, float * down3, unsigned int * timestamp){
    float forward_down[6];
    TSS_Error error=writeRead(device, &simple_commands[TSS_GET_TARED_TWO_VECTOR_IN_SENSOR_FRAME], NULL, (char*)&forward_down, timestamp);
    if( error == TSS_NO_ERROR){
        memcpy(forward3,&forward_down[0],sizeof(float)*3);
        memcpy(down3,&forward_down[3],sizeof(float)*3);
    }
    return error;
}
//12(0x0c)
TSS_EXPORT TSS_Error tss_getUntaredTwoVectorInSensorFrame(TSS_Device_Id device, float * north3, float * gravity3, unsigned int * timestamp){
    float north_gravity[6];
    TSS_Error error=writeRead(device, &simple_commands[TSS_GET_UNTARED_TWO_VECTOR_IN_SENSOR_FRAME], NULL, (char*)&north_gravity, timestamp);
    if( error == TSS_NO_ERROR){
        memcpy(north3,&north_gravity[0],sizeof(float)*3);
        memcpy(gravity3,&north_gravity[3],sizeof(float)*3);
    }
    return error;
}

//Embedded Commands
//29(0x1d)
//TSS_EXPORT TSS_Error tss_setInterruptType(TSS_Device_Id device, unsigned char mode, unsigned char pin, unsigned int * timestamp){
//    unsigned char args[2]={mode,pin};
//    return writeRead(device, &simple_commands[TSS_SET_PIN_MODE], (char*)&args, NULL, timestamp);
//}
TSS_EXPORT TSS_Error tss_setPinMode(TSS_Device_Id device, unsigned char mode, unsigned char pin, unsigned int * timestamp){
    unsigned char args[2]={mode,pin};
    return writeRead(device, &simple_commands[TSS_SET_PIN_MODE], (char*)&args, NULL, timestamp);
}
//30(0x1e)
//TSS_EXPORT TSS_Error tss_getInterruptType(TSS_Device_Id device, unsigned char * mode, unsigned char * pin, unsigned int * timestamp){
//    unsigned char args[2];
//    TSS_Error error= writeRead(device, &simple_commands[TSS_GET_PIN_MODE], NULL, (char*)&args, timestamp);
//    if(error == TSS_NO_ERROR){
//        *mode = args[0];
//        *pin = args[1];
//    }
//    return error;
//}
TSS_EXPORT TSS_Error tss_getPinMode(TSS_Device_Id device, unsigned char * mode, unsigned char * pin, unsigned int * timestamp){
    unsigned char args[2];
    TSS_Error error= writeRead(device, &simple_commands[TSS_GET_PIN_MODE], NULL, (char*)&args, timestamp);
    if(error == TSS_NO_ERROR){
        *mode = args[0];
        *pin = args[1];
    }
    return error;
}
//31(0x1f)
TSS_EXPORT TSS_Error tss_getInterruptStatus(TSS_Device_Id device, unsigned char * status, unsigned int * timestamp){
    return writeRead(device, &simple_commands[TSS_GET_INTERRUPT_STATUS], NULL, (char*)status, timestamp);
}

//Normalized Data Commands
//32(0x20)
TSS_EXPORT TSS_Error tss_getAllNormalizedComponentSensorData(TSS_Device_Id device, float * gyro_rate3, float * gravity_direction3, float * north_direction3, unsigned int * timestamp){
    float sensor_data9[9];
    TSS_Error error= writeRead(device, &simple_commands[TSS_GET_ALL_NORMALIZED_COMPONENT_SENSOR_DATA], NULL, (char*)&sensor_data9, timestamp);
    if( error == TSS_NO_ERROR){
        memcpy(gyro_rate3,&sensor_data9[0],sizeof(float)*3);
        memcpy(gravity_direction3,&sensor_data9[3],sizeof(float)*3);
        memcpy(north_direction3,&sensor_data9[6],sizeof(float)*3);
    }
    return error;
}
//33(0x21)
TSS_EXPORT TSS_Error tss_getNormalizedGyroRate(TSS_Device_Id device, float * gyro_rate3, unsigned int * timestamp){
    return writeRead(device, &simple_commands[TSS_GET_NORMALIZED_GYRO_RATE], NULL, (char*)gyro_rate3, timestamp);
}
//34(0x22)
TSS_EXPORT TSS_Error tss_getNormalizedAccelerometerVector(TSS_Device_Id device, float * gravity_direction3, unsigned int * timestamp){
    return writeRead(device, &simple_commands[TSS_GET_NORMALIZED_ACCELEROMETER_VECTOR], NULL, (char*)gravity_direction3, timestamp);
}
//35(0x23)
TSS_EXPORT TSS_Error tss_getNormalizedCompassVector(TSS_Device_Id device, float * north_direction3, unsigned int * timestamp){
    return writeRead(device, &simple_commands[TSS_GET_NORMALIZED_COMPASS_VECTOR], NULL, (char*)north_direction3, timestamp);
}

//Corrected Data Commands
//37(0x25)
TSS_EXPORT TSS_Error tss_getAllCorrectedComponentSensorData(TSS_Device_Id device, float * gyro_rate3, float * accelerometer3, float * compass3, unsigned int * timestamp){
    float sensor_data9[9];
    TSS_Error error= writeRead(device, &simple_commands[TSS_GET_ALL_CORRECTED_COMPONENT_SENSOR_DATA], NULL, (char*)&sensor_data9, timestamp);
    if( error == TSS_NO_ERROR){
        memcpy(gyro_rate3,&sensor_data9[0],sizeof(float)*3);
        memcpy(accelerometer3,&sensor_data9[3],sizeof(float)*3);
        memcpy(compass3,&sensor_data9[6],sizeof(float)*3);
    }
    return error;
}
//38(0x26)
TSS_EXPORT TSS_Error tss_getCorrectedGyroRate(TSS_Device_Id device, float * gyro_rate3, unsigned int * timestamp){
    return writeRead(device, &simple_commands[TSS_GET_CORRECTED_GYRO_RATE], NULL, (char*)gyro_rate3, timestamp);
}
//39(0x27)
TSS_EXPORT TSS_Error tss_getCorrectedAccelerometerVector(TSS_Device_Id device, float * accelerometer3, unsigned int * timestamp){
    return writeRead(device, &simple_commands[TSS_GET_CORRECTED_ACCELEROMETER_VECTOR], NULL, (char*)accelerometer3, timestamp);
}
//40(0x28)
TSS_EXPORT TSS_Error tss_getCorrectedCompassVector(TSS_Device_Id device, float * compass3, unsigned int * timestamp){
    return writeRead(device, &simple_commands[TSS_GET_CORRECTED_COMPASS_VECTOR], NULL, (char*)compass3, timestamp);
}
//41(0x29)
TSS_EXPORT TSS_Error tss_getCorrectedLinearAccelerationInGlobalSpace(TSS_Device_Id device, float * accelerometer3, unsigned int * timestamp){
    return writeRead(device, &simple_commands[TSS_GET_CORRECTED_LINEAR_ACCELERATION_IN_GLOBAL_SPACE], NULL, (char*)accelerometer3, timestamp);
}

//Other Data Commands
//43(0x2b)
TSS_EXPORT TSS_Error tss_getTemperatureC(TSS_Device_Id device, float * temperature, unsigned int * timestamp){
    return writeRead(device, &simple_commands[TSS_GET_TEMPERATURE_C], NULL, (char*)temperature, timestamp);
}
//44(0x2c)
TSS_EXPORT TSS_Error tss_getTemperatureF(TSS_Device_Id device, float * temperature, unsigned int * timestamp){
    return writeRead(device, &simple_commands[TSS_GET_TEMPERATURE_F], NULL, (char*)temperature, timestamp);
}
//45(0x2d)
TSS_EXPORT TSS_Error tss_getConfidenceFactor(TSS_Device_Id device, float * confidence, unsigned int * timestamp){
    return writeRead(device, &simple_commands[TSS_GET_CONFIDENCE_FACTOR], NULL, (char*)confidence, timestamp);
}

//Raw Data Commands
//64(0x40)
TSS_EXPORT TSS_Error tss_getAllRawComponentSensorData(TSS_Device_Id device, float * gyro_rate3, float * accelerometer3, float * compass3, unsigned int * timestamp){
    float sensor_data9[9];
    TSS_Error error= writeRead(device, &simple_commands[TSS_GET_ALL_RAW_COMPONENT_SENSOR_DATA], NULL, (char*)&sensor_data9, timestamp);
    if( error == TSS_NO_ERROR){
        memcpy(gyro_rate3,&sensor_data9[0],sizeof(float)*3);
        memcpy(accelerometer3,&sensor_data9[3],sizeof(float)*3);
        memcpy(compass3,&sensor_data9[6],sizeof(float)*3);
    }
    return error;
}
//65(0x41)
TSS_EXPORT TSS_Error tss_getRawGyroscopeRate(TSS_Device_Id device, float * gyro_rate3, unsigned int * timestamp){
    return writeRead(device, &simple_commands[TSS_GET_RAW_GYROSCOPE_RATE], NULL, (char*)gyro_rate3, timestamp);
}
//66(0x42)
TSS_EXPORT TSS_Error tss_getRawAccelerometerData(TSS_Device_Id device, float * accelerometer3, unsigned int * timestamp){
    return writeRead(device, &simple_commands[TSS_GET_RAW_ACCELEROMETER_DATA], NULL, (char*)accelerometer3, timestamp);
}
//67(0x43)
TSS_EXPORT TSS_Error tss_getRawCompassData(TSS_Device_Id device, float * compass3, unsigned int * timestamp){
    return writeRead(device, &simple_commands[TSS_GET_RAW_COMPASS_DATA], NULL, (char*)compass3, timestamp);
}

//Data-Logging Commands
//57(0x39)
TSS_EXPORT TSS_Error tss_turnOnMassStorage(TSS_Device_Id device, unsigned int * timestamp){
    return writeRead(device, &simple_commands[TSS_TURN_ON_MASSS_TORAGE], NULL, NULL, timestamp);
}
//58(0x3a)
TSS_EXPORT TSS_Error tss_turnOffMassStorage(TSS_Device_Id device, unsigned int * timestamp){
    return writeRead(device, &simple_commands[TSS_TURN_OFF_MASS_STORAGE], NULL, NULL, timestamp);
}
//59(0x3b)
TSS_EXPORT TSS_Error tss_formatAndInitializeSDCard(TSS_Device_Id device, unsigned int * timestamp){
    return writeRead(device, &simple_commands[TSS_FORMAT_AND_INITIALIZE_SD_CARD], NULL, NULL, timestamp);
}
//60(0x3c)
TSS_EXPORT TSS_Error tss_beginDataLoggingSession(TSS_Device_Id device, unsigned int * timestamp){
    return writeRead(device, &simple_commands[TSS_BEGIN_DATA_LOGGING_SESSION], NULL, NULL, timestamp);
}
//61(0x3d)
TSS_EXPORT TSS_Error tss_endDataLoggingSession(TSS_Device_Id device, unsigned int * timestamp){
    return writeRead(device, &simple_commands[TSS_END_DATA_LOGGING_SESSION], NULL, NULL, timestamp);
}
//62(0x3e)
TSS_EXPORT TSS_Error tss_setClockValues(TSS_Device_Id device,   unsigned char month,
                                                                unsigned char day,
                                                                unsigned char year,
                                                                unsigned char hour,
                                                                unsigned char minute,
                                                                unsigned char second,
                                                                unsigned int * timestamp){
    unsigned char date_time[6] = {month, day, year, hour, minute, second};
    return writeRead(device, &simple_commands[TSS_SET_CLOCK_VALUES], (char*)&date_time, NULL, timestamp);
}
//63(0x3f)
TSS_EXPORT TSS_Error tss_getClockValues(TSS_Device_Id device,    unsigned char * month,
                                                                unsigned char * day,
                                                                unsigned char * year,
                                                                unsigned char * hour,
                                                                unsigned char * minute,
                                                                unsigned char * second,
                                                                unsigned int * timestamp){
    unsigned char date_time[6];
    TSS_Error error;
    error = writeRead(device, &simple_commands[TSS_GET_CLOCK_VALUES], NULL, (char*)date_time, timestamp);
    if(error == TSS_NO_ERROR){
        *month = date_time[0];
        *day= date_time[1];
        *year= date_time[2];
        *hour= date_time[3];
        *minute= date_time[4];
        *second= date_time[5];
    }
    return error;
}

//Streaming Commands
//80(0x50)
TSS_EXPORT TSS_Error tss_setStreamingSlots(TSS_Device_Id device,const TSS_Stream_Command * slots8, unsigned int * timestamp){  //TYPECHECK
    int i;
    unsigned char command_bytes[8];
    TSS_Sensor * sensor;
    unsigned int tss_idx = ~(~device|TSS_NO_DONGLE_ID);

    unsigned int rtn_data_detail_len;

    if(!(device & simple_commands[TSS_SET_STREAMING_SLOTS].compatibility_mask)){
        return TSS_INVALID_ID;
    }

    sensor = sensor_list[tss_idx];
    memset(sensor->stream_parse_str, 0, sizeof(sensor->stream_parse_str));
    sensor->stream_byte_len=0;

    sensor->stream_slots[0] = TSS_NULL;
    sensor->stream_slots[1] = TSS_NULL;
    sensor->stream_slots[2] = TSS_NULL;
    sensor->stream_slots[3] = TSS_NULL;
    sensor->stream_slots[4] = TSS_NULL;
    sensor->stream_slots[5] = TSS_NULL;
    sensor->stream_slots[6] = TSS_NULL;
    sensor->stream_slots[7] = TSS_NULL;
    for( i=0; i < 8; i++){
        TSS_Command command =simple_commands[slots8[i]];
        command_bytes[i] = command.command;
        sensor->stream_slots[i] = slots8[i];
        sensor->stream_byte_len+= command.rtn_data_len;
        rtn_data_detail_len =strlen(command.rtn_data_detail);
        if((strlen(sensor->stream_parse_str)+rtn_data_detail_len) <= 256){
            strncat(sensor->stream_parse_str, command.rtn_data_detail, strlen(command.rtn_data_detail));
        }
        else{
            sensor->stream_byte_len = 0;
            #ifdef DEBUG_PRINT
            printf("Error: Slot Buffer is full!!!\n");
            #endif
            return TSS_ERROR_STREAM_SLOTS_FULL;
        }


    }
    sensor->last_stream_data = realloc(sensor->last_stream_data,sensor->stream_byte_len);
    #ifdef DEBUG_PRINT
    printf("Stream parse string:%s\n",sensor->stream_parse_str);
    printf("Command bytes: %X, %X, %X, %X, %X, %X, %X, %X\n", command_bytes[0],
                                                            command_bytes[1],
                                                            command_bytes[2],
                                                            command_bytes[3],
                                                            command_bytes[4],
                                                            command_bytes[5],
                                                            command_bytes[6],
                                                            command_bytes[7]);
    #endif
    return writeRead(device, &simple_commands[TSS_SET_STREAMING_SLOTS], (char*)command_bytes,
                      NULL, timestamp);
}
//81(0x51)
TSS_EXPORT TSS_Error tss_getStreamingSlots(TSS_Device_Id device, TSS_Stream_Command * slots8, unsigned int * timestamp){
    return writeRead(device, &simple_commands[TSS_GET_STREAMING_SLOTS], NULL,(char*)slots8,timestamp);
}
//82(0x52)
TSS_EXPORT TSS_Error tss_setStreamingTiming(TSS_Device_Id device, unsigned int interval, unsigned int duration, unsigned int delay, unsigned int * timestamp){
    unsigned int args[3]= {interval,duration, delay};
    return writeRead(device, &simple_commands[TSS_SET_STREAMING_TIMING],
                      (char*)args, NULL, timestamp);
}
//83(0x53)
TSS_EXPORT TSS_Error tss_getStreamingTiming(TSS_Device_Id device, unsigned int * interval, unsigned int * duration, unsigned int * delay, unsigned int * timestamp){
    unsigned int output[3];
    int return_val;
    return_val = writeRead(device, &simple_commands[TSS_GET_STREAMING_TIMING],
                            NULL, (char*)output, timestamp);
    *interval = output[0];
    *duration = output[1];
    *delay    = output[2];
    return return_val;
}
//84(0x54)
TSS_EXPORT TSS_Error tss_getStreamingBatch(TSS_Device_Id device, char * output_data, unsigned int output_data_len, unsigned int * timestamp){
    TSS_Command batch_command;
    unsigned int tss_idx = ~(~device|TSS_NO_DONGLE_ID);
    if(tss_idx < sensor_list_len && sensor_list[tss_idx]){
        if(sensor_list[tss_idx]->stream_byte_len){
            if( sensor_list[tss_idx]->stream_byte_len != output_data_len ){
                return TSS_ERROR_PARAMETER;
            }
            memcpy(&batch_command, &simple_commands[TSS_GET_STREAMING_BATCH], sizeof(TSS_Command));
            batch_command.rtn_data_len = sensor_list[tss_idx]->stream_byte_len;
            batch_command.rtn_data_detail = sensor_list[tss_idx]->stream_parse_str;
            return writeRead(device, &batch_command, NULL, output_data,timestamp);
        }
        else{
            return TSS_ERROR_STREAM_CONFIG;
        }
        return TSS_NO_ERROR;
    }
    return TSS_INVALID_ID;
}
//85(0x55)
TSS_EXPORT TSS_Error tss_startStreaming(TSS_Device_Id device, unsigned int * timestamp){ //TYPECHECK
    unsigned int tss_idx = ~(~device|TSS_NO_DONGLE_ID);
    if(!(device & simple_commands[TSS_START_STREAMING].compatibility_mask)){
        return TSS_INVALID_COMMAND;
    }
    if(tss_idx < sensor_list_len && sensor_list[tss_idx]){
        sensor_list[tss_idx]->record_count = 0;
        sensor_list[tss_idx]->stream_enabled=1;
        ResetEvent(sensor_list[tss_idx]->new_data_event);
        return writeRead(device, &simple_commands[TSS_START_STREAMING], NULL, NULL,timestamp);
    }
    return TSS_INVALID_ID;
}
//86(0x56)
TSS_EXPORT TSS_Error tss_stopStreaming(TSS_Device_Id device, unsigned int * timestamp){
    int error;
    #ifdef DEBUG_PRINT
    printf("Stopping stream\n");
    #endif
    if(!(device & simple_commands[TSS_STOP_STREAMING].compatibility_mask)){
        return TSS_INVALID_COMMAND;
    }
    error=writeRead(device, &simple_commands[TSS_STOP_STREAMING], NULL,
                    NULL,timestamp);

    if(!error){
        unsigned int tss_idx = ~(~device|TSS_NO_DONGLE_ID);
        if(tss_idx < sensor_list_len && sensor_list[tss_idx]){
            sensor_list[tss_idx]->stream_enabled=0;
        }
    }
   return error;
}
//95(0x5f)
TSS_EXPORT TSS_Error tss_updateCurrentTimestamp(TSS_Device_Id device, unsigned int set_timestamp, unsigned int * timestamp){
    return writeRead(device, &simple_commands[TSS_UPDATE_CURRENT_TIMESTAMP],(char *)&set_timestamp,NULL,timestamp);
}

//Configuration Write Commands
//16(0x10)
TSS_EXPORT TSS_Error tss_setEulerAngleDecompositionOrder(TSS_Device_Id device, unsigned char order, unsigned int * timestamp){
    return writeRead(device, &simple_commands[TSS_SET_EULER_ANGLE_DECOMPOSITION_ORDER],(char *)&order,NULL,timestamp);
}
//  17(0x11)
TSS_EXPORT TSS_Error tss_setMagnetoresistiveThreshold(TSS_Device_Id device, float threshold, unsigned int trust_frames, float lockout_decay, float perturbation_detection_value, unsigned int * timestamp){
    char threshold_params[16];
    memcpy(&threshold_params[0], &threshold, sizeof(float));
    memcpy(&threshold_params[4], &trust_frames, sizeof(unsigned int));
    memcpy(&threshold_params[8], &lockout_decay, sizeof(float));
    memcpy(&threshold_params[12], &perturbation_detection_value, sizeof(float));
    return writeRead(device, &simple_commands[TSS_SET_MAGNETORESISTIVE_THRESHOLD],(char *)threshold_params,NULL,timestamp);
}
//  18(0x12)
TSS_EXPORT TSS_Error tss_setAccelerometerResistanceThreshold(TSS_Device_Id device, float threshold, unsigned int lockout_frames, unsigned int * timestamp){
    char threshold_params[8];
    memcpy(&threshold_params[0], &threshold, sizeof(float));
    memcpy(&threshold_params[4], &lockout_frames, sizeof(unsigned int));
    return writeRead(device, &simple_commands[TSS_SET_ACCELEROMETER_RESISTANCE_THRESHOLD],(char *)threshold_params,NULL,timestamp);
}
//  19(0x13)
TSS_EXPORT TSS_Error tss_offsetWithCurrentOrientation(TSS_Device_Id device, unsigned int * timestamp){
    return writeRead(device, &simple_commands[TSS_OFFSET_WITH_CURRENT_ORIENTATION],NULL,NULL,timestamp);
}
//  20(0x14)
TSS_EXPORT TSS_Error tss_resetBaseOffset(TSS_Device_Id device, unsigned int * timestamp){
    return writeRead(device, &simple_commands[TSS_RESET_BASE_OFFSET],NULL,NULL,timestamp);
}
//  21(0x15)
TSS_EXPORT TSS_Error tss_offsetWithQuaternion(TSS_Device_Id device, const float * quat4, unsigned int * timestamp){
    return writeRead(device, &simple_commands[TSS_OFFSET_WITH_QUATERNION],(char*)quat4,NULL,timestamp);
}
//  22(0x16)
TSS_EXPORT TSS_Error tss_setBaseOffsetWithCurrentOrientation(TSS_Device_Id device, unsigned int * timestamp){
    return writeRead(device, &simple_commands[TSS_SET_BASE_OFFSET_WITH_CURRENT_ORIENTATION],NULL,NULL,timestamp);
}
//96(0x60)
TSS_EXPORT TSS_Error tss_tareWithCurrentOrientation(TSS_Device_Id device, unsigned int * timestamp){
    return writeRead(device, &simple_commands[TSS_TARE_WITH_CURRENT_ORIENTATION], NULL, NULL,timestamp);
}
//97(0x61)
TSS_EXPORT TSS_Error tss_tareWithQuaternion(TSS_Device_Id device,const float * quat4, unsigned int * timestamp){
    return writeRead(device, &simple_commands[TSS_TARE_WITH_QUATERNION],(char *)quat4,NULL,timestamp);
}
//98(0x62)
TSS_EXPORT TSS_Error tss_tareWithRotationMatrix(TSS_Device_Id device, const float * matrix9, unsigned int * timestamp){
    return writeRead(device, &simple_commands[TSS_TARE_WITH_ROTATION_MATRIX],(char *)matrix9, NULL,timestamp);
}
//99(0x63)//tss_setStaticAccelerometerTrustValue planned to be removed
//TSS_EXPORT TSS_Error tss_setStaticAccelerometerRhoMode(TSS_Device_Id device, float rho_value, unsigned int * timestamp){
//    return writeRead(device, &simple_commands[TSS_SET_STATIC_ACCELEROMETER_RHO_MODE],(char *)&rho_value,NULL,timestamp);
//}
TSS_EXPORT TSS_Error tss_setStaticAccelerometerTrustValue(TSS_Device_Id device, float trust_value, unsigned int * timestamp){
    float min_max_trust_value[] ={trust_value,trust_value};
    return writeRead(device, &simple_commands[TSS_SET_CONFIDENCE_ACCELEROMETER_TRUST_VALUES],(char *)&min_max_trust_value,NULL,timestamp);
}
//100(0x64)
//TSS_EXPORT TSS_Error tss_setConfidenceAccelerometerRhoMode(TSS_Device_Id device,float min_rho_value, float max_rho_value, unsigned int * timestamp){
//    float min_max_rho_value[] ={min_rho_value,max_rho_value};
//    return writeRead(device, &simple_commands[TSS_SET_CONFIDENCE_ACCELEROMETER_RHO_MODE],(char *)&min_max_rho_value,NULL,timestamp);
//}
TSS_EXPORT TSS_Error tss_setConfidenceAccelerometerTrustValues(TSS_Device_Id device, float min_trust_value, float max_trust_value, unsigned int * timestamp){
    float min_max_trust_value[] ={min_trust_value,max_trust_value};
    return writeRead(device, &simple_commands[TSS_SET_CONFIDENCE_ACCELEROMETER_TRUST_VALUES],(char *)&min_max_trust_value,NULL,timestamp);
}
//101(0x65)
//TSS_EXPORT TSS_Error tss_setStaticCompassRhoMode(TSS_Device_Id device, float rho_value, unsigned int * timestamp){
//    return writeRead(device, &simple_commands[TSS_SET_STATIC_COMPASS_RHO_MODE],(char *)&rho_value,NULL,timestamp);
//}
TSS_EXPORT TSS_Error tss_setStaticCompassTrustValue(TSS_Device_Id device, float trust_value, unsigned int * timestamp){
    float min_max_trust_value[] ={trust_value,trust_value};
    return writeRead(device, &simple_commands[TSS_SET_CONFIDENCE_COMPASS_TRUST_VALUES],(char *)&min_max_trust_value,NULL,timestamp);
}
//102(0x66)
//TSS_EXPORT TSS_Error tss_setConfidenceCompassRhoMode(TSS_Device_Id device,float min_rho_value, float max_rho_value, unsigned int * timestamp){
//    float min_max_rho_value[2] ={min_rho_value,max_rho_value};
//    return writeRead(device, &simple_commands[TSS_SET_CONFIDENCE_COMPASS_TRUST_VALUES],(char *)&min_max_rho_value,NULL,timestamp);
//}
TSS_EXPORT TSS_Error tss_setConfidenceCompassTrustValues(TSS_Device_Id device, float min_trust_value, float max_trust_value, unsigned int * timestamp){
    float min_max_trust_value[] ={min_trust_value,max_trust_value};
    return writeRead(device, &simple_commands[TSS_SET_CONFIDENCE_COMPASS_TRUST_VALUES],(char *)&min_max_trust_value,NULL,timestamp);
}
//103(0x67)
TSS_EXPORT TSS_Error tss_setDesiredUpdateRate(TSS_Device_Id device, unsigned int update_rate, unsigned int * timestamp){
    return writeRead(device, &simple_commands[TSS_SET_DESIRED_UPDATE_RATE],(char *)&update_rate,NULL,timestamp);
}
//105(0x69)
TSS_EXPORT TSS_Error tss_setReferenceVectorMode(TSS_Device_Id device, unsigned char mode, unsigned int * timestamp){
    return writeRead(device, &simple_commands[TSS_SET_REFERENCE_VECTOR_MODE],(char *)&mode,NULL,timestamp);
}
//106(0x6a)
TSS_EXPORT TSS_Error tss_setOversampleRate(TSS_Device_Id device, unsigned char samples_per_iteration, unsigned int * timestamp){
    return writeRead(device, &simple_commands[TSS_SET_OVERSAMPLE_RATE],(char *)&samples_per_iteration,NULL,timestamp);
}
//107(0x6b)
TSS_EXPORT TSS_Error tss_setGyroscopeEnabled(TSS_Device_Id device, unsigned char enabled, unsigned int * timestamp){
    return writeRead(device, &simple_commands[TSS_SET_GYROSCOPE_ENABLED],(char *)&enabled,NULL,timestamp);
}
//108(0x6c)
TSS_EXPORT TSS_Error tss_setAccelerometerEnabled(TSS_Device_Id device, unsigned char enabled, unsigned int * timestamp){
    return writeRead(device, &simple_commands[TSS_SET_ACCELEROMETER_ENABLED],(char *)&enabled,NULL,timestamp);
}
//109(0x6d)
TSS_EXPORT TSS_Error tss_setCompassEnabled(TSS_Device_Id device, unsigned char enabled, unsigned int * timestamp){
    return writeRead(device, &simple_commands[TSS_SET_COMPASS_ENABLED],(char *)&enabled,NULL,timestamp);
}
//116(0x74)
TSS_EXPORT TSS_Error tss_setAxisDirections(TSS_Device_Id device, unsigned char axis_direction_byte, unsigned int * timestamp){
    return writeRead(device, &simple_commands[TSS_SET_AXIS_DIRECTIONS],(char *)&axis_direction_byte,NULL,timestamp);
}
//117(0x75)
TSS_EXPORT TSS_Error tss_setRunningAveragePercent(TSS_Device_Id device, float running_average_percent, unsigned int * timestamp){
    return writeRead(device, &simple_commands[TSS_SET_RUNNING_AVERAGE_PERCENT],(char *)&running_average_percent,NULL,timestamp);
}
//118(0x76)
TSS_EXPORT TSS_Error tss_setCompassReferenceVector(TSS_Device_Id device, const float * reference_vector3, unsigned int * timestamp){
    return writeRead(device, &simple_commands[TSS_SET_COMPASS_REFERENCE_VECTOR],(char *)&reference_vector3,NULL,timestamp);
}
//119(0x77)
TSS_EXPORT TSS_Error tss_setAccelerometerReferenceVector(TSS_Device_Id device, const float * reference_vector3, unsigned int * timestamp){
    return writeRead(device, &simple_commands[TSS_SET_ACCELEROMETER_REFERENCE_VECTOR],(char *)&reference_vector3,NULL,timestamp);
}
//120(0x78)
TSS_EXPORT TSS_Error tss_resetKalmanFilter(TSS_Device_Id device, unsigned int * timestamp){
    return writeRead(device, &simple_commands[TSS_RESET_KALMAN_FILTER],NULL, NULL,timestamp);
}
//121(0x79)
TSS_EXPORT TSS_Error tss_setAccelerometerRange(TSS_Device_Id device, unsigned char accelerometer_range_setting, unsigned int * timestamp){
    return writeRead(device, &simple_commands[TSS_SET_ACCELEROMETER_RANGE],(char *)&accelerometer_range_setting,NULL,timestamp);
}
//123(0x7b)
TSS_EXPORT TSS_Error tss_setFilterMode(TSS_Device_Id device, unsigned char mode, unsigned int * timestamp){
    return writeRead(device, &simple_commands[TSS_SET_FILTER_MODE],(char *)&mode,NULL,timestamp);
}
//124(0x7c)
TSS_EXPORT TSS_Error tss_setRunningAverageMode(TSS_Device_Id device, unsigned char mode, unsigned int * timestamp){
    return writeRead(device, &simple_commands[TSS_SET_RUNNING_AVERAGE_MODE],(char *)&mode,NULL,timestamp);
}
//125(0x7d)
TSS_EXPORT TSS_Error tss_setGyroscopeRange(TSS_Device_Id device, unsigned char gyroscope_range_setting, unsigned int * timestamp){
    return writeRead(device, &simple_commands[TSS_SET_GYROSCOPE_RANGE],(char *)&gyroscope_range_setting,NULL,timestamp);
}
//126(0x7e)
TSS_EXPORT TSS_Error tss_setCompassRange(TSS_Device_Id device, unsigned char compass_range_setting, unsigned int * timestamp){
    return writeRead(device, &simple_commands[TSS_SET_COMPASS_RANGE],(char *)&compass_range_setting,NULL,timestamp);
}

// Configuration Read Commands
//128(0x80)  Get tare orientation as quaternion
TSS_EXPORT TSS_Error tss_getTareAsQuaternion(TSS_Device_Id device, float * quat4, unsigned int * timestamp){
    return writeRead(device, &simple_commands[TSS_GET_TARE_AS_QUATERNION],NULL,(char *)quat4,timestamp);
}
//129(0x81)  Get tare orientation as rotation matrix
TSS_EXPORT TSS_Error tss_getTareAsRotationMatrix(TSS_Device_Id device, float * matrix9, unsigned int * timestamp){
    return writeRead(device, &simple_commands[TSS_GET_TARE_AS_ROTATION_MATRIX],NULL,(char *)matrix9,timestamp);
}
//130(0x82)
//TSS_EXPORT TSS_Error tss_getAccelerometerRhoValue(TSS_Device_Id device, unsigned char * rho_mode, float * rho_min, float * rho_max, unsigned int * timestamp){ //TEST!!
//    TSS_Command command={0x82,"getAccelerometerRhoValue", 9, "Bff", 0, "", 0xfc000000, TSS_FW_20R7_COMPATIBLE};
//    unsigned char rho_mode_min_max[9];
//    TSS_Error error= writeRead(device, &command,NULL,(char *)&rho_mode_min_max,timestamp);
//    if(error == TSS_NO_ERROR){
//        *rho_mode = rho_mode_min_max[0];
//        *rho_min = (float)rho_mode_min_max[1];
//        *rho_max = (float)rho_mode_min_max[5];
//    }
//    return error;
//}
TSS_EXPORT TSS_Error tss_getAccelerometerTrustValues(TSS_Device_Id device, float * min_trust_value, float * max_trust_value, unsigned int * timestamp){ //TEST!!
    float min_max_trust_value[2];
    TSS_Error error= writeRead(device, &simple_commands[TSS_GET_ACCELEROMETER_TRUST_VALUES],NULL,(char *)&min_max_trust_value,timestamp);
    if(error == TSS_NO_ERROR){
        *min_trust_value = min_max_trust_value[0];
        *max_trust_value = min_max_trust_value[1];
    }
    return error;
}
//131(0x83)
//TSS_EXPORT TSS_Error tss_getCompassRhoValue(TSS_Device_Id device, unsigned char * rho_mode, float * rho_min, float * rho_max, unsigned int * timestamp){ //TEST!!
//    TSS_Command command={0x83,"getCompassRhoValue", 9, "Bff", 0, "", 0xfc000000, TSS_FW_20R7_COMPATIBLE};
//    unsigned char rho_mode_min_max[9];
//    TSS_Error error= writeRead(device, &command,NULL,(char *)&rho_mode_min_max,timestamp);
//    if(error == TSS_NO_ERROR){
//        *rho_mode = rho_mode_min_max[0];
//        *rho_min = (float)rho_mode_min_max[1];
//        *rho_max = (float)rho_mode_min_max[5];
//    }
//    return error;
//}
TSS_EXPORT TSS_Error tss_getCompassTrustValues(TSS_Device_Id device, float * min_trust_value, float * max_trust_value, unsigned int * timestamp){ //TEST!!
    float min_max_trust_value[2];
    TSS_Error error= writeRead(device, &simple_commands[TSS_GET_COMPASS_TRUST_VALUES],NULL,(char *)&min_max_trust_value,timestamp);
    if(error == TSS_NO_ERROR){
        *min_trust_value = min_max_trust_value[0];
        *max_trust_value = min_max_trust_value[1];
    }
    return error;
}
//132(0x84)
TSS_EXPORT TSS_Error tss_getCurrentUpdateRate(TSS_Device_Id device, unsigned int * last_update, unsigned int * timestamp){
    return writeRead(device, &simple_commands[TSS_GET_CURRENT_UPDATE_RATE],NULL,(char *)last_update,timestamp);
}
//133(0x85)
TSS_EXPORT TSS_Error tss_getCompassReferenceVector(TSS_Device_Id device, float * reference_vector, unsigned int * timestamp){
    return writeRead(device, &simple_commands[TSS_GET_COMPASS_REFERENCE_VECTOR],NULL,(char *)reference_vector,timestamp);
}
//134(0x86)
TSS_EXPORT TSS_Error tss_getAccelerometerReferenceVector(TSS_Device_Id device, float * reference_vector, unsigned int * timestamp){
    return writeRead(device, &simple_commands[TSS_GET_ACCELEROMETER_REFERENCE_VECTOR],NULL,(char *)reference_vector,timestamp);
}
//135(0x87)
TSS_EXPORT TSS_Error tss_getReferenceVectorMode(TSS_Device_Id device, unsigned char * mode, unsigned int * timestamp){
    return writeRead(device, &simple_commands[TSS_GET_REFERENCE_VECTOR_MODE],NULL,(char *)mode,timestamp);
}
//140(0x8c)
TSS_EXPORT TSS_Error tss_getGyroscopeEnabledState(TSS_Device_Id device, unsigned char * enabled, unsigned int * timestamp){
    return writeRead(device, &simple_commands[TSS_GET_GYROSCOPE_ENABLED_STATE],NULL,(char *)enabled,timestamp);
}
//141(0x8d)
TSS_EXPORT TSS_Error tss_getAccelerometerEnabledState(TSS_Device_Id device, unsigned char * enabled, unsigned int * timestamp){
    return writeRead(device, &simple_commands[TSS_GET_ACCELEROMETER_ENABLED_STATE],NULL,(char *)enabled,timestamp);
}
//142(0x8e)
TSS_EXPORT TSS_Error tss_getCompassEnabledState(TSS_Device_Id device, unsigned char * enabled, unsigned int * timestamp){
    return writeRead(device, &simple_commands[TSS_GET_COMPASS_ENABLED_STATE],NULL,(char *)enabled,timestamp);
}
//143(0x8f)
TSS_EXPORT TSS_Error tss_getAxisDirections(TSS_Device_Id device, unsigned char * axis_directions, unsigned int * timestamp){
    return writeRead(device, &simple_commands[TSS_GET_AXIS_DIRECTIONS],NULL,(char *)axis_directions,timestamp);
}
//144(0x90)
TSS_EXPORT TSS_Error tss_getOversampleRate(TSS_Device_Id device, unsigned char * oversample_rate, unsigned int * timestamp){
    return writeRead(device, &simple_commands[TSS_GET_OVERSAMPLE_RATE],NULL,(char *)oversample_rate,timestamp);
}
//145(0x91)
TSS_EXPORT TSS_Error tss_getRunningAveragePercent(TSS_Device_Id device, float * running_average_percent, unsigned int * timestamp){
    return writeRead(device, &simple_commands[TSS_GET_RUNNING_AVERAGE_PERCENT],NULL,(char *)running_average_percent,timestamp);
}
//146(0x92)
TSS_EXPORT TSS_Error tss_getDesiredUpdateRate(TSS_Device_Id device, unsigned int * update_rate, unsigned int * timestamp){
    return writeRead(device, &simple_commands[TSS_GET_DESIRED_UPDATE_RATE],NULL,(char *)update_rate,timestamp);
}
//148(0x94)
TSS_EXPORT TSS_Error tss_getAccelerometerRange(TSS_Device_Id device,unsigned char * accelerometer_range_setting, unsigned int * timestamp){
    return writeRead(device, &simple_commands[TSS_GET_ACCELEROMETER_RANGE],NULL,(char *)accelerometer_range_setting,timestamp);
}
//152(0x98)
TSS_EXPORT TSS_Error tss_getFilterMode(TSS_Device_Id device, unsigned char * mode, unsigned int * timestamp){
    return writeRead(device, &simple_commands[TSS_GET_FILTER_MODE],NULL,(char *)mode,timestamp);
}
//153(0x99)
TSS_EXPORT TSS_Error tss_getRunningAverageMode(TSS_Device_Id device, unsigned char * mode, unsigned int * timestamp){
    return writeRead(device, &simple_commands[TSS_GET_RUNNING_AVERAGE_MODE],NULL,(char *)mode, timestamp);
}
//154(0x9a)
TSS_EXPORT TSS_Error tss_getGyroscopeRange(TSS_Device_Id device, unsigned char * gyroscope_range_setting, unsigned int * timestamp){
    return writeRead(device, &simple_commands[TSS_GET_GYROSCOPE_RANGE],NULL,(char *)gyroscope_range_setting,timestamp);
}
//155(0x9b)
TSS_EXPORT TSS_Error tss_getCompassRange(TSS_Device_Id device, unsigned char * compass_range_setting, unsigned int * timestamp){
    return writeRead(device, &simple_commands[TSS_GET_COMPASS_RANGE],NULL,(char *)compass_range_setting, timestamp);
}
//156(0x9c)
TSS_EXPORT TSS_Error tss_getEulerAngleDecompositionOrder(TSS_Device_Id device, unsigned char * order, unsigned int * timestamp){
    return writeRead(device, &simple_commands[TSS_GET_EULER_ANGLE_DECOMPOSITION_ORDER],NULL,(char *)order,timestamp);
}
//157(0x9d)
TSS_EXPORT TSS_Error tss_getMagnetoresistiveThreshold(TSS_Device_Id device, float * threshold, unsigned int * trust_frames, float * lockout_decay, float * perturbation_detection_value, unsigned int * timestamp){
    char threshold_params[16];
    TSS_Error error= writeRead(device, &simple_commands[TSS_GET_MAGNETORESISTIVE_THRESHOLD],NULL,(char *)threshold_params,timestamp);
    if( error == TSS_NO_ERROR){
        memcpy(threshold, &threshold_params[0], sizeof(float));
        memcpy(trust_frames, &threshold_params[4] , sizeof(unsigned int));
        memcpy(lockout_decay, &threshold_params[8] , sizeof(float));
        memcpy(perturbation_detection_value, &threshold_params[12] , sizeof(float));
    }
    return error;
}
//158(0x9e)
TSS_EXPORT TSS_Error tss_getAccelerometerResistanceThreshold(TSS_Device_Id device, float * threshold, unsigned int * lockout_frames, unsigned int * timestamp){
    char threshold_params[8];
    TSS_Error error= writeRead(device, &simple_commands[TSS_GET_ACCELEROMETER_RESISTANCE_THRESHOLD],NULL,(char *)threshold_params,timestamp);
    if( error == TSS_NO_ERROR){
        memcpy(threshold, &threshold_params[0], sizeof(float));
        memcpy(lockout_frames, &threshold_params[4] , sizeof(unsigned int));
    }
    return error;
}
//159(0x9f)
TSS_EXPORT TSS_Error tss_getOffsetOrientationAsQuaternion(TSS_Device_Id device, float * quat4, unsigned int * timestamp){
    return writeRead(device, &simple_commands[TSS_GET_OFFSET_ORIENTATION_AS_QUATERNION],NULL,(char *)quat4,timestamp);
}

//Calibration Commands
//160(0xa0)
TSS_EXPORT TSS_Error tss_setCompassCalibrationCoefficients(TSS_Device_Id device,const float * matrix9, const float * bias3, unsigned int * timestamp){
    float matrix_bias[12];
    memcpy(&matrix_bias[0],matrix9,sizeof(float)*9);
    memcpy(&matrix_bias[9],bias3,sizeof(float)*3);
    return writeRead(device, &simple_commands[TSS_SET_COMPASS_CALIBRATION_COEFFICIENTS],(char *)&matrix_bias, NULL, timestamp);
}
//161(0xa1)
TSS_EXPORT TSS_Error tss_setAccelerometerCalibrationCoefficients(TSS_Device_Id device,const float * matrix9, const float * bias3, unsigned int * timestamp){
    float matrix_bias[12];
    memcpy(&matrix_bias[0],matrix9,sizeof(float)*9);
    memcpy(&matrix_bias[9],bias3,sizeof(float)*3);
    return writeRead(device, &simple_commands[TSS_SET_ACCELEROMETER_CALIBRATION_COEFFICIENTS],(char *)&matrix_bias, NULL, timestamp);
}
//162(0xa2)
TSS_EXPORT TSS_Error tss_getCompassCalibrationCoefficients(TSS_Device_Id device, float * matrix9, float * bias3, unsigned int * timestamp){
    float matrix_bias[12];
    TSS_Error error= writeRead(device, &simple_commands[TSS_GET_COMPASS_CALIBRATION_COEFFICIENTS],NULL,(char *)matrix_bias,timestamp);
    if(error == TSS_NO_ERROR){
        memcpy(matrix9,&matrix_bias[0],sizeof(float)*9);
        memcpy(bias3,&matrix_bias[9],sizeof(float)*3);
    }
    return error;
}
//163(0xa3)
TSS_EXPORT TSS_Error tss_getAccelerometerCalibrationCoefficients(TSS_Device_Id device, float * matrix9, float * bias3, unsigned int * timestamp){
    float matrix_bias[12];
    TSS_Error error= writeRead(device, &simple_commands[TSS_GET_ACCELEROMETER_CALIBRATION_COEFFICIENTS],NULL,(char *)matrix_bias,timestamp);
    if(error == TSS_NO_ERROR){
        memcpy(matrix9,&matrix_bias[0],sizeof(float)*9);
        memcpy(bias3,&matrix_bias[9],sizeof(float)*3);
    }
    return error;
}
//164(0xa4)
TSS_EXPORT TSS_Error tss_getGyroscopeCalibrationCoefficients(TSS_Device_Id device, float * matrix9, float * bias3, unsigned int * timestamp){
    float matrix_bias[12];
    TSS_Error error= writeRead(device, &simple_commands[TSS_GET_GYROSCOPE_CALIBRATION_COEFFICIENTS],NULL,(char *)matrix_bias,timestamp);
    if(error == TSS_NO_ERROR){
        memcpy(matrix9,&matrix_bias[0],sizeof(float)*9);
        memcpy(bias3,&matrix_bias[9],sizeof(float)*3);
    }
    return error;
}
//165(0xa5)
TSS_EXPORT TSS_Error tss_beginGyroscopeAutoCalibration(TSS_Device_Id device, unsigned int * timestamp){
    return writeRead(device, &simple_commands[TSS_BEGIN_GYROSCOPE_AUTO_CALIBRATION],NULL, NULL, timestamp);
}
//166(0xa6)
TSS_EXPORT TSS_Error tss_setGyroscopeCalibrationCoefficients(TSS_Device_Id device, const float * matrix9, const float * bias3, unsigned int * timestamp){
    float matrix_bias[12];
    memcpy(&matrix_bias[0],matrix9,sizeof(float)*9);
    memcpy(&matrix_bias[9],bias3,sizeof(float)*3);
    return writeRead(device, &simple_commands[TSS_SET_GYROSCOPE_CALIBRATION_COEFFICIENTS],(char *)&matrix_bias, NULL, timestamp);
}
//169(0xa9)
TSS_EXPORT TSS_Error tss_setCalibrationMode(TSS_Device_Id device, unsigned char mode, unsigned int * timestamp){
    return writeRead(device, &simple_commands[TSS_SET_CALIBRATION_MODE],(char *)&mode,NULL,timestamp);
}
//170(0xaa)
TSS_EXPORT TSS_Error tss_getCalibrationMode(TSS_Device_Id device, unsigned char * mode, unsigned int * timestamp){
    return writeRead(device, &simple_commands[TSS_GET_CALIBRATION_MODE],NULL,(char *)mode,timestamp);
}
//171(0xab)
TSS_EXPORT TSS_Error tss_setOrthoCalibrationDataPointFromCurrentOrientation(TSS_Device_Id device, unsigned int * timestamp){
    return writeRead(device, &simple_commands[TSS_SET_ORTHO_CALIBRATION_DATA_POINT_FROM_CURRENT_ORIENTATION],NULL, NULL, timestamp);
}
//172(0xac)
TSS_EXPORT TSS_Error tss_setOrthoCalibrationDataPointFromVector(TSS_Device_Id device, unsigned char type, unsigned char index, const float * vector3, unsigned int * timestamp){
    unsigned char type_index_vector[14];
    type_index_vector[0] = type;
    type_index_vector[1] = index;
    memcpy(&type_index_vector[2],vector3,sizeof(float)*3);
    return writeRead(device, &simple_commands[TSS_SET_ORTHO_CALIBRATION_DATA_POINT_FROM_VECTOR],(char *)&type_index_vector, NULL, timestamp);
}
//173(0xad)
TSS_EXPORT TSS_Error tss_getOrthoCalibrationDataPoint(TSS_Device_Id device, unsigned char type, unsigned char index, float * vector3, unsigned int * timestamp){
    unsigned char type_int[2] = {type,index};
    return writeRead(device, &simple_commands[TSS_GET_ORTHO_CALIBRATION_DATA_POINT],(char *)&type_int, (char *)vector3, timestamp);
}
//174(0xae)
TSS_EXPORT TSS_Error tss_performOrthoCalibration(TSS_Device_Id device, unsigned int * timestamp){
    return writeRead(device, &simple_commands[TSS_PERFORM_ORTHO_CALIBRATION],NULL, NULL,timestamp);
}
//175(0xaf)
TSS_EXPORT TSS_Error tss_clearOrthoCalibrationData(TSS_Device_Id device, unsigned int * timestamp){
    return writeRead(device, &simple_commands[TSS_CLEAR_ORTHO_CALIBRATION_DATA],NULL,NULL,timestamp);
}

//Dongle Commands
//176(0xb0)
TSS_EXPORT TSS_Error tss_setWirelessStreamingAutoFlushMode(TSS_Device_Id device,unsigned char mode, unsigned int * timestamp){
    return writeRead(device, &simple_commands[TSS_SET_WIRELESS_STREAMING_AUTO_FLUSH_MODE],(char *)&mode,NULL,timestamp);
}
//177(0xb1)
TSS_EXPORT TSS_Error tss_getWirelessStreamingAutoFlushMode(TSS_Device_Id device, unsigned char * mode, unsigned int * timestamp){
    return writeRead(device, &simple_commands[TSS_GET_WIRELESS_STREAMING_AUTO_FLUSH_MODE],NULL,(char *)mode,timestamp);
}
//178(0xb2)
TSS_EXPORT TSS_Error tss_setWirelessStreamingManualFlushBitfield(TSS_Device_Id device, unsigned short manual_flush_bitfield, unsigned int * timestamp){
    return writeRead(device, &simple_commands[TSS_SET_WIRELESS_STREAMING_MANUAL_FLUSH_BITFIELD],(char *)&manual_flush_bitfield,NULL,timestamp);
}
//179(0xb3)
TSS_EXPORT TSS_Error tss_getWirelessStreamingManualFlushBitfield(TSS_Device_Id device, unsigned short * manual_flush_bitfield, unsigned int * timestamp){
    return writeRead(device, &simple_commands[TSS_GET_WIRELESS_STREAMING_MANUAL_FLUSH_BITFIELD],NULL,(char *)manual_flush_bitfield,timestamp);
}
//180(0xb4)
TSS_EXPORT TSS_Error tss_getManualFlushSingle(TSS_Device_Id device,unsigned char index, char * data, int in_data_size, int * out_data_size, unsigned int * timestamp){ //TODO
    //Get device
    //get sizeof the stream packet
    //check indata to it
    //assign outdata size
    //special case write
    return TSS_INVALID_COMMAND;
    //return writeRead(device, &simple_commands[TSS_GET_MANUAL_FLUSH_SINGLE],(char*)&index,(char *)data,timestamp);
}
//181(0xb5)
TSS_EXPORT TSS_Error tss_getManualFlushBulk(TSS_Device_Id device, char * data, int in_data_size, int * out_data_size, unsigned int * timestamp){ //TODO
    return TSS_INVALID_COMMAND;
    //return writeRead(device, &simple_commands[TSS_GET_MANUAL_FLUSH_BULK],NULL,(char *)mode,timestamp);
}
//182(0xb6)
TSS_EXPORT TSS_Error tss_broadcastSynchronizationPulse(TSS_Device_Id device, unsigned int * timestamp){
    return writeRead(device, &simple_commands[TSS_BROADCAST_SYNCHRONIZATION_PULSE],NULL,NULL,timestamp);
}
//208(0xd0)
TSS_EXPORT TSS_Error tss_getSerialNumberAtLogicalID(TSS_Device_Id device, unsigned char logical_id, unsigned int * serial_number, unsigned int * timestamp){
    return writeRead(device, &simple_commands[TSS_GET_SERIAL_NUMBER_AT_LOGICAL_ID],(char *)&logical_id,(char *)serial_number,timestamp);
}
//209(0xd1)
TSS_EXPORT TSS_Error tss_setSerialNumberAtLogicalID(TSS_Device_Id device, unsigned char logical_id, unsigned int serial_number, unsigned int * timestamp){
    unsigned char logical_id_serial_number[5];
    logical_id_serial_number[0]=logical_id;
    memcpy(&logical_id_serial_number[1],&serial_number,sizeof(float));
    return writeRead(device, &simple_commands[TSS_SET_SERIAL_NUMBER_AT_LOGICAL_ID],(char *)&logical_id_serial_number,NULL,timestamp);
}
//210(0xd2)
TSS_EXPORT TSS_Error tss_getWirelessChannelNoiseLevels(TSS_Device_Id device, unsigned char * channel_strengths16, unsigned int * timestamp){
    return writeRead(device, &simple_commands[TSS_GET_WIRELESS_CHANNEL_NOISE_LEVELS],NULL,(char *)channel_strengths16,timestamp);
}
//211(0xd3)
TSS_EXPORT TSS_Error tss_setWirelessRetries(TSS_Device_Id device,unsigned char retries, unsigned int * timestamp){
    return writeRead(device, &simple_commands[TSS_SET_WIRELESS_RETRIES],(char *)&retries,NULL,timestamp);
}
//212(0xd4)
TSS_EXPORT TSS_Error tss_getWirelessRetries(TSS_Device_Id device, unsigned char * retries, unsigned int * timestamp){
    return writeRead(device, &simple_commands[TSS_GET_WIRELESS_RETRIES],NULL,(char *)retries,timestamp);
}
//213(0xd5)
TSS_EXPORT TSS_Error tss_getWirelessSlotsOpen(TSS_Device_Id device, unsigned char * slots_open, unsigned int * timestamp){
    return writeRead(device, &simple_commands[TSS_GET_WIRELESS_SLOTS_OPEN],NULL,(char *)slots_open,timestamp);
}
//214(0xd6)
TSS_EXPORT TSS_Error tss_getSignalStrength(TSS_Device_Id device, unsigned char * last_packet_signal_strength, unsigned int * timestamp){
    return writeRead(device, &simple_commands[TSS_GET_SIGNAL_STRENGTH],NULL,(char *)last_packet_signal_strength,timestamp);
}
//219(0xdb)
TSS_EXPORT TSS_Error tss_setWirelessResponseHeaderBitfield(TSS_Device_Id device, unsigned int header_bitfield, unsigned int * timestamp){
    return writeRead(device, &simple_commands[TSS_SET_WIRELESS_RESPONSE_HEADER_BITFIELD],(char *)&header_bitfield,NULL,timestamp);
}
//220(0xdc)
TSS_EXPORT TSS_Error tss_getWirelessResponseHeaderBitfield(TSS_Device_Id device, unsigned int * header_bitfield, unsigned int * timestamp){
    return writeRead(device, &simple_commands[TSS_GET_WIRELESS_RESPONSE_HEADER_BITFIELD],NULL,(char *)header_bitfield,timestamp);
}

//Wireless Sensor & Dongle Commands
//192(0xc0)
TSS_EXPORT TSS_Error tss_getWirelessPanID(TSS_Device_Id device, unsigned short * pan_id, unsigned int * timestamp){
    return writeRead(device, &simple_commands[TSS_GET_WIRELESS_PAN_ID],NULL,(char *)pan_id,timestamp);
}
//193(0xc1)
TSS_EXPORT TSS_Error tss_setWirelessPanID(TSS_Device_Id device, unsigned short pan_id, unsigned int * timestamp){
    return writeRead(device, &simple_commands[TSS_SET_WIRELESS_PAN_ID],(char *)&pan_id,NULL,timestamp);
}
//194(0xc2)
TSS_EXPORT TSS_Error tss_getWirelessChannel(TSS_Device_Id device, unsigned char * channel, unsigned int * timestamp){
    return writeRead(device, &simple_commands[TSS_GET_WIRELESS_CHANNEL],NULL,(char *)channel,timestamp);
}
//195(0xc3)
TSS_EXPORT TSS_Error tss_setWirelessChannel(TSS_Device_Id device, unsigned char channel, unsigned int * timestamp){
    return writeRead(device, &simple_commands[TSS_SET_WIRELESS_CHANNEL],(char *)&channel,NULL,timestamp);
}
//197(0xc5)
TSS_EXPORT TSS_Error tss_commitWirelessSettings(TSS_Device_Id device, unsigned int * timestamp){
    return writeRead(device, &simple_commands[TSS_COMMIT_WIRELESS_SETTINGS],NULL, NULL, timestamp);
}
//198(0xc6)
TSS_EXPORT TSS_Error tss_getWirelessAddress(TSS_Device_Id device, unsigned short * address, unsigned int * timestamp){
    return writeRead(device, &simple_commands[TSS_GET_WIRELESS_ADDRESS],NULL,(char *)address,timestamp);
}

//Battery Commands
//201(0xc9)
TSS_EXPORT TSS_Error tss_getBatteryVoltage(TSS_Device_Id device, float * battery_voltage, unsigned int * timestamp){
    return writeRead(device, &simple_commands[TSS_GET_BATTERY_VOLTAGE],NULL,(char *)battery_voltage,timestamp);
}
//202(0xca)
TSS_EXPORT TSS_Error tss_getBatteryPercentRemaining(TSS_Device_Id device, unsigned char * battery_percent, unsigned int * timestamp){
    return writeRead(device, &simple_commands[TSS_GET_BATTERY_PERCENT_REMAINING],NULL,(char *)battery_percent,timestamp);
}
//203(0xcb)
TSS_EXPORT TSS_Error tss_getBatteryStatus(TSS_Device_Id device, unsigned char * battery_charge_status, unsigned int * timestamp){
    return writeRead(device, &simple_commands[TSS_GET_BATTERY_STATUS],NULL,(char *)battery_charge_status,timestamp);
}

//General Commands
//196(0xc4)
TSS_EXPORT TSS_Error tss_setLEDMode(TSS_Device_Id device, unsigned char mode, unsigned int * timestamp){
    return writeRead(device, &simple_commands[TSS_SET_LED_MODE],(char *)&mode,NULL, timestamp);
}
//200(0xc8)
TSS_EXPORT TSS_Error tss_getLEDMode(TSS_Device_Id device, unsigned char * mode, unsigned int * timestamp){
    return writeRead(device, &simple_commands[TSS_GET_LED_MODE],NULL,(char *)mode, timestamp);
}
//221(0xdd)
TSS_EXPORT TSS_Error tss_setWiredResponseHeaderBitfield(TSS_Device_Id device, unsigned int header_bitfield, unsigned int * timestamp){
    return writeRead(device, &simple_commands[TSS_SET_WIRED_RESPONSE_HEADER_BITFIELD],(char *)&header_bitfield,NULL, timestamp);
}
//222(0xde)
TSS_EXPORT TSS_Error tss_getWiredResponseHeaderBitfield(TSS_Device_Id device, unsigned int * header_bitfield, unsigned int * timestamp){
    return writeRead(device, &simple_commands[TSS_GET_WIRED_RESPONSE_HEADER_BITFIELD],NULL,(char *)header_bitfield,timestamp);
}
//223(0xdf) note: adds null terminator
TSS_EXPORT TSS_Error tss_getFirmwareVersionString(TSS_Device_Id device, char * firmware_version13, unsigned int * timestamp){
    TSS_Error error =writeRead(device, &simple_commands[TSS_GET_FIRMWARE_VERSION_STRING],NULL,(char *)firmware_version13,timestamp);
    if(error == TSS_NO_ERROR){
        firmware_version13[12]=0;
    }
    return error;
}
//224(0xe0)
TSS_EXPORT TSS_Error tss_restoreFactorySettings(TSS_Device_Id device, unsigned int * timestamp){
    return writeRead(device, &simple_commands[TSS_RESTORE_FACTORY_SETTINGS],NULL, NULL, timestamp);
}
//225(0xe1)
TSS_EXPORT TSS_Error tss_commitSettings(TSS_Device_Id device, unsigned int * timestamp){
    return writeRead(device, &simple_commands[TSS_COMMIT_SETTINGS],NULL, NULL, timestamp);
}
//226(0xe2)
TSS_EXPORT TSS_Error tss_softwareReset(TSS_Device_Id device, unsigned int * timestamp){
    return writeRead(device, &simple_commands[TSS_SOFTWARE_RESET],NULL, NULL, timestamp);
}
//227(0xe3)
TSS_EXPORT TSS_Error tss_setSleepMode(TSS_Device_Id device,unsigned char mode, unsigned int * timestamp){
    return writeRead(device, &simple_commands[TSS_SET_SLEEP_MODE],(char *)&mode,NULL,timestamp);
}
//228(0xe4)
TSS_EXPORT TSS_Error tss_getSleepMode(TSS_Device_Id device, unsigned char * mode, unsigned int * timestamp){
    return writeRead(device, &simple_commands[TSS_GET_SLEEP_MODE],NULL,(char *)mode,timestamp);
}
//229(0xe5)
TSS_EXPORT TSS_Error tss_enterBootloaderMode(TSS_Device_Id device, unsigned int * timestamp){
    return writeRead(device, &simple_commands[TSS_ENTER_BOOTLOADER_MODE],NULL, NULL, timestamp);
}
//230(0xe6) note: adds null terminator
TSS_EXPORT TSS_Error tss_getHardwareVersionString(TSS_Device_Id device, char * hardware_version33, unsigned int * timestamp){
    TSS_Error error =writeRead(device, &simple_commands[TSS_GET_HARDWARE_VERSION_STRING],NULL,(char *)hardware_version33,timestamp);
    if(error == TSS_NO_ERROR){
        hardware_version33[32]=0;
    }
    return error;
}
//231(0xe7)
TSS_EXPORT TSS_Error tss_setUARTBaudRate(TSS_Device_Id device, unsigned int baud_rate, unsigned int * timestamp){
    return writeRead(device, &simple_commands[TSS_SET_UART_BAUD_RATE],(char *)&baud_rate,NULL,timestamp);
}
//232(0xe8)
TSS_EXPORT TSS_Error tss_getUARTBaudRate(TSS_Device_Id device, unsigned int * baud_rate, unsigned int * timestamp){
    return writeRead(device, &simple_commands[TSS_GET_UART_BAUD_RATE],NULL,(char *)baud_rate,timestamp);
}
//233(0xe9)
TSS_EXPORT TSS_Error tss_setUSBMode(TSS_Device_Id device, unsigned int mode, unsigned int * timestamp){
    return writeRead(device, &simple_commands[TSS_SET_USB_MODE],(char *)&mode,NULL,timestamp);
}
//234(0xea)
TSS_EXPORT TSS_Error tss_getUSBMode(TSS_Device_Id device, unsigned int * mode, unsigned int * timestamp){
    return writeRead(device, &simple_commands[TSS_GET_USB_MODE],NULL,(char *)mode,timestamp);
}
//237(0xed)
TSS_EXPORT TSS_Error tss_getSerialNumber(TSS_Device_Id device, unsigned int * serial_number, unsigned int * timestamp){
    return writeRead(device, &simple_commands[TSS_GET_SERIAL_NUMBER],NULL,(char *)serial_number,timestamp);
}
//238(0xee)
TSS_EXPORT TSS_Error tss_setLEDColor(TSS_Device_Id device,const float * rgb_color3, unsigned int * timestamp){
    return writeRead(device, &simple_commands[TSS_SET_LED_COLOR],(char *)rgb_color3,NULL,timestamp);
}
//239(0xef)
TSS_EXPORT TSS_Error tss_getLEDColor(TSS_Device_Id device, float * rgb_color3, unsigned int * timestamp){
    return writeRead(device, &simple_commands[TSS_GET_LED_COLOR],NULL,(char *)rgb_color3,timestamp);
}

//Wireless HID Commands
TSS_EXPORT TSS_Error tss_setWirelessHIDUpdateRate(TSS_Device_Id device, unsigned char HID_update_rate, unsigned int * timestamp){
    return writeRead(device, &simple_commands[TSS_SET_WIRELESS_HID_UPDATE_RATE],(char *)&HID_update_rate,NULL,timestamp);
}

TSS_EXPORT TSS_Error tss_getWirelessHIDUpdateRate(TSS_Device_Id device, unsigned char * HID_update_rate, unsigned int * timestamp){
    return writeRead(device, &simple_commands[TSS_GET_WIRELESS_HID_UPDATE_RATE],NULL,(char *)HID_update_rate,timestamp);
}

TSS_EXPORT TSS_Error tss_setWirelessHIDAsynchronousMode(TSS_Device_Id device, unsigned char HID_communication_mode , unsigned int * timestamp){
    return writeRead(device, &simple_commands[TSS_SET_WIRELESS_HID_ASYNCHRONOUS_MODE],(char *)&HID_communication_mode, NULL,timestamp);
}

TSS_EXPORT TSS_Error tss_getWirelessHIDAsynchronousMode(TSS_Device_Id device, unsigned char * HID_communication_mode , unsigned int * timestamp){
    return writeRead(device, &simple_commands[TSS_GET_WIRELESS_HID_ASYNCHRONOUS_MODE],NULL,(char *)HID_communication_mode,timestamp);
}

TSS_EXPORT TSS_Error tss_setJoystickLogicalID(TSS_Device_Id device, unsigned char logical_ID , unsigned int * timestamp){
    return writeRead(device, &simple_commands[TSS_SET_JOYSTICK_LOGICAL_ID],(char *)&logical_ID, NULL,timestamp);
}

TSS_EXPORT TSS_Error tss_setMouseLogicalID(TSS_Device_Id device, unsigned char * logical_ID , unsigned int * timestamp){
    return writeRead(device, &simple_commands[TSS_SET_MOUSE_LOGICAL_ID],(char *)&logical_ID, NULL,timestamp);
}

TSS_EXPORT TSS_Error tss_getJoystickLogicalID(TSS_Device_Id device, unsigned char * logical_ID , unsigned int * timestamp){
    return writeRead(device, &simple_commands[TSS_GET_JOYSTICK_LOGICAL_ID],NULL,(char *)logical_ID,timestamp);
}

TSS_EXPORT TSS_Error tss_getMouseLogicalID(TSS_Device_Id device, unsigned char * logical_ID , unsigned int * timestamp){
    return writeRead(device, &simple_commands[TSS_GET_MOUSE_LOGICAL_ID],NULL,(char *)logical_ID,timestamp);
}


//Wired HID Commands
TSS_EXPORT TSS_Error tss_setJoystickEnabled(TSS_Device_Id device, unsigned char enabled_state , unsigned int * timestamp){
    return writeRead(device, &simple_commands[TSS_SET_JOYSTICK_ENABLED],(char *)&enabled_state, NULL,timestamp);
}

TSS_EXPORT TSS_Error tss_setMouseEnabled(TSS_Device_Id device, unsigned char enabled_state , unsigned int * timestamp){
    return writeRead(device, &simple_commands[TSS_SET_MOUSE_ENABLED],(char *)&enabled_state, NULL,timestamp);
}

TSS_EXPORT TSS_Error tss_getJoystickEnabled(TSS_Device_Id device, unsigned char * enabled_state , unsigned int * timestamp){
    return writeRead(device, &simple_commands[TSS_GET_JOYSTICK_ENABLED],NULL,(char *)enabled_state,timestamp);
}

TSS_EXPORT TSS_Error tss_getMouseEnabled(TSS_Device_Id device, unsigned char * enabled_state , unsigned int * timestamp){
    return writeRead(device, &simple_commands[TSS_GET_MOUSE_ENABLED],NULL,(char *)enabled_state,timestamp);
}

//General HID Commands
TSS_EXPORT TSS_Error tss_setControlMode(TSS_Device_Id device, unsigned char control_class, unsigned char control_index, unsigned char handler_index, unsigned int * timestamp){
    unsigned char args[3] = {control_class,control_index, handler_index};
    return writeRead(device, &simple_commands[TSS_SET_WIRELESS_HID_ASYNCHRONOUS_MODE],(char *)args, NULL,timestamp);
}

TSS_EXPORT TSS_Error tss_setControlData(TSS_Device_Id device, unsigned char control_class, unsigned char control_index, unsigned char data_point_index, float data_point, unsigned int * timestamp){
    unsigned char args[7] = {control_class, control_index, data_point_index, 0,0,0,0};
    memcpy(&args[3],&data_point, sizeof(float));
    return writeRead(device, &simple_commands[TSS_SET_CONTROL_DATA],(char *)args, NULL,timestamp);
}

TSS_EXPORT TSS_Error tss_getControlMode(TSS_Device_Id device, unsigned char control_class, unsigned char control_index, unsigned char * handler_index, unsigned int * timestamp){
    unsigned char args[2] = {control_class,control_index};
    return writeRead(device, &simple_commands[TSS_GET_CONTROL_MODE],(char *)args, (char *)handler_index,timestamp);
}

TSS_EXPORT TSS_Error tss_getControlData(TSS_Device_Id device, unsigned char control_class, unsigned char control_index, unsigned char data_point_index, float * data_point, unsigned int * timestamp){
    unsigned char args[3] = {control_class,control_index, data_point_index};
    return writeRead(device, &simple_commands[TSS_GET_CONTROL_DATA],(char *)args, (char *)data_point,timestamp);
}

TSS_EXPORT TSS_Error tss_setButtonGyroDisableLength(TSS_Device_Id device, unsigned char number_of_frames, unsigned int * timestamp){
    return writeRead(device, &simple_commands[TSS_SET_BUTTON_GYRO_DISABLE_LENGTH],(char *)&number_of_frames, NULL,timestamp);
}

TSS_EXPORT TSS_Error tss_getButtonGyroDisableLength(TSS_Device_Id device, unsigned char * number_of_frames, unsigned int * timestamp){
    return writeRead(device, &simple_commands[TSS_GET_BUTTON_GYRO_DISABLE_LENGTH],NULL,(char *)number_of_frames,timestamp);
}

TSS_EXPORT TSS_Error tss_getButtonState(TSS_Device_Id device, unsigned char * button_state, unsigned int * timestamp){
    return writeRead(device, &simple_commands[TSS_GET_BUTTON_STATE],NULL,(char *)button_state,timestamp);
}

TSS_EXPORT TSS_Error tss_setMouseAbsoluteRelativeMode(TSS_Device_Id device, unsigned char mode, unsigned int * timestamp){
    return writeRead(device, &simple_commands[TSS_SET_MOUSE_ABSOLUTE_RELATIVE_MODE],(char *)&mode, NULL,timestamp);
}

TSS_EXPORT TSS_Error tss_getMouseAbsoluteRelativeMode(TSS_Device_Id device, unsigned char * mode, unsigned int * timestamp){
    return writeRead(device, &simple_commands[TSS_GET_MOUSE_ABSOLUTE_RELATIVE_MODE],NULL,(char *)mode,timestamp);
}

TSS_EXPORT TSS_Error tss_setJoystickAndMousePresentRemoved(TSS_Device_Id device, unsigned char joystick, unsigned char mouse, unsigned int * timestamp){
    unsigned char args[2] = {joystick,mouse};
    return writeRead(device, &simple_commands[TSS_SET_JOYSTICK_AND_MOUSE_PRESENT_REMOVED],(char *)args, NULL,timestamp);
}

TSS_EXPORT TSS_Error tss_getJoystickAndMousePresentRemoved(TSS_Device_Id device, unsigned char * joystick, unsigned char * mouse, unsigned int * timestamp){
    unsigned char args[2];
    TSS_Error error = writeRead(device, &simple_commands[TSS_GET_JOYSTICK_AND_MOUSE_PRESENT_REMOVED],NULL,(char *)args,timestamp);
    if(error == TSS_NO_ERROR){
        *joystick = args[0];
        *mouse = args[1];
    }
    return error;
}




