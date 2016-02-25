/********************************************//**
 * \file yei_threespace_core.c
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
#ifndef YEI_THREESPACE_CORE_H_INCLUDED
#define YEI_THREESPACE_CORE_H_INCLUDED
#include <stdio.h>
#include <string.h>
#include <windows.h>
#include <process.h>


#include "yei_enum_ports.h"

typedef enum TSS_Command_Enum
{
    TSS_SET_EULER_ANGLE_DECOMPOSITION_ORDER = 0x10,
    TSS_SET_MAGNETORESISTIVE_THRESHOLD = 0x11,
    TSS_SET_ACCELEROMETER_RESISTANCE_THRESHOLD = 0x12,
    TSS_OFFSET_WITH_CURRENT_ORIENTATION = 0x13,
    TSS_RESET_BASE_OFFSET = 0x14,
    TSS_OFFSET_WITH_QUATERNION = 0x15,
    TSS_SET_BASE_OFFSET_WITH_CURRENT_ORIENTATION = 0x16,
    TSS_SET_CHECK_LONG_COMMANDS = 0x19,
    TSS_SET_PIN_MODE = 0x1d,
    TSS_GET_PIN_MODE = 0x1e,
    TSS_GET_INTERRUPT_STATUS = 0x1f,
    TSS_TURN_ON_MASSS_TORAGE = 0x39,
    TSS_TURN_OFF_MASS_STORAGE = 0x3a,
    TSS_FORMAT_AND_INITIALIZE_SD_CARD = 0x3b,
    TSS_BEGIN_DATA_LOGGING_SESSION = 0x3c,
    TSS_END_DATA_LOGGING_SESSION = 0x3d,
    TSS_SET_CLOCK_VALUES = 0x3e,
    TSS_GET_CLOCK_VALUES = 0x3f,
    TSS_SET_STREAMING_SLOTS = 0x50,
    TSS_GET_STREAMING_SLOTS = 0x51,
    TSS_SET_STREAMING_TIMING = 0x52,
    TSS_GET_STREAMING_TIMING = 0x53,
    TSS_GET_STREAMING_BATCH = 0x54,
    TSS_START_STREAMING = 0x55,
    TSS_STOP_STREAMING = 0x56,
    TSS_UPDATE_CURRENT_TIMESTAMP = 0x5f,
    TSS_TARE_WITH_CURRENT_ORIENTATION = 0x60,
    TSS_TARE_WITH_QUATERNION = 0x61,
    TSS_TARE_WITH_ROTATION_MATRIX = 0x62,
    TSS_SET_STATIC_ACCELEROMETER_TRUST_VALUE = 0x63,
    TSS_SET_CONFIDENCE_ACCELEROMETER_TRUST_VALUES = 0x64,
    TSS_SET_STATIC_COMPASS_TRUST_VALUE = 0x65,
    TSS_SET_CONFIDENCE_COMPASS_TRUST_VALUES = 0x66,
    TSS_SET_DESIRED_UPDATE_RATE = 0x67,
    TSS_SET_MULTI_REFERENCE_VECTORS_WITH_CURRENT_ORIENTATION = 0x68,
    TSS_SET_REFERENCE_VECTOR_MODE = 0x69,
    TSS_SET_OVERSAMPLE_RATE = 0x6a,
    TSS_SET_GYROSCOPE_ENABLED = 0x6b,
    TSS_SET_ACCELEROMETER_ENABLED = 0x6c,
    TSS_SET_COMPASS_ENABLED = 0x6d,
    TSS_RESET_MULTI_REFERENCE_VECTORS_TO_ZERO = 0x6e,
    TSS_SET_MULTI_REFERENCE_TABLE_RESOLUTION = 0x6f,
    TSS_SET_COMPASS_MULTI_REFERENCE_VECTOR = 0x70,
    TSS_SET_COMPASS_MULTI_REFERENCE_CHECK_VECTOR = 0x71,
    TSS_SET_ACCELEROMETER_MULTI_REFERENCE_VECTOR = 0x72,
    TSS_SET_ACCELEROMETER_MULTI_REFERENCE_CHECK_VECTOR = 0x73,
    TSS_SET_AXIS_DIRECTIONS = 0x74,
    TSS_SET_RUNNING_AVERAGE_PERCENT = 0x75,
    TSS_SET_COMPASS_REFERENCE_VECTOR = 0x76,
    TSS_SET_ACCELEROMETER_REFERENCE_VECTOR = 0x77,
    TSS_RESET_KALMAN_FILTER = 0x78,
    TSS_SET_ACCELEROMETER_RANGE = 0x79,
    TSS_SET_MULTI_REFERENCE_WEIGHT_POWER = 0x7a,
    TSS_SET_FILTER_MODE = 0x7b,
    TSS_SET_RUNNING_AVERAGE_MODE = 0x7c,
    TSS_SET_GYROSCOPE_RANGE = 0x7d,
    TSS_SET_COMPASS_RANGE = 0x7e,
    TSS_GET_TARE_AS_QUATERNION = 0x80,
    TSS_GET_TARE_AS_ROTATION_MATRIX = 0x81,
    TSS_GET_ACCELEROMETER_TRUST_VALUES = 0x82,
    TSS_GET_COMPASS_TRUST_VALUES = 0x83,
    TSS_GET_CURRENT_UPDATE_RATE = 0x84,
    TSS_GET_COMPASS_REFERENCE_VECTOR = 0x85,
    TSS_GET_ACCELEROMETER_REFERENCE_VECTOR = 0x86,
    TSS_GET_REFERENCE_VECTOR_MODE = 0x87,
    TSS_GET_COMPASS_MULTI_REFERENCE_VECTOR = 0x88,
    TSS_GET_COMPASS_MULTI_REFERENCE_CHECK_VECTOR = 0x89,
    TSS_GET_ACCELEROMETER_MULTI_REFERENCE_VECTOR = 0x8a,
    TSS_GET_ACCELEROMETER_MULTI_REFERENCE_CHECK_VECTOR = 0x8b,
    TSS_GET_GYROSCOPE_ENABLED_STATE = 0x8c,
    TSS_GET_ACCELEROMETER_ENABLED_STATE = 0x8d,
    TSS_GET_COMPASS_ENABLED_STATE = 0x8e,
    TSS_GET_AXIS_DIRECTIONS = 0x8f,
    TSS_GET_OVERSAMPLE_RATE = 0x90,
    TSS_GET_RUNNING_AVERAGE_PERCENT = 0x91,
    TSS_GET_DESIRED_UPDATE_RATE = 0x92,
    TSS_GET_ACCELEROMETER_RANGE = 0x94,
    TSS_GET_MULTI_REFERENCE_MODE_POWER_WEIGHT = 0x95,
    TSS_GET_MULTI_REFERENCE_RESOLUTION = 0x96,
    TSS_GET_NUMBER_OF_MULTI_REFERENCE_CELLS = 0x97,
    TSS_GET_FILTER_MODE = 0x98,
    TSS_GET_RUNNING_AVERAGE_MODE = 0x99,
    TSS_GET_GYROSCOPE_RANGE = 0x9a,
    TSS_GET_COMPASS_RANGE = 0x9b,
    TSS_GET_EULER_ANGLE_DECOMPOSITION_ORDER = 0x9c,
    TSS_GET_MAGNETORESISTIVE_THRESHOLD = 0X9d,
    TSS_GET_ACCELEROMETER_RESISTANCE_THRESHOLD = 0x9e,
    TSS_GET_OFFSET_ORIENTATION_AS_QUATERNION = 0x9f,
    TSS_SET_COMPASS_CALIBRATION_COEFFICIENTS = 0xa0,
    TSS_SET_ACCELEROMETER_CALIBRATION_COEFFICIENTS = 0xa1,
    TSS_GET_COMPASS_CALIBRATION_COEFFICIENTS = 0xa2,
    TSS_GET_ACCELEROMETER_CALIBRATION_COEFFICIENTS = 0xa3,
    TSS_GET_GYROSCOPE_CALIBRATION_COEFFICIENTS = 0xa4,
    TSS_BEGIN_GYROSCOPE_AUTO_CALIBRATION = 0xa5,
    TSS_SET_GYROSCOPE_CALIBRATION_COEFFICIENTS = 0xa6,
    TSS_SET_CALIBRATION_MODE = 0xa9,
    TSS_GET_CALIBRATION_MODE = 0xaa,
    TSS_SET_ORTHO_CALIBRATION_DATA_POINT_FROM_CURRENT_ORIENTATION = 0xab,
    TSS_SET_ORTHO_CALIBRATION_DATA_POINT_FROM_VECTOR = 0xac,
    TSS_GET_ORTHO_CALIBRATION_DATA_POINT = 0xad,
    TSS_PERFORM_ORTHO_CALIBRATION = 0xae,
    TSS_CLEAR_ORTHO_CALIBRATION_DATA = 0xaf,
    TSS_SET_WIRELESS_STREAMING_AUTO_FLUSH_MODE = 0xb0,
    TSS_GET_WIRELESS_STREAMING_AUTO_FLUSH_MODE = 0xb1,
    TSS_SET_WIRELESS_STREAMING_MANUAL_FLUSH_BITFIELD = 0xb2,
    TSS_GET_WIRELESS_STREAMING_MANUAL_FLUSH_BITFIELD = 0xb3,
    TSS_GET_MANUAL_FLUSH_SINGLE = 0xb4,
    TSS_GET_MANUAL_FLUSH_BULK = 0xb5,
    TSS_BROADCAST_SYNCHRONIZATION_PULSE = 0xb6,
    TSS_GET_RECEPTION_BITFIELD = 0xb7,
    TSS_GET_WIRELESS_PAN_ID = 0xc0,
    TSS_SET_WIRELESS_PAN_ID = 0xc1,
    TSS_GET_WIRELESS_CHANNEL = 0xc2,
    TSS_SET_WIRELESS_CHANNEL = 0xc3,
    TSS_SET_LED_MODE = 0xc4,
    TSS_COMMIT_WIRELESS_SETTINGS = 0xc5,
    TSS_GET_WIRELESS_ADDRESS = 0xc6,
    TSS_GET_LED_MODE = 0xc8,
    TSS_GET_SERIAL_NUMBER_AT_LOGICAL_ID = 0xd0,
    TSS_SET_SERIAL_NUMBER_AT_LOGICAL_ID = 0xd1,
    TSS_GET_WIRELESS_CHANNEL_NOISE_LEVELS = 0xd2,
    TSS_SET_WIRELESS_RETRIES = 0xd3,
    TSS_GET_WIRELESS_RETRIES = 0xd4,
    TSS_GET_WIRELESS_SLOTS_OPEN = 0xd5,
    TSS_GET_SIGNAL_STRENGTH = 0xd6,
    TSS_SET_WIRELESS_HID_UPDATE_RATE = 0xd7,
    TSS_GET_WIRELESS_HID_UPDATE_RATE = 0xd8,
    TSS_SET_WIRELESS_HID_ASYNCHRONOUS_MODE = 0xd9,
    TSS_GET_WIRELESS_HID_ASYNCHRONOUS_MODE = 0xda,
    TSS_SET_WIRELESS_RESPONSE_HEADER_BITFIELD = 0xdb,
    TSS_GET_WIRELESS_RESPONSE_HEADER_BITFIELD = 0xdc,
    TSS_SET_WIRED_RESPONSE_HEADER_BITFIELD = 0xdd,
    TSS_GET_WIRED_RESPONSE_HEADER_BITFIELD = 0xde,
    TSS_GET_FIRMWARE_VERSION_STRING = 0xdf,
    TSS_RESTORE_FACTORY_SETTINGS = 0xe0,
    TSS_COMMIT_SETTINGS = 0xe1,
    TSS_SOFTWARE_RESET = 0xe2,
    TSS_SET_SLEEP_MODE = 0xe3,
    TSS_GET_SLEEP_MODE = 0xe4,
    TSS_ENTER_BOOTLOADER_MODE = 0xe5,
    TSS_GET_HARDWARE_VERSION_STRING = 0xe6,
    TSS_SET_UART_BAUD_RATE = 0xe7,
    TSS_GET_UART_BAUD_RATE = 0xe8,
    TSS_SET_USB_MODE = 0xe9,
    TSS_GET_USB_MODE = 0xea,
    TSS_GET_SERIAL_NUMBER = 0xed,
    TSS_SET_LED_COLOR = 0xee,
    TSS_GET_LED_COLOR = 0xef,
    TSS_SET_JOYSTICK_LOGICAL_ID = 0xf0,
    TSS_SET_MOUSE_LOGICAL_ID = 0xf1,
    TSS_GET_JOYSTICK_LOGICAL_ID = 0xf2,
    TSS_GET_MOUSE_LOGICAL_ID = 0xf3,
    TSS_SET_CONTROL_MODE = 0xf4,
    TSS_SET_CONTROL_DATA = 0xf5,
    TSS_GET_CONTROL_MODE = 0xf6,
    TSS_GET_CONTROL_DATA = 0xf7,
    TSS_SET_BUTTON_GYRO_DISABLE_LENGTH = 0xf8,
    TSS_GET_BUTTON_GYRO_DISABLE_LENGTH = 0xf9,
    TSS_SET_MOUSE_ABSOLUTE_RELATIVE_MODE = 0xfb,
    TSS_GET_MOUSE_ABSOLUTE_RELATIVE_MODE = 0xfc,
    TSS_SET_JOYSTICK_AND_MOUSE_PRESENT_REMOVED = 0xfd,
    TSS_GET_JOYSTICK_AND_MOUSE_PRESENT_REMOVED = 0xfe,
    TSS_SET_JOYSTICK_ENABLED = 0x100,
    TSS_SET_MOUSE_ENABLED = 0x101,
    TSS_GET_JOYSTICK_ENABLED = 0x102,
    TSS_GET_MOUSE_ENABLED = 0x103
}TSS_Command_Enum;

typedef void CALLBACK TSS_CallBack (TSS_Device_Id device, char * output_data,
                          unsigned int output_data_len,  unsigned int * timestamp);

typedef union TSS_Protocol_Header_Setup
{
  struct protocol_bits
  {
    unsigned success_failure : 1;
    unsigned timestamp : 1;
    unsigned command_echo : 1;
    unsigned checksum : 1;
    unsigned logical_id : 1;
    unsigned serial_number : 1;
    unsigned data_length : 1;
    unsigned pad : 1;
  }protocol_bits;
  unsigned char protocol_byte;
}TSS_Protocol_Header_Setup;

typedef struct TSS_Command{
    unsigned char command;
    char * description_str;
    unsigned char rtn_data_len;
    char * rtn_data_detail;
    unsigned char in_data_len;
    char * in_data_detail;
    unsigned int compatibility_mask;
    TSS_Firmware_Compatibility fw_compatibility;
} TSS_Command;

const TSS_Command simple_commands[260];

typedef struct TSS_Sensor{
    TSS_Type device_type;
    TSS_Firmware_Compatibility fw_compatibility;
    unsigned int device_serial;
    unsigned int device_id;
    int baudrate;
    char logical_id;
    unsigned int dongle;
    char protocol_header[8];
    TSS_Command_Enum stream_slots[9];
    char stream_parse_str[257];
    char stream_byte_len;
    char stream_enabled;
    char * last_stream_data;
    unsigned int last_stream_timestamp;
//    char last_command_data[256];
    void * last_header_data;
    char last_out_data[256];
    unsigned int record_count;
    TSS_Timestamp_Mode timestamp_mode;

    //Thread stuff
    volatile char is_active;
    //Windows Stuff
    HANDLE new_data_event;
    HANDLE reader_event;
    HANDLE writer_event;
    CRITICAL_SECTION reader_lock;
    CRITICAL_SECTION stream_lock;
    HANDLE reader_thread;
    HANDLE serial_port;
    TSS_CallBack * callback;
} TSS_Sensor;

typedef struct TSS_Dongle{
    TSS_Type device_type;
    TSS_Firmware_Compatibility fw_compatibility;
    unsigned int device_serial;
    unsigned int device_id;
    int baudrate;
    unsigned int w_sensors[15];
    char protocol_header[8];
    char * last_stream_data;
    void * last_header_data;
    char last_out_data[256];
    TSS_Timestamp_Mode timestamp_mode;
    //Thread stuff
    volatile char is_active;
    //Windows Stuff
//    HANDLE overlapped_reader_event;
//    HANDLE overlapped_writer_event;
    HANDLE reader_event;
    HANDLE writer_event;
    CRITICAL_SECTION reader_lock;
    CRITICAL_SECTION stream_lock;
    HANDLE reader_thread;
    HANDLE serial_port;
} TSS_Dongle;


#pragma pack(push,1)
typedef struct TSS_Header_69{
    char success_failure;
    unsigned char command_echo;
    unsigned char data_length;
} TSS_Header_69;
#pragma pack(pop)

#pragma pack(push,1)
typedef struct TSS_Header_71{
    char success_failure;
    unsigned int timestamp;
    unsigned char command_echo;
    unsigned char data_length;
} TSS_Header_71;
#pragma pack(pop)


#pragma pack(push,1)
typedef struct TSS_Header_85{
    char success_failure;
    unsigned char command_echo;
    unsigned char logical_id;
    unsigned char data_length;
} TSS_Header_85;
#pragma pack(pop)

#pragma pack(push,1)
typedef struct TSS_Header_87{
    char success_failure;
    unsigned int timestamp;
    unsigned char command_echo;
    unsigned char logical_id;
    unsigned char data_length;
} TSS_Header_87;
#pragma pack(pop)

LARGE_INTEGER timer_frequency;
unsigned int l_timestamp;
TSS_Sensor * sensor_list[64];
unsigned int sensor_list_len;
unsigned int sensor_list_counter;

TSS_Dongle * dongle_list[64];
unsigned int dongle_list_len;
unsigned int dongle_list_counter;

unsigned int wireless_retries;
int default_baud_rate;

#ifdef _HEXDUMP
FILE * f_hex_dump;
#endif


void endian_swap_16(unsigned short * x);

void endian_swap_32(unsigned int * x);

int compareFirmwareVersion(const char * version_string1,const char * version_string2);

TSS_Firmware_Compatibility getFirmwareCompatibility(const char * version_string);

#ifndef TSS_STATIC_LIB
int tss_initThreeSpaceAPI();

int tss_delThreeSpaceAPI();
#endif // TSS_STATIC_LIB

TSS_Device_Id _createTSSWirelessSensor(TSS_Device_Id device, char logical_id);  //move to core

unsigned char createChecksum(const char *command_bytes,const unsigned int num_bytes);

int parseData( char* data, int len, const char* parser_str);

int calculateParseDataSize(const char* parser_str);

TSS_Error _generateProtocolHeader(TSS_Protocol_Header_Setup protocol,char * parser_str8);  //move to core

TSS_Error f9Write(HANDLE com_handle,const TSS_Command * cmd_info, const char * input_data);

TSS_Error f7WriteRead(HANDLE com_handle, const TSS_Command * cmd_info,
                const char * input_data, char * output_data); //move to core

TSS_Error f8WriteRead(HANDLE com_handle, char logical_id, const TSS_Command * cmd_info, const char * input_data, char * output_data); //move to core

TSS_Error f9WriteRead(TSS_Sensor * sensor, const TSS_Command * cmd_info,
                const char * input_data, char * output_data, unsigned int * timestamp); //move to core

TSS_Error f9WriteReadDongle(TSS_Dongle * dongle, const TSS_Command * cmd_info,
                const char * input_data, char * output_data,
                unsigned int * timestamp); //move to core

TSS_Error faWriteReadDongle(TSS_Dongle * dongle, char logical_id, const TSS_Command * cmd_info,
                const char * input_data, char * output_data,
                unsigned int * timestamp); //move to core

TSS_Error writeRead(TSS_Device_Id device, const TSS_Command * cmd_info,
              const char * input_data, char * output_data,
              unsigned int * timestamp); //move to core

TSS_Error _parseWiredStreamData(TSS_Device_Id device, char * output_data,
                          unsigned int output_data_len,  unsigned int * timestamp); //move to core

unsigned int __stdcall _serialReadLoop(void * vsensor); //move to core

TSS_Error _parseWiredStreamDataThreaded(TSS_Sensor * sensor); //move to core

unsigned int __stdcall _serialReadLoopDongle(void * vdongle); //move to core

TSS_Error _parseWiredStreamDataThreadedDongle(TSS_Dongle * dongle); //move to core

#endif // YEI_THREESPACE_CORE_H_INCLUDED
