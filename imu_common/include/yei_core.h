#ifndef YEI_CORE_H
#define YEI_CORE_H
#include <stdio.h>
#include <string.h>

/********************************************//**
 * Macro for infinite duration for streaming setup.
 ***********************************************/
#define TSS_INFINITE_DURATION 0xffffffff

/********************************************//**
 * The default number of soft retries on wireless
 * If tss_resetThreeSpaceAPI is called retries will be set back to this value
 ***********************************************/
#define TSS_DEFAULT_WIRLESSS_RETRIES 5

/********************************************//**
 * The default baud rate that sensors will be opened with
 * If tss_resetThreeSpaceAPI is called baud rate will be set back to this value
 ***********************************************/
#define TSS_DEFAULT_BAUD_RATE 115200

/********************************************//**
 * YEI 3-Space API device identifier, a common parameter needed for all 3-Space API calls.
 ***********************************************/
typedef unsigned int TSS_Device_Id;

/********************************************//**
 * An enum expressing the masks used to quickly determine the type of a sensor based on their device ID.
 ***********************************************/
typedef enum TSS_Device_Id_Mask
{
    TSS_NO_DEVICE_ID    = 0x00800000,              /**< Invalid Device ID */
    TSS_BOOTLOADER_ID   = 0x01000000,              /**< Bootloader */
    TSS_DONGLE_ID       = 0x02000000,              /**< Wireless Dongle */
    TSS_USB_ID          = 0x04000000,              /**< USB, Micro USB, or Watertight USB */
    TSS_EMBEDDED_ID     = 0x08000000,              /**< Embedded */
    TSS_WIRELESS_ID     = 0x10000000,              /**< Wireless 2.4GHz DSSS plugged via USB */
    TSS_WIRELESS_W_ID   = 0x20000000,              /**< Wireless 2.4GHz DSSS communicating wirelessly */
    TSS_DATALOGGER_ID   = 0x40000000,              /**< Data-logging */
    TSS_BLUETOOTH_ID    = 0x80000000,              /**< Bluetooth */
    TSS_NO_DONGLE_ID    = 0xfd000000,              /**< Convenience ID, includes all but the Wireless Dongle */
    TSS_ALL_SENSORS_ID  = 0xff000000               /**< Convenience ID, includes all valid id masks */
}TSS_Device_Id_Mask;

/********************************************//**
 * An enum expressing API Timestamp Modes.
 ***********************************************/
typedef enum TSS_Timestamp_Mode{
    TSS_TIMESTAMP_NONE,   /**< Disables timestamp, timestamp will return 0 */
    TSS_TIMESTAMP_SENSOR, /**< 3-Space device's timestamp, this can be set with tss_updateCurrentTimestamp */
    TSS_TIMESTAMP_SYSTEM  /**< The data is timestamped on arrival to the system using the high-resolution performance counter */
}TSS_Timestamp_Mode;

/********************************************//**
 * An enum expressing the different types of errors a 3-Space API call can return.
 ***********************************************/
typedef enum TSS_Error
{
    TSS_NO_ERROR,                   /**< The API call successfully executed */
    TSS_ERROR_COMMAND_FAIL,         /**< The command returned a failed response */
    TSS_INVALID_COMMAND,            /**< The API call was made on a device type that does not support the attempted command */
    TSS_INVALID_ID,                 /**< The TSS_Device_Id parameter passed in to an API call is not associated with a connected 3-Space device */
    TSS_ERROR_PARAMETER,            /**< General parameter fail */
    TSS_ERROR_TIMEOUT,              /**< The command's timeout has been reached */
    TSS_ERROR_WRITE,                /**< The API call executed failed to write all the data necessary to execute the command to the intended 3-Space device */
    TSS_ERROR_READ,                 /**< The API call executed failed to read all the data necessary to execute the command to the intended 3-Space device */
    TSS_ERROR_STREAM_SLOTS_FULL,    /**< The 3-Space device's stream slots are full */
    TSS_ERROR_STREAM_CONFIG,        /**< The 3-Space device's stream configuration is corrupted */
    TSS_ERROR_MEMORY,               /**< A memory error occurred in the API */
    TSS_ERROR_FIRMWARE_INCOMPATIBLE /**< 3-Space device firmware does not support that command, firmware update highly recommended */
}TSS_Error;

static const char* const TSS_Error_String[] = {
    "TSS_NO_ERROR",
    "TSS_ERROR_COMMAND_FAIL",
    "TSS_INVALID_COMMAND",
    "TSS_INVALID_ID",
    "TSS_ERROR_PARAMETER",
    "TSS_ERROR_TIMEOUT",
    "TSS_ERROR_WRITE",
    "TSS_ERROR_READ",
    "TSS_ERROR_STREAM_SLOTS_FULL",
    "TSS_ERROR_STREAM_CONFIG",
    "TSS_ERROR_MEMORY",
    "TSS_ERROR_FIRMWARE_INCOMPATIBLE"
};

/********************************************//**
 * An enum denoting the compatibility level of the 3-Space device.
 ***********************************************/
typedef enum TSS_Firmware_Compatibility
{
    TSS_FW_NOT_COMPATIBLE,       /**< Firmware is not compatible with the API and should be updated */
    TSS_FW_20R7_COMPATIBLE,      /**< 3-Space device compatible with 2.0R7 commands and up */
    TSS_FW_20R10_COMPATIBLE,      /**< 3-Space device compatible with 2.0R10 commands and up */
    TSS_FW_20R13_COMPATIBLE      /**< 3-Space device compatible with 2.0R13 commands and up */
}TSS_Firmware_Compatibility;

static const char* const TSS_Firmware_Version_String[] = {
    "00Jan2000A00",
    "25Apr2013A00",
    "21Jun2013A00",
    "28Aug2013A00"
};

/********************************************//**
 * An enum expressing the command list of Streamable Commands.
 *
 * TSS_Stream_Command_Enum comtains the command bytes of streamable commands,
 * the commands match up with the functions with similar names
 *
 * Example: TSS_GET_TARED_ORIENTATION_AS_QUATERNION is tss_getTaredOrientationAsQuaternion
 ***********************************************/
typedef enum TSS_Stream_Command_Enum
{
    TSS_GET_TARED_ORIENTATION_AS_QUATERNION                 = 0x00, /**< \see tss_getTaredOrientationAsQuaternion */
    TSS_GET_TARED_ORIENTATION_AS_EULER_ANGLES               = 0x01, /**< \see tss_getTaredOrientationAsEulerAngles */
    TSS_GET_TARED_ORIENTATION_AS_ROTATION_MATRIX            = 0x02, /**< \see tss_getTaredOrientationAsRotationMatrix */
    TSS_GET_TARED_ORIENTATION_AS_AXIS_ANGLE                 = 0x03, /**< \see tss_getTaredOrientationAsAxisAngle */
    TSS_GET_TARED_ORIENTATION_AS_TWO_VECTOR                 = 0x04, /**< \see tss_getTaredOrientationAsTwoVector */
    TSS_GET_DIFFERENCE_QUATERNION                           = 0x05, /**< \see tss_getDifferenceQuaternion */
    TSS_GET_UNTARED_ORIENTATION_AS_QUATERNION               = 0x06, /**< \see tss_getUntaredOrientationAsQuaternion */
    TSS_GET_UNTARED_ORIENTATION_AS_EULER_ANGLES             = 0x07, /**< \see tss_getUntaredOrientationAsEulerAngles */
    TSS_GET_UNTARED_ORIENTATION_AS_ROTATION_MATRIX          = 0x08, /**< \see tss_getUntaredOrientationAsRotationMatrix */
    TSS_GET_UNTARED_ORIENTATION_AS_AXIS_ANGLE               = 0x09, /**< \see tss_getUntaredOrientationAsAxisAngle */
    TSS_GET_UNTARED_ORIENTATION_AS_TWO_VECTOR               = 0x0a, /**< \see tss_getUntaredOrientationAsTwoVector */
    TSS_GET_TARED_TWO_VECTOR_IN_SENSOR_FRAME                = 0x0b, /**< \see tss_getTaredTwoVectorInSensorFrame */
    TSS_GET_UNTARED_TWO_VECTOR_IN_SENSOR_FRAME              = 0x0c, /**< \see tss_getUntaredTwoVectorInSensorFrame */
    TSS_GET_ALL_NORMALIZED_COMPONENT_SENSOR_DATA            = 0x20, /**< \see tss_getAllNormalizedComponentSensorData */
    TSS_GET_NORMALIZED_GYRO_RATE                            = 0x21, /**< \see tss_getNormalizedGyroRate */
    TSS_GET_NORMALIZED_ACCELEROMETER_VECTOR                 = 0x22, /**< \see tss_getNormalizedAccelerometerVector */
    TSS_GET_NORMALIZED_COMPASS_VECTOR                       = 0x23, /**< \see tss_getNormalizedCompassVector */
    TSS_GET_ALL_CORRECTED_COMPONENT_SENSOR_DATA             = 0x25, /**< \see tss_getAllCorrectedComponentSensorData */
    TSS_GET_CORRECTED_GYRO_RATE                             = 0x26, /**< \see tss_getCorrectedGyroRate */
    TSS_GET_CORRECTED_ACCELEROMETER_VECTOR                  = 0x27, /**< \see tss_getCorrectedAccelerometerVector */
    TSS_GET_CORRECTED_COMPASS_VECTOR                        = 0x28, /**< \see tss_getCorrectedCompassVector */
    TSS_GET_CORRECTED_LINEAR_ACCELERATION_IN_GLOBAL_SPACE   = 0x29, /**< \see tss_getCorrectedLinearAccelerationInGlobalSpace */
    TSS_GET_TEMPERATURE_C                                   = 0x2b, /**< \see tss_getTemperatureC */
    TSS_GET_TEMPERATURE_F                                   = 0x2c, /**< \see tss_getTemperatureF */
    TSS_GET_CONFIDENCE_FACTOR                               = 0x2d, /**< \see tss_getConfidenceFactor */
    TSS_GET_ALL_RAW_COMPONENT_SENSOR_DATA                   = 0x40, /**< \see tss_getAllRawComponentSensorData */
    TSS_GET_RAW_GYROSCOPE_RATE                              = 0x41, /**< \see tss_getRawGyroscopeRate */
    TSS_GET_RAW_ACCELEROMETER_DATA                          = 0x42, /**< \see tss_getRawAccelerometerData */
    TSS_GET_RAW_COMPASS_DATA                                = 0x43, /**< \see tss_getRawCompassData */
    TSS_GET_BATTERY_VOLTAGE                                 = 0xc9, /**< \see tss_getBatteryVoltage */
    TSS_GET_BATTERY_PERCENT_REMAINING                       = 0xca, /**< \see tss_getBatteryPercentRemaining */
    TSS_GET_BATTERY_STATUS                                  = 0xcb, /**< \see tss_getBatteryStatus */
    TSS_GET_BUTTON_STATE                                    = 0xfa, /**< \see tss_getButtonState */
    TSS_NULL                                                = 0xff  /**< TSS_NULL, all unused stream slots must be filled with TSS_NULL */
}TSS_Stream_Command_Enum;

/********************************************//**
 * YEI 3-Space API streamable command.
 ***********************************************/
typedef unsigned char TSS_Stream_Command;

/********************************************//**
 * An enum expressing the find flags for the tss_getComPorts filter parameter.
 ***********************************************/
typedef enum TSS_Find
{
    TSS_FIND_BTL        = 0x00000001, /**< Find Bootloader */
    TSS_FIND_USB        = 0x00000002, /**< Find USB, Micro USB, or Watertight USB */
    TSS_FIND_DNG        = 0x00000004, /**< Find Wireless Dongle */
    TSS_FIND_WL         = 0x00000008, /**< Find Wireless 2.4GHz DSSS plugged via USB */
    TSS_FIND_EM         = 0x00000010, /**< Find Embedded */
    TSS_FIND_DL         = 0x00000020, /**< Find Data-logging */
    TSS_FIND_BT         = 0x00000040, /**< Find Bluetooth */
    TSS_FIND_UNKNOWN    = 0x80000000, /**< Find serial ports that may have 3-Space devices connected to them */
    TSS_FIND_ALL_KNOWN  = 0x7fffffff, /**< Find all known 3-Space devices */
    TSS_FIND_ALL        = 0xffffffff  /**< Find all com ports including "unknown" serial ports that may have 3-Space devices connected */
}TSS_Find;

/********************************************//**
 * An enum expressing the types of 3-Space devices.
 ***********************************************/
typedef enum TSS_Type
{
    TSS_UNKNOWN,    /**< Device type was not able to be determined */
    TSS_BTL,        /**< Bootloader */
    TSS_USB,        /**< USB, Micro USB, or Watertight USB */
    TSS_DNG,        /**< Wireless Dongle */
    TSS_WL,         /**< Wireless 2.4GHz DSSS plugged via USB */
    TSS_WL_W,       /**< Wireless 2.4GHz DSSS communicating wirelessly */
    TSS_EM,         /**< Embedded */
    TSS_DL,         /**< Data-logging */
    TSS_BT,         /**< Bluetooth */
}TSS_Type;

static const char* const TSS_Type_String[] = {
    "TSS_UNKNOWN",
    "TSS_BTL",
    "TSS_USB",
    "TSS_DNG",
    "TSS_WL",        //wireless wired (connected to PC)
    "TSS_WL_W",    //wireless wireless
    "TSS_EM",
    "TSS_DL",
    "TSS_BT",
};

/********************************************//**
 * An enum expressing the alternate directions for each of the natural axes of the sensor.
 ***********************************************/
typedef enum TSS_Axis_Direction
{
    TSS_XYZ, /**< X: Right, Y: Up, Z: Forward (left-handed system, standard operation) */
    TSS_XZY, /**< X: Right, Y: Forward, Z: Up (right-handed system) */
    TSS_YXZ, /**< X: Up, Y: Right, Z: Forward (right-handed system) */
    TSS_YZX, /**< X: Forward, Y: Right, Z: Up (left-handed system) */
    TSS_ZXY, /**< X: Up, Y: Forward, Z: Right (left-handed system) */
    TSS_ZYX, /**< X: Forward, Y: Up, Z: Right (right-handed system) */
}TSS_Axis_Direction;

/********************************************//**
 * An enum expressing the different available filter modes.
 ***********************************************/
typedef enum TSS_Filter_Mode
{
    TSS_FILTER_IMU,                        /**< IMU Mode */
    TSS_FILTER_KALMAN,                     /**< Kalman Filtered Mode */
    TSS_FILTER_ALTERNATING_KALMAN,         /**< Alternating Kalman Filtered Mode */
    TSS_FILTER_COMPLEMENTARY,              /**< Complementary Filter Mode */
    TSS_FILTER_QUATERNION_GRADIENT_DECENT, /**< Quaternion Gradient Descent Filter Mode */
}TSS_Filter_Mode;

/********************************************//**
 * A structure that contains basic information about a com port.
 ***********************************************/
typedef struct TSS_ComPort
{
    char com_port[32];      /**< The com port string. */
    char friendly_name[64]; /**< The frienly name of the com port. */
    int sensor_type;        /**< The type of 3-Space device connected through the com port. */ //Change?
}TSS_ComPort;

/********************************************//**
 * A structure that contains information about the connected 3-Space device.
 ***********************************************/
typedef struct TSS_ComInfo
{
    TSS_Type device_type; /**< The type of 3-Space device connected through the com port. */
    unsigned int serial_number; /**< The serial number for the 3-Space device connected through the com port. */
    char firmware_version[13]; /**< The version of the firmware installed on the connected 3-Space device. */
    char hardware_version[33]; /**< The hardware revision and type of the connected 3-Space device. */
    TSS_Firmware_Compatibility fw_compatibility; /**< Firmware compatibility level (Note level may be lower than current if no functional changes were made). */
}TSS_ComInfo;

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

void endian_swap_16(unsigned short * x);

void endian_swap_32(unsigned int * x);

#endif // YEI_CORE_H
