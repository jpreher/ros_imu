#! /usr/bin/env python

import rospy
from imu_common.msg import yei_msg
from std_msgs.msg import String
import threespace_api
from threespace_api import TSS_TIMESTAMP_NONE
import serial
import time
import math
import string
import colorsys
from socket import *

portSHIN = "/dev/ttyACM0"
portTHIGH = "/dev/ttyACM1"
portPFOOT = "/dev/ttyACM2"

TSS_START_BYTE = 0xf7
TSS_GET_TARED_ORIENTATION_AS_QUATERNION = 0x00
TSS_GET_CORRECTED_LINEAR_ACCELERATION_IN_GLOBAL_SPACE = 0x29
TSS_GET_BUTTON_STATE = 0xfa
TSS_NULL = 0xff # No command use to fill the empty slots in "set stream slots"
TSS_SET_STREAMING_SLOTS = 0x50
TSS_SET_STREAMING_TIMING = 0x52
TSS_START_STREAMING = 0x55
TSS_STOP_STREAMING = 0x56

# Streaming mode require the streaming slots and streaming timing being setup prior to start streaming
def setupStreaming(serial_port):
    print("   Setting up TSS_SET_STREAMING_SLOTS")
    # There are 8 streaming slots available for use, and each one can hold one of the streamable commands.
    # Unused slots should be filled with 0xff so that they will output nothing.
    write_bytes = bytearray()
    write_bytes.append(TSS_START_BYTE)
    write_bytes.append(TSS_SET_STREAMING_SLOTS)
    write_bytes.append(TSS_GET_TARED_ORIENTATION_AS_QUATERNION) # stream slot0
    write_bytes.append(TSS_GET_CORRECTED_LINEAR_ACCELERATION_IN_GLOBAL_SPACE) # stream slot1
    write_bytes.append(TSS_GET_BUTTON_STATE) # stream slot2
    write_bytes.append(TSS_NULL) # stream slot3
    write_bytes.append(TSS_NULL) # stream slot4
    write_bytes.append(TSS_NULL) # stream slot5
    write_bytes.append(TSS_NULL) # stream slot6
    write_bytes.append(TSS_NULL) # stream slot7

    # calculating the checksum (note the checksum doesn't include the start byte)
    write_bytes.append((sum(write_bytes) - write_bytes[0]) % 256)

    # Write the bytes to the serial
    serial_port.write(write_bytes)

    interval = 2000 # microseconds
    # Duration detemines how long the streaming session will run for
    # A duration of 0xffffffff will have the streaming session run till the stop stream command is called
    duration = 0xffffffff # microseconds
    delay = 0 # microseconds

    write_bytes = bytearray()
    write_bytes.append(TSS_START_BYTE)
    write_bytes.append(TSS_SET_STREAMING_TIMING)

    # The data must be flipped to big endian before sending to sensor
    write_bytes.extend(bytearray(struct.pack('>I', interval)))
    write_bytes.extend(bytearray(struct.pack('>I', duration)))
    write_bytes.extend(bytearray(struct.pack('>I', delay)))

    # calculating the checksum (note the checksum doesn't include the start byte)
    write_bytes.append((sum(write_bytes) - write_bytes[0]) % 256)

    # Write the bytes to the serial
    serial_port.write(write_bytes)

    # Now Stream
    write_bytes_start = bytearray((TSS_START_BYTE,
                                TSS_START_STREAMING,
                                TSS_START_STREAMING))
    # Write the bytes to the serial
    serial_port.write(write_bytes_start)

## IMU Publisher
def IMUtalker( shin, thigh ):
    pub = rospy.Publisher('IMUchatter', yei_msg, queue_size=0)
    rospy.init_node('IMU_pub')
    rate = rospy.Rate(50000) #2000 hz - Make the loop go top speed

    while not rospy.is_shutdown():
        msg = yei_msg()
        if shin is not None:
            # getAllCorrectedComponentSensorData format
            #   Gyro Rate in counts per degrees/sec (Vector x3),
            #   Acceleration Vector in counts per g (Vector x3),
            #   Compass Vector in counts per gauss (Vector x3)
            tempreadings = threespace_api.TSUSBSensor.getAllRawComponentSensorData( shin )
            msg.shin_gyr.x = tempreadings[0]
            msg.shin_gyr.y = tempreadings[1]
            msg.shin_gyr.z = tempreadings[2]
            msg.shin_acc.x = tempreadings[3]
            msg.shin_acc.y = tempreadings[4]
            msg.shin_acc.z = tempreadings[5]
            msg.shin_mag.x = tempreadings[6]
            msg.shin_mag.y = tempreadings[7]
            msg.shin_mag.z = tempreadings[8]
        if thigh is not None:
            tempreadings = threespace_api.TSUSBSensor.getAllRawComponentSensorData( thigh )
            msg.thigh_gyr.x = tempreadings[0]
            msg.thigh_gyr.y = tempreadings[1]
            msg.thigh_gyr.z = tempreadings[2]
            msg.thigh_acc.x = tempreadings[3]
            msg.thigh_acc.y = tempreadings[4]
            msg.thigh_acc.z = tempreadings[5]
            msg.thigh_mag.x = tempreadings[6]
            msg.thigh_mag.y = tempreadings[7]
            msg.thigh_mag.z = tempreadings[8]

        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    print('Opening Shin IMU')
    try:
        # Set up device
        IMUshin = threespace_api.TSUSBSensor(com_port=portSHIN, timestamp_mode=TSS_TIMESTAMP_NONE)
        print('   ' + str(IMUshin))
        ## Put in IMU mode (speed considerations)
        print('   Setting to raw IMU mode')
        threespace_api.TSUSBSensor.setFilterMode(IMUshin, 0)
        ## Set up streaming
        serial_port_shin = serial.Serial(portSHIN, timeout=0.5, writeTimeout=0.5, baudrate=115200)
        #setupStreaming(serial_port_shin)
    except:
        print(" Could not Open Shin IMU on {0} or error in setting configuration - closing".format(portSHIN))
        exit()

    print('Opening Thigh IMU')
    try:
        # Set up device
        IMUthigh = threespace_api.TSUSBSensor(com_port=portTHIGH, timestamp_mode=TSS_TIMESTAMP_NONE)
        print('   ' + str(IMUthigh))
        ## Put in IMU mode (speed considerations)
        print('   Setting to raw IMU mode')
        threespace_api.TSUSBSensor.setFilterMode(IMUthigh, 0)
        ## Set up streaming
        serial_port_shin = serial.Serial(portTHIGH, timeout=0.5, writeTimeout=0.5, baudrate=115200)
        #setupStreaming(serial_port_shin)
    except:
        print(" Could not Open Thigh IMU on {0} or error in setting configuration - closing".format(portTHIGH))
        exit()

    try:
        IMUtalker( IMUshin, IMUthigh )
    except rospy.ROSInterruptException:
        pass
