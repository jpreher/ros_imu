#! /usr/bin/env python2.7

import threespace_api
from threespace_api import TSS_TIMESTAMP_NONE
import time
import math
import string
import colorsys
from socket import *
import serial
import sys
import traceback

portSHIN = "/dev/ttyACM0"
portTHIGH = "/dev/ttyACM1"
portPFOOT = "/dev/ttyACM2"

###############################################################################################
## Get a single value from the IMU
def pingIMUOnce( sensorspace ):
    # getAllCorrectedComponentSensorData format
    #   Gyro Rate in counts per degrees/sec (Vector x3),
    #   Acceleration Vector in counts per g (Vector x3),
    #   Compass Vector in counts per gauss (Vector x3)
    tempreadings = threespace_api.TSUSBSensor.getAllCorrectedComponentSensorData( sensorspace )
    return tempreadings


###############################################################################################
def checkIMUStream( sensorspace ):
    # Get the latest sensor reading from the stream
    try:
        tempreadings = sensorspace.stream_last_data
    except:
        print("Could not get data")
        return [0,0,0,0,0,0,0,0]

    # Return the data without the checksum
    return tempreadings[1]

###############################################################################################
## IMU Publisher
def IMU_callback():
    global shinIMU
    global thighIMU
    global footIMU
    
    message = [0.0]*27
    time.sleep(0.0005)

    if shinIMU is not None:
        readings = checkIMUStream( shinIMU )
        # readings = pingIMUOnce( shin )
        message[0] = readings[0]
        message[1] = readings[1]
        message[2] = readings[2]
        message[3] = readings[3]
        message[4] = readings[4]
        message[5] = readings[5]
        message[6] = readings[6]
        message[7] = readings[7]
        message[8] = readings[8]
    if thighIMU is not None:
        readings = checkIMUStream( thighIMU )
        # readings = pingIMUOnce( thigh )
        message[9] = readings[0]
        message[10] = readings[1]
        message[11] = readings[2]
        message[12] = readings[3]
        message[13] = readings[4]
        message[14] = readings[5]
        message[15] = readings[6]
        message[16] = readings[7]
        message[17] = readings[8]
    if footIMU is not None:
        readings = checkIMUStream( footIMU )
        # readings = pingIMUOnce( foot )
        message[18] = readings[0]
        message[19] = readings[1]
        message[20] = readings[2]
        message[21] = readings[3]
        message[22] = readings[4]
        message[23] = readings[5]
        message[24] = readings[6]
        message[25] = readings[7]
        message[26] = readings[8]

    return message

###############################################################################################
def IMU_init():
    global shinIMU
    global thighIMU
    global footIMU

    ## Initial setup
    print('Opening Shin IMU')
    try:
        # Set up device
        shinIMU = threespace_api.TSUSBSensor( com_port=portSHIN, timestamp_mode=TSS_TIMESTAMP_NONE )
        print('   ' + str(shinIMU))
        # Set the axes properly for the Kalman Filter
        print("   Setting Axis Directions to YXZ - 002 with Z and X axis flipped about Y")
        threespace_api.TSUSBSensor.setAxisDirections( shinIMU, 0x2A )
        print('   Starting gyro autocalibration')
        threespace_api.TSUSBSensor.beginGyroscopeAutoCalibration( shinIMU )
        # Put in IMU mode (speed considerations)
        print('   Setting to raw IMU mode')
        threespace_api.TSUSBSensor.setFilterMode( shinIMU, 0 )
    except IOError as e:
        print "I/O error({0}): {1}".format(e.errno, e.strerror)
    except ValueError:
        print "Could not convert data.", traceback.format_exc()
    except:
        print "Unexpected error:", sys.exc_info()[0], traceback.format_exc()
        print("   Could not Open Shin IMU on {0} or error in setting configuration - closing".format(portSHIN))
        return 1

    print('Opening Thigh IMU')
    try:
        # Set up device
        thighIMU = threespace_api.TSUSBSensor( com_port=portTHIGH, timestamp_mode=TSS_TIMESTAMP_NONE )
        print('   ' + str(thighIMU))
        # Set the axes properly for the Kalman Filter
        print('   Setting Axis Directions to YXZ - 002 with Z and X axis flipped about Y')
        threespace_api.TSUSBSensor.setAxisDirections( thighIMU, 0x2A )
        # Set the gyroscope auto-scale
        print('   Starting gyro autocalibration')
        threespace_api.TSUSBSensor.beginGyroscopeAutoCalibration( thighIMU )
        # Put in IMU mode (speed considerations)
        print('   Setting to raw IMU mode')
        threespace_api.TSUSBSensor.setFilterMode( thighIMU, 0 )
    except IOError as e:
        print "I/O error({0}): {1}".format(e.errno, e.strerror)
    except ValueError:
        print "Could not convert data."
    except:
        print "Unexpected error:", sys.exc_info()[0], traceback.format_exc()
        print("   Could not Open Thigh IMU on {0} or error in setting configuration - closing".format(portSHIN))
        return 1

    #print('Opening Foot IMU')
    #try:
    #    # Set up device
    #    IMUfoot = threespace_api.TSUSBSensor( com_port=portFOOT, timestamp_mode=TSS_TIMESTAMP_NONE )
    #    print('   ' + str(IMUfoot))
    #    # Put in IMU mode (speed considerations)
    #    print('   Setting to raw IMU mode')
    #    threespace_api.TSUSBSensor.setFilterMode( IMUfoot, 0 )
    #except:
    #    print("   Could not Open Thigh IMU on {0} or error in setting configuration - closing".format(portFOOT))
    #    return 1

    # Streaming Setup
    try:
        print("Starting Stream")
        ## Set up streaming
        threespace_api.TSUSBSensor.setStreamingTiming( shinIMU, interval=0, duration=0xffffffff, delay=0 )
        threespace_api.TSUSBSensor.setStreamingTiming( thighIMU, interval=0, duration=0xffffffff, delay=0 )
    #    #threespace_api.TSUSBSensor.setStreamingTiming( IMUfoot, interval=0, duration=0xffffffff, delay=0 )
        threespace_api.TSUSBSensor.setStreamingSlots( shinIMU, slot0='getCorrectedAccelerometerVector', slot1='getCorrectedGyroRate', slot2='getCorrectedCompassVector')
        threespace_api.TSUSBSensor.setStreamingSlots( thighIMU, slot0='getCorrectedAccelerometerVector', slot1='getCorrectedGyroRate', slot2='getCorrectedCompassVector')
    #    #threespace_api.TSUSBSensor.setStreamingSlots( IMUfoot, slot0='getCorrectedAccelerometerVector', slot1='getCorrectedGyroRate', slot2='getCorrectedCompassVector')
        threespace_api.TSUSBSensor.startStreaming( shinIMU )
        threespace_api.TSUSBSensor.startStreaming( thighIMU )
    #    #threespace_api.TSUSBSensor.startStreaming( IMUfoot )
        print("   Stream Enabled on all sensors")
        time.sleep(1)
    except:
        print("   Could not start stream")
        exit()

    return 0


###############################################################################################
if __name__ == '__main__':
    global shinIMU
    global thighIMU
    global footIMU

    IMU_init()

    while

    ## Close
    shinIMU.close()
    thighIMU.close()

