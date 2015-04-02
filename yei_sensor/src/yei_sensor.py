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
def IMUtalker( shin, thigh ):
    pub = rospy.Publisher('IMUchatter', yei_msg, queue_size=0)
    rospy.init_node('IMU_pub')
    rate = rospy.Rate(500) #500 hz - Make the loop go fast

    while not rospy.is_shutdown():
        message = yei_msg()
        if shin is not None:
            readings = checkIMUStream( shin )
            # readings = pingIMUOnce( shin )
            message.shin_gyr.x = readings[0]
            message.shin_gyr.y = readings[1]
            message.shin_gyr.z = readings[2]
            message.shin_acc.x = readings[3]
            message.shin_acc.y = readings[4]
            message.shin_acc.z = readings[5]
            message.shin_mag.x = readings[6]
            message.shin_mag.y = readings[7]
            message.shin_mag.z = readings[8]
        if thigh is not None:
            readings = checkIMUStream( thigh )
            # readings = pingIMUOnce( thigh )
            message.thigh_gyr.x = readings[0]
            message.thigh_gyr.y = readings[1]
            message.thigh_gyr.z = readings[2]
            message.thigh_acc.x = readings[3]
            message.thigh_acc.y = readings[4]
            message.thigh_acc.z = readings[5]
            message.thigh_mag.x = readings[6]
            message.thigh_mag.y = readings[7]
            message.thigh_mag.z = readings[8]

        message.header.stamp = rospy.Time.now()
        pub.publish(message)
        rate.sleep()


###############################################################################################
if __name__ == '__main__':
    ## Initial setup
    print('Opening Shin IMU')
    try:
        # Set up device
        IMUshin = threespace_api.TSUSBSensor( com_port=portSHIN, timestamp_mode=TSS_TIMESTAMP_NONE )
        print('   ' + str(IMUshin))
        # Set the axes properly for the Kalman Filter
        print('   Setting Axis Directions to YXZ - 002 with Z and X axis flipped about Y')
        threespace_api.TSUSBSensor.setAxisDirections( IMUshin, 0x2A )
        # Tare the IMU at current orientation
        threespace_api.TSUSBSensor.setFilterMode( IMUshin, 1 )
        time.sleep(5)
        threespace_api.TSUSBSensor.tareWithCurrentOrientation( IMUshin )
        print('   Tared at {0}'.format(threespace_api.TSUSBSensor.getTareAsQuaternion( IMUshin )))
        # Put in IMU mode (speed considerations)
        print('   Setting to raw IMU mode')
        threespace_api.TSUSBSensor.setFilterMode( IMUshin, 0 )
    except:
        print("   Could not Open Shin IMU on {0} or error in setting configuration - closing".format(portSHIN))
        exit()

    #print('Opening Thigh IMU')
    #try:
    #    # Set up device
    #    IMUthigh = threespace_api.TSUSBSensor( com_port=portTHIGH, timestamp_mode=TSS_TIMESTAMP_NONE )
    #    print('   ' + str(IMUthigh))
    #    ## Put in IMU mode (speed considerations)
    #    print('   Setting to raw IMU mode')
    #    threespace_api.TSUSBSensor.setFilterMode( IMUthigh, 0 )
    #except:
    #    print("   Could not Open Thigh IMU on {0} or error in setting configuration - closing".format(portTHIGH))
    #    exit()

    ## Streaming Setup
    try:
        print("Starting Stream")
        ## Set up streaming
        threespace_api.TSUSBSensor.setStreamingTiming( IMUshin, interval=0, duration=0xffffffff, delay=0 )
        #threespace_api.TSUSBSensor.setStreamingTiming( IMUthigh, interval=0, duration=0xffffffff, delay=0 )
        threespace_api.TSUSBSensor.setStreamingSlots( IMUshin, slot0='getCorrectedGyroRate', slot1='getCorrectedAccelerometerVector', slot2='getCorrectedCompassVector')
        #threespace_api.TSUSBSensor.setStreamingSlots( IMUthigh, slot0='getCorrectedGyroRate', slot1='getCorrectedAccelerometerVector', slot2='getCorrectedCompassVector')
        threespace_api.TSUSBSensor.startStreaming( IMUshin )
        #threespace_api.TSUSBSensor.startStreaming( IMUthigh )
        print("   Stream Enabled on all sensors")
    except:
        print("   Could not start stream")
        exit()

    ## Main Loop
    try:
        IMUtalker( IMUshin, None )
    except rospy.ROSInterruptException:
        pass

    ## Close
    IMUshin.close()
    #IMUthigh.close()



