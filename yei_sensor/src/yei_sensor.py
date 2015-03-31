#! /usr/bin/env python

import rospy
from imu_common.msg import yei_msg
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

def IMUtalker( shin ):
    pub = rospy.Publisher('IMUchatter', yei_msg, queue_size=1)
    rospy.init_node('IMU_pub', anonymous=True)
    rate = rospy.Rate(200) #200 hz

    while not rospy.is_shutdown():
        msg = yei_msg()
        if IMUshin is not None:
            # getAllCorrectedComponentSensorData format
            #   Gyro Rate in counts per degrees/sec (Vector x3),
            #   Acceleration Vector in counts per g (Vector x3),
            #   Compass Vector in counts per gauss (Vector x3)
            tempreadings = threespace_api.TSUSBSensor.getAllCorrectedComponentSensorData( shin )
            msg.shin_gyr.x = tempreadings[0]
            msg.shin_gyr.y = tempreadings[1]
            msg.shin_gyr.z = tempreadings[2]
            msg.shin_acc.x = tempreadings[3]
            msg.shin_acc.y = tempreadings[4]
            msg.shin_acc.z = tempreadings[5]
            msg.shin_mag.x = tempreadings[6]
            msg.shin_mag.y = tempreadings[7]
            msg.shin_mag.z = tempreadings[8]

        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    print('Opening Shin IMU')
    try:
        IMUshin = threespace_api.TSUSBSensor(com_port=portSHIN, baudrate=115200, timestamp_mode=TSS_TIMESTAMP_NONE)
        print(IMUshin)
    except:
        print("Could not Open Shin IMU on {0} - closing".format(portSHIN))
        exit()

    try:
        IMUtalker( IMUshin )
    except rospy.ROSInterruptException:
        pass

