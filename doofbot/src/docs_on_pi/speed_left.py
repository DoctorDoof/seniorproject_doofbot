#!/usr/bin/env python

###########################################################
#       DoofBot Left Encoder Code (Use with encoderLib)   #
#           Senior Design Project -                       #
#                       Autonomous Navigation             #
#                   by: Robert Belle-Isle                 #
#                              & Tyler Werner             #
#                   Under the Direction of:               #
#                           Dr. Carlos Luck               #
###########################################################
#NOTE: Please refer to USER MANUAL for help understanding this code
# and troubleshooting any issues with the robot

import rospy
import time
import math
from std_msgs.msg import Float64
import encoderLibrary as enc


### The Classs used to set up the publishers
class publisher_l:

    def __init__(self):
        self.pub = rospy.Publisher("/left_wheel/vel", Float64, queue_size = 1)

class publisher_abs_l:

    def __init__(self):
        self.pub = rospy.Publisher("/left_wheel/abs_vel", Float64, queue_size = 1)


### Methods Called to publish current encoder value
def pub_l_val(encodeVal):
    #covert to feet
    encodeVal = encodeVal*(math.pi/2000)
    #initiate the node and create the publisher
    if abs(encodeVal) < 3:
        pubber_l.pub.publish(encodeVal)
        pubber_abs_l.pub.publish(abs(encodeVal))




if __name__ == '__main__':

    #Set up the encoders
    encoderL = enc.LS7366R(0, 1000000, 4)
    #Set up the publishers
    rospy.init_node("left_wheel_vel")
    pubber_l = publisher_l()
    pubber_abs_l = publisher_abs_l()

    #Loop the encoder count and publish the values
    while(True):
        prev_val = encoderL.readCounter()
        time.sleep(0.05)
        encoderLeft = encoderL.readCounter()
        encoderLeft = (encoderLeft-prev_val)/0.05
        pub_l_val(encoderLeft)
