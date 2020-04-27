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
        self.pub = rospy.Publisher("/left_wheel/state", Float64, queue_size = 1)



### Methods Called to publish current encoder value
def pub_l_val(encodeVal):
    #covert to feet
    encodeVal = encodeVal*(math.pi/2000)
    #initiate the node and create the publisher
    pubber_l.pub.publish(encodeVal)




if __name__ == '__main__':

    #Set up the encoders
    encoderL = enc.LS7366R(0, 1000000, 4)

    encoderL.clearCounter()

    #Set up the publishers
    rospy.init_node("left_wheel")
    pubber_l = publisher_l()

    #Loop the encoder count and publish the values
    while(True):
        encoderLeft = encoderL.readCounter()
        pub_l_val(encoderLeft)
        time.sleep(0.05)
