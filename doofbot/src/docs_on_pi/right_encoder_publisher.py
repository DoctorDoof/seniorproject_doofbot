#!/usr/bin/env python

###########################################################
#       DoofBot Right Encoder Code (Use with encoderLib)  #
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
class publisher_r:

    def __init__(self):
        self.pub = rospy.Publisher("/right_wheel/state", Float64, queue_size = 1)



### Methods Called to publish current encoder value
def pub_r_val(encodeVal):
    #covert to feet
    encodeVal = encodeVal*(math.pi/2000)
    #initiate the node and create the publisher
    pubber_r.pub.publish(encodeVal)




if __name__ == '__main__':

    #Set up the encoders
    encoderR = enc.LS7366R(1, 1000000, 4)

    encoderR.clearCounter()

    #Set up the publishers
    rospy.init_node("right_wheel")
    pubber_r = publisher_r()

    #Loop the encoder count and publish the values
    while(True):
        encoderRight = encoderR.readCounter()
        pub_r_val(encoderRight)
        time.sleep(0.05)