#!/usr/bin/env python

###########################################################
#       DoofBot Control Effort Right Wheel                #
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
from std_msgs.msg import Float64, Int16, String, Bool
try:
    import encoderLibrary as enc
except:
    print('Encoder Library not Imported')

class publisher_int:

    def __init__(self):
        self.pub = rospy.Publisher("serial_cmd", Int16, queue_size = 1)

class publisher_string:

    def __init__(self):
        self.pub = rospy.Publisher("key_command", String, queue_size = 1)

class publisher_right_bool:

    def __init__(self):
        self.pub = rospy.Publisher("/right_wheel/pid_enable", Bool, queue_size = 1)

class doofbot:

    def __init__(self):
        self.right_setpoint = 0
        self.cur_vel = 0
        self.cur_right = 0
        self.right_commanded_speed = 0

def pub_effort(current_effort):
    pubber_effort.pub.publish(current_effort)

def pub_command(command):
    pubber_command.pub.publish(command)

def pub_right_bool(val):
    pubber_right_bool.pub.publish(val)

def right_set(msg):
    db.right_setpoint = msg.data

def right_speed(msg):
    db.cur_vel = msg.data
    if db.right_setpoint > 0:
        if db.cur_right > db.right_setpoint and db.cur_vel != 0:
            pub_right_bool(False)
            pub_effort(64)
    elif db.right_setpoint < 0:
        if db.cur_right < db.right_setpoint and db.cur_vel != 0:
            pub_right_bool(False)
            pub_effort(64)
    else:
        pass

def right_check(msg):
    db.cur_right = msg.data
    if db.right_setpoint > 0:
        if db.cur_right > db.right_setpoint:
            pub_right_bool(False)
            pub_effort(64)
    elif db.right_setpoint < 0:
        if db.cur_right < db.right_setpoint:
            pub_right_bool(False)
            pub_effort(64)
    else:
        pass

def right_call(msg):
    if db.right_setpoint > 0:
        if db.cur_right > db.right_setpoint:
            pub_right_bool(False)
            pub_effort(64)
        else:
            db.right_commanded_speed = 69 + int(round(msg.data*10))
            pub_effort(db.right_commanded_speed)
    else:
        if db.cur_right < db.right_setpoint:
            pub_right_bool(False)
            pub_effort(64)
        else:
            db.right_commanded_speed = 59 + int(round(msg.data*11))
            pub_effort(db.right_commanded_speed)


def main():
    rospy.Subscriber("/right_wheel/control_effort", Float64, right_call)
    rospy.Subscriber("/right_wheel/setpoint", Float64, right_set)
    rospy.Subscriber("/right_wheel/state", Float64, right_check)
    rospy.Subscriber("/right_wheel/vel", Float64,right_speed)
    rospy.spin()

if __name__ == '__main__':

    db = doofbot()
    rospy.init_node("Control_Effort")
    pubber_command = publisher_string()
    pubber_effort = publisher_int()
    pubber_right_bool = publisher_right_bool()
    main()
