#!/usr/bin/env python

###########################################################
#       DoofBot Control Effort Left Wheel                 #
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


def pub_effort(current_effort):
    pubber_effort.pub.publish(current_effort)

def pub_command(command):
    pubber_command.pub.publish(command)

class doofbot:

    def __init__(self):
        self.left_setpoint = 0
        self.cur_vel = 0
        self.vel_ctr = 0
        self.cur_left = 0
        self.left_commanded_speed = 0
        self.right_setpoint = 0

def vel_control(msg):
    db.vel_ctr = msg.data

def left_set(msg):
    db.left_setpoint = msg.data

def right_set(msg):
    db.right_setpoint = msg.data

def left_speed(msg):
    db.cur_vel = msg.data
    if db.left_setpoint > 0:
        if db.cur_left > db.left_setpoint and db.cur_vel != 0:
            pub_effort(192)
    elif db.left_setpoint < 0:
        if db.cur_left < db.left_setpoint and db.cur_vel != 0:
            pub_effort(192)
    else:
        pass

def left_check(msg):
    db.cur_left = msg.data
    if db.left_setpoint > 0:
        if db.cur_left > db.left_setpoint:
            pub_effort(192)
    elif db.left_setpoint < 0:
        if db.cur_left < db.left_setpoint:
            pub_effort(192)
    else:
        pass

def left_call(msg):
    if db.right_setpoint > 0:
        if db.left_setpoint > 0:
            if db.cur_left > db.left_setpoint:
                pub_effort(192)
            else:
                db.left_commanded_speed = 197 + int(round(msg.data*10)) + int(round(msg.data*db.vel_ctr*11))
                pub_effort(db.left_commanded_speed)
        else:
            if db.cur_left < db.left_setpoint:
                pub_effort(192)
            else:
                db.left_commanded_speed = 183 - int(round(msg.data*12)) - int(round(msg.data*db.vel_ctr*15))
                pub_effort(db.left_commanded_speed)
    elif db.right_setpoint < 0:
            if db.left_setpoint > 0:
                if db.cur_left > db.left_setpoint:
                    pub_effort(192)
                else:
                    db.left_commanded_speed = 198 - int(round(msg.data*11)) - int(round(msg.data*db.vel_ctr*12))
                    pub_effort(db.left_commanded_speed)
            else:
                pass
    else:
        pass

def main():
    rospy.Subscriber("/right_wheel/control_effort", Float64, left_call)
    rospy.Subscriber("/left_wheel/setpoint", Float64, left_set)
    rospy.Subscriber("/right_wheel/setpoint", Float64, right_set)
    rospy.Subscriber("/left_wheel/state", Float64, left_check)
    rospy.Subscriber("/left_wheel/vel", Float64,left_speed)
    rospy.Subscriber("/left_velocity/control_effort", Float64,vel_control)
    rospy.spin()

if __name__ == '__main__':

    db = doofbot()
    rospy.init_node("Control_Effort")
    pubber_command = publisher_string()
    pubber_effort = publisher_int()
    main()
