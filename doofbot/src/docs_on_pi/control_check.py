#!/usr/bin/env python

###########################################################
#       DoofBot Controller Mod                            #
#           Senior Design Project -                       #
#                       Autonomous Navigation             #
#                   by: Robert Belle-Isle                 #
#                              & Tyler Werner             #
#                   Under the Direction of:               #
#                           Dr. Carlos Luck               #
###########################################################
#NOTE: Please refer to USER MANUAL for help understanding this code
# and troubleshooting any issues with the robot

#Import files needed to fun the robot
import rospy
import time
import serial
import math
#import the msg file that is used to send key commands
from std_msgs.msg import String, Int16, Float64, Bool
try:
    import encoderLibrary as enc
except:
    print('Encoder Library not Imported')

class publisher_gui_alert:

    def __init__(self):
        self.pub = rospy.Publisher("/gui_alert", Bool, queue_size = 1)


class publisher_right_controlswitch:

    def __init__(self):
        self.pub = rospy.Publisher("/right_wheel/pid_enable", Bool, queue_size = 1)


### Class used to hold values for checking controller
class doofbot:

    def __init__(self):
        self.controller_running = False
        self.cur_left = 0
        self.cur_right = 0
        self.prev_left = -1
        self.prev_right = -1
        self.left_wheel_dis = 0
        self.right_wheel_dis = 0
        self.check_left_bool = False
        self.check_right_bool = False

    def reset_vals(self):
        self.cur_left = 0
        self.cur_right = 0
        self.prev_left = -1
        self.prev_right = -1
        self.controller_running = False
        self.left_wheel_dis = 0
        self.right_wheel_dis = 0
        self.check_left_bool = False
        self.check_right_bool = False

    def set_to_running(self):
        self.controller_running = True

#Used to change variables within the doofbot class (Controller checking)
def check_controller():
    db.cur_left = rospy.wait_for_message("/left_wheel/state", Float64)
    db.cur_right = rospy.wait_for_message("/right_wheel/state", Float64)
    if db.cur_left == db.prev_left:
        db.check_left_bool = True
    if db.cur_right == db.prev_right:
        db.check_right_bool = True
    if db.check_left_bool == True:
        if db.right_wheel_dis < 0:
            if db.cur_right.data < db.right_wheel_dis:
                db.check_right_bool = True
        else:
            if db.cur_right.data > db.right_wheel_dis:
                db.check_right_bool = True
    if db.check_right_bool == True:
        if db.left_wheel_dis < 0:
            if db.cur_left.data < db.left_wheel_dis:
                db.check_left_bool = True
        else:
            if db.cur_left.data > db.left_wheel_dis:
                db.check_left_bool = True
    if db.check_left_bool and db.check_right_bool:
        db.controller_running = False
        rospy.loginfo("L: {} R: {}".format(db.cur_left,db.cur_right))
    if db.cur_left != 0 and db.cur_right != 0:
        db.prev_left = db.cur_left
        db.prev_right = db.cur_right

def wait_to_start():
    #rospy.wait_for_message("/left_wheel/control_effort", Bool)
    pass

def end_move():
    right_pubber_ctl.pub.publish(False)
    time.sleep(1)
    gui_alert.pub.publish(True)

def move_one():
    db.left_wheel_dis = 1
    db.right_wheel_dis = 1
    #check to see if controller is finished
    db.set_to_running()
    wait_to_start()
    while db.controller_running == True:
        check_controller()
    db.reset_vals()
    end_move()

def move_two():
    db.left_wheel_dis = 2
    db.right_wheel_dis = 2
    #check to see if controller is finished
    db.set_to_running()
    wait_to_start()
    while db.controller_running == True:
        check_controller()
    db.reset_vals()
    end_move()

def move_three():
    db.left_wheel_dis = 3
    db.right_wheel_dis = 3
    #check to see if controller is finished
    db.set_to_running()
    wait_to_start()
    while db.controller_running == True:
        check_controller()
    db.reset_vals()
    end_move()


def move_four():
    db.left_wheel_dis = 4
    db.right_wheel_dis = 4
    #check to see if controller is finished
    db.set_to_running()
    wait_to_start()
    while db.controller_running == True:
        check_controller()
    db.reset_vals()
    end_move()

def turn_left():
    db.left_wheel_dis = -1.68
    db.right_wheel_dis = 1.68
    #check to see if controller is finished
    db.set_to_running()
    wait_to_start()
    while db.controller_running == True:
        check_controller()
    db.reset_vals()
    end_move()

def turn_right():
    db.left_wheel_dis = 1.68
    db.right_wheel_dis = -1.68
    #check to see if controller is finished
    db.set_to_running()
    wait_to_start()
    while db.controller_running == True:
        check_controller()
    db.reset_vals()
    end_move()

def callback(msg):
    key_command = msg.data
    if key_command == "move1":
        move_one()
    elif key_command == "move2":
        move_two()
    elif key_command == "move3":
        move_three()
    elif key_command == "move4":
        move_four()
    elif key_command == "left":
        turn_left()
    elif key_command == "right":
        turn_right()
    else:
        pass


def main():
    rospy.init_node("control_mod")
    rospy.Subscriber("key_command", String, callback)
    rospy.spin()


if __name__ == '__main__':
    db = doofbot()
    right_pubber_ctl = publisher_right_controlswitch()
    gui_alert = publisher_gui_alert()
    main()