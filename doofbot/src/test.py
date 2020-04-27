#!/usr/bin/env python

import rospy
import time
from std_msgs.msg import Float64

class doofbot:

    def __init__(self):
        self.left_controller_running = False
        self.right_controller_running = False
        self.cur_left = 0
        self.cur_right = 0
        self.prev_left = -1
        self.prev_right = -1

    def reset_vals(self):
        self.cur_left = 0
        self.cur_right = 0
        self.prev_left = -1
        self.prev_right = -1

    def set_to_running(self):
        self.left_controller_running = True
        self.right_controller_running = True

#Used to change variables within the doofbot class (Controller checking)
def check_left_controller():
    db.cur_left = rospy.wait_for_message("/left_wheel/state", Float64)
    print("Left Wheel pos: {} vs {}".format(db.cur_left,db.prev_left))
    if db.cur_left == db.prev_left:
        db.left_controller_running = False
    else:
        db.prev_left = db.cur_left
    

def check_right_controller():
    print("Right Wheel pos: {} vs {}".format(db.cur_right,db.prev_right))
    db.cur_right = rospy.wait_for_message("/right_wheel/state", Float64)
    if db.cur_right == db.prev_right:
        db.right_controller_running = False
    else:
        db.prev_right = db.cur_right

if __name__ == '__main__':
    rospy.init_node("test_check")
    db = doofbot()
    db.set_to_running()
    while True:
        check_left_controller()
        check_right_controller()
        time.sleep(0.1)
        if db.left_controller_running == False and db.right_controller_running == False:
            break
    db.reset_vals()