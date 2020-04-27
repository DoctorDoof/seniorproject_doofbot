#!/usr/bin/env python

###########################################################
#       DoofBot Subscribing Pi Control System             #
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
try:
    import RPi.GPIO as gpio
except:
    print('GPIO not Imported')
try:
    gpio.setmode(gpio.BCM)
    port = "/dev/ttyS0"
    ser = serial.Serial()
    ser.port = port
    ser.baudrate = 9600
    ser.bytesize = serial.EIGHTBITS
    ser.parity = serial.PARITY_NONE
    ser.stopbits = serial.STOPBITS_ONE
    ser.timeout = 1
    ser.open()
except:
    print('Serial Communication with TX channel on Ras PI not Created')
try:
    port2 = "/dev/ttyUSB0"
    ser2 = serial.Serial()
    ser2.port = port2
    ser2.baudrate = 9600
    ser2.bytesize = serial.EIGHTBITS
    ser2.parity = serial.PARITY_NONE
    ser2.stopbits = serial.STOPBITS_ONE
    ser2.timeout = 1
    ser2.open()

except:
    print('Serial Communication with robot arm not Created')


class publisher_left:

    def __init__(self):
        self.pub = rospy.Publisher("/left_wheel/setpoint", Float64, queue_size = 1)

class publisher_right:

    def __init__(self):
        self.pub = rospy.Publisher("/right_wheel/setpoint", Float64, queue_size = 1)

class publisher_right_controlswitch:

    def __init__(self):
        self.pub = rospy.Publisher("/right_wheel/pid_enable", Bool, queue_size = 1)

class DoofDriver:

    def __init__(self):
        #Create inital speeds and values for the driver
        self.movetype = 'fluid'
        self.stop_right = 64
        self.stop_left = 192
        self.for_right = 76
        self.for_left = 204
        self.back_right = 52
        self.back_left = 177
        self.encoderLeft = enc.LS7366R(0, 1000000, 4)
        self.encoderRight = enc.LS7366R(1, 1000000, 4)
        self.err_l = 0
        self.err_r = 0
        self.err_lt = 0
        self.err_rt = 0

    def clear_encoders(self):
        self.encoderLeft.clearCounter()
        self.encoderRight.clearCounter()

    def stop_moving(self):
        ser.write(chr(0).encode())
        left_pubber.pub.publish(0)
        right_pubber.pub.publish(0)

    def move_fwd(self):
        if db.movetype == 'fluid':
            try:
                ser.write(bytearray([db.for_left]))
                ser.write(bytearray([db.for_right]))
            except:
                pass
        else:
            #controlled movement command here
            pass
    def move_bkw(self):
        if db.movetype == 'fluid':
            try:
                ser.write(bytearray([db.back_left]))
                ser.write(bytearray([db.back_right]))
            except:
                pass
        else:
            #controlled movement command here
            pass
    def turn_left(self):
        if db.movetype == 'fluid':
            try:
                ser.write(bytearray([db.back_left]))
                ser.write(bytearray([db.for_right]))
            except:
                pass
        else:
            #controlled movement command here
            pass
    def turn_right(self):
        if db.movetype == 'fluid':
            try:
                ser.write(bytearray([db.for_left]))
                ser.write(bytearray([db.back_right]))
            except:
                pass
        else:
            #controlled movement command here
            pass

    def controlled_move(self, command):
        if command == 'left':
            db.clear_encoders()
            right_pubber_ctl.pub.publish(True)
            left_pubber.pub.publish(-1.673 + db.err_lt)
            right_pubber.pub.publish(1.673 - db.err_rt)
            

        elif command == 'right':
            db.clear_encoders()
            right_pubber_ctl.pub.publish(True)
            left_pubber.pub.publish(1.673 - db.err_lt)
            right_pubber.pub.publish(-1.673 + db.err_rt)
            
        
        elif command == 'stop':
            ser.write(chr(0).encode())
            db.clear_encoders()
            left_pubber.pub.publish(0)
            right_pubber.pub.publish(0)

        else:
            db.clear_encoders()
            right_pubber_ctl.pub.publish(True)
            left_pubber.pub.publish(int(command) - db.err_l)
            right_pubber.pub.publish(int(command) - db.err_r)

def call_error_l(msg):
    db.err_l = msg.data

def call_error_r(msg):
    db.err_r = msg.data

def call_error_lt(msg):
    db.err_lt = msg.data

def call_error_rt(msg):
    db.err_rt = msg.data


class MicrobotArm:

    def __init__(self):
        pass

    def step_arm(self, msg):
        rospy.loginfo("{} recieved".format(msg))
        try:
            for i in msg:
                ser2.write(bytearray([ord(i)]))
        except:
            rospy.loginfo("Did not move arm")

def callback(msg):
    key_command = msg.data
    rospy.loginfo(key_command)
    #Determine action depending on subscribed string
    if key_command[0] == '@':
        micro.step_arm(key_command)
    elif key_command == 'fluid':
        db.movetype = 'fluid'
    elif key_command == 'stag':
        db.movetype = 'stag'
    elif key_command == 'stop':
        db.stop_moving()
    elif key_command == 'clear_encoder':
        db.clear_encoders()
    elif key_command == 'e':
        #Could be used for more depending
        db.stop_moving()
    elif key_command == 'w':
        db.move_fwd()
    elif key_command == 'a':
        db.turn_left()
    elif key_command == 's':
        db.move_bkw()
    elif key_command == 'd':
        db.turn_right()
    elif key_command == 'left':
        db.controlled_move('left')
    elif key_command == 'right':
        db.controlled_move('right')
    elif key_command == 'move1':
        db.controlled_move('1')
    elif key_command == 'move2':
        db.controlled_move('2')
    elif key_command == 'move3':
        db.controlled_move('3')
    elif key_command == 'move4':
        db.controlled_move('4')
    elif key_command == 'finished':
        db.controlled_move('stop')
    else:
        pass


def effort_callback(msg):
    ser.write(bytearray([msg.data]))


def main():
    rospy.Subscriber("key_command", String, callback)
    rospy.Subscriber("serial_cmd",Int16, effort_callback)
    rospy.Subscriber("/error/left_forward", Float64, call_error_l)
    rospy.Subscriber("/error/right_forward", Float64, call_error_r)
    rospy.Subscriber("/error/left_turn", Float64, call_error_lt)
    rospy.Subscriber("/error/right_turn", Float64, call_error_rt)
    rospy.spin()

if __name__ == '__main__':

    rospy.init_node("Doof_Control")
    left_pubber = publisher_left()
    left_pubber.pub.publish(0)
    right_pubber = publisher_right()
    right_pubber.pub.publish(0)
    right_pubber_ctl = publisher_right_controlswitch()
    right_pubber_ctl.pub.publish(False)
    db = DoofDriver()
    micro = MicrobotArm()
    main()

