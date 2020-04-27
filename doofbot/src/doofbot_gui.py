#!/usr/bin/env python

###########################################################
#       DoofBot Navigation Control System GUI             #
#           Senior Design Project -                       #
#                       Autonomous Navigation             #
#                   by: Robert Belle-Isle                 #
#                              & Tyler Werner             #
#                   Under the Direction of:               #
#                           Dr. Carlos Luck               #
###########################################################
#NOTE: Please refer to USER MANUAL for help understanding this code
# and troubleshooting any issues with the robot

#Import Files
import time
import os
import subprocess
import signal
import math
import numpy as np
#import the msg file that is used to send key commands
from std_msgs.msg import String, Float64, Bool
#Pc or Pi tester
try:
    from Tkinter import *
    import Tkinter as tk
    import tkMessageBox
    import tkFileDialog
    import rospy
except:
    rospy.loginfo("Please run this program through ROS")
class doofbot:

    def __init__(self):
        self.error_check = ''
        self.l_error = 0
        self.r_error = 0
        self.lt_error = 0
        self.rt_error = 0
        self.cur_left = 0
        self.cur_right = 0
        self.err_val_l = 0
        self.err_val_r = 0
        self.gridimage = []

    def reset(self):
        self.error_check = ''
        self.l_error = 0
        self.r_error = 0
        self.lt_error = 0
        self.rt_error = 0
        self.cur_left = 0
        self.cur_right = 0
        self.err_val_l = 0
        self.err_val_r = 0

### The Class used to set up the publisher named pub
class publisher:

    def __init__(self):
        self.pub = rospy.Publisher("key_command", String, queue_size = 1)

class publisher_error_l:

    def __init__(self):
        self.pub = rospy.Publisher("/error/left_forward", Float64, queue_size = 1)

class publisher_error_r:

    def __init__(self):
        self.pub = rospy.Publisher("/error/right_forward", Float64, queue_size = 1)

class publisher_error_lt:

    def __init__(self):
        self.pub = rospy.Publisher("/error/left_turn", Float64, queue_size = 1)

class publisher_error_rt:

    def __init__(self):
        self.pub = rospy.Publisher("/error/right_turn", Float64, queue_size = 1)

### Method Called to publish current key
def pub_key(currentkey, depress = False):
    #initiate the node and create the publisher
    if currentkey[0] != '@':
        currentkey = currentkey.lower()
    #Check the key and publish the msg
    if currentkey == 'e':
        rospy.loginfo("Closing Program")
        pubber.pub.publish(currentkey)
        proc.send_signal(signal.SIGINT)
        os.system('xset r on')
        dbnm.destroy()
    else:
        pubber.pub.publish(currentkey)
#Method used to do nothing
def PLACEHOLDER_FUNCTION(event):
    #Line added so it can be closed
    pass
#Used on certain depresses to cancel movement
def reset_Duty(event):
    global check_serial
    pub_key('stop',True)
    check_serial=1
#Keeps count to avoid error during run time
def key_counter(event):
    global press_ticks
    press_ticks+=1
#Checks the key input and sends to publish
def key_input(event):
    key_press = event.keysym
    global press_ticks
    global depress_ticks
    if key_press == 'e':
        encoder_clear()
        pub_key(key_press)
    elif key_press == '1':
        #Switch the 2/3 button Between Wrist Pitch, Grip and Roll 
        Switch_RollPitch()
    elif key_press == '2':
        #Roll/Pitch Wrist/grip CW/DOWN/OPEN
        RollPitch_CW(0)
    elif key_press == '3':
        #Roll/Pitch Wrist/grip CCW/UP/CLOSE
        RollPitch_CCW(0)
    elif key_press == '4':
        #Swivel robot arm left
        Swiv_Left(0)
    elif key_press == '5':
        #move the arm backwards
        Arm_Back(0)
    elif key_press == '6':
        #Swivel robot arm right
        Swiv_Right(0)
    elif key_press == '7':
        #Move arm DOWN
        Arm_Down(0)  
    elif key_press == '8':
        #move the Arm Forward
        Arm_For(0)
    elif key_press == '9':
        #move arm UP
        Arm_Up(0) 
    elif key_press in accepted_keys:
        if press_ticks == depress_ticks:
            wasd_Disable()
            pub_key(key_press)
    press_ticks+=1
#Checks the key release and sends to publish
def key_stop(event):
    key_press = event.keysym
    global press_ticks
    global depress_ticks
    if key_press in accepted_keys:
        if depress_ticks == press_ticks -1:
            wasd_Enable()
            pub_key('stop', True)
            pub_key('@RESET')
    depress_ticks +=1
#Methods that enable and disables the keyboard control
def wasd_Disable():
    global wasd_enabled
    global dbmm
    global f8_btn
    global f2_btn
    global s4_btn
    global s6_btn
    global r1_btn
    global r3_btn
    global u7_btn
    global u9_btn
    f8_btn.bind('<ButtonPress-1>', PLACEHOLDER_FUNCTION)
    f8_btn.bind('<ButtonRelease-1>', PLACEHOLDER_FUNCTION)
    f2_btn.bind('<ButtonPress-1>', PLACEHOLDER_FUNCTION)
    f2_btn.bind('<ButtonRelease-1>', PLACEHOLDER_FUNCTION)
    s4_btn.bind('<ButtonPress-1>', PLACEHOLDER_FUNCTION)
    s4_btn.bind('<ButtonRelease-1>', PLACEHOLDER_FUNCTION)
    s6_btn.bind('<ButtonPress-1>', PLACEHOLDER_FUNCTION)
    s6_btn.bind('<ButtonRelease-1>', PLACEHOLDER_FUNCTION)
    r1_btn.bind('<ButtonPress-1>', PLACEHOLDER_FUNCTION)
    r1_btn.bind('<ButtonRelease-1>', PLACEHOLDER_FUNCTION)
    r3_btn.bind('<ButtonPress-1>', PLACEHOLDER_FUNCTION)
    r3_btn.bind('<ButtonRelease-1>', PLACEHOLDER_FUNCTION)
    u7_btn.bind('<ButtonPress-1>', PLACEHOLDER_FUNCTION)
    u7_btn.bind('<ButtonRelease-1>', PLACEHOLDER_FUNCTION)
    u9_btn.bind('<ButtonPress-1>', PLACEHOLDER_FUNCTION)
    u9_btn.bind('<ButtonRelease-1>', PLACEHOLDER_FUNCTION)
    u7_btn.configure(state = DISABLED)
    u9_btn.configure(state = DISABLED)
    f8_btn.configure(state = DISABLED)
    f2_btn.configure(state = DISABLED)
    s4_btn.configure(state = DISABLED)
    s6_btn.configure(state = DISABLED)
    r1_btn.configure(state = DISABLED)
    r3_btn.configure(state = DISABLED)
    Button(dbnm, text= ' W ',state = DISABLED, padx=20,pady=20,command= w_press).grid(row=1,column=1)
    Button(dbnm, text= ' A ',state = DISABLED,padx=20,pady=20,command= a_press).grid(row=2,column=0)
    Button(dbnm, text= ' S ',state = DISABLED,padx=20,pady=20,command= s_press).grid(row=2,column=1)
    Button(dbnm, text= ' D ',state = DISABLED,padx=20,pady=20,command= d_press).grid(row=2,column=2)
    dbnm.bind('<KeyPress>', key_counter)
    wasd_enabled = False
def wasd_Enable():
        global wasd_enabled
        global dbmm
        global f8_btn
        global f2_btn
        global s4_btn
        global s6_btn
        global r1_btn
        global r3_btn
        global u7_btn
        global u9_btn
        f8_btn.configure(state = NORMAL)
        f8_btn.bind('<ButtonPress-1>', Arm_For)
        f8_btn.bind('<ButtonRelease-1>', Stop_Arm)
        f2_btn.configure(state = NORMAL)
        f2_btn.bind('<ButtonPress-1>', Arm_Back)
        f2_btn.bind('<ButtonRelease-1>', Stop_Arm)
        s4_btn.configure(state = NORMAL)
        s4_btn.bind('<ButtonPress-1>', Swiv_Left)
        s4_btn.bind('<ButtonRelease-1>', Stop_Arm)
        s6_btn.configure(state = NORMAL)
        s6_btn.bind('<ButtonPress-1>', Swiv_Right)
        s6_btn.bind('<ButtonRelease-1>', Stop_Arm)
        r1_btn.configure(state = NORMAL)
        r1_btn.bind('<ButtonPress-1>', RollPitch_CCW)
        r1_btn.bind('<ButtonRelease-1>', Stop_Arm)
        r3_btn.configure(state = NORMAL)
        r3_btn.bind('<ButtonPress-1>', RollPitch_CW)
        r3_btn.bind('<ButtonRelease-1>', Stop_Arm)
        u7_btn.configure(state = NORMAL)
        u7_btn.bind('<ButtonPress-1>', Arm_Up)
        u7_btn.bind('<ButtonRelease-1>', Stop_Arm)
        u9_btn.configure(state = NORMAL)
        u9_btn.bind('<ButtonPress-1>', Arm_Down)
        u9_btn.bind('<ButtonRelease-1>', Stop_Arm)
        w_btn = Button(dbnm, text= ' W ',padx=20,pady=20)
        w_btn.grid(row=1,column=1)
        w_btn.bind('<ButtonPress-1>', w_press)
        w_btn.bind('<ButtonRelease-1>', reset_Duty)
        a_btn = Button(dbnm, text= ' A ',padx=20,pady=20)
        a_btn.grid(row=2,column=0)
        a_btn.bind('<ButtonPress-1>', a_press)
        a_btn.bind('<ButtonRelease-1>', reset_Duty)
        s_btn = Button(dbnm, text= ' S ',padx=20,pady=20)
        s_btn.grid(row=2,column=1)
        s_btn.bind('<ButtonPress-1>', s_press)
        s_btn.bind('<ButtonRelease-1>', reset_Duty)
        d_btn = Button(dbnm, text= ' D ',padx=20,pady=20)
        d_btn.grid(row=2,column=2)
        d_btn.bind('<ButtonPress-1>', d_press)
        d_btn.bind('<ButtonRelease-1>', reset_Duty)
        dbnm.bind('<KeyPress>', key_input)
        dbnm.bind('<KeyRelease>',key_stop)
        wasd_enabled = True
#Methods for robot arm commands
def Switch_RollPitch():
    global RollPitch
    if RollPitch == 'Pitch':
        RollPitch = 'Roll'
        Button(dbnm, text = ' Switch Wrist:  \n Roll  \n      (1)    ', pady = 25, command = Switch_RollPitch).grid(row=3,column=4)
    elif RollPitch == 'Roll':
        RollPitch = 'Grip'
        Button(dbnm, text = ' Switch Wrist:  \n Grip \n      (1)    ', pady = 25, command = Switch_RollPitch).grid(row=3,column=4)
    else:
        RollPitch = 'Pitch'
        Button(dbnm, text = ' Switch Wrist:  \n Pitch \n      (1)    ', pady = 25, command = Switch_RollPitch).grid(row=3,column=4)
def Arm_Stepper(steps):
    global step_cmd
    pub_key(steps)
    step_cmd = '' 
def Arm_For(event):
    global armMoving
    global step_cmd
    armMoving = True
    step_cmd = "@STE "
    step_cmd = step_cmd + str(int(speed_roboArm))
    step_cmd = step_cmd + ","
    #Initiate types of r's needed to move arm'
    r[2] = -5000 #Arm Elbow UP
    r[5] = -5000 #Open grip as moves
    for i, j in enumerate(r):
        step_cmd = step_cmd + str(int(round(j)))
        if i != len(r)-1:
            step_cmd = step_cmd + ","
    step_cmd = step_cmd + '\r' 
    Arm_Stepper(step_cmd)
def Arm_Back(event):
    global armMoving
    global step_cmd
    armMoving = True
    step_cmd = "@STE "
    step_cmd = step_cmd + str(int(speed_roboArm))
    step_cmd = step_cmd + ","
    #Initiate types of r's needed to move arm'
    r[2] = 5000 #Arm Elbow UP
    r[5] = 5000 #Close Grip as moves
    for i, j in enumerate(r):
        step_cmd = step_cmd + str(int(round(j)))
        if i != len(r)-1:
            step_cmd = step_cmd + ","
    step_cmd = step_cmd + '\r' 
    Arm_Stepper(step_cmd)
def Arm_Up(event):
    global armMoving
    global step_cmd
    armMoving = True
    step_cmd = "@STE "
    step_cmd = step_cmd + str(int(speed_roboArm))
    step_cmd = step_cmd + ","
    #Initiate types of r's needed to move arm'
    r[1] = -5000 #Arm shoulder UP
    r[2] = -5000 #Arm Elbow UP
    r[5] = -5000 #Open grip as moves
    for i, j in enumerate(r):
        step_cmd = step_cmd + str(int(round(j)))
        if i != len(r)-1:
            step_cmd = step_cmd + ","
    step_cmd = step_cmd + '\r' 
    Arm_Stepper(step_cmd)
def Arm_Down(event):
    global armMoving
    global step_cmd
    armMoving = True
    step_cmd = "@STE "
    step_cmd = step_cmd + str(int(speed_roboArm))
    step_cmd = step_cmd + ","
    #Initiate types of r's needed to move arm'
    r[1] = 5000 #Arm shoulder DOWN
    r[2] = 5000 #Arm Elbow DOWN
    r[5] = 5000 #Close Grip as moves
    for i, j in enumerate(r):
        step_cmd = step_cmd + str(int(round(j)))
        if i != len(r)-1:
            step_cmd = step_cmd + ","
    step_cmd = step_cmd + '\r' 
    Arm_Stepper(step_cmd)
def Swiv_Left(event):
    global armMoving
    global step_cmd
    armMoving = True
    step_cmd = "@STE "
    step_cmd = step_cmd + str(int(speed_roboArm))
    step_cmd = step_cmd + ","
    #Initiate types of r's needed to move arm'
    r[0] = 5000 #Swivel Base LEFT
    for i, j in enumerate(r):
        step_cmd = step_cmd + str(int(round(j)))
        if i != len(r)-1:
            step_cmd = step_cmd + ","
    step_cmd = step_cmd + '\r' 
    Arm_Stepper(step_cmd)
def Swiv_Right(event):
    global armMoving
    global step_cmd
    armMoving = True
    step_cmd = "@STE "
    step_cmd = step_cmd + str(int(speed_roboArm))
    step_cmd = step_cmd + ","
    #Initiate types of r's needed to move arm'
    r[0] = -5000 #Swivel Base RIGHT
    for i, j in enumerate(r):
        step_cmd = step_cmd + str(int(round(j)))
        if i != len(r)-1:
            step_cmd = step_cmd + ","
    step_cmd = step_cmd + '\r' 
    Arm_Stepper(step_cmd)
def RollPitch_CCW(event):
    global armMoving
    global step_cmd
    armMoving = True
    step_cmd = "@STE "
    step_cmd = step_cmd + str(int(speed_roboArm))
    step_cmd = step_cmd + ","
    #Initiate types of r's needed to move arm'
    if RollPitch == 'Roll':
        r[3] = 5000 #Wrist CW
        r[4] = -5000
    elif RollPitch == 'Grip':
        r[5] = 5000 #Grip CLOSE
    else:
        r[3] = -5000 #Wrist UP
        r[4] = -5000
    for i, j in enumerate(r):
        step_cmd = step_cmd + str(int(round(j)))
        if i != len(r)-1:
            step_cmd = step_cmd + ","
    step_cmd = step_cmd + '\r' 
    Arm_Stepper(step_cmd)
def RollPitch_CW(event):
    global armMoving
    global step_cmd
    armMoving = True
    step_cmd = "@STE "
    step_cmd = step_cmd + str(int(speed_roboArm))
    step_cmd = step_cmd + ","
    #Initiate types of r's needed to move arm'
    if RollPitch == 'Roll':
        r[3] = -5000 #Wrist CCW
        r[4] = 5000
    elif RollPitch == 'Grip':
        r[5] = -5000 #Grip OPEN
    else:
        r[3] = 5000 #Wrist DOWN
        r[4] = 5000
    for i, j in enumerate(r):
        step_cmd = step_cmd + str(int(round(j)))
        if i != len(r)-1:
            step_cmd = step_cmd + ","
    step_cmd = step_cmd + '\r' 
    Arm_Stepper(step_cmd)
def Stop_Arm(event):
        global step_cmd
        step_cmd = "@RESET"
        step_cmd = step_cmd + '\r' 
        Arm_Stepper(step_cmd)
#Methods for wasd gui button response
def w_press(event):
    pub_key('w')
def a_press(event):
    pub_key('a')
def s_press(event):
    pub_key('s')
def d_press(event):
    pub_key('d')
#Method used to clear the encoder counters
def encoder_clear():
    pub_key("clear_encoder")
#Methods called by the Auto Nav System that send movement messages
def Controlled_Movement(string):
    rospy.loginfo("{} was published".format(string))
    pub_key(string)
    ctr_done = False
    while ctr_done == False:
        ctr_done = rospy.wait_for_message("/gui_alert", Bool)
        if ctr_done:
            db.cur_left = rospy.wait_for_message("/left_wheel/state", Float64)
            db.cur_right = rospy.wait_for_message("/right_wheel/state", Float64)
            pub_key("finished")
            rospy.loginfo("Moved: [ {} , {} ]".format(db.cur_left,db.cur_right))
            if db.error_check == 'for':
                db.l_error = db.l_error + (db.cur_left.data - db.err_val_l)
                db.r_error = db.r_error + (db.cur_right.data - db.err_val_r)
            elif db.error_check == 'left':
                db.lt_error = db.lt_error - (db.cur_left.data - db.err_val_l)
                db.rt_error = db.rt_error + (db.cur_right.data - db.err_val_r)
            elif db.error_check == 'right':
                db.lt_error = db.lt_error + (db.cur_left.data - db.err_val_l)
                db.rt_error = db.rt_error - (db.cur_right.data - db.err_val_r)
            rospy.loginfo("Error: l: {},r: {},lt: {},rt: {}".format(db.l_error,db.r_error,db.lt_error,db.rt_error))
            pub_error_l.pub.publish(db.l_error)
            pub_error_r.pub.publish(db.r_error)
            pub_error_lt.pub.publish(db.lt_error)
            pub_error_rt.pub.publish(db.rt_error)
            #IMAGE PROCESS CHECK FOR STUFF
            #db.cur_left = rospy.wait_for_message("/left_wheel/state", Float64)
            #Checkifnewstuff()

            time.sleep(1) 
def auto_Foward(distance):
    db.error_check = 'for'
    while distance >= 4:
        db.err_val_l = 4
        db.err_val_r = 4
        Controlled_Movement('move4')
        distance -=4
    if distance == 1:
        db.err_val_l = 1
        db.err_val_r = 1
        Controlled_Movement('move1')
    elif distance == 2:
        db.err_val_l = 2
        db.err_val_r = 2
        Controlled_Movement('move2')
    elif distance == 3:
        db.err_val_l = 3
        db.err_val_r = 3
        Controlled_Movement('move3')
def auto_Left(num):
    db.error_check = 'left'
    if num == 1:
        db.err_val_l = -1.673
        db.err_val_r = 1.673
        Controlled_Movement('left') 
    elif num == 2:
        db.err_val_l = -1.673
        db.err_val_r = 1.673
        Controlled_Movement('left')
        Controlled_Movement('left')
    else:
        print('Something went wrong in the auto_left program')
def auto_Right(num):
    db.error_check = 'right'
    if num == 1:
        db.err_val_l = 1.673
        db.err_val_r = -1.673
        Controlled_Movement('right')
    elif num == 2:
        db.err_val_l = 1.673
        db.err_val_r = -1.673
        Controlled_Movement('right')
        Controlled_Movement('right')
    else:
        print('Something went wrong in the auto_right program')


#Defines the Auto Press Button Response
def auto_press():
    ### Close Current Window
    dbnm.destroy()
    #### CREATES A CLASS THAT ALLOWS A SCROLLABLE FRAME
    class Scroll_Frame:

        def __init__(self, master, **kwargs):
            width = kwargs.pop('width', None)
            height = kwargs.pop('height', None)
            bg = kwargs.pop('bg', kwargs.pop('background', None))
            self.outer = tk.Frame(master, **kwargs)

            self.vsb = tk.Scrollbar(self.outer, orient=tk.VERTICAL)
            self.vsb.pack(fill=tk.Y, side=tk.RIGHT)

            self.hsb = tk.Scrollbar(self.outer, orient=tk.HORIZONTAL)
            self.hsb.pack(fill=tk.X, side=tk.BOTTOM)

            self.canvas = tk.Canvas(self.outer, highlightthickness=0, width=width, height=height, bg=bg)
            self.canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
            self.canvas['yscrollcommand'] = self.vsb.set
            self.canvas['xscrollcommand'] = self.hsb.set
            # mouse scroll does not seem to work with just "bind"; You have
            # to use "bind_all". Therefore to use multiple windows you have
            # to bind_all in the current widget
            self.canvas.bind("<Enter>", self._bind_mouse)
            self.canvas.bind("<Leave>", self._unbind_mouse)
            self.vsb['command'] = self.canvas.yview
            self.hsb['command'] = self.canvas.xview

            self.inner = tk.Frame(self.canvas, bg=bg)
            # pack the inner Frame into the Canvas with the topleft corner 4 pixels offset
            self.canvas.create_window(4, 4, window=self.inner, anchor='nw')
            self.inner.bind("<Configure>", self._on_frame_configure)

            self.outer_attr = set(dir(tk.Widget))

        def __getattr__(self, item):
            if item in self.outer_attr:
                # geometry attributes etc (eg pack, destroy, tkraise) are passed on to self.outer
                return getattr(self.outer, item)
            else:
                # all other attributes (_w, children, etc) are passed to self.inner
                return getattr(self.inner, item)

        def _on_frame_configure(self, event=None):
            x1, y1, x2, y2 = self.canvas.bbox("all")
            height = self.canvas.winfo_height()
            self.canvas.config(scrollregion = (0,0, x2, max(y2, height)))

        def _bind_mouse(self, event=None):
            self.canvas.bind_all("<4>", self._on_mousewheel)
            self.canvas.bind_all("<5>", self._on_mousewheel)
            self.canvas.bind_all("<MouseWheel>", self._on_mousewheel)

        def _unbind_mouse(self, event=None):
            self.canvas.unbind_all("<4>")
            self.canvas.unbind_all("<5>")
            self.canvas.unbind_all("<MouseWheel>")

        def _on_mousewheel(self, event):
            """Linux uses event.num; Windows / Mac uses event.delta"""
            if event.num == 4 or event.delta > 0:
                self.canvas.yview_scroll(-1, "units" )
            elif event.num == 5 or event.delta < 0:
                self.canvas.yview_scroll(1, "units" )
    ##### END CLASS FOR SCROLLING ITEMS #############

    # Method for the Menu that asks user for Rows and Columns of Workspace ##
    def checkSize():
        #Create a window that checks the expected size of the workspace
        RowColCheck = Tk()
        RowColCheck.title('Workspace Sizing')
        RowColCheck.geometry("350x250+300+150")
        #Method for Loading map
        def Load_Map():
            global Map
            global loaded
            
            loaded, Map_ok = True, True
            try:
                filename = tkFileDialog.askopenfilename(initialdir = "/home/robbles/sp_catkin/src/doofbot/src/Maps",
                        title = "Select a file", filetypes=(("csv files","*.csv"),("All files", "*.*")))
            except:
                tkMessageBox.showerror("ERROR", "Please check the location of the .askopenfilename \n Line 400")
            try:
                Map = np.loadtxt(filename, delimiter = ',')
                #Ensures that the csv file doesnt have more than one 2 or 3 or is missing one
                if np.count_nonzero(Map == 2)>1 or np.count_nonzero(Map == 2)==0 or np.count_nonzero(Map == 2)>1 or np.count_nonzero(Map == 2)==0:
                    Map_ok = False
                if np.count_nonzero(Map > 3)>0 or np.count_nonzero(Map < 0)>0:
                    Map_ok = False
                #Loads csv into the Rows, Columns, starts and goals
                if Map_ok == True:
                    global Rows
                    global Columns
                    Rows = len(Map)
                    Columns = len(Map[0])
                    global goalX
                    global goalY
                    global startX
                    global startY
                    startData = str(np.argwhere(Map == 2))
                    goalData = str(np.argwhere(Map == 3))
                    if len(startData) == 7:
                        startY, startX = int(startData[2]), int(startData[4])
                    elif len(startData) == 9:
                        startY, startX = int(startData[2]+startData[3]), int(startData[5]+startData[6])
                    else:
                        startY, startX = int(startData[2]+startData[3]+startData[4]), int(startData[6]+startData[7]+startData[8])
                    if len(goalData) == 7:
                        goalY, goalX = int(goalData[2]), int(goalData[4])
                    elif len(goalData) == 9:
                        goalY, goalX = int(goalData[2]+goalData[3]), int(goalData[5]+goalData[6])
                    else:
                        goalY, goalX = int(goalData[2]+goalData[3]+goalData[4]), int(goalData[6]+goalData[7]+goalData[8])
                    RowColCheck.destroy()
                    #Ask user what the robots direction is
                    CheckDir = Tk()
                    CheckDir.title('What is the direction of the Robot?')

                    CheckDir.geometry("300x50+300+300")
                    def Change_Dir():
                        global direction
                        direction = var.get()
                        if direction == 'Positive Y':
                            direction = 'PosY'
                        elif direction == 'Negative Y':
                            direction = 'NegY'
                        elif direction == 'Positive X':
                            direction = 'PosX'
                        else:
                            direction = 'NegX'
                        print(direction)
                        CheckDir.destroy()
                    var = StringVar()
                    Button(CheckDir, text = 'Enter', command = Change_Dir).grid(row = 0,column = 2)
                    direction_Dropdown = OptionMenu(CheckDir, var, 
                        "Positive X","Negative X", "Positive Y", "Negative Y")
                    var.set("Positive Y")
                    direction_Dropdown.grid(row = 0, column = 1)
                    Label(CheckDir, text = 'Choose initial direction:').grid(row = 0, column = 0)
                else:
                    tkMessageBox.showerror("ERROR", "Please make sure CSV file has 1 goal and 1 start \n and only contains 0,1,2,3's")
            except:
                tkMessageBox.showerror("ERROR", "please only import a csv file with a grid of \n 0,1,2,3's")

        #Check if entry fields are numbers and add the Rows and Columns fields to mapper method
        def enter(row,col): 
            global loaded
            loaded = False    
            if row.isdigit() == True and col.isdigit() == True:
                if int(row) > 400 or int(col) > 400:
                    tkMessageBox.showerror("ERROR", "Numbers above 40 may take are not allowed in current build \n Your input: " + row + " " + col)
                elif int(row) == 0 or int(col) == 0:
                    tkMessageBox.showerror("ERROR", "0 in the entry field, please try again \n Your input: " + row + " " + col)
                elif int(row) == 1 and int(col) == 1:
                    tkMessageBox.showerror("ERROR", "1x1 grids have no paths, please try again \n Your input: " + row + " " + col)
                else:
                    global Rows
                    global Columns
                    global direction
                    Rows = int(row)
                    Columns = int(col)
                    RowColCheck.destroy()
            else:
                tkMessageBox.showerror("ERROR", "Please Only Enter Integer Numbers \n Your input: " + row + " " + col)
        #Create the menu and options for RowCol Check
        Label(RowColCheck, text = '   ').grid(row = 0, column = 0)
        Label(RowColCheck, text = '   ').grid(row = 1, column = 0)
        Label(RowColCheck, text = '  Enter number of Rows and Columns in Workspace').place(x=0,y=0)
        Row_entry = Entry(RowColCheck, width = 20)
        Row_entry.grid(row = 1, column=2, padx = 10, pady = 20)
        Column_entry = Entry(RowColCheck, width = 20)
        Column_entry.grid(row = 2, column=2, padx = 10, pady = 20)
        Label(RowColCheck, text = 'Rows: ').grid(row=1,column=1)
        Label(RowColCheck, text = 'Columns: ').grid(row=2, column=1)
        RowCol_Fin = Button(RowColCheck, text = 'Enter', command = lambda: enter(Row_entry.get(),Column_entry.get()), padx = 10, pady = 10)
        RowCol_Fin.grid(row = 3, column = 2)
        Load_Btn = Button(RowColCheck, text = 'Load Map', command = Load_Map, padx = 10, pady = 10)
        Load_Btn.grid(row = 4, column = 2)
        #Add Enter key pressing
        def enter_press(event):
            enter(Row_entry.get(),Column_entry.get())
        RowColCheck.bind('<Return>', enter_press)
        RowColCheck.mainloop()
    ############# END ROW COL MENU ###########################################

    # Create Method for Adding Obstacles, Goal and Start to Map[]
    def Mapper():
        #Generate map of zeros for grid to be placed
        global Map
        global direction
        Map = np.zeros([Rows,Columns])
        mapper = Tk()
        mapper.title('Obstacle Mapping')
        mapper.geometry('600x600+300+50')
        #Create initial Grid
        def init_Grid():
            global Map_Frame
            Map_Frame = Scroll_Frame(mapper, borderwidth = 2, relief = tk.SUNKEN, height = 450, width = 550)
            Map_Frame.grid(row=0,column=0)
            for i, ii in enumerate(Map):
                Label(Map_Frame, text = '   ' + str(int(i)) + '   ', underline = 3).grid(row = i+1, column = 0)
                for j, k in enumerate(ii):
                    if i == 0:
                        Label(Map_Frame, text = '   ' + str(int(j)) + ':  ', underline = 3).grid(row = 0, column = j+1)
        
        #End current window and run AutoNav
        def endit():
            print('ending mapper')
            if 2 in Map and 3 in Map:
                global direction
                direction = var.get()
                if direction == 'Positive Y':
                    direction = 'PosY'
                elif direction == 'Negative Y':
                    direction = 'NegY'
                elif direction == 'Positive X':
                    direction = 'PosX'
                else:
                    direction = 'NegX'
                print(direction)
                mapper.destroy()
            else:
                tkMessageBox.showerror("ERROR", "The Map must Contain a Start (2) and a Goal (3)")

        #method used for changing the map data and showing the user the new map
        def enter_data(row,col,state):
            global Map
            global goalX
            global goalY
            global startX
            global startY
            global Map_Frame
            Obst_Row.delete(0,END)
            Obst_Col.delete(0,END)
            Obst_Stt.delete(0,END)
            if row.isdigit() == True and col.isdigit() == True and state.isdigit() == True:
                if int(state) == 0 or int(state) == 1 or int(state) == 2 or int(state) == 3:
                    #Replace any current 2 values with a 0
                    if int(state) == 2 and int(state) in Map:
                        startData = str(np.argwhere(Map == 2))
                        y, x = int(startData[2]), int(startData[4])
                        print(y,x)
                        Map[y][x] = 0
                        Label(Map_Frame, text = '       ', pady = 6).grid(row = x+1, column = y+1)

                    #Replace any current 3 values with a 0
                    if int(state) == 3 and int(state) in Map:
                        startData = str(np.argwhere(Map == 3))
                        y, x = int(startData[2]), int(startData[4])
                        Map[y][x] = 0
                        Label(Map_Frame, text = '       ', pady = 6).grid(row = x+1, column = y+1)

                    #Initiate the start and end values
                    if int(state) == 2:
                        startX = int(col)
                        startY = int(row)
                    if int(state) == 3:
                        goalX = int(col)
                        goalY = int(row) 
                    #Add value to Map and show user
                    Map[int(row)][int(col)] = int(state)
                    if int(state) == 1:
                        Label(Map_Frame, text = '   ' + str(int(state)) + '   ', pady = 6, fg = "red" ).grid(row = int(row)+1, column = int(col)+1)
                    elif int(state) == 2:
                        Label(Map_Frame, text = '   ' + str(int(state)) + '   ', pady = 6, fg = "green" ).grid(row = int(row)+1, column = int(col)+1)
                    elif int(state) == 3:
                        Label(Map_Frame, text = '   ' + str(int(state)) + '   ', pady = 6, fg = "magenta" ).grid(row = int(row)+1, column = int(col)+1)
                    else:   
                        Label(Map_Frame, text = '   ' + str(int(state)) + '   ', pady = 6).grid(row = int(row)+1, column = int(col)+1)
                #Else Errors
                else:
                    tkMessageBox.showerror("ERROR", "State must be [0, 1, 2 or 3] \n Your input: " + row + " " + col + " " + state)
            else:
                tkMessageBox.showerror("ERROR", "Please Only Enter Integer Numbers \n Your input: " + row + " " + col + " " + state)

        #Create entry area for changing map stuff
        Label(mapper, text = 'Enter Map Information: ', pady = 10).grid(row = 2, column = 0)
        Entry_Frame = LabelFrame(mapper)
        Entry_Frame.grid(row = 3, column = 0)
        Label(Entry_Frame, text = 'Row: ').grid(row = 1,column = 0)
        Obst_Row = Entry(Entry_Frame, width = 5)
        Obst_Row.grid(row = 1, column = 1)
        Label(Entry_Frame, text = 'Column: ').grid(row = 1, column = 2)
        Obst_Col = Entry(Entry_Frame, width = 5)
        Obst_Col.grid(row = 1, column = 3)
        Label(Entry_Frame, text = 'State: ').grid(row = 1, column = 4)
        Obst_Stt = Entry(Entry_Frame, width = 5)
        Obst_Stt.grid(row = 1, column = 6)
        Obst_Enter = Button(Entry_Frame, text = 'Enter', padx = 5, 
                command = lambda: enter_data(Obst_Row.get(),Obst_Col.get(),Obst_Stt.get()))
        Obst_Enter.grid(row = 1, column = 7)
        #Add Enter key pressing
        def enter_press(event):
            enter_data(Obst_Row.get(),Obst_Col.get(),Obst_Stt.get())
        mapper.bind('<Return>', enter_press)
        #Initialize the grid
        print('initing grid')
        init_Grid()
        #Button for ending the program
        Button(Entry_Frame, text = 'RUN', command = endit).grid(row = 2, column = 6)
        var = StringVar()
        #Dropdown box for picking direction
        direction_Dropdown = OptionMenu(Entry_Frame, var, 
            "Positive X","Negative X", "Positive Y", "Negative Y")
        var.set("Positive Y")
        direction_Dropdown.grid(row = 2, column = 2, columnspan = 2)
        Label(Entry_Frame, text = 'Choose initial direction:').grid(row = 2, column = 0, columnspan =2)
        mapper.mainloop()
    ####### END USER MAPPING #####################################################################

    # Main Loop for Auto Nav Processing #######################
    def AutoStart(direction):
        #Create vars for the loop
        Fin, Pathed = False, False
        nextX, nextY, current_Cost, FinY, FinX, tick, numits = 0, 0, 0, 0, 0, 0, 0
        #Create GUI to See Path Accuracy
        global AutoNav
        AutoNav = Tk()
        AutoNav.title('AutomaticNav Map')
        AutoNav.geometry('600x650+300+50')
        Auto = Scroll_Frame(AutoNav, borderwidth = 2, relief = tk.SUNKEN, height = 550, width = 550)
        Auto.grid(row = 0, column = 0)
        ############################################################
        #Create a GUI grid and Display state of each node as well as the map and H values
        for i, ii in enumerate(Map):
            Label(Auto, text = '   ' + str(int(i*Rows+1)) + '   ', underline = 3).grid(row = i+1, column = 0)
            for j, state in enumerate(ii):
                if i == 0:
                    Label(Auto, text = '   ' + str(int(j)) + ':  ', underline = 3).grid(row = 0, column = j+1)
                state = int(state)
                tick += 1
                Map_pos[i][j] = tick
                if state == 0:
                    pass
                    '''if Columns < 25:
                        Label(Auto, text = state, padx = 10, pady = 10).grid(row=i+1,column=j+1)
                    else:
                        Label(Auto, text = state, padx = 5).grid(row=i+1,column=j+1)'''
                elif state == 1:
                    if Columns < 25:
                        Label(Auto, text = state, padx = 10, pady = 10, fg='red').grid(row=i+1,column=j+1)
                    else:
                        Label(Auto, text = state, padx = 5, fg='red').grid(row=i+1,column=j+1)
                elif state == 2:
                    if Columns < 25:
                        Label(Auto, text = state, padx = 10, pady = 10, fg='blue').grid(row=i+1,column=j+1)
                    else:
                        Label(Auto, text = state, padx = 5, fg='blue').grid(row=i+1,column=j+1)
                    nextX = j
                    nextY = i
                    Close.append(tick)
                else:
                    if Columns < 25:
                        Label(Auto, text = state, padx = 10, pady = 10, fg='green').grid(row=i+1,column=j+1)
                    else:
                        Label(Auto, text = state, padx = 5, fg='green').grid(row=i+1,column=j+1)
                #Calculate HVal
                Hvals[i][j] = abs(goalX - j)+abs(goalY-i)
        ##############################################################

        # Set up a method to iterate over and find a path
        def Iterate_Auto(i,j,state,direction):
            #see if position is open and threatless
            if state != 1:
                #establish if current position has a parent or not
                #Check old G val against New Gvals (space in front)
                if i != 0 and Map_pos[i-1][j] not in Close:
                    if Map[i-1][j] == 1:
                                Close.append(int(Map_pos[i-1][j]))
                    else:
                        #If it is a new position, add it to open
                        if Map_pos[i-1][j] not in Open:
                            Open.append(int(Map_pos[i-1][j]))
                        #See if direction is Positive Y
                        if direction == 'PosY':
                            #See if Gvalue in spot is bigger than the current cost
                            if Gvals[i-1][j] > current_Cost+5 or Gvals[i-1][j] == 0:
                                #Change (or add) its parent
                                Parent[i-1][j] = Map_pos[nextY][nextX]
                                #Replace (or add) its new gvalue
                                Gvals[i-1][j] = 5 + current_Cost
                        #Repeat for different directions
                        elif direction == 'NegY':
                            if Gvals[i-1][j] > current_Cost+15 or Gvals[i-1][j] == 0:
                                Parent[i-1][j] = Map_pos[nextY][nextX]
                                Gvals[i-1][j] = 15 + current_Cost
                        else:
                            if Gvals[i-1][j] > current_Cost+10 or Gvals[i-1][j] == 0:
                                Parent[i-1][j] = Map_pos[nextY][nextX]
                                Gvals[i-1][j] = 10 + current_Cost
                #See if position is behind the robot 
                if i != Rows-1 and Map_pos[i+1][j] not in Close:
                    if Map[i+1][j] == 1:
                                Close.append(int(Map_pos[i+1][j]))
                    else:
                        if Map_pos[i+1][j] not in Open:
                                Open.append(int(Map_pos[i+1][j]))
                        if direction == 'PosY':
                            if Gvals[i+1][j] > current_Cost+15 or Gvals[i+1][j] == 0:
                                Parent[i+1][j] = Map_pos[nextY][nextX]
                                Gvals[i+1][j] = 15 + current_Cost
                        elif direction == 'NegY':
                            if Gvals[i+1][j] > current_Cost+5 or Gvals[i+1][j] == 0:
                                Parent[i+1][j] = Map_pos[nextY][nextX]
                                Gvals[i+1][j] = 5 + current_Cost
                        else:
                            if Gvals[i+1][j] > current_Cost+10 or Gvals[i+1][j] == 0:
                                Parent[i+1][j] = Map_pos[nextY][nextX]
                                Gvals[i+1][j] = 10 + current_Cost        
                #See if position is to our left
                if j != 0 and Map_pos[i][j-1] not in Close:
                    if Map[i][j-1] == 1:
                                Close.append(int(Map_pos[i][j-1]))
                    else:
                        if Map_pos[i][j-1] not in Open:
                                    Open.append(int(Map_pos[i][j-1]))
                        if direction == 'NegX':
                            if Gvals[i][j-1] > current_Cost+5 or Gvals[i][j-1] == 0:
                                Parent[i][j-1] = Map_pos[nextY][nextX]
                                Gvals[i][j-1] = 5 + current_Cost
                        elif direction == 'PosX':
                            if Gvals[i][j-1] > current_Cost+15 or Gvals[i][j-1] == 0:
                                Parent[i][j-1] = Map_pos[nextY][nextX]
                                Gvals[i][j-1] = 15 + current_Cost
                        else:
                            if Gvals[i][j-1] > current_Cost+10 or Gvals[i][j-1] == 0:
                                Parent[i][j-1] = Map_pos[nextY][nextX]
                                Gvals[i][j-1] = 10 + current_Cost
                #See if position is to our right
                if j != Columns-1 and Map_pos[i][j+1] not in Close:
                    if Map[i][j+1] == 1:
                                Close.append(int(Map_pos[i][j+1]))
                    else:
                        if Map_pos[i][j+1] not in Open:
                            Open.append(Map_pos[i][j+1])
                        if direction == 'NegX':
                            if Gvals[i][j+1] > current_Cost+15 or Gvals[i][j+1] == 0:
                                Parent[i][j+1] = Map_pos[nextY][nextX]
                                Gvals[i][j+1] = 15 + current_Cost
                        elif direction == 'PosX':
                            if Gvals[i][j+1] > current_Cost+5 or Gvals[i][j+1] == 0:
                                Parent[i][j+1] = Map_pos[nextY][nextX]
                                Gvals[i][j+1] = 5 + current_Cost
                        else:
                            if Gvals[i][j+1] > current_Cost+10 or Gvals[i][j+1] == 0:
                                Parent[i][j+1] = Map_pos[nextY][nextX]
                                Gvals[i][j+1] = 10 + current_Cost             
            else:
                pass
        ###############################################################################

        #Generate a path (Current speed = 50 iterations per second)
        while Fin == False:
            #Count iterations for data colection
            numits +=1
            #Now fix a process that repeats itself until finding the goal
            minF = 100000
            #Iterate over each block and calculate the cost in the block
            Iterate_Auto(nextY, nextX, Map[nextY][nextX],direction)
            #calculate F values and look for the next position to work from
            Fvals = np.add(Gvals, Hvals)
            #Iterate through Map looking for next position to work with
            prevX = nextX
            prevY = nextY
            for data in Open:
                #Find the position in the map
                CheckF = str(np.argwhere(Map_pos == data))
                if len(CheckF) == 7:
                    i, j = int(CheckF[2]),int(CheckF[4])
                elif len(CheckF) == 9:
                    i, j = int(CheckF[2]+CheckF[3]),int(CheckF[5]+CheckF[6])
                else:
                    i, j = int(CheckF[2]+CheckF[3]+CheckF[4]),int(CheckF[6]+CheckF[7]+CheckF[8])
                #Check if that positions F value is smallest
                if Fin == False:
                    if Fvals[i][j] < minF:
                                minF = Fvals[i][j]
                                nextX = j
                                nextY = i
                                current_Cost = Gvals[i][j]
                #Check if that value is the goal
                if Map[i][j] == 3:
                    Fin = True
                    FinX = j
                    FinY = i
                    nextX = j
                    nextY = i
            #Change direction to be what we are currently facing
            if Map_pos[prevY][prevX] == Map_pos[nextY][nextX]+Rows:
                direction = 'PosY'  
            if Map_pos[prevY][prevX] == Map_pos[nextY][nextX]-Rows:
                direction = 'NegY'
            if Map_pos[prevY][prevX] == Map_pos[nextY][nextX]+1:
                direction = 'NegX'
            if Map_pos[prevY][prevX] == Map_pos[nextY][nextX]-1:
                direction = 'PosX'
            #Append the Close and Open list with the mose recent vals
            Close.append(int(Map_pos[nextY][nextX]))
            Open.remove(int(Map_pos[nextY][nextX]))
            print('Next move at: ', nextX,nextY,' Iteration: ', numits)

        #Now write out the fastest path
        Path.append(Map_pos[goalY][goalX])
        Label(Auto, text = 'G', padx = 10, pady = 10,fg = 'magenta').grid(row=goalY+1,column=goalX+1)
        Path.append(Parent[FinY][FinX])
        while Pathed == False:
            for i, ii in enumerate(Map):
                for j, k in enumerate(ii):
                    if Map_pos[i][j] == Parent[FinY][FinX]:
                        if Parent[i][j] != 0:
                            Path.append(Parent[i][j])
                        FinY = i
                        FinX = j
                        if Columns < 25:
                            Label(Auto, text = 'P', padx = 10, pady = 10,fg = 'blue').grid(row=i+1,column=j+1)
                        else:
                            Label(Auto, text = 'P', padx = 5, fg = 'blue').grid(row=i+1,column=j+1)
            if 2 == Map[FinY][FinX]:
                Label(Auto, text = 'S', padx = 10, pady = 10,fg = 'green').grid(row=FinY+1,column=FinX+1)
                Pathed = True
                #Uncomment the END now feature to ignore loading gui and see total speed of auto nav
                #endNow = True

        #Used if testing the speed of the program
        #if endNow == True:
        #   sys.exit()

        #Methods that either allow the path to be run or not 
        def Run_The_Path():
            # Used to allow user to run the path
            global is_ok
            is_ok = True
            AutoNav.destroy()
        def Dont_Run_The_Path():
            # Used to allow user to opt out of running the path
            tkMessageBox.showerror("ERROR", "Path will not be run")
            global is_ok
            is_ok = False
            AutoNav.destroy()
        #Buttons that are used for the path to be run or not
        Run_btn = Button(AutoNav, text = 'Run Path', command = Run_The_Path)
        Run_btn.grid(row = 1, column = 0)
        DRun_btn = Button(AutoNav, text = 'Do Not Run Path', command = Dont_Run_The_Path)
        DRun_btn.grid(row = 2, column = 0)

        #Just for printing the Path for user checking 
        print('Fastest Path: ')
        Run_Path = np.zeros([len(Path)])
        for i,j in enumerate(reversed(Path)):
            Run_Path[i] = int(j)
        print(Run_Path)
        #End for Picture
        Auto.mainloop()
    ###### END MAIN LOOP FOR NAV PROCESS ###########################

    # Iterate movement over the Path
    def RunPath():
        #Create some variables that are used to keep count in the path maker
        total,current_pos,next_pos, turns = 0,0,0,0
        turn_L, turn_R = False,False
        #Flip path, since apparently reversed() doesnt work in this scenario, also they arent ints, they are doubles... uh
        Run_Path = np.zeros([len(Path)])
        for i,j in enumerate(reversed(Path)):
            Run_Path[i] = int(j)
        #initialize direction that will be used
        orient = current_Direction
        #Path Maker: Iterate over path and call movement functions
        for i, j in enumerate(Run_Path):
            #While iterating through list, assign path to variables
            if i == len(Run_Path)-1:
                current_pos = Run_Path[i]
            else:
                current_pos = Run_Path[i]
                next_pos = Run_Path[i+1]

            #Motion for Positive Y 
            if orient == 'PosY':
                #See if next position is simply a move forward
                if next_pos == current_pos-Columns:
                    total +=1
                #See if next position is to the left
                if next_pos == current_pos-1:
                    turn_L = True
                    turns +=1
                #See if next positon is to the right
                if next_pos == current_pos+1:
                    turn_R = True
                    turns +=1
                #See if next position is behind robot
                if next_pos == current_pos+Columns:
                    turn_L = True
                    turns +=2
                #Only breaks iteration of Run_Path if the next position is a turn
                if turn_L == True or turn_R == True:
                    #Check to see if any movement forward is needed
                    if total > 0:
                        auto_Foward(total)
                        total = 0
                    #then turn the direction needed
                    if turn_L == True:
                        if turns == 2:
                            orient = 'NegY'
                        else:
                            orient = 'NegX'
                        auto_Left(turns)
                        turns = 0
                        turn_L = False
                        total +=1
                    if turn_R == True:
                        orient = 'PosX'
                        auto_Right(turns)
                        turns = 0
                        turn_R = False
                        total +=1
            #Motion for Negative Y (Repeated Y motion with numbers reversed)
            elif orient == 'NegY':
                if next_pos == current_pos+Columns:
                    total +=1
                if next_pos == current_pos+1:
                    turn_L = True
                    turns +=1
                if next_pos == current_pos-1:
                    turn_R = True
                    turns +=1
                if next_pos == current_pos-Columns:
                    turn_L = True
                    turns +=2
                if turn_L == True or turn_R == True:
                    if total > 0:
                        auto_Foward(total)
                        total = 0
                    if turn_L == True:
                        if turns == 2:
                            orient = 'PosY'
                        else:
                            orient = 'PosX'
                        auto_Left(turns)
                        turns = 0
                        turn_L = False
                        total +=1
                    if turn_R == True:
                        orient = 'NegX'
                        auto_Right(turns)
                        turns = 0
                        turn_R = False
                        total +=1
            #Motion for Positive X
            elif orient == 'PosX':
                #See if next position is simply a move forward
                if next_pos == current_pos+1:
                    total +=1
                #See if next position is to the left
                if next_pos == current_pos-Columns:
                    turn_L = True
                    turns +=1
                #See if next positon is to the right
                if next_pos == current_pos+Columns:
                    turn_R = True
                    turns +=1
                #See if next position is behind robot
                if next_pos == current_pos-1:
                    turn_L = True
                    turns +=2
                #Only breaks iteration of Run_Path if the next position is a turn
                if turn_L == True or turn_R == True:
                    #Check to see if any movement forward is needed
                    if total > 0:
                        auto_Foward(total)
                        total = 0
                    #then turn the direction needed
                    if turn_L == True:
                        if turns == 2:
                            orient = 'NegX'
                        else:
                            orient = 'PosY'
                        auto_Left(turns)
                        turns = 0
                        turn_L = False
                        total +=1
                    if turn_R == True:
                        orient = 'NegY'
                        auto_Right(turns)
                        turns = 0
                        turn_R = False
                        total +=1
            #Motion for Negative X (Repeated from previous pos X one but reversed numbers)
            elif orient == 'NegX':
                if next_pos == current_pos-1:
                    total +=1
                if next_pos == current_pos+Columns:
                    turn_L = True
                    turns +=1
                if next_pos == current_pos-Columns:
                    turn_R = True
                    turns +=1
                if next_pos == current_pos+1:
                    turn_L = True
                    turns +=2
                if turn_L == True or turn_R == True:
                    if total > 0:
                        auto_Foward(total)
                        total = 0
                    if turn_L == True:
                        if turns == 2:
                            orient = 'PosX'
                        else:
                            orient = 'NegY'
                        auto_Left(turns)
                        turns = 0
                        turn_L = False
                        total +=1
                    if turn_R == True:
                        orient = 'PosY'
                        auto_Right(turns)
                        turns = 0
                        turn_R = False
                        total +=1
            #Always happens on last iteration of loop, finishes path up
            if next_pos == current_pos:
                if total > 0:
                    auto_Foward(total)
                print('Path Navigation is Finished, Returning to Main Screen')
            #if the map changed:
            #    restart the autostart()
        #### END PATH MAKER ############
    ############# END MOVEMENT OVER PATH, RESETS MAIN PROGRAM WHEN FIN ################



    #Set up sizing and workspace
    checkSize()
    if loaded == False:
        Mapper()


    print('Start Pos: ',startX, startY)
    print('Goal Pos: ',goalX, goalY)

    #Number the map
    Map_pos = np.zeros([Rows,Columns])
    #Set up corresponding values that can be used for AutoNaving
    #G value is the corresponding movementent cose
    Gvals = np.zeros([Rows,Columns])
    #H values are the distance from the goal for each spot
    Hvals = np.zeros([Rows,Columns])
    #F value are G+H
    Fvals = np.zeros([Rows,Columns])
    #Parents are the nodes that directly come from another
    Parent = np.zeros([Rows,Columns])
    #Create arrays to hold open and closed positions as well as the final path
    Open, Close, Path = [], [], []
    #Find the path (Gives error if no path can be found)
    try:
        AutoStart(direction)
        #Checks to see if user wanted to use path
        if is_ok == True:
        #Run the path iterator
            try:
                RunPath()
            except:
                tkMessageBox.showerror("ERROR", "Path not run, check controller/encoders")
    except:
        tkmessagebox.showerror("ERROR", "No Path could be found")
    try:
        AutoNav.destroy()
    except:
        pass
    # Reinitiate the main screen
    db.reset()
    init_Main()
#Defines the Main GUI

def checkforstuff():
    #Compare db.grid to db.newgrid
    # if different
    # changemap() ---> map[] = newmap[]
    #map directioons:
    # if direction == 'negX'
    # autostart()
    pass

def init_Main():
    #INITIALIZE A GUI MENU
    global dbnm
    dbnm = Tk()
    dbnm.title("DoofBot NavMenu")
    #dbnm.iconbitmap('C:/Program Files/Sublime Text 3/PythonCodes/tutorials/images/bidofftester.ico')
    dbnm.geometry("850x340")
    dbnm.resizable(width=False, height=False)
    #THIS IS THE END OF THE CODE FOR INITIALIZING A GUI



    #JUST A PLACE HOLDER
    Label(dbnm,text=('\n \n')).grid(row=0,column=0)
    #END PLACEHOLDER


    #Methods for robot arm
    #Step through command and make robot do the move
    def init_RobotArmBtns():
        #Buttons for robot arm
        global f8_btn
        f8_btn = Button(dbnm, text= ' Arm For  \n      (8)    ',pady=12)
        f8_btn.grid(row=1,column=5)
        f8_btn.bind('<ButtonPress-1>', Arm_For)
        f8_btn.bind('<ButtonRelease-1>', Stop_Arm)
        global f2_btn
        f2_btn = Button(dbnm, text= ' Arm Back  \n      (5)    ',pady=12)
        f2_btn.grid(row=2,column=5)
        f2_btn.bind('<ButtonPress-1>', Arm_Back)
        f2_btn.bind('<ButtonRelease-1>', Stop_Arm)
        global s4_btn
        s4_btn = Button(dbnm, text= ' Swiv Left  \n      (4)    ',pady=12)
        s4_btn.grid(row=2,column=4)
        s4_btn.bind('<ButtonPress-1>', Swiv_Left)
        s4_btn.bind('<ButtonRelease-1>', Stop_Arm)
        global s6_btn
        s6_btn = Button(dbnm, text= ' Swiv Right  \n      (6)    ',pady=12)
        s6_btn.grid(row=2,column=6)
        s6_btn.bind('<ButtonPress-1>', Swiv_Right)
        s6_btn.bind('<ButtonRelease-1>', Stop_Arm)
        global u7_btn
        u7_btn = Button(dbnm, text= ' Arm Up  \n      (9)    ',pady=12)
        u7_btn.grid(row = 1, column = 6)
        u7_btn.bind('<ButtonPress-1>', Arm_Up)
        u7_btn.bind('<ButtonRelease-1>', Stop_Arm)
        global u9_btn
        u9_btn = Button(dbnm, text= ' Arm Down  \n      (7)    ',pady=12)
        u9_btn.grid(row = 1, column = 4)
        u9_btn.bind('<ButtonPress-1>', Arm_Down)
        u9_btn.bind('<ButtonRelease-1>', Stop_Arm)
        global r1_btn
        r1_btn = Button(dbnm, text= ' Roll CCW   \n Pitch Down \n Grip Close \n       (2)    ', pady = 17)
        r1_btn.grid(row=3,column=5)
        r1_btn.bind('<ButtonPress-1>', RollPitch_CCW)
        r1_btn.bind('<ButtonRelease-1>', Stop_Arm)
        global r3_btn
        r3_btn = Button(dbnm, text= ' Roll CW    \n Pitch Up   \n Grip Open \n   (3)    ', pady = 17)
        r3_btn.grid(row=3,column=6)
        r3_btn.bind('<ButtonPress-1>', RollPitch_CW)
        r3_btn.bind('<ButtonRelease-1>', Stop_Arm)
        Button(dbnm, text = ' Switch Wrist:  \n Pitch \n      (1)    ', pady = 25, command = Switch_RollPitch).grid(row=3,column=4) 
    init_RobotArmBtns()
    #END OF ROBOT ARM GUI AND CONTROLS


    

    #SET UP RADIO BUTTONS TO SHIFT FROM AUTOMATIC NAVIGATION TO MANUAL CONTROL
    #Create a method to alter the values when a radio button is pressed
    def RadBut_Press(value,Move_Type):
        global control_mode
        control_mode = value
        Label(Info_Frame, text= 'The Control Method is Now: ').grid(row=4,column=4)
        if value == 'Automatic':
            Label(Info_Frame, text= value).grid(row=5,column=4)
            Label(Info_Frame, text= ' Please Press Auto to Start ').grid(row=6,column=4)
            wasd_Disable()
            Label(Info_Frame,text="                                                   ").grid(row=7,column=4)
            MoveType_Btn = Button(dbnm, text= 'Staggered/Fluid',state=DISABLED, padx=5,pady=20, command = MoveType_press)
            MoveType_Btn.grid(row = 1, column = 2, columnspan=2)
            Button(dbnm, text= 'Auto',padx=78,pady=40, command = auto_press).grid(row = 3, column = 0, columnspan = 3)
        elif value == 'Manual':
            Label(Info_Frame, text= '  ' + value + '  ').grid(row=5,column=4)
            Label(Info_Frame,text="                                                   ").grid(row=6,column=4)
            wasd_Enable()
            MoveType_Btn = Button(dbnm, text= 'Staggered/Fluid', padx=5,pady=20, command = MoveType_press)
            MoveType_Btn.grid(row = 1, column = 2, columnspan=2)
            Button(dbnm, text= 'Auto',state = DISABLED, padx=78,pady=40, command = auto_press).grid(row = 3, column = 0, columnspan = 3)
            if Move_Type == "Fluid":
                Label(Info_Frame,text='Current Movement Type is: ').grid(row=6,column=4)
                Label(Info_Frame,text='    Fluid    ').grid(row=7,column=4)
            else:
                Label(Info_Frame,text='Current Movement Type is: ').grid(row=6,column=4)
                Label(Info_Frame,text='Staggered').grid(row=7,column=4)
    #Create radio buttons for autoNav or Manual
    Current_Mode = StringVar()
    Current_Mode.set('Manual')
    RadBut_Frame = LabelFrame(dbnm)
    RadBut_Frame.grid(row=2,column=7,padx=10,pady=10)
    MODES = [("Automatic Navigation", "Automatic"),
            ("Manual Navigation      ", "Manual")]
    Label(RadBut_Frame, text = 'Pick Control Method:             ').grid(row=2,column=4)
    for count, List in enumerate(MODES):
        Radiobutton(RadBut_Frame,text = List[0],variable = Current_Mode,value = List[1],
                    command = lambda: RadBut_Press(Current_Mode.get(),Move_Type)).grid(row = count+3, column=4)
    #THIS IS THE END OF THE RADIO BUTTONS FOR SHIFTING NAVIGATION METHODS



    #Create end Process button (We all need a way out sometimes)
    def e_press():
        pub_key('e')
    Button(dbnm, text= 'END \n (e)',pady = 12,command= e_press).grid(row=1,column=0)
    #THIS IS THE END OF THE CODE FOR THE END PROCESS BUTTON



    #CREATE BUTTON TO ALTER WHETHER MOVEMENT IS FLUID OR STAGGERED #commented out till control set up and bugs fixed/ see bugs
    def MoveType_press():
        global Move_Type
        if Move_Type == "Fluid":
            Move_Type = "Staggered"
            Label(Info_Frame,text='Current Movement Type is: ').grid(row=6,column=4)
            Label(Info_Frame,text='Staggered').grid(row=7,column=4)
            pub_key('stag')
        else:
            Move_Type ="Fluid"
            Label(Info_Frame,text='Current Movement Type is: ').grid(row=6,column=4)
            Label(Info_Frame,text='    Fluid    ').grid(row=7,column=4)
            pub_key('fluid')
    #THIS IS THE END OF THE CODE FOR ALTERING MOVEMENT BUTTON



    #SET UP A DIALOG BOX TO COMMUNICATE WITH USER
    Info_Frame = LabelFrame(dbnm)
    Info_Frame.grid(row=3,column=7,padx=10,pady=10)
    Label(Info_Frame,text="Info: ").grid(row=3,column=4)
    Label(Info_Frame,text="                                                   ").grid(row=4,column=4)
    Label(Info_Frame,text="                                                   ").grid(row=5,column=4)
    Label(Info_Frame,text="                                                   ").grid(row=6,column=4)
    Label(Info_Frame,text="                                                   ").grid(row=7,column=4)
    RadBut_Press(Current_Mode.get(),Move_Type)
    #THIS IS THE END OF THE CODE FOR THE DIALOG BOX



    #CREATES A BUTTON FOR CHANGING THE MANUAL SPEED CONTROL
    #Set up methods to change the serial values when sliders are used
    def For_Left_Changed(var):
        global For_Left
        For_Left = 64 + For_Left_Slider.get()
        Label(options,text= 'Current Forward/Left Speed Setting at = ' + str(For_Left) + '\n' + 
                    'Adjusted, It is : ' + str(int((100/63)*For_Left_Slider.get())) + ' Percent').grid(row=2)

    def For_Right_Changed(var):
        global For_Right
        For_Right = 192 + For_Right_Slider.get()
        Label(options,text= 'Current Forward/Right Speed Setting at = ' + str(For_Right) + '\n' + 
                    'Adjusted, It is : ' + str(int((100/63)*For_Right_Slider.get())) + ' Percent').grid(row=5)
    def Back_Left_Changed(var):
        global Back_Left
        Back_Left = 64 - Back_Left_Slider.get()
        Label(options,text= 'Current Backwards/Left Speed Setting at = ' + str(Back_Left) + '\n' + 
                    'Adjusted, It is : ' + str(int((100/63)*Back_Left_Slider.get())) + ' Percent').grid(row=8)
    def Back_Right_Changed(var):
        global Back_Right
        Back_Right = 192 - Back_Right_Slider.get()
        Label(options,text= 'Current Backwards/Right Speed Setting at = ' + str(Back_Right) + '\n' + 
                    'Adjusted, It is : ' + str(int((100/63)*Back_Right_Slider.get())) + ' Percent').grid(row=11)
    def end_options():
        global optionsOpen
        optionsOpen = False
        options.destroy()
    #Define the actual menu and sliders in the options GUI
    def open_options():
        global optionsOpen
        if optionsOpen == False:
            global options
            global For_Left_Slider
            global For_Right_Slider
            global Back_Left_Slider
            global Back_Right_Slider
            options = Tk()
            options.title("DoofBot Options")
            options.geometry("350x500")
            options.resizable(width=False, height=False)
            #options.iconbitmap('C:/Program Files/Sublime Text 3/PythonCodes/tutorials/images/bidofftester.ico')
            Label(options,text='\n Manually Adjust the Left Wheels Forward Speed').grid(row=0)
            For_Left_Slider = Scale(options, from_=1, to= 63,orient=HORIZONTAL,command= For_Left_Changed)
            For_Left_Slider.grid(row=1)
            For_Left_Slider.set(For_Left-64)
            Label(options,text= 'Current Forward/Left Speed Setting at = ' + str(For_Left) + '\n' + 
                        'Adjusted, It is : ' + str(int((100/63)*For_Left_Slider.get())) + ' Percent').grid(row=2)

            Label(options,text='\n Manually Adjust the Right Wheels Forward Speed').grid(row=3)
            For_Right_Slider = Scale(options, from_=1, to= 63,orient=HORIZONTAL,command= For_Right_Changed)
            For_Right_Slider.grid(row=4)
            For_Right_Slider.set(For_Right-192)
            Label(options,text= 'Current Forward/Right Speed Setting at = ' + str(For_Right) + '\n' + 
                        'Adjusted, It is : ' + str(int((100/63)*For_Right_Slider.get())) + ' Percent').grid(row=5)

            Label(options,text='\n Manually Adjust the Left Wheels Backwards Speed').grid(row=6)
            Back_Left_Slider = Scale(options, from_=1, to= 63,orient=HORIZONTAL,command= Back_Left_Changed)
            Back_Left_Slider.grid(row=7)
            Back_Left_Slider.set(64 - Back_Left)
            Label(options,text= 'Current Backwards/Left Speed Setting at = ' + str(Back_Left) + '\n' + 
                        'Adjusted, It is : ' + str(int((100/63)*Back_Left_Slider.get())) + ' Percent').grid(row=8)

            Label(options,text='\n Manually Adjust the Right Wheels Backwards Speed').grid(row=9)
            Back_Right_Slider = Scale(options, from_=1, to= 63,orient=HORIZONTAL,command= Back_Right_Changed)
            Back_Right_Slider.grid(row=10)
            Back_Right_Slider.set(192 - Back_Right)
            Label(options,text= 'Current Backwards/Right Speed Setting at = ' + str(Back_Right) + '\n' + 
                        'Adjusted, It is : ' + str(int((100/63)*Back_Right_Slider.get())) + ' Percent').grid(row=11)
            optionsOpen = True
            options.protocol("WM_DELETE_WINDOW", end_options)
        else:
            pass
    #Creates the button to open the options GUI
    option_btn = Button(dbnm, text= 'Speed \n Options',pady=33,command=open_options, state = DISABLED)
    option_btn.grid(row=3,column=3)
    #THIS IS THE END OF THE OPTIONS TAB FOR MANUAL SPEED CONTROL



    #CREATE BOX TO HOLD CURRENT ENCODER VALUES
    #PI VIEW
    try:
        encoder1 = enc.LS7366R(0, 1000000, 4)
        encoder2 = enc.LS7366R(1, 1000000, 4)
        Sout_Frame = LabelFrame(dbnm)
        Sout_Frame.grid(row=1,column=7,padx=10,pady=10)
        check_serial=1
        if check_serial == 1:
            s1 = str(encoder1.readCounter())
            s2 = str(encoder2.readCounter())
            Label(Sout_Frame,text='    Current Encoder Values:      \n Left:         ' + s1 + '\n Right:         ' + s2).grid(row=0,column=4)
            check_serial=0
    #PC VIEW
    except:
        Sout_Frame = LabelFrame(dbnm)
        Sout_Frame.grid(row=1,column=7,padx=10,pady=10)
        check_serial=1
        if check_serial == 1:
            s1 = 'N/A'
            s2 = 'N/A'
            Label(Sout_Frame,text='    Current Encoder Values:      \n Left:         ' + s1 + '\n Right:         ' + s2).grid(row=0,column=4)
            check_serial=0
    ##########   THIS IS THE END OF CODE FOR THE ENCODER READING BOX



    wasd_Enable()
    dbnm.mainloop()


if __name__ == '__main__':
	
    #Turn off repeating keys (needed to run the keyboard control correctly)
    os.system('xset r off')

    #Import the camera image as a subprocess
    proc = subprocess.Popen(["roslaunch","doofbot","db_image.launch"])


    #Create a publisher note that publishes a string readable by the robot
    rospy.init_node("key_publisher")
    pubber = publisher()
    pub_error_l = publisher_error_l()
    pub_error_r = publisher_error_r()
    pub_error_lt = publisher_error_lt()
    pub_error_rt = publisher_error_rt()


    ### GUI INITIALIZATION ####
    # INITILIZE VALUES
    Stop_Left = 64
    Stop_Right = 192
    For_Left = 78
    For_Right = 204
    Back_Left = 51
    Back_Right = 177
    current_Direction = 'PosY'
    Move_Type = "Fluid"
    buttonPressed = False
    optionsOpen = False
    RollPitch = 'Pitch'
    speed_roboArm = 235
    r = np.zeros(7)
    step_cmd = ''
    armMoving = False
    cur_left = 0
    cur_right = 0
    press_ticks, depress_ticks = 0,0
    accepted_keys = ('w','a','s','d','e','1','2','3','4',
                        '5','6','7','8','9','stag','fluid','clear_encoder',
                            'rotateLeft','rotateRight','move1','move2','move3','move4')

    db = doofbot()
    #Initilize the main GUI
    init_Main()
