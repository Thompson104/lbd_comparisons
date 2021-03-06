"""--------------------------------------------------------------------
COPYRIGHT 2015 Stanley Innovation Inc.

Software License Agreement:

The software supplied herewith by Stanley Innovation Inc. (the "Company") 
for its licensed Segway RMP Robotic Platforms is intended and supplied to you, 
the Company's customer, for use solely and exclusively with Stanley Innovation 
products. The software is owned by the Company and/or its supplier, and is 
protected under applicable copyright laws.  All rights are reserved. Any use in 
violation of the foregoing restrictions may subject the user to criminal 
sanctions under applicable laws, as well as to civil liability for the 
breach of the terms and conditions of this license. The Company may 
immediately terminate this Agreement upon your use of the software with 
any products that are not Stanley Innovation products.

The software was written using Python programming language.  Your use 
of the software is therefore subject to the terms and conditions of the 
OSI- approved open source license viewable at http://www.python.org/.  
You are solely responsible for ensuring your compliance with the Python 
open source license.

You shall indemnify, defend and hold the Company harmless from any claims, 
demands, liabilities or expenses, including reasonable attorneys fees, incurred 
by the Company as a result of any claim or proceeding against the Company 
arising out of or based upon: 

(i) The combination, operation or use of the software by you with any hardware, 
    products, programs or data not supplied or approved in writing by the Company, 
    if such claim or proceeding would have been avoided but for such combination, 
    operation or use.
 
(ii) The modification of the software by or on behalf of you 

(iii) Your use of the software.

 THIS SOFTWARE IS PROVIDED IN AN "AS IS" CONDITION. NO WARRANTIES,
 WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED
 TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE COMPANY SHALL NOT,
 IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR
 CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 
 \file   robotiq_85_test.py

 \brief  Node for testing Robotiq 85 communication

 \Platform: Linux/ROS Indigo
--------------------------------------------------------------------"""
import rospy
from robotiq_85_msgs.msg import GripperCmd, GripperStat
import evdev
import pickle
PICKLE_DIR_BASE = '/home/hwang/log_files/gripper_status/'

class Robotiq85GripperTest:

    def __init__(self):
    
        self._num_grippers = rospy.get_param('~num_grippers',1)
        
        if (self._num_grippers == 1):
            rospy.Subscriber("/gripper/stat", GripperStat, self._update_gripper_stat, queue_size=10)
            self._gripper_pub = rospy.Publisher('/gripper/cmd', GripperCmd, queue_size=10)      
        elif (self._num_grippers == 2):
            rospy.Subscriber("/left_gripper/stat", GripperStat, self._update_gripper_stat, queue_size=10)
            self._left_gripper_pub = rospy.Publisher('/left_gripper/stat', GripperCmd, queue_size=10)
            rospy.Subscriber("/right_gripper/stat", GripperStat, self._update_right_gripper_stat, queue_size=10)
            self._right_gripper_pub = rospy.Publisher('/right_gripper/cmd', GripperCmd, queue_size=10)
        else:
            rospy.logerr("Number of grippers not supported (needs to be 1 or 2)")
            return
            
        self._gripper_stat = [GripperStat()] * self._num_grippers
        self._gripper_cmd = [GripperCmd()]  * self._num_grippers
            
        self._run_test()
        
        
    def _update_gripper_stat(self, stat):
        self._gripper_stat[0] = stat
    def _update_right_gripper_stat(self, stat):
        self._gripper_stat[1] = stat
    
    
    def _run_test(self):
        test_state = 0
        state = False
        r = rospy.Rate(25)
	
        #device = evdev.InputDevice('/dev/input/event18')
        while not rospy.is_shutdown():
            ready = False
            while not (ready):
                ready = True
                for i in range(self._num_grippers):
                    ready &= self._gripper_stat[i].is_ready
            '''
            if state == True:
                GRIPPER = 'close'
            else:
                GRIPPER = 'open'
            print GRIPPER, state
            button_key_event_middle = False
            for event in device.read_loop():
                if event.type == evdev.ecodes.EV_KEY:
                    #event.code for |> is 164, event.val: 01 for press and 00 for release
                    if event.code == 164 and event.value == 0:
                        button_key_event_middle = True
                    if button_key_event_middle == True and GRIPPER == 'open':
                        button_key_event_middle = False
                        print("Close Gripper!")
                        GRIPPER = 'close'
                    #    set_gripper_status(gripper=GRIPPER)
                        break
                        
                    if button_key_event_middle == True and GRIPPER == 'close':
                        button_key_event_middle = False
                        print("Open Gripper!")
                        GRIPPER = 'open'
                    #    set_gripper_status(gripper=GRIPPER)
                        break

            #flip the state
            if GRIPPER == "open" and state == True:
                state = False
            elif GRIPPER == "close" and state == False:
                state = True
            '''
            pkl_file = open(PICKLE_DIR_BASE+'gripper_status.pkl', 'rb')
            status = pickle.load(pkl_file)
	    print status
            if status == "open" and state == True:
                state = False
            elif status == "close" and state == False:
                state = True

            #close the gripper when button trigging the signal
            if (state == True):
                for i in range(self._num_grippers):
                    self._gripper_cmd[i].position = 0.0
            # self._gripper_cmd[i].position = 0.085/2.0
                    self._gripper_cmd[i].speed = 0.1
                    self._gripper_cmd[i].force = 90.0             

            #open the gripper when the gripper is closed and the signal is triggered
            if (state == False):
                for i in range(self._num_grippers):
                    self._gripper_cmd[i].position = 0.085
                    self._gripper_cmd[i].speed = 0.1
                    self._gripper_cmd[i].force = 90.0
                    
            if (self._num_grippers == 1):
                self._gripper_pub.publish(self._gripper_cmd[0])    
            elif (self._num_grippers == 2):
                self._left_gripper_pub.publish(self._gripper_cmd[0])
                self._right_gripper_pub.publish(self._gripper_cmd[1])
            r.sleep()
