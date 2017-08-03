#!/usr/bin/env python
# code: utf-8 -*-
# author: hwang
# created on: 05/25/2017
# modified on: 06/01/2017
import threading
import rospy
from sensor_msgs.msg import JointState
from playbackUtils import *
import csv
import time
import socket
import copy
from robotiq_85_msgs.msg import GripperCmd, GripperStat

playback_file_path_ur = '/home/hwang/log_files/kinematics_demo/data_tables/'
playback_file_path_gripper = '/home/hwang/log_files/kinematics_demo/data_tables/'
BAG_NAME_BASE = '/home/hwang/log_files/kinematics_demo/'

"""
This module helps communicate with the UR robot via xmlrpc. The class is designed to contain all the remote procedure
calls (prefixed by rpc) and helper functions for the server side. The server runs here and the client runs on the UR
robot. The UR continuously updates it's controller parameters through these rpc calls. The rpc server runs in a
seperate thread. Once the main thread is complete the server thread exits.
"""

def get_prefix():
    with open(BAG_NAME_BASE+'bag_name.txt', 'rb') as bag_name_file:
    	prefix=bag_name_file.readline()
    return prefix.rstrip('\n').rstrip(".bag")

def process_time_table(time_ns=None):
    tDiv = 1000000000.0
    init_time = copy.deepcopy(time_ns[0])
    time_ns[0] = 0.0
    for i in range(1, len(time_ns)):
	time_ns[i] = (time_ns[i] - init_time) / tDiv
    return time_ns

def find_closest(A, target):
    #A must be sorted
    idx = A.searchsorted(target)
    idx = np.clip(idx, 1, len(A)-1)
    left = A[idx-1]
    right = A[idx]
    idx -= target - left < right - target
    return idx

def moveJ_ur5(js=None, sock=None):
    q = js
    command = "movej([{0},{1},{2},{3},{4},{5}],t=6)".format(str(q[0]), str(q[1]), str(q[2]), str(q[3]), str(q[4]),str(q[5])) + "\n"
    sock.send(command)

def move_ur5(js):
    q = js
    gain = '200'
    command = "servoj([{0},{1},{2},{3},{4},{5}],t=.13,lookahead_time=0.01,gain={6})".format(str(q[0]),str(q[1]),str(q[2]),str(q[3]),str(q[4]),str(q[5]),gain) + "\n"
    sock.send(command)

def pubGripperROS(cmd=None,gripper_cmd_val=None,publisher=None):
    gripper_cmd_val.position = cmd[0]
    gripper_cmd_val.speed = cmd[1]
    gripper_cmd_val.force = cmd[2]
    publisher.publish(gripper_cmd_val)
    
def publish(js=None, sock=None, g_cmd=None, gripper_cmd_val=None, publisher=None):
    pubGripperROS(cmd=g_cmd, gripper_cmd_val=gripper_cmd_val, publisher=publisher)
    move_ur5(js)

def load_ur5_motion(prefix=None):
    data_table = []
    with open(playback_file_path_ur+prefix+'_ur.csv', 'rb') as csv_file:
	csv_reader = csv.reader(csv_file)
	for row in csv_reader:
	    stamp_tmp = float(row[0])
	    js_tmp = row[1].lstrip('[').rstrip(']').split(',')
	    js_float_tmp = [float(js) for js in js_tmp]
	    data_table.append([stamp_tmp, js_float_tmp])
    times_np = [item[0] for item in data_table]
    joint_states = [item[1] for item in data_table]
    return times_np, joint_states

def load_gripper_motion(prefix=None):
    data_table = []
    with open(playback_file_path_gripper+prefix+'_gripper.csv', 'rb') as csv_file:
	csv_reader = csv.reader(csv_file)
	for row in csv_reader:
	    stamp_tmp = float(row[0])
	    gripper_cmd = [float(i) for i in row[1:]]
	    data_table.append([stamp_tmp, gripper_cmd])
    times_np_gripper = [item[0] for item in data_table]
    gripper_cmd = [item[1] for item in data_table]
    return times_np_gripper, gripper_cmd

if __name__ == "__main__":
    # set socket to send command to robot
    a = rospy.init_node("play_back_test")
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect(("192.168.1.108",30002))
    gripper_publisher = rospy.Publisher('/gripper/cmd', GripperCmd, queue_size=10)
    gripper_cmd_val = GripperCmd()

    file_name_basis = get_prefix()
    # load ur5 data from csv
    times_np_ur, joint_states = load_ur5_motion(prefix=file_name_basis)
    times_np_gripper, gripper_cmd = load_gripper_motion(prefix=file_name_basis)

    # load gripper data from csv
    init_state = joint_states[0]
    # move to the initial joint state 
    moveJ_ur5(init_state, sock=sock)
    rospy.sleep(12)
    # convert time from nsec to sec, start from 0
    csv_times_ur = process_time_table(times_np_ur)
    csv_times_gripper = process_time_table(times_np_gripper)
    # stupid things here, should be revised
    times_np_ur = np.array(csv_times_ur)
    times_np_gripper = np.array(csv_times_gripper)
    
    tDiv = 1000000000.0
    endTime_ur = csv_times_ur[-1]
    endTime_gripper = csv_times_gripper[-1]
    startTime = float(rospy.Time.now().to_nsec())
    currTime = 0.0
    while not currTime >= max(endTime_ur, endTime_gripper) and not rospy.is_shutdown():
#        encoder = []
        currTime = float(rospy.Time.now().to_nsec() - startTime) / tDiv
        idx_ur = find_closest(times_np_ur, currTime)
        idx_gripper = find_closest(times_np_gripper, currTime)
        print idx_ur, idx_gripper
#        encoder = csv_encoder[idx]
#        q = csv_quat[idx]
#        q = rotate_quat(q, -.707, 'z')
        # q = rotate_quat(q, .707, 'x')
        # q = rotate_quat(q, -M.pi/2, 'z')
#        print currTime
        js_cmd = joint_states[idx_ur]
	g_cmd = gripper_cmd[idx_gripper]
	# this is quite important
        rospy.sleep(0.004)
        # do something on the gripper
        
        # move the robot
        publish(js_cmd, sock=sock, g_cmd=g_cmd, gripper_cmd_val=gripper_cmd_val, publisher=gripper_publisher)
        currTime = float(rospy.Time.now().to_nsec() - startTime) / tDiv
    '''
    init_state = joint_states[0]
    rcom_inst.is_init = 1
    rcom_inst.pose_state = init_state
    a = rospy.init_node("play_back_test")
    time.sleep(7)
    rcom_inst.is_init = 0
    idx = 1
    rate = rospy.Rate(27)
    rcom_inst.is_init = 0
    while True:
        rcom_inst.is_init = 0
	if idx < len(joint_states):
	    rcom_inst.pose_state = joint_states[idx]
	    idx+=1
        else:
	    break
	rate.sleep()
    '''
    
    
    


