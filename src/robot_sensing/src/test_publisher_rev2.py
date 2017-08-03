#!/usr/bin/env python
# license removed for brevity
# code: -*- utf-8 -*-
# author: hwang
# created on: 06/14/2017
# modified on: 07/20/2017
import rospy
import numpy as np
import sys
import csv
import random
import math

import tf as tf_rev
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

from PyQt4 import QtGui
from PyQt4.QtCore import *
from PyQt4.QtGui import *

from matplotlib.backends.backend_qt4agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt4agg import NavigationToolbar2QT as NavigationToolbar
from matplotlib import colors as mcolors
import matplotlib.pyplot as plt

from transformations.transformations import quaternion_disp, quaternion_multiply, quaternion_matrix

# motion switches
IS_FILTERED = False
IS_REVISED = True
IS_LAST_MOTION = False

# global vars
LBD_PLAY_BACK_BASE_DIR = '/home/hwang/log_lbd_playback/csv_file/'
PARAM_RT_DIR_BASE = '/home/hwang/log_lbd_playback/param_log_file/'
PREDEFINED_POS_OBJ_THRESHOLD = 0.0001
topic = 'visualization_marker_array'
objects = ('s_p', 's_l', 'elb', 'w_1', 'w_2', 'w_3')

# set dict for matplotlib color lists
colors = dict(mcolors.BASE_COLORS, **mcolors.CSS4_COLORS)

# ROS settings
publisher = rospy.Publisher(topic, MarkerArray, queue_size=10)
pub = rospy.Publisher("ur5/joint_states", JointState, queue_size=10)
broadCaster = tf_rev.TransformBroadcaster()

#pub = rospy.Publisher("joint_states", JointState, queue_size=10)
rospy.init_node("test_playback_node", anonymous=True)

# define markerArray type to show trajecotry
markerArray = MarkerArray()

# define jointState type to publish real robot states
js = JointState()
js.name = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
js.position = [ 3.47172217e+00,  -2.14456987e+00,  -1.91532757e+00,   9.18540937e-01, -3.30283928e-01,  -3.21397317e-04]

# Only used for distinguish from file names
if IS_LAST_MOTION:
    JS_FILE_NAME = "js_mg_627_2_rev.csv"
    MOTION_FILE_NAME = "mg_627_2_rev.csv"
    PARAM_FILE_NAME = "mg_627_2_rev.csv"
    ERR_FILE_NAME = "err_mg_627_2_rev.csv"
else:
    if IS_FILTERED:
        JS_FILE_NAME = "js_test_traj_1_fil.csv"
        MOTION_FILE_NAME = "test_traj_1_fil.csv"
        PARAM_FILE_NAME = "test_traj_1_fil.csv"
        ERR_FILE_NAME = "err_test_traj_1_rev.csv"
        '''
        JS_FILE_NAME = "js_test_traj_0_fil.csv"
        MOTION_FILE_NAME = "test_traj_0_fil.csv"
        PARAM_FILE_NAME = "test_traj_0_fil.csv"
        '''
    elif IS_REVISED:
        '''
        JS_FILE_NAME = "js_test_traj_3_rev.csv"
        MOTION_FILE_NAME = "test_traj_3_rev.csv"
        PARAM_FILE_NAME = "test_traj_3_rev.csv"
        ERR_FILE_NAME = "err_test_traj_3_rev.csv"
        '''

        JS_FILE_NAME = "js_crappy_motion_0_rev.csv"
        MOTION_FILE_NAME = "crappy_motion_0_rev.csv"
        PARAM_FILE_NAME = "crappy_motion_0_rev.csv"
        ERR_FILE_NAME = "err_crappy_motion_0_rev.csv"
        '''
        JS_FILE_NAME = "js_test_traj_1_rev.csv"
        MOTION_FILE_NAME = "test_traj_1_rev.csv"
        PARAM_FILE_NAME = "test_traj_1_rev.csv"
        ERR_FILE_NAME = "err_test_traj_1_rev.csv"
        '''
        
        '''
        JS_FILE_NAME = "js_test_traj_2_rev.csv"
        MOTION_FILE_NAME = "test_traj_2_rev.csv"
        PARAM_FILE_NAME = "test_traj_2_rev.csv"
        ERR_FILE_NAME = "err_test_traj_2_rev.csv"
        '''
    else:  
        JS_FILE_NAME = "js_test_traj_1.csv"
        MOTION_FILE_NAME = "test_traj_1.csv"
        PARAM_FILE_NAME = "test_traj_1.csv"
        ERR_FILE_NAME = "err_test_traj_1_rev.csv"
        '''
        JS_FILE_NAME = "js_test_traj_0.csv"
        MOTION_FILE_NAME = "test_traj_0_raw.csv"
        PARAM_FILE_NAME = "test_traj_0.csv"
        '''

def rotate_quat(q, rot, axis):
    rot_mat = quaternion_matrix(q)
    if axis == 'x':
        a = rot_mat[0:3,0]
    elif axis == 'y':
        a = rot_mat[0:3,1]
    elif axis == 'z':
        a = rot_mat[0:3,2]
    else:
        raise ValueError('Invalid axis in rotate_quat!')

    rot_w = math.cos(rot/2.0)
    rot_axis = math.sin(rot/2.0)*a
    rot_quat = [rot_w, rot_axis[0], rot_axis[1], rot_axis[2]]
    return quaternion_multiply(rot_quat, q)

def load_motion_data(global_dir=None, data_type=None):
    '''type indicate data type of csv files. kin stands for pos and orientation
       gripper stands for encoder value of the tongs'''
    csv_times = []
    csv_pos = []
    csv_quat = []
    with open(LBD_PLAY_BACK_BASE_DIR+global_dir, 'rb') as csvfile:
        spamreader = csv.reader(csvfile)
        for row in spamreader:
            tmp_pos_list = row[1].lstrip('[').rstrip(']').split(',')
            tmp_quat_list = row[2].lstrip('[').rstrip(']').split(',')
            csv_times.append(float(row[0]))
            tmp_pos = [float(item) for item in tmp_pos_list]
            tmp_quat = [float(item) for item in tmp_quat_list]
            #tmp_pos = [element+0.06 if ele_idx == 0 else element for ele_idx, element in enumerate(tmp_pos)]
            tmp_quat=rotate_quat(tmp_quat, 1.57075, 'x')
            tmp_quat=rotate_quat(tmp_quat, -1.57075, 'z')
            csv_pos.append(tmp_pos)
            csv_quat.append(tmp_quat)
    return np.array(csv_times), csv_pos, csv_quat

def load_param_data(global_dir=None):
    params_tmp = []
    with open(PARAM_RT_DIR_BASE+global_dir, 'rb') as csvfile:
        spamreader = csv.reader(csvfile)
        for row in spamreader:
            params_tmp.append([float(item) for item in row])
    return params_tmp

def load_joint_states(file_dir="js_test_traj_1_fil.csv"):
    joint_angle_table = []
    with open(file_dir, 'rb') as csvfile:
	spamreader = csv.reader(csvfile)
	for row in spamreader:
            joint_angle_table.append([float(r) for r in row])
    return joint_angle_table

def load_ee_status(file_dir="js_test_traj_1_fil.csv"):
    '''
    load data of end effector for real time check of the error
    '''
    ee_pos = []
    ee_quat = []
    row_counter = 0
    with open(file_dir, 'rb') as csvfile:
        spamreader = csv.reader(csvfile)
        for row in spamreader:
            if row_counter == 0:
                row_counter+=1
                continue
            else:
                ee_pos.append([float(r)+0.06 if r_idx==0 else float(r) for r_idx, r in enumerate(row[0:3])])
                ee_quat.append([float(r) for r in row[3:]])
            row_counter+=1
    return ee_pos, ee_quat

def get_robot_configuration_change(js_table="err_traj_1_rev.csv"):
    config_change_list = []
    for js_idx, js_state in enumerate(js_table):
        if js_idx == 0:
            config_change_list.append(0)
        else:
            config_change_list.append(np.linalg.norm(np.subtract(js_state, js_table[js_idx-1])))
	    if config_change_list[js_idx] >= 0.45:
                print(js_idx)
                print('------------------------------------------------------------------------')
    return config_change_list

def check_fail_frames(params_list=None):
    fail_frames_indices = []
    for param_index, params in enumerate(params_list):
        if params[0] >= PREDEFINED_POS_OBJ_THRESHOLD:
	    fail_frames_indices.append(param_index)
    return fail_frames_indices

def check_motion_mapping_err(obj_pos=None, obj_quat=None, ee_pos=None, ee_quat=None):
    '''check performance of exact IK solver'''
    pos_err_list = []
    quat_err_list = []
    fixed_quat_for_plot = []
    # check for data length
    assert len(obj_pos) == len(obj_quat)
    assert len(obj_pos) == len(ee_pos) 
    for idx in range(len(obj_pos)):
        tmp_quat = [-elem for elem in ee_quat[idx]]
        metric_0 = np.linalg.norm(np.subtract(obj_quat[idx], ee_quat[idx]))
        metric_1 = np.linalg.norm(np.subtract(obj_quat[idx], tmp_quat))
        if metric_0 > metric_1:
            fixed_quat_for_plot.append(tmp_quat)
        else:
            fixed_quat_for_plot.append(ee_quat[idx])
        pos_err_list.append(np.linalg.norm(np.subtract(obj_pos[idx], ee_pos[idx])))
        quat_err_list.append(np.linalg.norm(quaternion_disp(obj_quat[idx], ee_quat[idx])))
    return pos_err_list, quat_err_list, fixed_quat_for_plot

def fill_marker_array(pos, quat, forbiden_frames):
    global markerArray, publisher
    for idx in range(len(pos)):
        marker = Marker()
        marker.header.frame_id = "ur5/world"
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.scale.x = 0.01
        marker.scale.y = 0.01
        marker.scale.z = 0.01
        if idx not in forbiden_frames:
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.2
        else:
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0
        marker.pose.orientation.w = quat[idx][0]
        marker.pose.orientation.x = quat[idx][1]
        marker.pose.orientation.y = quat[idx][2]
        marker.pose.orientation.z = quat[idx][3]
        marker.pose.position.x = pos[idx][0]
        marker.pose.position.y = pos[idx][1] 
        marker.pose.position.z = pos[idx][2]
        marker.id = idx
        markerArray.markers.append(marker)

def get_ee_js_vel(time_stamp=None, ee_pos=None, js_table=None):
    ee_vel = []
    js_vel = []
    js_space_vel = []
    for idx in range(len(ee_pos)):
        tmp_js_vel = []
        if idx == 0:
            ee_vel.append(0.0)
            js_space_vel.append(0.0)
            js_vel.append([0.0]*6)
        else:
            ee_vel.append(np.linalg.norm(np.subtract(ee_pos[idx], ee_pos[idx-1]))/(time_stamp[idx]-time_stamp[idx-1]))
            js_space_vel.append(np.linalg.norm(np.subtract(js_table[idx], js_table[idx-1]))/(time_stamp[idx]-time_stamp[idx-1]))
            for j_idx, j in enumerate(js_table[idx]):
                tmp_js_vel.append(abs(j-js_table[idx-1][j_idx])/float(time_stamp[idx]-time_stamp[idx-1]))    
            js_vel.append(tmp_js_vel)
    return ee_vel, js_vel, js_space_vel

class Window(QWidget):
    def __init__(self, parent=None, pos=None, quat=None, js_table=None, params=None, forbiden_frames=None, 
		robot_config_change_list=None, ee_pos=None, ee_quat=None, pos_err=None, quat_err=None,
        ee_vel=None, js_vel=None, js_space_vel=None):
        '''
        pos, quat stands for the position and orientation of tongs in a certain demonstration
        js_table: the joint states solved by RIK
        '''
        super(Window, self).__init__(parent)
        # figure instances to plot on
        self.figure_1 = plt.figure(1)
        self.figure_2 = plt.figure(2)

        # get init of data tables
        self._pos = pos
        self._orientation = quat
        self._js_table = js_table
        self._params = params
        # use for checking performance
        self._forbiden_frames = forbiden_frames
        self._robot_config_change = robot_config_change_list
        self._ee_pos_list = ee_pos
        self._ee_quat_list = ee_quat
        self._pos_err_list = pos_err
        self._quat_err_list = quat_err
        self._ee_velocity = ee_vel
        self._js_velocity = js_vel
        self._js_space_velocity = js_space_vel

        layout = QVBoxLayout()

        # Add label to show some texts
        self.l1 = QLabel("LBD Playback Simulator")
        self.l1.setAlignment(Qt.AlignCenter)
        self.l1.setFont(QFont("Arial", 30))
        layout.addWidget(self.l1)

        # Add other labels to show other parameter values
        # 'Orien Weight', 'EE Vel', 'JS Vel', 'Obj Val'
        '''
        self.l2 = QLabel("Position Err: 0.0")
        self.l2.setAlignment(Qt.AlignLeft)
        self.l2.setFont(QFont("Arial", 15))

        self.l3 = QLabel("Orientation Err: 0.0")
        self.l3.setAlignment(Qt.AlignLeft)
        self.l3.setFont(QFont("Arial", 15))

        self.l4 = QLabel("EE Vel: 0.0")
        self.l4.setAlignment(Qt.AlignLeft)
        self.l4.setFont(QFont("Arial", 15))

        self.l5 = QLabel("JS Val: 0.0")
        self.l5.setAlignment(Qt.AlignLeft)
        self.l5.setFont(QFont("Arial", 15))
        '''

        self.l6 = QLabel("Self Collision: False")
        self.l6.setAlignment(Qt.AlignLeft)
        self.l6.setFont(QFont("Arial", 15))

        # this is the Canvas Widget that displays the `figure`
        # it takes the `figure` instance as a parameter to __init__
        self.canvas_1 = FigureCanvas(self.figure_1)
        self.canvas_2 = FigureCanvas(self.figure_2)

        # this is the Navigation widget
        # it takes the Canvas widget and a parent
        self.toolbar_1 = NavigationToolbar(self.canvas_1, self)
        self.toolbar_2 = NavigationToolbar(self.canvas_2, self)

        self.sl = QSlider(Qt.Horizontal)
        self.sl.setMinimum(0)
        self.sl.setMaximum(len(self._pos)-1)
        self.sl.setValue(500)
        self.sl.setTickPosition(QSlider.TicksBelow)
        self.sl.setTickInterval(1)

        layout.addWidget(self.toolbar_1)
        layout.addWidget(self.canvas_1)
        layout.addWidget(self.toolbar_2)
        layout.addWidget(self.canvas_2)	
        layout.addWidget(self.sl)
#        layout.addWidget(self.l2)
#        layout.addWidget(self.l3)
#        layout.addWidget(self.l4)
#        layout.addWidget(self.l5)
        layout.addWidget(self.l6)
        self.sl.valueChanged.connect(self.valuechange)
        self.setLayout(layout)
        self.setWindowTitle("Playback Simulator with UI by HWang")

    def slider_response(self, sl_index, forbiden_frames=None):
        global markerArray, publisher, pub, js, broadCaster
        marker = Marker()
        markerArray.markers[sl_index].scale.x = 0.02
        markerArray.markers[sl_index].scale.y = 0.02
        markerArray.markers[sl_index].scale.z = 0.02
        markerArray.markers[sl_index].color.a = 1.0
        markerArray.markers[sl_index].color.r = 0.0
        markerArray.markers[sl_index].color.g = 0.0
        markerArray.markers[sl_index].color.b = 1.0
        markerArray.markers[sl_index].type = marker.CUBE
        js.position = self._js_table[sl_index]
        js.header.stamp = rospy.Time.now()
        pub.publish(js)
        publisher.publish(markerArray)
        # change last marker to normal
        markerArray.markers[sl_index].scale.x = 0.01
        markerArray.markers[sl_index].scale.y = 0.01
        markerArray.markers[sl_index].scale.z = 0.01
        if sl_index not in forbiden_frames:
            markerArray.markers[sl_index].color.a = 0.2
            markerArray.markers[sl_index].color.r = 1.0
            markerArray.markers[sl_index].color.g = 1.0
            markerArray.markers[sl_index].color.b = 0.0
        else:
            markerArray.markers[sl_index].color.a = 1.0
            markerArray.markers[sl_index].color.r = 1.0
            markerArray.markers[sl_index].color.g = 0.0
            markerArray.markers[sl_index].color.b = 0.0
        broadCaster.sendTransform((markerArray.markers[sl_index].pose.position.x, markerArray.markers[sl_index].pose.position.y, markerArray.markers[sl_index].pose.position.z),
                     (markerArray.markers[sl_index].pose.orientation.x, markerArray.markers[sl_index].pose.orientation.y, markerArray.markers[sl_index].pose.orientation.z, markerArray.markers[sl_index].pose.orientation.w),
                     rospy.Time.now(),
                     "Marker/base_link",
                     "ur5/world"
                     )            

    def setLabelValue(self, widget=None ,params=None, sl_index=None, type=None, index=None):
        if isinstance(params[0], list):
            widget.setText(type+str(params[sl_index][index]))
        else:
            widget.setText(type+str(params[sl_index]))
    
    def valuechange(self):
        sl_index = self.sl.value()
        self.l1.setText("Frame Index: " + str(sl_index))
        self.slider_response(sl_index=sl_index, forbiden_frames=self._forbiden_frames)
        self.plot(params=self._params, sl_index=sl_index)
#        self.setLabelValue(widget=self.l2, params=self._pos_err_list, sl_index=sl_index, type='Position Err: ')
#        self.setLabelValue(widget=self.l3, params=self._quat_err_list, sl_index=sl_index, type='Orientation Err: ')
#        self.setLabelValue(widget=self.l4, params=self._params, sl_index=sl_index, type='EE Vel: ', index=3)
#        self.setLabelValue(widget=self.l5, params=self._params, sl_index=sl_index, type='JS Vel: ', index=4)
#        self.setLabelValue(widget=self.l6, params=self._robot_config_change, sl_index=sl_index, type='Self Collision: ')

    def plot(self, params=None, sl_index=None):
        ''' plot some random stuff '''
        # random data
        data = params[sl_index][0:2]
        js_state = self._js_table[sl_index][:]

        # create an axis
        ax = self.figure_1.add_subplot(131)
        # update plot for motion checking
        ax.plot([i for i in range(len(self._ee_pos_list))], [pos[0] for pos in self._ee_pos_list], ':', color=colors['tomato'], linewidth=2.0, label='ee_x')
        ax.hold(True)
        ax.plot([i for i in range(len(self._ee_pos_list))], [pos[1] for pos in self._ee_pos_list], ':', color=colors['lightgreen'], linewidth=2.0, label='ee_y')
        ax.plot([i for i in range(len(self._ee_pos_list))], [pos[2] for pos in self._ee_pos_list], ':', color=colors['lightblue'], linewidth=2.0, label='ee_z')
        ax.plot([i for i in range(len(self._pos))], [pos[0] for pos in self._pos], '-', color=colors['darkred'], linewidth=1.0, label='g_x')
        ax.plot([i for i in range(len(self._pos))], [pos[1] for pos in self._pos], '-', color=colors['darkgreen'], linewidth=1.0, label='g_y')
        ax.plot([i for i in range(len(self._pos))], [pos[2] for pos in self._pos], '-', color=colors['darkblue'], linewidth=1.0, label='g_z')
        ax.axvline(sl_index, 0, 1, linewidth=1, color='k', linestyle='-')
        ax.legend(loc='upper center', bbox_to_anchor=(0.5, 1.15), fancybox=True, shadow=True, ncol=6)
#        ax.legend(bbox_to_anchor=(1.05, 1), loc=0, borderaxespad=0.)
        axx = self.figure_1.add_subplot(132)
        axx.plot([i for i in range(len(self._pos_err_list))], self._pos_err_list, '-r', label="Position Err")
        axx.hold(True)
        axx.axvline(sl_index, 0, 1, linewidth=1, color='k', linestyle='-')
        axx.legend(loc='upper center', bbox_to_anchor=(0.5, 1.15), fancybox=True, shadow=True, ncol=1)
        a3x = self.figure_1.add_subplot(133)
        a3x.plot([i for i in range(len(self._ee_velocity))], [val for val in self._ee_velocity], '-r', label="EE Vel")
        a3x.hold(True)
        a3x.axvline(sl_index, 0, 1, linewidth=1, color='k', linestyle='-')
        a3x.legend(loc='upper center', bbox_to_anchor=(0.5, 1.15), fancybox=True, shadow=True, ncol=1)

        bx = self.figure_2.add_subplot(141)
        # note that quaternion are in order of x, y, z, w
        bx.plot([i for i in range(len(self._ee_quat_list))], [quat[0] for quat in self._ee_quat_list], ':', color=colors['tomato'], linewidth=2.0, label='ee_x')
        bx.hold(True)
        bx.plot([i for i in range(len(self._ee_quat_list))], [quat[1] for quat in self._ee_quat_list], ':', color=colors['lightgreen'], linewidth=2.0, label='ee_y')
        bx.plot([i for i in range(len(self._ee_quat_list))], [quat[2] for quat in self._ee_quat_list], ':', color=colors['lightblue'], linewidth=2.0, label='ee_z')
        bx.plot([i for i in range(len(self._ee_quat_list))], [quat[3] for quat in self._ee_quat_list], ':', color=colors['lightyellow'], linewidth=2.0, label='ee_w')
        bx.plot([i for i in range(len(self._orientation))], [pos[0] for pos in self._orientation], '-', color=colors['darkred'], linewidth=1.0, label='g_x')
        bx.plot([i for i in range(len(self._orientation))], [pos[1] for pos in self._orientation], '-', color=colors['darkgreen'], linewidth=1.0, label='g_y')
        bx.plot([i for i in range(len(self._orientation))], [pos[2] for pos in self._orientation], '-', color=colors['darkblue'], linewidth=1.0, label='g_z')
        bx.plot([i for i in range(len(self._orientation))], [pos[3] for pos in self._orientation], '-', color=colors['darkorange'], linewidth=1.0, label='g_w')
        bx.axvline(sl_index, 0, 1, linewidth=1, color='k', linestyle='-')
        bx.legend(loc='upper center', bbox_to_anchor=(0.5, 1.15), fancybox=True, shadow=True, ncol=8)
        bxx = self.figure_2.add_subplot(142)
        bxx.plot([i for i in range(len(self._quat_err_list))], self._quat_err_list, '-b', label='Orientation Err')
        bxx.hold(True)
        bxx.axvline(sl_index, 0, 1, linewidth=1, color='k', linestyle='-')
        bxx.legend(loc='upper center', bbox_to_anchor=(0.7, 1.15), fancybox=True, shadow=True, ncol=1)
        b3x = self.figure_2.add_subplot(143)
        b3x.plot([i for i in range(len(self._js_velocity))], [j for j in self._js_space_velocity], label="JS Vel")
        b3x.hold(True)

        b3x.axvline(sl_index, 0, 1, linewidth=1, color='k', linestyle='-')
        b3x.legend(loc='upper center', bbox_to_anchor=(0.5, 1.15), fancybox=True, shadow=True, ncol=1)
        b4x = self.figure_2.add_subplot(144)
        b4x.bar([i for i in range(6)],self._js_velocity[sl_index], align='center', width=0.5, alpha=0.7)

        # discards the old graph
        ax.hold(False)
        axx.hold(False)
        bx.hold(False)
        bxx.hold(False)
        a3x.hold(False)
        b3x.hold(False)
        b4x.hold(False)
        plt.xticks([i for i in range(6)], objects)

        # refresh canvas
        self.canvas_1.draw()
        self.canvas_2.draw()
    
    # Create actions for button
    @pyqtSlot()
    def on_click_fwd(self):
        print('clicked fwd')
    @pyqtSlot()
    def on_click_bwd(self):
        print('clicked bwd')

def main():
    time_stamp, csv_pos, csv_quat = load_motion_data(global_dir=MOTION_FILE_NAME)
    ee_pos_array, ee_quat_array = load_ee_status(file_dir=ERR_FILE_NAME)

    pos_err, quat_err, fixed_quat=check_motion_mapping_err(obj_pos=csv_pos, obj_quat=csv_quat, ee_pos=ee_pos_array, ee_quat=ee_quat_array)

    params = load_param_data(global_dir=PARAM_FILE_NAME)[1:]
    joint_angle_table = load_joint_states(JS_FILE_NAME)
    # ee vel should be a #frame * 1 vector, js vel should be a #frame * #DOF matrix
    ee_vel, js_vel, js_space_vel = get_ee_js_vel(time_stamp=time_stamp, ee_pos=ee_pos_array, js_table=joint_angle_table)

    config_change_list = get_robot_configuration_change(js_table=joint_angle_table)
    forbiden_frames = check_fail_frames(params_list=params)
    fill_marker_array(csv_pos, csv_quat, forbiden_frames)
    app = QApplication(sys.argv)
    ex = Window(pos=csv_pos, quat=csv_quat, js_table=joint_angle_table, params=params, 
                forbiden_frames=forbiden_frames, robot_config_change_list=config_change_list, 
                ee_pos=ee_pos_array, ee_quat=fixed_quat, pos_err=pos_err, quat_err=quat_err,
                ee_vel=ee_vel, js_vel=js_vel, js_space_vel=js_space_vel)
    ex.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
    '''
    try:
        update_state_and_publish()
    except rospy.ROSInterruptException:
        pass
    '''
