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

import tf as tf_rev
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

from PyQt4 import QtGui
from PyQt4.QtCore import *
from PyQt4.QtGui import *

from matplotlib.backends.backend_qt4agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt4agg import NavigationToolbar2QT as NavigationToolbar
import matplotlib.pyplot as plt

# motion switches
IS_FILTERED = False
IS_REVISED = True
IS_LAST_MOTION = True

# global vars
LBD_PLAY_BACK_BASE_DIR = '/home/hwang/log_lbd_playback/csv_file/'
PARAM_RT_DIR_BASE = '/home/hwang/log_lbd_playback/param_log_file/'
PREDEFINED_POS_OBJ_THRESHOLD = 0.001
topic = 'visualization_marker_array'
objects = ('Pos Obj', 'Orien Obj')

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
else:
    if IS_FILTERED:
        JS_FILE_NAME = "js_test_traj_1_fil.csv"
        MOTION_FILE_NAME = "test_traj_1_fil.csv"
        PARAM_FILE_NAME = "test_traj_1_fil.csv"
        '''
        JS_FILE_NAME = "js_test_traj_0_fil.csv"
        MOTION_FILE_NAME = "test_traj_0_fil.csv"
        PARAM_FILE_NAME = "test_traj_0_fil.csv"
        '''
    elif IS_REVISED:
        JS_FILE_NAME = "js_test_traj_1_rev.csv"
        MOTION_FILE_NAME = "test_traj_1_rev.csv"
        PARAM_FILE_NAME = "test_traj_1_rev.csv"
    else:  
        JS_FILE_NAME = "js_test_traj_1.csv"
        MOTION_FILE_NAME = "test_traj_1.csv"
        PARAM_FILE_NAME = "test_traj_1.csv"
        '''
        JS_FILE_NAME = "js_test_traj_0.csv"
        MOTION_FILE_NAME = "test_traj_0_raw.csv"
        PARAM_FILE_NAME = "test_traj_0.csv"
        '''

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
            csv_pos.append([float(item) for item in tmp_pos_list])
            csv_quat.append([float(item) for item in tmp_quat_list])
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

def get_robot_configuration_change(js_table=None):
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

# A fake function for this UI
def update_state_and_publish():
    global markerArray, publisher
    fill_marker_array(csv_pos, csv_quat)
    i = 0
    while i <= len(joint_angle_table):
	# trace the current gripper position
	markerArray.markers[i].scale.x = 0.02
        markerArray.markers[i].scale.y = 0.02
        markerArray.markers[i].scale.z = 0.02
        markerArray.markers[i].color.a = 1.0
        markerArray.markers[i].color.r = 0.0
        markerArray.markers[i].color.g = 0.0
        markerArray.markers[i].color.b = 1.0
	markerArray.markers[i].type = marker.CUBE
        cur_joint_anlge = joint_angle_table[i]
        js.position = cur_joint_anlge
        js.header.stamp = rospy.Time.now()
	print js
        pub.publish(js)
        publisher.publish(markerArray)
	# change last marker to normal
	markerArray.markers[i].scale.x = 0.01
        markerArray.markers[i].scale.y = 0.01
        markerArray.markers[i].scale.z = 0.01
        markerArray.markers[i].color.a = 0.2
        markerArray.markers[i].color.r = 1.0
        markerArray.markers[i].color.g = 1.0
        markerArray.markers[i].color.b = 0.0
	i += 1

class Window(QWidget):
    def __init__(self, parent = None, pos = None, quat = None, js_table = None, params=None, forbiden_frames=None, 
		robot_config_change_list=None):
        '''
        pos, quat stands for the position and orientation of tongs in a certain demonstration
        js_table stands for the joint states solved by RIK
        '''
        super(Window, self).__init__(parent)
        # a figure instance to plot on
        self.figure = plt.figure()
	# get init of data tables
        self._pos = pos
        self._orientation = quat
        self._js_table = js_table
	self._params = params
	# use for checking performance
	self._forbiden_frames = forbiden_frames
	self._robot_config_change = robot_config_change_list
	
        layout = QVBoxLayout()

	# Add label to show some texts
        self.l1 = QLabel("Hello!")
        self.l1.setAlignment(Qt.AlignCenter)
        self.l1.setFont(QFont("Arial", 30))
        layout.addWidget(self.l1)
		
	# Add other labels to show other parameter values
	# 'Orien Weight', 'EE Vel', 'JS Vel', 'Obj Val'	
	self.l2 = QLabel("Orientation Weights: 0.0")
	self.l2.setAlignment(Qt.AlignLeft)
        self.l2.setFont(QFont("Arial", 15))

	self.l3 = QLabel("EE Vel: 0.0")
	self.l3.setAlignment(Qt.AlignLeft)
        self.l3.setFont(QFont("Arial", 15))

	self.l4 = QLabel("JS Vel: 0.0")
	self.l4.setAlignment(Qt.AlignLeft)
        self.l4.setFont(QFont("Arial", 15))

	self.l5 = QLabel("Obj Val: 0.0")
	self.l5.setAlignment(Qt.AlignLeft)
        self.l5.setFont(QFont("Arial", 15))

	self.l6 = QLabel("Robot Config Change: 0.0")
	self.l6.setAlignment(Qt.AlignLeft)
        self.l6.setFont(QFont("Arial", 15))

	# this is the Canvas Widget that displays the `figure`
        # it takes the `figure` instance as a parameter to __init__
        self.canvas = FigureCanvas(self.figure)

        # this is the Navigation widget
        # it takes the Canvas widget and a parent
        self.toolbar = NavigationToolbar(self.canvas, self)
		
        self.sl = QSlider(Qt.Horizontal)
        self.sl.setMinimum(0)
        self.sl.setMaximum(len(self._pos)-1)
        self.sl.setValue(500)
        self.sl.setTickPosition(QSlider.TicksBelow)
        self.sl.setTickInterval(1)

      	# add QPushButton to implement single step
	'''
        self.btn_fwd = QPushButton('>>')
        layout.addWidget(self.btn_fwd)
        self.btn_fwd.resize(self.btn_fwd.sizeHint())
        self.btn_bwd = QPushButton('<<')
        layout.addWidget(self.btn_bwd)
        self.btn_bwd.resize(self.btn_bwd.sizeHint())
	'''

	layout.addWidget(self.toolbar)
        layout.addWidget(self.canvas)	
        layout.addWidget(self.sl)
	layout.addWidget(self.l2)
        layout.addWidget(self.l3)
        layout.addWidget(self.l4)
        layout.addWidget(self.l5)
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
        self.l1.setText(str(sl_index))
	self.slider_response(sl_index=sl_index, forbiden_frames=self._forbiden_frames)
	self.plot(params=self._params, sl_index=sl_index)
	self.setLabelValue(widget=self.l2, params=self._params, sl_index=sl_index, type='Orientation Weights: ', index=2)
	self.setLabelValue(widget=self.l3, params=self._params, sl_index=sl_index, type='EE Vel: ', index=3)
	self.setLabelValue(widget=self.l4, params=self._params, sl_index=sl_index, type='JS Vel: ', index=4)
	self.setLabelValue(widget=self.l5, params=self._params, sl_index=sl_index, type='Obj Val: ', index=5)
        self.setLabelValue(widget=self.l6, params=self._robot_config_change, sl_index=sl_index, type='Robot Config Change: ')

    def plot(self, params=None, sl_index=None):
        ''' plot some random stuff '''
        # random data
        data = params[sl_index][0:2]

        # create an axis
        ax = self.figure.add_subplot(111)

        # discards the old graph
        ax.hold(False)

        # plot data, bar plot
        #ax.plot(data, '*-')
	ax.bar([i for i in range(2)], data, align='center', alpha=0.7)

	# set up options for axis
	plt.xticks([i for i in range(2)], objects)
	
        # refresh canvas
        self.canvas.draw()
    
    # Create actions for button
    @pyqtSlot()
    def on_click_fwd(self):
        print('clicked fwd')
    @pyqtSlot()
    def on_click_bwd(self):
        print('clicked bwd')

def main():
    _, csv_pos, csv_quat = load_motion_data(global_dir=MOTION_FILE_NAME)
    params = load_param_data(global_dir=PARAM_FILE_NAME)[1:]
    joint_angle_table = load_joint_states(JS_FILE_NAME)
    config_change_list = get_robot_configuration_change(js_table=joint_angle_table)
    forbiden_frames = check_fail_frames(params_list=params)
    fill_marker_array(csv_pos, csv_quat, forbiden_frames)
    app = QApplication(sys.argv)
    ex = Window(pos=csv_pos, quat=csv_quat, js_table=joint_angle_table, params=params, forbiden_frames=forbiden_frames, robot_config_change_list=config_change_list)
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
