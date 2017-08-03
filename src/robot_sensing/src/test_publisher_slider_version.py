#!/usr/bin/env python
# license removed for brevity
# author: hwang
# created on: 06/14/2017
# modified on: XX/XX/XXXX
import rospy
import numpy as np
import sys
import csv
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from PyQt4.QtCore import *
from PyQt4.QtGui import *

LBD_PLAY_BACK_BASE_DIR = '/home/hwang/log_lbd_playback/csv_file/'
topic = 'visualization_marker_array'
publisher = rospy.Publisher(topic, MarkerArray, queue_size=10)
pub = rospy.Publisher("ur5/joint_states", JointState, queue_size=10)
#pub = rospy.Publisher("joint_states", JointState, queue_size=10)

rospy.init_node("test_playback_node", anonymous=True)

# define markerArray type to show trajecotry
markerArray = MarkerArray()

# define jointState type to publish real robot states
js = JointState()
js.name = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
js.position = [ 3.47172217e+00,  -2.14456987e+00,  -1.91532757e+00,   9.18540937e-01,
  -3.30283928e-01,  -3.21397317e-04]

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

def load_joint_states(file_dir="out_js_disp_test_4.csv"):
    joint_angle_table = []
    with open(file_dir, 'rb') as csvfile:
	spamreader = csv.reader(csvfile)
	for row in spamreader:
            joint_angle_table.append([float(r) for r in row])
    return joint_angle_table

def fill_marker_array(pos, quat):
    global markerArray, publisher
    for idx in range(len(pos)):
        marker = Marker()
        marker.header.frame_id = "ur5/world"
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.scale.x = 0.01
        marker.scale.y = 0.01
        marker.scale.z = 0.01
        marker.color.a = 0.2
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.pose.orientation.w = quat[idx][0]
        marker.pose.orientation.x = quat[idx][1]
	marker.pose.orientation.y = quat[idx][2]
	marker.pose.orientation.z = quat[idx][3]
        marker.pose.position.x = pos[idx][0]
        marker.pose.position.y = pos[idx][1] 
        marker.pose.position.z = pos[idx][2]
        marker.id = idx
        markerArray.markers.append(marker)

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
        markerArray.markers[i].color.r = 1.0
        markerArray.markers[i].color.g = 0.0
        markerArray.markers[i].color.b = 0.0
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

class sliderdemo(QWidget):
    def __init__(self, parent = None, pos = None, quat = None, js_table = None):
        '''
        pos, quat stands for the position and orientation of tongs in a certain demonstration
        js_table stands for the joint states solved by RIK
        '''
        super(sliderdemo, self).__init__(parent)
        self._pos = pos
        self._orientation = quat
        self._js_table = js_table
        layout = QVBoxLayout()
        self.l1 = QLabel("Hello!")
        self.l1.setAlignment(Qt.AlignCenter)
        self.l1.setFont(QFont("Arial", 30))
        layout.addWidget(self.l1)
		
        self.sl = QSlider(Qt.Horizontal)
        self.sl.setMinimum(0)
        self.sl.setMaximum(len(self._pos)-1)
        self.sl.setValue(500)
        self.sl.setTickPosition(QSlider.TicksBelow)
        self.sl.setTickInterval(1)

      	# add QPushButton to implement single step
        self.btn_fwd = QPushButton('>>')
        layout.addWidget(self.btn_fwd)
        self.btn_fwd.resize(self.btn_fwd.sizeHint())
        self.btn_bwd = QPushButton('<<')
        layout.addWidget(self.btn_bwd)
        self.btn_bwd.resize(self.btn_bwd.sizeHint())
		
        layout.addWidget(self.sl)
        self.sl.valueChanged.connect(self.valuechange)
	self.btn_fwd.clicked.connect(self.on_click_fwd)
        self.btn_bwd.clicked.connect(self.on_click_bwd)
        self.setLayout(layout)
        self.setWindowTitle("Play Simulator with Slider")

    def slider_response(self, sl_index):
        global markerArray, publisher, pub, js
	markerArray.markers[sl_index].scale.x = 0.02
        markerArray.markers[sl_index].scale.y = 0.02
        markerArray.markers[sl_index].scale.z = 0.02
        markerArray.markers[sl_index].color.a = 1.0
        markerArray.markers[sl_index].color.r = 1.0
        markerArray.markers[sl_index].color.g = 0.0
        markerArray.markers[sl_index].color.b = 0.0
        js.position = self._js_table[sl_index]
        js.header.stamp = rospy.Time.now()
        pub.publish(js)
        publisher.publish(markerArray)
	# change last marker to normal
	markerArray.markers[sl_index].scale.x = 0.01
        markerArray.markers[sl_index].scale.y = 0.01
        markerArray.markers[sl_index].scale.z = 0.01
        markerArray.markers[sl_index].color.a = 1.0
        markerArray.markers[sl_index].color.r = 1.0
        markerArray.markers[sl_index].color.g = 1.0
        markerArray.markers[sl_index].color.b = 0.0

    def valuechange(self):
        sl_index = self.sl.value()
        self.l1.setText(str(sl_index))
	self.slider_response(sl_index=sl_index)
    
    # Create actions for button
    @pyqtSlot()
    def on_click_fwd(self):
        print('clicked fwd')
    @pyqtSlot()
    def on_click_bwd(self):
        print('clicked bwd')

def main():
    _, csv_pos, csv_quat = load_motion_data(global_dir="test_new.csv")
    joint_angle_table = load_joint_states()
    fill_marker_array(csv_pos, csv_quat)
    app = QApplication(sys.argv)
    ex = sliderdemo(pos=csv_pos, quat=csv_quat, js_table=joint_angle_table)
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
