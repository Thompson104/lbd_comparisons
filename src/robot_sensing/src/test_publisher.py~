#!/usr/bin/env python
# license removed for brevity
# author: hwang
# created on: 06/14/2017
# modified on: XX/XX/XXXX
import rospy
from sensor_msgs.msg import JointState
import csv
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import numpy as np
import time

SOLUTION_FILE = 'out_js_disp_test_3.csv'

if SOLUTION_FILE == 'out_js_disp.csv':
    predefined_elbow_flip_indices = range(2226, 4368)
else:
    predefined_elbow_flip_indices = []
predefined_self_collision = []

LBD_PLAY_BACK_BASE_DIR = '/home/hwang/log_lbd_playback/csv_file/'

topic = 'visualization_marker_array'
publisher = rospy.Publisher(topic, MarkerArray, queue_size=10)

#rospy.init_node('register')

markerArray = MarkerArray()

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
        marker.color.a = 1.0
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

def publish_marker_array():
    global publisher
    while not rospy.is_shutdown():
        publisher.publish(markerArray)
        rospy.sleep(0.5)

def update_state_and_publish():
    global markerArray, publisher
    _, csv_pos, csv_quat = load_motion_data(global_dir="test_new.csv")
    fill_marker_array(csv_pos, csv_quat)
    joint_angle_table = []
    pub = rospy.Publisher("ur5/joint_states", JointState, queue_size=10)
#    pub = rospy.Publisher("joint_states", JointState, queue_size=10)
    rospy.init_node("test_playback_node", anonymous=True)
    with open(SOLUTION_FILE, 'rb') as csvfile:
	spamreader = csv.reader(csvfile)
	for row in spamreader:
            joint_angle_table.append([float(r) for r in row])
    js = JointState()
    js.name = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
    js.position = [ 3.47172217e+00,  -2.14456987e+00,  -1.91532757e+00,   9.18540937e-01,
  -3.30283928e-01,  -3.21397317e-04]
    i = 1000
#    for i in range(len(joint_angle_table)):
    while i < len(joint_angle_table):
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
        markerArray.markers[i].color.a = 1.0
        markerArray.markers[i].color.r = 1.0
        markerArray.markers[i].color.g = 1.0
        markerArray.markers[i].color.b = 0.0
	i += 1
        time.sleep(0.01)
if __name__ == '__main__':
    try:
        update_state_and_publish()
  	if len(predefined_elbow_flip_indices) > 0:
	    print "Detect Elbow Flip:"
	    print "Elbow Flip happens at frames:"
	    print predefined_elbow_flip_indices
        else:
	    print "No Elbow Flip Detected!"
	if len(predefined_self_collision) > 0:
	    print "Detect Self Collisions:"
	    print "Self Collisions happens at frames:"
	    print predefined_self_collision
        else:
	    print "No Self Collisions Detected!"
    except rospy.ROSInterruptException:
        pass
