#!/usr/bin/env python
import roslib; roslib.load_manifest('visualization_marker_tutorials')
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import rospy
import math
import csv
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

LBD_PLAY_BACK_BASE_DIR = '/home/hwang/log_lbd_playback/csv_file/'

topic = 'visualization_marker_array'
publisher = rospy.Publisher(topic, MarkerArray, queue_size=10)

rospy.init_node('register')

markerArray = MarkerArray()

count = 0
MARKERS_MAX = 100

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

'''
while not rospy.is_shutdown():
   marker = Marker()
   marker.header.frame_id = "world"
   marker.type = marker.SPHERE
   marker.action = marker.ADD
   marker.scale.x = 0.2
   marker.scale.y = 0.2
   marker.scale.z = 0.2
   marker.color.a = 1.0
   marker.color.r = 1.0
   marker.color.g = 1.0
   marker.color.b = 0.0
   marker.pose.orientation.w = 1.0
   marker.pose.position.x = math.cos(count / 50.0)
   marker.pose.position.y = math.cos(count / 40.0) 
   marker.pose.position.z = math.cos(count / 30.0) 

   # We add the new marker to the MarkerArray, removing the oldest
   # marker from it when necessary
   if(count > MARKERS_MAX):
       markerArray.markers.pop(0)

   markerArray.markers.append(marker)

   # Renumber the marker IDs
   id = 0
   for m in markerArray.markers:
       m.id = id
       id += 1

   # Publish the MarkerArray
   publisher.publish(markerArray)

   count += 1

   rospy.sleep(0.05)
'''
if __name__ == "__main__":
    time_stamp, csv_pos, csv_quat = load_motion_data(global_dir="test_new.csv")
    fill_marker_array(csv_pos, csv_quat)
    publish_marker_array()
#    fig2 = plt.figure()
#    ax = fig2.add_subplot(111, projection='3d')
#    ax.plot(np.array([float(a[0]) for a in csv_pos]), np.array([float(a[1]) for a in csv_pos]), np.array([float(a[2]) for a in csv_pos]), c='r', linewidth=2)
#    plt.show()
