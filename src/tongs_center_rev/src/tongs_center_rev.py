#!/usr/bin/env python  
import roslib
import rospy
import math
import tf
from geometry_msgs.msg import PoseStamped

def calc_mid_point(vec1, vec2):
    vec3 = []
    for i_idx in range(len(vec1)):
	vec3.append((vec1[i_idx]+vec2[i_idx])/2.0)
    return vec3

if __name__ == '__main__':
    rospy.init_node('tongs_center_rev')

    listener = tf.TransformListener()

    gripper_center_pub = rospy.Publisher('tongs_center_rev', PoseStamped, queue_size=10)

    rate = rospy.Rate(100.0)
    while not rospy.is_shutdown():
        try:
            (trans_center, rot_center) = listener.lookupTransform('ur5/base_link', 'Tongs_center/base_link',
							      rospy.Time())
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
#	print gripper_center
	cmd = PoseStamped()
        cmd.header.stamp = rospy.Time.now()
        cmd.pose.position.x = trans_center[0]
	cmd.pose.position.y = trans_center[1]
	cmd.pose.position.z = trans_center[2]
        cmd.pose.orientation.x = rot_center[0]
	cmd.pose.orientation.y = rot_center[1]
	cmd.pose.orientation.z = rot_center[2]
	cmd.pose.orientation.w = rot_center[3]
#	cmd.orientation = rot_left
        gripper_center_pub.publish(cmd)

        rate.sleep()
