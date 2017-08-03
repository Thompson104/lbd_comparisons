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
    rospy.init_node('tongs_center')

    listener = tf.TransformListener()

    gripper_center_pub = rospy.Publisher('Tongs_trans/center_position', PoseStamped, queue_size=10)

    rate = rospy.Rate(100.0)
    while not rospy.is_shutdown():
        try:
            (trans, rot) = listener.lookupTransform('ur5/world', 'Tongs_center/base_link',
							      rospy.Time())
#            print(rot)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
#	print gripper_center
	cmd = PoseStamped()
        cmd.pose.position.x = trans[0]
	cmd.pose.position.y = trans[1]
	cmd.pose.position.z = trans[2]
	cmd.pose.orientation.x = rot[0]
        cmd.pose.orientation.y = rot[1]
        cmd.pose.orientation.z = rot[2]
        cmd.pose.orientation.w = rot[3]
#	cmd.orientation = rot_left
        gripper_center_pub.publish(cmd)

        rate.sleep()
