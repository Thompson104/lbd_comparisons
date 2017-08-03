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
    rospy.init_node('gripper_center')

    listener = tf.TransformListener()

    gripper_center_pub = rospy.Publisher('gripper/center_position', PoseStamped, queue_size=10)

    rate = rospy.Rate(100.0)
    while not rospy.is_shutdown():
        try:
            (trans_left, rot_left) = listener.lookupTransform('ur5/world', '/gripper/robotiq_85_left_finger_tip_link',
							      rospy.Time())
	    #print trans_left
	    (trans_right, rot_right) = listener.lookupTransform('ur5/world', '/gripper/robotiq_85_right_finger_tip_link',
                                                              rospy.Time())
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
	
	gripper_center = calc_mid_point(trans_left, trans_right)
#	print gripper_center
	cmd = PoseStamped()
        cmd.pose.position.x = gripper_center[0]
	cmd.pose.position.y = gripper_center[1]
	cmd.pose.position.z = gripper_center[2]
	print gripper_center
#	cmd.orientation = rot_left
        gripper_center_pub.publish(cmd)

        rate.sleep()
