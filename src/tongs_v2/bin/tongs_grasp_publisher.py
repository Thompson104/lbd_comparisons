#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32
import numpy as np
import tf.transformations as tf
pose1 = PoseStamped()
pose2 = PoseStamped()
from copy import deepcopy
import tf as tf_rev

# distances of markers from base of mocap mount
a1 = 0.05
a2 = 0.05
broad_caster = tf_rev.TransformBroadcaster()

def get_center_of_tongs(p1,p2):
    p1 = deepcopy(p1)
    p2 = deepcopy(p2)
    tongs_center = PoseStamped()
    tongs_center.pose.position.x = (p1.pose.position.x + p2.pose.position.x) / 2
    tongs_center.pose.position.y = (p1.pose.position.y + p2.pose.position.y) / 2
    tongs_center.pose.position.z = (p1.pose.position.z + p2.pose.position.z) / 2


    # quaternion averaging:
    # ref: https://ntrs.nasa.gov/archive/nasa/casi.ntrs.nasa.gov/20070017872.pdf
    w1 = p1.pose.orientation.w
    x1 = p1.pose.orientation.x
    y1 = p1.pose.orientation.y
    z1 = p1.pose.orientation.z

    w2 = p2.pose.orientation.w
    x2 = p2.pose.orientation.x
    y2 = p2.pose.orientation.y
    z2 = p2.pose.orientation.z

    q1 = np.array([x1,y1,z1,w1])
    q2 = np.array([x2, y2, z2, w2])
    q1_rot = tf.quaternion_multiply(q1,tf.quaternion_about_axis(np.pi,(1,0,0)))


    q_avg = tf.quaternion_slerp(q1_rot,q2,0.5)


    tongs_center.pose.orientation.x = q_avg[0]
    tongs_center.pose.orientation.y = q_avg[1]
    tongs_center.pose.orientation.z = q_avg[2]
    tongs_center.pose.orientation.w = q_avg[3]

    tongs_center.pose.orientation.x = q1_rot[0]
    tongs_center.pose.orientation.y = q1_rot[1]
    tongs_center.pose.orientation.z = q1_rot[2]
    tongs_center.pose.orientation.w = q1_rot[3]

    ## Distance correction:

    trans1 = tf.translation_matrix((0,0,-a1))
    trans2 = tf.translation_matrix((0,0,-a2))

    rot1 = tf.quaternion_matrix(q1)
    rot2 = tf.quaternion_matrix(q2)

    pos1_trans = tf.translation_matrix((p1.pose.position.x,p1.pose.position.y,p1.pose.position.z))
    pos2_trans = tf.translation_matrix((p2.pose.position.x,p2.pose.position.y,p2.pose.position.z))

    tongs_side_1 = tf.concatenate_matrices(pos1_trans,rot1,trans1)
    tongs_side_2 = tf.concatenate_matrices(pos2_trans,rot2,trans2)
    # tongs_side_1 = tf.concatenate_matrices(trans1,trans1)
    # tongs_side_2 = trans2



    tongs_side_1_trans = tf.translation_from_matrix(tongs_side_1)
    tongs_side_1_quart = tf.quaternion_from_matrix(tongs_side_1)

    tongs_side_2_trans = tf.translation_from_matrix(tongs_side_2)
    tongs_side_2_quart = tf.quaternion_from_matrix(tongs_side_2)

    pose_tongs_side_1 = p1
    pose_tongs_side_2 = p2

    pose_tongs_side_1.pose.position.x = tongs_side_1_trans[0]
    pose_tongs_side_1.pose.position.y = tongs_side_1_trans[1]
    pose_tongs_side_1.pose.position.z = tongs_side_1_trans[2]

    pose_tongs_side_1.pose.orientation.x = tongs_side_1_quart[0]
    pose_tongs_side_1.pose.orientation.y = tongs_side_1_quart[1]
    pose_tongs_side_1.pose.orientation.z = tongs_side_1_quart[2]
    pose_tongs_side_1.pose.orientation.w = tongs_side_1_quart[3]


    pose_tongs_side_2.pose.position.x = tongs_side_2_trans[0]
    pose_tongs_side_2.pose.position.y = tongs_side_2_trans[1]
    pose_tongs_side_2.pose.position.z = tongs_side_2_trans[2]

    pose_tongs_side_2.pose.orientation.x = tongs_side_2_quart[0]
    pose_tongs_side_2.pose.orientation.y = tongs_side_2_quart[1]
    pose_tongs_side_2.pose.orientation.z = tongs_side_2_quart[2]
    pose_tongs_side_2.pose.orientation.w = tongs_side_2_quart[3]

    wid = ((p1.pose.position.x - p2.pose.position.x) ** 2 +
           (p1.pose.position.y - p2.pose.position.y) ** 2 +
           (p1.pose.position.z - p2.pose.position.z) ** 2) ** 0.5

    return tongs_center,wid,pose_tongs_side_1,pose_tongs_side_2



def callback_pose1(data):
    global pose1
    pose1 = data

def callback_pose2(data):
    global pose1
    global pose2
    global broad_caster
    pose2 = data
    tongs_center, wid,tongs_side_1,tongs_side_2 = get_center_of_tongs(pose1, pose2)
    tongs_center.header.frame_id = "world"
    tongs_center.header.stamp = pose2.header.stamp
    tongs_center_pub.publish(tongs_center)
    tongs_width_pub.publish(wid)
    tongs_side1_pub.publish(tongs_side_1)
    tongs_side2_pub.publish(tongs_side_2)
    # tongs_side1_pub.publish(pose1)
    # tongs_side2_pub.publish(pose2)
    broad_caster.sendTransform((tongs_center.pose.position.x, tongs_center.pose.position.y, tongs_center.pose.position.z),
                     (tongs_center.pose.orientation.x, tongs_center.pose.orientation.y, tongs_center.pose.orientation.z, tongs_center.pose.orientation.w),
                     rospy.Time.now(),
                     "Tongs_center/base_link",
                     "world"
                     )



def tongs_publisher():
    rospy.init_node('mocap_publisher', anonymous=True)
    global tongs_center_pub
    global tongs_width_pub
    global tongs_side1_pub
    global tongs_side2_pub

    tongs_center_pub = rospy.Publisher('/Tongs_center', PoseStamped, queue_size=10)
    tongs_width_pub = rospy.Publisher('/Tongs_width', Float32, queue_size=10)

    tongs_side1_pub = rospy.Publisher('/Tongs_side_1', PoseStamped, queue_size=10)
    tongs_side2_pub = rospy.Publisher('/Tongs_side_2', PoseStamped, queue_size=10)

    rospy.Subscriber("/Robot_1/pose", PoseStamped, callback_pose1)
    rospy.Subscriber("/Robot_2/pose", PoseStamped, callback_pose2)

    rospy.spin()




if __name__ == "__main__":
    tongs_publisher()
