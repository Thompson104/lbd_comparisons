#!/usr/bin/env python
# license removed for brevity
# author: hwang
# created on: 06/14/2017
# modified on: XX/XX/XXXX
import rospy
from sensor_msgs.msg import JointState
import csv
#from playbackVars import *
from Relaxed_IK_Solver.relaxedIK import *
from colors import *
import rospy
#from playbackVars import *
from IK.Broadcaster import *
from playbackUtils import *
from IK.tongsCenter import *
from IK.transformations import *
import math as M
from geometry_msgs.msg import PoseStamped
from Relaxed_IK_Solver.RelaxedIK.weight_function import *
ur5_info = ('/home/hwang/log_lbd_playback/urdf_file/ur5_robot_rotated.urdf', 'shoulder_pan_joint', 'wrist_3_joint', 'ee_fixed_joint')
#ur5_initState =  [M.pi, -0.38, -1.2, -M.pi/2, -M.pi/2, -M.pi/2]
#ur5_initState =  [0, 0, 0, 0, 0, 0]
ur5_initState = [0.048352230340242386, -1.0092552343951624, -1.3978832403766077, -3.8894718329059046, -1.5495994726764124, -0.0146563688861292]
js = JointState()
ik = RelaxedIK(ur5_info, init_state=ur5_initState, rotation_mode='absolute', position_mode='absolute', weight_funcs=(Identity_Weight(), Identity_Weight(), Identity_Weight(), Identity_Weight()))

def rotate_quat(q, rot, axis):
    rot_mat = T.quaternion_matrix(q)
    if axis == 'x':
        a = rot_mat[0:3,0]
    elif axis == 'y':
        a = rot_mat[0:3,1]
    elif axis == 'z':
        a = rot_mat[0:3,2]
    else:
        raise ValueError('Invalid axis in rotate_quat!')

    rot_w = M.cos(rot/2.0)
    rot_axis = M.sin(rot/2.0)*a
    rot_quat = [rot_w, rot_axis[0], rot_axis[1], rot_axis[2]]
    return T.quaternion_multiply(rot_quat, q)

def update_state_and_publish(xopt=None):
    joint_angle_table = []
    pub = rospy.Publisher("ur5/joint_states", JointState, queue_size=10)
    rospy.init_node("test_playback_node", anonymous=True)
    rate = rospy.Rate(100) # 1kHz
    with open("out_js_disp.csv", 'rb') as csvfile:
	spamreader = csv.reader(csvfile)
	for row in spamreader:
            joint_angle_table.append([float(r) for r in row])
    js = JointState()
    js.name = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
    js.position = ur5_initState

    while not rospy.is_shutdown():
        js.header.stamp = rospy.Time.now()
	pub.publish(js)
        rate.sleep()
    '''
    for i in range(len(joint_angle_table)):
        cur_joint_anlge = joint_angle_table[i]
        js.position = cur_joint_anlge
        js.header.stamp = rospy.Time.now()
        print js
        pub.publish(js)
        rate.sleep()
    ''' 
ii = 1
counter = 0
def callback(data):
    global ii, ik, counter
    if ii % 5 != 0:
        ii = ii + 1
        return
    ii = 1
    global js, ur5_info, ur5_initState
    pos = [0.0] * 3
    orientation = [0.0] * 4
    pos[0] = data.pose.position.x
    pos[1] = data.pose.position.y
    pos[2] = data.pose.position.z
    orientation[0] = data.pose.orientation.w
    orientation[1] = data.pose.orientation.x
    orientation[2] = data.pose.orientation.y
    orientation[3] = data.pose.orientation.z
    orientation=rotate_quat(orientation, 1.57075, 'x')
    orientation=rotate_quat(orientation, -1.57075, 'z')
    pub = rospy.Publisher("ur5/joint_states", JointState, queue_size=10)
    js.name = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

    if counter == 0:
        xopt = ik.solve(pos, orientation, vel_objectives_on=True)
        print "here"
    else:
        xopt = ik.solve(pos, orientation,vel_objectives_on=True)
    print xopt
    js.position = xopt
    js.header.stamp = rospy.Time.now()
    pub.publish(js)
    counter = 1
    
def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('real_time_test', anonymous=True)

    rospy.Subscriber("/tongs_center_rev", PoseStamped, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
