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
from Relaxed_IK_Solver.RelaxedIK.weight_function import *
ur5_info = ('/home/hwang/log_lbd_playback/urdf_file/ur5_robot_rotated.urdf', 'shoulder_pan_joint', 'wrist_3_joint', 'ee_fixed_joint')
ur5_initState = [0.048352230340242386, -1.0092552343951624, -1.3978832403766077, -3.8894718329059046, -1.5495994726764124, -0.0146563688861292]

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
#    robot_data_reader = UR5_dataReader()
#    while not rospy.is_shutdown():
#	robot_data_reader.update()
#        verify_str = "%.4f, %.4f, %.4f, %.4f, %.4f, %.4f" % (robot_data_reader.q_actual[0],robot_data_reader.q_actual[1], robot_data_reader.q_actual[2],robot_data_reader.q_actual[3],robot_data_reader.q_actual[4],robot_data_reader.q_actual[5])
#        rospy.loginfo(verify_str)
#	rospy.loginfo(robot_data_reader.joint_state.position)
    js = JointState()
    js.name = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
    js.position = xopt

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
if __name__ == '__main__':
    try:
	ik = RelaxedIK(ur5_info, init_state=ur5_initState, rotation_mode='absolute', position_mode='absolute', weight_funcs=(Identity_Weight(), Identity_Weight(), Identity_Weight(), Identity_Weight()))
        xopt = ik.solve([-0.556422600298, -0.048559261641, 0.00706802628991], [0.136671553333, 0.21493295326, 0.706124668612, 0.660691050711], vel_objectives_on=False)
#        q = [1,0,0,0]
#	q = rotate_quat(q, -.707, 'z')
        #q = rotate_quat(q, 1, 'x')
#        xopt = ik.solve([0, 0, 0.5], q, vel_objectives_on=False)
        #xopt = ik.solve([0.4789335324121369, -0.033555808140184246, 0.22914383823689588], [0,0,0,1], vel_objectives_on=False)
        print xopt
        update_state_and_publish(xopt)
    except rospy.ROSInterruptException:
        pass
