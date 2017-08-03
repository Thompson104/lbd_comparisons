#!/usr/bin/env python
__author__ = 'drakita'

from Relaxed_IK_Solver.relaxedIK import *
from colors import *
import rospy
from playbackVars import *
from IK.Broadcaster import *
from playbackUtils import *
from IK.tongsCenter import *
from IK.transformations import *
import IK.transformations as trans
import math as M
from Relaxed_IK_Solver.RelaxedIK.weight_function import *
import csv
from sensor_msgs.msg import JointState
from datetime import datetime
########################################################################################################################

playback_file_path = '/home/hwang/log_lbd_playback/csv_file/test_new.csv'
data_offline = False
move_robot = True

########################################################################################################################

ur5_info = ('/home/hwang/log_lbd_playback/urdf_file/ur5_robot_rotated.urdf', 'shoulder_pan_joint', 'wrist_3_joint', 'ee_fixed_joint')
#ur5_initState = [0.24758361279964447, -1.5356391111956995, -2.024224583302633, -2.668980900441305, -1.1982539335833948, -0.12823516527284795]
ur5_initState = [0.048352230340242386, -1.0092552343951624, -1.3978832403766077, -3.8894718329059046, -1.5495994726764124, -0.0146563688861292]
q_base = [-0.0261065628807, -0.0350005473483, 0.688807537904, 0.723628070757]
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



if __name__ == '__main__':
    rospy.init_node('playback_mocap')
    test_pub = rospy.Publisher("ur5/joint_states", JointState, queue_size=10)
    print bcolors.OKGREEN + 'playback mocap' + bcolors.ENDC
    ik = RelaxedIK(ur5_info, init_state=ur5_initState, rotation_mode='absolute', position_mode='absolute', weight_funcs=(Identity_Weight(), Identity_Weight(), Identity_Weight(), Identity_Weight()))

#    vars = PlaybackVars('/home/drakita/VREP/ik_state.txt', dataOffline=data_offline, move_robot=move_robot)
    vars = PlaybackVars('solutionFile.txt', dataOffline=data_offline, move_robot=move_robot)

    csv_times = []
    csv_pos = []
    csv_quat = []
    csv_encoder = []
    fp = playback_file_path
    f = open(fp, 'r')
    line = f.readline()
    line = line.replace('[', '')
    line = line.replace(']', '')
    line = line.replace('"', '')
    while not line == '':
        lineArr = line.split(',')
        csv_times.append(float(lineArr[0]))
        csv_pos.append([float(lineArr[1]), float(lineArr[2]), float(lineArr[3])])
        csv_quat.append([float(lineArr[4]), float(lineArr[5]), float(lineArr[6]), float(lineArr[7])])
        csv_encoder.append(float(lineArr[8]))
        line = f.readline()
        line = line.replace('[', '')
        line = line.replace(']', '')
        line = line.replace('"', '')

    times_np = np.array(csv_times)

    utils = PlaybackUtils(vars)
    vars.Utils = utils
    vars.TongsTransform = GetTongsTransform()

    idx = utils.find_closest(times_np, 0.0)
    encoder = csv_encoder[idx]

    q = csv_quat[idx]
    # q = [1,0,0,0]
    #q = rotate_quat(q, -.707, 'z')
    # q = rotate_quat(q, .707, 'x')
    #==============================================================================

    q=rotate_quat(q, 1.57075, 'x')
    q=rotate_quat(q, -1.57075, 'z')

    #==============================================================================
#    q = [1, 0, 0, 0]
#    q=trans.quaternion_dispQ(q_base, q)
    xopt = ik.solve(csv_pos[idx], q, vel_objectives_on=False)
    vars.xopt = xopt
    xopt = N.array(xopt)
    vars.xoptPub = xopt
#    vars.initialIteration = False
    #safely place robot in initial configuration
#    print 'Resetting Robot'

    pubVREP(vars)
    if vars.move_robot:
        moveJ_ur5(vars)
    rospy.sleep(20)

    tDiv = 1000000000.0
    endTime = csv_times[-1]
    startTime = float(rospy.Time.now().to_nsec())
    currTime = 0.0
    idx = 1
    while not idx >= len(csv_times) and not rospy.is_shutdown():
        q = csv_quat[idx]
        q=rotate_quat(q, 1.57075, 'x')
        q=rotate_quat(q, -1.57075, 'z')
        xopt = ik.solve(csv_pos[idx], q, vel_objectives_on=False)
        vars.xopt = xopt
        xopt = N.array(xopt)
        vars.xoptPub = xopt
        if vars.move_robot:
            moveJ_ur5(vars)
        print(idx)
        rospy.sleep(0.25)
        idx += 1
        '''
        encoder = []
        currTime = float(rospy.Time.now().to_nsec() - startTime) / tDiv
        idx = utils.find_closest(times_np, currTime)
        encoder = csv_encoder[idx]
        q = csv_quat[idx]
        #======================================================================
        q=rotate_quat(q, 1.57075, 'x')
        q=rotate_quat(q, -1.57075, 'z')
        #======================================================================
        #q = rotate_quat(q, -.707, 'z')
        # q = rotate_quat(q, .707, 'x')
        # q = rotate_quat(q, -M.pi/2, 'z')
#        q = [1, 0, 0, 0]
#        q=trans.quaternion_dispQ(q_base, q)
        xopt = ik.solve(csv_pos[idx], q, vel_objectives_on=False)
        print xopt
        # print csv_pos[idx]
        vars.xopt = xopt
        vars.xopt
        vars.xoptPub = xopt
        # print currTime
        vars.encoderValue = encoder
        rospy.sleep(0.005)
        publish(vars)
        currTime = float(rospy.Time.now().to_nsec() - startTime) / tDiv
        '''