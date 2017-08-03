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

from IK.transformations import quaternion_from_matrix
########################################################################################################################
LBD_PLAY_BACK_BASE_DIR = '/home/hwang/log_lbd_playback/csv_file/'
data_offline = True
move_robot = False
########################################################################################################################

ur5_info = ('/home/hwang/log_lbd_playback/urdf_file/ur5_robot_rotated.urdf', 'shoulder_pan_joint', 'wrist_3_joint', 'ee_fixed_joint')
#ur5_initState = [0.24758361279964447, -1.5356391111956995, -2.024224583302633, -2.668980900441305, -1.1982539335833948, -0.12823516527284795]
#ur5_initState = [0.048352230340242386, -1.0092552343951624, -1.3978832403766077, -3.8894718329059046, -1.5495994726764124, -0.0146563688861292]
#ur5_initState =  [M.pi, -0.38, -1.2, -M.pi/2, -M.pi/2, -M.pi/2]
ur5_initState = [0.00, -2.7, -1, 0.00, 1.50, 0.00]

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

def load_motion_data(global_dir=None, data_type=None):
    '''type indicate data type of csv files. kin stands for pos and orientation
       gripper stands for encoder value of the tongs'''
    if data_type == "kin":
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
    elif data_type == "gripper":
        csv_encoder = []
        csv_times_gp = []
        with open(LBD_PLAY_BACK_BASE_DIR+global_dir, 'rb') as csvfile:
            spamreader = csv.reader(csvfile)
            for row in spamreader:
                csv_times_gp.append(float(row[0]))
                csv_encoder.append(float(row[1]))
        return np.array(csv_times_gp), csv_encoder

if __name__ == '__main__':
    rospy.init_node('playback_mocap')
    test_pub = rospy.Publisher("ur5/joint_states", JointState, queue_size=10)
    print bcolors.OKGREEN + 'playback mocap' + bcolors.ENDC
    ik = RelaxedIK(ur5_info, init_state=ur5_initState, 
                    rotation_mode='absolute', 
                    position_mode='absolute', 
                    weight_funcs=(Identity_Weight(), Identity_Weight(), Identity_Weight(), Identity_Weight()))
    #                weight_funcs=(Identity_Weight(), Hand_Vel_Weight(), Identity_Weight(), Identity_Weight()))
    vars = PlaybackVars('solutionFile.txt', dataOffline=data_offline, move_robot=move_robot)
    # load human demonstration
#    times_np, csv_pos, csv_quat = load_motion_data(global_dir="test_traj_1_rev.csv", data_type='kin')
#    times_np_gripper, csv_encoder = load_motion_data(global_dir="test_traj_1_gripper.csv", data_type='gripper')
    times_np, csv_pos, csv_quat = load_motion_data(global_dir="crappy_motion_0_rev.csv", data_type='kin')
    times_np_gripper, csv_encoder = load_motion_data(global_dir="crappy_motion_0_gripper.csv", data_type='gripper')
    utils = PlaybackUtils(vars)
    vars.Utils = utils
    vars.TongsTransform = GetTongsTransform()

    idx = utils.find_closest(times_np, 0.0)
    encoder = csv_encoder[idx]

    q = csv_quat[idx]
    # q = rotate_quat(q, -.707, 'z')
    # q = rotate_quat(q, .707, 'x')
    #==============================================================================
    '''manually hack the coordinate frame here'''
    q=rotate_quat(q, 1.57075, 'x')
    q=rotate_quat(q, -1.57075, 'z')
    with open(PARAM_RT_DIR_BASE+'crappy_motion_0_rev_mod.csv', 'wb') as csv_file:
        pass
    with open('err_crappy_motion_0_rev_mod.csv', 'wb') as csv_file:
        pass
    #==============================================================================
    xopt = ik.solve(csv_pos[idx], q, vel_objectives_on=False)
#    ik.log_parameters()
    vars.xopt = xopt
    xopt = N.array(xopt)
    vars.xoptPub = xopt
    vars.initialIteration = False
    # safely place robot in initial configuration
    print 'Resetting Robot'

    pubVREP(vars)
    if vars.move_robot:
        moveJ_ur5(vars)
        rospy.sleep(12)

    tDiv = 1000000000.0
    endTime = times_np[-1]
    startTime = float(rospy.Time.now().to_nsec())
    currTime = 0.0
    if not move_robot:
        # this will only solve RIK for all joint states and run on simulator
#        with open("js_test_traj_1_rev.csv", "wb") as out_file:
        with open("js_crappy_motion_0_rev_mod.csv", "wb") as out_file:
            csv_writer = csv.writer(out_file, dialect='excel')
            for idx in range(len(csv_pos)):
                csv_pos[idx][0] = csv_pos[idx][0]
#                csv_pos[idx][1] = csv_pos[idx][0]+0.06
                q = csv_quat[idx]
                q=rotate_quat(q, 1.57075, 'x')
                q=rotate_quat(q, -1.57075, 'z')
                xopt = ik.solve(csv_pos[idx], q, vel_objectives_on=False, frame_idx=idx)
                csv_writer.writerow(xopt)
        print("Done!")
        exit()
    else:
        while not currTime >= endTime and not rospy.is_shutdown():
            encoder = []
            currTime = float(rospy.Time.now().to_nsec() - startTime) / tDiv
            idx = utils.find_closest(times_np, currTime)
            idx_gripper = utils.find_closest(times_np_gripper, currTime)
            encoder = csv_encoder[idx_gripper]
            q = csv_quat[idx]
            #======================================================================
            q=rotate_quat(q, 1.57075, 'x')
            q=rotate_quat(q, -1.57075, 'z')
            #======================================================================
            csv_pos[idx][0] = csv_pos[idx][0]
            xopt = ik.solve(csv_pos[idx], q, vel_objectives_on=False)

            vars.xopt = xopt
            vars.xopt
            vars.xoptPub = xopt
            vars.encoderValue = encoder
            rospy.sleep(0.004)
            publish(vars)
            currTime = float(rospy.Time.now().to_nsec() - startTime) / tDiv