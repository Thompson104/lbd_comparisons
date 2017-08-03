__author__ = 'drakita'
import numpy as N
import math
from playbackUtils import *
import csv


def pubVREP(vars):
    # solution file
    xoptS = ",".join(str(e) for e in vars.xoptPub)
    f = open(vars.solutionFile, 'w')
    f.write(xoptS)
    f.close()


def pubGripperROS(vars):
    utils = vars.Utils
    posValue = utils.getGripperValue()
    vars.gripper_cmd.position = posValue
    vars.gripper_cmd.speed = 0.5
    vars.gripper_cmd.force = 100.0
    vars.gripper_pub.publish(vars.gripper_cmd)


def pubROS(vars):
    # TODO: complete this
    rotDisp = N.array(vars.rosDisp)
    a = []

def move_ur5(vars):
    '''
    author: drakita
    moves the physical ur5 robot using urscript
    this uses the command servoj to be used for real-time streaming commands, no interpolation is used
    :param q: configuration of robot in radians [q1, q2, q3 ,..., q6]
    :param sock: socket that communicates with robot via urscript (port number should be 30002)
    :return:
    '''
    sock = vars.urscript_sock


    xopt = vars.xoptPub

    xopt = N.array(xopt)
    disp = N.array(vars.rosDisp)
#    xopt = (xopt + disp).tolist()
    q = xopt
    gain = '200'
    command = "servoj([{0},{1},{2},{3},{4},{5}],t=.13,lookahead_time=0.01,gain={6})".format(str(q[0]),str(q[1]),str(q[2]),str(q[3]),str(q[4]),str(q[5]),gain) + "\n"
#    command = "servoj([{0},{1},{2},{3},{4},{5}],t=.2,lookahead_time=0.01,gain={6})".format(str(q[0]),str(q[1]),str(q[2]),str(q[3]),str(q[4]),str(q[5]),gain) + "\n"
    if vars.move_robot:
        sock.send(command)
    # print q

def moveJ_ur5(vars):
    sock = vars.urscript_sock

    xopt = vars.xoptPub
    xopt = N.array(xopt)
#    disp = N.array(vars.rosDisp)
#    q = (xopt + disp).tolist()
    q = xopt
    command = "movej([{0},{1},{2},{3},{4},{5}],t=6)".format(str(q[0]), str(q[1]), str(q[2]), str(q[3]), str(q[4]),str(q[5])) + "\n"
#    command = "movej([{0},{1},{2},{3},{4},{5}],t=2.5)".format(str(q[0]), str(q[1]), str(q[2]), str(q[3]), str(q[4]),str(q[5])) + "\n"
    sock.send(command)

def publish(vars):
    pubVREP(vars)
    pubROS(vars)
    pubGripperROS(vars)
    if vars.move_robot:
        move_ur5(vars)
