__author__ = 'drakita'

'''
Contains functions necessary for relaxed mimik optimization, including objective and constraints
'''

import numpy as N
import IK.transformations as T
# from ad import gh
# from finiteDifference import *

def obj(x, *args):
    vars = args[0]

    v = x - N.array(vars.prevS)

    frames = vars.arm.getFrames(x)
    vJ = frames[0][-1] - vars.allEEPos[-1]

    velMax = vars.handVelMax
    velW = (max(0.0, velMax - vars.handVelNorm) / velMax) ** 2

    init = vars.initialIteration
    if init:
        return pScore(x,frames,vars)**2 + \
               oScore(x, frames, vars) ** 2

    return vars.velW*N.linalg.norm(v)**2 + \
           vars.eepW*pScore(x,frames,vars)**2 + \
           vars.jtPtW * N.linalg.norm(vJ)**2 + \
           vars.eeoW*oScore(x,frames,vars)**2


def pScore(x, frames, vars):
    positions = frames[0]
    eePos = positions[-1]
    return N.linalg.norm(eePos - vars.eeGoalPos)

def jtPtScore(x, frames, vars):
    jtPt = frames[0][-1]
    return N.linalg.norm(vars.prev_eePt - jtPt)

def oScore(x, frames, vars):
    frames = frames[1]
    eeMat = frames[-1]
    eeOr = T.quaternion_from_matrix(eeMat)

    eeGoalOr1 = vars.eeGoalOr
    eeGoalOr2 = [-vars.eeGoalOr[0], -vars.eeGoalOr[1], -vars.eeGoalOr[2], -vars.eeGoalOr[3]]

    disp1 = T.quaternion_disp(eeGoalOr1,eeOr)
    disp2 = T.quaternion_disp(eeGoalOr2,eeOr)

    return min( N.linalg.norm(disp1), N.linalg.norm(disp2))
    # return N.linalg.norm(disp1)


def uScore(x, frames, vars):
    eeMat = frames[1][-1]
    if vars.arm.__name__ == "Robot:UR5":
        axis = eeMat[:,2]
    elif vars.arm.__name__ == "Robot:IIWA7":
        axis = eeMat[:,0]
    else:
        raise Exception('ERROR: uScore must have valid robot name!')

    return abs(N.dot(N.array([0,0,1]), axis))

def singScore(x,frames,*args):
    arm, prevState,prevState2,prevState3,prev_eePt,eeGoalPos, eeGoalOr,hsf, esf,ee_origQuat, eg, handVelNorm = args
    j = arm.getJacobian(x)
    jT = j.T
    m = N.dot(j,jT)
    det = N.linalg.det(m)
    return N.sqrt(abs(det))

# singularity avoidance constraint
# based on manipulability measure y Yoshikawa (Manipulability of Robotic Mechanisms)
# sqrt( |det(J(x) J.T(x))| )
# 2^-24 seems to be a good lower bound for now
# will always be determinant of 6 x 6 matrix regardless of the DOF
def singCon(x, *args):
    # arm, prevState,prevState2,prevState3,prev_eePt,eeGoalPos, eeGoalOr,hsf, esf,ee_origQuat, eg, handVelNorm = args
    vars = args[0]
    arm = vars.arm
    frames = arm.getFrames(x)
    j = arm.getJacobian(x)
    jT = j.T
    m = N.dot(j,jT)
    det = N.linalg.det(m)
    sMin = N.power(2,-24.0)
    return det - sMin

def cons(x, *args):
    vars = args[0]
    s = sCon(x, vars)
    # return [s]
    return 0.1

def sCon(x, vars):
    v = x - vars.prevS
    max = N.amax(v)
    maxRot = vars.maxRotCon
    return maxRot - max
