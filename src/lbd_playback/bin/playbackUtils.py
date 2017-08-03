import numpy as np
import IK.transformations as T
from usingRosBag_linear import *
from IK.tongsCenter import *


class PlaybackUtils:
    def __init__(self, vars):
        # Global variables
        self.positionDisplacement = np.array([0,0,-0.22])
        self.vars = vars
        # self.positionDisplacement = np.array([0,0,-0.38])
        self.tongsLength = 0.2286
        self.gripperLength = 0.1524

    def getNextDataColumn(self, time, parser, tongsTransform):
        '''
        :param time: the desired time to be found (in seconds)
        :param parser: the bag parser that contains the data table
        :param tongsTransform: tongs transform object to get center
        :return: pos, quaternion, encoder, force values corresponding to the given time
        '''
        timeIdx = self.find_closest(parser.timeStampArray, time)
        time = parser.resample_time_stamp[timeIdx]
        pos = parser.vivePos_interpolated[timeIdx]
        pos = self.transformPosition(pos)
        quat = parser.viveQuat_interpolated[timeIdx]
        encoder = parser.encoderarray_interpolated[timeIdx]
        pos = tongsTransform.getCenterPosition(pos,quat,encoder)
        return time, pos, quat, encoder


    def find_closest(self,A, target):
        #A must be sorted
        idx = A.searchsorted(target)
        idx = np.clip(idx, 1, len(A)-1)
        left = A[idx-1]
        right = A[idx]
        idx -= target - left < right - target
        return idx


    def checkValidConfig(self,pos, quat):
        '''
        checks if the given position and orientation are reachable by the ur5 robot arm
        :param pos:
        :param quat:
        :return:
        '''
        # stub
        return True

    def transformPosition(self, pos):
        '''
        rotates the position to the VREP and urscript version of global space
        :param pos:
        :return:
        '''
        # posRet = [pos[1],-pos[0],pos[2]]
        posRet = pos
        posRet = np.array(posRet)
        posRet -= self.positionDisplacement
        return posRet.tolist()

    def transformQuat(self, quat):
        '''
        rotates the quaternion to the VREP and urscript version of global space
        :param pos:
        :return:
        '''
        quatMat = T.quaternion_matrix(quat)
        retMat = np.zeros((4,4))
        retMat[:,0] = quatMat[:,1]
        retMat[:,1] = -quatMat[:,0]
        retMat[:,2] = quatMat[:,2]
        return T.quaternion_from_matrix(quatMat)

    def getGripperValue(self):
        '''
        returns the gripper value (between 0 and 0.085) that corresponds to the encoder value in radians
        :param enocder: encoder value in radians
        :return:
        '''
        # 0 for open, 1 for close
        flag = 0
        pos = self.vars.eeGoalPos
        quat = self.vars.eeGoalOr
        encoder = self.vars.encoderValue
        if encoder < 0.035 and flag == 0:
            encoder = 0.0
            flag = 1
        elif encoder >= 0.035 and flag == 1:
            encoder = 0.085
            flag = 0
        return encoder
        '''
        distance = self.vars.TongsTransform.getTongsDistance(pos,quat,encoder)

        if distance < 0.0:
            return 0.0

        #if distance > 0.066675:
        #    return 0.085

        u = distance / 0.34
        return u*0.085
        '''




################## DEPRECATED FUNCTIONS ##############################################

'''
# get the next position and orientation in the file
# note that the position and orientation don't have to be at the same time in the csv
# quaternion is [w,x,y,z]
def getNextHandConfig(filePtr):
    DEPRECATED
    get the next position and orientation in the file
    note that the position and orientation don't have to be at the same time in the csv
    quaternion is [w,x,y,z]
    :param filePtr:
    :return:
    posRet = []
    quatRet = []
    timeRet = []

    while posRet == [] or quatRet == []:
        line = filePtr.readline()
        if line == '':
            return [posRet, quatRet, timeRet]
        lineArr = line.split(',')
        time = extractTime(lineArr[0])

        if posRet == []:
            posXs = lineArr[1]
            posYs = lineArr[2]
            posZs = lineArr[3]
            if not posXs == '':
                posRet = [float(posXs), float(posYs), float(posZs)]
                if not posRet == [] and not quatRet == []:
                    timeRet = time
                    break

        if quatRet == []:
            quatWs = lineArr[4]
            quatXs = lineArr[5]
            quatYs = lineArr[6]
            quatZs = lineArr[7]
            if not quatWs == '':
                quatRet = [float(quatWs),float(quatXs),float(quatYs),float(quatZs)]
                if not posRet == [] and not quatRet == []:
                    timeRet = time
                    break

    posRet = transformPosition(posRet)
    quatRet = transformQuat(quatRet)
    # posRet = transformToGripperPos(posRet,quatRet)
    posRet = tc.getCenterPosition(posRet, quatRet, 0.5)
    return [posRet, quatRet, timeRet]
'''

'''
def extractTime(timeLine):

    takes in the time header from the bag file and extracts the time
    :param timeLine:
    :return:

    timeArr = timeLine.split(':')
    return float(timeArr[-1])
'''





