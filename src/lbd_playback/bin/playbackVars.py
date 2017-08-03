import numpy as np

import socket
from Spacetime.arm import *
import math as M
import numpy as N
import IK.transformations as T
from IK.UR5_dataReader import *
from MikesToys.timer import Timer
from robotiq_85_msgs.msg import GripperCmd, GripperStat
import rospy


class PlaybackVars:
    def __init__(self, solutionFile, arm = UR5(), ip='192.168.1.106', urip = '192.168.1.108', sampleRate = 1000, move_robot=False, dataOffline=False):
        # NETWORKING OPTIONS#################################################################
        self.ip = ip
        self.dataOffline = dataOffline
        self.UDP_PORT = 11000
        if not dataOffline:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.bind((self.ip, self.UDP_PORT))
            self.sock.setsockopt(socket.SOL_SOCKET,socket.SO_RCVBUF,512)
        self.urip = urip
        if move_robot:
            self.urip = urip
            self.UR_PORT = 30002
            print self.urip
            self.urscript_sock = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
            self.urscript_sock.connect((self.urip, self.UR_PORT))
            self.ur5_dataReader = UR5_dataReader()
        #####################################################################################

        # ARM OPTIONS########################################################################
        self.arm = arm
        self.numDOFs = len(self.arm.axes)
        if self.arm.__name__ == 'Robot:UR5':
            self.initState =  [M.pi, -0.38, -1.2, -M.pi/2, -M.pi/2, -M.pi/2] # VREP, alternate start with wrist rotated down
            self.rosDisp = [-M.pi, -M.pi/2, 0, -M.pi/2, 0, M.pi/2]
            self.opBounds = [(-10,10),(-1.5,1.5),(-2.5,2.5),(-2.3,2.3),(-1.8,1.8),(-10,10)]
        elif self.arm.__name__ == 'Robot:IIWA7':
            self.initState = [-0.6,1,1.2,-.8,0,0,.7]
            self.opBounds =  [tuple((-2*M.pi,2*M.pi)) for i in range(0,self.numDOFs)]
        else:
            raise Exception('ERROR: Trying to initialize playback with invalid robot.')
        #####################################################################################


         # SOLVER VARIABLES ##################################################################
        self.xopt =           self.initState # current Solution
        self.xoptPub =        []
        self.allConfigs =     [] # list of all configurations calculated by the solver
        self.allConfigsNP =   []
        for i in range(10):
            self.allConfigsNP.append(N.array(self.initState)) # jumpstart list by having 10 initial configs for finite differencing
        self.allEEPos =       []
        self.timer =          Timer()
        self.prevS =          self.initState # previous configuration state
        self.prevV =          self.numDOFs * [0] # previous velocity
        self.prevA =          self.numDOFs * [0] # previous acceleration
        self.ee_origPos =     self.arm.getFrames(self.initState)[0][-1] # original end effector point
        for i in range(10):
            self.allEEPos.append(self.ee_origPos)  # jumpstart list by having 10 initial configs for finite differencing
        self.prev_eePt =      self.ee_origPos # previous end effector point for cartesian space smoothing
        self.ee_origQuat =    T.quaternion_from_matrix(self.arm.getFrames(self.initState)[1][-1]) # original orientation quaternion
        self.prevHandPos =    N.array([0,0,0]) # previous hand position for calculating hand velocity
        self.maxVel =         0.0 # maximum velocity detected
        self.eeGoalPos =      self.ee_origPos
        self.eeGoalOr =       self.ee_origQuat
        self.handState =      0
        self.handPos =        N.array([0,0,0])
        self.handVelNorm =    0.0
        self.handVelMax =     0.04
        self.maxRotCon =      0.2 # maximum rotation allowed as constraint (at each joint)
        self.prevGoalOr =     self.ee_origQuat
        self.move_robot =     move_robot
        self.sampleRate =     sampleRate
        self.encoderValue =   6.8
        self.initialIteration =  True
        #####################################################################################



        # OPTIMIZATION WEIGHTS ##############################################################
        self.velW =  3.0 # previous state smoothness weight
        self.accW =  10.0 # accleration smoothing weight
        self.jrkW =  1.0 # jerk smoothing weight
        self.eepW =  15.0 # end effector positional weight
        self.uoW =   4.0 # underconstrained orientation weight
        self.eeoW =  8.0 # end effector orientation weight
        self.elbW =  0.0 # elbow goal weight
        self.jtPtW = 1.0 # end effector joint point velocity weight
        #####################################################################################


        # FILES #############################################################################
        self.solutionFile = solutionFile
        #####################################################################################


        # TEST INFORMATION ##################################################################
        self.posDeviation = 0.0 # meters
        self.rotDeviation = 0.0 # radians
        self.numSolutions = 0.0 # number of total solutions solved for

        # OBJECTS ###########################################################################
        self.TongsTransform = [] # tongs transform object
        self.Parser = []
        self.Utils = []

        # GRIPPER ############################################################################
        self.gripper_pub = rospy.Publisher('/gripper/cmd', GripperCmd, queue_size=10)
        self.gripper_cmd = GripperCmd()
