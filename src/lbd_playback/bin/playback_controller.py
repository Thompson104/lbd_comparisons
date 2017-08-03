import os
from playbackUtils import *
from Spacetime.arm import *
from playbackVars import *
from IK.OptimizationMode import *
from IK.Broadcaster import *
from usingRosBag_linear import *
from IK.tongsCenter import *


class playback_controller:

    def __init__(self, bagName):
        ###################
        dataOffline = True
        move_robot = False
        sampleRate = 1000
        ###################

        #load file from csv folder
        dir = os.path.dirname(__file__)
        fp = str(dir) + '/bagFiles/' + str(bagName) + '.bag'
        solutionFilePath = str(dir) + '/solutionFile.txt'
        self.arm = UR5()
        self.vars = PlaybackVars(solutionFilePath, sampleRate=sampleRate, dataOffline=dataOffline, move_robot=move_robot)
        self.vars.TongsTransform = GetTongsTransform()
        self.vars.Parser = RosBagParser(resampleRate=sampleRate)
        self.vars.Parser.parseTongsBag(fp)
        self.om = OptimizationMode(self.vars)
        self.utils = PlaybackUtils(self.vars)
        self.vars.Utils = self.utils

    def play(self):
        # pos, quat, time = getNextHandConfig(self.dataFile)
        time, pos, quat, encoder = self.utils.getNextDataColumn(0.01,self.vars.Parser,self.vars.TongsTransform)
        self.om.update(pos,quat)
        self.om.update(pos, quat)
        self.vars.initialIteration = False
        #safely place robot in initial configuration
        print 'Resetting Robot'
        pubVREP(self.vars)
        if self.vars.move_robot:
            moveJ_ur5(self.vars)

        rospy.sleep(5)


        tDiv = 1000000000.0
        endTime = self.vars.Parser.resample_time_stamp[-1]
        startTime = float(rospy.Time.now().to_nsec())
        currTime = 0.0
        while not currTime >= endTime and not rospy.is_shutdown():
            currTime = float(rospy.Time.now().to_nsec() - startTime) / tDiv
            time, pos, quat, encoder = self.utils.getNextDataColumn(currTime, self.vars.Parser, self.vars.TongsTransform)
            self.om.update(pos, quat)
            print currTime
            self.vars.encoderValue = encoder
            publish(self.vars)
            currTime = float(rospy.Time.now().to_nsec() - startTime) / tDiv






