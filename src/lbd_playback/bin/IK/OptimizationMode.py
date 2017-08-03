__author__ = 'drakita'

from ControlMode import ControlMode
from MimIKOptimizeFuncs import *
import scipy.optimize as O
from playbackVars import *
from Spacetime.arm import *
import scipy


class OptimizationMode(ControlMode):

    '''
    optimization mode is a control mode that updates robot configuration based on objectives, constraints, and joint limits

    these are solved using scipy minimize function, slsqp method
    '''

    def __init__(self,vars):
        self.ID = 0 # id for optimization mode
        self.prevMode = self.ID
        self.vars = vars
        self.cons = ({'type': 'ineq', 'fun':singCon, 'args':(self.vars,)})
        self.bounds = self.vars.opBounds
        self.success = 0.0
        self.totalIterations = 0.0
        self.xvalSum = 0.0
        self.successRate = 0.0

    def update(self, goalPos, goalQuat):
        self.vars.timer.start()

        self.vars.prevGoalOr = self.vars.eeGoalOr
        self.vars.prevHandPos = self.vars.handPos
        self.vars.handPos = N.array(goalPos)
        self.vars.handVelNorm = N.linalg.norm(self.vars.handPos - self.vars.prevHandPos)
        self.vars.eeGoalPos = N.array(goalPos)
        self.vars.eeGoalOr = N.array(goalQuat)

        initSol = self.vars.prevS

        xopt1 = O.minimize(obj,initSol,args=(self.vars,),method='slsqp',options={'maxiter':75,'disp':False})

        xopt = xopt1.x

        self.vars.timer.stop()
        self.vars.allConfigs.append(xopt)
        self.vars.allConfigsNP.append(N.array(xopt))
        self.vars.prevS = self.vars.xopt
        self.vars.prev_eePt = self.vars.arm.getFrames(self.vars.prevS)[0][-1]
        self.vars.allEEPos.append(self.vars.prev_eePt)
        self.vars.xopt = xopt
        self.vars.xoptPub = xopt.copy()
        self.vars.xoptPub[0] -= math.pi / 2.0 # rotate to ur5 frame (90 degrees off from rviz and vive frame)
        print self.vars.xoptPub

        if xopt1.status == 0:
            self.success += 1
        self.totalIterations += 1
        self.xvalSum += xopt1.fun
        # print xopt
        # print self.vars.timer

        # just for testing deviations!

        frames = self.vars.arm.getFrames(xopt)
        pDev = pScore(xopt, frames, self.vars)
        self.vars.posDeviation += pDev
        rDev = oScore(xopt, frames, self.vars)
        self.vars.rotDeviation += rDev
        self.vars.numSolutions += 1.0
        smoothScore = jtPtScore(xopt,frames,self.vars)**2
        posScore = pDev**2
        rotScore = rDev**2
        v = xopt - N.array(self.vars.prevS)
        velScore = N.linalg.norm(v)**2
        handVel = self.vars.handVelNorm

        # outLine = str(handVel) + ',' + str(posScore) + ',' + str(rotScore) + ',' + str(smoothScore) + ',' + str(velScore) + '\n'
        # self.vars.mimicOutputFile.write(outLine)

        #print 'iters: ' + str(self.vars.numSolutions) + '. ' + 'pos deviation: ' + str(self.vars.posDeviation / self.vars.numSolutions) + '. ' \
        #      + 'rot deviation: ' + str(self.vars.rotDeviation / self.vars.numSolutions)
        return xopt





