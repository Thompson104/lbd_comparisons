__author__ = 'drakita'

from abc import ABCMeta,abstractmethod

class ControlMode:
    '''
    Control mode is the abstract class of all the different modes to control the robot during mimicry control

    Such control modes could be optimization mode, controller rotation mode, elbow flip mode...
    '''
    __metaclass__ = ABCMeta

    def __init__(self):
        self.ID = -1 # override this field in child class
        self.prevMode = self.ID

    def ModeChange(self,prevModeID):
        '''
        signifies that at this update iteration, there was a mode change.
        :param prevModeID: The ID of the previous mode being switched from
        :return:
        '''
        self.prevMode = prevModeID

    # all update modes must return a robot configuration for the current robot
    @abstractmethod
    def update(self): pass

