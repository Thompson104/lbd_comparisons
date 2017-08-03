from playbackUtils import *
import transformations as t
# from tongsCenter import *
from tongsCenter import *
# from usingRosBag_linear_version0 import *
from usingRosBag_linear_version2 import *
import numpy as np

rq = t.random_quaternion()

utils = PlaybackUtils(None)
utils.transformQuat(rq)
quatRot = t.quaternion_from_euler(0, 0, 90)
print quatRot

tc = GetTongsTransform()

'''
rbp = RosBagParser(resampleRate=1000)
rbp.parseTongsBag('/home/danny/Desktop/bagFiles/bagFiles_1_13_17/assemblingLegos.bag')
print len(rbp.vivePos_interpolated)
print len(rbp.viveQuat_interpolated)
print rbp.resample_time_stamp[12]
'''

A = np.array([1.0, 5.0, 6.0, 7.0, 11.0])
print tc.getTongsDistance([0,0,0], [1,0,0,0], 6.5)

a = np.zeros((4,4))
a[:,0] = np.array([0,1,2,3])
print a



