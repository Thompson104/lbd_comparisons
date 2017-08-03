#!/usr/bin/env python

import rospy
from playback_controller import playback_controller
from colors import *

if __name__ == '__main__':
    rospy.init_node('playback_node')
    print bcolors.OKGREEN + 'Playback Successfully Itialized.' + bcolors.ENDC
    #openDrawer2
    #setDownPlate
    filename = rospy.get_param('~playback_filename', 'graspBottleTest5')
    pc = playback_controller(filename)
    pc.play()