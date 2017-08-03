#!/usr/bin/env python
# code: utf-8 -*-
# author: hwang
# created on: 05/31/2017
# modified on:
import rosbag 
import rospy
from sensor_msgs.msg import JointState
from datetime import datetime
#BAG_DIR = "/home/hwang/log_files/kinematics_demo/bag_files/" + str(datetime.now()).replace(' ','-') + '.bag'
BAG_DIR = "/home/hwang/log_files/kinematics_demo/bag_files/" + "test0.bag"

"""
This module helps communicate with the UR robot via xmlrpc. The class is designed to contain all the remote procedure
calls (prefixed by rpc) and helper functions for the server side. The server runs here and the client runs on the UR
robot. The UR continuously updates it's controller parameters through these rpc calls. The rpc server runs in a
seperate thread. Once the main thread is complete the server thread exits.
"""
class RosBagLogger:
    def __init__(self, bag=None):
        self._bag = bag

    def _callback_ur5(self,data):
        '''log robot joint states into rosbag file'''
        self._bag.write('/ur5/joint_states', data)

    def _callback_gripper(self,data):
        '''log gripper joint states into rosbag file'''
        self._bag.write('/gripper/joint_states', data)

    def listen(self):
        """listen the joint state from /joint state topic"""
        rospy.init_node('robot_sensing_listener', anonymous=True)
        rospy.Subscriber("ur5/joint_states", JointState, self._callback_ur5)
        rospy.Subscriber("gripper/joint_states", JointState, self._callback_gripper)
        rospy.spin()

if __name__ == "__main__":
    with rosbag.Bag(BAG_DIR, 'w') as bag:
        listener = RosBagLogger(bag)
        try:
            listener.listen()
        except rospy.ROSInterruptException:
            pass
