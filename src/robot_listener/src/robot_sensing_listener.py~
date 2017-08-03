#!/usr/bin/env python
# code: utf-8 -*-
# author: hwang
# created on: 06/25/2017
# modified on: 
import xmlrpclib
from SimpleXMLRPCServer import SimpleXMLRPCServer
import threading
import rospy
from sensor_msgs.msg import JointState

"""
This module helps communicate with the UR robot via xmlrpc. The class is designed to contain all the remote procedure
calls (prefixed by rpc) and helper functions for the server side. The server runs here and the client runs on the UR
robot. The UR continuously updates it's controller parameters through these rpc calls. The rpc server runs in a
seperate thread. Once the main thread is complete the server thread exits.
"""
data_table = []
class rpc_ur_com:
    def __init__(self):
        # Remember to lock everytime something is accessed
        self.rlock = threading.RLock()

        # "1" for close, "0" for open
        self.pose_state = [0, 0, 0, 0, 0, 0]
	self._data_buffer = JointState()

    def get_gripper_cmd(self):
        with self.rlock:
            state = self.pose_state
        return state

server = SimpleXMLRPCServer(("", 8080), allow_none=True)
#print "Listening on port 8080..."
#rcom_inst = rpc_ur_com()

def callback(data):
    # data.position is a tuple here
    global data_table
#    rcom_inst.pose_state = list(data.position)
#    print rcom_inst.pose_state
    data_table.append([data.header.stamp, list(data.position)])

def listen():
    """listen the joint state from /joint state topic"""
    rospy.init_node('robot_sensing_listener', anonymous=True)
    rospy.Subscriber("ur5/joint_states_cmd", JointState, callback)
    rospy.spin()


if __name__ == "__main__":
#    server.register_instance(rcom_inst)
#    server_thread = threading.Thread(target = server.serve_forever)
#    server_thread.daemon = True
#    server_thread.start()
    try:
        listen()
    except rospy.ROSInterruptException:
        pass
    for item in data_table:
	print item
	print "==========================================================="
