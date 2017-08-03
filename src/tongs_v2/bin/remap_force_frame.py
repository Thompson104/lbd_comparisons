#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Vector3Stamped
from geometry_msgs.msg import WrenchStamped

from copy import deepcopy

#Notes:
# Force sensor 1 is attached to rigid body 2 and vice versa. I may fix this later. This 
# is reflected in the subscriber creation, I flipped the subscribed signals. 

force_sensor1_pub = rospy.Publisher('/force_sensor_tongs_frame1', WrenchStamped, queue_size=10)
force_sensor2_pub = rospy.Publisher('/force_sensor_tongs_frame2', WrenchStamped, queue_size=10)


def callback_force_sensor_remap1(data):
    force_tongs_frame = WrenchStamped()
    force_tongs_frame.header.stamp = data.header.stamp
    force_tongs_frame.header.frame_id = "tongs_arm1"

    force_tongs_frame.wrench.force.x = -data.vector.y
    force_tongs_frame.wrench.force.y = data.vector.x
    force_tongs_frame.wrench.force.z = data.vector.z
    force_sensor1_pub.publish(force_tongs_frame)
    

def callback_force_sensor_remap2(data):
    force_tongs_frame = WrenchStamped()
    force_tongs_frame.header.stamp = data.header.stamp
    force_tongs_frame.header.frame_id = "tongs_arm2"

    force_tongs_frame.wrench.force.x = -data.vector.y
    force_tongs_frame.wrench.force.y = data.vector.x
    force_tongs_frame.wrench.force.z = data.vector.z
    force_sensor2_pub.publish(force_tongs_frame)
    


def tongs_force_remap_publisher():
    rospy.init_node('force_sensors_remap', anonymous=True)
    
    rospy.Subscriber("/forceSensor2", Vector3Stamped, callback_force_sensor_remap1)
    rospy.Subscriber("/forceSensor1", Vector3Stamped, callback_force_sensor_remap2)

    rospy.spin()




if __name__ == "__main__":
    tongs_force_remap_publisher()
