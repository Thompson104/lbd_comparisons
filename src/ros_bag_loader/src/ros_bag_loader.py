#!/usr/bin/env python
# license removed for brevity
import rosbag
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import JointState
import csv

ROS_BAG_DIR = '/home/hwang/log_files/kinematics_demo/bag_files/'
OUT_UR_CSV_DIR = '/home/hwang/log_files/kinematics_demo/data_tables/'
OUT_GRIPPER_CSV_DIR = '/home/hwang/log_files/kinematics_demo/data_tables/'
BAG_NAME_BASE = '/home/hwang/log_files/kinematics_demo/'

def get_prefix():
    with open(BAG_NAME_BASE+'bag_name.txt', 'rb') as bag_name_file:
    	prefix=bag_name_file.readline()
    return prefix.rstrip('\n')

def out_to_ur_csv(data=None, out_writer=None):
    """write out the data table to csv file"""
    out_writer.writerow([data.header.stamp.to_nsec(), list(data.position)])

def out_to_gripper_csv(data=None, out_writer=None, t=None):
    """write out the data table to csv file"""
    out_writer.writerow([t.to_nsec(), data.position, data.speed, data.force])

if __name__ == "__main__":
#    parser = argparse.ArgumentParser()
#    parser.add_argument('-f', '--filename', help='The file name/path of the bag.', required=True)
#    args = vars(parser.parse_args())
    bag_file_name=get_prefix()
    bag_file_base=bag_file_name.rstrip('.bag')
    bag = rosbag.Bag(ROS_BAG_DIR+bag_file_name)
    rospy.init_node('ros_bag_loader', anonymous=True)
    pub_ur5 = rospy.Publisher('ur5/joint_states_cmd', JointState, queue_size=10)
    # pub_gripper = rospy.Publisher('gripper/joint_states_cmd', JointState, queue_size=10)
    rate = rospy.Rate(1000) # publishing with 100 Hz
    with open(OUT_UR_CSV_DIR+bag_file_base+'_ur.csv', 'wb') as ur_csv_file, open(OUT_GRIPPER_CSV_DIR+bag_file_base+'_gripper.csv', 'wb') as gripper_csv_file:
        ur_csv_writer = csv.writer(ur_csv_file)
	gripper_csv_writer = csv.writer(gripper_csv_file)
        for topic, msg, t in bag.read_messages(topics=['/ur5/joint_states', '/gripper/cmd']):
	    if topic == '/ur5/joint_states':
	        pub_ur5.publish(msg)
	        out_to_ur_csv(data=msg, out_writer=ur_csv_writer)
	    elif topic == '/gripper/cmd':
	        # pub_gripper.publish(msg)
		out_to_gripper_csv(data=msg, out_writer=gripper_csv_writer, t=t)
	    rate.sleep()
    bag.close()

