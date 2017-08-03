#!/usr/bin/env python
import serial
from struct import *
import time
from copy import copy
import rospy
from geometry_msgs.msg import Accel
g = 9.8

class acceleration_sensor:
    def __init__(self,usb_port = '/dev/ttyACM0'):
        self.ser = serial.Serial( # port='/dev/cu.usbmodem1422',
            port='/dev/ttyACM0',
            baudrate=2000000
            )
        self.a1 = Accel()
        self.a2 = Accel()

    def read_data(self,package_format = 'ffffffffffff'):
        try:
            data_ = get_next_data_block(self.ser.read)
            data = unpack(package_format, ''.join(data_))

            self.a1.linear.x = data[0]/g
            self.a1.linear.y = -data[1]/g
            self.a1.linear.z = -data[2]/g
            self.a1.angular.x = data[3]/g
            self.a1.angular.y = -data[4]/g
            self.a1.angular.z = -data[5]/g

            self.a2.linear.x = data[6]/g
            self.a2.linear.y = -data[7]/g
            self.a2.linear.z = -data[8]/g
            self.a2.angular.x = data[9]/g
            self.a2.angular.y = -data[10]/g
            self.a2.angular.z = -data[11]/g
        except:
            data = {}
            print "data corrupt or something went wrong!"

        return data

def get_next_data_block(next_f):
    if not hasattr(get_next_data_block, "data_block"):
        get_next_data_block.data_block = []
    while (1):
        try:
            current_item = next_f()
            if current_item == '^':
                next_item = next_f()
                if next_item == '^':
                    get_next_data_block.data_block.append(next_item)
                else:
                    out = copy(get_next_data_block.data_block)
                    get_next_data_block.data_block = []
                    get_next_data_block.data_block.append(next_item)
                    return out
            else:
                get_next_data_block.data_block.append(current_item)
        except StopIteration:
            break

def accel_node():
    pub1 = rospy.Publisher('accel1', Accel, queue_size=10)
    pub2 = rospy.Publisher('accel2', Accel, queue_size=10)

    rospy.init_node('accel_sensors', anonymous=True)
    acc_sen = acceleration_sensor()


    # rate = rospy.Rate(1000)  # 10hz
    while not rospy.is_shutdown():
        acc_sen.read_data()

        pub1.publish(acc_sen.a1)
        pub2.publish(acc_sen.a2)

        # rate.sleep()


if __name__ == "__main__":
    try:
        accel_node()
    except rospy.ROSInterruptException:
        pass


