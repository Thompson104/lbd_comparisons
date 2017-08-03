import rosbag
import argparse
from tf import TransformerROS as rtf
from urdf_parser_py.urdf import URDF
#import roslib; roslib.load_manifest('urdf_parser_py')


def  parseTongsBag(bagname):

    bag = rosbag.Bag(bagname)
    forceSensor1array = []
    forceSensor2array = []
    encoderarray = []
    vivePos = []
    viveQuat = []
    topics = set()

    for topic, msg, t in bag.read_messages():
        topics.add(topic)
        if (topic == '/tongs/encoder'):
            encoderarray.append([msg.header.stamp, msg.position[0]])
        elif (topic == '/forceSensor1'):
            forceSensor1array.append([msg.header.stamp, msg.vector.x, msg.vector.y, msg.vector.z])
        elif (topic == '/forceSensor2'):
            forceSensor2array.append([msg.header.stamp, msg.vector.x, msg.vector.y, msg.vector.z])
        elif (topic == '/ViveQuat'):
            vivePos.append(msg)
        elif (topic == '/VivePos'):
            viveQuat.append(msg)
        else:
            pass

    data = {};
    data['forceSensor1array'] = forceSensor1array
    data['forceSensor2array'] = forceSensor2array
    data['encoderarray'] = encoderarray
    data['vivePosarray'] = vivePos
    data['viveQuatarray'] = viveQuat
    data['topics'] = topics
    bag.close()
    # return data['topics']
    return data


if __name__ == "__main__":
    # parser = argparse.ArgumentParser()
    # parser.add_argument('-f', '--filename', help='The file name/path of the bag.', required=True)
    #
    # args = vars(parser.parse_args())
    # bagname = args["filename"]
    # data = parseTongsBag(bagname)
    # print data
    bagname = './bagFiles/bagFiles_1_13_17/assemblingLegos.bag'
    data = parseTongsBag(bagname)
    print data['viveQuatarray']

