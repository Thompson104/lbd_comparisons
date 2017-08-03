__author__ = 'drakita'

import socket

class Networking:
    def __init__(self):
        # just containers, don't want to initialze with these since UR or regular port
        # may not always be necessary
        self.ip = '192.168.1.101'
        self.urip = '192,168.1.101'
        self.UDP_PORT = 11000
        self.UR_PORT = 30002

    def init_generic_socket(self, ip, port):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind((ip,port))
        sock.setsockopt(socket.SOL_SOCKET,socket.SO_RCVBUF,512)
        return sock

    def init_ur_socket(self, ip, port):
        sock = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
        sock.connect((ip, port))
        return sock

    def get_socket_data(self, sock):
        data, addr = sock.recvfrom(256)
        return data


