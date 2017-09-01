#!/usr/bin/env python
import rospy
from std_msgs.msg import UInt32
from std_msgs.msg import Float32MultiArray

import socket
import datetime
import time
import numpy as np
from scipy import signal

class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'


def psd_callback(data, wave):
    wave.vector = np.array(data.data).astype(float)

def nowstr():
    return datetime.datetime.fromtimestamp(
        int(time.time())
    ).strftime('[%Y-%m-%d,%H:%M:%S]')

class Wave(object):
    def __init__(self):
        self._vector = np.zeros(1024)

    @property
    def vector(self):
        return self._vector

    @vector.setter
    def vector(self, v):
        self._vector = v

    def genString(self):
        w = self.vector / max(abs(self.vector))
        s = ",".join(str(x) for x in list(w))

        return s


def udp_bridge():

    rospy.init_node('udp_bridge', anonymous=True)




    UDP_IP = "192.168.0.159"
    UDP_PORT = 6100
    sock = socket.socket(socket.AF_INET,  # Internet
                         socket.SOCK_DGRAM)  # UDP
    sock.bind((UDP_IP, UDP_PORT))

    wave = Wave()
    lat, lon, alt = 0, 0, 0
    height, width = 0, 0

    rospy.Subscriber("psd", Float32MultiArray, psd_callback, wave)

    while not rospy.is_shutdown():

        data, addr = sock.recvfrom(1024)  # buffer size is 1024 bytes
        msg = data.split(',')
        sock.sendto("ACK", addr)

        try:
            if msg[0] == "LOC":
                lat = float(msg[1])
                lon = float(msg[2])
                alt = float(msg[3])
                print nowstr() + bcolors.OKBLUE + " Location : ", lat, lon, alt, bcolors.ENDC
            elif msg[0] == "MOU":
                mx = float(msg[1])
                my = float(msg[2])
                print nowstr() + bcolors.OKGREEN + " X/Y : ", mx, my, bcolors.ENDC
            elif msg[0] == "PSD":
                sock.sendto("PSD," + wave.genString(), addr)
                print nowstr() + bcolors.OKGREEN + " PSD ", msg[1], " requested", bcolors.ENDC
            elif msg[0] == "SCR":
                height = int(msg[1])
                width = int(msg[2])
                print nowstr() + bcolors.OKGREEN + " SIZE : ", str(width) + "x" + str(height), bcolors.ENDC
            else:
                print nowstr() + bcolors.WARNING + " Received : ", data, bcolors.ENDC
        except:
            print nowstr() + bcolors.FAIL + " Error reading MSG" + bcolors.ENDC


if __name__ == '__main__':
    try:
        udp_bridge()
    except rospy.ROSInterruptException:
        pass
