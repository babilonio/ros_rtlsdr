#!/usr/bin/env python
import rospy
from std_msgs.msg import UInt32
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import NavSatFix

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
        self.sample_rate = 1e6
        self.center_freq = 102.8e6

    @property
    def vector(self):
        return self._vector

    @vector.setter
    def vector(self, v):
        self._vector = v

    def genString(self):
        w = self.vector /50# / max(abs(self.vector))
        s = ",".join(str(x) for x in list(w))

        return s

def sample_rate_callback(data, wave):
    wave.sample_rate = data.data

def center_freq_callback(data, wave):
    wave.center_freq = data.data

def udp_bridge():

    pub_location = rospy.Publisher('location', NavSatFix, queue_size=10)
    pub_bw = rospy.Publisher('selected_bandwidth', Float32, queue_size=10)

    rospy.init_node('udp_bridge', anonymous=True)

    UDP_IP = "224.0.0.1"
    UDP_PORT = 6100
    sock = socket.socket(socket.AF_INET,  # Internet
                         socket.SOCK_DGRAM)  # UDP
    sock.bind((UDP_IP, UDP_PORT))

    wave = Wave()
    lat, lon, alt = 0, 0, 0
    height, width = 0, 0

    rospy.Subscriber("psd", Float32MultiArray, psd_callback, wave)
    rospy.Subscriber("sample_rate", UInt32, sample_rate_callback, wave)
    rospy.Subscriber("center_freq", UInt32, center_freq_callback, wave)

    while not rospy.is_shutdown():

        data, addr = sock.recvfrom(1024)  # buffer size is 1024 bytes
        msg = data.split(',')
        sock.sendto("ACK", addr)

        try:
            if msg[0] == "LOC":
                nav = NavSatFix()
                nav.latitude = float(msg[1])
                nav.longitude = float(msg[2])
                nav.altitude = float(msg[3])
                pub_location.publish(nav)
                print nowstr() + bcolors.OKBLUE + " Location : ", nav.latitude, nav.longitude, nav.altitude, bcolors.ENDC
            elif msg[0] == "MOU":
                mx = float(msg[1])
                my = float(msg[2])
                print nowstr() + bcolors.OKGREEN + " X/Y : ", mx, my, bcolors.ENDC
            elif msg[0] == "PSD":
                sock.sendto("FRE," + str(wave.sample_rate) + "," + str(wave.center_freq), addr)
                sock.sendto("PSD," + wave.genString(), addr)
                bw = Float32()
                bw.data = float(msg[1])
                pub_bw.publish(bw)
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
