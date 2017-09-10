#!/usr/bin/env python
import rospy
from std_msgs.msg import UInt32
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import NavSatFix
from rosrtlsdr.srv import ParamSet

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


def nowstr():
    return datetime.datetime.fromtimestamp(
        int(time.time())
    ).strftime('[%Y-%m-%d,%H:%M:%S]')


class Bridge(object):
    def __init__(self):
        self._vector = np.zeros(1024)
        self.sample_rate = 1e6
        self.center_freq = 102.8e6
        self.estimated_power = 0.0
        self.sock = None;
        self.addr = None;

    @property
    def vector(self):
        return self._vector

    @vector.setter
    def vector(self, v):
        self._vector = v

    def genString(self):
        w = self.vector / 48  # / max(abs(self.vector))
        s = ",".join(str(x) for x in list(w))

        return s


    def psd_callback(self, data):
        self.vector = np.array(data.data).astype(float)
        if self.addr != None:
            self.sock.sendto("PSD," + self.genString(), self.addr)


    def sample_rate_callback(self, data):
        self.sample_rate = data.data
        if self.addr != None:
            self.sock.sendto("SRT," + str(self.sample_rate), self.addr)


    def center_freq_callback(self, data):
        self.center_freq = data.data
        if self.addr != None:
            self.sock.sendto("CFQ," + str(self.center_freq), self.addr)


    def estimated_power_callback(self, data):
        self.estimated_power = data.data
        if self.addr != None:
            self.sock.sendto("POW," + str(self.estimated_power) + " dB", self.addr)


def call_set_sample_rate(sr):
    rospy.wait_for_service('set_sample_rate')
    try:
        set_sample_rate = rospy.ServiceProxy('set_sample_rate', ParamSet)
        resp1 = set_sample_rate(sr)
        return resp1.error
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


def call_set_center_freq(cf):
    rospy.wait_for_service('set_center_freq')
    try:
        set_center_freq = rospy.ServiceProxy('set_center_freq', ParamSet)
        resp1 = set_center_freq(cf)
        return resp1.error
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


def udp_bridge():

    pub_location = rospy.Publisher('location', NavSatFix, queue_size=10)
    pub_bwl = rospy.Publisher(
        'selected_bandwidth_left', UInt32, queue_size=10)
    pub_bwr = rospy.Publisher(
        'selected_bandwidth_right', UInt32, queue_size=10)

    rospy.init_node('udp_bridge', anonymous=True)

    bridge = Bridge()

    UDP_IP = "224.0.0.1"
    UDP_PORT = 6100
    bridge.sock = socket.socket(socket.AF_INET,  # Internet
                         socket.SOCK_DGRAM)  # UDP
    bridge.sock.bind((UDP_IP, UDP_PORT))

    lat, lon, alt = 0, 0, 0
    height, width = 0, 0

    rospy.Subscriber("psd", Float32MultiArray, bridge.psd_callback)
    rospy.Subscriber("sample_rate", UInt32, bridge.sample_rate_callback)
    rospy.Subscriber("center_freq", UInt32, bridge.center_freq_callback)
    rospy.Subscriber("estimated_power", Float32, bridge.estimated_power_callback)

    while not rospy.is_shutdown():

        data, bridge.addr = bridge.sock.recvfrom(1024)  # buffer size is 1024 bytes
        msg = data.split(',')
        bridge.sock.sendto("ACK", bridge.addr)

        try:
            if msg[0] == "LOC":
                print nowstr() + bcolors.OKBLUE + " Location : ", nav.latitude, nav.longitude, nav.altitude, bcolors.ENDC
                nav = NavSatFix()
                nav.latitude = float(msg[1])
                nav.longitude = float(msg[2])
                nav.altitude = float(msg[3])
                pub_location.publish(nav)

            elif msg[0] == "BWL":
                print nowstr() + bcolors.OKGREEN + " BWL ", msg[1], bcolors.ENDC
                bw = Float32()
                bw.data = float(msg[1])
                pub_bwl.publish(bw)

            elif msg[0] == "BWR":
                print nowstr() + bcolors.OKGREEN + " BWR ", msg[1], bcolors.ENDC
                bw = Float32()
                bw.data = float(msg[1])
                pub_bwr.publish(bw)

            # elif msg[0] == "PSD":
            #     print nowstr() + bcolors.OKGREEN + " PSD ", msg[1], " requested", bcolors.ENDC
            #     sock.sendto("CFQ," + str(wave.center_freq), addr)
            #     sock.sendto("SRT," + str(wave.sample_rate), addr)
            #     sock.sendto("POW," + str(wave.estimated_power) + " dB", addr)
            #     sock.sendto("PSD," + wave.genString(), addr)

            elif msg[0] == "SSR":
                print nowstr() + bcolors.OKGREEN + " SSR : ", msg[1], bcolors.ENDC
                sr = UInt32()
                sr.data = int(msg[1])
                call_set_sample_rate(sr)

            elif msg[0] == "SCF":
                print nowstr() + bcolors.OKGREEN + " SCF : ", msg[1], bcolors.ENDC
                cf = UInt32()
                cf.data = int(msg[1])
                call_set_center_freq(cf)

            else:
                print nowstr() + bcolors.WARNING + " Received : ", data, bcolors.ENDC
        except:
            print nowstr() + bcolors.FAIL + " Error reading MSG" + bcolors.ENDC


if __name__ == '__main__':
    try:
        udp_bridge()
    except rospy.ROSInterruptException:
        pass
