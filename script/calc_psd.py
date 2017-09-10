#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from std_msgs.msg import UInt32
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import NavSatFix


import argparse
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


class PSDCalc(object):
    def __init__(self):
        self._fft_size = 1024
        self._sample_rate = 0
        self._inbytes = np.zeros(10)
        self.sbwl = 512
        self.sbwr = 512
        self.estimated_power = 0.0

    @property
    def sample_rate(self):
        return self._sample_rate

    @sample_rate.setter
    def sample_rate(self, sr):
        self._sample_rate = sr

    @property
    def inbytes(self):
        return self._inbytes

    @inbytes.setter
    def inbytes(self, inb):
        self._inbytes = inb
        # T = len(_inbytes)/(2.0*sample_rate)

    def estimate_psd(self):
        start = time.time()
        norm = np.empty(len(self._inbytes) // 2)
        norm.fill(128)
        samples = np.empty(len(self._inbytes) // 2, 'complex')
        samples.real = (self._inbytes[::2] - norm) / 128.0
        samples.imag = (self._inbytes[1::2] - norm) / 128.0

        f, Pxx_den = signal.welch(
            samples, self._sample_rate, nperseg=self._fft_size)

        N = len(Pxx_den) / 2
        psd = 10 * np.log10(self._sample_rate *
                            np.concatenate((Pxx_den[N:], Pxx_den[1:N])))
        nf = np.concatenate((f[N:], f[1:N]))

        if(self.sbwr > self.sbwl):
            power_window = sum(
                self._sample_rate * Pxx_den[self.sbwl:self.sbwr])

            rospy.loginfo("%sestimated_power : %f%s",
                          bcolors.OKBLUE, 10 * np.log10(power_window), bcolors.ENDC)
            self.estimated_power = 10 * np.log10(power_window)

        rospy.loginfo("%sestimate_psd process time : %f%s",
                      bcolors.OKBLUE, time.time() - start, bcolors.ENDC)
        return nf, psd.astype(float)


def selected_bandwidth_left_callback(data, psd):
    psd.sbwl = data.data
    rospy.loginfo("%sselected_bandwidth_left_callback : %d%s",
                  bcolors.OKGREEN, psd.sbwl, bcolors.ENDC)


def selected_bandwidth_right_callback(data, psd):
    psd.sbwr = data.data
    rospy.loginfo("%sselected_bandwidth_right_callback : %d%s",
                  bcolors.OKGREEN, psd.sbwr, bcolors.ENDC)


def sample_rate_callback(data, psd):
    psd.sample_rate = data.data
    rospy.loginfo("%ssample_rate_callback : %0.3fMHz%s",
                  bcolors.OKGREEN, psd.sample_rate / 1.0e6, bcolors.ENDC)


def path_to_samples_callback(data, psd):
    try:
        psd.inbytes = np.fromfile(data.data, dtype=np.uint8)
    except:
        rospy.logerr("Error reading samples")


def calc_psd():

    # PARSE INPUT
    parser = argparse.ArgumentParser(description='Calculate a signal PSD ')
    parser.add_argument("--plot", help='show psd',
                        action="store_true", default=False)
    try:
        args = parser.parse_args(rospy.myargv()[1:])
    except:
        print bcolors.FAIL, "argparse error", bcolors.ENDC

    pub_pow = rospy.Publisher('estimated_power', Float32, queue_size=10)
    pub = rospy.Publisher('psd', Float32MultiArray, queue_size=10)
    rospy.init_node('calc_psd', anonymous=True)

    psd = PSDCalc()
    rospy.Subscriber("selected_bandwidth_left", UInt32,
                     selected_bandwidth_left_callback, psd)
    rospy.Subscriber("selected_bandwidth_right", UInt32,
                     selected_bandwidth_right_callback, psd)
    rospy.Subscriber("sample_rate", UInt32, sample_rate_callback, psd)
    rospy.Subscriber("path_to_samples", String, path_to_samples_callback, psd)

    rate = rospy.Rate(10)  # 10hz

    while not rospy.is_shutdown() and psd.sample_rate == 0:
        rospy.logwarn("Waiting for topic to be published")
        rate.sleep()

    if args.plot:
        import matplotlib.pyplot as plt
        plt.ion()

    while not rospy.is_shutdown():
        try:
            psd_vector = Float32MultiArray()
            f, psd_vector.data = psd.estimate_psd()
            pub.publish(psd_vector)
            pub_pow.publish(psd.estimated_power)
        except:
            rospy.logerr("Error estimate_psd")

        if args.plot:
            plt.plot(f, psd_vector.data, hold=False)
            plt.ylim([-50, 20])
            plt.pause(0.005)

        rate.sleep()


if __name__ == '__main__':
    try:
        calc_psd()
    except rospy.ROSInterruptException:
        pass
