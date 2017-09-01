#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from std_msgs.msg import UInt32
from std_msgs.msg import Float32MultiArray

import argparse
import time
import numpy as np
import matplotlib.pyplot as plt
from scipy import signal


class PSDCalc(object):
    def __init__(self):
        self._fft_size = 1024
        self._sample_rate = 1e6
        self._inbytes = np.zeros(self._fft_size)

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
        print("estimate_psd process time : ", time.time() - start)
        return f[0:len(psd)], psd.astype(float)


def sample_rate_callback(data, psd):
    psd.sample_rate = data.data
    rospy.loginfo("sample_rate set to %u", psd.sample_rate)


def path_to_samples_callback(data, psd):
    try:
        psd.inbytes = np.fromfile(data.data, dtype=np.uint8)
    except:
        rospy.logerror("Error reading samples")


def calc_psd():

    # PARSE INPUT
    parser = argparse.ArgumentParser(description='Calculate a signal PSD ')
    parser.add_argument("--plot", help='show psd', action="store_true")
    args = parser.parse_args()

    pub = rospy.Publisher('psd', Float32MultiArray, queue_size=10)
    rospy.init_node('calc_psd', anonymous=True)
    psd = PSDCalc()

    rospy.Subscriber("sample_rate", UInt32, sample_rate_callback, psd)
    rospy.Subscriber("path_to_samples", String, path_to_samples_callback, psd)

    # spin() simply keeps python from exiting until this node is stopped
    # rospy.spin()

    if args.plot:
        plt.ion()
    rate = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown():
        psd_vector = Float32MultiArray()
        f, psd_vector.data = psd.estimate_psd()

        if args.plot:
            plt.plot(f, psd_vector.data, hold=False)
            plt.pause(0.005)

        pub.publish(psd_vector)
        rate.sleep()


if __name__ == '__main__':
    try:
        calc_psd()
    except rospy.ROSInterruptException:
        pass
