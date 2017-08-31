#!/usr/bin/env python
import rospy
from std_msgs.msg import String

import numpy as np
from scipy import signal
import time
from pylab import *
import matplotlib.pyplot as plt
plt.ion()

sample_rate = 2.4e6

start = time.time()
def callback(data):
    global start
    end = time.time()
    rospy.loginfo(" Rate %f", 1 / (end - start ))
    start = time.time()

    rospy.loginfo(rospy.get_caller_id() + " I heard %d", len(data.data))

    inbytes = np.fromfile("/tmp/samples.bin", dtype=np.uint8)
    print("Reading ", inbytes, " bytes, ", len(inbytes)/(2.0*sample_rate), " seconds")
    norm = np.empty(len(inbytes)//2)
    norm.fill(128)
    samples = np.empty(len(inbytes)//2, 'complex')

    samples.real = (inbytes[::2] - norm)/ 128.0
    samples.imag = (inbytes[1::2] - norm)/ 128.0
    end = time.time()
    print("Reading time: ", end - start)

    # start = time.time()
    # f, Pxx_den = signal.welch(samples, sample_rate, nperseg=1024)
    # N = len(Pxx_den)/2
    # wave =  10*np.log10(1e6*np.concatenate( (Pxx_den[N:], Pxx_den[1:N]) ) )
    # # wave = w  / max(abs(w))
    # print("Process time : ", time.time() - start)
    #
    # start = time.time()
    # # plt.plot(f/1e6, 10*np.log10(wave * 1e6), hold=False )
    # plt.plot(np.arange(0,len(wave)), wave, hold=False )
    psd(samples, NFFT=1024, Fs=sample_rate/1e6, Fc=102.4, hold=False)
    plt.xlabel('frequency [MHz]')
    plt.ylabel('PSD [dB/MHz]')
    ylim([-20,20])
    print("Plotting time : ", time.time() - start)

    plt.pause(0.005)


def listener():

    rospy.init_node('plotter', anonymous=True)

    rospy.Subscriber("path_to_samples", String, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
