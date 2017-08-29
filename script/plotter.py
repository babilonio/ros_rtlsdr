#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32MultiArray

import numpy as np
import time
import matplotlib.pyplot as plt
plt.ion()


start = time.time()
def callback(data):
    global start
    end = time.time()
    rospy.loginfo(" Rate %f", 1 / (end - start ))
    start = time.time()

    rospy.loginfo(rospy.get_caller_id() + " I heard %d", len(data.data))
    rospy.loginfo("Max is  %f", max(data.data))
    n = np.arange(0,1,1.0/len(data.data))
    x = np.array(data.data)

    try:
        plt.plot(n, x, hold=False)
        plt.axis([0, 1, -1 , 1])
        plt.pause(0.01)
    except:
        print("error in plotting")


def listener():

    rospy.init_node('plotter', anonymous=True)

    rospy.Subscriber("psd", Float32MultiArray, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
