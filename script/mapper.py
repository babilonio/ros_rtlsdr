#!/usr/bin/env python
import rospy
from std_msgs.msg import Int16
from std_msgs.msg import Float32
from sensor_msgs.msg import NavSatFix

import time
from os.path import expanduser
import numpy as np
from math import radians, cos, sin, asin, sqrt

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm


def haversine(lat1, lon1, lat2, lon2):
    """
    Calculate the great circle distance between two points
    on the earth (specified in decimal degrees)
    """
    # convert decimal degrees to radians
    lon1, lat1, lon2, lat2 = map(radians, [lon1, lat1, lon2, lat2])

    # haversine formula
    dlon = lon2 - lon1
    dlat = lat2 - lat1
    a = sin(dlat / 2)**2 + cos(lat1) * cos(lat2) * sin(dlon / 2)**2
    c = 2 * asin(sqrt(a))
    r = 6371000  # Radius of earth in kilometers. Use 3956 for miles
    return c * r


class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'


class Location(object):
    def __init__(self, latitude, longitude):
        self.latitude = latitude
        self.longitude = longitude

    def __hash__(self):
        return hash((self.latitude, self.longitude))

    def __eq__(self, other):
        return (self.latitude, self.longitude) == (other.latitude, other.longitude)

    def __ne__(self, other):
        # Not strictly necessary, but to avoid having both x==y and x!=y
        # True at the same time
        return not(self == other)


class MAP(object):
    def __init__(self):
        self._ref = Location(0, 0)
        self._box_side_length = 10
        self._height = 10
        self._width = 10
        self._grid = np.zeros((self._width, self._height))

    @property
    def grid(self):
        return self._grid

    @grid.setter
    def grid(self, grid):
        self.grid = grid

    def position(self, location):
        x = haversine(self._ref.latitude, self._ref.longitude,
                      self._ref.latitude, location.longitude) / self._box_side_length
        y = haversine(self._ref.latitude, self._ref.longitude,
                      location.latitude, self._ref.longitude) / self._box_side_length
        return np.floor(x), np.floor(y)

    def resizeGrid(self, locations):
        latitudes = list(l.latitude for l in locations)
        longitudes = list(l.longitude for l in locations)

        self._ref = Location(max(latitudes), min(longitudes))
        self._height = np.ceil(haversine(max(latitudes), min(longitudes),
                                         min(latitudes), min(longitudes)) / self._box_side_length)
        self._width = np.ceil(haversine(max(latitudes), min(longitudes),
                                        max(latitudes), max(longitudes)) / self._box_side_length)
        self._grid = np.zeros((int(self._width), int(self._height)))

        print " self._ref : ", self._ref.latitude, self._ref.longitude
        print " self._height : ", self._height
        print " self._width : ", self._width

    def loadData(self, data):
        self.resizeGrid(data.keys())
        for key, value in data.iteritems():
            x, y = self.position(key)
            self._grid[int(x), int(y)] = value

    def interpolate(self):
        m, n = datamap.grid.shape
        again = True
        while again:
            print(bcolors.WARNING + "interpolating..." + bcolors.ENDC)

            again = False
            interpolated = np.zeros((m, n))

            for x in np.arange(0, m):
                for y in np.arange(0, n):
                    if self._grid[x, y] == 0:
                        mgs = 4  # minigrid_size
                        val, counter = 0, (mgs * 2 + 1) * (mgs * 2 + 1) - 1
                        for i in np.arange(-mgs, mgs + 1):
                            for j in np.arange(-mgs, mgs + 1):
                                if (i != 0 or j != 0):
                                    try:
                                        if (self._grid[x + i, y + j] == 0) or (x + i < 0) or (y + j < 0):
                                            counter = counter - 1
                                        else:
                                            val += self._grid[x + i, y + j]
                                    except IndexError:
                                        counter = counter - 1
                        if counter > 0 and val != 0:
                            interpolated[x, y] = val / counter
                        if val == 0:
                            again = True

            self._grid += interpolated


class Node(object):
    def __init__(self):
        self.data = {}
        self.value = 0
        self.mapping = 0

    def estimated_power_callback(self, data):
        self.value = data.data

    def mapping_callback(self, data):
        self.mapping = data.data

    def location_callback(self, nav):

        latitude = nav.latitude
        longitude = nav.longitude

        key = Location(latitude, longitude)

        rospy.loginfo("%slocation_callback, {[%f,%f], %f}%s",
                      bcolors.OKGREEN, latitude, longitude, self.value, bcolors.ENDC)

        self.data.update({key: self.value})

    def mapper(self):

        rospy.init_node('mapper', anonymous=True)

        rospy.Subscriber("mapping", Int16, self.mapping_callback)
        rospy.Subscriber("location", NavSatFix, self.location_callback)
        rospy.Subscriber("estimated_power", Float32,
                         self.estimated_power_callback)

        lastsave = time.time()

        rate = rospy.Rate(10)  # 10hz
        while not rospy.is_shutdown():

            if(self.mapping):
                if(time.time() - lastsave > 10):
                    lastsave = time.time()

                    rospy.loginfo("%sSaving data, n : %d%s",
                                  bcolors.OKBLUE, len(self.data.keys()), bcolors.ENDC)

                    np.save(expanduser("~") + "/catkin_ws/data/data.npy", self.data)
            else:
                rospy.loginfo("%sNothing to do...%s",
                              bcolors.OKBLUE, bcolors.ENDC)

            rate.sleep()


if __name__ == '__main__':
    try:
        m = Node()
        m.mapper()
    except rospy.ROSInterruptException:
        pass

    # data = {}
    # for i in range(1000):
    #
    #     latitude = 36.73500 + np.random.rand(1).astype(float)[0] / 100.0
    #     longitude = -4.55400 + np.random.rand(1).astype(float)[0] / 100.0
    #     #val = np.random.rand(1).astype(float)[0] * 100
    #     val = haversine(36.73500, -4.55400, latitude, longitude)
    #     key = Location(latitude, longitude)
    #     value = val
    #     data.update({key: value})
    #
    # np.save('data.npy', data)
    # datap = np.load('data.npy').item()
    # datamap = MAP()
    # datamap.loadData(datap)
    # datamap.interpolate()
    #
    # plt.imshow(datamap.grid, cmap='hot', interpolation='none')
    # plt.show()

    # m, n = datamap.grid.shape
    # X = np.arange(0, m)
    # Y = np.arange(0, n)
    # X, Y = np.meshgrid(X, Y)
    # Z = datamap.grid[X, Y]
    #
    # fig = plt.figure()
    # ax = fig.add_subplot(1, 2, 1, projection='3d')
    # surf = ax.plot_surface(X, Y, Z, rstride=1, cstride=1, cmap=cm.coolwarm,
    #                        linewidth=0, antialiased=False)
    #
    # ax = fig.add_subplot(1, 2, 2, projection='3d')
    #
    # datamap.interpolate()
    # Z = datamap.grid[X, Y]
    #
    # surf = ax.plot_surface(X, Y, Z, rstride=1, cstride=1, cmap=cm.coolwarm,
    #                        linewidth=0, antialiased=False)
    # plt.show()
