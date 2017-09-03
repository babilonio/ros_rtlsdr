#!/usr/bin/python
import numpy as np
from math import radians, cos, sin, asin, sqrt


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
        self._box_side_length = 1.0
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

            print x, y, value


if __name__ == '__main__':
    data = {}

    for i in range(10000):

        latitude = 36.73500 + np.random.rand(1).astype(float)[0] / 10000.0
        longitude = -4.55400 + np.random.rand(1).astype(float)[0] / 10000.0
        val = np.random.rand(1).astype(float)[0] * 100
        key = Location(latitude, longitude)
        value = val
        data.update({key: value})

    datamap = MAP()
    datamap.loadData(data)
    print datamap.grid
