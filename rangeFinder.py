'''
Created on Nov 12, 2014

@author: Jesse
'''

from math import *
from numpy import median

class RangeFinder(object):
    '''
    Analyze x-y coordinate data and add additional dimensions
    The motion model needs linear and angular velocity and the heading
    Get this data by analyzing the x-y coordinates
    '''

    def __init__(self, ranges, minX, minY, maxX, maxY):
        """
        Initialize the data analyzer
        :type ranges: list
        :type minX: float
        :type minY: float
        :type maxX: float
        :type maxY: float
        :return:
        """
        self.data = ranges
        self.newData = [ [ self.data[0][0], self.data[0][1], 0, 0, 0 ] ]
        self.velocities = []
        if minX == 0 and minY == 0 and maxX == 0 and maxY == 0:
            # if here, then no pre-learned data is being used
            self.maxX = self.data[0][0]
            self.minX = self.data[0][0]
            self.maxY = self.data[0][1]
            self.minY = self.data[0][1]
        else:
            # if here, then we passed in some prelearned data
            self.maxX = maxX
            self.minX = minX
            self.maxY = maxY
            self.minY = minY

    def getRange(self):
        """
        :return: min and max ranges in the data
        """
        return (self.minX, self.minY, self.maxX, self.maxY)

    def getMediaVelocity(self):
        """
        :return: median velocity
        """
        return median(self.velocities)

    def getNewData(self):
        """
        :return: The new expanded data including x, y, heading, velocity, and yaw rate
        """
        return self.newData

    def dist(self, point1, point2):
        """
        Get distance between 2 points
        :type point1: list
        :type point2: list
        :return:
        """
        x1, y1 = point1[0], point1[1]
        x2, y2 = point2[0], point2[1]
        return sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

    def angle_trunc(self, a):
        """
        Make sure the angle remains reasonable
        :type a: float
        :return: truncated angle
        """
        while a < 0.0:
            a += pi * 2
        return ((a + pi) % (pi * 2)) - pi

    def updateAll(self):
        """
        Run through all data and find the walls.
        Add heading, velocity and yaw rate to all points.
        :return: None
        """
        for i in xrange(1, len(self.data)):
            # first, find the walls
            if self.data[i][0] != -1:
                self.minX = min(self.minX, self.data[i][0])
            self.maxX = max(self.maxX, self.data[i][0])
            if self.data[i][1] != -1:
                self.minY = min(self.minY, self.data[i][1])
            self.maxY = max(self.maxY, self.data[i][1])

            # Don't throw away bad data (-1,-1) because it's still a timestep and we should account for all time steps
            if self.data[i][0] == -1 and self.data[i][1] == -1:
                # bad data points so just used the heading, velocity, and yaw rate from previous point
                previousHeading, previousVelocity, previousYawRate = self.newData[i-1][2:]
                self.velocities.append( previousVelocity )
                self.newData.append( [ self.data[i][0], self.data[i][1], previousHeading, previousVelocity, previousYawRate ] )
            else:
                # get heading, velocity and angular velocity then append to new data
                heading = self.angle_trunc( atan2(self.data[i][1] - self.data[i-1][1], self.data[i][0] - self.data[i-1][0]) )
                velocity = self.dist(self.data[i], self.data[i-1])
                self.velocities.append( velocity )
                yawRate = heading - self.newData[i-1][2]
                self.newData.append( [ self.data[i][0], self.data[i][1], heading, velocity, yawRate ] )



   