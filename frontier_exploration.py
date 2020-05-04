import rospy
from sensor_msgs.msg import LaserScan
import time
import math
import random
import numpy as np


class frontier_exp:


    def front_exp(self, msg):

#        print("in front_exp")
        self.frontiers = []
        t = 0

        for i in range(360):
            if msg.ranges[i] > 50:
                t = t+1
                if msg.ranges[i+1] < 50:
                    self.frontiers.append(i - t/2)
                    self.front_length.append(t)
                    t = 0
        self.front = self.front_length.index(np.amax(np.array(self.front_length)))
        self.front = self.frontiers[self.front]
        self.get = 1


    def __init__(self, v):

        rospy.init_node('obstacle_values')
        self.rate = rospy.Rate(10)
        self.frontiers = []
        self.front_length = []
        self.front = 0
        self.get = 0
        rospy.Subscriber('/laser/scan', LaserScan, self.front_exp)
        self.v = v

    def show(self):
#        print("in show")
#        print("All frontiers")
#        print(self.frontiers)
#        print("Choosen frontier")
#        print(self.front)
        return self.front
        self.get = 0


    def get_front(self):
        while True:
            while True:
                if self.get != 0:
                    break
            return self.front
            self.get = 0

if __name__ == "__main__":


    f = frontier_exp()
    frontier = f.get_front()
    print(frontier)
