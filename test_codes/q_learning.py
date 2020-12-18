import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TwistStamped
import time
import math
import numpy as np
import random

class q_exp:

    def __init__(self):

        self.sub = rospy.Subscriber('/laser/scan', LaserScan, self.front_exp)
        rospy.init_node('obstacle_values')
        self.rate = rospy.Rate(10)
        self.front = 0
        self.front_length = []
        self.prob = 0.7

    def q_exp(self, msg):

        frontiers = []
        t = 0

        for i in range(360):
           if msg.ranges[i] == float("inf"):
               t = t+1
               if msg.ranges[i+1] >= 0 :
                   frontiers.append(i - t/2)
                   t = 0
        for i in range(1, len(frontiers),2):
            front_length = frontiers[i]

        if random.random() < self.prob:
            self.front = np.amax(np.array(front_length))
        else:
            self.front = front_length[random.randrange(0,len(front_length))

        return self.front


