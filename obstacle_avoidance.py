import rospy
from sensor_msgs.msg import LaserScan
import time
import math
from angle import angle

class obstacle_avoidance:

    def __init__(self):

        self.sub2 = rospy.Subscriber('/laser/scan', LaserScan, self.o_void)
#        rospy.init_node('obstacle_values')
        self.rate = rospy.Rate(10)
        self.get = 0
        self.turn = ""

    def o_void(self, msg):

        o1 = 0
        o2 = 0
        a = angle()
        an = a.get_angle()
        ang  = an.angle
        if msg.ranges[ang] <20:
            for i in range(ang, ang-20, -1):

                if msg.ranges[i] > 20:
                    o1 = i-1
                    break

            for i in range(ang, ang+20):
                if msg.ranges[i] > 20:
                    o2 = i-1
                    break

            if 90-o1 < 90-o2:
                self.turn =  "right"
            else:
                self.turn =  "left"
        else:
            turn = "straight"
        self.get = 1


    def get_turn(self):
        while True:
            while True:
                if self.get != 0:
                    break
            return self.turn
            self.get = 0

if __name__ == "__main__":

    ob = get_turn.obstacle_avoidance()
    print(ob)
