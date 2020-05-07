import rospy
from sensor_msgs.msg import LaserScan
import time
import math
from angle import angle

class avoid:

    def __init__(self):

        self.sub2 = rospy.Subscriber('/laser/scan', LaserScan, self.o_void)
#        rospy.init_node('obstacle_values')
        self.rate = rospy.Rate(10)
        self.get = 0
        self.turn = 0
        self.word = " "
        self.drone_angle = 0

    def drone_lidar(self,t):
        tnew = t
        if 0<=t<180:
            tnew = 180 + t
        elif 180<=t<360:
            tnew = t - 180
        return tnew

    def o_void(self, msg):

        o1 = 0
        o2 = 0
        a = angle()
        ang_d = int(a.get_angle())
        self.drone_angle = ang_d
        ang = self.drone_lidar(ang_d)

#        if 171 <= ang <= 359:
#            ang = 540 - ang
#        elif 0 <= ang <= 180:
#            ang = 180 - ang
#        print("Drone: " +str(ang_d))
#        print("LIDAR: " +str(ang))
#        print("Dist.: " + str(msg.ranges[ang]))
#        print(" ")
#        print(" ")



        if msg.ranges[ang] <=35:
            for i in range(ang, 0, -1):
                if i<0:
                    j = 360 - i
                else:
                    j = i
                if msg.ranges[j] > 35 or msg.ranges[j] == float("inf"):
                    o1 = j
                    break

            for i in range(ang, 360):
                if i>359:
                    j = 360+i
                else:
                    j = i
                if msg.ranges[j] > 35 or msg.ranges[j] == float("inf"):
                    o2 = j
                    break


            if abs(90-o1) <= abs(90-o2):
                self.turn =  (o1-10)
                self.word = "right"
            else:
                self.turn =  (o2+10)
                self.word = "left"

        else:
            self.turn = ang_d
            self.word = "straight"
        self.get = 1

    def get_turn(self):
        while True:
            while True:
                if self.get != 0:
                    break
            return self.turn, self.word, self.drone_angle
            self.get = 0

if __name__ == "__main__":

    while True:
        ob = avoid()
        o = ob.get_turn()
        print(o)
