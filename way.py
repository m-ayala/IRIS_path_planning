import rospy
import numpy as np
from numpy import asarray
from mavros_msgs.msg import State, PositionTarget
from geometry_msgs.msg import PoseStamped, Point, Quaternion, TransformStamped, PoseArray
from geometry_msgs.msg import TwistStamped
import math
import time
from std_msgs.msg import String, Header
import random

from angle import angle
from obstacle_avoidance import obstacle_avoidance



class OffbPosCtl:


    curr_pose = PoseStamped()
    waypointIndex = 0
    distThreshold= 2
    KP=0.005
    KD=0.0004
    KI=0.00005
    des_pose = PoseStamped()
    saved_location = None
    isReadyToFly = False
    hover_loc = [5,5,10,0,0,0,0]
    mode="HOVER"
    count = 1

    vel_pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=10)
    pub = rospy.Publisher('/data', String, queue_size=10)

    def __init__(self):
        rospy.init_node('offboard_test', anonymous=True)
        self.loc = []
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, callback= self.mocap_cb)
        rospy.Subscriber('/mavros/state',State, callback= self.state_cb)
        rospy.Subscriber('/dreams/state',String,callback=self.update_state_cb)
        self.hover()

    def copy_pose(self , pose):
        pt = pose.pose.position
        quat = pose.pose.orientation
        copied_pose = PoseStamped()
        copied_pose.header.frame_id = pose.header.frame_id
        copied_pose.pose.position = Point(pt.x, pt.y, pt.z)
        copied_pose.pose.orientation = Quaternion(quat.x , quat.y , quat.z , quat.w)
        return copied_pose

    def mocap_cb(self,msg1):
        self.curr_pose = msg1
        #print msg1

    def state_cb(self,msg):
        if msg.mode == 'OFFBOARD':
            #print msg
            #print("The mode is OFFBOARD")
            self.isReadyToFly = True
        else:
            #print("I am in state_cb")
            #print msg
            #print msg.mode
            pass

    def update_state_cb(self,data):
        self.mode= data.data
        #print self.mode


    def hover(self):
        location = self.hover_loc
        loc = [location,
               location,
               location,
               location,
               location,
               location,
               location,
               location,
               location]

        rate = rospy.Rate(10)
        shape = len(loc)
        pose_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size= 10)
        des_pose = self.copy_pose(self.curr_pose)
        waypoint_index = 0
        sim_ctr = 1
        #print("I am here")

        dev = [100,0,10,0,0,0,0]
        testy = 0
        currs = angle()
        turn = obstacle_avoidance()


        while self.mode=="HOVER" and not rospy.is_shutdown():


            ang = currs.get_angle()
            print(turn.get_turn())
            #print("Direction angle: " + str(ang))
            if testy < 50:
                #print("basic: " + str(testy))
                location = [5,0,10,0,0,0,0]
                testy = testy+1
                time.sleep(1)

            elif turn.get_turn() == "straight":
                mag = math.sqrt( (dev[0]-location[0])**2 + (dev[1]-location[1])**2 )
                loc = [(dev[0]-location[0])/mag + location[0],(dev[1]-location[1])/mag + location[1],10,0,0,0,0]
                location = loc


            else:
                vx = dev[0] - location[0]
                vy = dev[1] - location[1]
                vx = vx/math.sqrt(int(vx)**2 +int(vy)**2)
                vy = vy/math.sqrt(int(vx)**2 +int(vy)**2)
                ux_r = 1
                ux_l = -1
                uy_r = ux_r * vx/vy
                uy_l = ux_l * vx/vy
                mag_r = math.sqrt(int(ux_r)**2 + int(uy_r)**2)
                mag_l = math.sqrt(ux_l**2 + uy_l**2)
                angle_r = math.atan2(uy_r,ux_r)
                angle_l = math.atan2(uy_l,ux_l)
                if angle_r <0:
                   angle_r += angle_r + 360
                if angle_l <0:
                    angle_l += angle_l + 360
                if angle_r > angle_l:
                    t = ux_r
                    ux_r = ux_l
                    ux_l = t
                    t = uy_r
                    uy_r = uy_l
                    uy_l = t
                right = [ux_r/mag_r + location[0], uy_r/mag_r + location[1], 20,0,0,0,0]
                left = [ux_l/mag_l + location[0], uy_l/mag_l + location[1],20,0,0,0,0]

                if turn.get_turn() == "right":
                    location = right
                if turn.get_turn() == "left":
                    location = left

            #print("Location is: " + str(location))

            if waypoint_index==shape:
                waypoint_index = 0            # changing the way point index to 0
                sim_ctr = sim_ctr + 1
                print "HOVER STOP COUNTER:" + str(sim_ctr)
            if self.isReadyToFly:
                des_x = location[0]
                des_y = location[1]
                des_z = location[2]
                des_pose.pose.position.x = des_x
                des_pose.pose.position.y = des_y
                des_pose.pose.position.z = des_z
                des_pose.pose.orientation.x = location[3]
                des_pose.pose.orientation.y = location[4]
                des_pose.pose.orientation.z = location[5]
                des_pose.pose.orientation.w = location[6]
                curr_x = self.curr_pose.pose.position.x
                curr_y = self.curr_pose.pose.position.y
                curr_z = self.curr_pose.pose.position.z
            if sim_ctr == 50:
                pass

            for t in range(10000):
                pose_pub.publish(des_pose)
            rate.sleep()

    def get_descent(self,x,y,z):
        des_vel = PositionTarget()
        des_vel.header.frame_id = "world"
        des_vel.header.stamp=rospy.Time.from_sec(time.time())
        des_vel.coordinate_frame= 8
        des_vel.type_mask = 3527
        des_vel.velocity.x = x
        des_vel.velocity.y = y
        des_vel.velocity.z = z
        return des_vel


if __name__=='__main__':
    OffbPosCtl()
