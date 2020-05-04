import rospy
from mavros_msgs.msg import State, PositionTarget
from geometry_msgs.msg import PoseStamped, Point, Quaternion, TransformStamped, PoseArray
import math
import time
from std_msgs.msg import String, Header
from obstacle_avoidance import obstacle_avoidance
from angle import angle


class OffbPosCtl:
    curr_pose = PoseStamped()
    waypointIndex = 0
    distThreshold= 2
    des_pose = PoseStamped()
    saved_location = None
    isReadyToFly = False
    hover_loc = [4,0,10,0,0,0,0]
    mode="SURVEY"
    vel_pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=10)
    pub = rospy.Publisher('/data', String, queue_size=10)

    def __init__(self):
        rospy.init_node('offboard_test', anonymous=True)
        self.loc = []
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, callback= self.mocap_cb)
        rospy.Subscriber('/mavros/state',State, callback= self.state_cb)
        rospy.Subscriber('/dreams/state',String,callback=self.update_state_cb)

        self.nohit()

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

    def state_cb(self,msg):
        if msg.mode == 'OFFBOARD':
            #print msg
            #print("The mode is OFFBOARD")
            self.isReadyToFly = True
        else:
            #print("I am in state_cb")
            #print msg
            pass

    def update_state_cb(self,data):
        self.mode= data.data


    def nohit(self):
        print("I am in no hitting")
        while True:
            #destination point
            dev = [45,45,20,0,0,0,0]

            rate = rospy.Rate(10)
            self.des_pose = self.copy_pose(self.curr_pose)
            shape = len(self.loc)
            pose_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
            print self.mode
            t = 0
            currs = angle()
            turn = obstacle_avoidance()
            while self.mode == "SURVEY" and not rospy.is_shutdown():

                curr = currs.get_angle()

                if t < 10:
                    print("basic")
                    self.loc = [5,5,20,0,0,0,0]
                    t += 1


                elif turn.get_turn() == "straight":
                    print(turn.get_turn())
                    self.loc = dev
                else:
                    print(turn.get_turn())
                    vx = dev[0] - curr.x
                    vy = dev[1] - curr.y
                    vx = vx/math.sqrt(int(vx)^2 +int(vy)^2)
                    vy = vy/math.sqrt(int(vx)^2 +int(vy)^2)
                    ux_r = 1
                    ux_l = -1
                    uy_r = ux_r * vx/vy
                    uy_l = ux_l * vx/vy
                    mag_r = math.sqrt(ux_r^2 + uy_r^2)
                    mag_l = math.sqrt(ux_l^2 + uy_l^2)
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

                    right = [ux_r/mag_r, uy_r/mag_r,20,0,0,0,0]
                    left = [ux_l/mag_l, uy_l/mag_l,20,0,0,0,0]

                    if turn.get_turn() == "right":
                        self.loc += right
                    if turn.get_turn() == "left":
                        self.loc += left

                if self.waypointIndex == shape :
                    self.waypointIndex = 1                  # resetting the waypoint index

                if self.isReadyToFly:
                    des_x = self.loc[0]
                    des_y = self.loc[1]
                    des_z = self.loc[2]
                    self.des_pose.pose.position.x = des_x
                    self.des_pose.pose.position.y = des_y
                    self.des_pose.pose.position.z = des_z
                    self.des_pose.pose.orientation.x = self.loc[3]
                    self.des_pose.pose.orientation.y = self.loc[4]
                    self.des_pose.pose.orientation.z = self.loc[5]
                    self.des_pose.pose.orientation.w = self.loc[6]
                    curr_x = self.curr_pose.pose.position.x
                    curr_y = self.curr_pose.pose.position.y
                    curr_z = self.curr_pose.pose.position.z

                pose_pub.publish(self.des_pose)
                rate.sleep()


if __name__=='__main__':
    OffbPosCtl()

