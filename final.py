#!/usr/bin/env python3
import rospy
from mmpug_msgs.msg import controller_path, controller_state
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Pose, PointStamped, PoseStamped, PoseArray
from std_msgs.msg import Bool, String, Float32, UInt8
from tf.transformations import euler_from_quaternion
import math
import numpy as np
from itertools import combinations


class robot:
    def __init__(self,syst_id):
        # Params
        self.system_id = syst_id

        # Subscribers and Publishers
        self.convoy_switch_sub = rospy.Subscriber("/"+self.system_id+"/odom_to_base_link", Odometry, self.odomcb)
        self.p1sub = rospy.Subscriber("/"+self.system_id+"/path_topic", Path, self.p1cb)

        self.wp_pub = rospy.Publisher("/"+self.system_id+"/mux/goal_input", PoseArray, queue_size=5)
        self.mode_pub=rospy.Publisher("/"+self.system_id+"/command_interface/mode",UInt8,queue_size=5)

        self.odom = Odometry()
        self.has_odom = False
        self.path1=PoseArray()
        self.has_path1=False

        self.has_time=False 
        self.time=0
        self.current_wp=0
        self.has_next_wp=False
        self.reached=False
        self.perm_reached=False 

        self.init=False

        self.to_be_sent=PoseArray()
        self.to_be_sent.header.frame_id='global'


    
    def p1cb(self, msg):
        self.has_path1=True
        for i in msg.poses:
            temp=Pose()
            temp=i.pose
            self.path1.poses.append(temp)
        
        self.to_be_sent.poses.append(self.path1.poses[0])


    def odomcb(self, msg):
        self.odom = msg
        self.has_odom = True

        if self.init:
            if (math.sqrt((self.odom.pose.pose.position.x-self.path1.poses[self.current_wp-1].position.x)**2+(self.odom.pose.pose.position.y-self.path1.poses[self.current_wp-1].position.y)**2)<0.3):
                self.reached=True
                if self.current_wp==len(self.path1.poses)-1:
                    self.perm_reached=True


    def send_wp(self):
        if (len(self.to_be_sent.poses))==0:
            return
        self.to_be_sent.header.stamp=rospy.Time.now()
        self.wp_pub.publish(self.to_be_sent)
        self.to_be_sent.poses.clear()
        if self.current_wp==len(self.path1.poses)-1:
            self.has_next_wp=False
            return
        self.has_next_wp=True
        self.current_wp+=1
        self.to_be_sent.poses.append(self.path1.poses[self.current_wp])
        mode=UInt8()
        mode.data=3
        self.mode_pub.publish(mode)

    

class ConvoyOrder():
    def __init__(self):
        # Params
        self.update_hz = rospy.get_param("~update_hz", 3)
        self.bst_id = rospy.get_param("~bst_id", "basestation")

        # Subscribers and Publishers
        self.convoy_switch_sub = rospy.Subscriber("/cmu_rc1/odom_to_base_link", Odometry, self.convoySwitchCallback)
        self.convoy_switch_sub2 = rospy.Subscriber("/cmu_rc2/odom_to_base_link", Odometry, self.convoySwitchCallback2)
        self.convoy_switch_sub3 = rospy.Subscriber("/cmu_rc3/odom_to_base_link", Odometry, self.convoySwitchCallback3)

        self.p1sub = rospy.Subscriber("/cmu_rc1/path_topic", Path, self.p1cb)
        self.p2sub = rospy.Subscriber("/cmu_rc2/path_topic", Path, self.p2cb)
        self.p3sub = rospy.Subscriber("/cmu_rc3/path_topic", Path, self.p3cb)

        self.robot_names = []
        self.robots = {}
        self.robots["cmu_rc1"] = robot("cmu_rc1")
        self.robots["cmu_rc2"] = robot("cmu_rc2")
        self.robots["cmu_rc3"] = robot("cmu_rc3")
        self.robot_names.append("cmu_rc1")
        self.robot_names.append("cmu_rc2")
        self.robot_names.append("cmu_rc3")
        


        # self.contch_sub = rospy.Subscriber(
        #     "/pubtreaj", Bool, self.soemt)

        # self.reached_init_wpts_pub = rospy.Publisher(
        #     "/mmpug_mt_002/local_planner/controller", controller_path, queue_size=5)
        # self.reache_wpts_pub = rospy.Publisher(
        #     "/shit", Path, queue_size=5)

        # Variables
        self.flag = True
        self.odom1= Odometry()
        self.odom2= Odometry()
        self.odom3= Odometry()
        self.has_odom1 = False
        self.has_odom2 = False
        self.has_odom3 = False

        self.path1=PoseArray()
        self.path2=PoseArray()
        self.path3=PoseArray()
        self.has_path1=False
        self.has_path2=False
        self.has_path3=False




    def p1cb(self, msg):
        self.has_path1=True
        for i in msg.poses:
            temp=Pose()
            temp=i.pose
            self.path1.poses.append(temp)

    def p2cb(self, msg):
        self.has_path2=True
        for i in msg.poses:
            temp=Pose()
            temp=i.pose
            self.path2.poses.append(temp)

    def p3cb(self, msg):
        self.has_path3=True
        for i in msg.poses:
            temp=Pose()
            temp=i.pose
            self.path3.poses.append(temp)

    def run(self):
        r = rospy.Rate(self.update_hz)

        while not rospy.is_shutdown():
            r.sleep()
            if self.has_odom1 and self.has_odom2 and self.has_odom3:
                if self.has_path1 and self.has_path2 and self.has_path3:
                    
                    if (self.flag):
                        self.pub_current_wpts()
                        self.flag=False
                        continue
                    if self.check_reached():
                        self.pub_current_wpts()
                        continue
                        
        
            
    def check_reached(self):
        skipped_names = []
        for i in self.robot_names:
            if self.robots[i].path1.poses[self.robots[i].current_wp] == self.robots[i].path1.poses[self.robots[i].current_wp-1]:
                print("found common", i)
                if not self.robots[i].has_time:
                    self.robots[i].time = rospy.get_time()
                    self.robots[i].has_time = True
                    skipped_names.append(i)
                elif self.robots[i].has_time:
                    print("time ",rospy.get_time() - self.robots[i].time)
                    if rospy.get_time() - self.robots[i].time > 2:
                        self.robots[i].reached = True
                        self.robots[i].time=0
                        self.robots[i].has_time=False
                    else:
                        skipped_names.append(i)
        count=0
        for i in self.robot_names:
            if i in skipped_names:
                return False
            if self.robots[i].reached or self.robots[i].perm_reached:
                count=count+1
        if count==len(self.robot_names):
            return True
        return False

    def pub_current_wpts(self):
        for i in self.robot_names:
            self.robots[i].send_wp()
            self.robots[i].init=True
            self.robots[i].reached=False


    def publist(self):
        some=2


    def wrap2pi(self, yaw):
        if yaw > math.pi:
            yaw -= 2*math.pi
        if yaw <= -math.pi:
            yaw += 2*math.pi
        return yaw

    def convoySwitchCallback(self, msg):
        self.odom1 = msg
        self.has_odom1 = True
    
    def convoySwitchCallback2(self, msg):
        self.odom2 = msg
        self.has_odom2 = True
    
    def convoySwitchCallback3(self, msg):
        self.odom3 = msg
        self.has_odom3 = True
    
    def soemt(self,msg):
        self.flag=True

    def convoyFormupSwitchCallback(self, msg):
        self.convoy_formup_switch = msg
        self.cancel_formup = False

    def convoyFormupCallback(self, msg):
        self.cancel_formup = not msg.data

    def convoyPosCallback(self, convoy_position):
        self.convoy_pos_arr = convoy_position

        for i in range(0, len(convoy_position.convoy)):
            if convoy_position.convoy[i].system_id not in self.robot_names:
                syst_id = convoy_position.convoy[i].system_id
                self.robots[syst_id] = ConvoyBotManager(syst_id, self.bst_id)
                self.robot_names.append(syst_id)
                self.robot_names.sort()


if __name__ == "__main__":
    rospy.init_node("convoy_formup")
    tf_manager = ConvoyOrder()
    # tf_manager.dominique()
    # tf_manager.burhan()
    tf_manager.run()