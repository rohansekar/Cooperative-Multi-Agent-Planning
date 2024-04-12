#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Pose, PointStamped, PoseStamped, PoseArray
from std_msgs.msg import Bool, String, Float32, UInt8
from tf.transformations import euler_from_quaternion
import math
import numpy as np
from itertools import combinations

class path:
    def __init__(self, x):
        self.path = x
        self.orientation = math.atan2(x[-1][1]-x[-2][1], x[-1][0]-x[-2][0])
        self.theta = math.atan2(x[-1][1]-x[0][1], x[-1][0]-x[0][0])
        self.last_point = x[-1]
        self.r=math.sqrt((x[-1][0])**2+(x[-1][1])**2)
class node:
    def __init__(self):
        self.path = None
        self.x = None
        self.y = None
        self.yaw = None
        self.parent = None
        self.children = []
        self.id=0
class loc:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.occupancy = 0
        self.path_ids=[]

class ConvoyOrder():
    def __init__(self):
        

        # Subscribers and Publishers
        self.convoy_switch_sub = rospy.Subscriber(
            "/cmu_rc1/odom_to_base_link", Odometry, self.convoySwitchCallback1)
        
        self.convoy_sdwitch_sub = rospy.Subscriber(
            "/cmu_rc2/odom_to_base_link", Odometry, self.convoySwitchCallback2)
        
        useless = [96, 97, 98, 99, 100, 101,102,103,104,105,106,107,108, 109, 110, 111, 112, 113,114,115,116,121,118,123,124,125,126,127]
        all_paths=np.load('paths32.npy')
        all_rel_paths= []
        for i in range(all_paths.shape[0]):
            if (all_paths[i][-1] == 2): 
                all_rel_paths.append(all_paths[i])
        all_rel_paths_ids=[]
        for i in range(len(all_rel_paths)):
            if (all_rel_paths[i][3] not in all_rel_paths_ids):
                if (all_rel_paths[i][3] in useless):
                    continue
                all_rel_paths_ids.append(all_rel_paths[i][3])
        print("number of relative self.paths: ", len(all_rel_paths_ids))
        self.path_dict={}
        for i in range(len(all_rel_paths_ids)):
            self.path_dict[all_rel_paths_ids[i]] = []
        for i in range(len(all_rel_paths)):
            if (all_rel_paths[i][3] in self.path_dict.keys()):
                self.path_dict[all_rel_paths[i][3]].append(all_rel_paths[i])
        self.paths={}
        for i in range(len(all_rel_paths_ids)):
            if self.path_dict[all_rel_paths_ids[i]] !=[]:
                self.self.paths[all_rel_paths_ids[i]] = path(self.path_dict[all_rel_paths_ids[i]])
        self.used=[]
        for i in range(len(all_rel_paths)):
            if (all_rel_paths[i][3] in self.path_dict.keys()):
                if (all_rel_paths[i][3] in useless):
                    continue
                self.used.append(all_rel_paths[i][3])
                self.path_dict[all_rel_paths[i][3]].append(all_rel_paths[i])
        self.used=np.unique(self.used)
        self.nodes1={}
        self.nodes2={}

        self.node_ids1=[]
        self.node_ids2=[]
        self.node_counter=0

        
        self.current_ids=[]
        self.next_ids=[]

        

        self.contch_sub = rospy.Subscriber(
            "/pubtreaj", Bool, self.soemt)

        self.reache_wpts_pub = rospy.Publisher(
            "/shit", PoseArray, queue_size=5)

        # Variables
        self.flag = True
        self.odom1= Odometry()
        self.odom2=Odometry()
        self.has_odom1= False
        self.has_odom2 = False

    def valid_path(self,path_id,node_id,nodes):
        print('path id',path_id)
        for i in range(len(self.paths[path_id].path)):
            rad=math.sqrt((self.paths[path_id].path[i][0])**2+(self.paths[path_id].path[i][1])**2)
            theta=math.atan2(self.paths[path_id].path[i][1],self.paths[path_id].path[i][0])
            xnew=nodes[node_id].x+rad*math.cos(theta+nodes[node_id].yaw)
            ynew=nodes[node_id].y+rad*math.sin(theta+nodes[node_id].yaw)
            if (grid[(int(xnew),int(ynew))].occupancy==1):
                return False
        return True
        
    def draw_children(self,node_id,node_ids,nodes):
        node_counter = node_ids[-1]
        for i in range(len(self.used)):
            node_counter+=1
            if (not self.valid_path(i,node_id)):
                continue
            rad=self.paths[i].r
            theta=self.paths[i].theta
            # print("theta: ", theta)
            xnew=nodes[node_id].x+rad*math.cos(theta+nodes[node_id].yaw)
            ynew=nodes[node_id].y+rad*math.sin(theta+nodes[node_id].yaw)
            theta_new=self.paths[i].orientation+nodes[node_id].yaw
            nodes[node_counter]=node()
            nodes[node_counter].x=xnew
            nodes[node_counter].y=ynew
            nodes[node_counter].yaw=theta_new
            nodes[node_counter].id=node_counter
            nodes[node_counter].parent=node_id
            nodes[node_id].children.append(node_counter)
            node_ids.append(node_counter)
            self.next_ids.append(node_counter)
        return node_counter
    def run(self):
        r = rospy.Rate(self.update_hz)

        while not rospy.is_shutdown():

            # print(self.flag)
            if self.flag and self.has_odom1 and self.has_odom2:
                self.publist()
            r.sleep()



    def publist(self):
        self.nodes1[0]=node()
        self.nodes1[0].x=self.odom1.pose.pose.position.x
        self.nodes1[0].y=self.odom1.pose.pose.position.y
        self.nodes1[0].yaw=euler_from_quaternion([self.odom1.pose.pose.orientation.x,self.odom1.pose.pose.orientation.y,self.odom1.pose.pose.orientation.z,self.odom1.pose.pose.orientation.w])[2]
        self.nodes1[0].id=0
        self.nodes1[0].parent=None
        self.current_ids=[]
        self.next_ids=[0]
        self.node_counter=0
        for i in range(5):
            self.current_ids=self.next_ids
            self.next_ids=[]
            for j in range(len(self.current_ids)):
                self.node_ids1.append(self.draw_children(self.current_ids[j],self.node_ids1,self.nodes1))
                
        shit = PoseArray()
        shit.header.frame_id="cmu_rc1_odom"
        shit.header.stamp=rospy.Time.now()
        for j in self.nodes1.keys():
            some=Pose
            some.position.x=self.nodes1[i].x
            some.position.y=self.nodes1[i].y
            shit.pose.append(some)
        
        self.reache_wpts_pub.publish(shit)
        self.flag=False



    def wrap2pi(self, yaw):
        if yaw > math.pi:
            yaw -= 2*math.pi
        if yaw <= -math.pi:
            yaw += 2*math.pi
        return yaw

    def convoySwitchCallback1(self, msg):
        self.odom1 = msg
        self.has_odom1 = True
    def convoySwitchCallback2(self, msg):
        self.odom2 = msg
        self.has_odom2 = True

    def soemt(self,msg):
        self.flag=True

    def convoyFormupSwitchCallback(self, msg):
        self.convoy_formup_switch = msg
        self.cancel_formup = False

    def convoyFormupCallback(self, msg):
        self.cancel_formup = not msg.data



if __name__ == "__main__":
    rospy.init_node("convoy_formup")
    tf_manager = ConvoyOrder()
    tf_manager.run()