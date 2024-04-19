#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Pose, PointStamped, PoseStamped, PoseArray
from std_msgs.msg import Bool, String, Float32, UInt8
from tf.transformations import euler_from_quaternion
import math
import numpy as np
import time
from itertools import combinations
import time as timer
import pickle
import copy
import heapq
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
class MAPF:
    def __init__(self):
        self.num_of_agents=3
        self.starts=[(-33.48821258544922,-9.2613845147593956),(-24.13109588623047,-0.3281878924291778),(2.989964485168457,5.2117018699646)]
        self.goals=[(-21,-1),(-5,-1),(-14,0)]
        self.heuristics = []
        self.used = []
        # Subscribers and Publishers
        self.astar_path=rospy.Publisher('/cmu_rc1/path_topic', Path, queue_size=10)
        self.astar_path2=rospy.Publisher('/cmu_rc2/path_topic', Path, queue_size=10)
        self.astar_path3=rospy.Publisher('/cmu_rc3/path_topic', Path, queue_size=10)
        self.convoy_switch_sub = rospy.Subscriber(
            "/cmu_rc1/odom_to_base_link", Odometry, self.Odom1CB
        )
        self.convoy_switch_sub = rospy.Subscriber(
            "/cmu_rc2/odom_to_base_link", Odometry, self.Odom2CB
        )
        self.convoy_switch_sub = rospy.Subscriber(
            "/cmu_rc3/odom_to_base_link", Odometry, self.Odom3CB
        )
        self.reache_wpts_pub = rospy.Publisher(
            "/shit", PoseArray, queue_size=5)
        self.reache_wpts_pub2 = rospy.Publisher(
            "/shit2", PoseArray, queue_size=5)
        self.reache_wpts_pub3 = rospy.Publisher(
            "/shit3", PoseArray, queue_size=5)
        # self.convoy_sdwitch_sub = rospy.Subscriber(
        #     "/cmu_rc2/odom_to_base_link", Odometry, self.convoySwitchCallback2
        # )
        useless = [
            96,
            97,
            98,
            99,
            100,
            101,
            102,
            103,
            104,
            105,
            106,
            107,
            108,
            109,
            110,
            111,
            112,
            113,
            114,
            115,
            116,
            121,
            118,
            123,
            124,
            125,
            126,
            127,
        ]
        all_paths=np.load('/home/developer/mmpug_ws/src/mmpug_autonomy/mmpug_nav_layer/local_planner/scripts/mapf_rc.npy')
        all_rel_paths= []
        for i in range(all_paths.shape[0]):
            if (all_paths[i][-1] == 2): 
                all_rel_paths.append(all_paths[i])
        all_rel_paths_ids=[]
        for i in range(len(all_rel_paths)):
            if (all_rel_paths[i][4] not in all_rel_paths_ids):
                # if (all_rel_paths[i][4] in useless):
                #     continue
                all_rel_paths_ids.append(all_rel_paths[i][4])
        # print("number of relative self.paths: ", len(all_rel_paths_ids))
        self.path_dict={}
        for i in range(len(all_rel_paths_ids)):
            self.path_dict[all_rel_paths_ids[i]] = []
        for i in range(len(all_rel_paths)):
            if (all_rel_paths[i][4] in self.path_dict.keys()):
                self.path_dict[all_rel_paths[i][4]].append(all_rel_paths[i])
        self.paths={}
        for i in range(len(all_rel_paths_ids)):
            if self.path_dict[all_rel_paths_ids[i]] !=[]:
                self.paths[i] = path(self.path_dict[all_rel_paths_ids[i]])
        self.used=[]
        for i in range(len(all_rel_paths)):
            if (all_rel_paths[i][4] in self.path_dict.keys()):
                # if (all_rel_paths[i][4] in useless):
                #     continue
                self.used.append(all_rel_paths[i][4])
                self.path_dict[all_rel_paths[i][4]].append(all_rel_paths[i])
        self.used=np.unique(self.used)
        self.nodes1={}
        self.nodes2={}
        self.nodes3={}
        with open('/home/developer/mmpug_ws/src/mmpug_autonomy/mmpug_nav_layer/local_planner/scripts/obstacles2.pkl', 'rb') as file:
            obstacles = pickle.load(file)
        x_coords, y_coords = zip(*obstacles)
        self.conmap1={}
        self.conmap2={}
        self.directions={}
        # for i in range(4):
        #     self.directions[i]=0
        self.node_ids1=[]
        self.node_ids2=[]
        self.node_ids3=[]
        self.node_counter=0
        
        self.current_ids=[]
        self.next_ids=[]
        self.grid={}
        xc=[]
        yc=[]
        for i in range(-600, 600):
            for j in range(-600, 600):
                self.grid[(i, j)]=loc(i, j) 
        for i in range(len(x_coords)):
            self.grid[(int(x_coords[i]*10), int(y_coords[i]*10))].occupancy=100
        # self.node_ids1 = []
        # self.node_ids2 = []
        self.node_counter = 0
        self.direction_counter=[]
        self.current_ids = []
        self.next_ids = []
        # self.contch_sub = rospy.Subscriber("/pubtreaj", Bool, self.soemt)
        # Variables
        self.flag = True
        self.odom1 = Odometry()
        self.odom2 = Odometry()
        self.odom3 = Odometry()
        self.has_odom1 = False
        self.has_odom2 = False
        self.has_odom3 = False
        self.update_hz=1

    def Odom1CB(self,msg):
        self.odom1=msg
        self.has_odom1=True
    
    def Odom2CB(self,msg):
        self.odom2=msg
        self.has_odom2=True
    def Odom3CB(self,msg):
        self.odom3=msg
        self.has_odom3=True

    # def move(self,loc,parent,id):
    #     # print(len(self.used))
    #     self.directions[0]=node()
    #     self.directions[0].x=loc[0]
    #     self.directions[0].y=loc[1]
    #     self.directions[0].yaw=parent["dir"]
    #     self.directions[0].id=0
    #     self.directions[0].parent=parent
    #     self.current_ids=[]
    #     self.next_ids=[0]
    #     self.node_counter=0
    #     self.direction_counter.append(0)
    #     self.next_ids=[]
    #     # print('directions',self.directions)
    #     rad=self.paths[id].r
    #     theta=self.paths[id].theta
    #     xnew=loc[0]+rad*math.cos(theta+parent["dir"])
    #     ynew=loc[1]+rad*math.sin(theta+parent["dir"])
    #     theta_new=self.paths[id].orientation+parent["dir"]
    #     # print('xnew and ynew',xnew,ynew)
    #     if (self.grid[(int(xnew*10),int(ynew*10))].occupancy==100):
    #         return

    #     return xnew, ynew,theta_new
    
    def compute_heuristics(self,pos, goal):
        return math.sqrt(pow(abs(goal[0] - pos[0]),2) + pow(abs(goal[1] - pos[1]),2))
    
    def valid_path(self,path_id,node_id,nodes):
        # print('path id',path_id)
        for i in range(len(self.paths[path_id].path)):
            rad=math.sqrt((self.paths[path_id].path[i][0])**2+(self.paths[path_id].path[i][1])**2)
            theta=math.atan2(self.paths[path_id].path[i][1],self.paths[path_id].path[i][0])
            xnew=nodes[node_id].x+rad*math.cos(theta+nodes[node_id].yaw)
            ynew=nodes[node_id].y+rad*math.sin(theta+nodes[node_id].yaw)
            if (self.grid[(int(xnew*10),int(ynew*10))].occupancy==100):
                return False
        return True
    
    def draw_children(self,node_id,node_ids,nodes):
        node_counter = node_ids[-1]
        for i in range(len(self.used)):
            node_counter+=1
            if (not self.valid_path(i,node_id,nodes)):
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
        r.sleep()
        while not rospy.is_shutdown():
            r.sleep()
            print('Inside Run')
            all_paths=[]
            if self.flag and self.has_odom1 and self.has_odom2:
            # if self.has_odom1 and self.flag:
                self.publist()
            elif self.flag:
                continue
            else:
                continue
            for i in range(self.num_of_agents):
                if i==0:
                    path = self.a_star(
                        self.starts[i],
                        self.goals[i],
                        self.nodes1
                    )
                    if path==None:
                        print("no path 1")
                        exit()
                elif i==1:
                    print("agent 2")
                    path = self.a_star(
                        self.starts[i],
                        self.goals[i],
                        self.nodes2
                    )
                    if path==None:
                        print("no path 2")
                        exit()
                else:
                    print("agent 3")
                    path = self.a_star(
                        self.starts[i],
                        self.goals[i],
                        self.nodes3)
                    if path==None:
                        print("no path 3")
                        exit()
                # print(self.nodes3.keys())
                all_paths.append(path)

                
            print('PATH', all_paths)

            if not path:
                print("No path found, continuing...")
                r.sleep()
                continue 

            for i in range(self.num_of_agents):
                if i==0:
                    continue
                
                if i==1:
                    temp=[]
                    for j in range(len(all_paths[i])):
                        temp.append(all_paths[i][j])

                        if j>=len(all_paths[i-1]):
                            continue
                        print(self.disttance_pt(all_paths[i][j][0],all_paths[i-1][j][0],all_paths[i][j][1],all_paths[i-1][j][1]),i)
                        if (self.disttance_pt(all_paths[i][j][0],all_paths[i-1][j][0],all_paths[i][j][1],all_paths[i-1][j][1])<5):
                            print("rohan lawda hai###########################")
                            temp.append(temp[-1])
                    all_paths[i]=temp
                else:
                    temp=[]
                    for j in range(len(all_paths[i])):
                        temp.append(all_paths[i][j])

                        if j>=len(all_paths[0]):
                            continue
                        print(self.disttance_pt(all_paths[i][j][0],all_paths[0][j][0],all_paths[i][j][1],all_paths[0][j][1]),i)
                        if (self.disttance_pt(all_paths[i][j][0],all_paths[0][j][0],all_paths[i][j][1],all_paths[0][j][1])<5):
                            print("rohan harami hai!!!!!!!!!!!!!!!!!!!!!!!!")
                            temp.append(temp[-1])
                    all_paths[i]=temp
                    temp=[]
                    for j in range(len(all_paths[i])):
                        temp.append(all_paths[i][j])

                        if j>=len(all_paths[1]):
                            continue
                        print(self.disttance_pt(all_paths[i][j][0],all_paths[1][j][0],all_paths[i][j][1],all_paths[1][j][1]),i)
                        if (self.disttance_pt(all_paths[i][j][0],all_paths[1][j][0],all_paths[i][j][1],all_paths[1][j][1])<5):
                            print("rohan chutiya hai@@@@@@@@@@@@@@@@@@@@@@@@")
                            temp.append(temp[-1])
                    all_paths[i]=temp


            pub_path = Path()
            pub_path.header.frame_id = "global"
            pub_path.header.stamp = rospy.Time.now()

            for position in all_paths[0]:
                pose = PoseStamped()
                pose.header.frame_id = "global"
                pose.header.stamp = rospy.Time.now()
                pose.pose.position.x = position[0]
                pose.pose.position.y = position[1]
                pose.pose.orientation.w = 1.0
                pub_path.poses.append(pose)

            self.astar_path.publish(pub_path)

            pub_path = Path()
            pub_path.header.frame_id = "global"
            pub_path.header.stamp = rospy.Time.now()

            for position in all_paths[1]:
                pose = PoseStamped()
                pose.header.frame_id = "global"
                pose.header.stamp = rospy.Time.now()
                pose.pose.position.x = position[0]
                pose.pose.position.y = position[1]
                pose.pose.orientation.w = 1.0
                pub_path.poses.append(pose)
            self.astar_path2.publish(pub_path)

            # print("path1",all_paths[0])
            # print("path2",all_paths[1])
           
            pub_path = Path()
            pub_path.header.frame_id = "global"
            pub_path.header.stamp = rospy.Time.now()

            for position in all_paths[2]:
                pose = PoseStamped()
                pose.header.frame_id = "global"
                pose.header.stamp = rospy.Time.now()
                pose.pose.position.x = position[0]
                pose.pose.position.y = position[1]
                pose.pose.orientation.w = 1.0
                pub_path.poses.append(pose)
            self.astar_path3.publish(pub_path)

            print("FINALLLL", all_paths[0])
            print("FINALLLL2", all_paths[1])
            print("FINALLLL3", all_paths[2])
            print("Path published successfully")


    def publist(self):
        start = time. time()
        self.nodes1[0]=node()
        self.nodes1[0].x=self.odom1.pose.pose.position.x
        self.nodes1[0].y=self.odom1.pose.pose.position.y
        self.nodes1[0].yaw=euler_from_quaternion([self.odom1.pose.pose.orientation.x,self.odom1.pose.pose.orientation.y,self.odom1.pose.pose.orientation.z,self.odom1.pose.pose.orientation.w])[2]
        self.nodes1[0].id=0
        self.nodes1[0].parent=None
        self.current_ids=[]
        self.next_ids=[0]
        self.node_counter=0
        self.node_ids1.append(0)
        for i in range(6):
            self.current_ids=self.next_ids
            self.next_ids=[]
            for j in range(len(self.current_ids)):
                self.node_ids1.append(self.draw_children(self.current_ids[j],self.node_ids1,self.nodes1))



        end = time. time()
        duration = end - start
        print("Time: {} seconds". format(round(duration, 3)))
        self.nodes2[0]=node()
        self.nodes2[0].x=self.odom2.pose.pose.position.x
        self.nodes2[0].y=self.odom2.pose.pose.position.y
        self.nodes2[0].yaw=euler_from_quaternion([self.odom2.pose.pose.orientation.x,self.odom2.pose.pose.orientation.y,self.odom2.pose.pose.orientation.z,self.odom2.pose.pose.orientation.w])[2]
        self.nodes2[0].id=0
        self.nodes2[0].parent=None
        self.current_ids=[]
        self.next_ids=[0]
        self.node_counter=0
        self.node_ids2.append(0)
        for i in range(6):
            self.current_ids=self.next_ids
            self.next_ids=[]
            for j in range(len(self.current_ids)):
                self.node_ids2.append(self.draw_children(self.current_ids[j],self.node_ids2,self.nodes2))

        self.nodes3[0]=node()
        self.nodes3[0].x=self.odom3.pose.pose.position.x
        self.nodes3[0].y=self.odom3.pose.pose.position.y
        self.nodes3[0].yaw=euler_from_quaternion([self.odom3.pose.pose.orientation.x,self.odom3.pose.pose.orientation.y,self.odom3.pose.pose.orientation.z,self.odom3.pose.pose.orientation.w])[2]
        self.nodes3[0].id=0
        self.nodes3[0].parent=None
        self.current_ids=[]
        self.next_ids=[0]
        self.node_counter=0
        self.node_ids3.append(0)
        for i in range(6):
            self.current_ids=self.next_ids
            self.next_ids=[]
            for j in range(len(self.current_ids)):
                self.node_ids3.append(self.draw_children(self.current_ids[j],self.node_ids3,self.nodes3))


        # for i in self.nodes1.keys():
        #     self.conmap1[i]=[]
        #     for j in self.nodes2.keys():
        #         if (math.sqrt((self.nodes1[i].x-self.nodes2[j].x)**2+(self.nodes1[i].y-self.nodes2[j].y)**2)<1):
        #             self.conmap1[i].append(j)
        # for i in self.nodes2.keys():
        #     self.conmap2[i]=[]
        #     for j in self.nodes1.keys():
        #         if (math.sqrt((self.nodes1[j].x-self.nodes2[i].x)**2+(self.nodes1[j].y-self.nodes2[i].y)**2)<1):
        #             self.conmap2[i].append(j)
            




        shit = PoseArray()
        shit.header.frame_id="global"
        shit.header.stamp=rospy.Time.now()
        for j in self.nodes1.keys():
            some=Pose()
            # print(self.nodes1[j].x)
            some.position.x=self.nodes1[j].x
            some.position.y=self.nodes1[j].y
            shit.poses.append(some)
        
        self.reache_wpts_pub.publish(shit)


        shit = PoseArray()
        shit.header.frame_id="global"
        shit.header.stamp=rospy.Time.now()
        for j in self.nodes2.keys():
            some=Pose()
            # print(self.nodes1[j].x)
            some.position.x=self.nodes2[j].x
            some.position.y=self.nodes2[j].y
            shit.poses.append(some)
        
        self.reache_wpts_pub2.publish(shit)


        shit = PoseArray()
        shit.header.frame_id="global"
        shit.header.stamp=rospy.Time.now()
        for j in self.nodes3.keys():
            some=Pose()
            # print(self.nodes1[j].x)
            some.position.x=self.nodes3[j].x
            some.position.y=self.nodes3[j].y
            shit.poses.append(some)
        
        self.reache_wpts_pub3.publish(shit)
        self.flag=False

    def wrap2pi(self, yaw):
        if yaw > math.pi:
            yaw -= 2 * math.pi
        if yaw <= -math.pi:
            yaw += 2 * math.pi
        return yaw
    def disttance_pt(self,x1,x2,y1,y2):
        return math.sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2))

    
    def push_node(self,open_list, node):
        heapq.heappush(
            open_list, (node["g_val"] + node["h_val"], node["h_val"], node["loc"], node)
        )
    def pop_node(open_list):
        _, _, _, curr = heapq.heappop(open_list)
        return curr
    
    def build_constraint_table(constraints, agent):
        table={}
        max_t=0
        for constraint in constraints:
            if constraint['agent'] == agent:
                if constraint['timestep'] not in table:
                    table[constraint["timestep"]]=[]
                table[constraint["timestep"]].append(constraint)
                if max_t<constraint["timestep"]:
                    max_t=constraint["timestep"]
        return table,max_t
    def in_map(self,map, loc):
        # print('loc',loc[0],len(map))
        if loc[0] >= len(map) or loc[1] >= len(map[0]) or min(loc) < 0:
            return False
        else:
            return True
    def compare_nodes(self,n1, n2):
        return n1["g_val"] + n1["h_val"] < n2["g_val"] + n2["h_val"]
    
    def a_star(self, start_loc, goal_loc,nodes):
        open_list = []
        closed_list = dict()
        h_value = self.compute_heuristics([nodes[0].x,nodes[0].y],goal_loc)
        root = {
            "loc": tuple([nodes[0].x,nodes[0].y]),
            "g_val": 0,
            "h_val": h_value,
            "parent": None,
            "timestep": 0,
            "node_id":0
        }
        heapq.heappush(open_list, (root["g_val"] + root["h_val"], root["h_val"], root["loc"], root))
        closed_list[((root["loc"], root["timestep"]))] = root
        while len(open_list) > 0:
            _, _, _, curr = heapq.heappop(open_list)
            if self.compute_heuristics(curr["loc"],goal_loc)<3.0:
                # print("************************")
                return self.get_path(curr)
            # print(curr["loc"]," num child ",len(nodes[curr["node_id"]].children))
            for i in range(len(nodes[curr["node_id"]].children)):
                # print('i******',i)
                child_loc = [nodes[nodes[curr["node_id"]].children[i]].x,nodes[nodes[curr["node_id"]].children[i]].y]
                child_timestep = curr["timestep"] + 1
                # print('child**********',child_loc[0],child_loc[1])
                child = {
                    "loc": [child_loc[0],child_loc[1]],
                    "g_val": curr["g_val"] + 1,
                    "h_val": self.compute_heuristics(child_loc,goal_loc),
                    "parent": curr,
                    "timestep": child_timestep,
                    "node_id":nodes[curr["node_id"]].children[i]
                }
                # print('hval********',self.compute_heuristics(child_loc,goal_loc))
                if (tuple(child["loc"]), child["timestep"]) in closed_list:
                    existing_node = closed_list[tuple(child["loc"]), child["timestep"]]
                    if self.compare_nodes(child, existing_node):
                        closed_list[tuple(child["loc"]), child["timestep"]] = child
                        heapq.heappush(open_list, (child["g_val"] + child["h_val"], child["h_val"], child["loc"], child))
                else:
                    closed_list[tuple(child["loc"]), child["timestep"]] = child
                    heapq.heappush(open_list, (child["g_val"] + child["h_val"], child["h_val"], child["loc"], child))
        return None 
    
    def get_path(self,goal_node):
        path = []
        curr = goal_node
        while curr is not None:
            temp=list(curr["loc"])
            temp.append(curr["node_id"])
            path.append(temp)
            curr = curr["parent"]
        path.reverse()
        return path
if __name__ == "__main__":
    rospy.init_node("convoy_formup")
    tf_manager = MAPF()
    tf_manager.run()
