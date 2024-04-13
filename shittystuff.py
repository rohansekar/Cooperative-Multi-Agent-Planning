#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Pose, PointStamped, PoseStamped, PoseArray
from std_msgs.msg import Bool, String, Float32, UInt8
from tf.transformations import euler_from_quaternion
import math
import numpy as np
from itertools import combinations
import time as timer
import pickle
import copy
import heapq


class path:
    def __init__(self, x):
        self.path = x
        self.orientation = math.atan2(x[-1][1] - x[-2][1], x[-1][0] - x[-2][0])
        self.theta = math.atan2(x[-1][1] - x[0][1], x[-1][0] - x[0][0])
        self.last_point = x[-1]
        self.r = math.sqrt((x[-1][0]) ** 2 + (x[-1][1]) ** 2)



class node:
    def __init__(self):
        self.path = None
        self.x = None
        self.y = None
        self.yaw = None
        self.parent = None
        self.children = []
        self.id = 0


class loc:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.occupancy = 0
        self.path_ids = []


class MAPF:
    def __init__(self):

        self.num_of_agents=1
        self.starts=[(2,0)]
        self.goals=[(10,0)]
        with open('obstacles2.pkl', 'rb') as file:
            self.map = pickle.load(file)
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(self.compute_heuristics(self.map, goal))
        # Subscribers and Publishers
        self.convoy_switch_sub = rospy.Subscriber(
            "/cmu_rc1/odom_to_base_link", Odometry, self.convoySwitchCallback1
        )

        self.convoy_sdwitch_sub = rospy.Subscriber(
            "/cmu_rc2/odom_to_base_link", Odometry, self.convoySwitchCallback2
        )

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
        all_paths = np.load("paths32.npy")
        all_rel_paths = []
        for i in range(all_paths.shape[0]):
            if all_paths[i][-1] == 2:
                all_rel_paths.append(all_paths[i])
        all_rel_paths_ids = []
        for i in range(len(all_rel_paths)):
            if all_rel_paths[i][3] not in all_rel_paths_ids:
                if all_rel_paths[i][3] in useless:
                    continue
                all_rel_paths_ids.append(all_rel_paths[i][3])
        print("number of relative self.paths: ", len(all_rel_paths_ids))
        self.path_dict = {}
        for i in range(len(all_rel_paths_ids)):
            self.path_dict[all_rel_paths_ids[i]] = []
        for i in range(len(all_rel_paths)):
            if all_rel_paths[i][3] in self.path_dict.keys():
                self.path_dict[all_rel_paths[i][3]].append(all_rel_paths[i])
        self.paths = {}
        for i in range(len(all_rel_paths_ids)):
            if self.path_dict[all_rel_paths_ids[i]] != []:
                self.self.paths[all_rel_paths_ids[i]] = path(
                    self.path_dict[all_rel_paths_ids[i]]
                )
        self.used = []
        for i in range(len(all_rel_paths)):
            if all_rel_paths[i][3] in self.path_dict.keys():
                if all_rel_paths[i][3] in useless:
                    continue
                self.used.append(all_rel_paths[i][3])
                self.path_dict[all_rel_paths[i][3]].append(all_rel_paths[i])
        self.used = np.unique(self.used)
        self.nodes1 = {}
        self.nodes2 = {}
        self.node_ids1 = []
        self.node_ids2 = []
        self.node_counter = 0

        self.current_ids = []
        self.next_ids = []

        self.contch_sub = rospy.Subscriber("/pubtreaj", Bool, self.soemt)
        self.reache_wpts_pub = rospy.Publisher("/shit", PoseArray, queue_size=5)
        # Variables
        self.flag = True
        self.odom1 = Odometry()
        self.odom2 = Odometry()
        self.has_odom1 = False
        self.has_odom2 = False
    def move(self,loc, dir):
        directions = [(0, -1), (1, 0), (0, 1), (-1, 0),(0,0)]
        for i in range(len(self.used)):
            
        return loc[0] + directions[dir][0], loc[1] + directions[dir][1]
    
    def compute_heuristics(self,my_map, goal):
        open_list = []
        closed_list = dict()
        root = {"loc": goal, "cost": 0}
        heapq.heappush(open_list, (root["cost"], goal, root))
        closed_list[goal] = root
        while len(open_list) > 0:
            (cost, loc, curr) = heapq.heappop(open_list)
            for dir in range(4):
                child_loc = self.move(loc, dir)
                child_cost = cost + 1
                if (
                    child_loc[0] < 0
                    or child_loc[0] >= len(my_map)
                    or child_loc[1] < 0
                    or child_loc[1] >= len(my_map[0])
                ):
                    continue
                if my_map[child_loc[0]][child_loc[1]]:
                    continue
                child = {"loc": child_loc, "cost": child_cost}
                if child_loc in closed_list:
                    existing_node = closed_list[child_loc]
                    if existing_node["cost"] > child_cost:
                        closed_list[child_loc] = child
                        heapq.heappush(open_list, (child_cost, child_loc, child))
                else:
                    closed_list[child_loc] = child
                    heapq.heappush(open_list, (child_cost, child_loc, child))

        # build the heuristics table
        h_values = dict()
        for loc, node in closed_list.items():
            h_values[loc] = node["cost"]
        return h_values

    def valid_path(self, path_id, node_id, nodes):
        print("path id", path_id)
        for i in range(len(self.paths[path_id].path)):
            rad = math.sqrt(
                (self.paths[path_id].path[i][0]) ** 2
                + (self.paths[path_id].path[i][1]) ** 2
            )
            theta = math.atan2(
                self.paths[path_id].path[i][1], self.paths[path_id].path[i][0]
            )
            xnew = nodes[node_id].x + rad * math.cos(theta + nodes[node_id].yaw)
            ynew = nodes[node_id].y + rad * math.sin(theta + nodes[node_id].yaw)
            if self.map[(int(xnew), int(ynew))].occupancy == 100:
                return False
        return True

    def draw_children(self, node_id, node_ids, nodes):
        node_counter = node_ids[-1]
        for i in range(len(self.used)):
            node_counter += 1
            if not self.valid_path(i, node_id):
                continue
            rad = self.paths[i].r
            theta = self.paths[i].theta
            # print("theta: ", theta)
            xnew = nodes[node_id].x + rad * math.cos(theta + nodes[node_id].yaw)
            ynew = nodes[node_id].y + rad * math.sin(theta + nodes[node_id].yaw)
            theta_new = self.paths[i].orientation + nodes[node_id].yaw
            nodes[node_counter] = node()
            nodes[node_counter].x = xnew
            nodes[node_counter].y = ynew
            nodes[node_counter].yaw = theta_new
            nodes[node_counter].id = node_counter
            nodes[node_counter].parent = node_id
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
            self.cbs()
            r.sleep()

    def publist(self):
        self.nodes1[0] = node()
        self.nodes1[0].x = self.odom1.pose.pose.position.x
        self.nodes1[0].y = self.odom1.pose.pose.position.y
        self.nodes1[0].yaw = euler_from_quaternion(
            [
                self.odom1.pose.pose.orientation.x,
                self.odom1.pose.pose.orientation.y,
                self.odom1.pose.pose.orientation.z,
                self.odom1.pose.pose.orientation.w,
            ]
        )[2]
        self.nodes1[0].id = 0
        self.nodes1[0].parent = None
        self.current_ids = []
        self.next_ids = [0]
        self.node_counter = 0
        for i in range(5):
            self.current_ids = self.next_ids
            self.next_ids = []
            for j in range(len(self.current_ids)):
                self.node_ids1.append(
                    self.draw_children(self.current_ids[j], self.node_ids1, self.nodes1)
                )

        view_nodes = PoseArray()
        view_nodes.header.frame_id = "cmu_rc1_odom"
        view_nodes.header.stamp = rospy.Time.now()
        for j in self.nodes1.keys():
            some = Pose
            some.position.x = self.nodes1[i].x
            some.position.y = self.nodes1[i].y
            view_nodes.pose.append(some)

        self.reache_wpts_pub.publish(view_nodes)
        self.flag = False

    def wrap2pi(self, yaw):
        if yaw > math.pi:
            yaw -= 2 * math.pi
        if yaw <= -math.pi:
            yaw += 2 * math.pi
        return yaw


    def cbs(self):
        self.start_time = timer.time()
        root = {"cost": 0, "constraints": [], "paths": [], "collisions": []}
        for i in range(self.num_of_agents):  # Find initial path for each agent
            path = self.a_star(
                self.map,
                self.starts[i],
                self.goals[i],
                self.heuristics[i],
                i,
                root["constraints"],
            )
            if path is None:
                raise BaseException("No solutions")
            root["paths"].append(path)
        root["collisions"] = detect_collisions_among_all_paths(root["paths"])
        root["cost"] = get_sum_of_cost(root["paths"])
        self.push_node(root)
        a = 0
        while len(self.open_list) > 0:
            curr = self.pop_node()
            curr["collisions"] = detect_collisions_among_all_paths(curr["paths"])
            if len(curr["collisions"]) == 0:
                self.print_results(curr)
                return curr["paths"]
            for j in curr["collisions"]:
                new_constraints = standard_splitting(j)
                for i in new_constraints:
                    child = copy.deepcopy(curr)
                    if i not in curr["constraints"]:
                        child["constraints"].append(i)
                    agent_id = i["agent"]
                    path = a_star(
                        self.my_map,
                        self.starts[agent_id],
                        self.goals[agent_id],
                        self.heuristics[agent_id],
                        agent_id,
                        child["constraints"],
                    )
                    a += 1
                    if a > 500:
                        print("TimeOut")
                        self.print_results(curr)
                        return curr["paths"]
                    print("path", path)
                    if path is None:
                        return None
                    child["paths"][agent_id] = path
                    child["collisions"] = detect_collisions_among_all_paths(
                        child["paths"]
                    )
                    child["cost"] = get_sum_of_cost(child["paths"])
                    self.push_node(child)
        return None
    
    def a_star(my_map, start_loc, goal_loc, h_values, agent, constraints):

            thresh = 0
            for row in my_map:
                thresh += len(row) - sum(row)
            thresh1 = agent * thresh
            open_list = []
            closed_list = dict()
            h_value = h_values[start_loc]
            constraint_table, max_time = build_constraint_table(constraints, agent)
            if max_time < thresh:
                thresh = max_time
            root = {
                "loc": start_loc,
                "g_val": 0,
                "h_val": h_value,
                "parent": None,
                "timestep": 0,
            }
            push_node(open_list, root)
            closed_list[((root["loc"], root["timestep"]))] = root
            while len(open_list) > 0:
                curr = pop_node(open_list)
                #############################
                # Task 2.2: Adjust the goal test condition to handle goal constraints
                if curr["loc"] == goal_loc:
                    if thresh < max_time:
                        flag = False
                        for i in range(curr["timestep"], max_time):
                            if is_constrained(curr["loc"], goal_loc, i, constraint_table):
                                flag = True
                        if not flag:
                            return get_path(curr)
                    elif curr["timestep"] > thresh and curr["loc"] == goal_loc:
                        return get_path(curr)
                if curr["timestep"] > thresh1 and thresh1 != 0:
                    continue
                for dir in range(5):
                    child_loc = move(curr["loc"], dir)
                    if not in_map(my_map, child_loc):
                        continue
                    if my_map[child_loc[0]][child_loc[1]]:
                        continue
                    if is_constrained(
                        curr["loc"], child_loc, curr["timestep"] + 1, constraint_table
                    ):
                        continue

                    child_timestep = curr["timestep"] + 1
                    child = {
                        "loc": child_loc,
                        "g_val": curr["g_val"] + 1,
                        "h_val": h_values[child_loc],
                        "parent": curr,
                        "timestep": child_timestep,
                    }
                    print("loc", child_loc)
                    if (child["loc"], child["timestep"]) in closed_list:
                        existing_node = closed_list[(child["loc"], child["timestep"])]
                        if compare_nodes(child, existing_node):
                            closed_list[(child["loc"], child["timestep"])] = child
                            push_node(open_list, child)
                    else:
                        closed_list[(child["loc"], child["timestep"])] = child
                        push_node(open_list, child)
            return None  # Failed to find solutions


if __name__ == "__main__":
    rospy.init_node("convoy_formup")
    tf_manager = MAPF()
    tf_manager.run()
