#!/usr/bin/env python
"""
Creates the same interface of sensors and frames between
the vehicle and the sim
"""
#!/usr/bin/env python
"""
A script to interface with a simulation environment to extract and store the coordinates of obstacles
in an occupancy grid published by a simulated vehicle.
"""
import rospy
import pickle
from nav_msgs.msg import OccupancyGrid

class ObstacleMap(object):
    def __init__(self):
        rospy.init_node('obstacle_map')
        # self.obstacles = []
        self.odom_sub = rospy.Subscriber("/cmu_rc1/local_mapping_lidar_node/voxel_grid/obstacle_map", OccupancyGrid, self.OccupancyGridCB)

    def OccupancyGridCB(self, msg):
        self.obstacles=[]
        for i, value in enumerate(msg.data):
            if value == 100:
                x = msg.info.origin.position.x+(i % msg.info.width)*0.15
                y = msg.info.origin.position.y+(i // msg.info.width)*0.15
                self.obstacles.append((x, y))

    def run(self):
        rospy.spin()

    def save_obstacles(self):
        print('here')
        with open("obstacles2.pkl", "wb") as file:
            pickle.dump(self.obstacles, file)
        print("Obstacle coordinates have been saved to 'obstacles.pkl'")

if __name__ == '__main__':
    node = ObstacleMap()
    node.run()
    
    node.save_obstacles()
