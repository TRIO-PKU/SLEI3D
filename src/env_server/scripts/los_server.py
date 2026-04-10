#! /usr/bin/env python

import rospy, numpy as np
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from geometry_msgs.msg import Point32
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from msgs.srv import check_los, check_losRequest, check_losResponse
from voxel_map_server import VoxelMapServer

VOXEL_SIZE = 0.2

class LOSServer():       
    def __init__(self, voxel_map: VoxelMapServer) -> None: 
        self.los_range = rospy.get_param("/communication_range", 10)
        # odom
        self.exp_num = rospy.get_param("/exp_num")
        self.ins_num = rospy.get_param("/ins_num")
        self.odom_subs: list[rospy.Subscriber] = []
        self.id_position_dict: dict[String, Point32] = {}
        self.init_odom_subs()
        
        # global map
        self.voxel_grid = voxel_map.voxel_grid
        self.translation = voxel_map.translation
  
        # main server
        self.check_los_server = rospy.Service("/check_los", check_los , self.check_los_cb)
        
        # visualization
        self.los_vis_pub = rospy.Publisher("/los_vis", String, queue_size=10)
        
    def check_los_cb(self, req: check_losRequest) -> check_losResponse:
        client_position = self.id_position_dict[req.client_id]
        target_position = self.id_position_dict[req.target_id]
        p1 = np.array([client_position.x, client_position.y, client_position.z]) + self.translation
        p2 = np.array([target_position.x, target_position.y, target_position.z]) + self.translation
        step = 0.2
        distance = np.linalg.norm(p2 - p1)
        if distance > self.los_range:
            resp = check_losResponse()
            resp.success = False
            self.los_vis_pub.publish(req.client_id + "," + req.target_id + "," + "false")
            return resp
        direction = (p2 - p1) / np.linalg.norm(p2 - p1)
        for i in range(int(np.linalg.norm(p2 - p1) / step)):
            p = p1 + i * step * direction
            voxel = np.array([int(p[0] / VOXEL_SIZE), int(p[1] / VOXEL_SIZE), int(p[2] / VOXEL_SIZE)])
            # 判断voxel以及其周围的voxel是否有障碍物
            for x in range(-1, 2):
                for y in range(-1, 2):
                    for z in range(-1, 2):
                        index = tuple(voxel + np.array([x, y, z]))
                        if index[0] >= 0 and index[0] < self.voxel_grid.shape[0] and index[1] >= 0 and index[1] < self.voxel_grid.shape[1] and index[2] >= 0 and index[2] < self.voxel_grid.shape[2]:
                            if self.voxel_grid[index] > 0:
                                resp = check_losResponse()
                                resp.success = False
                                self.los_vis_pub.publish(req.client_id + "," + req.target_id + "," + "false")
                                return resp

        resp = check_losResponse()
        resp.success = True
        self.los_vis_pub.publish(req.client_id + "," + req.target_id + "," + "true")
        return resp
                
    def init_odom_subs(self) -> None:
        for i in range(1, self.exp_num + 1):
            self.register_odom_sub("exp_" + str(i))
        for i in range(1, self.ins_num + 1):
            self.register_odom_sub("ins_" + str(i))
        self.register_odom_sub("gcs")
        
    def register_odom_sub(self, id: String) -> None:
        self.id_position_dict[id] = Point32()
        topic = str(id) + "/odom"
        self.odom_subs.append(rospy.Subscriber(topic, Odometry, self.odom_cb, callback_args=id))
    
    def odom_cb(self, msg: Odometry, drone_id: String) -> None:
        position = Point32()
        position.x = msg.pose.pose.position.x
        position.y = msg.pose.pose.position.y
        position.z = msg.pose.pose.position.z
        self.id_position_dict[drone_id] = position