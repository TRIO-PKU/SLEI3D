#! /usr/bin/env python

import rospy, numpy as np
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from geometry_msgs.msg import Point32
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from msgs.srv import check_los, check_losRequest, check_losResponse
from functools import partial
import os
from msgs.msg import VoxelMap

VOXEL_SIZE = 0.2

class VoxelMapServer():       
    def __init__(self) -> None: 
        # global map
        self.have_map = False
        self.sparse_point_map = None
        self.voxel_map_path = rospy.get_param("/voxel_map_path")
        self.map_sub = rospy.Subscriber("/map_generator/global_cloud", PointCloud2, self.map_cb)
        self.map_pub = rospy.Publisher("/voxel_map", VoxelMap, queue_size=1)
        while not self.publish_map():
            rospy.sleep(1)
    
    def map_cb(self, msg: PointCloud2) -> None:
        if self.have_map:
            return
        points = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        points = list(points)
        sparse_points = []
        for i in range(0, len(points), 100):
            sparse_points.append(points[i])
        points_np = np.array(sparse_points)
        self.sparse_point_map = points_np
        
        min_coords = np.min(points_np, axis=0)
        max_coords = np.max(points_np, axis=0)
        grid_size = np.ceil((max_coords - min_coords) / VOXEL_SIZE).astype(np.int32)
        self.translation = -min_coords
        points_translated = points_np + self.translation
        
        self.voxel_grid = np.zeros(grid_size, dtype=np.int32)
        for point in points_translated:
            x, y, z = point
            voxel_x = int(x / VOXEL_SIZE)
            voxel_y = int(y / VOXEL_SIZE)
            voxel_z = int(z / VOXEL_SIZE)
            if 0 <= voxel_x < grid_size[0] and 0 <= voxel_y < grid_size[1] and 0 <= voxel_z < grid_size[2]:
                self.voxel_grid[voxel_x, voxel_y, voxel_z] += 1
        self.have_map = True
        
    def publish_map(self) -> bool:
        if not self.have_map:
            return False
        
        rospy.set_param("/voxel_map/resolution", VOXEL_SIZE)
        rospy.set_param("/voxel_map/remap_x", float(self.translation[0]))
        rospy.set_param("/voxel_map/remap_y", float(self.translation[1]))
        rospy.set_param("/voxel_map/remap_z", float(self.translation[2]))
        
        np.save(self.voxel_map_path, self.voxel_grid)
        return True
        # voxel_map: VoxelMap = VoxelMap()
        # voxel_map.resolution = VOXEL_SIZE
        # voxel_map.size_x, voxel_map.size_y, voxel_map.size_z = self.voxel_grid.shape
        # voxel_map.remap_x, voxel_map.remap_y, voxel_map.remap_z = self.translation
        # voxel_map.data = self.voxel_grid.flatten().tolist()
        # self.map_pub.publish(voxel_map)