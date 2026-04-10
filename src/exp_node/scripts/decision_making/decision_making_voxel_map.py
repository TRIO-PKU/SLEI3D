import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from typing import List, Dict, Tuple
from decision_making.decision_making_data_structure import (
    Decision,
    DecisionMakingData,
    ExpData,
    InsData,
    Meeting,
    InspectionPoint,
    AStarPath,
    PointWithTimeWindow
)
import itertools
import numpy as np
from decision_making.astar_3d import AStar3D
from msgs.msg import VoxelMap
from msgs.srv import plan_traj, plan_trajRequest, plan_trajResponse
from bspline.msg import Bspline
from decision_making.astar_to_bspline import astar_to_bspline

class DecisionMakingVoxelMap:
    # def __init__(self, voxel_map_topic: str, resolution = 0.2, remap = np.array([0,0,0])) -> None:
    #     self.voxel_map_subscriber = rospy.Subscriber(voxel_map_topic, VoxelMap, self.voxel_map_callback)
    #     self.resolution: float = resolution
    #     self.remap: np.ndarray = remap
    
    # def voxel_map_callback(self, msg: VoxelMap) -> None:
    #     self.voxel_map = np.array(msg.data).reshape((msg.size_x, msg.size_y, msg.size_z))
    #     self.resolution = msg.resolution
    #     self.remap = np.array([msg.remap_x, msg.remap_y, msg.remap_z])
        
    #     self.voxel_map_subscriber.unregister()
    
    def __init__(self, voxel_map) -> None:
        self.voxel_map = voxel_map
        self.resolution= rospy.get_param("/voxel_map/resolution")
        remap_x, remap_y, remap_z = rospy.get_param("/voxel_map/remap_x"), rospy.get_param("/voxel_map/remap_y"), rospy.get_param("/voxel_map/remap_z")
        self.remap: np.ndarray = np.array([remap_x, remap_y, remap_z])
        
        self.plan_traj_server = rospy.Service("dmvm/plan_traj", plan_traj, self.plan_traj_cb)
        self.point_cloud_pub = rospy.Publisher("dmvm/point_cloud", PointCloud2, queue_size=100)
        
    def sample_meeting_points(self, dm_data: DecisionMakingData, bbox: List[float], range: float = 10.0, max_num: int = 50, density: float = 0.2) -> DecisionMakingData:
        for ins_data in dm_data.ins_data_list:
            for ip in ins_data.inspection_points:
                self._sample_meeting_points_for_ip(ip, bbox, range, max_num, density)
        return dm_data
    
    def _sample_meeting_points_for_ip(self, ip: InspectionPoint, bbox: List[float], los_range: float, max_num, dencity) -> None:
        sample_points = []
        for x in np.arange(ip.position[0] - los_range, ip.position[0] + los_range, self.resolution / dencity):
            for y in np.arange(ip.position[1] - los_range, ip.position[1] + los_range, self.resolution / dencity):
                for z in np.arange(ip.position[2] - los_range, ip.position[2] + los_range, self.resolution / dencity):
                    # in bbox
                    if x < bbox[0] or x > bbox[3] or y < bbox[1] or y > bbox[4] or z < bbox[2] or z > bbox[5]:
                        continue
                    
                    # in voxel map
                    voxel = self.point_to_voxel((x, y, z))
                    if voxel[0] < 0 or voxel[0] >= self.voxel_map.shape[0] or \
                        voxel[1] < 0 or voxel[1] >= self.voxel_map.shape[1] or \
                        voxel[2] < 0 or voxel[2] >= self.voxel_map.shape[2]:
                        continue
                    
                    # in range
                    if (x - ip.position[0])**2 + (y - ip.position[1])**2 + (z - ip.position[2])**2 > (los_range-1)**2:
                        continue
                    
                    # LOS
                    fit_los = True
                    step = self.resolution
                    distance = np.sqrt((x - ip.position[0])**2 + (y - ip.position[1])**2 + (z - ip.position[2])**2)
                    unit_vector = np.array([ip.position[0] - x, ip.position[1] - y, ip.position[2] - z]) / distance
                    
                    for t in np.arange(0, distance, step):
                        point_along_line = (x + unit_vector[0] * t, y + unit_vector[1] * t, z + unit_vector[2] * t)
                        voxel_ = self.point_to_voxel(point_along_line)
                        if np.any(self.voxel_map[
                            max(0, voxel_[0] - 2):min(self.voxel_map.shape[0], voxel_[0] + 3),
                            max(0, voxel_[1] - 2):min(self.voxel_map.shape[1], voxel_[1] + 3),
                            max(0, voxel_[2] - 2):min(self.voxel_map.shape[2], voxel_[2] + 3)
                        ] > 0):
                            fit_los = False
                            break
                    if not fit_los:
                        continue
                    
                    sample_points.append(PointWithTimeWindow((x, y, z), ip.position, ip.time_window))

        # Perform uniform downsampling if the number of sample points exceeds the maximum number
        if len(sample_points) > max_num:
            indices = np.round(np.linspace(0, len(sample_points) - 1, max_num)).astype(int)
            indices = list(dict.fromkeys(indices)) # remove duplicates
            sample_points = [sample_points[i] for i in indices]
            
        ip.sampling_meeting_points = sample_points
        # self.publish_point_cloud(sample_points)

    def publish_point_cloud(self, sample_points):
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "world"  # 设置坐标系

        fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1)
        ]

        points = [(p[0], p[1], p[2]) for p in sample_points]

        point_cloud = pc2.create_cloud(header, fields, points)
        self.point_cloud_pub.publish(point_cloud)
    
    def astar_find_path(self, start_point: Tuple[float, float, float], goal_point: Tuple[float, float, float]) -> AStarPath:
        # remap the start_point and goal_point to voxel        
        start_voxel = self.point_to_voxel(start_point)
        goal_voxel = self.point_to_voxel(goal_point)
        # rospy.logwarn(f"Start Voxel: {start_voxel}")
        # rospy.logwarn(f"Goal Voxel: {goal_voxel}")
        
        path: List[Tuple[int, int, int]] = AStar3D(self.voxel_map).find_path(start_voxel, goal_voxel)
        if path == None:
            return None
        
        # remap the path to the real world
        real_path = [self.voxel_to_point(p) for p in path]
        return AStarPath(real_path)
    
    def point_to_voxel(self, point: Tuple[float, float, float]) -> Tuple[int, int, int]:
        return (int((point[0] + self.remap[0]) / self.resolution), 
                int((point[1] + self.remap[1]) / self.resolution), 
                int((point[2] + self.remap[2]) / self.resolution))
    
    def voxel_to_point(self, voxel: Tuple[int, int, int]) -> Tuple[float, float, float]:
        return (voxel[0] * self.resolution - self.remap[0] + self.resolution / 2, 
                voxel[1] * self.resolution - self.remap[1] + self.resolution / 2, 
                voxel[2] * self.resolution - self.remap[2] + self.resolution / 2)
        
    def plan_traj_cb(self, req: plan_trajRequest) -> plan_trajResponse:
        start_point: Tuple[float] = req.pos.x, req.pos.y, req.pos.z
        goal_point: Tuple[float] = req.next_pos.x, req.next_pos.y, req.next_pos.z
        path: AStarPath = self.astar_find_path(start_point, goal_point)
        if path == None:
            rospy.logerr("astar Failed to find path")
            res = plan_trajResponse()
            res.success = False
            return res
        bspline = astar_to_bspline(path)
        res = plan_trajResponse()
        res.bspline = bspline
        res.success = True
        res.duration = (bspline.end_time - bspline.start_time).to_sec()
        return res

    
if __name__ == "__main__":
    # 假设 VOXEL_SIZE 是 1.0
    VOXEL_SIZE = 0.2
    
    # 定义楼房的尺寸和位置
    building_min_coords = np.array([10, 10, 0])  # 楼房的最小坐标
    building_max_coords = np.array([20, 20, 10])  # 楼房的最大坐标

    # 定义边界空间
    boundary_space = 5  # 在楼房周围增加 5 个体素的空间

    # 计算体素网格的尺寸，包括边界空间
    grid_min_coords = building_min_coords - boundary_space
    grid_max_coords = building_max_coords + boundary_space
    grid_size = np.ceil((grid_max_coords - grid_min_coords) / VOXEL_SIZE).astype(np.int32)

    # 初始化体素网格
    voxel_grid = np.zeros(grid_size, dtype=np.int32)

    # 填充楼房的体素
    for x in range(building_min_coords[0], building_max_coords[0]):
        for y in range(building_min_coords[1], building_max_coords[1]):
            for z in range(building_min_coords[2], building_max_coords[2]):
                voxel_x = int((x - grid_min_coords[0]) / VOXEL_SIZE)
                voxel_y = int((y - grid_min_coords[1]) / VOXEL_SIZE)
                voxel_z = int((z - grid_min_coords[2]) / VOXEL_SIZE)
                voxel_grid[voxel_x, voxel_y, voxel_z] = 1  # 将楼房区域的体素设置为 1

    # 打印体素网格
    print("Voxel Grid:")
    print(voxel_grid[5])
    # endregion

    path: AStarPath = DecisionMakingVoxelMap(voxel_grid).astar_find_path((0, 0, 0), (3.9, 3.9, 3.9))
    print(path)
    print(path.length())
    print(path.travle_time(1.0, 2.0))
    
    dm_data = DecisionMakingData()
    dm_data.exp_data = ExpData("exp_1", (0, 0, 0), [(1, 1, 1), (2, 2, 2)])
    dm_data.ins_data_list = [InsData("ins_1", [InspectionPoint((1, 1, 1),(1,1))])]
    print(dm_data)
    sample_points = DecisionMakingVoxelMap(voxel_grid).sample_meeting_points(dm_data, 3, 10, 0.1)
    print(sample_points)
    
    
    
    