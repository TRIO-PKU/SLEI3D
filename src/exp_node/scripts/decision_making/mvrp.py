import numpy as np
from typing import Tuple, List
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from decision_making.decision_making_voxel_map import DecisionMakingVoxelMap as DMVM
from decision_making.decision_making_data_structure import (
    Decision,
    DecisionMakingData,
    ExpData,
    InsData,
    Meeting,
    InspectionPoint,
    AStarPath
)
id = str
point = Tuple[float, float, float]
time_window = Tuple[float, float]

class Agent:
    def __init__(self, id, start_time: float, inspection_time: float, start_point: point, dmvm: DMVM) -> None:
        self.id = id
        self.start_time = start_time    # 开始时间，每个agent的开始时间不同
        self.inspection_time = inspection_time   # 每个feature的拍照时间
        self.finsh_time = start_time  # agent的任务完成时间，初始化为开始时间
        self.waste_time = 0.0    # 浪费的时间
        self.current_position: point = start_point   # 当前位置，初始化为起始位置
        self.features = [] # 被分到的feature点
        self.dmvm = dmvm

    def travel_to(self, feature_point: point):  # TODO: call A* or compute the distance directly
        distance: float = ((self.current_position[0] - feature_point[0])**2 + (self.current_position[1] - feature_point[1])**2 + (self.current_position[2] - feature_point[2])**2)**0.5
        travel_time = self.calculate_time_according_to_distance(distance, 1.0, 2.0)   # TODO: 1.0, 2.0 are constants, need to be replaced
        # 更新完成时间，考虑到agent当前的完成时间和旅行时间
        self.finsh_time = self.finsh_time + travel_time + self.inspection_time
        self.waste_time = self.waste_time + travel_time
        self.current_position = feature_point
        self.features.append(feature_point)

    def calculate_time_according_to_distance(self, distance:float, acc: float, max_v: float) -> float:
        t_acc = max_v / acc  # Time to accelerate to max velocity
        d_acc = 0.5 * acc * t_acc ** 2  # Distance covered during acceleration

        if distance < 2 * d_acc:  # Never reach max velocity
            t_total = (2 * distance / acc) ** 0.5
        else:
            d_const = distance - 2 * d_acc  # Distance covered at constant velocity
            t_const = d_const / max_v  # Time at constant velocity
            t_total = 2 * t_acc + t_const

        return t_total

class MVRP:
    def __init__(self, dm_data: DecisionMakingData, decision: Decision, inspection_time: float, dm_voxel_map: DMVM) -> None:
        self.dm_data: DecisionMakingData = dm_data
        self.decision: Decision = decision
        self.features: List[point] = dm_data.exp_data.features_to_allocate
        self.inspection_time: float = inspection_time
        self.dmvm: DMVM = dm_voxel_map
        self.ins_list: List[Agent] = self.initialize_ins(decision) # 初始化agents

    def initialize_ins(self, decision: Decision) -> List[Agent]:
        agents: List[Agent] = []
        for meeting in decision.meeting_list:
            agent = Agent(
                id = meeting.ins_id,
                start_time = self.calculate_start_time(meeting),
                inspection_time = self.inspection_time,
                start_point = self.find_start_point(meeting.ins_id),
                dmvm = self.dmvm
            )
            agents.append(agent)
        return agents
    
    def find_start_point(self, ins_id: id) -> point:
        for ins_data in self.dm_data.ins_data_list:
            if ins_data.id == ins_id:
                return ins_data.inspection_points[-1].position
    
    def calculate_start_time(self, meeting: Meeting) -> float:
        for ins_data in self.dm_data.ins_data_list:
            if ins_data.id == meeting.ins_id:
                start_next_inspection: float = 0.0
                if ins_data.inspection_points == []:
                    start_next_inspection = exp_meeting_time
                else:
                    start_last_inspection: float = ins_data.inspection_points[-1].time_window[0]
                    exp_meeting_time: float = meeting.predicted_arrive_time
                    if exp_meeting_time < (start_last_inspection + self.inspection_time):
                        start_next_inspection = start_last_inspection + self.inspection_time
                    else:
                        start_next_inspection = exp_meeting_time
        return start_next_inspection

    def calculate_total_completion_time(self):
        return max(agent.finsh_time for agent in self.ins_list)
    
    def update_decision(self, right_axis: float):
        self.decision.right_axis = right_axis
        for agent in self.ins_list:
            for meeting in self.decision.meeting_list:
                if meeting.ins_id == agent.id:
                    meeting.allocated_features = agent.features
                    self.decision.mvrp_time_cost += agent.waste_time + right_axis - agent.finsh_time
        return self.decision

    def assign_points_to_agents(self):
        unvisited = self.features.copy()

        while unvisited:
            for agent in self.ins_list:
                if not unvisited:
                    break
                
                # 找到最近的未访问点
                closest_point = None
                closest_distance = float('inf')

                for point in unvisited:
                    distance = ((agent.current_position[0] - point[0])**2 + (agent.current_position[1] - point[1])**2 + (agent.current_position[2] - point[2])**2)**0.5
                    if distance < closest_distance:
                        closest_distance = distance
                        closest_point = point

                # 移动到最近的点
                if closest_point is not None:
                    agent.travel_to(closest_point)
                    unvisited.remove(closest_point)

        right_axis: float = self.calculate_total_completion_time()
        self.decision = self.update_decision(right_axis)

        return self.decision
    
def generate_voxel_map():
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
        
        return voxel_grid

if __name__ == "__main__":
    # 生成测试数据
    np.random.seed(0)
    num_agents = 3
    num_features = 20

    # 初始化agents
    ins_data_list = [InsData(id=str(i), inspection_points=[InspectionPoint(position=(np.random.rand() * 10, np.random.rand() * 10, np.random.rand() * 10), time_window=(0, 10))]) for i in range(num_agents)]

    # 初始化features
    features_to_allocate = [(np.random.rand() * 10, np.random.rand() * 10, np.random.rand() * 10) for _ in range(num_features)]

    # 初始化exp_data
    exp_data = ExpData(id="exp1", current_position=(0, 0, 0), features_to_allocate=features_to_allocate)

    # 初始化decision
    decision = Decision()
    for agent in ins_data_list:
        meeting_point = (np.random.rand() * 10, np.random.rand() * 10, np.random.rand() * 10)
        meeting = Meeting()
        meeting.ins_id=agent.id
        meeting.meeting_point=meeting_point
        meeting.predicted_arrive_time=np.random.rand() * 10
        meeting.time_window=(0, 10)
        decision.meeting_list.append(meeting)

    # 初始化dm_data
    dm_data = DecisionMakingData()
    dm_data.exp_data = exp_data
    dm_data.ins_data_list = ins_data_list

    # 初始化dm_voxel_map
    dm_voxel_map = DMVM(generate_voxel_map())

    # 初始化MVRP
    inspection_time = 1.0
    mvrp = MVRP(dm_data=dm_data, decision=decision, inspection_time=inspection_time, dm_voxel_map=dm_voxel_map)

    # 分配特征点给agents
    updated_decision, right_axis = mvrp.assign_points_to_agents()

    # 绘制三维图
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    colors = ['r', 'g', 'b']
    for meeting, color in zip(updated_decision.meeting_list, colors):
    # 绘制agent的开会位置
        initial_position = meeting.meeting_point
        ax.scatter(initial_position[0], initial_position[1], initial_position[2], c=color, marker='o', label=f'Agent {meeting.ins_id} Meeting Point')

        # 绘制agent分配的特征点
        for feature in meeting.allocated_features:
            ax.scatter(feature[0], feature[1], feature[2], c=color, marker='^', label=f'Agent {meeting.ins_id} Feature')

    # 设置图例
    handles, labels = ax.get_legend_handles_labels()
    by_label = dict(zip(labels, handles))
    ax.legend(by_label.values(), by_label.keys())

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    plt.show()