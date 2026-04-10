import rospy
from typing import List, Tuple, Dict, Union
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path, Odometry
from dataclasses import dataclass, field
from bspline.msg import Bspline # 测试时注释掉
from msgs.msg import Combine_bspline_inspection_pose

@dataclass
class InsData:
    inspection_time: float = rospy.get_param("/inspection_time")
    ins_bsp_list: List[Union[PoseStamped, Bspline]] = field(default_factory=list)
    
    @property
    def bspline_list(self):
        return [ins for ins in self.ins_bsp_list if isinstance(ins, Bspline)]
    
    @property
    def inspection_pose_list(self):
        return [ins for ins in self.ins_bsp_list if isinstance(ins, PoseStamped)]
    
    @property
    def finish_time(self):
        for i in range(len(self.ins_bsp_list)):
            pose_or_bspline: Union[PoseStamped, Bspline] = self.ins_bsp_list[::-1][i]
            if isinstance(pose_or_bspline, Bspline):
                pose_or_bspline: Bspline
                return pose_or_bspline.end_time + rospy.Duration(i * self.inspection_time)
        return rospy.Time.now() - rospy.Duration(1)
    
    @property
    def finish_pose(self) -> PoseStamped:
        return self.ins_bsp_list[-1]
    
    @property
    def point_tw_list(self) -> List[Tuple[Tuple[float], Tuple[float]]]:
        point_tw_list = []
        tw_left = rospy.Time.now().to_sec() - 1 
        
        if not any(isinstance(pose_or_bspline, Bspline) for pose_or_bspline in self.ins_bsp_list):
            # No bspline, set time window to infinity
            for p_or_b in self.ins_bsp_list:
                p_or_b: PoseStamped
                point = (p_or_b.pose.position.x, p_or_b.pose.position.y, p_or_b.pose.position.z)
                point_tw_list.append((point, (tw_left, float("inf"))))
            return point_tw_list

        for i in range(len(self.ins_bsp_list)):
            p_or_b_current: Union[PoseStamped, Bspline] = self.ins_bsp_list[i]
            if isinstance(p_or_b_current, PoseStamped):
                p_or_b_current: PoseStamped
                point = (p_or_b_current.pose.position.x, p_or_b_current.pose.position.y, p_or_b_current.pose.position.z)
                point_tw_list.append((point, (tw_left, tw_left + self.inspection_time)))
                tw_left += self.inspection_time
            else:
                p_or_b_current: Bspline
                tw_left = p_or_b_current.end_time.to_sec()

        point_tw_list[-1] = (point_tw_list[-1][0], (point_tw_list[-1][1][0], float("inf")))
        for i in range(len(point_tw_list)):
            left, right = point_tw_list[i][1]
            if left > right:
                point_tw_list[i] = (point_tw_list[i][0], (right, right))
        return point_tw_list
    
    @property
    def combine_list(self) -> list:
        combine_list = []
        cbi = Combine_bspline_inspection_pose()
        if not any(isinstance(pose_or_bspline, Bspline) for pose_or_bspline in self.ins_bsp_list):
            for p_or_b in self.ins_bsp_list:
                p_or_b: PoseStamped
                cbi.inspection_poses.append(p_or_b)
            return combine_list
        
        if isinstance(self.ins_bsp_list[0], PoseStamped):
            cbi.inspection_poses.append(self.ins_bsp_list[0])
        else:
            cbi.bsplines.append(self.ins_bsp_list[0])

        for i in range(len(self.ins_bsp_list) - 1):
            p_or_b_current: Union[PoseStamped, Bspline] = self.ins_bsp_list[i]
            p_or_b_next: Union[PoseStamped, Bspline] = self.ins_bsp_list[i+1]
            if isinstance(p_or_b_current, PoseStamped) and isinstance(p_or_b_next, Bspline):
                combine_list.append(cbi)
                cbi = Combine_bspline_inspection_pose()
                cbi.bsplines.append(p_or_b_next)
            elif isinstance(p_or_b_next, Bspline):
                cbi.bsplines.append(p_or_b_next)
            else:
                cbi.inspection_poses.append(p_or_b_next)
        combine_list.append(cbi)
        return combine_list

class MeetingWithGcs:
    pass
   
class MeetingWithIns:
    def __init__(self, ins_id: str = "_", meeting_point = (0, 0, 0), time_window = (0, 0), allocated_features = []) -> None:
        self.ins_id: str = ins_id
        self.meeting_point: Tuple[float, float, float] = meeting_point
        self.meeting_pose: PoseStamped = self.tuple_2_PoseStamped(meeting_point)
        self.time_window: Tuple[float] = time_window
        self.allocated_features_tuple: List[Tuple[float, float, float, rospy.Time]] = allocated_features
        self.allocated_features_Path: Path = self.tuple_list_2_Path(allocated_features)
        
        self.go_to_pose: PoseStamped = PoseStamped()
        self.go_to_pose.header.stamp = rospy.Time(0)
    
    def __str__(self) -> str:
        return f"{self.ins_id}: {self.meeting_pose} {self.time_window} {self.allocated_features_tuple}"
    
    def tuple_2_PoseStamped(self, tuple: Tuple[float, float, float]) -> PoseStamped:
        pose = PoseStamped()
        pose.pose.position.x = tuple[0]
        pose.pose.position.y = tuple[1]
        pose.pose.position.z = tuple[2]
        pose.header.stamp = rospy.Time.now() if len(tuple) == 3 else tuple[3]
        return pose

    def tuple_list_2_Path(self, tuple_list: List[Tuple[float, float, float]]) -> Path:
        path = Path()
        for tuple in tuple_list:
            pose = self.tuple_2_PoseStamped(tuple)
            path.poses.append(pose)
        return path
    
if __name__ == "__main__":
    
    class Bspline:
        def __init__(self) -> None:
            self.start_time: rospy.Time = rospy.Time(0)
            self.end_time: rospy.Time = rospy.Time(0)
            
        def __str__(self) -> str:
            return f"start_time: {self.start_time.to_sec():.1f}, end_time: {self.end_time.to_sec():.1f}"
        
        def __repr__(self) -> str:
            return self.__str__()
        
        def __eq__(self, o: object) -> bool:
            if not isinstance(o, Bspline):
                return False
            return self.start_time == o.start_time and self.end_time == o.end_time

    # 产生一系列 posestamp 和 bspline
    ins_data = InsData()
    for i in range(20, 100, 10):
        pose = PoseStamped()
        pose.pose.position.x = i
        pose.pose.position.y = i
        pose.pose.position.z = i
        bspline = Bspline()
        bspline.start_time = rospy.Time(i*10)
        bspline.end_time = bspline.start_time + rospy.Duration((i+1)*10)
        ins_data.ins_bsp_list.append((pose, bspline))
    print(ins_data.point_tw_list)