from typing import Iterable, List, Dict, Tuple, Any
from math import dist
from functools import total_ordering 
id = str
point = Tuple[float, float, float]
time_window = Tuple[float, float]

class InspectionPoint:
    def __init__(self, position: point, time_window: Tuple[float, float]):
        self.position = position
        self.time_window: Tuple[float, float] = time_window

        # record attributes
        self.sampling_meeting_points: List[PointWithTimeWindow] = []
        
    def __str__(self) -> str:
        return f"({self.position[0]:.1f}, {self.position[1]:.1f}, {self.position[2]:.1f})->({self.time_window[0]:.1f}, {self.time_window[1]:.1f}) : {self.sampling_meeting_points}"
    
class PointWithTimeWindow(tuple):
    def __new__(cls, iterable: Iterable = ..., inspection_point: point = None, time_window: Tuple[float, float] = None):
        instance = super().__new__(cls, iterable)
        instance.inspection_point = inspection_point
        instance.time_window = time_window # type: ignore
        return instance
class Meeting:
    def __init__(self, ins_id: str = "_", meeting_point: point = (0, 0, 0), ins_inspection_point: point = (0, 0, 0), predicted_arrive_time: float = 0.0, time_window: Tuple[float, float] = (0, 0), allocated_features: List[point] = []):
        self.ins_id = ins_id
        self.meeting_point = meeting_point
        self.ins_inspection_point = ins_inspection_point
        self.predicted_arrive_time = predicted_arrive_time  # calculated by GA, used to compute start time in mvrp
        self.time_window = time_window
        self.allocated_features = allocated_features
    
    def __str__(self) -> str:
        return f"{self.ins_id}: {self.meeting_point} {self.time_window} {self.allocated_features}"

class InsData:
    def __init__(self, id: str = "_", inspection_points: List[InspectionPoint] = []) -> None:
        self.id = id
        self.inspection_points = inspection_points

    def __init__(self, id: str, inspection_points: List[InspectionPoint]):
        self.id = id
        self.inspection_points = inspection_points
        
    def __str__(self) -> str:
        return f"{self.id}: {[str(i) for i in self.inspection_points]}"

class ExpData:
    def __init__(self, id: str = "_", current_position: point = (0, 0, 0), features_to_allocate: List[point] = []) -> None:
        self.id = id
        self.current_position = current_position
        self.features_to_allocate = features_to_allocate

    def __init__(self, id: str, current_position: point, features_to_allocate: List[point]) -> None:
        self.id = id
        self.current_position = current_position
        self.features_to_allocate = features_to_allocate
        
    def __str__(self) -> str:
        return f"{self.id}: position: {self.current_position} feature_num: {len(self.features_to_allocate)}"
        

class DecisionMakingData:
    def __init__(self) -> None:
        self.exp_data: ExpData = ExpData("_", (0, 0, 0), [])
        self.ins_data_list: List[InsData] = []
    
    def __str__(self) -> str:
        return f"dm_data: \n{self.exp_data}\n{[str(i) for i in self.ins_data_list]}"


@total_ordering
class Decision:
    def __init__(self) -> None:
        self.meeting_list: List[Meeting] = []
        self.ga_time_cost: float = 0.0
        self.mvrp_time_cost: float = 0.0
        self.other_ins_time_cost: float = 0.0
        self.right_axis: float = -0.1
    
    def __eq__(self, other) -> bool:
        return self.ga_time_cost + self.mvrp_time_cost + self.other_ins_time_cost == other.ga_time_cost + other.mvrp_time_cost + other.other_ins_time_cost
    
    def __lt__(self, other) -> bool:
        return self.ga_time_cost + self.mvrp_time_cost + self.other_ins_time_cost < other.ga_time_cost + other.mvrp_time_cost + other.other_ins_time_cost
    
    def __str__(self) -> str:
        return f"Decision: GA: {self.ga_time_cost} MVRP: {self.mvrp_time_cost} Other: {self.other_ins_time_cost} ins: {[m.ins_id for m in self.meeting_list]}"

class AStarPath(list):
    def length(self) -> float:
        return sum(dist(self[i - 1], self[i]) for i in range(1, len(self)))
        
    def travle_time(self, acc: float, max_v: float) -> float:
        if len(self) < 2:
            return 0.0

        total_distance = self.length()
        t_acc = max_v / acc  # Time to accelerate to max velocity
        d_acc = 0.5 * acc * t_acc ** 2  # Distance covered during acceleration

        if total_distance < 2 * d_acc:  # Never reach max velocity
            t_total = (2 * total_distance / acc) ** 0.5
        else:
            d_const = total_distance - 2 * d_acc  # Distance covered at constant velocity
            t_const = d_const / max_v  # Time at constant velocity
            t_total = 2 * t_acc + t_const

        return t_total