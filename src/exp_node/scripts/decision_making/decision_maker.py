import rospy, multiprocessing, pickle
from multiprocessing import Pool, Process, Manager
from typing import List, Dict, Tuple
from itertools import chain, combinations, permutations
from decision_making.decision_making_data_structure import (
    Decision,
    DecisionMakingData,
    ExpData,
    InsData,
    Meeting,
    InspectionPoint,
)
import time
from decision_making.decision_making_voxel_map import DecisionMakingVoxelMap
from decision_making.genetic_algorithm import GA
from decision_making.mvrp import MVRP
import numpy as np

id = str
point = Tuple[float, float, float]
time_window = Tuple[float, float]
features = List[Tuple[float, float, float, rospy.Time]]
predicted_arrive_time = float

class DecisionMaker:
    def __init__(self, voxel_map_path: str) -> None:
        self.los_range = rospy.get_param("/communication_range", 10.0)
        self.inspection_time = rospy.get_param("/inspection_time", 10.0)
        voxel_map = np.load(voxel_map_path)
        self.dm_voxel_map: DecisionMakingVoxelMap = DecisionMakingVoxelMap(voxel_map)
        self.wall_start_time = rospy.Time.now().to_sec()

    def make_decision(
        self,
        bbox: List[float], 
        exp_id: str,
        exp_position: point,
        features_to_allocate: features,
        ins_inspection_points_dict: Dict[id, List[Tuple[point, time_window]]],
        is_exploration_finished: bool = False,
    ) -> List[Tuple[id, point, point, time_window, features, predicted_arrive_time]]:
        rospy.logerr("DecisionMaker.make_decision")
        time_log = time.time()
        
        # Step 0: Initialize decision-making data
        dm_data = DecisionMakingData()
        dm_data.exp_data = ExpData(exp_id, exp_position, features_to_allocate)
        dm_data.ins_data_list = []
        for ins_id, inspection_points in ins_inspection_points_dict.items():
            ins_data = InsData(
                ins_id, [InspectionPoint(ip[0], ip[1]) for ip in inspection_points]
            )
            dm_data.ins_data_list.append(ins_data)
            
        # Step 1: Sampling candidate meeting points for each inspection_pose.
        # ---Note: input decision_making_data and then return it.
        dm_data = self.dm_voxel_map.sample_meeting_points(dm_data, bbox, self.los_range, 1000, 0.5)
        # print(dm_data)
        
        # outer loop
        candidiate_decisions: List[Decision] = []
        # This combination doesn't contain empty set because it's time cost is unknown until right time axis is determined.
        ins_combinations: List[Tuple[id, ...]] = self.generate_all_combinations(
            [ins_data.id for ins_data in dm_data.ins_data_list]
        )

        lock = multiprocessing.Lock()
        manager = Manager()
        results = manager.list()
        pools = []
        # 将所有process_combination的结果汇总到Manager中
        for current_combination in ins_combinations:
            ins_permutation = list(permutations(current_combination))
            for ins_meeting_order in ins_permutation:
                p = Process(target=self.process_combination, args=(ins_meeting_order, dm_data, time_log, self.inspection_time, results, lock))
                pools.append(p)
                p.start()
        for p in pools:
            p.join()
        
        for result in results:
            if isinstance(result, Exception):
                raise result
            candidiate_decisions.extend(result)
        # for d in candidiate_decisions:
        #     rospy.logwarn(d)

        # Step 5: Choose the best decision from candidate_decision_pool and compare it with "no_meeting".
        best_decision = min(candidiate_decisions)
        if not is_exploration_finished:
            no_meeting_decision = self.calculate_no_meeting_decision(dm_data, best_decision.right_axis, self.inspection_time)
            best_decision = min(best_decision, no_meeting_decision)
            rospy.logwarn(no_meeting_decision)
            

        rospy.logwarn(f"best: {best_decision}")
        
        # Step 6: Return the best decision.
        return_list = []
        for meeting in best_decision.meeting_list:
            return_list.append(
                (
                    meeting.ins_id,
                    meeting.meeting_point,
                    meeting.ins_inspection_point,
                    meeting.time_window,
                    meeting.allocated_features,
                    meeting.predicted_arrive_time - self.wall_start_time
                )
            )
        
        print(f"decision_making time cost: {time.time() - time_log} s")
        return return_list
    
    def process_combination(self, ins_meeting_order: Tuple[id,...], dm_data: DecisionMakingData, time_log: float, inspection_time: float, results, lock):
        local_decisions = []
        decision = Decision()

        # Step 2: Get the best meeting points and corresponding 
        # time cost (exp_travel + exp_wait + ins_wait_before_meeting) through genetic algorithm.
        # ---Note: input decision_making_data and then return it.
        
        # import pdb; pdb.set_trace()
        ga = GA(dm_data, ins_meeting_order, self.dm_voxel_map, 100, 20, decision, inspection_time)
        decision = ga.genetic_algorithm(time_log)

        # Step 3: Allocate features to ins through MVRP(start time is different) 
        # and get time cost (ins_travel + ins_free_after_finish).
        # ---Note: ins_free_after_finish need to consider other ins which are not in this combination.
        # ---Note: input decision_making_data and then return decision.
        mvrp = MVRP(dm_data, decision, inspection_time, self.dm_voxel_map)
        decision = mvrp.assign_points_to_agents()
        
        # Step 4: calculate other ins time cost according to the right axis and decision.
        decision = self.calculate_other_ins_time_cost(dm_data, decision)
        
        # Step 5: Add the decision into candidate_decision_pool.
        local_decisions.append(decision)
            
        # 加锁
        with lock:
            results.append(local_decisions) 
        
        return local_decisions

    def generate_all_combinations(self, ins_ids: List[id]) -> List[Tuple[id, ...]]:
        return list(
            chain.from_iterable(
                combinations(ins_ids, r) for r in range(1, len(ins_ids) + 1)
            )
        )
    
    def calculate_no_meeting_decision(self, dm_data: DecisionMakingData, right_axis: float, inspection_time: float) -> Decision:
        decision = Decision()
        decision.ga_time_cost = 0.0
        decision.mvrp_time_cost = 0.0
        decision.other_ins_time_cost = 0.0
        decision.right_axis = right_axis
        for ins_data in dm_data.ins_data_list:
            last_inspection_point_finish_time = ins_data.inspection_points[-1].time_window[0] + inspection_time if ins_data.inspection_points[-1].time_window[0] + inspection_time > rospy.Time.now().to_sec() else rospy.Time.now().to_sec()
            if last_inspection_point_finish_time < right_axis:
                decision.other_ins_time_cost += right_axis - last_inspection_point_finish_time
        return decision
    
    def calculate_other_ins_time_cost(self, dm_data: DecisionMakingData, decision: Decision) -> Decision:
        for ins_data in dm_data.ins_data_list:
            if ins_data.id not in [meeting.ins_id for meeting in decision.meeting_list]:
                last_inspection_point_finish_time = ins_data.inspection_points[-1].time_window[0] + self.inspection_time
                if last_inspection_point_finish_time < decision.right_axis:
                    decision.other_ins_time_cost += decision.right_axis - last_inspection_point_finish_time
        return decision
