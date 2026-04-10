#! /usr/bin/env python

import rospy, time, copy, yaml, os, math
from typing import Tuple, List, Dict
from nav_msgs.msg import Path, Odometry
from msgs.srv import *
from std_msgs.msg import String, Int32, Float64
from geometry_msgs.msg import PoseStamped, Point32, Vector3
from exp_mission_executer import ExpMissionExecuter
from feature_scan import FeatureScan
from meeting_manager_data_structure import MeetingWithIns, MeetingWithGcs, InsData
import numpy as np
from typing import Dict, List
from bspline.msg import Bspline
import tf.transformations
from msgs.msg import PlanVis, Combine_bspline_inspection_pose, AnimationSingleMeet, AnimationMeetList
from geometry_msgs.msg import Point
sys.path.append(os.path.join(os.path.dirname(__file__), "../../"))
from visualizer_node.scripts.animation import ResultVisualizer
from collections import deque

class ExpMeetingManager():
    def __init__(self, exp_mission_executer: ExpMissionExecuter) -> None:
        self.exp_mission_executer: ExpMissionExecuter = exp_mission_executer
        self.drone_id=rospy.get_param("~drone_id","-1")
        
        # wall time
        self.wall_start_time: float = rospy.Time.now().to_sec()

        # decision_making
        from decision_making.decision_maker import DecisionMaker
        voxel_map_path = rospy.get_param("/voxel_map_path")
        while not rospy.has_param("/voxel_map/remap_z"):
            time.sleep(1)
        self.decision_maker = DecisionMaker(voxel_map_path)
        self.ins_meeting_list = [] # 和ins的开会列表，由 decision_maker 产生
        self.decision_making_timer = rospy.Timer(rospy.Duration(5), self.decision_making_timer_cb)

        # yaml
        self.current_dir = os.path.dirname(__file__)  
        self.prediction_file_path = os.path.join(self.current_dir, f'{self.drone_id}_prediction.yaml') 
        self.init_yaml()

        # volume
        self.explored_volume: float = 0
        self.request_volume_client = rospy.ServiceProxy("exploration_node/volume", volume)

        # feature
        self.allocated_feature_vis_pub = rospy.Publisher("/allocated_feature_vis", Path, queue_size=10) # 发布allocated feature
        self.predicted_feature_num: int = 0 # 预测的当前 bbox 的 feature 数量
        self.total_feature_num: int = 0 # 当前 bbox 的实际总 feature 数量（用于写入yaml文件）
        self.received_feature_num: int = 0  # exp 已经接收到的 feature 数量
        self.bbox_feature_num_dict: dict[int, int] = {}  # bbox_id: feature_num 字典：存每个bbox的feature数量
        from std_msgs.msg import String
        self.bbox_feature_num_sub = rospy.Subscriber("/bbox_feature_num", String, self.bbox_feature_num_cb)
        self.inspection_pose_list: list[PoseStamped] = [] # 当前扫到但没分配的 feature
        self.feature_scan = FeatureScan(self.inspection_pose_list)
        self.exp_mission_executer.feature_scan = self.feature_scan

        # exploration
        self.exp_max_vel = rospy.get_param("~exp_max_vel", 2.0)
        self.explored_surface_area: float = 0
        self.exploration_time: int = 90
        self.remained_exploration_time: int = 100

        # inspection
        self.ins_data_dict: dict[str, InsData] = {} # ins 的数据
        self.finished_inspection_poses: Path = Path()
        self.inspection_time = rospy.get_param("/inspection_time")
        self.ins_max_vel = rospy.get_param("~ins_max_vel")
        
        # ins odom
        self.ins_position_dict: dict[str, Point32] = {}
        self.ins_pose_dict: dict[str, PoseStamped] = {}

        # prediction
        self.explored_time: rospy.Duration = rospy.Duration(0)
        
        # call FUEL
        self.plan_traj_client = rospy.ServiceProxy("exploration_node/plan_traj", plan_traj)
        self.get_pos_on_bspline_client = rospy.ServiceProxy("get_pos_on_bspline", get_pos_on_bspline)
        
        # init
        self.ins_ids: list[str] = []
        self.ins_odom_sub_dict: dict[str, Odometry] = {}
        self.exp_init_server = rospy.Service("init", gcs_exp_meeting, self.gcs_init_cb)
        self.init_ins_topic="/exp_ins_meeting"
        self.init_ins_clients:dict[str, rospy.ServiceProxy] = {}
        
        # meeting
        self.new_bbox_state: bool = False
        self.exp_position_new_bbox: PoseStamped = PoseStamped() # exp 进入新 bbox 的位置，拿到新 bbox 后更新
        self.los_range = rospy.get_param("/communication_range", 10)
        self.new_bbox: List[float] = []  # the newest bbox, store the bbox's 6 coordinates
        self.whether_new_bbox: bool = False
        self.finished_exploration: bool = False
        self.finished_bbox: bool = False
        self.finish_bbox_time: rospy.Time = rospy.Time(0)
        self.message_received_ins_ids = []
        self.meeting_pose_with_gcs: PoseStamped = PoseStamped()
        self.meeting_with_ins: MeetingWithIns = MeetingWithIns()
        self.exp_ins_topic="/exp_ins_meeting"
        self.exp_ins_clients_dict: dict [str, rospy.ServiceProxy] = {}
        self.gcs_exp_server = rospy.Service("gcs_exp_meeting", gcs_exp_meeting, self.gcs_exp_meeting_cb)
        self.exp_gcs_client = rospy.ServiceProxy("/exp_gcs_meeting", exp_gcs_meeting)
        self.meeting_timer = rospy.Timer(rospy.Duration(1), self.meeting_timer_cb)
        self.meet_status_list: List[float] = [] # 存储每次开会的时间，animation_node请求时返回最新的开会时间
        
        # state machine
        self.state = "init"
        self.timer = rospy.Timer(rospy.Duration(1), self.timer_cb)
        
        # odom
        self.position = self.exp_mission_executer.position
        
        # visualization
        self.meeting_pose_vis_pub = rospy.Publisher("/meeting_pose_vis", PoseStamped, queue_size=10)
        self.debug_path_pub = rospy.Publisher("/path_vis", Path, queue_size=10)
        self.debug_inspection_point_pub = rospy.Publisher("/inspection_point_vis", Path, queue_size=10) # set for visualizing the inspection poses (pink), currently not used
        self.plan_vis_pub = rospy.Publisher("/plan_vis", PlanVis, queue_size=10)
        self.decision_text_pub = rospy.Publisher("/decision_text", String, queue_size=10)
        
        # animation
        # TODO: the fequency of the timer must be the same as the frequency of 
        # the timer in animation.py, otherwise the status will not be conituously updated
        self.animation_status_server = rospy.Service("/"+self.drone_id+"/status_animation", animation_status, self.animation_status_cb)
        self.animation_meet_server = rospy.Service("/meet_animation", animation_meet, self.animation_meet_cb)
        self.animation_meet_data = deque(maxlen=1)


        # record
        from std_msgs.msg import String
        self.waste_time = None
        self.record_csv_pub = rospy.Publisher("/record_csv", String, queue_size=10)

    # region timer
    def animation_status_cb(self, req):
        if req.robot_id == self.drone_id:
            if self.meet_status_list != []:
                status = "2"
                time_info: str[float] = str(self.meet_status_list[-1])
                self.meet_status_list.pop()
            elif self.state == "working":
                status = "0"
                time_info = "None"
            elif self.state == "go to meeting with ins":
                status = "1"
                time_info = "None"
            else:
                status = "0"
                time_info = "None"
            status_string = f"{self.drone_id},{status},{time_info}"
            rospy.logerr(f"status_string: {status_string}")
            resp = animation_statusResponse()
            resp.status = status_string
            return resp
        else:
            return
        
    
    def animation_meet_cb(self, req):
        if req.exp_id == self.drone_id:
            if len(self.animation_meet_data) > 0:
                meet_result = self.animation_meet_data.pop()
                resp = animation_meetResponse()
                meet_data_list = AnimationMeetList()
                for meet_data in meet_result:
                    meet_item = AnimationSingleMeet()
                    meet_item.ins_id = meet_data[0]
                    meet_item.feature_num = meet_data[1]
                    meet_item.meet_time = meet_data[2]
                    meet_data_list.meet_list.append(meet_item)
                    # rospy.logerr(f"meet data is: {(meet_data[0], meet_data[1], meet_data[2])}")
                resp.meet_list = meet_data_list 
                return resp
            
            else:
                return AnimationMeetList()
        else:
            return AnimationMeetList()
        
        

    def timer_cb(self, event):        
        if self.state == "init":
            pass
        elif self.state == "init_ins":
            self.init_ins()
            self.change_state_to("working")
            self.exp_mission_executer.go_to_exploration(self.exp_position_new_bbox)
        elif self.state == "new bbox":
            # 切换到 new bbox 状态
            self.new_bbox_state = True
            # 计算与 ins 的 meeting points
            ins_new_bbox_pose_dict = self.compute_ins_data_new_bbox(self.new_bbox, self.position)
            for id, ins_data in self.ins_data_dict.items():
                meeting_point = (ins_data.finish_pose.pose.position.x, ins_data.finish_pose.pose.position.y, ins_data.finish_pose.pose.position.z)
                meeting_with_ins = MeetingWithIns(id, meeting_point, (-float("inf"), float("inf")), [])
                meeting_with_ins.go_to_pose = ins_new_bbox_pose_dict[id]
                self.ins_meeting_list.append(meeting_with_ins)
            
            # 进入 working 状态进行开会，完成所有会议后 go to exploration
            self.change_state_to("working")
            
        elif self.state == "go to meeting with gcs":
            rospy.logerr(self.drone_id + ": try_to_communicate_with_gcs()")
            if self.try_to_communicate_with_gcs():
                self.change_state_to("wait reply from gcs")
        elif self.state == "reach meeting with gcs":
            rospy.logerr(self.drone_id + ": try_to_communicate_with_gcs()")
            if self.try_to_communicate_with_gcs():
                self.change_state_to("wait reply from gcs")
        elif self.state == "wait reply from gcs":
            pass
        
        elif self.state == "go to meeting with ins":
            rospy.logerr(self.drone_id + ": try_to_communicate_with_ins()")
            if self.try_to_communicate_with_ins(self.meeting_with_ins):
                self.change_state_to("working")
            else:
                self.check_ins_meeting_time_window(self.meeting_with_ins)
        elif self.state == "reach meeting with ins":
            rospy.logerr(self.drone_id + ": try_to_communicate_with_ins()")
            if self.try_to_communicate_with_ins(self.meeting_with_ins):
                self.change_state_to("working")
            else:
                self.check_ins_meeting_time_window(self.meeting_with_ins)
        
        elif self.state == "working":
            self.finished_exploration=self.exp_mission_executer.finish_bbox
            
            if self.new_bbox_state and self.ins_meeting_list == []:
                self.new_bbox_state = False
                self.exp_mission_executer.go_to_exploration(self.exp_position_new_bbox)
            # if self.finished_exploration != True:   
            #     self.explored_volume, self.explored_surface_area = self.request_volume()
    
    def meeting_timer_cb(self, event):
        if self.state != "working":
            return

        if self.meeting_pose_with_gcs.header.stamp - rospy.Time.now() < rospy.Duration(10):
            self.change_state_to("go to meeting with gcs")
            self.record_csv_pub.publish(f"meeting_count,{self.drone_id}-gcs")
            self.refine_meeting_pose_with_gcs()
            
            # visualization
            meeting_pose_vis = copy.deepcopy(self.meeting_pose_with_gcs)
            meeting_pose_vis.header.frame_id = self.drone_id + " with g"
            self.meeting_pose_vis_pub.publish(meeting_pose_vis)

            self.exp_mission_executer.go_to_meeting_pose(self.meeting_pose_with_gcs)

        if self.ins_meeting_list == []:
            return 
        else:
            self.meeting_with_ins: MeetingWithIns = self.ins_meeting_list[0]
            self.change_state_to("go to meeting with ins")
            self.record_csv_pub.publish(f"meeting_count,{self.drone_id}-{self.meeting_with_ins.ins_id}")
            
            # visualization
            meeting_pose_vis = copy.deepcopy(self.meeting_with_ins.meeting_pose)
            meeting_pose_vis.header.frame_id = self.drone_id + " with " + self.meeting_with_ins.ins_id
            self.meeting_pose_vis_pub.publish(meeting_pose_vis)
            
            self.exp_mission_executer.go_to_meeting_pose(self.meeting_with_ins.meeting_pose)

    def decision_making_timer_cb(self, event):
        if self.state != "working":
            rospy.logwarn(self.drone_id + ": decision_making: state != working")
            return
        if self.inspection_pose_list == []:
            rospy.logwarn(self.drone_id + ": decision_making: inspection_pose_list == []")
            return
        if self.ins_meeting_list != []:
            rospy.logwarn(self.drone_id + ": decision_making: ins_meeting_list != []")
            return
        
        ins_inspection_points_dict = {}
        for ins_id, ins_data in self.ins_data_dict.items():
            ins_inspection_points_dict[ins_id] = ins_data.point_tw_list

        rospy.logwarn(ins_inspection_points_dict)
        
        start_time = time.time()
        result = self.decision_maker.make_decision(
            self.new_bbox,
            self.drone_id,
            (self.position.x, self.position.y, self.position.z),
            [(f.pose.position.x, f.pose.position.y, f.pose.position.z, f.header.stamp) for f in self.inspection_pose_list],
            ins_inspection_points_dict,
            self.finished_exploration
        )
        decision_time = time.time() - start_time
        rospy.logwarn(f"{self.drone_id}: decision_making time: {decision_time}")
        rospy.logwarn(f"{self.drone_id}: decision_making result: {result}")
        
        decision_text: String = ""  
        animation_meet_list = []

        # 如果决定要去开会，那就清空分完的 feature
        if result != []:
            allocated_features_stamps = [feature for _, _, _, _, features, _ in result for feature in features]
            # self.inspection_pose_list = [f for f in self.inspection_pose_list if f.header.stamp not in allocated_features_stamps]
            for _ in range(len(allocated_features_stamps)):
                self.inspection_pose_list.pop(0)
            
            # visualization of exp_meeting_point with ins
            plan_vis = PlanVis()
            plan_vis.msg = f"{self.drone_id},meeting with ins"
            path = Path()
            path.header.frame_id = "world"
            self_pose = PoseStamped()
            self_pose.header.frame_id = "world"
            self_pose.pose.position.x = self.position.x
            self_pose.pose.position.y = self.position.y
            self_pose.pose.position.z = self.position.z
            self_pose.pose.orientation.w = 1
            path.poses.append(self_pose)

            # turn every element in result into MeetingWithIns, and store them in self.ins_meeting_list
            for meeting_data in result:
                id = meeting_data[0]
                point = meeting_data[1]
                ins_inspection_point = meeting_data[2]
                time_window = meeting_data[3]
                features = meeting_data[4]
                prediction_arrive_time: float = meeting_data[5]
                
                if len(features) == 0:
                    continue
                
                animation_single_meet = (id.split("_")[1], len(features), prediction_arrive_time)
                animation_meet_list.append(animation_single_meet)

                decision_text += f"{id}:{len(features)},"

                meeting_with_ins: MeetingWithIns = MeetingWithIns(id, point, time_window, features)
                    
                self.ins_meeting_list.append(meeting_with_ins)

                rospy.logwarn(f"{id}: ins_inspection_pose: {ins_inspection_point}")
                # visualization of ins_inspection_points
                ins_inspection_pose = PoseStamped()
                ins_inspection_pose.header.frame_id = "world"
                ins_inspection_pose.pose.position.x = ins_inspection_point[0]
                ins_inspection_pose.pose.position.y = ins_inspection_point[1]
                ins_inspection_pose.pose.position.z = ins_inspection_point[2]
                ins_inspection_pose.pose.orientation.w = 1
                plan_vis.ins_inspection_poses.append(ins_inspection_pose)

                # visualization of path between points
                pose = PoseStamped()
                pose.header.frame_id = "world"
                pose.pose.position.x = point[0]
                pose.pose.position.y = point[1]
                pose.pose.position.z = point[2]
                pose.pose.orientation.w = 1
                path.poses.append(pose)
                
                pose.header.frame_id = f"{id}"
                plan_vis.poses.append(pose)
                
            plan_vis.paths.append(path)
            self.plan_vis_pub.publish(plan_vis)
        self.decision_text_pub.publish(decision_text)
        if animation_meet_list == []:
            animation_meet_list = [("0", 0, rospy.Time.now().to_sec()-self.wall_start_time)]
        self.animation_meet_data.append(animation_meet_list)
                
            
    # end region

    # region init

    def init_yaml(self):
        data = {}
        self.meeting_count = 0
        with open(self.prediction_file_path, 'w') as f:
                yaml.dump(data, f)

    def bbox_feature_num_cb(self, msg: String) -> None:
        lines = msg.data.split("\n")
        for line in lines:
            if line == "":
                continue
            parts = line.split(":")
            bbox_id = int(parts[0])
            feature_num = int(parts[1])
            self.bbox_feature_num_dict[bbox_id] = feature_num
    
    def gcs_init_cb(self, req: gcs_exp_meetingRequest) -> gcs_exp_meetingResponse:
        if self.bbox_feature_num_dict == {}:
            return gcs_exp_meetingResponse(False)
        self.ins_ids = req.ins_ids
        self.new_bbox = [float(i) for i in req.bbox[0].split(",")]
        self.bbox_id: int = int(req.bbox_id)
        self.total_feature_num = self.bbox_feature_num_dict[self.bbox_id]
        self.exp_mission_executer.receive_bbox(req.bbox)
        self.feature_scan.receive_bbox(req.bbox)
        self.meeting_pose_with_gcs = req.meeting_pose
        self.meeting_pose_with_gcs.header.stamp = rospy.Time(1e11)
        self.explored_time = rospy.Duration(0)
        # self.meeting_pose_with_gcs = self.generate_meeting_pose_with_gcs(req.meeting_pose)
        # for ins_id in self.ins_ids:
        #     self.ins_finish_time_dict[ins_id] = rospy.Time(0)
        # need self.meeing_pose_with_ins to generate the pose of ins in the new bbox and tell ins
        # TODO: when go to a new bbox, initialize the ins data and send it to decision maker, let to decide when to meet with ins
        self.change_state_to("init_ins")
        return gcs_exp_meetingResponse(True)

    def init_ins(self):
        self.initialize_init_ins_clients()
        self.initialize_exp_ins_clients()
        self.init_odom_sub()
        self.let_ins_go_to_new_bbox()
                
    def let_ins_go_to_new_bbox(self):
        ins_new_bbox_pose_dict = self.compute_ins_data_new_bbox(self.new_bbox, self.position)
        for ins_id in self.ins_ids:
            req=exp_ins_meetingRequest()
            req.exp_id=self.drone_id
            req.bbox = f"{self.new_bbox[0]},{self.new_bbox[1]},{self.new_bbox[2]},{self.new_bbox[3]},{self.new_bbox[4]},{self.new_bbox[5]}"
            req.meeting_pose = ins_new_bbox_pose_dict[ins_id]
            req.inspection_poses=Path()
            
            # try to communicate with ins
            self.init_ins_clients[ins_id].wait_for_service()
            can_communicate: bool = self.exp_mission_executer.check_los(ins_id)
            while not can_communicate:
                time.sleep(1)
                can_communicate: bool = self.exp_mission_executer.check_los(ins_id)
            resp: exp_ins_meetingResponse=self.init_ins_clients[ins_id].call(req)
            while resp.success==False:
                time.sleep(1)
                resp=self.init_ins_clients[ins_id].call(req)
        
            # init or update ins_data
            self.init_new_bbox_ins_data_dict(ins_new_bbox_pose_dict)

    def initialize_init_ins_clients(self):
        for ins_id in self.ins_ids:
            topic = "/" + ins_id + self.init_ins_topic
            self.init_ins_clients[ins_id] = rospy.ServiceProxy(topic, exp_ins_meeting)

    def initialize_exp_ins_clients(self):
        for ins_id in self.ins_ids:
            topic = "/" + ins_id + self.exp_ins_topic 
            self.exp_ins_clients_dict[ins_id] = rospy.ServiceProxy(topic, exp_ins_meeting)

    def init_new_bbox_ins_data_dict(self, ins_new_bbox_pose_dict: Dict[str, PoseStamped]):
        for ins_id in self.ins_ids:
            ins_data: InsData = InsData()
            pose: PoseStamped = ins_new_bbox_pose_dict[ins_id] # 此处 pose 为 ins 刚进入 new bbox 还未分到 feature 时的停留位置
            ins_data.ins_bsp_list = [pose]
            self.ins_data_dict[ins_id] = ins_data

    # endregion
    
    # region meeting process

    def try_to_communicate_with_ins(self, meeting_with_ins: MeetingWithIns) -> bool: 
        ins_id = meeting_with_ins.ins_id
        ins_data = self.ins_data_dict[ins_id]
        client = self.exp_ins_clients_dict[ins_id]
        
        try:
            client.wait_for_service(timeout=10)
        except:
            # ins fail，撤回分配的 feature
            del self.ins_data_dict[ins_id]
            rospy.logerr(f"{self.drone_id}: {ins_id} failed")
            for pose in meeting_with_ins.allocated_features_Path.poses:
                self.inspection_pose_list.append(pose)
            self.ins_meeting_list.pop(0)
            self.exp_mission_executer.meeting_end()
            self.change_state_to("working")
            return False
        
        can_communicate: bool = self.exp_mission_executer.check_los(ins_id)
        if can_communicate:
            client.wait_for_service()
            req: exp_ins_meetingRequest = exp_ins_meetingRequest()
            req.exp_id = self.drone_id
            
            # stamp = 0 表示无效点，ins 不会去
            req.meeting_pose = meeting_with_ins.go_to_pose
            
            if not self.new_bbox_state:
                req.inspection_poses = meeting_with_ins.allocated_features_Path
                new_ins_data = self.plan_inspection_traj(ins_data, meeting_with_ins.allocated_features_Path)
            else:
                req.inspection_poses = Path()
                new_ins_data: InsData = InsData()
                new_ins_data.ins_bsp_list = [meeting_with_ins.go_to_pose]

            req.combine_list = new_ins_data.combine_list

            # try to communicate with ins
            resp: exp_ins_meetingResponse = client.call(req)
            if resp.success:
                rospy.logerr(self.drone_id + ": try_to_communicate_with_ins() success")

                # if meeeting success and not new_bbox, send allocated features to visualizer; else, do not send
                if not self.new_bbox_state:
                    path = Path()
                    path.header.frame_id = ins_id
                    for pose in meeting_with_ins.allocated_features_Path.poses:
                        pose.header.frame_id = ins_id
                        path.poses.append(pose)
                    self.allocated_feature_vis_pub.publish(path)
                        
                # if meeting success, update ins_data and receive finished inspection poses
                self.ins_data_dict[ins_id] = new_ins_data
                
                for pose in resp.finished_inspection_poses.poses:
                    self.finished_inspection_poses.poses.append(pose)
                    self.received_feature_num += 1
                    
                self.ins_meeting_list.pop(0) # remove this meeting from meeting_with_ins
                self.exp_mission_executer.meeting_end()

                # 加入self.meet_status_list
                self.meet_status_list.append(rospy.Time.now().to_sec()-self.wall_start_time)
                return True
            else:
                return False
        else:
            return False
    
    def check_ins_meeting_time_window(self, meeting_with_ins: MeetingWithIns):
        rospy.logwarn(f"{self.drone_id}: time_window: {meeting_with_ins.time_window}")
        if meeting_with_ins.time_window[1] < rospy.Time.now().to_sec() - 10:
            # 开会失败，撤回分配的 feature
            rospy.logerr(f"{self.drone_id}: Meeting failed")
            for pose in meeting_with_ins.allocated_features_Path.poses:
                self.inspection_pose_list.append(pose)
            self.ins_meeting_list.pop(0)
            self.exp_mission_executer.meeting_end()
            self.change_state_to("working")
        
    def gcs_exp_meeting_cb(self, req: gcs_exp_meetingRequest) -> gcs_exp_meetingResponse:
        bbox = req.bbox
        if bbox==["no bbox left"]:
            pass
        elif bbox==["None"]:
            pass
        else:
            self.exp_mission_executer.receive_bbox(req.bbox)
            self.feature_scan.receive_bbox(req.bbox)
            self.finished_exploration=False
            self.finished_bbox=False
            self.new_bbox=[float(i) for i in bbox[0].split(",")]
            self.whether_new_bbox=True
            # need self.meeing_pose_with_ins to generate the pose of ins in the new bbox and tell ins
            # TODO: when go to a new bbox, initialize the ins data and send it to decision maker, let to decide when to meet with ins
            self.change_state_to("new bbox")
            self.bbox_id=req.bbox_id
            self.total_feature_num=self.bbox_feature_num_dict[self.bbox_id]
            self.received_feature_num: int = 0
            self.feature_scan.scaned_num = 0
            self.explored_time = rospy.Duration(0)
            self.explored_surface_area = 0
            with open(self.prediction_file_path, 'r') as f:
                content = yaml.load(f, Loader=yaml.FullLoader)
                if self.bbox_id not in content.keys():
                    content[self.bbox_id] = {}
                if self.meeting_count not in content[self.bbox_id].keys():
                    content[self.bbox_id][self.meeting_count] = {}
                current_time = rospy.Time.now()
                volume_predicted_time: rospy.Duration = req.meeting_pose.header.stamp - current_time
                content[self.bbox_id][self.meeting_count]["volume_predicted_time"] = volume_predicted_time.to_sec()
            with open(self.prediction_file_path, 'w') as f:
                yaml.dump(content, f, default_flow_style=False, sort_keys=False)
            self.meeting_pose_with_gcs = req.meeting_pose
            self.exp_mission_executer.meeting_end()
            return gcs_exp_meetingResponse(True)
        
        self.meeting_pose_with_gcs = req.meeting_pose
        self.exp_mission_executer.meeting_end()
        self.change_state_to("working")
        return gcs_exp_meetingResponse(True)
    
    def try_to_communicate_with_gcs(self) -> bool:
        self.scaned_feature_num: int = self.feature_scan.scaned_num
        if self.finished_exploration != True:   
            self.explored_volume, self.explored_surface_area = self.request_volume()
        else:
            bbox_volume = (self.new_bbox[3] - self.new_bbox[0]) * (self.new_bbox[4] - self.new_bbox[1]) * (self.new_bbox[5] - self.new_bbox[2])
            self.explored_volume = bbox_volume
        rospy.logwarn(self.drone_id + ": try_to_communicate_with_gcs()")
        can_communicate: bool = self.exp_mission_executer.check_los("gcs")
        if can_communicate:
            self.exp_gcs_client.wait_for_service()
            req = exp_gcs_meetingRequest()
            req.exp_id = self.drone_id
            req.finished_bbox = False
            if self.finished_exploration == True and self.inspection_pose_list == []:
                req.finished_bbox = True
            req.finished_inspection_poses=self.finished_inspection_poses
            original_predicted_finish_time = self.predict_finish_time(self.explored_volume)
            if original_predicted_finish_time > rospy.Duration(600):
                req.predicted_finish_time = rospy.Duration(600)
            else:
                req.predicted_finish_time = original_predicted_finish_time
            resp: exp_gcs_meetingResponse = self.exp_gcs_client.call(req)
            if resp.success:
                
                with open(self.prediction_file_path, 'r') as f:
                    content = yaml.load(f, Loader=yaml.FullLoader)
                self.meeting_count += 1
                if self.bbox_id not in content.keys():
                    content[self.bbox_id] = {}
                content[self.bbox_id][self.meeting_count] = {}
                content[self.bbox_id][self.meeting_count]["scaned_feature_num"] = self.scaned_feature_num
                content[self.bbox_id][self.meeting_count]["received_feature_num"] = self.received_feature_num
                content[self.bbox_id][self.meeting_count]["predicted_feature_num"] = self.predicted_feature_num
                content[self.bbox_id][self.meeting_count]["total_feature_num"] = self.total_feature_num
                content[self.bbox_id][self.meeting_count]["explored_volume"] = self.explored_volume
                content[self.bbox_id][self.meeting_count]["explored_surface_area"] = self.explored_surface_area
                content[self.bbox_id][self.meeting_count]["explored_time"] = self.explored_time.to_sec()
                content[self.bbox_id][self.meeting_count]["whether_finished_bbox"] = self.finished_bbox
                if self.finished_bbox:
                    finish_time = self.finish_bbox_time
                    current_time = rospy.Time.now()
                    wait_time = current_time - finish_time
                    content[self.bbox_id][self.meeting_count]["current_bbox_finish_time"] = (original_predicted_finish_time).to_sec()
                    content[self.bbox_id][self.meeting_count]["wait_time"] = wait_time.to_sec()
                else:
                    content[self.bbox_id][self.meeting_count]["predicted_finish_time"] = (req.predicted_finish_time).to_sec()
                with open(self.prediction_file_path, 'w') as f:
                    yaml.dump(content, f, default_flow_style=False, sort_keys=False)
                
                self.finished_inspection_poses = Path()
                return True
            else:
                return False
        else:
            return False
        
    def request_volume(self):
        self.request_volume_client.wait_for_service()
        req = volumeRequest()
        req.exp_id = self.drone_id
        resp = self.request_volume_client.call(req)
        explored_volume = resp.volume
        explored_surface_area: float = resp.surface_area
        return explored_volume, explored_surface_area
                
    def predict_finish_time(self, explored_volume) -> rospy.Duration:
        current_time = rospy.Time.now()
        bbox_start_time = self.exp_mission_executer.bbox_start_time
        self.explored_time = current_time - bbox_start_time
        
        # send the total time of the current bbox to gcs
        if self.finished_bbox == True:
            total_time: rospy.Duration = self.finish_bbox_time - self.exp_mission_executer.bbox_start_time
            return total_time
        
        else:
            if self.finished_exploration == True:
                # 找到最早结束的inspector的time
                predict_finish_time: rospy.Time = self.ins_data_dict[self.ins_ids[0]].finish_time
                for ins_id in self.ins_data_dict.keys():
                    if self.ins_data_dict[ins_id].finish_time < predict_finish_time:
                        predict_finish_time = self.ins_data_dict[ins_id].finish_time
                if predict_finish_time < current_time:
                    predict_finish_time = current_time
                current_position: Point32 = self.position
                for ins_id in self.ins_data_dict.keys():
                    ins_pose: PoseStamped = self.ins_data_dict[ins_id].finish_pose
                    distance = ((ins_pose.pose.position.x - current_position.x) ** 2 + (ins_pose.pose.position.y - current_position.y) ** 2) ** 0.5
                    travel_time = distance / self.exp_max_vel
                    current_position = ins_pose.pose.position
                    predict_finish_time += rospy.Duration(travel_time)
                remaining_time: rospy.Duration = predict_finish_time - current_time
                if remaining_time < rospy.Duration(50):
                    remaining_time = rospy.Duration(50)
                return remaining_time
            else:
                # bbox_volume: int = (self.new_bbox[3] - self.new_bbox[0]) * (self.new_bbox[4] - self.new_bbox[1]) * (self.new_bbox[5] - self.new_bbox[2])
                # passed_time: rospy.Duration = self.explored_time
                # predicted_total_time: rospy.Duration = rospy.Duration((bbox_volume / explored_volume)*passed_time.to_sec())
                # remaining_time: rospy.Duration = predicted_total_time - passed_time
                # if remaining_time < rospy.Duration(50):
                #     remaining_time = rospy.Duration(50)
                # return remaining_time
                if explored_volume == 0:
                    return rospy.Duration(50)
                density: int = self.scaned_feature_num / explored_volume
                bbox_volume: int = (self.new_bbox[3] - self.new_bbox[0]) * (self.new_bbox[4] - self.new_bbox[1]) * (self.new_bbox[5] - self.new_bbox[2])
                self.predicted_feature_num: int = math.floor(density*bbox_volume)
                if self.scaned_feature_num >= self.predicted_feature_num or self.scaned_feature_num == 0:
                    passed_time: rospy.Duration = self.explored_time
                    predicted_total_time: rospy.Duration = rospy.Duration((bbox_volume / explored_volume)*passed_time.to_sec())
                    remaining_time: rospy.Duration = predicted_total_time - passed_time
                if self.scaned_feature_num < self.predicted_feature_num:
                    passed_time: rospy.Duration = self.explored_time
                    if self.received_feature_num == 0:
                        remaining_time: rospy.Duration = rospy.Duration(self.remained_exploration_time)
                    else:
                        predicted_total_time: rospy.Duration = rospy.Duration((self.predicted_feature_num / self.received_feature_num)*passed_time.to_sec())
                        remaining_time: rospy.Duration = predicted_total_time - passed_time
                if remaining_time < rospy.Duration(50):
                    remaining_time = rospy.Duration(50)
                return remaining_time
    
    # endregion
    
    # region refine meeting pose
    
    def refine_meeting_pose_with_gcs(self):
        self.meeting_pose_with_gcs.pose.position.z += 0.5
        # bbox = copy.deepcopy(self.exp_mission_executer.bbox_list[0])
        # bbox = [bbox[0]+1, bbox[1]+1, bbox[2]+1, bbox[3]-1, bbox[4]-1, bbox[5]-1]
        # guide_point = [self.meeting_pose_with_gcs.pose.position.x, self.meeting_pose_with_gcs.pose.position.y, self.meeting_pose_with_gcs.pose.position.z]
        # point = [self.position.x, self.position.y, self.position.z]
        # meeting_point =  self.project_point(bbox, point, guide_point)
        # meeting_pose = copy.deepcopy(self.meeting_pose_with_gcs)
        # meeting_pose.pose.position.x = meeting_point[0]
        # meeting_pose.pose.position.y = meeting_point[1]
        # meeting_pose.pose.position.z = meeting_point[2]
        # self.meeting_pose_with_gcs = meeting_pose
        
    # endregion
    
    # region compute meeting pose

    def compute_ins_data_new_bbox(self, bbox, exp_current_position) -> Dict[str,PoseStamped]:
        d=1
        point1=[float(bbox[0])+d,float(bbox[1])+d,float(bbox[2])+d]
        point2=[float(bbox[0])+d,float(bbox[4])-d,float(bbox[2])+d]
        point3=[float(bbox[3])-d,float(bbox[1])+d,float(bbox[2])+d]
        point4=[float(bbox[3])-d,float(bbox[4])-d,float(bbox[2])+d]
        point_list=[point1,point2,point3,point4]
        # 找出距离上一个meeting点最近的顶点
        min_distance=float("inf")
        nearest_vertex=PoseStamped() # 这个bbox四个角中距离上一个meeting点最近的角
        for i in range(len(point_list)):
            distance=((point_list[i][0]-exp_current_position.x)**2+(point_list[i][1]-exp_current_position.y)**2)**0.5
            if distance<min_distance:
                min_distance=distance
                nearest_vertex.pose.position.x=point_list[i][0]
                nearest_vertex.pose.position.y=point_list[i][1]
                nearest_vertex.pose.position.z=point_list[i][2]
        
        # 产生exp在新bbox中的进入点
        self.exp_position_new_bbox: PoseStamped = nearest_vertex
        
        # 产生ins在新bbox中的进入点，ins在新bbox中的进入点与exp在新bbox中的进入点相同
        ins_new_bbox_pose_dict: dict[str,PoseStamped] = {}
        for ins_id in self.ins_ids:
            pose=PoseStamped()
            pose.pose.position.x = nearest_vertex.pose.position.x
            pose.pose.position.y = nearest_vertex.pose.position.y
            pose.pose.position.z = nearest_vertex.pose.position.z
            pose.header.stamp = rospy.Time.now()
            ins_new_bbox_pose_dict[ins_id]=pose
        return ins_new_bbox_pose_dict
    
    def predict_left_inspection_time(self):
        current_time: rospy.Time = rospy.Time.now() 
        bbox_start_time: rospy.Time = self.exp_mission_executer.bbox_start_time
        left_inspection_time: rospy.Duration = rospy.Duration(0)
        left_feature: int = len(self.inspection_pose_list)
        already_inspected_feature: int = self.received_feature_num
        already_inspected_time: rospy.Duration = current_time - bbox_start_time
        left_inspection_time = rospy.Duration(already_inspected_time.to_sec() * (left_feature / already_inspected_feature))
        return left_inspection_time
        
    # endregion

    # region algorithm
    
    def change_state_to(self, state: str) -> None:
        rospy.logwarn(self.drone_id + ": #[" + state + "]#")
        self.state = state
        if state != "working" and self.waste_time == None:
            self.waste_time = rospy.Time.now()
        elif state == "working" and self.waste_time != None:
            self.record_csv_pub.publish(f"waste_time,{self.drone_id},{str(rospy.Time.now().to_sec() - self.waste_time.to_sec())}")
            self.waste_time = None
    
    def project_point(self, bbox, point, guide_point):
        min_coord = bbox[:3]
        max_coord = bbox[3:]
        
        # bbox的四条纵向棱的顶点
        edges = [
            ([min_coord[0], min_coord[1], min_coord[2]], [min_coord[0], min_coord[1], max_coord[2]]),
            ([min_coord[0], max_coord[1], min_coord[2]], [min_coord[0], max_coord[1], max_coord[2]]),
            ([max_coord[0], min_coord[1], min_coord[2]], [max_coord[0], min_coord[1], max_coord[2]]),
            ([max_coord[0], max_coord[1], min_coord[2]], [max_coord[0], max_coord[1], max_coord[2]])
        ]
        
        def distance_to_edge(edge, guide_point):
            p1, p2 = edge
            p1 = np.array(p1)
            p2 = np.array(p2)
            gp = np.array(guide_point)
            # 向量的起点到guide_point的向量
            vec1 = gp - p1
            # 边的向量
            vec2 = p2 - p1
            # 投影长度
            projection_length = np.dot(vec1, vec2) / np.linalg.norm(vec2)
            projection_length = max(0, min(np.linalg.norm(vec2), projection_length))
            # 投影点
            projection_point = p1 + (projection_length / np.linalg.norm(vec2)) * vec2
            return np.linalg.norm(gp - projection_point)
        
        # 找出距离guide_point最近的棱
        nearest_edge = min(edges, key=lambda edge: distance_to_edge(edge, guide_point))
        
        # 确定棱对应的两个面
        p1, p2 = nearest_edge
        if p1[0] == p2[0]:
            faces = [0, 1] if p1[0] == min_coord[0] else [2, 3]
        else:
            faces = [4, 5]
        
        # 确定两个面
        def project_to_face(face, point):
            x, y, z = point
            if face == 0:
                return [min_coord[0], y, z]
            elif face == 1:
                return [max_coord[0], y, z]
            elif face == 2:
                return [x, min_coord[1], z]
            elif face == 3:
                return [x, max_coord[1], z]
            elif face == 4:
                return [x, y, min_coord[2]]
            elif face == 5:
                return [x, y, max_coord[2]]
        
        # 投影到最近的面
        projected_point_1 = project_to_face(faces[0], point)
        projected_point_2 = project_to_face(faces[1], point)
        
        dist_1 = np.linalg.norm(np.array(projected_point_1) - np.array(guide_point))
        dist_2 = np.linalg.norm(np.array(projected_point_2) - np.array(guide_point))
        
        projected_point =  projected_point_1 if dist_1 < dist_2 else projected_point_2
        
        # 如果投影点超出了los_range, 则将投影点移动到los_range的位置
        distance_to_guide_point = np.linalg.norm(np.array(projected_point) - np.array(guide_point))
        if distance_to_guide_point > self.los_range:
            projected_point:np.ndarray = guide_point + (self.los_range - 1) * (np.array(projected_point) - np.array(guide_point)) / distance_to_guide_point
            projected_point = projected_point.tolist()
        return projected_point
    
    # endregion
    
    # region call FUEL
    
    def plan_inspection_traj(self, ins_data: InsData, inspection_poses: Path) -> InsData:
        inspection_poses = copy.deepcopy(inspection_poses)
        inspection_poses.poses.insert(0, ins_data.finish_pose)
        start_time = ins_data.finish_time if ins_data.finish_time > rospy.Time.now() else rospy.Time.now()
        new_ins_data = InsData()

        if len(inspection_poses.poses) == 1:
            new_ins_data.ins_bsp_list.append(inspection_poses.poses[0])
            rospy.logerr(f"{self.drone_id}: inspection_poses only have one pose.")
            return new_ins_data
        
        for i in range(len(inspection_poses.poses) - 1):
            
            start_pose: PoseStamped = inspection_poses.poses[i]
            end_pose: PoseStamped = inspection_poses.poses[i + 1]
            distance_square = (start_pose.pose.position.x - end_pose.pose.position.x) ** 2 + (start_pose.pose.position.y - end_pose.pose.position.y) ** 2 + (start_pose.pose.position.z - end_pose.pose.position.z) ** 2
            if distance_square < 0.1:
                new_ins_data.ins_bsp_list.append(end_pose)
                start_time += rospy.Duration(self.inspection_time) + rospy.Duration(0.5)
                continue

            success, bspline, traj_duration = self.plan_traj(start_time, start_pose, end_pose)

            while not success:
                rospy.sleep(1)
                success, bspline, traj_duration = self.plan_traj(start_time, start_pose, end_pose)

            current_end_point: Point = bspline.pos_pts[-1]
            distance_square = (current_end_point.x - end_pose.pose.position.x) ** 2 + (current_end_point.y - end_pose.pose.position.y) ** 2 + (current_end_point.z - end_pose.pose.position.z) ** 2
            while distance_square > 0.5:
                rospy.logerr(f"{self.drone_id}: distance between end point and bspline end point is too large.")
                bspline.end_time = start_time + rospy.Duration(traj_duration)
                start_time += rospy.Duration(traj_duration + 1) # TODO: +3 ???
                new_ins_data.ins_bsp_list.append(bspline)

                start_pose.pose.position = current_end_point
                success, bspline, traj_duration = self.plan_traj(start_time, start_pose, end_pose)
                while not success:
                    rospy.sleep(1)
                    success, bspline, traj_duration = self.plan_traj(start_time, start_pose, end_pose)
                current_end_point: Point = bspline.pos_pts[-1]
                distance_square = (current_end_point.x - end_pose.pose.position.x) ** 2 + (current_end_point.y - end_pose.pose.position.y)
                rospy.sleep(1)

            bspline.end_time = start_time + rospy.Duration(traj_duration)
            start_time += rospy.Duration(traj_duration + self.inspection_time) + rospy.Duration(0.5)
            new_ins_data.ins_bsp_list.append(bspline)
            new_ins_data.ins_bsp_list.append(end_pose)

        return new_ins_data
    
    def plan_traj(self, start_time: rospy.Time, start_pose: PoseStamped, end_pose: PoseStamped):
        start_pos = Vector3(start_pose.pose.position.x, start_pose.pose.position.y, start_pose.pose.position.z)
        end_pos = Vector3(end_pose.pose.position.x, end_pose.pose.position.y, end_pose.pose.position.z)
        start_yaw = Vector3(0, 0, 0)
        start_yaw.x = tf.transformations.euler_from_quaternion([
            start_pose.pose.orientation.x,
            start_pose.pose.orientation.y,
            start_pose.pose.orientation.z,
            start_pose.pose.orientation.w
        ])[2]
        end_yaw = tf.transformations.euler_from_quaternion([
            end_pose.pose.orientation.x,
            end_pose.pose.orientation.y,
            end_pose.pose.orientation.z,
            end_pose.pose.orientation.w
        ])[2]
        
        req: plan_trajRequest = plan_trajRequest()
        req.start_time = start_time
        req.pos = start_pos
        req.vel = Vector3(0, 0, 0)
        req.acc = Vector3(0, 0, 0)
        req.yaw = start_yaw
        req.next_pos = end_pos
        req.next_yaw = end_yaw
        
        self.plan_traj_client.wait_for_service()
        resp: plan_trajResponse = self.plan_traj_client.call(req)
        if not resp.success:
            rospy.logerr(f"{self.drone_id}: Failed to plan trajectory.")
        return resp.success, resp.bspline, resp.duration
    
    # endregion
    
    # region ins odom
    
    def init_odom_sub(self):
        for ins_id in self.ins_ids:
            self.ins_odom_sub_dict[ins_id] = rospy.Subscriber(f"/{ins_id}/odom", Odometry, self.ins_odom_cb, callback_args=ins_id, queue_size=1)

    def ins_odom_cb(self, msg: Odometry, name: str):
        self.ins_position_dict[name] = msg.pose.pose.position
        self.ins_pose_dict[name] = PoseStamped()
        self.ins_pose_dict[name].pose = msg.pose.pose
        
    # endregion