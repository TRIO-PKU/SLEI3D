#! /usr/bin/env python

import rospy, time, copy, os, yaml
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from gcs_bbox_allocator import BboxAllocator
from gcs_ins_allocator import InsAllocator
from gcs_mission_executer import GcsMissionExecuter
from msgs.srv import gcs_exp_meeting,gcs_exp_meetingRequest,gcs_exp_meetingResponse
from msgs.srv import exp_gcs_meeting,exp_gcs_meetingRequest,exp_gcs_meetingResponse
from msgs.srv import check_los,check_losRequest,check_losResponse

class GCSMeetingManager():
    def __init__(self,bbox_allocator: BboxAllocator,ins_allocator: InsAllocator,gcs_mission_executer: GcsMissionExecuter, bbox_filename: str):

        # params
        self.gcs_max_vel=rospy.get_param("~gcs_max_vel")
        self.exp_num = rospy.get_param("/exp_num")
        self.ins_num = rospy.get_param("/ins_num")
        self.drone_id="gcs"
        
        # meeting_time
        self.init_exploration_time = 60
        self.current_bbox_finish_time: rospy.Duration = None
        self.exp_predicted_finish_time: rospy.Duration = None

        # bbox allocator
        self.bbox_filename: str = bbox_filename
        self.bbox_dict: dict[int, list[float]] = {}
        self.read_bbox_file()
        self.finish_bbox: bool = False
        self.bbox_allocator = bbox_allocator
        self.exp_bbox_dict: dict[int,list[str]] = {}
        self.exp_bbox_id_dict: dict[int, int] = {}

        # ins allocator
        self.ins_allocator = ins_allocator
        self.ins_dict = {}

        # mission executer
        self.gcs_mission_executer = gcs_mission_executer

        # check los
        self.check_los_client = rospy.ServiceProxy("/check_los", check_los)

        # meeting
        self.exp_meeting_clients:dict[int, rospy.ServiceProxy] = {}
        self.meeting_topic="/gcs_exp_meeting"
        self.init_exp_meeting_clients() 
        self.meeting_list = []
        self.state = "finish_meeting"
        self.next_meeting_pose=PoseStamped()
        self.next_meeting_exp_id: int = 0
        rospy.Timer(rospy.Duration(1), self.state_timer)
        self.exp_server = rospy.Service("/exp_gcs_meeting", exp_gcs_meeting , self.exp_meeting_cb)

        # init
        self.exp_init_clients:dict[int, rospy.ServiceProxy] = {}
        self.init_topic="/init"
        self.finished_init = False
        self.init()

        # visualization
        self.feature_vis_pub = rospy.Publisher("/finished_feature_vis", Path, queue_size=10)
        self.meeting_pose_vis_pub = rospy.Publisher("/meeting_pose_vis", PoseStamped, queue_size=10)

    def init(self):
        self.exp_init_client()
        self.exp_bbox_dict, self.exp_bbox_id_dict=self.bbox_allocator.init_bbox()
        rospy.logerr(self.exp_bbox_dict)
        self.ins_dict: dict[int,list[str]]=self.ins_allocator.init_ins()
        self.init_meeting_pose()
        self.init_exp_call()
        self.finished_init=True

    def exp_init_client(self):
        exp_num = int(self.exp_num)
        for exp_id in range(1,exp_num+1):
            topic = "/exp_"+str(exp_id) + self.init_topic
            self.exp_init_clients[exp_id] = rospy.ServiceProxy(topic,gcs_exp_meeting)

    def init_meeting_pose(self):
        last_pose=PoseStamped()
        last_pose.pose.position.x=0
        last_pose.pose.position.y=0
        last_pose.pose.position.z=0.5
        last_pose.header.stamp=rospy.Time.now() + rospy.Duration(self.init_exploration_time)
        rospy.logerr(self.bbox_dict)
        for exp_id in self.exp_bbox_dict.keys():
            bbox=self.exp_bbox_dict[exp_id]
            rospy.logerr("bbox: "+str(bbox))
            pose=self.find_nearest_verticie(bbox,last_pose)
            distance=((pose.pose.position.x-last_pose.pose.position.x)**2+(pose.pose.position.y-last_pose.pose.position.y)**2)**0.5
            time = distance/self.gcs_max_vel + 20
            pose.header.stamp=last_pose.header.stamp + rospy.Duration(time)
            list=[exp_id,pose]
            self.meeting_list.append(list)
            last_pose=pose

    def find_nearest_verticie(self,bbox,last_pose):
        bbox=bbox[0].split(",")
        # 算出bbox的四个顶点，找出距离上一个meeting点最近的顶点
        d=1
        point1=[float(bbox[0])+d,float(bbox[1])+d,0.5]
        point2=[float(bbox[0])+d,float(bbox[4])-d,0.5]
        point3=[float(bbox[3])-d,float(bbox[1])+d,0.5]
        point4=[float(bbox[3])-d,float(bbox[4])-d,0.5]
        point_list=[point1,point2,point3,point4]
        # 找出距离上一个meeting点最近的顶点
        min_distance=float("inf")
        meeting_pose=PoseStamped()
        for i in range(len(point_list)):
            distance=(point_list[i][0]-last_pose.pose.position.x)**2+(point_list[i][1]-last_pose.pose.position.y)**2
            if distance<min_distance:
                min_distance=distance
                meeting_pose.pose.position.x=point_list[i][0]
                meeting_pose.pose.position.y=point_list[i][1]
                meeting_pose.pose.position.z=point_list[i][2]
        return meeting_pose
    
    def init_exp_call(self):
        for exp_id in self.exp_init_clients.keys():
            req=gcs_exp_meetingRequest()
            req.bbox=self.exp_bbox_dict[exp_id]
            req.bbox_id=self.exp_bbox_id_dict[exp_id]
            req.ins_ids=self.ins_dict[exp_id]
            req.meeting_pose=self.meeting_list[exp_id-1][1]
            self.exp_init_clients[exp_id].wait_for_service()
            can_communicate=self.check_los("exp_"+str(exp_id))
            while not can_communicate:
                time.sleep(1)
                can_communicate=self.check_los("exp_"+str(exp_id))
            resp: gcs_exp_meetingResponse=self.exp_init_clients[exp_id].call(req)
            while not resp.success:
                time.sleep(1)
                resp=self.exp_init_clients[exp_id].call(req)
            time.sleep(7)
            

    def init_exp_meeting_clients(self):
        exp_num = int(self.exp_num)
        for exp_id in range(1,exp_num+1):
            topic = "/exp_"+str(exp_id) + self.meeting_topic
            self.exp_meeting_clients[exp_id] = rospy.ServiceProxy(topic, gcs_exp_meeting)
    
    def state_timer(self,event):
        if self.state=="finish_meeting":
            if self.finished_init==False:
                return
            if len(self.meeting_list)==0:
                rospy.logerr("meeting list is empty")
                return
            next_meeting=self.meeting_list[0]
            self.next_meeting_exp_id=next_meeting[0]
            self.next_meeting_pose=next_meeting[1]
            
            # visualizaiton
            meeting_pose_vis: PoseStamped = copy.deepcopy(self.next_meeting_pose)
            meeting_pose_vis.header.frame_id = self.drone_id
            # 检查是否有meeting_pose_vis_pub这个attribute
            if hasattr(self, 'meeting_pose_vis_pub'):
                self.meeting_pose_vis_pub.publish(meeting_pose_vis)
            else:
                return
            
            self.change_state_to("go to meeting")
            self.gcs_mission_executer.go_to_meeting_pose(self.next_meeting_pose)
        elif self.state=="go to meeting":
            # distance=((self.position.pose.position.x-self.next_meeting_pose.pose.position.x)**2+(self.position.pose.position.y-self.next_meeting_pose.pose.position.y)**2)**0.5
            # if distance<1:
            #     self.state="reach meeting"
            return
        elif self.state=="reach meeting":
            return
        elif self.state=="reply to exp":
            self.reply_to_exp(self.reply_bbox)

    def exp_meeting_cb(self,req:exp_gcs_meetingRequest)->exp_gcs_meetingResponse:
        if self.state=="reach meeting":
            # req.exp_id是一个string，前面是字母，最后是一个数字，取出string的最后一个字母，将这最后一个字母转换为int
            string:str = req.exp_id
            exp_id=int(string.split("_")[-1])
            if exp_id==self.next_meeting_exp_id:
                self.finish_bbox=req.finished_bbox
                if self.finish_bbox == True:
                    self.current_bbox = self.exp_bbox_dict[exp_id]
                    self.current_bbox_finish_time = req.predicted_finish_time
                else:
                    self.exp_predicted_finish_time = req.predicted_finish_time
                bbox, bbox_id=self.bbox_allocator.reallocate_bbox(exp_id,self.finish_bbox) 
                if bbox!=["None"] and bbox!=["no bbox left"]:
                    self.exp_bbox_dict[exp_id]=bbox
                    self.exp_bbox_id_dict[exp_id]=bbox_id
                self.reply_bbox: list[str] = bbox
                self.feature_vis_pub.publish(req.finished_inspection_poses)
                self.change_state_to("reply to exp")
                return exp_gcs_meetingResponse(True)
            else:
                rospy.logerr(f"{self.drone_id}: not exp_{str(self.next_meeting_exp_id)}, can't reply to it")
                return exp_gcs_meetingResponse(False)
        else:
            rospy.logerr(self.drone_id + ": not in reach meeting state, can't reply to exp")
            return exp_gcs_meetingResponse(False)
        
    def reply_to_exp(self,bbox: list):
        self.exp_meeting_clients[self.next_meeting_exp_id].wait_for_service()
        req=gcs_exp_meetingRequest()
        req.bbox=bbox
        if bbox==["no bbox left"]:
            # 将req.meeting_pose.header.stamp设为无穷
            req.meeting_pose.header.stamp = rospy.Time(2**31 - 1, 999999999)
            req.bbox_id=0
        else:
            req.meeting_pose=self.compute_next_meeting_pose()
            req.bbox_id=self.exp_bbox_id_dict[self.next_meeting_exp_id]
            self.meeting_list=self.sort_meeting_list(self.next_meeting_exp_id,req.meeting_pose)
        self.meeting_list.pop(0)
        req.ins_ids= []
        can_communicate=self.check_los("exp_"+str(self.next_meeting_exp_id))
        while not can_communicate:
            time.sleep(1)
            can_communicate=self.check_los("exp_"+str(self.next_meeting_exp_id))
            continue
        resp: gcs_exp_meetingResponse = self.exp_meeting_clients[self.next_meeting_exp_id].call(req)
        while not resp.success:
            time.sleep(1)
            resp=self.exp_meeting_clients[self.next_meeting_exp_id].call(req)
            continue
        self.change_state_to("finish_meeting")
    # TODO: check the logic of delay time here
    def compute_next_meeting_pose(self):
        last_pose:PoseStamped=self.compute_average_position()
        bbox=self.exp_bbox_dict[self.next_meeting_exp_id]
        pose=self.find_nearest_verticie(bbox,last_pose)
        if self.finish_bbox == True:
            current_time = rospy.Time.now()
            meeting_time = current_time + rospy.Duration(self.init_exploration_time)
        else:
            current_time = rospy.Time.now()
            meeting_time = current_time + self.exp_predicted_finish_time
        pose.header.stamp = meeting_time
        return pose
    
    def predict_next_bbox(self) -> rospy.Time:

        precise_bbox_finish_time, precise_bbox = self.most_precise_prediction()

        current_time = rospy.Time.now()

        for i in range(6):
            precise_bbox[i] = float(precise_bbox[i])
        precise_bbox_volume: float = (precise_bbox[3] - precise_bbox[0]) * (precise_bbox[4] - precise_bbox[1]) * (precise_bbox[5] - precise_bbox[2])
        
        next_bbox = self.reply_bbox[0].split(",")
        for i in range(6):
            next_bbox[i] = float(next_bbox[i])
        next_bbox_volume: float = (next_bbox[3] - next_bbox[0]) * (next_bbox[4] - next_bbox[1]) * (next_bbox[5] - next_bbox[2])
        
        next_bbox_predict_finish_time: rospy.Duration = rospy.Duration((next_bbox_volume / precise_bbox_volume) * precise_bbox_finish_time)
        return current_time + next_bbox_predict_finish_time
    
    def most_precise_prediction(self):
        # 打开src/exp_node/scripts/exp_1_prediction.yaml文件，从中读取
        base_dir = os.path.dirname(os.path.dirname(os.path.dirname(__file__)))
        min_meeting_num = float("inf")
        for i in range(1, self.exp_num + 1):
            yaml_file_path = os.path.join(base_dir, 'exp_node', 'scripts', 'exp_' + str(i) + '_prediction.yaml')
            with open(yaml_file_path, 'r') as file:
                prediction_data: dict = yaml.safe_load(file)
                for bbox_id in prediction_data.keys():
                    meeting_in_bbox: dict = prediction_data[bbox_id]
                    temp_key = list(meeting_in_bbox.keys())[-1]
                    # 如果"current_bbox_finish_time"不在meeting_in_bbox[temp_key]的键中，就跳过这个bbox_id
                    if "current_bbox_finish_time" not in meeting_in_bbox[temp_key].keys():
                        continue
                    if len(meeting_in_bbox) < min_meeting_num:
                        min_meeting_num = len(meeting_in_bbox)
                        min_meeting_bbox_id = bbox_id
                        last_key = list(meeting_in_bbox.keys())[-1]
                bbox_finish_time: float = float(prediction_data[min_meeting_bbox_id][last_key]["current_bbox_finish_time"])
        precise_bbox = self.bbox_dict[min_meeting_bbox_id]
        
        return bbox_finish_time, precise_bbox
                    
    def compute_average_position(self):
        position=PoseStamped()
        position.pose.position.x=0
        position.pose.position.y=0
        if len(self.meeting_list) == 0:
            return position
        for meeting in self.meeting_list:
            position.pose.position.x+=meeting[1].pose.position.x
            position.pose.position.y+=meeting[1].pose.position.y
        position.pose.position.x/=len(self.meeting_list)
        position.pose.position.y/=len(self.meeting_list)
        return position
    
    
    def sort_meeting_list(self, exp_id, meeting_pose):
        meeting_list=self.meeting_list
        for i in range(len(meeting_list)):
            if meeting_list[i][1].header.stamp > meeting_pose.header.stamp:
                meeting_list.insert(i,[exp_id,meeting_pose])
                return meeting_list
        meeting_list.append([exp_id,meeting_pose])
        return meeting_list
    
    def read_bbox_file(self):
        f = open(self.bbox_filename, "r")
        boxes = yaml.load(f, Loader=yaml.FullLoader)
        i: int = 1
        for box in boxes:
            box = boxes[box]["boundingbox"]
            self.bbox_dict[i] = box
            i += 1

    def check_los(self, target_id: str) -> bool:
        req = check_losRequest()
        req.client_id = "gcs"
        req.target_id = target_id
        self.check_los_client.wait_for_service()
        resp: check_losResponse = self.check_los_client.call(req)
        return resp.success

    def change_state_to(self, state: str) -> None:
        rospy.logwarn(self.drone_id + ": #[" + state + "]#")
        self.state = state