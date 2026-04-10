#! /usr/bin/env python
import threading
from typing import Generator, List, Tuple
import rospy, copy, time
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseStamped, Point
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

from nav_msgs.msg import Path, Odometry
from jsk_rviz_plugins.msg import OverlayText
from datetime import datetime
import math
from std_msgs.msg import ColorRGBA
from msgs.msg import PlanVis
import tf
import os
import sys
from msgs.srv import animation_status, animation_statusResponse
# sys.path.append(os.path.join(os.path.dirname(__file__), "../../"))
# from visualizer_node.scripts.animation import ResultVisualizer

REGISTER_FEATURE_COLOR: ColorRGBA = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.7)
FOUND_FEATURE_COLOR: ColorRGBA = ColorRGBA(r=1.0, g=0.55, b=0.0, a=0.8)
ALLOCATED_FEATURE_COLOR: ColorRGBA = ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0)
INSPECTED_FEATURE_COLOR: ColorRGBA = ColorRGBA(r=0.6, g=1.0, b=0.6, a=1.0)
FINISHED_FEATURE_COLOR: ColorRGBA = ColorRGBA(r=0.0, g=0.4, b=0.0, a=1.0)
GCS_COLOR: ColorRGBA = ColorRGBA(r=0.0, g=0.0, b=1.0, a=0.8)
EXP_COLOR: ColorRGBA = ColorRGBA(r=0.0, g=0.5, b=1.0, a=0.8)
INS_COLOR: ColorRGBA = ColorRGBA(r=0.0, g=1.0, b=1.0, a=0.8)

INS_1_COLOR: ColorRGBA = ColorRGBA(r=1.0, g=0.078, b=0.576, a=1.0)
INS_2_COLOR: ColorRGBA = ColorRGBA(r=0.72, g=0.33, b=0.827, a=1.0)
INS_3_COLOR: ColorRGBA = ColorRGBA(r=1, g=0.75, b=0.75, a=1.0)

MEETING_WITH_INS_NS: str = ",meeting with ins"
INSPECTION_NS: str = ",inspection_task"

PROGRESS_BAR_LENGTH: int = 2

class Visualizer():
    def __init__(self):

        rospy.logerr("visualizer!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
        # drone num
        self.exp_num = rospy.get_param("/exp_num")
        self.ins_num = rospy.get_param("/ins_num")
        
        # feature
        self.feature_id = 0
        self.feature_marker_dict: dict = {}
        self.feature_marker_change_list: list[Marker] = []
        self.init_feature_sub = rospy.Subscriber("/init_feature_vis", Path, self.init_feature_cb)
        self.found_feature_sub = rospy.Subscriber("/found_feature_vis", PoseStamped, self.found_feature_cb)
        self.allocated_feature_sub = rospy.Subscriber("/allocated_feature_vis", Path, self.allocated_feature_cb)
        self.inspected_feature_sub = rospy.Subscriber("/inspected_feature_vis", Path, self.inspected_feature_cb)
        self.finished_feature_sub = rospy.Subscriber("/finished_feature_vis", Path, self.finished_feature_cb)
        self.feature_pub = rospy.Publisher("/feature", MarkerArray, queue_size=10)
        self.animation_feature_server = rospy.Service("/featuers_stat", animation_status, self.animation_feature_cb)
        self.feature_allocating_pub = rospy.Publisher("/feature_allocating", MarkerArray, queue_size=10)
        self.feature_inspecting_pub = rospy.Publisher("/feature_inspecting", Marker, queue_size=10)
        self.inspect_progress_pub = rospy.Publisher("/inspect_progress", MarkerArray, queue_size=10)
        
        # stl map
        self.stl_path = rospy.get_param("/stl_path")
        self.pub_stl_count=0
        self.start_time = rospy.Time.now()
        self.stl_pub = rospy.Publisher("/map_generator/stl", Marker, queue_size=10)
        self.map_pub = rospy.Publisher("/visualzer/sdf_map/occupancy_all", PointCloud2, queue_size=10)
        for i in range(3):
            self.pub_stl_map()
            rospy.sleep(0.5)
        
        # scannd cloud
        self.map: list = []
        self.map_sub = rospy.Subscriber("/sdf_map/occupancy_old", PointCloud2, self.map_cb)
        
        # los
        self.lock = threading.Lock()
        self.los_range = rospy.get_param("/communication_range")
        self.los_pose_dict:dict[str, PoseStamped] = {}
        self.los_marker_dict:dict[str, Marker] = {}
        self.los_sub = rospy.Subscriber("/los_vis", String, self.los_cb)
        self.los_pub = rospy.Publisher("/los_line_vis", MarkerArray, queue_size=1)
        self.Timer = rospy.Timer(rospy.Duration(0.03), self.update_los_marker)
        
        # odom
        self.odom_sub_dict:dict[String, rospy.Subscriber] = {}
        self.init_odom_sub()

        # score and time
        self.gcs_received = 0
        self.Timer=rospy.Timer(rospy.Duration(1), self.update_time_score_marker)
        self.text_pub = rospy.Publisher("/time_score_text", OverlayText, queue_size=1)

        # record
        self.record_csv_pub = rospy.Publisher("/record_csv", String, queue_size=10)
        
        # meeting 
        # TODO: to be removed
        self.meeting_id = 0
        self.meeting_pose_marker_dict: dict[str, Marker] = {}
        self.meeting_pose_sub = rospy.Subscriber("/meeting_pose_vis", PoseStamped, self.meeting_pose_cb)
        self.meeting_pose_pub = rospy.Publisher("/meeting_pose", Marker, queue_size=10)
        
        # inspection poses
        # TODO: to be removed
        self.inspection_line_id = 0
        self.inspection_line_marker_dict: dict[rospy.Time, Marker] = {}
        self.inspection_time: float = rospy.get_param("/inspection_time")
        self.inspection_line_sub = rospy.Subscriber("/inspection_line_vis", Path, self.inspection_line_cb)
        self.inspection_line_pub = rospy.Publisher("/inspection_line", Marker, queue_size=10)
        
        # inspection state
        # TODO: to be removed
        self.inspection_state_timer = rospy.Timer(rospy.Duration(0.2), self.show_inspection_state_timer)
        self.inspection_state_marker_dict: dict[str, Marker] = {}
        self.start_inspection_sub = rospy.Subscriber("/start_inspection_vis", String, self.start_inspection_cb)
        self.inspection_state_pub = rospy.Publisher("/inspection_state_vis", MarkerArray, queue_size=10)
        
        # debug service
        self.path_sub = rospy.Subscriber("/path_vis", Path, self.path_cb)
        self.path_pub = rospy.Publisher("/debug_path", Marker, queue_size=10)
        self.debug_inspection_point_sub = rospy.Subscriber("/inspection_point_vis", Path, self.debug_inspection_point) # set for debug purpose, visualize inspection points
        self.debug_inspection_point_pub = rospy.Publisher("/inspection_point", Marker, queue_size=10)

        # plan
        # TODO: show exp meeting plan (straight line + meeting poses with ins)
        self.plan_sub = rospy.Subscriber("/plan_vis", PlanVis, self.plan_cb)
        self.plan_pub = rospy.Publisher("/plan", Marker, queue_size=10)
        self.register_all_decision_text_pub()
        self.decision_text_sub = rospy.Subscriber("/decision_text", String, self.decision_text_cb)
        
        # 镜头跟随
        self.tf_timer = rospy.Timer(rospy.Duration(0.01), self.tf_cb)

        # reset
        self.clean_feature()
        self.clean_los()
        self.clean_exp()
        self.clean_ins()
        self.clean_map()
        self.count=0
    
    
    # 统计所有的features特征
    def animation_feature_cb(self, req):
        featuer_marker_stat = {"reg":0, "found":0, "alloc":0, "insp":0}
        for _, marker in self.feature_marker_dict.items():
            if marker.color == REGISTER_FEATURE_COLOR:
                featuer_marker_stat["reg"] += 1
            elif marker.color == FOUND_FEATURE_COLOR:
                featuer_marker_stat["found"] += 1
            elif marker.color == ALLOCATED_FEATURE_COLOR:
                featuer_marker_stat["alloc"] += 1
            elif marker.color == INSPECTED_FEATURE_COLOR:
                featuer_marker_stat["insp"] += 1
        
        feature_markder_msg = f"{featuer_marker_stat['reg']},{featuer_marker_stat['found']},{featuer_marker_stat['alloc']},{featuer_marker_stat['insp']}"
        resp = animation_statusResponse()
        resp.status = feature_markder_msg
        return resp
        
        
        
    # 镜头跟随
    def tf_cb(self, event):
        br = tf.TransformBroadcaster()
        for key in self.los_pose_dict:
            pose = self.los_pose_dict[key]
            br.sendTransform((pose.pose.position.x, pose.pose.position.y, pose.pose.position.z),
                             (pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w),
                             rospy.Time.now(),
                             key,
                             "world")
        
    # debug inspection point
    def debug_inspection_point(self, msg: Path):
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "inspection_point"
        marker.id = 0
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 1
        marker.scale.y = 1
        marker.scale.z = 1
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        for pose in msg.poses:
            marker.points.append(pose.pose.position)
        self.debug_inspection_point_pub.publish(marker)
    # end region
        
    # region plan
    
    def plan_cb(self, msg: PlanVis):
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = rospy.Time.now()
        marker.type = Marker.CUBE
        marker.id = 0
        marker.color.a = 0.0
        marker.ns = msg.msg
        marker.pose.orientation.w = 1.0
        for i in range(10):
            marker.id = i
            self.plan_pub.publish(marker)

        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = rospy.Time.now()
        marker.type = Marker.LINE_STRIP
        marker.id = 0
        marker.color.a = 0.0
        marker.ns = msg.msg
        marker.pose.orientation.w = 1.0
        for i in range(100):
            marker.id = i
            self.plan_pub.publish(marker)

        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = rospy.Time.now()
        marker.type = Marker.SPHERE_LIST
        marker.id = 0
        marker.color.a = 0.0
        marker.ns = msg.msg
        marker.pose.orientation.w = 1.0
        for i in range(100):
            marker.id = i
            self.plan_pub.publish(marker)
            
        marker.id = 0
        
        id, task = msg.msg.split(",") 
        rospy.logwarn(f"the namespace for {id}: {msg.msg}")
        
        marker.action = Marker.ADD
        marker.header.stamp = rospy.Time.now()

        # show shpere around ins_inspection point of "exp" in id
        if "exp" in id:
            marker.type = Marker.SPHERE_LIST
            marker.scale.x = 2*self.los_range
            marker.scale.y = 2*self.los_range
            marker.scale.z = 2*self.los_range
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.0
            marker.points.clear()
            for pose in msg.ins_inspection_poses:
                point = Point()
                point.x = pose.pose.position.x
                point.y = pose.pose.position.y
                point.z = pose.pose.position.z
                marker.points.append(point)
                marker.id += 1
                self.plan_pub.publish(marker)
        
        # show the center of the sphere, mainly for debug purpose, could be removed
        if "exp" in id:
            for pose in msg.ins_inspection_poses:
                marker.type = Marker.SPHERE
                marker.scale.x = 0.3
                marker.scale.y = 0.3
                marker.scale.z = 0.3
                marker.color = EXP_COLOR
                marker.color.a = 0.7
                marker.pose = pose.pose
                self.plan_pub.publish(marker)
                marker.id += 1

        # show points
        for pose in msg.poses:
            marker.type = Marker.CUBE
            marker.scale.x = 0.3
            marker.scale.y = 0.3
            marker.scale.z = 0.3
            if "exp" in id:  # 如果是exp，显示exp和ins或者gcs的开会地点
                if "ins" in task:
                    if "ins_1" in pose.header.frame_id:
                        marker.color = INS_1_COLOR
                    elif "ins_2" in pose.header.frame_id:
                        marker.color = INS_2_COLOR
                    elif "ins_3" in pose.header.frame_id:
                        marker.color = INS_3_COLOR
                    else:
                        marker.color = INS_COLOR
                elif "gcs" in task:
                    marker.color = GCS_COLOR
            elif "ins" in id:   # 如果是ins，显示ins的拍照点
                marker.color = INSPECTED_FEATURE_COLOR
                marker.color.a = 0.0 # 暂时去掉
            marker.header.frame_id = "world"
            marker.pose = pose.pose
            self.plan_pub.publish(marker)
            marker.id += 1
            
        # show lines
        marker.pose = PoseStamped().pose
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        if "exp" in id:
            color: ColorRGBA = ColorRGBA(r=0.0, g=0.5, b=1.0, a=1.0)
            self.show_bspline_in_sphere(marker, color, msg.paths, marker.id)
        elif "ins" in id:
            number_id = int(id.split("_")[1])
            if number_id == 1:
                color: ColorRGBA = INS_1_COLOR
            elif number_id == 2:
                color: ColorRGBA = INS_2_COLOR
            elif number_id == 3:
                color: ColorRGBA = INS_3_COLOR
            else:
                color: ColorRGBA = INS_COLOR
                
            self.show_bspline_in_sphere(marker, color, msg.paths, marker.id)
                
    def show_bspline_in_sphere(self, input_marker: Marker, color: ColorRGBA, paths: List[Path], marker_id: int):
        radius = 0.01
        marker = input_marker
        marker.header.frame_id = "world"
        marker.header.stamp = rospy.Time.now()
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.DELETE
        self.plan_pub.publish(marker)

        marker.action =Marker.ADD
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.color = color
        for path in paths:
            marker.id = marker_id
            marker.points.clear()
            for i in range(len(path.poses)-1):
                prev_pose = path.poses[i]
                next_pose = path.poses[i+1]
                distance = math.sqrt((prev_pose.pose.position.x - next_pose.pose.position.x)**2 + (prev_pose.pose.position.y - next_pose.pose.position.y)**2 + (prev_pose.pose.position.z - next_pose.pose.position.z)**2)
                if distance > radius:
                    point_num = int(distance / radius)
                    for j in range(point_num):
                        point = Point()
                        point.x = prev_pose.pose.position.x + j * (next_pose.pose.position.x - prev_pose.pose.position.x) / point_num
                        point.y = prev_pose.pose.position.y + j * (next_pose.pose.position.y - prev_pose.pose.position.y) / point_num
                        point.z = prev_pose.pose.position.z + j * (next_pose.pose.position.z - prev_pose.pose.position.z) / point_num
                        marker.points.append(point)
            self.plan_pub.publish(marker)
            marker_id += 1


        # decode msg

    def register_all_decision_text_pub(self):
        self.decision_text_pub_dict: dict[str, rospy.Publisher] = {}
        self.decision_text_pub_dict["begin_decision"] = rospy.Publisher("/decision_text_begin_decision", OverlayText, queue_size=10)
        for i in range(1, self.ins_num+1):
            ins_id = f"ins_{i}"
            self.register_decision_text_pub(ins_id)

    def decision_text_cb(self, msg: String):
        rospy.logwarn("decision_text_cb====================================================")
        for id in self.decision_text_pub_dict.keys():
            # rospy.logwarn(f"DELETE over lay text: {id}")
            text = OverlayText()
            text.action = OverlayText.DELETE
            text.width = 1
            text.height = 1
            decision_pub = self.decision_text_pub_dict[id]
            decision_pub.publish(text)

        # display begin decision
        text = OverlayText()
        text.action = OverlayText.ADD
        text.width = 240
        text.height = 50
        text.left = 10
        text.top = 80  # Adjust the position for each OverlayText
        text.bg_color.a = 0.5
        text.bg_color.r = 0.5
        text.bg_color.g = 0.5
        text.bg_color.b = 0.5
        text.line_width = 1
        text.text_size = 15.0
        text.font = "DejaVu Sans Mono"
        text.text = "\nDecision Result:"
        text.fg_color.r = 1.0
        text.fg_color.g = 1.0
        text.fg_color.b = 1.0
        text.fg_color.a = 1.0
        self.decision_text_pub_dict["begin_decision"].publish(text)

        if "," in msg.data:
            meeting_list = msg.data.split(",")
            i = 0
            for meeting in meeting_list:
                # rospy.logwarn(f"SHOW over lay text: {meeting}")
                list = meeting.split(":")
                if len(list) < 2:
                    break
                ins_id = list[0]
                feature_num = list[1]
                if ins_id not in self.decision_text_pub_dict.keys():
                    self.register_decision_text_pub(ins_id)
                decision_text_pub = self.decision_text_pub_dict[ins_id]
                text = OverlayText()
                text.action = OverlayText.ADD
                text.width = 240
                text.height = 40
                text.left = 10
                text.top = 130 + i * 40  # Adjust the position for each OverlayText
                text.bg_color.a = 0.5
                text.bg_color.r = 0.5
                text.bg_color.g = 0.5
                text.bg_color.b = 0.5
                text.line_width = 1
                text.text_size = 15.0
                text.font = "DejaVu Sans Mono"
                if ins_id == "ins_1":
                    text.fg_color = INS_1_COLOR
                elif ins_id == "ins_2":
                    text.fg_color = INS_2_COLOR
                elif ins_id == "ins_3":
                    text.fg_color = INS_3_COLOR
                else:
                    text.fg_color.r = 1.0
                    text.fg_color.g = 1.0
                    text.fg_color.b = 1.0
                    text.fg_color.a = 1.0
                text.text = f"{ins_id}: {feature_num}"
                decision_text_pub.publish(text)
                i+=1
        start_time: float = rospy.Time.now().to_sec()
        show_duration = 4
        while rospy.Time.now().to_sec() - start_time < show_duration:
            pass
        for id in self.decision_text_pub_dict.keys():
            text = OverlayText()
            text.action = OverlayText.DELETE
            text.width = 1
            text.height = 1
            self.decision_text_pub_dict[id].publish(text)
    
    def register_decision_text_pub(self, ins_id: str):
        self.decision_text_pub_dict[ins_id] = rospy.Publisher(f"/decision_text_{ins_id}", OverlayText, queue_size=10)
    
    # endregion
        
    # region debug service
    
    def path_cb(self, msg: Path):
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "path"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.1
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.pose.orientation.w = 1
        for pose in msg.poses:
            marker.points.append(pose.pose.position)
        self.path_pub.publish(marker)
    
    # endregion
        
    # region meeting
    
    def meeting_pose_cb(self, msg: PoseStamped):
        if msg.header.frame_id not in self.meeting_pose_marker_dict.keys():
            self.register_meeting_pose_marker(msg)
        else:
            self.update_meeting_pose_marker(msg)
    
    def register_meeting_pose_marker(self, pose: PoseStamped):
        marker = Marker()
        self.meeting_id += 1
        marker.id = self.meeting_id
        marker.pose = pose.pose
        marker.pose.orientation.w = 1
        marker.pose.orientation.x = 0
        marker.pose.orientation.y = 0
        marker.pose.orientation.z = 0
        marker.header.frame_id = "world"
        marker.ns = "meeting"
        if "gcs" in pose.header.frame_id:
            marker.type = Marker.CYLINDER
            marker.color.r = 0.82
            marker.color.g = 0.41
            marker.color.b = 0.11
        elif "with i" in pose.header.frame_id:
            if "ins_1" in pose.header.frame_id:
                marker.color = INS_1_COLOR
            elif "ins_2" in pose.header.frame_id:
                marker.color = INS_2_COLOR
            elif "ins_3" in pose.header.frame_id:
                marker.color = INS_3_COLOR
            else:
                marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            marker.type = Marker.CUBE
        elif "with g" in pose.header.frame_id:
            marker.type = Marker.CUBE
            marker.color.r = 0.82
            marker.color.g = 0.41
            marker.color.b = 0.11
        elif "ins" in pose.header.frame_id:
            marker.type = Marker.SPHERE
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 1.0
        marker.action = Marker.ADD
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3
        marker.color.a = 1.0
        self.meeting_pose_marker_dict[copy.deepcopy(pose.header.frame_id)] = marker
        self.meeting_pose_pub.publish(marker)
    
    def update_meeting_pose_marker(self, pose: PoseStamped):
        marker = self.meeting_pose_marker_dict[pose.header.frame_id]
        marker.pose = pose.pose
        marker.pose.orientation.w = 1
        marker.pose.orientation.x = 0
        marker.pose.orientation.y = 0
        marker.pose.orientation.z = 0
        marker.header.frame_id = "world"
        self.meeting_pose_pub.publish(marker)
    
    # endregion
    
    # region inspection state
    
    def start_inspection_cb(self, msg: String):
        if msg.data not in self.inspection_state_marker_dict.keys():
            marker = Marker()
            marker.header.frame_id = "world"
            marker.header.stamp = rospy.Time.now()
            marker.action = Marker.ADD
            marker.type = Marker.SPHERE
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker.color.a = 0.0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.pose.position = self.los_pose_dict[msg.data].pose.position
            marker.pose.position.z += 0.3
            marker.pose.orientation.w = 1.0
            marker.lifetime = rospy.Duration(self.inspection_time)
            self.inspection_state_marker_dict[msg.data] = marker            
            
    def show_inspection_state_timer(self, event):
        # # clear all markers
        # ma = MarkerArray()
        # delete_marker = Marker()
        # delete_marker.action = Marker.DELETEALL
        # ma.markers.append(delete_marker)
        # self.inspection_state_pub.publish(ma)
        
        ma = MarkerArray()
        for ins_id, marker in self.inspection_state_marker_dict.items():
            if rospy.Time.now() - marker.header.stamp > rospy.Duration(self.inspection_time):
                marker.action = Marker.DELETE
                ma.markers.append(marker)
                del self.inspection_state_marker_dict[ins_id]
                continue
            
            # 红点闪烁，a 在 0 和 1 之间跳跃
            marker.color.a = 1.0 if marker.color.a == 0.0 else 0.0
            
            ma.markers.append(marker)
        self.inspection_state_pub.publish(ma)
    
    # endregion

    # region inspection line
    
    def inspection_line_cb(self, msg: Path):
        if msg.header.frame_id not in self.inspection_line_marker_dict.keys():
            self.register_inspection_line_marker(msg)
        else:
            self.update_inspection_line_marker(msg)
            
    def register_inspection_line_marker(self, path: Path):
        marker = Marker()
        self.inspection_line_id += 1
        marker.id = self.inspection_line_id
        marker.header = path.header
        marker.ns = "inspection"
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.1
        marker.color.a = 0.8
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.pose.orientation.w = 1
        marker.pose.orientation.x = 0
        marker.pose.orientation.y = 0
        marker.pose.orientation.z = 0
        for pose in path.poses:
            marker.points.append(pose.pose.position)
        self.inspection_line_marker_dict[path.header.stamp] = marker
        self.inspection_line_pub.publish(marker)
    
    def update_inspection_line_marker(self, path: Path):
        marker = self.inspection_line_marker_dict[path.header.stamp]
        marker.header = path.header
        marker.points.clear()
        for pose in path.poses:
            marker.points.append(pose.pose.position)
        self.inspection_line_pub.publish(marker)
    
    # endregion

    # region overlay text
    
    def update_time_score_marker(self, event):
        gcs_received=0
        if len(self.feature_marker_dict)==0:
            percentage=0
        else:
            for key in self.feature_marker_dict:
                if self.feature_marker_dict[key].color == INSPECTED_FEATURE_COLOR:
                    gcs_received+=1
            total_num=len(self.feature_marker_dict)
            percentage=gcs_received/total_num
        # 将秒数转换为分钟
        elapsed_time = (rospy.Time.now() - self.start_time).to_sec()
        elapsed_time_min = int(elapsed_time // 60)
        elapsed_time_sec = int(elapsed_time % 60)
        if len(str(elapsed_time_sec))==1:
            elapsed_time_sec="0" + str(elapsed_time_sec)
        Time="Time: "+str(elapsed_time_min)+":"+str(elapsed_time_sec)
        msg=OverlayText()
        msg.action=OverlayText.ADD

        msg.width=240
        msg.height=70
        msg.left=10
        msg.top=10
        msg.bg_color.a=0.5
        msg.bg_color.r=0.5
        msg.bg_color.g=0.5
        msg.bg_color.b=0.5
        msg.line_width=1
        msg.text_size=20.0
        msg.font="DejaVu Sans Mono"
        msg.fg_color.r=0.0
        msg.fg_color.g=0.0
        msg.fg_color.b=0.0
        msg.fg_color.a=1.0
        msg.text=Time+"\nScore: "+str('{:.2f}'.format(percentage*100))+"%"
        self.text_pub.publish(msg)
        self.record_csv_pub.publish("feature_score"+","+str(percentage))
        
    # endregion

    # region los
        
    def los_cb(self, msg: String):
        drone1, drone2, success = msg.data.split(",")
        drones_name=drone1+","+drone2
        if drone1 not in self.los_pose_dict or drone2 not in self.los_pose_dict:
            return
        drone1_pose = self.los_pose_dict[drone1]
        drone2_pose = self.los_pose_dict[drone2]
        self.add_los_marker(drones_name, drone1_pose, drone2_pose, success)

    def add_los_marker(self, drones_name: String, drone1_pose: PoseStamped, drone2_pose: PoseStamped, success: String):
        marker: Marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = drone1_pose.header.stamp
        marker.id = self.count
        marker.ns = "los"
        marker.lifetime = rospy.Duration(0.1)
        marker.type = Marker.LINE_LIST
        marker.action = 0
        marker.scale.x = 0.1
        if success=="true":
            marker.color.a = 0.5
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
        else:
            marker.color.a = 0.5
            marker.color.r = 1.0
            marker.color.g = 0.2
            marker.color.b = 0.2
        point1= Point()
        point1.x = drone1_pose.pose.position.x
        point1.y = drone1_pose.pose.position.y
        point1.z = drone1_pose.pose.position.z
        marker.points.append(point1)
        point2= Point()
        point2.x = drone2_pose.pose.position.x
        point2.y = drone2_pose.pose.position.y
        point2.z = drone2_pose.pose.position.z
        marker.points.append(point2)
        marker.pose.orientation.w = 1
        self.los_marker_dict[drones_name]=marker
        self.count+=1

    def update_los_marker(self, event):
        with self.lock:
            marker_array:MarkerArray = MarkerArray()
            temp_dict = self.los_marker_dict.copy()
            los_marker_dict_copy = copy.deepcopy(temp_dict)
        # 注意：确保deepcopy操作也在锁的保护下进行，以避免在复制过程中字典被修改
        delete_list=[]
        for key in los_marker_dict_copy:
            with self.lock:
                if rospy.Time.now()-self.los_marker_dict[key].header.stamp <= rospy.Duration(1.0):
                    marker: Marker = self.los_marker_dict[key]
                    marker.points.clear()
                    drone1, drone2 = key.split(",")

                    drone1_pose = self.los_pose_dict[drone1]
                    point1= Point()
                    point1.x = drone1_pose.pose.position.x
                    point1.y = drone1_pose.pose.position.y
                    point1.z = drone1_pose.pose.position.z
                    marker.points.append(point1)

                    drone2_pose = self.los_pose_dict[drone2]
                    point2= Point()
                    point2.x = drone2_pose.pose.position.x
                    point2.y = drone2_pose.pose.position.y
                    point2.z = drone2_pose.pose.position.z
                    marker.points.append(point2)

                else:
                    marker: Marker = self.los_marker_dict[key]
                    marker.action = Marker.DELETE
                    marker_array.markers.append(marker)
                    delete_list.append(key)

            marker_array.markers.append(marker)
            
        for key in delete_list:
            del self.los_marker_dict[key]

        self.los_pub.publish(marker_array)

    # endregion

    # region map
    
    def pub_stl_map(self):
        rospy.sleep(1)
        marker:Marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = rospy.Time.now()
        marker.id = 0
        marker.ns = "map"
        marker.type = marker.MESH_RESOURCE
        marker.action = Marker.ADD
        marker.scale.x = 1
        marker.scale.y = 1
        marker.scale.z = 1
        marker.mesh_use_embedded_materials = True
        marker.pose.position.x = 0
        marker.pose.position.y = 0
        marker.pose.position.z = 0
        marker.pose.orientation.w = 1.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.color.a = 0.3
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.mesh_resource = self.stl_path
        self.stl_pub.publish(marker)

    def map_cb(self, msg: PointCloud2):
            map_gen = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
            m = list(map_gen)
            self.map.extend(m)
            output = pc2.create_cloud(msg.header, msg.fields, self.map)
            self.map_pub.publish(output) 
            
    # endregion
    
    # region feature
    
    def init_feature_cb(self, msg: Path): # red
        marker_array: MarkerArray = MarkerArray()
        for pose in msg.poses:
            self.register_feature_marker(pose)
            marker_array.markers.append(self.feature_marker_dict[pose.header.stamp])
        self.feature_pub.publish(marker_array) 
       
    def found_feature_cb(self, msg: PoseStamped): # orange
        marker_array: MarkerArray = MarkerArray()
        pos = msg.pose.position
        for fm in self.feature_marker_dict.values():
            if pos.x == fm.pose.position.x and pos.y == fm.pose.position.y and pos.z == fm.pose.position.z:
                fm.color = FOUND_FEATURE_COLOR
                marker_array.markers.append(fm)
        if msg.header.stamp in self.feature_marker_dict:
            self.feature_marker_dict[msg.header.stamp].color = FOUND_FEATURE_COLOR
            marker: Marker = self.feature_marker_dict[msg.header.stamp]
            marker.color = FOUND_FEATURE_COLOR
            marker_array.markers.append(marker)
        self.feature_pub.publish(marker_array)

    def allocated_feature_cb(self, msg: Path): # yellow
        line_marker_array: MarkerArray = MarkerArray() # line from ins to feature
        feature_point_marker_array: MarkerArray = MarkerArray() # feature point
        marker_id = 0
        for pose in msg.poses:
            if not pose.header.stamp in self.feature_marker_dict:
                continue
            # draw line from feature to ins
            id = pose.header.frame_id
            # get ins position
            ins_pose = self.los_pose_dict[id]
            feature_marker: Marker = self.feature_marker_dict[pose.header.stamp]
            feature_pose = feature_marker.pose
            # draw line
            line_marker: Marker = Marker()
            line_marker.header.frame_id = "world"
            line_marker.header.stamp = rospy.Time.now()
            line_marker.id = marker_id
            line_marker.ns = id + "allocating"
            line_marker.type = Marker.LINE_STRIP
            line_marker.action = Marker.ADD
            line_marker.lifetime = rospy.Duration(1)
            line_marker.scale.x = 0.1
            line_marker.color = ALLOCATED_FEATURE_COLOR
            line_marker.pose.orientation.w = 1
            line_marker.points.append(ins_pose.pose.position)
            line_marker.points.append(feature_pose.position)
            line_marker_array.markers.append(line_marker)

            # change the color of the allocated feature
            self.feature_marker_dict[pose.header.stamp].color = ALLOCATED_FEATURE_COLOR
            feature_point_marker: Marker = self.feature_marker_dict[pose.header.stamp]
            feature_point_marker.color = ALLOCATED_FEATURE_COLOR
            feature_point_marker_array.markers.append(feature_marker)
            marker_id += 1
        rospy.logwarn("len(line_marker_array.markers): " + str(len(line_marker_array.markers)))
        self.feature_allocating_pub.publish(line_marker_array)
        self.feature_pub.publish(feature_point_marker_array)
    
    def inspected_feature_cb(self, msg: Path): # light green
        rospy.logwarn(f"inspected_feature_cb: {len(msg.poses)}")
        for pose in msg.poses:
            if not pose.header.stamp in self.feature_marker_dict:
                continue
            id = pose.header.frame_id
            # get ins position
            ins_pose = self.los_pose_dict[id]
            feature_marker: Marker = self.feature_marker_dict[pose.header.stamp]
            feature_pose = feature_marker.pose
            # draw line
            marker: Marker = Marker()
            marker.header.frame_id = "world"
            marker.header.stamp = rospy.Time.now()
            marker.id = 0
            marker.ns = id + "inspecting"
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.lifetime = rospy.Duration(self.inspection_time)
            marker.scale.x = 0.1
            marker.color = INSPECTED_FEATURE_COLOR
            marker.color.a = 1.0
            marker.pose.orientation.w = 1
            marker.points.append(ins_pose.pose.position)
            marker.points.append(feature_pose.position)
            self.feature_inspecting_pub.publish(marker)

            start_inspecting_time: float = rospy.Time.now().to_sec()
            update_duration: float = 0.1
            while rospy.Time.now().to_sec() - start_inspecting_time <= self.inspection_time + update_duration:
                ins_pose: PoseStamped = self.los_pose_dict[id]
                progress = (rospy.Time.now().to_sec() - start_inspecting_time) / self.inspection_time
                marker_array: MarkerArray = MarkerArray()
                progress_marker, remain_marker = self.progress_bar(id, ins_pose, progress)
                marker_array.markers.append(progress_marker)
                marker_array.markers.append(remain_marker)
                self.inspect_progress_pub.publish(marker_array)
                rospy.sleep(update_duration)
            # delete progress bar
            marker_array = MarkerArray()
            for marker_id in range(2):
                marker: Marker = Marker()
                marker.header.frame_id = "world"
                marker.header.stamp = rospy.Time.now()
                marker.id = marker_id
                marker.ns = id + "progress_bar"
                marker.type = Marker.CUBE
                marker.action = Marker.DELETE
                marker_array.markers.append(marker)
            self.inspect_progress_pub.publish(marker_array)
            
            # update inspected feature color
            marker_array: MarkerArray = MarkerArray()
            self.feature_marker_dict[pose.header.stamp].color = INSPECTED_FEATURE_COLOR
            marker: Marker = self.feature_marker_dict[pose.header.stamp]
            marker.color = INSPECTED_FEATURE_COLOR
            marker.color.a = 1.0
            marker_array.markers.append(marker)
            self.feature_pub.publish(marker_array)

    def progress_bar(self, ins_id: String, ins_pose: PoseStamped, progress: float) -> Tuple[Marker, Marker]:
        if progress < 0:
            return
        if progress > 1:
            progress = 1
        if progress < 0.5:
            progress_offset: float = 0.5-0.5*progress
            progress_middle_x: float = ins_pose.pose.position.x-(progress_offset*PROGRESS_BAR_LENGTH)
            remained_middle_x: float = progress_middle_x+(0.5*PROGRESS_BAR_LENGTH)
        else:
            progress_offset: float = 0.5-0.5*progress
            progress_middle_x: float = ins_pose.pose.position.x-(progress_offset*PROGRESS_BAR_LENGTH)
            remained_middle_x: float = progress_middle_x+(0.5*PROGRESS_BAR_LENGTH)
        progress_marker: Marker = Marker()
        progress_marker.header.frame_id = "world"
        progress_marker.header.stamp = rospy.Time.now()
        progress_marker.id = 0
        progress_marker.ns = ins_id + "progress_bar"
        progress_marker.type = Marker.CUBE
        progress_marker.action = Marker.ADD
        progress_marker.pose.position.x = progress_middle_x
        progress_marker.pose.position.y = ins_pose.pose.position.y
        progress_marker.pose.position.z = ins_pose.pose.position.z + 0.5
        progress_marker.pose.orientation.w = 1
        progress_marker.scale.x = PROGRESS_BAR_LENGTH * progress
        progress_marker.scale.y = 0.01
        progress_marker.scale.z = 0.3
        progress_marker.color.a = 1.0
        progress_marker.color.r = 0.0
        progress_marker.color.g = 1.0
        progress_marker.color.b = 0.0

        remain_marker: Marker = Marker()
        remain_marker.header.frame_id = "world"
        remain_marker.header.stamp = rospy.Time.now()
        remain_marker.id = 1
        remain_marker.ns = ins_id + "progress_bar"
        remain_marker.type = Marker.CUBE
        remain_marker.action = Marker.ADD
        remain_marker.pose.position.x = remained_middle_x
        remain_marker.pose.position.y = ins_pose.pose.position.y
        remain_marker.pose.position.z = ins_pose.pose.position.z + 0.5
        remain_marker.pose.orientation.w = 1
        remain_marker.scale.x = PROGRESS_BAR_LENGTH * (1-progress)
        remain_marker.scale.y = 0.01
        remain_marker.scale.z = 0.3
        remain_marker.color.a = 1.0
        remain_marker.color.r = 1.0
        remain_marker.color.g = 1.0
        remain_marker.color.b = 1.0
        return progress_marker, remain_marker
        
    def finished_feature_cb(self, msg: Path): # dark green
        rospy.logwarn("GCS received: " + str(len(msg.poses)))    
        marker_array: MarkerArray = MarkerArray()
        for pose in msg.poses:
            if pose.header.stamp in self.feature_marker_dict:
                marker: Marker = self.feature_marker_dict[pose.header.stamp]
                if marker.color == INSPECTED_FEATURE_COLOR:
                    self.feature_marker_dict[pose.header.stamp].color = FINISHED_FEATURE_COLOR
                    marker.color = FINISHED_FEATURE_COLOR
                    marker_array.markers.append(marker)
                    self.gcs_received+=1
        self.feature_pub.publish(marker_array)
        
    def register_feature_marker(self, pose: PoseStamped):
        marker = Marker()
        self.feature_id += 1
        marker.id = self.feature_id
        marker.pose = pose.pose
        marker.header = pose.header
        marker.ns = "feature"
        marker.type = Marker.SPHERE
        marker.action = 0
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3
        marker.color = REGISTER_FEATURE_COLOR
        marker.pose.orientation.w = 1
        marker.pose.orientation.x = 0
        marker.pose.orientation.y = 0
        marker.pose.orientation.z = 0
        self.feature_marker_dict[pose.header.stamp] = marker
        
    # endregion
         
    # region clean
    
    def clean_ins(self):
        marker_array:MarkerArray = MarkerArray()
        # clean inspection point
        for i in range(1,self.ins_num+1):
            marker = Marker()
            marker.header.frame_id = "world"
            marker.header.stamp = rospy.Time.now()
            marker.id = 0
            marker.ns = "ins_" + str(i) + INSPECTION_NS 
            marker.type = Marker.CUBE
            marker.action = Marker.DELETEALL
            marker.scale.x = 0.1
            marker.pose.orientation.w = 1.0
            marker_array.markers.append(marker)
        # clean inspection line
        for i in range(1,self.ins_num+1):
            marker = Marker()
            marker.header.frame_id = "world"
            marker.header.stamp = rospy.Time.now()
            marker.id = 0
            marker.ns = "ins_" + str(i) + INSPECTION_NS 
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.DELETEALL
            marker.scale.x = 0.1
            marker.pose.orientation.w = 1.0
            marker_array.markers.append(marker)
        self.feature_pub.publish(marker_array)

    
    def clean_exp(self):
        marker_array:MarkerArray = MarkerArray()
        # clean meeting point
        for i in range(1,self.exp_num+1):
            marker = Marker()
            marker.header.frame_id = "world"
            marker.header.stamp = rospy.Time.now()
            marker.id = 0
            marker.ns = "exp_" + str(i) + MEETING_WITH_INS_NS
            rospy.logwarn(f"exp_{i} namespace: " + marker.ns)
            marker.type = Marker.CUBE
            marker.action = Marker.DELETEALL
            marker.scale.x = 0.3
            marker.scale.y = 0.3
            marker.scale.z = 0.3
            marker.pose.position.x = 0
            marker.pose.position.y = 0
            marker.pose.position.z = 0
            marker.pose.orientation.w = 1.0
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker_array.markers.append(marker)
        # clean meeting line
        for i in range(1,self.exp_num+1):
            marker = Marker()
            marker.header.frame_id = "world"
            marker.header.stamp = rospy.Time.now()
            marker.id = 0
            marker.ns = "exp_" + str(i) + MEETING_WITH_INS_NS
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.DELETEALL
            marker.scale.x = 0.3
            marker.scale.y = 0.3
            marker.scale.z = 0.3
            marker.pose.position.x = 0
            marker.pose.position.y = 0
            marker.pose.position.z = 0
            marker.pose.orientation.w = 1.0
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker_array.markers.append(marker)
        self.feature_pub.publish(marker_array)
        
    def clean_map(self):
        self.map.clear()
        self.map_pub.publish(PointCloud2())
        
    def clean_feature(self):
        marker_array:MarkerArray = MarkerArray()
        marker:Marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = rospy.Time.now()
        marker.id = 0
        marker.ns = "feature"
        marker.type = Marker.SPHERE
        marker.action = Marker.DELETEALL
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3
        marker.pose.position.x = 0
        marker.pose.position.y = 0
        marker.pose.position.z = 0
        marker.pose.orientation.w = 1.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker_array.markers.append(marker)
        self.feature_pub.publish(marker_array)

    def clean_los(self):
        marker_array:MarkerArray = MarkerArray()
        marker:Marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = rospy.Time.now()
        marker.id = 0
        marker.ns = "los"
        marker.type = Marker.LINE_LIST
        marker.action = Marker.DELETEALL
        marker.pose.position.x = 0
        marker.pose.position.y = 0
        marker.pose.position.z = 0
        marker.pose.orientation.w = 1.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker_array.markers.append(marker)
        self.feature_pub.publish(marker_array)
    # endregion  
    
    # region odom

    def init_odom_sub(self):
        exp_num = int(self.exp_num)
        for i in range(1, exp_num+1):
            exp_id="exp_"+str(i)
            topic = f"/{exp_id}/odom"
            self.odom_sub_dict[exp_id] = rospy.Subscriber(topic, Odometry, self.odom_cb, callback_args=exp_id)
        ins_num = int(self.ins_num)
        for j in range(1, ins_num+1):
            ins_id="ins_"+str(j)
            topic = f"/{ins_id}/odom"
            self.odom_sub_dict[ins_id] = rospy.Subscriber(topic, Odometry, self.odom_cb, callback_args=ins_id)
        self.odom_sub_dict["gcs"] = rospy.Subscriber("/gcs/odom", Odometry, self.odom_cb, callback_args="gcs")

    def odom_cb(self, msg: Odometry, name: str):
        pose=PoseStamped()
        pose.header=msg.header
        pose.pose=msg.pose.pose
        self.los_pose_dict[name]=pose
    # endregion
    
if __name__ == "__main__":
    rospy.init_node("visualizer", anonymous=True)
    visualizer = Visualizer()
    # visualizer.pub_stl_map()
    rospy.spin()