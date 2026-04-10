#! /usr/bin/env python

from typing import List
import rospy, sys, os
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from bspline.msg import Bspline
from std_msgs.msg import String
from msgs.srv import animation_status, animation_statusResponse

sys.path.append(os.path.join(os.path.dirname(__file__), "../../"))
from mission_executer.scripts.mission_executer import MissionExecuter

sys.path.append(os.path.join(os.path.dirname(__file__), "../../"))
from visualizer_node.scripts.animation import ResultVisualizer

class InsMissionExecuter(MissionExecuter):       
    def __init__(self) -> None:
        super(InsMissionExecuter, self).__init__()
        
        # inspection
        self.inspection_time = rospy.get_param("/inspection_time", 10)
        self.finished_inspection_poses = Path()
        self.state = "idle"
        self.timer = rospy.Timer(rospy.Duration(0.1), self.mission_execute_timer_cb)
        
        self.ins_bsp_list = [] # main mission list
        self.bspline_pub = rospy.Publisher("planning/bspline", Bspline, queue_size=10)
        
        # visualization
        self.feature_vis_pub = rospy.Publisher("/inspected_feature_vis", Path, queue_size=10)

        # animation
        # TODO: the fequency of the timer must be the same as the frequency of the timer in animation.py, otherwise the status will not be conituously updated
        self.animation_status_server = rospy.Service("/"+self.drone_id+"/status_animation", animation_status, self.animation_status_cb)

        # record
        from std_msgs.msg import String
        self.waste_time = None
        self.record_csv_pub = rospy.Publisher("/record_csv", String, queue_size=10)
        
    def animation_status_cb(self, req):
        if req.robot_id == self.drone_id:
            if self.state == "inspecting":
                status = "0" # "inspect"
                time_info = None
            elif self.state == "idle":
                status = "2" # "wait"
                time_info = None
            elif self.state == "going to inspection pose":
                status = "1" # travel"
                time_info = None
            else:
                status = "2" # "wait"
                time_info = None
            status_string = f"{self.drone_id},{status},{time_info}"
            resp = animation_statusResponse()
            resp.status = status_string
            return resp
        else:
            return
        

    def set_meeting_manager(self, ins_meeting_manager):
        self.ins_meeting_manager = ins_meeting_manager
        
    def mission_execute_timer_cb(self, event) -> None:
        if len(self.ins_bsp_list) == 0:
            return
        
        if self.state == "idle":
            mission = self.ins_bsp_list.pop(0)
            if isinstance(mission, PoseStamped):
                mission: PoseStamped
                path = Path()
                path.header.frame_id = "world"
                path.header.stamp = rospy.Time.now()
                mission.header.frame_id = self.drone_id
                path.poses.append(mission)
                self.feature_vis_pub.publish(path)
                self.finished_inspection_poses.poses.append(mission)
                self.start_inspecting_time = rospy.Time.now()
                self.change_state_to("inspecting")
            else:
                mission: Bspline
                duration = mission.end_time - mission.start_time
                mission.start_time = rospy.Time.now()
                mission.end_time = mission.start_time + duration
                self.bspline_pub.publish(mission)
                self.reach_time = mission.end_time
                self.change_state_to("going to inspection pose")
                
        elif self.state == "inspecting":
            if rospy.Time.now() - self.start_inspecting_time > rospy.Duration(self.inspection_time):
                self.change_state_to("idle")
                
        elif self.state == "going to inspection pose":
            if rospy.Time.now() > self.reach_time:
                self.change_state_to("idle")

    def change_state_to(self, state: str) -> None:
        self.state = state
        
        if state == "idle" and self.waste_time == None:
            self.waste_time = rospy.Time.now()
        elif state != "idle" and self.waste_time != None:
            self.record_csv_pub.publish(f"waste_time,{self.drone_id},{str(rospy.Time.now().to_sec() - self.waste_time.to_sec())}")
            self.waste_time = None