#! /usr/bin/env python

import rospy, copy, time, os, numpy as np
from msgs.srv import *
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point
from ins_mission_executer import InsMissionExecuter
from bspline.msg import Bspline
from std_msgs.msg import String
from msgs.msg import PlanVis, Combine_bspline_inspection_pose

class InsMeetingManager():
    def __init__(self, ins_mission_executer: InsMissionExecuter) -> None:
        self.ins_mission_executer: InsMissionExecuter = ins_mission_executer
        self.drone_id=rospy.get_param("~drone_id","-1")
        
        # meeting
        self.meeting_pose: PoseStamped | None = None
        self.ins_server = rospy.Service("exp_ins_meeting", exp_ins_meeting, self.exp_ins_meeting_cb)

        # visualization
        self.plan_vis_pub = rospy.Publisher("/plan_vis", PlanVis, queue_size=10)
        
        # main timer
        self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_cb)
        
    def timer_cb(self, event) -> None:
        # 如果完成了inspection任务，且meeting_pose被设置了，那么就去meeting_pose等着，一般用在切换bbox过程中
        if len(self.ins_mission_executer.ins_bsp_list) == 0 and self.meeting_pose and self.meeting_pose.header.stamp != rospy.Time(0):
            self.ins_mission_executer.go_to_meeting_pose(self.meeting_pose)
            self.meeting_pose = None

    def exp_ins_meeting_cb(self, req: exp_ins_meetingRequest) -> exp_ins_meetingResponse:
        rospy.logwarn(f"{self.drone_id}: Received exp_ins_meeting request.")
        

        self.meeting_pose = req.meeting_pose
     
        for cbi in req.combine_list:
            cbi: Combine_bspline_inspection_pose
            for bspline in cbi.bsplines:
                self.ins_mission_executer.ins_bsp_list.append(bspline)
            for pose in cbi.inspection_poses:
                self.ins_mission_executer.ins_bsp_list.append(pose)
        
        self.visualize(self.ins_mission_executer.ins_bsp_list)

        res: exp_ins_meetingResponse = exp_ins_meetingResponse()
        res.success = True
        res.finished_inspection_poses = self.ins_mission_executer.finished_inspection_poses
        self.ins_mission_executer.finished_inspection_poses = Path()
        return res

    def visualize(self, ins_bsp_list) -> None:
        plan_vis: PlanVis = PlanVis()
        plan_vis.msg = f"{self.drone_id},inspection_task"   
        for p_or_b in ins_bsp_list:
            if isinstance(p_or_b, Bspline):
                bspline: Bspline = p_or_b
                path = Path()
                for point in bspline.pos_pts:
                    pose = PoseStamped()
                    pose.pose.orientation.w = 1
                    pose.pose.position = Point(point.x, point.y, point.z)
                    path.poses.append(pose)
                plan_vis.paths.append(copy.deepcopy(path))
            else:
                pose: PoseStamped = p_or_b
                pose.pose.orientation.w = 1
                plan_vis.poses.append(copy.deepcopy(pose))
        self.plan_vis_pub.publish(plan_vis)
        