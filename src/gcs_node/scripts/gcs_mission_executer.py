#! /usr/bin/env python

import rospy, time, os, sys
sys.path.append(os.path.join(os.path.dirname(__file__), "../../"))
from mission_executer.scripts.mission_executer import MissionExecuter

class GcsMissionExecuter(MissionExecuter):       
    def __init__(self) -> None:
        super(GcsMissionExecuter, self).__init__()
        
    def set_meeting_manager(self, gcs_meeting_manager):
        self.gcs_meeting_manager = gcs_meeting_manager
        
    def after_finish_nav(self) -> None:
        if self.gcs_meeting_manager.state == "go to meeting":
            self.gcs_meeting_manager.change_state_to("reach meeting")
            return
        
    # override
    def launch_fastplanner(self) -> None:
        fast_map_size_x = abs(self.destination.poses[0].pose.position.x - self.position.x) + 20
        fast_map_size_y = abs(self.destination.poses[0].pose.position.y - self.position.y) + 20
        fast_map_size_z = 15
        rospy.set_param("fast_planner_node/sdf_map/map_size_x", fast_map_size_x)
        rospy.set_param("fast_planner_node/sdf_map/map_size_y", fast_map_size_y)
        rospy.set_param("fast_planner_node/sdf_map/map_size_z", fast_map_size_z)
        rospy.set_param("fast_planner_node/sdf_map/map_origin_x", min(self.destination.poses[0].pose.position.x, self.position.x) - 10)
        rospy.set_param("fast_planner_node/sdf_map/map_origin_y", min(self.destination.poses[0].pose.position.y, self.position.y) - 10)
        rospy.set_param("fast_planner_node/sdf_map/map_origin_z", min(self.destination.poses[0].pose.position.z, self.position.z) - 10)
        rospy.set_param("fast_planner_node/sdf_map/virtual_ceil_height", fast_map_size_z - 0.01)
        rospy.set_param("pcl_render_node/map/x_size", 100)
        rospy.set_param("pcl_render_node/map/y_size", 100)
        rospy.set_param("pcl_render_node/map/z_size", 100)
        
        os.system("roslaunch mission_executer fastplanner_node.launch &")
        self.start_nav_client.wait_for_service()
        resp = self.start_nav_client.call(self.destination)
        while not resp.success:
            rospy.logerr(self.drone_id + ": fail to trigger fastplanner node")
            time.sleep(1)
            resp = self.start_nav_client.call(self.destination)
