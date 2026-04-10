#! /usr/bin/env python

import rospy, sys
from std_msgs.msg import Empty
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_srvs.srv import Trigger, TriggerResponse
from sensor_msgs.msg import PointCloud2
from feature_scan import FeatureScan
from msgs.srv import waypoint
import time, os, copy
sys.path.append(os.path.join(os.path.dirname(__file__), "../../"))
from mission_executer.scripts.mission_executer import MissionExecuter

class ExpMissionExecuter(MissionExecuter):       
    def __init__(self) -> None:
        super(ExpMissionExecuter, self).__init__()

        # bbox
        self.bbox_start_time = None
        
        # exploration
        self.is_go_to_exploration: bool = False
        self.bbox_list: list = []
        self.finish_bbox = False
        self.start_exp_client = rospy.ServiceProxy("exploration_node/start_exploration", Trigger)
        self.finish_exp_sub = rospy.Subscriber("finish_exploration", Empty, self.finish_exp_cb)
        
        # visualization
        self.map: PointCloud2 = PointCloud2()
        self.map_vis_sub = rospy.Subscriber("sdf_map/occupancy_all", PointCloud2, self.map_vis_cb)
        self.map_vis_pub = rospy.Publisher("/sdf_map/occupancy_old", PointCloud2, queue_size=10)
        
        # go to meeting in exploration
        self.exp_pause_pub = rospy.Publisher("exploration_node/exp_pause", Empty, queue_size=10)
        self.nav_in_exp_client = rospy.ServiceProxy("exploration_node/waypoint", waypoint)
        self.plan_fail_signals: list = []
        self.plan_fail_sub = rospy.Subscriber("exploration_node/plan_fail", Empty, self.plan_fail_cb)

        # exp chck alive 
        self.exp_check_alive_timer_on : bool = False
        self.exp_check_alive_timer = rospy.Timer(rospy.Duration(5), self.exp_check_alive_cb)
        
        # feature scan control
        self.feature_scan: FeatureScan | None = None
      
    # region meeting  
      
    def set_meeting_manager(self, exp_meeting_manager):
        self.exp_meeting_manager = exp_meeting_manager
    
    # override  
    def go_to_meeting_pose(self, meeting_pose: PoseStamped):
        rospy.logerr(self.drone_id + ": go_to_meeting_pose_in_exp()")
        self.exp_pause_pub.publish(Empty())
        self.destination = Path()
        self.destination.poses.append(meeting_pose)
        self.nav_in_exp_client.wait_for_service()
        self.nav_in_exp_client.call(self.destination)
    
    # override 
    def meeting_end(self):
        if f"/{self.drone_id}/fast_planner_node" in os.popen("rosnode list").read():
            os.system(f"rosnode kill /{self.drone_id}/fast_planner_node")
        if self.is_go_to_exploration:
            rospy.logwarn(self.drone_id + ": meeting_end() and is_go_to_exploration")
            self.go_to_exploration(self.exp_meeting_manager.exp_position_new_bbox) # TODO
        elif self.finish_bbox == False:
            rospy.logwarn(self.drone_id + ": meeting_end() and finish_bbox == False, keep exploring")
            if f"/{self.drone_id}/exploration_node" not in os.popen("rosnode list").read():
                self.launch_exploration()
            try:
                self.start_exp_client.wait_for_service(timeout=15)
                self.start_exp_client.call()
                self.feature_scan.feature_scan_on = True
            except rospy.service.ServiceException as e:
                print(f"Service call failed: {e}")
                self.finish_bbox = True
                return
            self.reset_check_alive_params()
            self.exp_check_alive_timer_on = True
        else:
            return
        
    def plan_fail_cb(self, msg):
        rospy.logwarn(self.drone_id + ": plan_fail_cb()")
        self.plan_fail_signals.append(rospy.Time.now())
        for t in self.plan_fail_signals:
            if rospy.Time.now() - t > rospy.Duration(10):
                self.plan_fail_signals.remove(t)
            else:
                break
        if len(self.plan_fail_signals) > 20:
            rospy.logerr(self.drone_id + ": plan_fail too many times, try to recover")
            self.move_vibrate()
            self.plan_fail_signals.clear()
        
            
    # endregion
    
    # region exploration
    
    def go_to_exploration(self, exp_position_new_bbox: PoseStamped) -> None: # TODO
        rospy.logwarn(self.drone_id + ": go_to_exploration()")
        self.finish_bbox = False
        if len(self.bbox_list) == 0:
            rospy.logerr(self.drone_id + ": No bbox to go.")
            return
        self.destination.poses.append(exp_position_new_bbox)
        distance: float = (self.destination.poses[0].pose.position.x - self.position.x) ** 2 + (self.destination.poses[0].pose.position.y - self.position.y) ** 2 + (self.destination.poses[0].pose.position.z - self.position.z) ** 2
        if distance < 0.1:
            rospy.logwarn(self.drone_id + ": distance < 1, directly do_exploration()")
            self.is_go_to_exploration = False
            self.bbox_start_time = rospy.Time.now()
            rospy.logwarn(self.drone_id + ": set bbox_start_time")
            self.do_exploration()
        else:
            rospy.logwarn(self.drone_id + ": distance >= 1, do_navigation()")
            self.is_go_to_exploration = True
            self.do_navigation()
    
    # this function is currently not used, the exp_position_new_bbox is same as ins_position_new_bbox in exp_meeting_manager
    def generate_destination_from_bbox(self) -> Path:
        bbox_center_x = (self.bbox_list[0][0] + self.bbox_list[0][3]) / 2
        bbox_center_y = (self.bbox_list[0][1] + self.bbox_list[0][4]) / 2
        destination = Path()
        pose = PoseStamped()
        pi = 3.1416
        if self.position.x >= bbox_center_x and self.position.y >= bbox_center_y:
            pose.pose.position.x = self.bbox_list[0][3] - 1
            pose.pose.position.y = self.bbox_list[0][4] - 1
            pose.pose.position.z = self.bbox_list[0][2] + 1  
            self.ideal_yaw = -0.75 * pi
        elif self.position.x < bbox_center_x and self.position.y >= bbox_center_y:
            pose.pose.position.x = self.bbox_list[0][0] + 1
            pose.pose.position.y = self.bbox_list[0][4] - 1
            pose.pose.position.z = self.bbox_list[0][2] + 1 
            self.ideal_yaw = -0.25 * pi  
        elif self.position.x < bbox_center_x and self.position.y < bbox_center_y:
            pose.pose.position.x = self.bbox_list[0][0] + 1
            pose.pose.position.y = self.bbox_list[0][1] + 1
            pose.pose.position.z = self.bbox_list[0][2] + 1
            self.ideal_yaw = 0.25 * pi
        elif self.position.x >= bbox_center_x and self.position.y < bbox_center_y:
            pose.pose.position.x = self.bbox_list[0][3] - 1
            pose.pose.position.y = self.bbox_list[0][1] + 1
            pose.pose.position.z = self.bbox_list[0][2] + 1  
            self.ideal_yaw = 0.75 * pi
        destination.poses.append(pose)
        return destination
    
    def do_exploration(self) -> None:
        rospy.logwarn(self.drone_id + ": do_exploration()")
        self.turn_to(self.ideal_yaw)
        self.launch_exploration()
        self.stop_time = rospy.Time.now()
        self.exp_check_alive_timer_on = True
        
    def launch_exploration(self) -> None:
        rospy.set_param("drone_id", self.drone_id)  
        rospy.set_param("exploration_node/sdf_map/box_min_x", self.bbox_list[0][0])
        rospy.set_param("exploration_node/sdf_map/box_min_y", self.bbox_list[0][1])
        rospy.set_param("exploration_node/sdf_map/box_min_z", self.bbox_list[0][2])
        rospy.set_param("exploration_node/sdf_map/box_max_x", self.bbox_list[0][3])
        rospy.set_param("exploration_node/sdf_map/box_max_y", self.bbox_list[0][4])
        rospy.set_param("exploration_node/sdf_map/box_max_z", self.bbox_list[0][5])
        
        exp_map_size_x: float = self.bbox_list[0][3] - self.bbox_list[0][0] + 4
        exp_map_size_y: float = self.bbox_list[0][4] - self.bbox_list[0][1] + 4
        exp_map_size_z: float = self.bbox_list[0][5] - self.bbox_list[0][2] + 4
        exp_map_origin_x: float = self.bbox_list[0][0] - 2
        exp_map_origin_y: float = self.bbox_list[0][1] - 2
        exp_map_origin_z: float = self.bbox_list[0][2] - 2
        
        rospy.set_param("exploration_node/sdf_map/map_origin_x", exp_map_origin_x)
        rospy.set_param("exploration_node/sdf_map/map_origin_y", exp_map_origin_y)
        rospy.set_param("exploration_node/sdf_map/map_origin_z", exp_map_origin_z)
        rospy.set_param("exploration_node/sdf_map/map_size_x", exp_map_size_x)
        rospy.set_param("exploration_node/sdf_map/map_size_y", exp_map_size_y)
        rospy.set_param("exploration_node/sdf_map/map_size_z", exp_map_size_z)

        rospy.set_param("pcl_render_node/map/origin_x", exp_map_origin_x)
        rospy.set_param("pcl_render_node/map/origin_y", exp_map_origin_y)
        rospy.set_param("pcl_render_node/map/origin_z", exp_map_origin_z)
        rospy.set_param("pcl_render_node/map/x_size", exp_map_size_x)
        rospy.set_param("pcl_render_node/map/y_size", exp_map_size_y)
        rospy.set_param("pcl_render_node/map/z_size", exp_map_size_z)

        if "/" + self.drone_id + "/fast_planner_node" in os.popen("rosnode list").read():
            rospy.logerr(self.drone_id + ": fast planner node is still running")
            os.system("rosnode kill /" + self.drone_id + "/fast_planner_node")
            time.sleep(1)
        if "/" + self.drone_id + "/exploration_node" in os.popen("rosnode list").read():
            rospy.logerr(self.drone_id + ": exploration node is already running")
            os.system("rosnode kill /" + self.drone_id + "/exploration_node")
            time.sleep(1)
        
        os.system("roslaunch mission_executer exploration_node.launch &")
        
        self.start_exp_client.wait_for_service()
        time.sleep(2)
        self.look_around()
        self.move_vibrate()
        # while flag.success == 0:
        #     rospy.logerr(self.drone_id + ": fail to trigger exploration node")
        #     time.sleep(1)
        try:
                flag: TriggerResponse = self.start_exp_client.call()
                self.feature_scan.feature_scan_on = True
                self.call_exploration_time = rospy.Time.now()
        except rospy.service.ServiceException as e:
                print(f"Service call failed: {e}")
                self.relaunch_exploration()
        self.reset_check_alive_params()
        
    def exp_check_alive_cb(self, event):
        if self.exp_check_alive_timer_on == False:
            rospy.logwarn(self.drone_id + "whether exp_check_alive_on" + str(self.exp_check_alive_timer_on))
            return
        if self.finish_bbox == True:
            rospy.logwarn(self.drone_id + "whether finish bbox" + str(self.finish_bbox))
            self.exp_check_alive_timer_on = False
            return
        if self.velocity != 0:
            self.reset_check_alive_params()
            return
        current_time: rospy.Time = rospy.Time.now()
        if current_time - self.stop_time > rospy.Duration(10):
            if current_time - self.call_exploration_time > rospy.Duration(20):
                for i in range(1):  # 尝试三次
                    try:
                        self.start_exp_client.wait_for_service(timeout=5)
                        break  # 如果成功，跳出循环
                    except rospy.ROSException:
                        rospy.logwarn(self.drone_id + "等待服务超时，尝试重新连接...")
                        rospy.logerr(self.drone_id + "尝试次数: " + str(i + 1))
                        rospy.logerr(self.drone_id + "whether finish bbox" + str(self.finish_bbox))
                        time.sleep(1)  # 等待1秒后重试
                else:  # 如果三次尝试都失败了
                    rospy.logerr(self.drone_id + "无法连接到服务, 重启exploration_node")
                    self.relaunch_exploration()
            if current_time - self.stop_time > rospy.Duration(10):
                self.move_vibrate()

    def recovery_after_long_time_cb(self, event):
        current_time: rospy.Time = rospy.Time.now()
        if current_time - self.stop_time > rospy.Duration(20):
            if self.whether_reach_destination == False:
                self.re_do_navigation()
                self.nav_check_alive_timer_on = True
                rospy.logerr(self.drone_id + ": recovery_after_long_time_cb()")
            else:
                self.relaunch_exploration()
            
    def relaunch_exploration(self) -> None:
        current_time: rospy.Time = rospy.Time.now()
        if current_time - self.call_exploration_time > rospy.Duration(10):
            self.launch_exploration()
            
    def finish_exp_cb(self, msg) -> None:
        self.feature_scan.feature_scan_on = False
        self.finish_bbox = True
        self.exp_check_alive_timer_on = False
        self.map_vis_pub.publish(self.map)
        # time.sleep(1)
        # if "/" + self.drone_id + "/exploration_node" in os.popen("rosnode list").read():
        #     rospy.logerr(self.drone_id + "finish_exp_cb(): kill exploration node")
        #     os.system("rosnode kill /" + self.drone_id + "/exploration_node")
        # time.sleep(1)
        
    # endregion
     
    def receive_bbox(self, bbox):
        self.bbox_list.clear()
        for box in bbox:
            boundingbox = box.split(",")
            for i in range(6):
                boundingbox[i] = float(boundingbox[i])
            box = copy.deepcopy(boundingbox)
            self.bbox_list.append(box)     

    def after_finish_nav(self) -> None:
        if self.exp_meeting_manager.state == "go to meeting with gcs":
            self.exp_meeting_manager.change_state_to("reach meeting with gcs")
            return
        if self.exp_meeting_manager.state == "go to meeting with ins":
            self.exp_meeting_manager.change_state_to("reach meeting with ins")
            return
        if self.is_go_to_exploration:
            self.is_go_to_exploration = False
            self.bbox_start_time = rospy.Time.now() 
            rospy.logwarn(self.drone_id + ": set bbox_start_time")
            self.do_exploration() 
       
    def map_vis_cb(self, msg: PointCloud2) -> None:
        self.map = msg
