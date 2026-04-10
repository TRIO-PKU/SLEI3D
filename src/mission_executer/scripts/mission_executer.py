from abc import abstractmethod
import rospy, os, time, random, copy, tf.transformations
import numpy as np
from geometry_msgs.msg import Point32, PoseStamped
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Float32, String, Empty
from msgs.srv import waypoint, waypointResponse, check_los, check_losResponse, check_losRequest

class MissionExecuter:       
    def __init__(self) -> None:
        self.drone_id: String = rospy.get_param("~drone_id", "-1")
        
        # odom information
        self.position: Point32 = Point32()
        self.yaw = 0
        self.velocity = 0
        self.odom_sub = rospy.Subscriber("odom", Odometry, self.odom_cb)
        
        # navigation
        self.whether_reach_destination: bool = False
        self.destination: Path = Path()
        self.start_nav_client = rospy.ServiceProxy("waypoint", waypoint)
        self.finish_nav_sub = rospy.Subscriber("finish_fastplanning", Empty, self.finish_nav_cb)
        
        # recovery mode
        self.start_time: rospy.Time = rospy.Time.now() 
        self.stop_time: rospy.Time = rospy.Time.now()
        self.check_alive_timer_on: bool = False
        self.check_alive_times = []
        self.check_alive_timer = rospy.Timer(rospy.Duration(0.5), self.nav_check_alive_cb)
        self.recovery_after_long_time = rospy.Timer(rospy.Duration(10), self.recovery_after_long_time_cb)
        
        # direct control
        self.ideal_yaw = 0
        self.ideal_yaw_pub = rospy.Publisher("ideal_yaw", Float32, queue_size=10)
        self.ideal_pos_pub = rospy.Publisher("ideal_pos", Point32, queue_size=10)
        
        # check los
        self.check_los_client = rospy.ServiceProxy("/check_los", check_los)
        self.fastplanner_call_time = rospy.Time.now()
    
    # region meeting
         
    def go_to_meeting_pose(self, meeting_pose: PoseStamped):
        rospy.logerr(self.drone_id + ": go_to_meeting_pose_normal()")
        if "/" + self.drone_id + "/fast_planner_node" in os.popen("rosnode list").read():
            rospy.logerr(self.drone_id + ": When go_to_meeting_pose_normal(), need kill fast planner node first.")
            self.check_alive_timer_on = False
            os.system("rosnode kill /" + self.drone_id + "/fast_planner_node")

        self.destination = Path()
        self.destination.header.frame_id = "world"
        self.destination.header.stamp = rospy.Time.now()
        self.destination.poses.append(meeting_pose)
        self.do_navigation()
        
    def meeting_end(self):
        pass
    
    # endregion
    
    # region navigation
            
    def do_navigation(self) -> None:
        '''After set self.destination, run do_navigation to go to the destination'''
        if self.is_close_destination():
            rospy.logwarn(self.drone_id + ": is close to destination.")
            self.check_alive_timer_on = False
            self.after_finish_nav()
        else:
            self.whether_reach_destination = False
            self.reset_check_alive_params()
            self.check_alive_timer_on = True
            self.launch_fastplanner()

    def reset_check_alive_params(self) -> None:
        self.start_time = rospy.Time.now()
        self.stop_time = rospy.Time.now()

    def recovery_after_long_time_cb(self, event):
        current_time: rospy.Time = rospy.Time.now()
        if current_time - self.stop_time > rospy.Duration(20) and self.whether_reach_destination == False:
            self.re_do_navigation()
            self.nav_check_alive_timer_on = True
            rospy.logerr(self.drone_id + ": recovery_after_long_time_cb()")
                  
    def nav_check_alive_cb(self, event):
        current_time: rospy.Time = rospy.Time.now()

        if not hasattr(self, 'call_times'):
            self.check_alive_times = []
        self.check_alive_times.append(current_time)
        self.check_alive_times = [t for t in self.check_alive_times if current_time - t <= rospy.Duration(5)]
        if len(self.check_alive_times) >= 9:
            rospy.sleep(3)
            self.check_alive_times.clear()
            return

        if not self.check_alive_timer_on:
            self.reset_check_alive_params()
            return
        
        if self.velocity != 0:
            self.reset_check_alive_params()
            return
        
        if current_time - self.stop_time > rospy.Duration(30):
            rospy.logwarn(self.drone_id + ": check alive timer start")
            if current_time - self.fastplanner_call_time > rospy.Duration(60):
                rospy.logerr(self.drone_id + ": check alive timer: re_do_navigation()")
                self.re_do_navigation()
                return
            else:
                rospy.logerr(self.drone_id + ": check alive timer: move vibrate")
                self.move_vibrate()
                return
                
    def launch_fastplanner(self) -> None:
        fast_map_size_x = abs(self.destination.poses[0].pose.position.x - self.position.x) + 20
        fast_map_size_y = abs(self.destination.poses[0].pose.position.y - self.position.y) + 20
        fast_map_size_z = abs(self.destination.poses[0].pose.position.z - self.position.z) + 3
        fast_map_origin_x = min(self.destination.poses[0].pose.position.x, self.position.x) - 10
        fast_map_origin_y = min(self.destination.poses[0].pose.position.y, self.position.y) - 10
        fast_map_origin_z = min(self.destination.poses[0].pose.position.z, self.position.z) - 1.5
        
        rospy.set_param("fast_planner_node/sdf_map/map_origin_x", fast_map_origin_x)
        rospy.set_param("fast_planner_node/sdf_map/map_origin_y", fast_map_origin_y)
        rospy.set_param("fast_planner_node/sdf_map/map_origin_z", fast_map_origin_z)
        rospy.set_param("fast_planner_node/sdf_map/map_size_x", fast_map_size_x)
        rospy.set_param("fast_planner_node/sdf_map/map_size_y", fast_map_size_y)
        rospy.set_param("fast_planner_node/sdf_map/map_size_z", fast_map_size_z)
        rospy.set_param("fast_planner_node/sdf_map/virtual_ceil_height", fast_map_origin_z + fast_map_size_z - 0.1)
        rospy.set_param("fast_planner_node/map_ros/visualization_truncate_height", fast_map_origin_z + fast_map_size_z - 0.2)
        rospy.set_param("fast_planner_node/map_ros/visualization_truncate_low", fast_map_origin_z + 0.1)
        
        rospy.set_param("pcl_render_node/map/origin_x", fast_map_origin_x)
        rospy.set_param("pcl_render_node/map/origin_y", fast_map_origin_y)
        rospy.set_param("pcl_render_node/map/origin_z", fast_map_origin_z)
        rospy.set_param("pcl_render_node/map/x_size", fast_map_size_x)
        rospy.set_param("pcl_render_node/map/y_size", fast_map_size_y)
        rospy.set_param("pcl_render_node/map/z_size", fast_map_size_z)
        
        if "/" + self.drone_id + "/fast_planner_node" in os.popen("rosnode list").read():
            rospy.logerr(self.drone_id + ": fast planner node is already running")
            os.system("rosnode kill /" + self.drone_id + "/fast_planner_node")
            time.sleep(2)
        os.system(f"roslaunch mission_executer fastplanner_node.launch namespace:={self.drone_id} &")
        
        self.start_nav_client.wait_for_service()
        time.sleep(0.5)
        try:
            resp: waypointResponse = self.start_nav_client.call(self.destination)
            self.fastplanner_call_time = rospy.Time.now()
        except rospy.ServiceException:
            rospy.logerr(self.drone_id + ": fail to trigger fast planner node")
            self.re_do_navigation()
            return
    
    def re_do_navigation(self) -> None:
        # self.check_alive_timer_on = False
        rospy.logerr(self.drone_id + ": re_do_navigation()")
        if "/" + self.drone_id + "/fast_planner_node" in os.popen("rosnode list").read():
            rospy.logwarn(self.drone_id + ": re_do_navigation(), kill fast planner node.")
            os.system("rosnode kill /" + self.drone_id + "/fast_planner_node")
        self.do_navigation()
    
    def finish_nav_cb(self, msg: Empty) -> None:
        self.whether_reach_destination = True
        self.check_alive_timer_on = False
        rospy.logerr(self.drone_id + ": finish_nav_cb(), kill fast planner node.")
        os.system("rosnode kill /" + self.drone_id + "/fast_planner_node")
        self.after_finish_nav()
        
    def after_finish_nav(self) -> None:
        '''This function will be called after the drone reaches the destination. You should implement this function to do something after the drone reaches the destination.'''
        pass
    
    # endregion
        
    # region direct_control
    
    def turn_to(self, yaw: float) -> None:
        d = 1
        if yaw < self.yaw:
            d = -1
        for i in range(int(self.yaw * 100), int(yaw * 100), d):
            y =  0.01 * i
            if y > 3.1416:
                y -= 6.2832
            elif y < -3.1416:
                y += 6.2832
            self.ideal_yaw_pub.publish(y)
            time.sleep(0.01)
            
    def look_around(self) -> None:
        current_i = int(self.yaw * 180 / 3.1416)
        for i in range(current_i, 360 + current_i, 1):
            if i > 180:
                yaw = (i - 360) / 180 * 3.1416
            else:
                yaw = i / 180 * 3.1416
            self.ideal_yaw_pub.publish(yaw)
            time.sleep(0.01)
            
    def move_vibrate(self) -> None:
        pos = Point32()
        pos.x = self.position.x + random.uniform(-0.2, 0.2)
        pos.y = self.position.y + random.uniform(-0.2, 0.2)
        pos.z = self.position.z
        cur_pos = np.zeros(3)
        cur_pos[0] = self.position.x 
        cur_pos[1] = self.position.y
        cur_pos[2] = self.position.z
        vector = np.zeros(3)
        vector[0] = pos.x - self.position.x
        vector[1] = pos.y - self.position.y
        vector[2] = pos.z - self.position.z
        for i in range(1, 101):
            pos_pub = Point32()
            pos_pub.x = cur_pos[0] + vector[0] * 0.01 * i
            pos_pub.y = cur_pos[1] + vector[1] * 0.01 * i
            pos_pub.z = cur_pos[2] + vector[2] * 0.01 * i
            self.ideal_pos_pub.publish(pos_pub)
            time.sleep(0.01)
            
    def move_circle(self) -> None:
        center = copy.deepcopy(self.position)
        for i in range(100):
            pos = Point32()
            pos.x = center.x + i * 0.01
            pos.y = center.y
            pos.z = center.z
            self.ideal_pos_pub.publish(pos)
            time.sleep(0.01)
        for i in range(360):
            pos = Point32()
            pos.x = center.x + np.cos(i / 180 * np.pi)
            pos.y = center.y + np.sin(i / 180 * np.pi)
            pos.z = center.z
            self.ideal_pos_pub.publish(pos)
            time.sleep(0.01)
        for i in range(100):
            pos = Point32()
            pos.x = center.x + 1 - i * 0.01
            pos.y = center.y
            pos.z = center.z
            self.ideal_pos_pub.publish(pos)
            time.sleep(0.01)

    # endregion
    
    # region others

    def check_los(self, target_id: str) -> bool:
        req = check_losRequest()
        req.client_id = self.drone_id
        req.target_id = target_id
        self.check_los_client.wait_for_service()
        resp: check_losResponse = self.check_los_client.call(req)
        return resp.success
    
    def is_close_destination(self) -> bool:
        distance = np.sqrt((self.destination.poses[0].pose.position.x - self.position.x) ** 2 + (self.destination.poses[0].pose.position.y - self.position.y) ** 2 + (self.destination.poses[0].pose.position.z - self.position.z) ** 2)
        if distance < 0.1:
            return True
        else:
            return False
        
    def odom_cb(self, msg: Odometry) -> None:
        self.position.x = msg.pose.pose.position.x
        self.position.y = msg.pose.pose.position.y
        self.position.z = msg.pose.pose.position.z
        self.yaw = tf.transformations.euler_from_quaternion([
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ])[2]
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        vz = msg.twist.twist.linear.z
        self.velocity = np.sqrt(vx ** 2 + vy ** 2 + vz ** 2)
        
    # endregion