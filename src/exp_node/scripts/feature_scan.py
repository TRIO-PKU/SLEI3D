#! /usr/bin/env python

import rospy, numpy as np, tf.transformations, copy
from geometry_msgs.msg import PoseStamped, Point32
from nav_msgs.msg import Path, Odometry

CAMERA_RANGE = 5

class FeatureScan():
    def __init__(self, point_list):

        self.drone_id=rospy.get_param("~drone_id","-1")

        # control
        self.feature_scan_on = False
        
        # feature
        self.feature_all = None
        self.point_list: list[PoseStamped] = point_list
        self.scaned_num: int = 0
        self.get_feature_all: bool = False
        self.feature_all_sub = rospy.Subscriber("/feature_all", Path, self.feature_all_cb)

        # bbox
        self.bbox: list = None
        
        # odom information
        self.position: Point32 = Point32()
        self.yaw = 0
        self.odom_sub = rospy.Subscriber("odom", Odometry, self.odom_cb)
        
        # scan feature
        self.scan_feature_timer = rospy.Timer(rospy.Duration(1), self.scan_feature)
        
        # visualization
        self.found_feature_vis_pub = rospy.Publisher("/found_feature_vis", PoseStamped, queue_size=10)
        
    def scan_feature(self, event):
        if not self.feature_scan_on:
            return
        #判断是否有 feature 点落在球坐标系下以odom位置为远点，yaw方向为正前方，theta、phi、r定义的相机视野范围内
        if self.feature_all is None:
            return
        delete_index_list = []
        for i in range(len(self.feature_all)):
            feature = self.feature_all[i]
            dx = feature[0] - self.position.x
            dy = feature[1] - self.position.y
            dz = feature[2] - self.position.z
            theta = np.arctan2(dy, dx)
            phi = np.arctan2(dz, np.sqrt(dx**2 + dy**2))
            r = np.sqrt(dx**2 + dy**2 + dz**2)
            if theta - self.yaw < np.pi / 4 and theta - self.yaw > -np.pi / 4 and phi < np.pi / 6 and phi > -np.pi / 6:
                if r < CAMERA_RANGE and self.bbox is not None and feature[0] > self.bbox[0] and feature[0] < self.bbox[3] and feature[1] > self.bbox[1] and feature[1] < self.bbox[4] and feature[2] > self.bbox[2] and feature[2] < self.bbox[5]:
                    found_feature = PoseStamped()
                    found_feature.header.stamp = self.feature_all[i][3]
                    found_feature.header.frame_id = "world"
                    found_feature.pose.position.x = feature[0]
                    found_feature.pose.position.y = feature[1]
                    found_feature.pose.position.z = feature[2]
                    self.found_feature_vis_pub.publish(found_feature)  
                    
                    pose = PoseStamped()
                    pose.header.stamp = self.feature_all[i][3]
                    pose.header.frame_id = "world"
                    pose.pose.position.x = self.position.x
                    pose.pose.position.y = self.position.y
                    pose.pose.position.z = self.position.z 
                    pose.pose.orientation.z = np.sin(self.yaw / 2)
                    pose.pose.orientation.w = np.cos(self.yaw / 2)
                    pose.pose.orientation.x = 0
                    pose.pose.orientation.y = 0
                    self.point_list.append(pose)
                    self.scaned_num = self.scaned_num + 1
                    
                    delete_index_list.append(i) 
                    break
        self.feature_all = np.delete(self.feature_all, delete_index_list, axis=0)

    def receive_bbox(self, bbox):
        boundingbox = bbox[0].split(",")
        for i in range(6):
            boundingbox[i] = float(boundingbox[i])
        box = copy.deepcopy(boundingbox)
        self.bbox = box  
        
    def feature_all_cb(self, msg: Path):
        if self.get_feature_all:
            return
        self.feature_all = np.array([[pose.pose.position.x, pose.pose.position.y, pose.pose.position.z, pose.header.stamp] for pose in msg.poses])
        self.get_feature_all = True
        
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
        