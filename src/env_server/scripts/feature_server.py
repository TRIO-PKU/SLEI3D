#! /usr/bin/env python

import rospy, time, yaml, random, os
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from voxel_map_server import VoxelMapServer

FEATURE_NUM = 150

class FeatureServer():
    def __init__(self, voxel_map_server: VoxelMapServer):
        self.sparse_point_map = voxel_map_server.sparse_point_map
        self.feature_all: list[PoseStamped] = []
        self.bbox_filename = rospy.get_param("/bbox_description_path")
        self.bbox_feature_num_dict = {}
        self.read_bbox_file()
        self.split_pcl_to_bbox()
        self.generate_feature()
        self.feature_vis_pub = rospy.Publisher("/init_feature_vis", Path, queue_size=10)
        self.feature_pub = rospy.Publisher("/feature_all", Path, queue_size=10)
        self.bbox_feature_num_pub = rospy.Publisher("/bbox_feature_num", String, queue_size=10)
        self.feature_pub_timer = rospy.Timer(rospy.Duration(1), self.publish_feature_cb)
        time.sleep(1)
        self.publish_init_feature_vis()
        for i in range(5):
            time.sleep(3)
            self.publish_feature_num()
        

    def read_bbox_file(self):
        self.boundingbox_dict = {}
        f = open(self.bbox_filename, "r")
        boxes = yaml.load(f, Loader=yaml.FullLoader)
        i: int = 1
        for box in boxes:
            box = boxes[box]["boundingbox"]
            self.boundingbox_dict[i] = box
            i += 1

    def split_pcl_to_bbox(self):
        self.upper_points_map = []
        for point in self.sparse_point_map:
            if point[2] > 1:
                self.upper_points_map.append(point)

        self.pcl_to_bbox_dict = {}
        for i in range(1, len(self.boundingbox_dict) + 1):
            bbox = self.boundingbox_dict[i]
            for point in self.upper_points_map:
                if bbox[0] < point[0] < bbox[3] and bbox[1] < point[1] < bbox[4] and bbox[2] < point[2] < bbox[5]:
                    if i not in self.pcl_to_bbox_dict:
                        self.pcl_to_bbox_dict[i] = []
                    self.pcl_to_bbox_dict[i].append(point)

    def generate_feature(self):
        bbox_random_denisty_dict = self.generate_random_density()
        rospy.logerr("Random density: " + str(bbox_random_denisty_dict))

        for i in self.pcl_to_bbox_dict.keys():
            bbox_pcl_list: list = self.pcl_to_bbox_dict[i]
            density = bbox_random_denisty_dict[i]
            bbox_feature_num = int(FEATURE_NUM * density)  
            self.bbox_feature_num_dict[i] = bbox_feature_num
            if bbox_feature_num == 0:
                continue
            rospy.logerr("Bbox " + str(i) + " feature num: " + str(bbox_feature_num))

            step = len(bbox_pcl_list) // bbox_feature_num

            for j in range(step, len(bbox_pcl_list), step):
                point = bbox_pcl_list[j]
                feature = PoseStamped()
                feature.header.stamp = rospy.Time.now()
                feature.header.frame_id = "world"
                feature.pose.position.x = point[0]
                feature.pose.position.y = point[1]
                feature.pose.position.z = point[2]
                self.feature_all.append(feature)

        rospy.logerr("Feature points generated: " + str(len(self.feature_all)))

    def generate_random_density(self):
        random_numbers = []
        for i in range(len(self.pcl_to_bbox_dict)):
            random_numbers.append(10)
        random_numbers[0] = 30
        normalized_numbers_list = [num / sum(random_numbers) for num in random_numbers]

        bbox_random_denisty_dict = {}
        for i in range(len(normalized_numbers_list)):
            bbox_random_denisty_dict[i+1] = normalized_numbers_list[i]
        return bbox_random_denisty_dict
    
    def publish_feature_num(self):
        feature_string = String()
        feature_string.data = ""
        for i in self.bbox_feature_num_dict.keys():
            feature_string.data += str(i) + ":" + str(self.bbox_feature_num_dict[i]) + "\n"
        self.bbox_feature_num_pub.publish(feature_string)

            
    def publish_feature_cb(self, event):
        feature_list: Path = Path()
        for feature in self.feature_all:
            feature_list.poses.append(feature)
        feature_list.header.frame_id = "world"
        feature_list.header.stamp = rospy.Time.now() 
        self.feature_pub.publish(feature_list)
        
    def publish_init_feature_vis(self):
        feature_list: Path = Path()
        for feature in self.feature_all:
            feature_list.poses.append(feature)
        feature_list.header.frame_id = "world"
        feature_list.header.stamp = rospy.Time.now() 
        self.feature_vis_pub.publish(feature_list)
        