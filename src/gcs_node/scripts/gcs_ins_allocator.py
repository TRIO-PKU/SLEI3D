#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
import yaml
import os
import numpy as np
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import  Path
from msgs.srv import bbox,bboxRequest,bboxResponse
from msgs.srv import inspector_allocation,inspector_allocationRequest,inspector_allocationResponse


class InsAllocator():
    def __init__(self):
        self.current_dir = os.path.dirname(__file__)  
        self.yaml_file_path = os.path.join(self.current_dir, 'exp_info.yaml') 
        self.exp_num = rospy.get_param("/exp_num")
        self.ins_num = rospy.get_param("/ins_num")
        self.exp_bbox_volume_dict = {} 
        self.exp_insnum_dict = {}
        self.exp_ins_dict = {}

    # def init_ins(self):
    #     total_bbox_volume = 0
    #     while total_bbox_volume == 0:
    #         self.init_exp_bbox_volume()
    #         total_bbox_volume = self.compute_total_volume()
    #     already_allocated_ins_num = 0
    #     for i in range(1, self.exp_num+1):
    #         ratio = self.exp_bbox_volume_dict["exp_"+str(i)]/total_bbox_volume
    #         # 每个exp的ins数量是ins_num*ratio再向下取整
    #         ins_num = int(self.ins_num*ratio)
    #         self.exp_insnum_dict["exp_"+str(i)] = ins_num
    #         already_allocated_ins_num += ins_num
    #     left_ins_num = int(self.ins_num)-already_allocated_ins_num
    #     for i in range(1, left_ins_num+1):
    #         self.exp_insnum_dict["exp_"+str(i)] += 1
    #     self.allocate_inspector()
    #     return self.exp_ins_dict

    # def allocate_inspector(self):
    #     first_inspector_id = 1
    #     with open(self.yaml_file_path, 'r') as f:
    #         content = yaml.load(f, Loader=yaml.FullLoader)
    #     for i in range(1, self.exp_num+1):
    #         ins_num = self.exp_insnum_dict["exp_"+str(i)]
    #         first_inspector_id=first_inspector_id
    #         last_inspector_id=first_inspector_id+ins_num-1
    #         ins_list=[]
    #         for j in range(first_inspector_id, last_inspector_id+1):
    #             ins_id="ins_"+str(j)
    #             ins_list.append(ins_id)
    #         self.exp_ins_dict[i] = ins_list
    #         first_inspector_id += ins_num
    #         content[i-1]["exp_"+str(i)][1]["ins_id"] = list(range(first_inspector_id-ins_num, first_inspector_id))
    #         content[i-1]["exp_"+str(i)][1]["ins_num"] = ins_num
    #         with open(self.yaml_file_path, 'w') as f:
    #             yaml.dump(content, f)


    # def init_exp_bbox_volume(self):
    #     with open(self.yaml_file_path, 'r') as f:
    #         content = yaml.load(f, Loader=yaml.FullLoader)
    #         for i in range(1, self.exp_num+1):
    #             exp_id = "exp_"+str(i)                
    #             bbox_list = content[i-1][exp_id][0]["bbox"]
    #             bbox_volume = 0
    #             for j in range(len(bbox_list)):
    #                 bbox = bbox_list[j]
    #                 bbox_volume += (bbox[3]-bbox[0])*(bbox[4]-bbox[1])*(bbox[5]-bbox[2])
    #             self.exp_bbox_volume_dict[exp_id] = bbox_volume

    # def compute_total_volume(self):
    #     total_bbox_volume = 0
    #     with open(self.yaml_file_path, 'r') as f:
    #         content = yaml.load(f, Loader=yaml.FullLoader)
    #         for i in range(1, self.exp_num+1):
    #             exp_id = "exp_"+str(i)                
    #             bbox_list = content[i-1][exp_id][0]["bbox"]
    #             for j in range(len(bbox_list)):
    #                 bbox = bbox_list[j]
    #                 total_bbox_volume += (bbox[3]-bbox[0])*(bbox[4]-bbox[1])*(bbox[5]-bbox[2])
    #     return total_bbox_volume

    def init_ins(self):
        with open(self.yaml_file_path, 'r') as f:
            content = yaml.load(f, Loader=yaml.FullLoader)
        each_ins_base = int(self.ins_num)// int(self.exp_num)
        left_ins_num = int(self.ins_num) % int(self.exp_num)
        add_ins=left_ins_num
        first_ins_id=1
        for i in range(1,int(add_ins)+1):
            first_ins_id=first_ins_id
            last_ins_id=first_ins_id+each_ins_base
            ins_list=[]
            for j in range(first_ins_id, last_ins_id+1):
                ins_id="ins_"+str(j)
                ins_list.append(ins_id)
            self.exp_ins_dict[i] = ins_list
            content[i-1]["exp_"+str(i)][1]["ins_id"] = list(range(first_ins_id, last_ins_id+1))
            first_ins_id = last_ins_id + 1
            with open(self.yaml_file_path, 'w') as f:
                yaml.dump(content, f)
            
        for i in range(int(add_ins)+1,int(self.exp_num)+1):
            rospy.logwarn("i:{}".format(i)) 
            first_ins_id=first_ins_id
            rospy.logwarn("first_inspector_id:{}".format(first_ins_id))
            last_ins_id=first_ins_id+each_ins_base-1
            rospy.logwarn("last_inspector_id:{}".format(last_ins_id))
            ins_list=[]
            for j in range(first_ins_id, last_ins_id+1):
                ins_id="ins_"+str(j)
                ins_list.append(ins_id)
                rospy.logwarn("ins_id:{}".format(ins_id))
            self.exp_ins_dict[i] = ins_list
            content[i-1]["exp_"+str(i)][1]["ins_id"] = list(range(first_ins_id, last_ins_id+1))
            first_ins_id = last_ins_id + 1
            with open(self.yaml_file_path, 'w') as f:
                yaml.dump(content, f)
        return self.exp_ins_dict
    