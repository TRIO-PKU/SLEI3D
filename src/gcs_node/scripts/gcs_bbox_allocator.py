#!/usr/bin/env python
# license removed for brevity
import rospy
import copy
from std_msgs.msg import String
import numpy as np
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import  Path
import yaml
import os


class BboxAllocator():
    def __init__(self, drone_id, bbox_filename, each_bbox_num):
        self.current_dir = os.path.dirname(__file__)  
        self.yaml_file_path = os.path.join(self.current_dir, 'exp_info.yaml') 

        self.drone_id = drone_id
        self.bbox_filename = bbox_filename  
        self.exp_num = rospy.get_param("/exp_num")
        self.ins_num = rospy.get_param("/ins_num")

        self.each_bbox_num=each_bbox_num
        self.exp_bbox_dict={}
        self.exp_bbox_id_dict={}
        self.boundingbox_dict={}
        self.last_bbox_dict={}
        for i in range(int(self.exp_num)):
            self.last_bbox_dict[i+1]=[0,0,0,0,0,0]
        self.init_yaml()
        self.read_file()

    def init_yaml(self):
        # 先清空yaml文件中原有内容
        document=[]
        for i in range(1,int(self.exp_num)+1):
            data = {
                "exp_"+str(i): [{"bbox_id": [], "bbox": []},{"ins_id":[],"ins_num":0},{"feature_num":0},{"meeting_time":0,"meeting_point":[0,0,0]}]
                }
            document.append(data)
        with open(self.yaml_file_path, 'w') as f:
                yaml.dump(document, f)
                
    def read_file(self):
        f = open(self.bbox_filename, "r")
        boxes = yaml.load(f, Loader=yaml.FullLoader)
        # 打印boxes
        bbox_list=[]
        for box in boxes:
            box=boxes[box]["boundingbox"]
            bbox_list.append(box)    
        for i in range(len(bbox_list)):
            self.boundingbox_dict[i+1]=bbox_list[i]

    def init_bbox(self):
        with open(self.yaml_file_path, 'r') as f:
            content = yaml.load(f, Loader=yaml.FullLoader)
            for i in range(1,self.exp_num+1):
                bbox=[]
                last_bbox=self.last_bbox_dict[i]
                nearest_bbox, nearest_bbox_id=self.compare_bbox(last_bbox)
                del self.boundingbox_dict[nearest_bbox_id]
                string=""
                string+=str(nearest_bbox[0])+","+str(nearest_bbox[1])+","+str(nearest_bbox[2])+","+str(nearest_bbox[3])+","+str(nearest_bbox[4])+","+str(nearest_bbox[5])
                bbox.append(string)
                content[i-1]["exp_"+str(i)][0]["bbox"].append(nearest_bbox)
                content[i-1]["exp_"+str(i)][0]["bbox_id"].append(nearest_bbox_id)
                self.last_bbox_dict[i]=nearest_bbox
                self.exp_bbox_dict[i]=bbox
                self.exp_bbox_id_dict[i]=nearest_bbox_id
                with open(self.yaml_file_path, 'w') as f:
                    yaml.dump(content, f)
        return self.exp_bbox_dict, self.exp_bbox_id_dict
    
    def reallocate_bbox(self,exp_id,finished_bbox):
        with open(self.yaml_file_path, 'r') as f:
            content = yaml.load(f, Loader=yaml.FullLoader)
            if finished_bbox==False:
                return ["None"],0
            else:
                if len(self.boundingbox_dict)==0:
                    return ["no bbox left"],0
                bbox=[]
                temp_last_bbox=self.last_bbox_dict[exp_id]
                nearest_bbox, nearest_bbox_id=self.compare_bbox(temp_last_bbox)
                del self.boundingbox_dict[nearest_bbox_id]
                string=""
                string+=str(nearest_bbox[0])+","+str(nearest_bbox[1])+","+str(nearest_bbox[2])+","+str(nearest_bbox[3])+","+str(nearest_bbox[4])+","+str(nearest_bbox[5])
                content[exp_id-1]["exp_"+str(exp_id)][0]["bbox"].append(nearest_bbox)
                content[exp_id-1]["exp_"+str(exp_id)][0]["bbox_id"].append(nearest_bbox_id)
                bbox.append(string)
                self.last_bbox_dict[exp_id]=nearest_bbox
                with open(self.yaml_file_path, 'w') as f:
                    yaml.dump(content, f)
                return bbox, nearest_bbox_id

    def compare_bbox(self, last_bbox):
        min_distance=float("inf")
        nearest_bbox=[]
        nearest_bbox_id: int = 0
        if len(self.boundingbox_dict)==1:
            for i in self.boundingbox_dict.keys():
                nearest_bbox=self.boundingbox_dict[i]
                nearest_bbox_id=i
            return nearest_bbox,nearest_bbox_id
        for i in self.boundingbox_dict.keys():
            center_x=(self.boundingbox_dict[i][0]+self.boundingbox_dict[i][3])/2
            center_y=(self.boundingbox_dict[i][1]+self.boundingbox_dict[i][4])/2
            last_bbox_center_x=(last_bbox[0]+last_bbox[3])/2
            last_bbox_center_y=(last_bbox[1]+last_bbox[4])/2
            distance=(center_x-last_bbox_center_x)**2+(center_y-last_bbox_center_y)**2
            if distance<min_distance and distance!=0:
                min_distance=distance
                nearest_bbox=self.boundingbox_dict[i]
                nearest_bbox_id=i
        return nearest_bbox,nearest_bbox_id

    # def bbox_timer_cb(self,event):
    #     with open(self.yaml_file_path, 'r') as f:
    #         content = yaml.load(f, Loader=yaml.FullLoader)
    #         for i in range(1,self.exp_num+1):
    #             bbox=[]
    #             nearest_bbox_list=[]
    #             temp_last_bbox=self.last_bbox_dict[i]
    #             self.temp_bbox_dict=copy.deepcopy(self.boundingbox_dict)
    #             for j in range(self.each_bbox_num):
    #                 if j<len(self.boundingbox_dict):
    #                     nearest_bbox, nearest_bbox_id=self.compare_bbox(temp_last_bbox)
    #                     temp_last_bbox=nearest_bbox
    #                     del self.temp_bbox_dict[nearest_bbox_id]
    #                     nearest_bbox_list.append(nearest_bbox_id)
    #                     string=""
    #                     string+=str(nearest_bbox[0])+","+str(nearest_bbox[1])+","+str(nearest_bbox[2])+","+str(nearest_bbox[3])+","+str(nearest_bbox[4])+","+str(nearest_bbox[5])
    #                     bbox.append(string)
    #             self.bbox_clients[i].wait_for_service()
    #             req=bboxRequest()
    #             req.bbox=bbox
    #             resp=self.bbox_clients[i].call(req)
    #             if resp.accepted:
    #                 rospy.logwarn("----------------------client topic: "+str(self.bbox_clients[i].resolved_name) + "-----------")
    #                 for k in range(len(nearest_bbox_list)):
    #                     rospy.logwarn("delete bbox id: "+str(nearest_bbox_list[k]))
    #                     self.last_bbox_dict[i]=self.boundingbox_dict[nearest_bbox_list[k]]
    #                     content[i-1]["exp_"+str(i)][0]["bbox"].append(self.boundingbox_dict[nearest_bbox_list[k]])
    #                     content[i-1]["exp_"+str(i)][0]["bbox_id"].append(nearest_bbox_list[k])
    #                     del self.boundingbox_dict[nearest_bbox_list[k]]
    #                 with open(self.yaml_file_path, 'w') as f:
    #                     yaml.dump(content, f)
        