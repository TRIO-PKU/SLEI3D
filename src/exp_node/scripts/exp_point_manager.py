#! /usr/bin/env python

import rospy, copy
from geometry_msgs.msg import PoseStamped
from msgs.srv import inspector_allocation,inspector_allocationRequest,inspector_allocationResponse
from msgs.srv import ins_pose,ins_poseRequest,ins_poseResponse
from msgs.srv import check_los,check_losRequest,check_losResponse
from msgs.srv import pose, poseRequest, poseResponse, gcs_pose, gcs_poseRequest
from msgs.srv import chat, chatRequest, chatResponse
from feature_scan import FeatureScan

class ExpPointManager():
    def __init__(self):
        
        self.drone_id=rospy.get_param("~drone_id","-1")
        self.check_los_client = rospy.ServiceProxy("/check_los", check_los)
        self.ins_service = rospy.Service("inspector_allocation", inspector_allocation, self.ins_allocation_cb)
        self.point_list: list[PoseStamped]=[]
        self.last_pose_list=[]
        self.ins_pose_clients:dict[int, rospy.ServiceProxy] = {}
        self.ins_pose_topic="/ins_pose"
        rospy.Timer(rospy.Duration(1), self.timer_cb)
        
        # allocation
        self.new_ins_id_list: list[str] = []
        self.exp_allocation_client = None
        
        # receive and publish inspected inspection pose
        self.inspected_inspection_pose_buffer: list[PoseStamped] = []
        self.inspected_inspection_pose_server = rospy.Service("inspected_inspection_pose", pose, self.inspected_inspection_pose_cb)
        self.call_gcs_client = rospy.ServiceProxy("/call_gcs", gcs_pose)
        
        # feature scan module
        self.feature_scan = FeatureScan(self.point_list)
        
    def inspected_inspection_pose_cb(self,req: poseRequest):
        self.inspected_inspection_pose_buffer.append(req.pose)
        return poseResponse(True)
    
    def ins_allocation_cb(self,req):
        for ins_id in range(req.first_inspector_id,req.last_inspector_id+1):
            self.new_ins_id_list.append("ins_"+str(ins_id))
            ins_topic = "/ins_"+str(ins_id) + self.ins_pose_topic
            self.ins_pose_clients[ins_id] = rospy.ServiceProxy(ins_topic,ins_pose)
        rospy.logwarn("Allocation request received:"+str(req.first_inspector_id)+","+str(req.last_inspector_id))
        for ins_id in range(req.first_inspector_id,req.last_inspector_id+1):
            each_ins_last_pose=PoseStamped()
            each_ins_last_pose.pose.position.x=0
            each_ins_last_pose.pose.position.y=0
            each_ins_last_pose.pose.position.z=0
            self.last_pose_list.append([ins_id,each_ins_last_pose,0.0])
        return inspector_allocationResponse(True)
    
    def timer_cb(self, event) -> None:
        if len(self.new_ins_id_list) > 0:
            for ins_id in copy.deepcopy(self.new_ins_id_list):
                if not self.check_los(ins_id):
                    continue
                self.exp_allocation_client = rospy.ServiceProxy("/" + ins_id + "/allocate_exp_to_ins", chat)
                self.exp_allocation_client.wait_for_service()
                req = chatRequest()
                req.message = self.drone_id
                res: chatResponse = self.exp_allocation_client.call(req)
                if res.success:
                    self.new_ins_id_list.remove(ins_id)
                    
        if len(self.inspected_inspection_pose_buffer) > 0:
            can_communicate: bool = self.check_los("gcs")
            if not can_communicate:
                return
            self.call_gcs_client.wait_for_service()
            req=gcs_poseRequest()
            for i in range(len(self.inspected_inspection_pose_buffer)):
                req.pose=self.inspected_inspection_pose_buffer.pop(0)
                self.call_gcs_client.call(req)
                    
        if len(self.point_list)==0:
            return
        point=self.point_list[0]
        sorted_list=self.compare(point)
        for i in range(len(sorted_list)):
            ins_id=sorted_list[i][0]
            self.ins_pose_clients[ins_id].wait_for_service()
            req = ins_poseRequest()
            req.pose=point
            can_communicate: bool = self.check_los("ins_"+str(ins_id))
            if not can_communicate:
                return
            resp: ins_poseResponse=self.ins_pose_clients[ins_id].call(req)
            if resp.accepted:
                self.point_list.pop(0)
                self.last_pose_list[i][1]=point
                break
            
    def compare(self,point):
        # 计算self.last_pose_list中每个点到point的距离，依据距离排序
        for i in range(len(self.last_pose_list)):
            distance=((self.last_pose_list[i][1].pose.position.x-point.pose.position.x)**2+(self.last_pose_list[i][1].pose.position.y-point.pose.position.y)**2+(self.last_pose_list[i][1].pose.position.z-point.pose.position.z)**2)
            self.last_pose_list[i][2]=distance
            if i>0:
                j=i
                while j>0 and self.last_pose_list[j][2]<self.last_pose_list[j-1][2]:
                    self.last_pose_list[j],self.last_pose_list[j-1]=self.last_pose_list[j-1],self.last_pose_list[j]
                    j-=1
        self.sorted_list=self.last_pose_list
        return self.sorted_list
    
    def check_los(self, target_id: str) -> bool:
        req = check_losRequest()
        req.client_id = self.drone_id
        req.target_id = target_id
        self.check_los_client.wait_for_service()
        resp: check_losResponse = self.check_los_client.call(req)
        return resp.success

# if __name__ == "__main__":
    
#     rospy.init_node("exp_point_manager", anonymous=True)
#     exp_point_manager=ExpPointManager()
#     rospy.spin()