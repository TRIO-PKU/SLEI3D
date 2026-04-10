#! /usr/bin/env python

import rospy
from gcs_bbox_allocator import BboxAllocator
from gcs_ins_allocator import InsAllocator
from gcs_mission_executer import GcsMissionExecuter
from gcs_meeting_manager import GCSMeetingManager


if __name__ == "__main__":
    rospy.init_node("gcs_node", anonymous=True)
    drone_id = rospy.get_param("~drone_id","-1")
    bbox_filename = rospy.get_param("/bbox_description_path")
    each_bbox_num=rospy.get_param("~each_bbox_num")
    # 将参数传递给BboxAllocator
    bbox_allocator = BboxAllocator(drone_id, bbox_filename, each_bbox_num)
    ins_allocator = InsAllocator()
    gcs_mission_executer = GcsMissionExecuter()
    gcs_meeting_manager = GCSMeetingManager(bbox_allocator,ins_allocator,gcs_mission_executer, bbox_filename)
    gcs_mission_executer.set_meeting_manager(gcs_meeting_manager)
    rospy.spin()