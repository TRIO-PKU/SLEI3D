#! /usr/bin/env python

import rospy
from exp_mission_executer import ExpMissionExecuter
from exp_meeting_manager import ExpMeetingManager


if __name__ == "__main__":
    
    rospy.init_node("exp_node", anonymous=True)
    exp_mission_executer = ExpMissionExecuter()
    exp_meeting_manager = ExpMeetingManager(exp_mission_executer)
    exp_mission_executer.set_meeting_manager(exp_meeting_manager)
    
    rospy.spin()