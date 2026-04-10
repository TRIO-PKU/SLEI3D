#! /usr/bin/env python

import rospy
from ins_mission_executer import InsMissionExecuter
from ins_meeting_manager import InsMeetingManager

if __name__ == "__main__":

    rospy.init_node("ins_node", anonymous=True)
    ins_mission_executer = InsMissionExecuter()
    ins_meeting_manager = InsMeetingManager(ins_mission_executer)
    ins_mission_executer.set_meeting_manager(ins_meeting_manager)

    rospy.spin()
