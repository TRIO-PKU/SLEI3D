#! /usr/bin/env python

import rospy, time
from animation import ResultVisualizer

if __name__ == "__main__":
    rospy.init_node("animation_node", anonymous=True)
    rospy.set_param('/save_fig_path', "/home/cjf/ExpInsSim/src/visualizer_node/figures/")
    feature_server = ResultVisualizer()
    rospy.spin()