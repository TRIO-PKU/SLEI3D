#! /usr/bin/env python

import rospy, time
from visualizer import Visualizer
from animation import ResultVisualizer

if __name__ == "__main__":
    rospy.init_node("visualizer_node", anonymous=True)
    visualizer_server = Visualizer()
    rospy.spin()