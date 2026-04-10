#! /usr/bin/env python

import rospy, time
from feature_server import FeatureServer
from los_server import LOSServer
from voxel_map_server import VoxelMapServer

if __name__ == "__main__":
    rospy.init_node("env_server", anonymous=True)
    voxel_map_server = VoxelMapServer()
    while not voxel_map_server.have_map:
        time.sleep(0.5)
    los_server = LOSServer(voxel_map_server)
    feature_server = FeatureServer(voxel_map_server)
    rospy.spin()