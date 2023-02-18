#!/usr/bin/env python
import rospy

from flash_mcqueen_api import FlashMcQueenApi

NODE_NAME = "bigbrain"

class RosApi:
    def __init__(self):
        self.flash_mcqueen = FlashMcQueenApi()

        # Init ROS node
        rospy.init_node(NODE_NAME)

if __name__ == '__main__':
    while 1:
        ros_api = RosApi()
