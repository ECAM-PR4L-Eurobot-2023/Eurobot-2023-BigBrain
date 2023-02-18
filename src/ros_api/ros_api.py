#!/usr/bin/env python
import rospy

from flash_mcqueen_api import FlashMcQueenApi

NODE_NAME = "bigbrain"

class RosApi:
    """ API for ROS communication """
    def __init__(self):
        self.flash_mcqueen = FlashMcQueenApi()

        # Init ROS node
        rospy.init_node(NODE_NAME)

if __name__ == '__main__':
    import time

    from flash_mcqueen_api import Position, PidParameters

    while 1:
        position = Position(1, 2, 3.0)
        pid_param = PidParameters(5, 6, 7)

        ros_api = RosApi()
        ros_api.flash_mcqueen.set_position(position)
        ros_api.flash_mcqueen.set_rotation(90)
        ros_api.flash_mcqueen.set_pid_angle(pid_param)

        time.sleep(2)
        
