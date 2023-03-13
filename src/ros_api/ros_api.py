#!/usr/bin/env python
import rospy

from flash_mcqueen_api import FlashMcQueenApi

NODE_NAME = "bigbrain"
DEFAULT_RATE = 10

class RosApi:
    """ API for ROS communication """
    def __init__(self, rate=DEFAULT_RATE):
        self.flash_mcqueen = FlashMcQueenApi()
        self.rate = rate

    def start_node(self):
        self.flash_mcqueen.start()

        # Init ROS node
        rospy.init_node(NODE_NAME)

        # Set loop rate
        rate = rospy.Rate(10)

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    import time

    from flash_mcqueen_api import Position, PidParameters

    ros_api = RosApi()
    ros_api.start_node()
    time.sleep(1)

    # while 1:
    ros_api.flash_mcqueen.set_displacement(45.0, 90.0, 5000)
    ros_api.run()
    #     time.sleep(30)
    #     ros_api.run()
        
