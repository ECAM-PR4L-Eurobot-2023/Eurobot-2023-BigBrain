#!/usr/bin/env python
import rospy

from ros_api.flash_mcqueen_api import FlashMcQueenApi

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

    while 1:
        position = Position(1, 2, 3.0)
        pid_param = PidParameters(5, 6, 7)

        ros_api.flash_mcqueen.set_displacement(2, 3, 4)
        ros_api.flash_mcqueen.set_position(position)
        ros_api.flash_mcqueen.set_rotation(90)
        ros_api.flash_mcqueen.set_pid_angle(pid_param)

        time.sleep(2)
        ros_api.run()
