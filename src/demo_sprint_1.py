#! /usr/bin/env python
import rospy


from ros_api.ros_api import RosApi

if __name__ == '__main__':
    ros_api = RosApi()

    ros_api.start_node()

    ros_api.flash_mcqueen.set_displacement(0.0, 45.0, 10029)
    ros_api.run()
