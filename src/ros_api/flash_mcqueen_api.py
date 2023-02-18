#!/usr/bin/env python

import rospy

import ros_tools as tools

# TODO:Create messages for displacement, position and PID messages


class FlashMcQueenApi:
    def __init__(self):
        # Callbacks for the subscribers
        self.get_data_all_callback = tools.default_callback
        self.urgency_stop_callback = tools.default_callback
        self.distance_reached_callback = tools.default_callback

        self._define_publishers()
        self._define_subscribers()

    def _define_publishers(self):
        self._set_displacement_pub = tools.create_publisher('set-displacement')
        self._set_position_pub = tools.create_publisher('set-position')
        self._set_rotation_pub = tools.create_publisher('set-rotation')
        self._set_rotation_pub = tools.create_publisher('set-distance-ticks')
        self._set_distance_ticks_pub = tools.create_publisher('set-distance-mm')
        self._set_distance_mm_pub = tools.create_publisher('set-distance-reached')
        self._set_pid_left_wheel_pub = tools.create_publisher('set-pid-left-wheel')
        self._set_pid_left_right_pub = tools.create_publisher('set-pid-right-wheel')
        self._set_pid_position_pub = tools.create_publisher('set-pid-position')
        self._set_pid_angle_pub = tools.create_publisher('set-pid-angle')

    def _define_subscribers(self):
        self._get_data_all_sub = tools.create_subscriber('get-data-all', self.get_data_all_callback)
        self._urgency_stop_sub = tools.create_subscriber('urgency-stop', self.urgency_stop_callback)
        self._distance_reached_sub = tools.create_subscriber('set-distance-reached', 
                                                            self.distance_reached_callback)
