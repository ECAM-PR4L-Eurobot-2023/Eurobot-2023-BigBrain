#!/usr/bin/env python

from enum import IntEnum

import eurobot2023.msg as msg
import ros_api.ros_tools as tools

# TODO:Create messages for displacement, position and PID messages

class DistanceUnitType(IntEnum):
    """ Available Unit for distance"""
    TICKS = 0
    MM = 1


class Position:
    """ Container for the position informations """
    def __init__(self, x, y, angle):
        self.x = x
        self.y = y
        self.angle = angle


class PidParameters:
    """ Parameters of a PID """
    def __init__(self, Kp, Ki, Kd=0.0):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

    def __str__(self):
        return "{} - {} - {}".format(self.Kp, self.Ki, self.Kd)


class FlashMcQueenApi:
    """ API to communicate with Flash McQueen using ROS topics """
    def __init__(self):
        # Callbacks for the subscribers
        self.get_data_all_callback = tools.default_callback
        self.urgency_stop_callback = tools.default_callback
        self.distance_reached_callback = tools.default_callback

    def start(self):
        self._define_publishers()
        self._define_subscribers()

    def set_displacement(self, displacement):
        self._set_displacement_pub.publish(
            msg.Displacement(
                displacement.angle_start, 
                displacement.angle_end, 
                displacement.distance
            ))
    
    def set_position(self, position):
        self._set_position_pub.publish(msg.Position(position.x, position.y))

    def set_rotation(self, angle):
        self._set_rotation_pub.publish(angle)

    def set_distance(self, distance, unit=DistanceUnitType.TICKS):
        if unit == UnitType.TICKS:
            self._set_distance_ticks_pub.publish(distance)
        else:
            self._set_distance_mm_pub.publish(distance)

    def set_pid_left_wheel(self, pid_parameters):
        self._set_pid_left_wheel_pub.publish(msg.PidParameters(pid_parameters.Kp,  
                                                                pid_parameters.Ki, 
                                                                pid_parameters.d))

    def set_pid_right_wheel(self, pid_parameters):
        self._set_pid_right_wheel_pub.publish(msg.PidParameters(pid_parameters.Kp,  
                                                                pid_parameters.Ki,  
                                                                pid_parameters.Kd))

    def set_pid_position(self, pid_parameters):
        self._set_pid_position_pub.publish(msg.PidParameters(pid_parameters.Kp,  
                                            pid_parameters.Ki,  
                                            pid_parameters.Kd))

    def set_pid_angle(self, pid_parameters):
        self._set_pid_angle_pub.publish(msg.PidParameters(pid_parameters.Kp,  
                                                            pid_parameters.Ki,  
                                                            pid_parameters.Kd))

    def _define_publishers(self):
        self._set_displacement_pub = tools.create_publisher('set-displacement')
        self._set_position_pub = tools.create_publisher('set-position')
        self._set_rotation_pub = tools.create_publisher('set-rotation')
        self._set_distance_ticks_pub = tools.create_publisher('set-distance-ticks')
        self._set_distance_mm_pub = tools.create_publisher('set-distance-mm')
        self._set_pid_left_wheel_pub = tools.create_publisher('set-pid-left-wheel')
        self._set_pid_left_right_pub = tools.create_publisher('set-pid-right-wheel')
        self._set_pid_position_pub = tools.create_publisher('set-pid-position')
        self._set_pid_angle_pub = tools.create_publisher('set-pid-angle')

    def _define_subscribers(self):
        self._get_data_all_sub = tools.create_subscriber('get-data-all', self.get_data_all_callback)
        self._urgency_stop_sub = tools.create_subscriber('urgency-stop', self.urgency_stop_callback)
        self._distance_reached_sub = tools.create_subscriber('distance-reached', 
                                                            self.distance_reached_callback)
