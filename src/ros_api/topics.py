"""
Topics related to all the nodes
"""
import std_msgs.msg as std_msgs

import eurobot2023.msg as msg

DEFAULT_QUEUE_SIZE = 10

#######################################################################
#                           Flash McQueen
#######################################################################
# Flash McQueen headers
FLASH_MCQUEEN_TOPIC_HEADER = "/robot"
FLASH_MCQUEEN_DATA_TOPIC_HEADER = "{}/data".format(FLASH_MCQUEEN_TOPIC_HEADER)
FLASH_MCQUEEN_PID_TOPIC_HEADER = "{}/pid".format(FLASH_MCQUEEN_TOPIC_HEADER)

# Flash McQueen topics
TOPIC_SET_DISPLACEMENT = "{}/displacement/set".format(FLASH_MCQUEEN_DATA_TOPIC_HEADER)
TOPIC_SET_POSITION = "{}/position/set".format(FLASH_MCQUEEN_DATA_TOPIC_HEADER)
TOPIC_SET_ROTATION = "{}/rotation/set".format(FLASH_MCQUEEN_DATA_TOPIC_HEADER)
TOPIC_SET_DISTANCE_TICKS = "{}/distance/set/ticks".format(FLASH_MCQUEEN_DATA_TOPIC_HEADER)
TOPIC_SET_DISTANCE_MM = "{}/distance/set/mm".format(FLASH_MCQUEEN_DATA_TOPIC_HEADER)
TOPIC_DISTANCE_REACHED = "{}/distance/reach".format(FLASH_MCQUEEN_DATA_TOPIC_HEADER)
TOPIC_URGENCY_STOP = "{}/urgency-stop".format(FLASH_MCQUEEN_TOPIC_HEADER)
TOPIC_DATA_ALL = "{}/all".format(FLASH_MCQUEEN_DATA_TOPIC_HEADER)
TOPIC_SET_PID_LEFT_WHEEL = "{}/wheel/left/set".format(FLASH_MCQUEEN_PID_TOPIC_HEADER)
TOPIC_SET_PID_RIGHT_WHEEL = "{}/wheel/right/set".format(FLASH_MCQUEEN_PID_TOPIC_HEADER)
TOPIC_SET_PID_POSITION = "{}/position/set".format(FLASH_MCQUEEN_PID_TOPIC_HEADER)
TOPIC_SET_PID_ANGLE = "{}/angle/set".format(FLASH_MCQUEEN_PID_TOPIC_HEADER)
TOPIC_GET_PID_LEFT_WHEEL = "{}/wheel/left/get".format(FLASH_MCQUEEN_PID_TOPIC_HEADER)
TOPIC_GET_PID_RIGHT_WHEEL = "{}/wheel/right/get".format(FLASH_MCQUEEN_PID_TOPIC_HEADER)
TOPIC_GET_PID_POSITION = "{}/position/get".format(FLASH_MCQUEEN_PID_TOPIC_HEADER)
TOPIC_GET_PID_ANGLE = "{}/angle/get".format(FLASH_MCQUEEN_PID_TOPIC_HEADER)
TOPIC_SET_MAX_SPEED = "{}/max-speed/set".format(FLASH_MCQUEEN_TOPIC_HEADER)
TOPIC_SET_STOP = "{}/stop".format(FLASH_MCQUEEN_TOPIC_HEADER)
TOPIC_WIGGLE = "{}/wiggle".format(FLASH_MCQUEEN_TOPIC_HEADER)
TOPIC_WIGGLE_DONE = "{}/wiggle/done".format(FLASH_MCQUEEN_TOPIC_HEADER)

#######################################################################
#                          General Purpose
#######################################################################
# General Purpose header
GENERAL_PURPOSE_HEADER = '/robot'
GENERAL_PURPOSE_FAN_HEADER = '{}/fan'.format(GENERAL_PURPOSE_HEADER)
GENERAL_PURPOSE_CHERRY_DOOR_HEADER = '{}/door'.format(GENERAL_PURPOSE_HEADER)

# General Purpose topics
GENERAL_PURPOSE_SET_DISPLAY = '{}/display/set'.format(GENERAL_PURPOSE_HEADER)
GENERAL_PURPOSE_START_PLATE = '{}/startPlate/set'.format(GENERAL_PURPOSE_HEADER)
GENERAL_PURPOSE_FAN_ON = '{}/on'.format(GENERAL_PURPOSE_FAN_HEADER)
GENERAL_PURPOSE_FAN_OFF = '{}/off'.format(GENERAL_PURPOSE_FAN_HEADER)
GENERAL_PURPOSE_OPEN_CHERRY_DOOR = '{}/open'.format(GENERAL_PURPOSE_CHERRY_DOOR_HEADER)
GENERAL_PURPOSE_CLOSE_CHERRY_DOOR = '{}/close'.format(GENERAL_PURPOSE_CHERRY_DOOR_HEADER)
GENERAL_PURPOSE_DISGUISE= '{}/disguise'.format(GENERAL_PURPOSE_HEADER)
GENERAL_PURPOSE_START = '{}/start'.format(GENERAL_PURPOSE_HEADER)

#######################################################################
#                               LiDAR
#######################################################################
# Batsignal header
BATSIGNAL_HEADER = '/batsignal'
BATSIGNAL_SCAN_HEADER = '{}/scan'.format(BATSIGNAL_HEADER)

# Bat signal topics
BATSIGNAL_START_SCAN = '{}/start'.format(BATSIGNAL_SCAN_HEADER)
BATSIGNAL_STOP_SCAN = '{}/stop'.format(BATSIGNAL_SCAN_HEADER)
BATSIGNAL_PRECISION_SET = '{}/precision/set'.format(BATSIGNAL_HEADER)
BATSIGNAL_RATE_SET = '{}/rate/set'.format(BATSIGNAL_HEADER)
BATSIGNAL_LIDAR_DATA = '{}/lidar/data'.format(BATSIGNAL_HEADER)

#######################################################################
#                              Topics
#######################################################################
TOPICS = {
    'set-displacement': {'topic': TOPIC_SET_DISPLACEMENT, 'data-type': msg.Displacement},
    'set-position': {'topic': TOPIC_SET_POSITION, 'data-type': msg.Position},
    'set-rotation': {'topic': TOPIC_SET_ROTATION, 'data-type': std_msgs.Float32},
    'set-distance-ticks': {'topic': TOPIC_SET_DISTANCE_TICKS, 'data-type': std_msgs.Int64},
    'set-distance-mm': {'topic': TOPIC_SET_DISTANCE_MM, 'data-type': std_msgs.Float32},
    'distance-reached': {'topic': TOPIC_DISTANCE_REACHED, 'data-type': std_msgs.Empty},
    'urgency-stop': {'topic': TOPIC_URGENCY_STOP, 'data-type': std_msgs.Int16},
    'get-data-all': {'topic': TOPIC_DATA_ALL, 'data-type': msg.Coordinate},
    'set-pid-left-wheel': {'topic': TOPIC_SET_PID_LEFT_WHEEL, 'data-type': msg.PidParameters},
    'set-pid-right-wheel': {'topic': TOPIC_SET_PID_RIGHT_WHEEL, 'data-type': msg.PidParameters},
    'set-pid-position': {'topic': TOPIC_SET_PID_POSITION, 'data-type': msg.PidParameters},
    'set-pid-angle': {'topic': TOPIC_SET_PID_ANGLE, 'data-type': msg.PidParameters},
    'get-pid-left-wheel': {'topic': TOPIC_GET_PID_LEFT_WHEEL, 'data-type': msg.PidParameters},
    'get-pid-right-wheel': {'topic': TOPIC_GET_PID_RIGHT_WHEEL, 'data-type': msg.PidParameters},
    'get-pid-position': {'topic': TOPIC_GET_PID_POSITION, 'data-type': msg.PidParameters},
    'get-pid-angle': {'topic': TOPIC_GET_PID_ANGLE, 'data-type': msg.PidParameters},
    'wiggle': {'topic': TOPIC_WIGGLE, 'data-type': std_msgs.Empty},
    'wiggle-done': {'topic': TOPIC_WIGGLE_DONE, 'data-type': std_msgs.Empty},
    'set-display': {'topic': GENERAL_PURPOSE_SET_DISPLAY, 'data-type': std_msgs.Int16},
    'start-plate': {'topic': GENERAL_PURPOSE_START_PLATE, 'data-type': std_msgs.Int16},
    'fan-on': {'topic': GENERAL_PURPOSE_FAN_ON, 'data-type': std_msgs.Int16},
    'fan-off': {'topic': GENERAL_PURPOSE_FAN_OFF, 'data-type': std_msgs.Empty},
    'open-cherry-door': {'topic': GENERAL_PURPOSE_OPEN_CHERRY_DOOR, 'data-type': std_msgs.Empty},
    'close-cherry-door': {'topic': GENERAL_PURPOSE_CLOSE_CHERRY_DOOR, 'data-type': std_msgs.Empty},
    'robot-disguise': {'topic': GENERAL_PURPOSE_DISGUISE, 'data-type': std_msgs.Empty},
    'robot-start': {'topic': GENERAL_PURPOSE_START, 'data-type': std_msgs.Empty},
    'set-max-speed': {'topic': TOPIC_SET_MAX_SPEED, 'data-type': std_msgs.Float32},
    'set-stop': {'topic': TOPIC_SET_STOP, 'data-type': std_msgs.Empty},
    'start-scan': {'topic': BATSIGNAL_START_SCAN, 'data-type': std_msgs.Empty},
    'stop-scan': {'topic': BATSIGNAL_STOP_SCAN, 'data-type': std_msgs.Empty},
    'set-precision': {'topic': BATSIGNAL_PRECISION_SET, 'data-type': std_msgs.Float32},
    'set-rate': {'topic': BATSIGNAL_RATE_SET, 'data-type': std_msgs.Float32},
    'lidar-data': {'topic': BATSIGNAL_LIDAR_DATA, 'data-type': msg.LidarData},
}