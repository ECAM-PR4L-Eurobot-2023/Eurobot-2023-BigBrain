#!/usr/bin/env python

"""
Topics related to all the nodes
"""
import std_msgs.msg as msg

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
TOPIC_DISTANCE_REACHED = "{}/data/distance/reach".format(FLASH_MCQUEEN_DATA_TOPIC_HEADER)
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


#######################################################################
#                              Topics
#######################################################################
TOPICS = {
    'set-displacement': {'topic': TOPIC_SET_DISPLACEMENT, 'data-type': msg.String},
    'set-position': {'topic': TOPIC_SET_POSITION, 'data-type': msg.String},
    'set-rotation': {'topic': TOPIC_SET_ROTATION, 'data-type': msg.String},
    'set-distance-ticks': {'topic': TOPIC_SET_DISTANCE_TICKS, 'data-type': msg.String},
    'set-distance-mm': {'topic': TOPIC_SET_DISTANCE_MM, 'data-type': msg.String},
    'set-distance-reached': {'topic': TOPIC_DISTANCE_REACHED, 'data-type': msg.String},
    'urgency-stop': {'topic': TOPIC_URGENCY_STOP, 'data-type': msg.String},
    'get-data-all': {'topic': TOPIC_DATA_ALL, 'data-type': msg.String},
    'set-pid-left-wheel': {'topic': TOPIC_SET_PID_LEFT_WHEEL, 'data-type': msg.String},
    'set-pid-right-wheel': {'topic': TOPIC_SET_PID_RIGHT_WHEEL, 'data-type': msg.String},
    'set-pid-position': {'topic': TOPIC_SET_PID_POSITION, 'data-type': msg.String},
    'set-pid-angle': {'topic': TOPIC_SET_PID_ANGLE, 'data-type': msg.String},
    'get-pid-left-wheel': {'topic': TOPIC_GET_PID_LEFT_WHEEL, 'data-type': msg.String},
    'get-pid-right-wheel': {'topic': TOPIC_GET_PID_RIGHT_WHEEL, 'data-type': msg.String},
    'get-pid-position': {'topic': TOPIC_GET_PID_POSITION, 'data-type': msg.String},
    'get-pid-angle': {'topic': TOPIC_GET_PID_ANGLE, 'data-type': msg.String},
}