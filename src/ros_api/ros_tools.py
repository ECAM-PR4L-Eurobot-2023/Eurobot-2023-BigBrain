#!/usr/bin/env python

import rospy

from topics import DEFAULT_QUEUE_SIZE, TOPICS

def default_callback(data):
   rospy.loginfo("{} - Received: {}".format(rospy.get_caller_id(), data))

def create_publisher(topic_name, queue_size=DEFAULT_QUEUE_SIZE):
    return rospy.Publisher(TOPICS[topic_name]['topic'], TOPICS[topic_name]['data-type'], 
                            queue_size=queue_size)

def create_subscriber(topic_name, callback=default_callback):
    return rospy.Subscriber(TOPICS[topic_name]['topic'], TOPICS[topic_name]['data-type'], 
                            callback)