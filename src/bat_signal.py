#! /usr/bin/env python3

import json
import threading
import time

import rospy

from driver.lidar_360 import Lidar360


DEFAULT_LIDAR_DATA_RATE = 10  # Hz


class BatSignal:
    def __init__(self, lidar, lidar_data_rate=DEFAULT_LIDAR_DATA_RATE):
        self._lidar = lidar # Lidar360(GS2_CONFIG_FILE)
        self._lidar_data_period = 1 / lidar_data_rate
        self._mem_time = time.time()

        # Start scan
        self._lidar.start_scan()

    def run(self):
        try:
            while not rospy.is_shutdown():
                if (time.time() - self._mem_time) > self._lidar_data_period:
                    self._mem_time = time.time()
                    self._lidar.compute_distances()

                self._lidar.run()
        except KeyboardInterrupt:
            exit()

if __name__ == '__main__':
    try:
        bat_signal = BatSignal()
        bat_signal.run()
    except rospy.ROSInterruptException:
        pass

