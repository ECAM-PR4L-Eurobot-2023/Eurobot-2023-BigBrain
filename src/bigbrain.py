#!/usr/bin/env python3

import threading
import time

import rospy

from builders.map_builder import MapBuilder
from models.coordinate import Coordinate
from models.displacement import Displacement
from models.lidar_bottom import LidarBottom
from models.limit_switches import LimitSwitches
from modules.bat_signal import BatSignal
from modules.emergency_stop import EmergencyStopDetector
from modules.robot_displacement import RobotDisplacement
from modules.strategy import Strategy
from ros_api.ros_api import RosApi
from driver.lidar_360 import Lidar360


MAP_CONFIG_FILE = './configs/map.json'
GS2_CONFIG_FILE = "./configs/gs2_config.json"
START_PLATE = 'plate-7'

class BigBrain:
    def __init__(self):
        self._map = MapBuilder.from_file(MAP_CONFIG_FILE)
        self._current_position = self._load_current_position_from_plate(START_PLATE)
        self._lidar_bottom = LidarBottom()
        self._lidar = Lidar360(GS2_CONFIG_FILE)
        self._bat_signal = BatSignal(self._lidar)
        self._ros_api = RosApi()
        self._strategy = Strategy(self._ros_api, self._map, self._current_position, START_PLATE)
        self._limit_switches = LimitSwitches(0)

        # Set callbacks
        self._ros_api.flash_mcqueen.urgency_stop_callback = self._on_urgency_stop
        self._ros_api.flash_mcqueen.get_data_all_callback = self._on_get_data_all
        self._ros_api.flash_mcqueen.distance_reached_callback = self._on_distance_reached
        self._ros_api.lidar._on_lidar_data = self._on_lidar_data

        self._ros_api.start_node()

        # Start LiDAR thread
        self._lidar_thread = threading.Thread(target=self._bat_signal.run, daemon=True)
        self._lidar_thread.start()
        self._test = True

    @property
    def map(self):
        return self._map

    def start(self):
        self._ros_api.lidar.start_scan()
        self._strategy.start()
        time.sleep(2.0)
        print('yep')
        self._ros_api.flash_mcqueen.set_displacement(Displacement(
            angle_start=0.0,
            angle_end=0.0,
            x=0.0,
            y=2000.0,
            backward=False,
        ))

    def run(self):
        # while True:
        #     if self.dest_reach:
        #         self.dest_reach = False

        #         if self.dir == 'up':
        #             self.dir = 'right'
        #             self._ros_api.flash_mcqueen.set_displacement(
        #                 Displacement(
        #                     angle_start=0.0,
        #                     angle_end=90.0,
        #                     x=self._current_position.x,
        #                     y=2000,
        #                     backward=False,
        #                 ))
        #         elif self.dir == "right":
        #             self.dir = 'down'
        #             self._ros_api.flash_mcqueen.set_displacement(
        #                 Displacement(
        #                     angle_start=90.0,
        #                     angle_end=90.0,
        #                     x=self._current_position.x,
        #                     y=self._current_position.y,
        #                     backward=False,
        #                 ))
        #         elif self.dir == "down":
        #             # self.dir = 'left'
        #             self._ros_api.flash_mcqueen.set_displacement(
        #                 Displacement(
        #                     angle_start=180.0,
        #                     angle_end=180.0,
        #                     x=self._current_position.x,
        #                     y=0.0,
        #                     backward=False,
        #                 ))
                # elif self.dir == "left":
                #     self.dir = 'left'
                #     self._ros_api.flash_mcqueen.set_displacement(
                #         RobotDisplacement.get_displacement_to_coordinate(
                #             '',
                #             self._current_position,
                #             Coordinate(x=0, y=0, angle=0.0,),
                #             backward=True,
                #         ))
        while not rospy.is_shutdown():
            if EmergencyStopDetector.detect_emergency_stop(self._lidar):
                print('--- Obstacle ---')
                print(self._lidar.distances)
                # time.sleep(10.0)
                self._ros_api.flash_mcqueen.set_stop()
            
            # self._strategy.run()

    def _load_current_position_from_plate(self, start_plate):
        start_plate_obj = self._map.plates[start_plate]
        plate_coordinate = Coordinate(
            x=int(start_plate_obj['x_pos']), 
            y=int(start_plate_obj['y_pos']), 
            angle=0.0)
        center_offset = RobotDisplacement.get_offset_center(plate_coordinate.angle)
        return Coordinate(
            x=int(start_plate_obj['x_pos']) + center_offset.x,
            y=int(start_plate_obj['y_pos']) + center_offset.y, 
            angle=0.0)

    def _on_get_data_all(self, data):
        # print(f'--- Data all ---')
        # print(data.x, data.y, data.angle)
        self._current_position.x = data.x
        self._current_position.y = data.y
        self._current_position.angle = data.angle
        self._strategy.current_position = self._current_position

    def _on_distance_reached(self, *arg, **kwargs):
        print('--- Distance reached ---')
        self.dest_reach = True
        self._strategy.set_destination_reached()

    def _on_urgency_stop(self, data):
        print('urgency-stop: ', data)
        self._limit_switches.value = data.data
        self._strategy.manage_limit_switches(self._limit_switches)
        # self._ros_api.flash_mcqueen.set_stop()

    def _on_lidar_data(self, data):
        self._lidar_bottom.set_distances(data.data, data.precision)


if __name__ == '__main__':
    bigbrain = BigBrain()
    bigbrain.start()


    bigbrain.run()
