#!/usr/bin/env python3

import math
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
START_PLATE = 'plate-2'


class BigBrain:
    def __init__(self):
        self._map = MapBuilder.from_file(MAP_CONFIG_FILE)
        self._start_position = self._load_current_position_from_plate(START_PLATE)
        self._current_position = Coordinate(x=0.0, y=0.0, angle=0.0)
        self._lidar_bottom = LidarBottom()
        self._lidar = Lidar360(GS2_CONFIG_FILE)
        self._bat_signal = BatSignal(self._lidar)
        self._ros_api = RosApi()
        self._strategy = Strategy(self._ros_api, self._map, self._current_position, self._start_position, START_PLATE, self._lidar)
        self._limit_switches = LimitSwitches(0)

        # Set callbacks
        self._ros_api.flash_mcqueen.urgency_stop_callback = self._on_urgency_stop
        self._ros_api.flash_mcqueen.get_data_all_callback = self._on_get_data_all
        self._ros_api.flash_mcqueen.distance_reached_callback = self._on_distance_reached
        self._ros_api.general_purpose.set_start_plate_callback = self._on_start_plate
        self._ros_api.general_purpose.start_callback = self._on_start
        self._ros_api.kobe.get_cherry_callback = self._get_cherry
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
        self._ros_api.kobe.request_cherry()
        # self._strategy.start()

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
            self._strategy.run()

    def _load_current_position_from_plate(self, start_plate):
        start_plate_obj = self._map.plates[start_plate]

        if start_plate == 'plate-2':
            plate_coordinate = Coordinate(
                x=40 + 160.0, 
                y=start_plate_obj['y_pos'] - 110.0, 
                angle=start_plate_obj['start_angle'],
            )
        elif start_plate == 'plate-9':
            plate_coordinate = Coordinate(
                x=self._map.width - 40 -  160.0, 
                y=start_plate_obj['y_pos'] - 110.0, 
                angle=start_plate_obj['start_angle'],
            )
        else:
            plate_coordinate = Coordinate(
                x=start_plate_obj['x_pos'], 
                y=start_plate_obj['y_pos'], 
                angle=start_plate_obj['start_angle'])

        center_offset = RobotDisplacement.get_offset_center(plate_coordinate.angle)
        return Coordinate(
            x=plate_coordinate.x + center_offset.x,
            y=plate_coordinate.y + center_offset.y, 
            angle=plate_coordinate.angle)

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

    def _on_start(self, data):
        print('--- START ---')
        self._strategy.start()
    
    def _on_start_plate(self, data):
        print(f'--- START PLATE: {data.data}')
        plate = data.data + 1

        if not 1 <= plate <= 10:
            return

        self._strategy.start_plate = f'plate-{plate}'
        self._start_position = self._load_current_position_from_plate(self._strategy.start_plate)
        self._ros_api.flash_mcqueen.set_position(self._start_position)
        self._ros_api.flash_mcqueen.set_rotation(math.radians(self._start_position.angle))

    def _get_cherry(self, data):
        print(f'--- Cherry received: {data.data}')
        self._strategy.score_simulator.score_cherry = data.data
        print(self._strategy.score_simulator.score_cherry)


if __name__ == '__main__':
    bigbrain = BigBrain()
    bigbrain.start()


    bigbrain.run()
