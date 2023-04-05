#!/usr/bin/env python3

import time

from builders.map_builder import MapBuilder
from modules.robot_displacement import RobotDisplacement
from modules.strategy import Strategy
from ros_api.ros_api import RosApi

MAP_CONFIG_FILE = './configs/map.json'
START_PLATE = 'plate-4'
counter = 0


class BigBrain:
    def __init__(self):
        self._map = MapBuilder.from_file(MAP_CONFIG_FILE)
        self._current_position = self._load_current_position_from_plate(START_PLATE)
        self._ros_api = RosApi()
        self._strategy = Strategy(self._ros_api, self._map, self._current_position, START_PLATE)

        # Set callbacks
        self._ros_api.flash_mcqueen.get_data_all_callback = self._on_get_data_all
        self._ros_api.flash_mcqueen.distance_reached_callback = self._on_distance_reached

        self._ros_api.start_node()

        # self._ros_thread_handler = threading.Thread(target=self._ros_api_thread, daemon=True)

    def _ros_api_thread(self):
        while True:
            self._ros_api.run()

    @property
    def map(self):
        return self._map

    def start(self):
        self._strategy.start()

    def run(self):
        while True:
            self._strategy.run()

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
        global counter
        print(f'--- Data all {counter}---')
        counter += 1
        print(data.x, data.y, data.angle)
        self._current_position.x = data.x
        self._current_position.y = data.y
        self._current_position.angle = data.angle
        self._strategy.current_position = self._current_position

    def _on_distance_reached(self, *arg, **kwargs):
        print('--- Distance reached ---')
        self._strategy.set_destination_reached()


if __name__ == '__main__':
    from models.coordinate import Coordinate
    bigbrain = BigBrain()
    bigbrain.start()
    bigbrain.run()
