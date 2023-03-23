#!/usr/bin/env python

from builders.map_builder import MapBuilder
from modules.robot_displacement import RobotDisplacement
from modules.strategy import Strategy
from ros_api.ros_api import RosApi

MAP_CONFIG_FILE = './configs/map.json'


class BigBrain:
    def __init__(self):
        self._map = MapBuilder.from_file(MAP_CONFIG_FILE)
        self._current_position = Coordinate(x=225.0, y=225.0, angle=0.0)
        self._ros_api = RosApi()
        self._strategy = Strategy(self._ros_api, self._map, self._current_position, 'plate-4')

        # Set callbacks
        self._ros_api.flash_mcqueen.get_data_all_callback = self._on_get_data_all
        self._ros_api.flash_mcqueen.distance_reached_callback = self._on_distance_reached

        self._ros_api.start_node()

    @property
    def map(self):
        return self._map

    def start(self):
        self._strategy.start()

    def run(self):
        while True:
            self._strategy.run()

    def _on_get_data_all(self, data):
        print('--- Data all ---')
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

    current_coordinate = Coordinate(x=225.0, y=225.0, angle=0.0)
    dest_coordinate = Coordinate(x=60.0, y=-45.0, angle=0.0)
    map_item = bigbrain.map.plates['plate-1']
    bigbrain.start()
    print(RobotDisplacement.get_displacement_to_map_item(current_coordinate, map_item))

    bigbrain.run()
