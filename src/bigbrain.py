#!/usr/bin/env python

from builders.map_builder import MapBuilder
from modules.robot_displacement import RobotDisplacement


MAP_CONFIG_FILE = './configs/map.json'

class BigBrain:
    def __init__(self):
        self._map = MapBuilder.from_file(MAP_CONFIG_FILE)

    @property
    def map(self):
        return self._map

    def run():
        pass


if __name__ == '__main__':
    from models.coordinate import Coordinate
    bigbrain = BigBrain()

    current_coordinate = Coordinate(x=225.0, y=225.0, angle=0.0)
    dest_coordinate = Coordinate(x=60.0, y=-45.0, angle=0.0)
    map_item = bigbrain.map.plates['plate-1']

    print(RobotDisplacement.get_displacement_to_map_item(current_coordinate, map_item))
