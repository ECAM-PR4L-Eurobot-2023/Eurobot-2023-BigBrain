#!/usr/bin/env python

from builders.map_builder import MapBuilder


MAP_CONFIG_FILE = './configs/map.json'

class BigBrain:
    def __init__(self):
        self._map = MapBuilder.from_file(MAP_CONFIG_FILE)

    def run():
        pass


if __name__ == '__main__':
    bigbrain = BigBrain()
