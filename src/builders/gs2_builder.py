#! /usr/bin/env python

""" Module that build the GS2 instances"""

import io
import json

from driver.g2s import G2S
from driver.g2s_defs import DEFAULT_BAUDRATE, MAXIMUM_LIDAR_CHAIN
from models.spatial_parameters import SpatialParameters


class GS2Builder:
    @classmethod
    def from_file(cls, file_path):
        gs2_list = []

        with io.open(file_path, 'r', encoding='utf8') as file:
            for json_data in json.loads(file.read()):
                # Get devices
                devices = json_data.get('devices', [])

                # Sort by address
                devices.sort(key=lambda device: device.get('address', 0))

                # Load the Spatial Parameters from file
                spatial_parameters = [SpatialParameters(
                    x_offset = device.get("x_offset", 0.0), 
                    y_offset = device.get("y_offset", 0.0), 
                    angle_offset = device.get("angle_offset", 0.0))
                    for device in devices]

                # Create the GS2
                gs2_list.append(G2S(
                    port=json_data.get('port', ''), 
                    baudrate=json_data.get('baudrate', DEFAULT_BAUDRATE),
                    spatial_parameters=spatial_parameters))

        return gs2_list[:]

    
if __name__ == '__main__':
    print(GS2Builder.from_file('configs/gs2_config.json'))