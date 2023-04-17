import math
import numpy as np

from driver.g2s_defs import (START_SCAN, SCAN_DATA_LENGTH, ANGLE_P_X, ANGLE_P_Y, ANGLE_P_ANGLE, 
                             MIN_DISTANCE, MAX_DISTANCE, CLOUD_POINT)
from models.command_data_models.command_data import CommandData
from wrapper import convert_distance_iterator, DeviceParametersCpp, SpatialParametersCpp

class ScanData(CommandData, object):
    def __init__(self, distances, angles):
        super(ScanData, self).__init__()
        self._distances = distances
        self._angles = angles
        
    @property
    def distances(self):
        return self._distances
    
    @property
    def angles(self):
        return self._angles

    @classmethod
    def from_command(cls, command, spatial_param, device_param):
        # If the command type is not [START_SCAN], return None
        if command.command_type != START_SCAN:
            return None
        
        # If data length is lower than < [SCAN_DATA_LENGTH], then return None
        if command.data_length != SCAN_DATA_LENGTH:
            return None
        
        # If no parameters has been passed, return None
        if device_param is None:
            return None

        # Create the Device and Spatial parameters for the C++ code
        device_param_cpp = DeviceParametersCpp(device_param.k0, device_param.b0, device_param.k1, 
            device_param.b1, device_param.bias)
        spatial_param_cpp = SpatialParametersCpp(spatial_param.angle_offset, 
            spatial_param.x_offset, spatial_param.y_offset, spatial_param.norm, spatial_param.angle)

        # Allocate space for the destination numpy array's
        distances, angles = np.arange(160, dtype=np.float32), np.arange(160, dtype=np.float32)
        convert_distance_iterator(distances, angles, command.data, spatial_param_cpp, device_param_cpp)
        
        # Filter data to max distances
        filter_indices_min, filter_indices_max = distances < MIN_DISTANCE + spatial_param.norm, distances > MAX_DISTANCE + spatial_param.norm
        # distances[filter_indices_min], distances[filter_indices_max] = float('inf'), float('inf')
        # angles[filter_indices_min], angles[filter_indices_max] = float('inf'), float('inf')
        distances[filter_indices_min] = float('inf')
        angles[filter_indices_min] = float('inf')

        return ScanData(distances=distances, angles=angles)
