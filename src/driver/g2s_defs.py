""" Definition found in the datasheet of the GS2 """
import numpy as np


# PACKET_HEADER = bytearray(b'\xa5' * 4)
PACKET_HEADER = np.array([0xa5] * 4)
MAXIMUM_LIDAR_CHAIN = 3
DEFAULT_BAUDRATE = 921600

######################################################################
#                           Command Code
######################################################################
DEVICE_ADDRESS = 0x60
DEVICE_PARAMETERS = 0x61
VERSION_INFO = 0x62
START_SCAN = 0x63
STOP_SCAN = 0x64
RESTART = 0x67
SET_BAUDRATE = 0x68
SET_EDGE_MODE = 0x69

######################################################################
#                           Version info
######################################################################
VERSION_LENGTH = 3  # Bytes
SERIAL_NUMBER_LENGTH = 16  # Bytes

######################################################################
#                           Device Parameters
######################################################################
K_COMPENSATION = 10000.0
B_COMPENSATION = 10000.0
BIAS_COMPENSATION = 10.0

######################################################################
#                           Scan Data
######################################################################
SCAN_DATA_LENGTH = 322
CLOUD_POINT = 160
ANGLE_P_X = 1.22
ANGLE_P_Y = 5.315
ANGLE_P_ANGLE = 22.5
MIN_DISTANCE = 25  # mm
MAX_DISTANCE = 300  # mm