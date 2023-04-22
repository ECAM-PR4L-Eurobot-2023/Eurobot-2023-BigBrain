import math
import numpy as np
import time

from builders.gs2_builder import GS2Builder
from driver.g2s import G2S


DEFAULT_ANGLE_PRECISION = 0.5  # Precision of the angle in degree
MINIMUM_ANGLE_PRECISION = 0.1
MAXIMUM_ANGLE_PRECISION = 2.0
SWITCH_TIMEOUT = 1.0 # s


class Lidar360:
    def __init__(self, config_file, angle_precision=DEFAULT_ANGLE_PRECISION):
        self._gs2_list = GS2Builder.from_file(config_file)
        self._angle_precision = angle_precision
        self._distances = np.full(round(360.0 / self._angle_precision), [float('inf')])
        self._compute_distances_vectorized = np.vectorize(self._compute_distances_vector)
        self._chrono = time.time()
        self._lidar_index = 0

        # Setup the LiDAR
        self._setup()

    @property
    def angle_precision(self):
        return self._angle_precision

    @angle_precision.setter
    def angle_precision(self, precision):
        if MINIMUM_ANGLE_PRECISION <= precision <= MAXIMUM_ANGLE_PRECISION:
            self._angle_precision = precision

            # Change the size of the distances array
            self._distances = np.full(round(360.0 / self._angle_precision), [float('inf')])

    @property
    def distances(self):
        return np.copy(self._distances)

    def run(self):
        # if len(self._gs2_list) > 1:
        #     old_index = self._lidar_index

        #     if (time.time() - self._chrono) > SWITCH_TIMEOUT:
        #         self._chrono = time.time()
        #         self._lidar_index += 1

        #         if self._lidar_index >= len(self._gs2_list):
        #             self._lidar_index = 0
            
        #     self._gs2_list[old_index].stop_scan()
        #     time.sleep(0.020)
        #     self._gs2_list[self._lidar_index].start_scan()
            
        [gs2.run() for gs2 in self._gs2_list]

    def start_scan(self):
        # if len(self._gs2_list) > 1:
        #     self._lidar_index = 0
        #     self._gs2_list[self._lidar_index].start_scan()
        # else:
        for gs2 in self._gs2_list:
            gs2.start_scan()

    def stop_scan(self):
        for gs2 in self._gs2_list:
            gs2.stop_scan()

    def compute_distances(self):
        # Reset value
        self._distances.fill(float('inf'))

        # Compute distances for each LiDAR
        for gs2 in self._gs2_list:
            for i, scan_data in enumerate([gs2.scan_data[0], gs2.scan_data[1]]):
                self._compute_scan_data(i, scan_data)
                # self._compute_scan_data(scan_data)
            # if gs2.scan_data[1] and all(np.isinf(gs2.scan_data[1].angles)):
            #     print(f'gs2.scan_data[1], angle: ')
        # print(self._distances)
    def _compute_scan_data(self, i, scan_data):
        # If not data available, return
        if scan_data is None:
            return
        
        self._compute_distances_vectorized(i, scan_data.distances, scan_data.angles)

    def _compute_distances_vector(self, index, distance, angle):
        # If the angle has an infinite value, return
        if math.isinf(angle):
            return

        if index == 0:
            angle *= -1

        if angle < 0:
            angle += 2 * math.pi

        angle_degree = angle * 180.0 / math.pi
        angle_modulo = angle_degree % self._angle_precision

        # Put the angle to the nearest limit
        rounded_angle = angle_degree - angle_modulo

        if angle_modulo >= (self._angle_precision / 2):
            rounded_angle += self._angle_precision

        # With the rounded angle, you can get the index in the array
        index = int(round(rounded_angle / self._angle_precision) % len(self._distances))

        # You can put the value in the destination buffer
        self._distances[index] = min(self._distances[index], distance)

    def _setup(self):
        for gs2 in self._gs2_list:
            # self._gs2.set_baudrate(0)
            # time.sleep(0.3)
            print('---- SETUP ---')
            gs2.system_reset()
            time.sleep(0.5)
            gs2.get_device_address()
            time.sleep(0.5)

            # Stop scan to get its information
            gs2.stop_scan()
            time.sleep(0.5)

            # Get device parameters
            gs2.get_device_parameters()
            time.sleep(0.5)

