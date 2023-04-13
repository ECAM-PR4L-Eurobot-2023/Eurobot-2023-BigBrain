import math
import numpy as np
import time

from builders.gs2_builder import GS2Builder
from driver.g2s import G2S


DEFAULT_ANGLE_PRECISION = 0.5  # Precision of the angle in degree
MINIMUM_ANGLE_PRECISION = 0.1
MAXIMUM_ANGLE_PRECISION = 2.0

class Lidar360:
    def __init__(self, config_file, angle_precision=DEFAULT_ANGLE_PRECISION):
        self._gs2_list = GS2Builder.from_file(config_file)
        self._angle_precision = angle_precision
        self._distances = np.full(round(360.0 / self._angle_precision), [float('inf')])
        self._compute_distances_vectorized = np.vectorize(self._compute_distances_vector)

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
        [gs2.run() for gs2 in self._gs2_list]

    def start_scan(self):
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
            for scan_data in gs2.scan_data:
                self._compute_scan_data(scan_data)

    def _compute_scan_data(self, scan_data):
        # If not data available, return
        if scan_data is None:
            return
        # print('dist: ', scan_data.distances)
        # print('angle: ', scan_data.angles)
        self._compute_distances_vectorized(scan_data.distances, scan_data.angles)

    def _compute_distances_vector(self, distance, angle):
        # If the angle has an infinite value, return
        if math.isinf(angle):
            return

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

