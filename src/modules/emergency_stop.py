import math

import numpy as np

from modules.robot_displacement import X_SIZE_ROBOT, Y_SIZE_ROBOT


DISTANCE_EMERGENCY = 150.0  # mm
CENTER_TO_EDGE = 115
TOTAL_DISTANCE_EMERGENCY = CENTER_TO_EDGE + DISTANCE_EMERGENCY
TOTAL_DISTANCE_EMERGENCY_TEST = math.sqrt((X_SIZE_ROBOT / 2)**2 + (Y_SIZE_ROBOT / 2)**2)
X_MAX_DIST = 150 #  X_SIZE_ROBOT / 2  # + 10.0
FORWARD_ANGLE = 40
MAX_COUNTER = 5
MAX_WINDOW_VALID = 3
WINDOW = 20


class EmergencyStopDetector:
    def __init__(self):
        pass

    @classmethod
    def detect_emergency_stop(cls, lidar):
        if np.size(lidar.distances) == 0:
            return False

        valid_window = 0
        dist_map = lidar.distances
        angle_step = lidar.angle_precision
        index_range = round(FORWARD_ANGLE // angle_step)
        dist_map = np.concatenate((dist_map[-index_range:], dist_map[:index_range]))

        for index in range(len(dist_map) - WINDOW):
            counter = 0

            for i, dist in enumerate(dist_map[index:index + WINDOW]):
                angle = (index + i) * angle_step - FORWARD_ANGLE
                angle_radians = angle * math.pi / 180.0

                if dist < TOTAL_DISTANCE_EMERGENCY / math.cos(angle_radians) and \
                    -X_MAX_DIST < dist * math.sin(angle_radians) < X_MAX_DIST:
                    # print(x_dist)
                    counter += 1

            if counter >= MAX_COUNTER:
                valid_window += 1

            if valid_window >= MAX_WINDOW_VALID:
                return True

        return False

            # print(len(dist_map))
            # print(f'index: {index}')
            # print((index) * angle_step - FORWARD_ANGLE)
            # print(dist_map[index:index + WINDOW])

            

            # for i, dist in enumerate(dist_map[index:index + WINDOW]):
            #     # if dist < (TOTAL_DISTANCE_EMERGENCY / math.cos(((index + i) * angle_step - FORWARD_ANGLE * math.pi / 180.0) - FORWARD_ANGLE)):
            #     angle = (index + i) * angle_step - FORWARD_ANGLE

            #     if dist < (TOTAL_DISTANCE_EMERGENCY / math.cos(angle * math.pi / 180.0)):
            #         counter += 1

            #     if counter >= MAX_COUNTER:
            #         return True    
            # higher_values = [
            #     1 for i, dist in enumerate(dist_map[index:index + WINDOW])
            #     if dist < (TOTAL_DISTANCE_EMERGENCY * math.cos(((index + i) * angle_step - FORWARD_ANGLE * math.pi / 180.0) - FORWARD_ANGLE))
            # ]
            # if sum(higher_values) >= MAX_COUNTER:
            #     return True
                # counter += 1

                # if counter >= MAX_COUNTER:
            # else:
            #     counter = 0
        return False
