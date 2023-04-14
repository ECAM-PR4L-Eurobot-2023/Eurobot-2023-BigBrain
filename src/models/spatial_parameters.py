import math


""" 
Module that stores the spatial parameters of the LiDAR
such as the angle, x and y position from the center of the robot
"""
class SpatialParameters:
    def __init__(self, angle_offset, x_offset, y_offset):
        self.angle_offset = angle_offset * math.pi / 180.0
        self.x_offset = x_offset
        self.y_offset = y_offset
        self.norm = math.sqrt(self.x_offset**2 + self.y_offset**2)
        self.angle = math.atan(self.x_offset / self.y_offset)

        if self.angle < 0:
            self.angle += (math.pi if self.y_offset < 0 else 2.0 * math.pi)
        self.angle %= 2.0 * math.pi

        print(f"x: {self.x_offset}, y: {self.y_offset}, norm: {self.norm}, angle: {self.angle * 180 / math.pi}, angle_offset: {angle_offset}")
