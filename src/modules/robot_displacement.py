import math

from models.coordinate import Coordinate
from models.displacement import Displacement


STEP_PROJECTION = 10


class RobotDisplacement:
    @classmethod
    def get_norm(cls, a, b):
        return math.sqrt((a ** 2) + (b ** 2))

    @classmethod
    def get_displacement_to_coordinate(cls, current_coordinate, dest_coordinate):
        delta_x = current_coordinate.x - dest_coordinate.x
        delta_y = current_coordinate.y - dest_coordinate.y
        angle = cls.get_angle_to_coordinate(current_coordinate, dest_coordinate)

        return Displacement(
            distance=cls.get_norm(delta_x, delta_y),
            angle_start=angle,
            angle_end=angle,
        )

    @classmethod
    def get_displacement_to_map_item(cls, current_coordinate, map_item):
        dest_coordinate = Coordinate(x=map_item['x_pos'], y=map_item['y_pos'], angle=0.0)
        
        return cls.get_displacement_to_coordinate(current_coordinate, dest_coordinate)
    @classmethod
    def get_angle_to_coordinate(cls, current_coordinate, dest_coordinate):
        delta_x_dest = dest_coordinate.x - current_coordinate.x
        delta_y_dest = dest_coordinate.y - current_coordinate.y
        delta_x_proj = STEP_PROJECTION * math.sin(current_coordinate.angle)
        delta_y_proj = STEP_PROJECTION * math.cos(current_coordinate.angle)
        delta_dest_norm = cls.get_norm(delta_x_dest, delta_y_dest)
        delta_proj_norm = cls.get_norm(delta_x_proj, delta_y_proj)

        # Compute the angle to return
        cos_angle = ((delta_x_dest * delta_x_proj) + (delta_y_dest * delta_y_proj)) / \
            (delta_dest_norm * delta_proj_norm)
        angle = math.degrees(math.acos(cos_angle))

        # Check the sign of the angle, for that compute the cross product
        cross_product = delta_x_dest * delta_y_proj - delta_x_proj * delta_y_dest

        # If the cross product is negatif, then the point is at the left
        if cross_product < 0:
            angle = (angle - (2 * angle)) % 360.0

        return angle

    @classmethod
    def get_angle_to_map_item(cls,current_coordinate, map_item):
        dest_coordinate = Coordinate(x=map_item['x_pos'], y=map_item['y_pos'], angle=0.0)

        return cls.get_angle_to_coordinate(current_coordinate, dest_coordinate)
