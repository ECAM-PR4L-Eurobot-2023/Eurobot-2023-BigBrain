import math

from models.coordinate import Coordinate
from models.displacement import Displacement


STEP_PROJECTION = 10
LEFT_PLATES = {'plate-1', 'plate-2', 'plate-3', 'plate-4', 'plate-5'}
RIGHT_PLATES = {'plate-6', 'plate-7', 'plate-8', 'plate-9', 'plate-10'}


class RobotDisplacement:
    @classmethod
    def get_norm(cls, a, b):
        return math.sqrt((a ** 2) + (b ** 2))

    @classmethod
    def is_on_left(cls, game_map, position):
        return (game_map['width'] / 2) < position.x

    @classmethod
    def is_on_write(cls, game_map, position):
        return not cls.is_on_left(game_map, position)

    def is_on_top(cls, game_map, position):
        return (game_map['length'] / 2) < position.y

    def is_on_bottom(cls, game_map, position):
        return not cls.is_on_top(game_map, position)

    @classmethod
    def get_nearest_cherries(self, game_map, current_position, cherries_filter={}):
        dist_cherries = {key: RobotDisplacement.get_norm(
            current_position.x - cherries['x_pos'], current_position.y - cherries['y_pos']) 
            for key, cherries in game_map.cherries.items()}

        # Filter the cherries to return
        if cherries_filter:
            dist_cherries =  {key: value for key, value in dist_cherries.items() \
                if key in cherries_filter}

        nearest_cherries = min(dist_cherries, key=dist_cherries.get)

        return game_map.cherries[nearest_cherries]

    @classmethod
    def get_nearest_plate(self, game_map, current_position, plates_filter={}, color=None):
        plates = game_map.plates.items()

        # Filter by color
        if color is not None:
            plates = {key: value for key, value in  game_map.plates.items() if value['color'] == color}

        dist_plates = {key: RobotDisplacement.get_norm(
            current_position.x - value['x_pos'], current_position.y - value['y_pos']) 
            for key, value in plates.items()}
        
        print(dist_plates)

        # Filter plates
        if plates_filter:
            dist_plates = {key: value for key, value in dist_plates.items() \
                if key in plates_filter}
        print(dist_plates)
        nearest_plates = min(dist_plates, key=dist_plates.get)

        return game_map.plates[nearest_plates]

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
        if current_coordinate == dest_coordinate:
            return 0.0

        delta_x_dest = dest_coordinate.x - current_coordinate.x
        delta_y_dest = dest_coordinate.y - current_coordinate.y
        delta_x_proj = STEP_PROJECTION * math.sin(math.radians(current_coordinate.angle))
        delta_y_proj = STEP_PROJECTION * math.cos(math.radians(current_coordinate.angle))
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
            absolute_angle = current_coordinate.angle - angle
        else:
            absolute_angle = current_coordinate.angle + angle

        return round(absolute_angle % 360.0, 2)

    @classmethod
    def get_angle_to_map_item(cls,current_coordinate, map_item):
        dest_coordinate = Coordinate(x=map_item['x_pos'], y=map_item['y_pos'], angle=0.0)

        return cls.get_angle_to_coordinate(current_coordinate, dest_coordinate)
