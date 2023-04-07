import math

from models.coordinate import Coordinate
from models.displacement import Displacement


STEP_PROJECTION = 10
LEFT_PLATES = {'plate-1', 'plate-2', 'plate-3', 'plate-4', 'plate-5'}
RIGHT_PLATES = {'plate-6', 'plate-7', 'plate-8', 'plate-9', 'plate-10'}
LEFT_MAP_CHERRIES = {'left', 'up'}
RIGHT_MAP_CHERRIES = {'right', 'up'}
X_SIZE_ROBOT = 320  # mm
Y_SIZE_ROBOT = 230  # mm
OFFSET_CENTER = 72  # mm 


class RobotDisplacement:
    @classmethod
    def get_norm(cls, a, b):
        return math.sqrt((a ** 2) + (b ** 2))

    @classmethod
    def get_offset_center(cls, angle):
        angle_radians = math.radians(angle)
        
        return Coordinate(
            x = -OFFSET_CENTER * math.sin(angle_radians),
            y = -OFFSET_CENTER * math.cos(angle_radians),
            angle = 0.0,
        )

    @classmethod
    def is_on_left(cls, game_map, position):
        return (game_map.width / 2) < position.x

    @classmethod
    def is_on_write(cls, game_map, position):
        return not cls.is_on_left(game_map, position)

    @classmethod
    def is_on_top(cls, game_map, position):
        return (game_map.length / 2) < position.y

    @classmethod
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
    def get_displacement_to_coordinate(cls, key, current_coordinate, dest_coordinate, backward=False):
        delta_x = current_coordinate.x - dest_coordinate.x
        delta_y = current_coordinate.y - dest_coordinate.y
        angle = cls.get_angle_to_coordinate(current_coordinate, dest_coordinate) % 360

        # Set offset center
        # delta_angle_radians = math.radians(angle - current_coordinate.angle)
        # delta_x, delta_y = OFFSET_CENTER * math.sin(delta_angle_radians), OFFSET_CENTER * (1 - math.cos(delta_angle_radians))
        # supposed_coordinate = Coordinate(
        #     x=current_coordinate.x + delta_y * math.cos(current_coordinate.angle) - delta_x * math.sin(current_coordinate.angle),
        #     y=current_coordinate.y - (delta_y * math.sin(current_coordinate.angle) + delta_x * math.cos(current_coordinate.angle)), 
        #     angle=current_coordinate.angle)

        # current_angle_radians = math.radians(current_coordinate.angle)
        # delta_angle_radians = math.radians(angle - current_coordinate.angle)
        # rotation_center_x, rotation_center_y = current_coordinate.x - OFFSET_CENTER * math.sin(current_angle_radians), current_coordinate.y - OFFSET_CENTER * math.cos(current_angle_radians)
        # point_from_rotation_center_x, point_from_rotation_center_y = current_coordinate.x - rotation_center_x, current_coordinate.y - rotation_center_y
        # rotated_point_x = point_from_rotation_center_x * math.cos(delta_angle_radians) - point_from_rotation_center_y * math.sin(delta_angle_radians)
        # rotated_point_y = point_from_rotation_center_x * math.sin(delta_angle_radians) + point_from_rotation_center_y * math.cos(delta_angle_radians)
        # supposed_coordinate = Coordinate(
        #     x=rotated_point_x + rotation_center_x,
        #     y=rotated_point_y + rotation_center_y,
        #     angle=current_coordinate.angle)

        offset_center = cls.get_offset_center(angle)

        print(f'angle: {angle}')
        print(f'(angle + 180) % 360.0: {(angle + 180) % 360.0}')

        disp = Displacement(
            x=dest_coordinate.x + offset_center.x if not backward else dest_coordinate.x,
            y=dest_coordinate.y + offset_center.y if not backward else dest_coordinate.y,
            angle_start=angle if not backward else (angle + 180) % 360.0,
            angle_end=angle if not backward else (angle + 180) % 360.0,
            backward=backward,
        )

        # print(key)
        # print(str(disp))
        # print('current coord: ', str(current_coordinate))
        # print('dest_coordinate: ', str(dest_coordinate))
        return disp

    @classmethod
    def get_displacement_to_map_item(cls, key, current_coordinate, map_item):
        dest_coordinate = Coordinate(x=map_item['x_pos'], y=map_item['y_pos'], angle=0.0)
        
        return cls.get_displacement_to_coordinate(key, current_coordinate, dest_coordinate)
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

    @classmethod
    def get_displacement_start_collect_cherries(cls, current_coordinate, cherry_key, map):
        cherries = map.cherries
        if cherry_key in {'left', 'right'}:
            # Compute y
            if cls.is_on_top(map, current_coordinate):
                y = cherries[cherry_key]['y_pos'] + (cherries[cherry_key]['y_size'] / 2) + \
                    10.0
                angle_end = 180.0
            else:
                y = cherries[cherry_key]['y_pos'] - (cherries[cherry_key]['y_size'] / 2) - \
                    10.0
                angle_end = 0.0
            
            # Compute x
            if cherry_key == 'left':
                x = cherries[cherry_key]['x_pos'] + (cherries[cherry_key]['x_size'] / 2) + \
                    (X_SIZE_ROBOT / 2) + 10.0
            else:
                x = cherries[cherry_key]['x_pos'] - (cherries[cherry_key]['x_size'] / 2) - \
                    (X_SIZE_ROBOT / 2) + 10.0
        elif cherry_key in {'up', 'down'}:
            # Compute y
            if cherry_key == 'up':
                y = cherries[cherry_key]['y_pos'] - (cherries[cherry_key]['y_size'] / 2) - \
                    - Y_SIZE_ROBOT - 10.0
                angle_end = 180.0
            else:
                y = cherries[cherry_key]['y_pos'] + (cherries[cherry_key]['y_size'] / 2) + \
                + Y_SIZE_ROBOT + 10.0
                angle_end = 0.0
                print('here')
            
            # Compute x
            if cls.is_on_left(map, current_coordinate):
                x = map.cherries[cherry_key]['x_pos'] + (map.cherries[cherry_key]['x_size'] / 2) + \
                    (X_SIZE_ROBOT / 2) + 10.0
            else:
                x = map.cherries[cherry_key]['x_pos'] - (map.cherries[cherry_key]['x_size'] / 2) - \
                    (X_SIZE_ROBOT / 2) + 10.0
        
        dest_coordinate = Coordinate(x=x, y=y, angle=0.0)
        displacement = cls.get_displacement_to_coordinate(cherry_key, current_coordinate, dest_coordinate)
        displacement.angle_end = angle_end

        return displacement

    @classmethod
    def backtrace_cherry_pickup(cls, map, current_coordinate, key):
        if key not in {'up', 'down'}:
            return None

        if key == 'up':
            dest_coordinate = Coordinate(
                x=current_coordinate.x,
                y=map.length,
                angle=0.0
            )
        else:
            dest_coordinate = Coordinate(
                x=current_coordinate.x,
                y=0,
                angle=0.0
            )

        return cls.get_displacement_to_coordinate(key, current_coordinate, dest_coordinate, True)

    @classmethod
    def forward_cherry_pickup(cls, map, current_coordinate, key):
        if key not in {'left', 'right'}:
            return None

        if cls.is_on_top(map, current_coordinate):
            print("YOURE ON TOP")
            dest_coordinate = Coordinate(
                x=current_coordinate.x,
                y=map.cherries[key]['y_pos'] - map.cherries[key]['y_size'] / 2,
                angle=0.0,
            )
        else:
            print("YOURE ON DOWN")
            dest_coordinate = Coordinate(
                x=current_coordinate.x,
                y=map.cherries[key]['y_pos'] + map.cherries[key]['y_size'] / 2,
                angle=0.0,
            )

        return cls.get_displacement_to_coordinate(key, current_coordinate, dest_coordinate)

    @classmethod
    def getout_cherry(cls, map, current_coordinate, key):
        if key not in {'up', 'down'}:
            return None

        cherries = map.cherries

        if key == 'up':
            dest_coordinate = Coordinate(
                x=current_coordinate.x,
                y=map.length - cherries[key]['y_size'] - 20,
                angle=0.0
            )
        else:
            dest_coordinate = Coordinate(
                x=current_coordinate.x,
                y=cherries[key]['y_size'] + 20,
                angle=0.0
            )

        return cls.get_displacement_to_coordinate(key, current_coordinate, dest_coordinate)
