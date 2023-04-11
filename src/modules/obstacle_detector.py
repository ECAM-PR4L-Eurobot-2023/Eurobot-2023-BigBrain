import math


DISTANCE_TOLERANCE = 20.0  # mm
INFINITY_TOLERANCE = 5


class ObjectCoordinate:
    def __init__(self, x, y, distance, angle):
        self.x = x
        self.y = y
        self.distance = distance
        self.angle = angle


class Obstacle:
    def __init__(self, x, y, radius):
        self._x = x
        self._y = y
        self._radius = radius

    @property
    def x(self):
        return self._x

    @property
    def y(self):
        return self._y

    @property
    def radius(self):
        return self._radius

    @classmethod
    def from_coordinates(cls, current_coordinate, start_coord, end_coord):
        x1 = current_coordinate.x + start_coord.distance * math.sin(start_coord.angle)
        y1 = current_coordinate.y + start_coord.distance * math.cos(start_coord.angle)
        x2 = current_coordinate.x + end_coord.distance * math.sin(end_coord.angle)
        y2 = current_coordinate.y + end_coord.distance * math.cos(end_coord.angle)
        length = math.sqrt((x1 - x2)**2 + (y1 - y2)**2)

        return Obstacle(
            x=(x1 + x2) / 2,
            y=(y1 + y2) / 2,
            radius=length / 2,
        )

    def __str__(self):
        string = "--- Obstacle ---\n"
        string += f"x: {self._x}, y: {self._y}, radius: {self._radius}"

        return string


class ObstacleDetectors:
    def __init__(self):
        pass

    @classmethod
    def detect_obstacles(cls, current_coordinate, distances, angle_precision):
        if not distances:
            return []
        
        obstacles = []
        start_coord = ObjectCoordinate(
            x=distances[0] * math.sin(angle_precision),
            y=distances[0] * math.cos(angle_precision),
            distance=distances[0],
            angle=angle_precision,
        )
        end_coord = ObjectCoordinate(
            x=distances[0] * math.sin(angle_precision),
            y=distances[0] * math.cos(angle_precision),
            distance=distances[0],
            angle=angle_precision,
        )
        infinity_counter = 0

        for i in range(1, len(distances)):
            if math.isinf(distances[i]):
                infinity_counter += 1

                if infinity_counter == INFINITY_TOLERANCE:
                    obstacles.append(Obstacle.from_coordinates(current_coordinate, 
                        start_coord, end_coord))
            else:
                x = distances[i] * math.sin(angle_precision * i)
                y = distances[i] * math.cos(angle_precision * i)

                if infinity_counter >= INFINITY_TOLERANCE:
                    start_coord.x = x
                    start_coord.y = y
                    start_coord.distance = distances[i]
                    start_coord.angle = angle_precision * i
                else:
                    end_coord.x = x
                    end_coord.y = y
                    end_coord.distance = distances[i]
                    end_coord.angle = angle_precision * i

                infinity_counter = 0

        return obstacles
            



