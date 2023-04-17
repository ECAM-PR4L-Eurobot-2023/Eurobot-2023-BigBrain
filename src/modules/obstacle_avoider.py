import math

from models.action import Action
from models.coordinate import Coordinate
from modules.robot_displacement import RobotDisplacement

BACKWARD_PROJECTION = 10.0  # mm
DISTANCE_PROJECTION = 320.0  # mm


class ObstacleAvoiderState:
    BACKWARD = 0
    FIRST = 1
    SECOND = 2
    THIRD = 3
    GO_TO_DESTINATION = 4
    FINISH = 5

class ObstacleAvoider:
    def __init__(self, ros_api, current_position, destination):
        self._ros_api = ros_api
        self._current_position = current_position
        self.destination = destination
        self._state = ObstacleAvoiderState.BACKWARD

    @property
    def state(self):
        return self._state

    def reset(self):
        self._state = ObstacleAvoiderState.FIRST

    def run(self):
        if self._state == ObstacleAvoiderState.BACKWARD:
            print('--- OBS BACKWARD ---')
            self._state = ObstacleAvoiderState.FIRST
            dest_coordinate = self._get_projection(BACKWARD_PROJECTION, 0.0, True)

            return Action(
                key='backward',
                start_coord=self._current_position,
                end_coord=dest_coordinate,
                displacement=RobotDisplacement.get_displacement_to_coordinate(
                    'backward', self._current_position, dest_coordinate, True
                )
            )
        elif self._state == ObstacleAvoiderState.FIRST:
            print('--- OBS FIRST ---')
            self._state = ObstacleAvoiderState.SECOND
            dest_coordinate = self._get_projection(BACKWARD_PROJECTION, 90.0)

            return Action(
                key='first',
                start_coord=self._current_position,
                end_coord=dest_coordinate,
                displacement=RobotDisplacement.get_displacement_to_coordinate(
                    'first', self._current_position, dest_coordinate
                )
            )
        elif self._state == ObstacleAvoiderState.SECOND:
            print('--- OBS SECOND ---')
            self._state = ObstacleAvoiderState.THIRD
            dest_coordinate = self._get_projection(BACKWARD_PROJECTION, -90.0)

            return Action(
                key='second',
                start_coord=self._current_position,
                end_coord=dest_coordinate,
                displacement=RobotDisplacement.get_displacement_to_coordinate(
                    'second', self._current_position, dest_coordinate
                )
            )
        elif self._state == ObstacleAvoiderState.THIRD:
            print('--- OBS THIRD ---')
            self._state = ObstacleAvoiderState.GO_TO_DESTINATION
            dest_coordinate = self._get_projection(BACKWARD_PROJECTION, -90.0)

            return Action(
                key='third',
                start_coord=self._current_position,
                end_coord=dest_coordinate,
                displacement=RobotDisplacement.get_displacement_to_coordinate(
                    'third', self._current_position, dest_coordinate
                )
            )
        elif self._state == ObstacleAvoiderState.GO_TO_DESTINATION:
            print('--- OBS GO TO DEST ---')
            self._state = ObstacleAvoiderState.FINISH
            return Action(
                key='destination',
                start_coord=self._current_position,
                end_coord=self.destination,
                displacement=RobotDisplacement.get_displacement_to_coordinate(
                    'third', self._current_position, self.destination
                )
            )
        elif self._state == ObstacleAvoiderState.FINISH:
            print('--- OBS FINISH ---')
            return None
        else:
            self._state = ObstacleAvoiderState.BACKWARD
    
    def _get_projection(self, projection, offset_angle, backward=False):
        angle = math.radians(self._current_position.angle + offset_angle)
        x = projection * math.sin(angle)
        y = projection * math.cos(angle)

        if backward:
            return Coordinate(
                x=self._current_position.x - x,
                y=self._current_position.x - y,
                angle=angle,
            )
        else:
            return Coordinate(
                x=self._current_position.x + x,
                y=self._current_position.x + y,
                angle=angle,
            )
        