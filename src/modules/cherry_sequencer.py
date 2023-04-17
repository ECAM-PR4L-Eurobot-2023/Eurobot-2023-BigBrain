import math

from models.action import Action
from models.coordinate import Coordinate
from modules.robot_displacement import RobotDisplacement, LEFT_PLATES, RIGHT_PLATES


BACKWARD_PROJECTION_DISTANCE = 200.0  # mm
BACKWARD_PROJECTION_ANGLE = math.radians(45)  # degree
NORMAL_SPEED = 255
REDUCED_SPEED = 80
BACKWARD_SPEED = 120
LEFT_FAN = 2
RIGHT_FAN = 1


class SequencerCherryState:
    WAIT = 0
    GO_TO_CHERRIES = 1
    PICK_UP = 2
    GET_IN = 3
    BACKWARD = 4


class CherrySequencer:
    def __init__(self, ros_api, map, current_position, cherry):
        self._ros_api = ros_api
        self._map = map
        self._state = SequencerCherryState.GO_TO_CHERRIES
        self.cherry = cherry
        self.current_position = current_position

    @property
    def state(self):
        return self._state

    def reset(self):
        self._state = SequencerCherryState.GO_TO_CHERRIES

    def run(self):
        if self._state == SequencerCherryState.WAIT:
            print("CHERRY WAIT")
            self._state = SequencerCherryState.GO_TO_CHERRIES
            dest_coordinate, displacement = RobotDisplacement.get_displacement_start_collect_cherries(
                    self.current_position,
                    self.cherry,
                    self._map,
            )

            return Action(
                key=self.cherry,
                start_coord=self.current_position,
                end_coord=dest_coordinate,
                displacement=displacement,
            )
        elif self._state == SequencerCherryState.GO_TO_CHERRIES:
            print("CHERRY GO_TO_CHERRIES")

            if self.cherry in {'left', 'right'}:
                self._reduce_speed()
                self._ros_api.general_purpose.turn_on_fan(self._get_fan_to_turn_on())
                self._state = SequencerCherryState.BACKWARD
                dest_coordinate, displacement = RobotDisplacement.forward_cherry_pickup(
                        self._map, self.current_position, self.cherry
                    )

                return Action(
                    key=self.cherry,
                    start_coord=self.current_position,
                    end_coord=dest_coordinate,
                    displacement=displacement,
                )
            else:
                self._state = SequencerCherryState.GET_IN
                self._set_backward_speed()
                dest_coordinate, displacement = RobotDisplacement.backtrace_cherry_pickup(
                        self._map, self.current_position, self.cherry
                    )

                return Action(
                    key=self.cherry,
                    start_coord=self.current_position,
                    end_coord=dest_coordinate,
                    displacement=displacement
                )
        elif self._state == SequencerCherryState.PICK_UP:
            print("CHERRY PICK_UP")
            self._state = SequencerCherryState.WAIT
            self._ros_api.general_purpose.turn_off_fan()
            self._set_normal_speed()

            return None
            # return Action(
            #         key=self.cherry,
            #         start_coord=self.current_position,
            #         displacement=RobotDisplacement.get_displacement_to_coordinate(
            #             self.cherry, self.current_position, self.current_position, 
            #         )
            #     )
        elif self._state == SequencerCherryState.GET_IN:
            print("CHERRY GET_IN")
            self._ros_api.general_purpose.turn_on_fan(self._get_fan_to_turn_on())
            self._state = SequencerCherryState.PICK_UP
            self._reduce_speed()
            dest_coordinate, displacement = RobotDisplacement.getout_cherry(
                    self._map, self.current_position, self.cherry,
                )

            return Action(
                key=self.cherry,
                start_coord=self.current_position,
                end_coord=dest_coordinate,
                displacement=displacement,
            )
        elif self._state == SequencerCherryState.BACKWARD:
            print("CHERRY BACKWARD")
            self._state = SequencerCherryState.PICK_UP
            self._ros_api.general_purpose.turn_off_fan()
            self._set_backward_speed()
            dest_coordinate = self._get_coordinate_backward_projection()
            print(f'dest back: {dest_coordinate}')

            return Action(
                key=self.cherry,
                start_coord=self.current_position,
                end_coord=dest_coordinate,
                displacement=RobotDisplacement.get_displacement_to_coordinate(
                    self.cherry, self.current_position, dest_coordinate, True
                )
            )
        else:
            self._state = SequencerCherryState.WAIT

    def _reduce_speed(self):
        self._ros_api.flash_mcqueen.set_max_speed(REDUCED_SPEED)

    def _set_normal_speed(self):
        self._ros_api.flash_mcqueen.set_max_speed(NORMAL_SPEED)
    
    def _set_backward_speed(self):
        self._ros_api.flash_mcqueen.set_max_speed(BACKWARD_SPEED)

    def _get_coordinate_backward_projection(self):
        if self.cherry == 'left':
            return Coordinate(
                x=self.current_position.x + BACKWARD_PROJECTION_DISTANCE * math.sin(BACKWARD_PROJECTION_ANGLE),
                y=self.current_position.y + BACKWARD_PROJECTION_DISTANCE * math.cos(BACKWARD_PROJECTION_ANGLE),
                angle=0.0,
            )
        else:
            return Coordinate(
                x=self.current_position.x + BACKWARD_PROJECTION_DISTANCE * math.sin(-BACKWARD_PROJECTION_ANGLE),
                y=self.current_position.y + BACKWARD_PROJECTION_DISTANCE * math.cos(-BACKWARD_PROJECTION_ANGLE),
                angle=0.0,
            )

    def _get_fan_to_turn_on(self):
        cherry = self._map.cherries[self.cherry]
        cherry_coordinate = Coordinate(x=cherry['x_pos'], y=cherry['y_pos'], angle=0.0)

        return LEFT_FAN if RobotDisplacement.is_object_at_left(self.current_position, 
            cherry_coordinate) else RIGHT_FAN

