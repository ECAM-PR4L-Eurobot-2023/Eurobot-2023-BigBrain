from models.action import Action
from models.coordinate import Coordinate
from models.displacement import Displacement
from modules.robot_displacement import RobotDisplacement


class DummySequencerState:
    WAIT = 0
    STRAIGHT_ON = 1
    TURN_90 = 2


class DummySequencer:
    def __init__(self, ros_api, map, current_position):
        self._ros_api = ros_api
        self._map = map
        self._state = DummySequencerState.WAIT
        self.current_position = current_position

    @property
    def state(self):
        return self._state

    def reset(self):
        self._state = DummySequencerState.WAIT
    
    def run(self):
        if self._state == DummySequencerState.WAIT:
            print("CHERRY WAIT")
            self._state = DummySequencerState.STRAIGHT_ON
            dest_coordinate = Coordinate(x=0.0, y=2000.0, angle=0.0)

            return Action(
                key='dummy',
                start_coord=self.current_position,
                end_coord=dest_coordinate,
                displacement=RobotDisplacement.get_displacement_to_coordinate(
                    'dummy',
                    self.current_position,
                    dest_coordinate,
                    )
            )
        elif self._state == DummySequencerState.STRAIGHT_ON:
            self._state = DummySequencerState.WAIT
            dest_coordinate = Coordinate(x=self.current_position.x, y=0, angle=0.0)

            return Action(
                key='dummy',
                start_coord=self.current_position,
                end_coord=dest_coordinate,
                displacement=RobotDisplacement.get_displacement_to_coordinate(
                    'dummy',
                    self.current_position,
                    dest_coordinate,
                    )
            )
        elif self._state == DummySequencerState.TURN_90:
            return None
        else:
            self._state = DummySequencerState.WAIT