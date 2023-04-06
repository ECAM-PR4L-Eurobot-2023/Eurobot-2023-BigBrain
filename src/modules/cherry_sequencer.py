from models.action import Action
from modules.robot_displacement import RobotDisplacement, LEFT_PLATES, RIGHT_PLATES


NORMAL_SPEED = 255
REDUCED_SPEED = 70


class SequencerState:
    WAIT = 0
    GO_TO_CHERRIES = 1
    PICK_UP = 2
    GET_IN = 3


class CherrySequencer:
    def __init__(self, ros_api, map, current_position, cherry):
        self._ros_api = ros_api
        self._map = map
        self._state = SequencerState.WAIT
        self.cherry = cherry
        self.current_position = current_position

    @property
    def state(self):
        return self._state

    def reset(self):
        self._state = SequencerState.WAIT

    def run(self):
        if self._state == SequencerState.WAIT:
            print('WAIT')
            # Calcul displacement to nearest cherry
            self._state = SequencerState.GO_TO_CHERRIES

            return Action(
                key=self.cherry,
                start_coord=self.current_position,
                displacement=RobotDisplacement.get_displacement_start_collect_cherries(
                    self.current_position,
                    self.cherry,
                    self._map,
            ))
        elif self._state == SequencerState.GO_TO_CHERRIES:
            print('GO_TO_CHERRIES')
            if self.cherry in {'left', 'right'}:
                print('Turn on fan')
                self._ros_api.general_purpose.turn_on_fan(1)
                self._reduce_speed()
                self._state = SequencerState.PICK_UP

                return Action(
                    key=self.cherry,
                    start_coord=self.current_position,
                    displacement=RobotDisplacement.backtrace_cherry_pickup(
                        self._map, self.current_position, self.cherry
                    )
                )
            else:
                self._state = SequencerState.GET_IN

                return Action(
                    key=self.cherry,
                    start_coord=self.current_position,
                    displacement=RobotDisplacement.backtrace_cherry_pickup(
                        self._map, self.current_position, self.cherry
                    )
                )
        elif self._state == SequencerState.PICK_UP:
            print('PICK_UP')
            self._state = SequencerState.WAIT
            print('turn off fan')
            self._ros_api.general_purpose.turn_off_fan()
            self._set_normal_speed()

            return Action(
                    key=self.cherry,
                    start_coord=self.current_position,
                    displacement=RobotDisplacement.backtrace_cherry_pickup(
                        self._map, self.current_position, self.cherry,
                    )
                )
        elif self._state == SequencerState.GET_IN:
            print('GET_IN')
            self._ros_api.general_purpose.turn_on_fan(1)
            self._reduce_speed()
            self._state = SequencerState.PICK_UP

            return Action(
                key=self.cherry,
                start_coord=self.current_position,
                displacement=RobotDisplacement.getout_cherry(
                    self._map, self.current_position, self.cherry,
                )
            )
            pass
        else:
            self._state = SequencerState.WAIT

    def _reduce_speed(self):
        self._ros_api.flash_mcqueen.set_max_speed(REDUCED_SPEED)

    def _set_normal_speed(self):
        self._ros_api.flash_mcqueen.set_max_speed(NORMAL_SPEED)
    