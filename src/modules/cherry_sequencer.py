from modules.robot_displacement import RobotDisplacement, LEFT_PLATES, RIGHT_PLATES


class SequencerState:
    WAIT = 0
    GO_TO_CHERRIES = 1
    PICK_UP = 2
    GET_OUT = 3


class CherrySequencer:
    def __init__(self, ros_api, current_position, cherry):
        self._ros_api = ros_api
        self._state = SequencerState.WAIT
        self.cherry = cherry
        self.current_position

    @property
    def state(self):
        return self._state

    def reset(self):
        self._state = SequencerState.WAIT

    def run(self):
        if self._state == SequencerState.WAIT:
            # Calcul displacement to nearest cherry
            self._state = SequencerState.GO_TO_CHERRIES
            
        elif self._state == SequencerState.GO_TO_CHERRIES:
            self._ros_api.general_purpose.turn_on_fan(1)
            # Reduce speed
            # Turn on fan
            # Calcul position to get cherry
            # self._state = SequencerState.PICK_UP
            pass
        elif self._state == SequencerState.PICK_UP:
            # Turn off fan
            # Put normal speed
            # self._state = SequencerState.WAIT
            pass
        elif self._state == SequencerState.GET_IN:
            # Turn on fan
            pass
        else:
            self._state = SequencerState.WAIT