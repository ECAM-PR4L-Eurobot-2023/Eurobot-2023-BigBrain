from models.action import Action
from modules.robot_displacement import RobotDisplacement, LEFT_PLATES, RIGHT_PLATES


NORMAL_SPEED = 255
REDUCED_SPEED = 65
BACKWARD_SPEED = 120

class SequencerCherryState:
    WAIT = 0
    GO_TO_CHERRIES = 1
    PICK_UP = 2
    GET_IN = 3


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
                print('cherry left or right')
                self._reduce_speed()
                self._ros_api.general_purpose.turn_on_fan(1)
                self._state = SequencerCherryState.PICK_UP
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
            self._ros_api.general_purpose.turn_on_fan(1)
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
        else:
            self._state = SequencerCherryState.WAIT

    def _reduce_speed(self):
        self._ros_api.flash_mcqueen.set_max_speed(REDUCED_SPEED)

    def _set_normal_speed(self):
        self._ros_api.flash_mcqueen.set_max_speed(NORMAL_SPEED)
    
    def _set_backward_speed(self):
        self._ros_api.flash_mcqueen.set_max_speed(BACKWARD_SPEED)