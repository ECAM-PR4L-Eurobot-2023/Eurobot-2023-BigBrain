import time

from models.action import Action
from models.coordinate import Coordinate
from models.displacement import Displacement
from modules.robot_displacement import RobotDisplacement, LEFT_PLATES, RIGHT_PLATES


NORMAL_SPEED = 255
PARK_SPEED = 200
TREMBLING_TIME = 5  # Seconds


class DropoutCherrySequencerState:
    WAIT = 0
    GO_TO_PLATE = 1
    GO_TO_BASKET = 2
    OPEN_TANK = 3
    TREMBLING = 4
    FINISH = 5

class DropoutCherrySequencer:
    def __init__(self, ros_api, map, current_position, start_plate):
        self._ros_api = ros_api
        self._map = map
        self.current_position = current_position
        self._start_plate = start_plate
        self._state = DropoutCherrySequencerState.WAIT

        # Chrono for the trembling action
        self._trembling_chrono = time.time()

    @property
    def state(self):
        return self._state

    def reset(self):
        self._state = DropoutCherrySequencerState.WAIT

    def run(self):
        if self._state == DropoutCherrySequencerState.WAIT:
            print("DROPOUT WAIT")
            self._state = DropoutCherrySequencerState.GO_TO_PLATE

            return Action(
                key=self._start_plate,
                start_coord=self.current_position,
                end_coord=None,
                displacement=RobotDisplacement.front_basket_plate(
                    self._map, 
                    self.current_position, 
                    self._start_plate
                ))
        elif self._state == DropoutCherrySequencerState.GO_TO_PLATE:
            print("DROPOUT GO_TO_PLATE")
            self._state = DropoutCherrySequencerState.GO_TO_BASKET
            self._set_park_speed()
            self._ros_api.general_purpose.open_cherry_door()
            
            return Action(
                key=self._start_plate,
                start_coord=self.current_position,
                end_coord=None,
                displacement=self._go_to_wall_park()
            )
        elif self._state == DropoutCherrySequencerState.GO_TO_BASKET:
            print("DROPOUT GO_TO_BASKET")
            self._state = DropoutCherrySequencerState.OPEN_TANK
            # Make trembling the basket
            # pass
        elif self._state == DropoutCherrySequencerState.OPEN_TANK:
            print("DROPOUT OPEN_TANK")
            self._state = DropoutCherrySequencerState.TREMBLING
            self._trembling_chrono = time.time()
            
            return None
        elif self._state == DropoutCherrySequencerState.TREMBLING:
            if (time.time() - self._trembling_chrono) > TREMBLING_TIME:
                self._state = DropoutCherrySequencerState.FINISH
                basket_plate = RobotDisplacement.get_basket_plate(self._map, self._start_plate)

                return Action(
                    key='center',
                    start_coord=self.current_position,
                    end_coord=None,
                    displacement=RobotDisplacement.get_displacement_to_map_item(
                        '',
                        self.current_position,
                        self._map.plates[basket_plate],
                        backward=True
                    )
                )
        elif self._state == DropoutCherrySequencerState.FINISH:
            print("DROPOUT FINISH")
            return None
        else:
            self._state = DropoutCherrySequencerState.WAIT

    def _go_to_wall_park(self):
        dest_coordinate = Coordinate(
            x=self.current_position.x,
            y=self._map.length,
            angle=0.0,
        )

        return RobotDisplacement.get_displacement_to_coordinate(
            'park',
            self.current_position, 
            dest_coordinate,
        )

    def _set_normal_speed(self):
        self._ros_api.flash_mcqueen.set_max_speed(NORMAL_SPEED)
    
    def _set_park_speed(self):
        self._ros_api.flash_mcqueen.set_max_speed(PARK_SPEED)