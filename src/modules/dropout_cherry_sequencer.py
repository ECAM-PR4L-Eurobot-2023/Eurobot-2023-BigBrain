import time

from models.action import Action
from models.coordinate import Coordinate
from models.displacement import Displacement
from modules.robot_displacement import RobotDisplacement, LEFT_PLATES, RIGHT_PLATES


NORMAL_SPEED = 200
PARK_SPEED = 150
TREMBLING_TIME = 5.0  # Seconds
MAX_BACK_COUNTER = 1


class DropoutCherrySequencerState:
    WAIT = 0
    GO_TO_PLATE = 1
    GO_TO_BASKET = 2
    OPEN_TANK = 3
    TREMBLING = 4
    FINISH = 5
    GO_TO_CENTER = 6
    BACKWARD = 7

class DropoutCherrySequencer:
    def __init__(self, ros_api, map, current_position, start_plate):
        self._ros_api = ros_api
        self._map = map
        self._state = DropoutCherrySequencerState.WAIT
        self._counter = 0
        self.current_position = current_position
        self.start_plate = start_plate
        self.run_next_state = False

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
            self._set_normal_speed()
            self._state = DropoutCherrySequencerState.GO_TO_CENTER
            dest, disp = RobotDisplacement.side_basket_plate(
                        self._map, 
                        self.current_position, 
                        self.start_plate
                    )

            if self.start_plate in {'plate-2', 'plate-9'}:
                return Action(
                    key=self.start_plate,
                    start_coord=self.current_position,
                    end_coord=dest,
                    displacement=disp)
            else:
                dest, disp = self._go_to_aruco_tag()
                return Action(
                    key=self.start_plate,
                    start_coord=self.current_position,
                    end_coord=dest,
                    displacement=disp,
                )
        elif self._state == DropoutCherrySequencerState.GO_TO_CENTER:
            self._state = DropoutCherrySequencerState.GO_TO_PLATE
            basket_plate = RobotDisplacement.get_basket_plate(self._map, self.start_plate)
            dest, disp = self._go_to_center()
            print("DROPOUT GO_TO_CENTER")

            return Action(
                    key='center',
                    start_coord=self.current_position,
                    end_coord=dest,
                    displacement=disp
                )
        elif self._state == DropoutCherrySequencerState.GO_TO_PLATE:
            print("DROPOUT GO_TO_PLATE")
            if self._counter <= MAX_BACK_COUNTER:
                self._state = DropoutCherrySequencerState.GO_TO_BASKET
                self._set_park_speed()
                dest, disp = self._go_to_wall_park()
                
                return Action(
                    key=self.start_plate,
                    start_coord=self.current_position,
                    end_coord=dest,
                    displacement=disp
                )
            else:
                self._state = DropoutCherrySequencerState.FINISH
                return None
        elif self._state == DropoutCherrySequencerState.GO_TO_BASKET:
            print("DROPOUT GO_TO_BASKET")
            self._ros_api.general_purpose.open_cherry_door()
            self._state = DropoutCherrySequencerState.OPEN_TANK
            return None
        elif self._state == DropoutCherrySequencerState.OPEN_TANK:
            print("DROPOUT OPEN_TANK")
            self._state = DropoutCherrySequencerState.TREMBLING
            # self._ros_api.flash_mcqueen.wiggle()
            self._trembling_chrono = time.time()
            
            return None
        elif self._state == DropoutCherrySequencerState.TREMBLING:
            if (time.time() - self._trembling_chrono) > TREMBLING_TIME:
                self._state = DropoutCherrySequencerState.FINISH
                basket_plate = RobotDisplacement.get_basket_plate(self._map, self.start_plate)
                self._ros_api.general_purpose.close_cherry_door()
                dest, disp = self._go_to_center_end()
                self._counter += 1
                self._state = DropoutCherrySequencerState.GO_TO_PLATE

                return Action(
                    key='center',
                    start_coord=self.current_position,
                    end_coord=dest,
                    displacement=disp
                )
        elif self._state == DropoutCherrySequencerState.FINISH:
            return None
        elif self._state == DropoutCherrySequencerState.BACKWARD:
            self._counter += 1

        else:
            self._state = DropoutCherrySequencerState.WAIT

    def _go_to_wall_park(self):
        dest_coordinate = Coordinate(
            x=self.current_position.x,
            y=self._map.length + 1000,
            angle=0.0,
        )

        return dest_coordinate, RobotDisplacement.get_displacement_to_coordinate(
            'park',
            self.current_position, 
            dest_coordinate,
        )

    def _go_to_center(self):
        end_plate = RobotDisplacement.get_basket_plate(self._map, self.start_plate)
        plate = self._map.plates[end_plate]

        if end_plate == 'plate-10': # RobotDisplacement.is_on_right(self._map, self.current_position):
            dest_coordinate = Coordinate(
                x=plate['x_pos'] - 120.0,
                y=plate['y_pos'] - plate['y_size'] / 4,
                angle=0.0,
            )
        else:
            dest_coordinate = Coordinate(
                x=plate['x_pos'] + 70.0,
                y=plate['y_pos'] - plate['y_size'] / 4,
                angle=0.0,
            )

        return dest_coordinate, RobotDisplacement.get_displacement_to_coordinate('center', self.current_position, dest_coordinate)

    def _go_to_center_end(self):
        end_plate = RobotDisplacement.get_basket_plate(self._map, self.start_plate)
        plate = self._map.plates[end_plate]
        dest_coordinate = Coordinate(
            x=self.current_position.x,
            y=self.current_position.y - plate['y_size'] / 2,
            angle=0.0,
        )

        return dest_coordinate, RobotDisplacement.get_displacement_to_coordinate('center', self.current_position, dest_coordinate, backward=True)

    def _go_to_aruco_tag(self):
        if self.start_plate == 'plate-8':
            aruco = self._map.aruco_tags['tag-21']
            dest_coordinate = Coordinate(
                x=aruco['x_pos'] - 70.0,
                y=aruco['y_pos'],
                angle=0.0,
            )
        else:
            aruco = self._map.aruco_tags['tag-20']
            dest_coordinate = Coordinate(
                x=aruco['x_pos'] + 70.0,
                y=aruco['y_pos'],
                angle=0.0,
            )

        return dest_coordinate, RobotDisplacement.get_displacement_to_coordinate('aruco', self.current_position, dest_coordinate, backward=False)

    def _set_normal_speed(self):
        self._ros_api.flash_mcqueen.set_max_speed(NORMAL_SPEED)
    
    def _set_park_speed(self):
        self._ros_api.flash_mcqueen.set_max_speed(PARK_SPEED)