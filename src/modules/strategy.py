import time

from models.action import Action
from models.coordinate import Coordinate
from models.displacement import Displacement
from modules.robot_displacement import RobotDisplacement, LEFT_PLATES, RIGHT_PLATES


LEFT_MAP_CHERRIES = {'left', 'up'}
RIGHT_MAP_CHERRIES = {'right', 'up'}


class Strategy:
    def __init__(self, ros_api, game_map, current_position, start_plate):
        self._ros_api = ros_api
        self._map = game_map
        self._queue = []
        self._is_destination_reached = False

        # Memorize the actions done during the game
        self._cherries_visited = set()
        self._plates_visited = set()

        # Store the current robot position
        self.current_position = current_position
        self._current_destination = None

        # Store the start plate to adapt the strategy
        self._start_plate = start_plate

        # Store the time from the start of the game
        self._chrono = time.time()

        self.current_position.angle = 250.0

    def reset(self):
        self._is_destination_reached = False
        self.clear_queue()
        self._cherries_visited.clear()
        self._plates_visited.clear()

    def set_destination_reached(self):
        self._is_destination_reached = True
        self._set_visited()

    def clear_queue(self):
        self._queue = []

    def _set_visited(self):
        if not self._current_destination:
            return

        key = self._current_destination.key

        if key in self._map.plates:
            self._plates_visited.add(key)
        elif key in self._map.cherries:
            self._cherries_visited.add(key)

        print('plate visited: ', self._plates_visited)

    def start(self):
        self.reset()
        self._cross_sequence()
        self._go_to_destination()

    def run(self):
        if self._is_destination_reached:
            if not self._queue:
                self._cross_sequence()

            self._go_to_destination()

    def _cross_sequence(self):
        CROSS_SEQUENCE = ['plate-1', 'plate-7', 'plate-4', 'plate-10']

        map_center = Coordinate(x=self._map.width / 2, y=self._map.length / 2, angle=0.0)
        disp = Action(
            key='', 
            start_coord=self.current_position, 
            displacement=RobotDisplacement.get_displacement_to_coordinate(self.current_position, map_center)
        )

        for plate in CROSS_SEQUENCE:
            if plate not in self._plates_visited:
                disp = Action(
                    key=plate,
                    start_coord=self.current_position,
                    displacement=RobotDisplacement.get_displacement_to_map_item(
                        self.current_position, 
                        self._map.plates[plate])
                )

                break

        self._queue.append(disp)

    def _go_to_destination(self):
        if not self._queue:
            return

        if self._queue:
            self._is_destination_reached = False
            self._current_destination = self._queue.pop(0)
            self._ros_api.flash_mcqueen.set_displacement(self._current_destination.displacement)

    def _is_cherries_available_from_start(self):
        on_side_cherry = LEFT_MAP_CHERRIES if (self._start_plate in LEFT_PLATES) \
            else RIGHT_MAP_CHERRIES
        
        return len([cherry for cherry in self.cherries if (cherry in on_side_cherry) and \
            (cherry not in self._cherries_visited)]) > 0

    def get_nearest_cherries_from_start(self):
        on_side_cherry = LEFT_MAP_CHERRIES if (self._start_plate in LEFT_PLATES) \
            else RIGHT_MAP_CHERRIES

        return RobotDisplacement.get_nearest_cherries(self._map, self.current_position, 
            on_side_cherry | self._cherries_visited)

    def _is_cherries_available():
        return len(self._cherries_visited)
