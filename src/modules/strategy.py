import time

from models.action import Action
from models.coordinate import Coordinate
from models.displacement import Displacement
from modules.cherry_sequencer import CherrySequencer, SequencerCherryState
from modules.robot_displacement import RobotDisplacement, LEFT_PLATES, RIGHT_PLATES, LEFT_MAP_CHERRIES, RIGHT_MAP_CHERRIES


class StrategyState:
    WAIT = 0
    START = 1
    PICK_UP_CHERRIES = 2
    DROP_CHERRIES = 3
    GO_TO_PLATE = 4
    FINISH = 5


class Strategy:
    def __init__(self, ros_api, game_map, current_position, start_plate):
        self._ros_api = ros_api
        self._map = game_map
        self._queue = []
        self._is_start = False
        self._is_destination_reached = False
        self._state = StrategyState.WAIT

        # Memorize the actions done during the game
        self._cherries_to_visit = []
        self._cherries_visited = set()
        self._plates_visited = set()

        # Store the current robot position
        self._current_position = current_position
        self._current_destination = None

        # Store the start plate to adapt the strategy
        self._start_plate = start_plate

        # Store the time from the start of the game
        self._chrono = time.time()

        # Sequencer
        self._cherry_sequencer = CherrySequencer(self._ros_api, self._map, self._current_position, '')

    @property
    def current_position(self):
        return self._current_destination

    @current_position.setter
    def current_position(self, current_position):
        self._current_position = current_position
        self._cherry_sequencer.current_position = current_position

    def reset(self):
        self._chrono = time.time()
        self._is_destination_reached = False
        self._is_start = True
        self.clear_queue()
        self._cherries_visited.clear()
        self._plates_visited.clear()

        # Set the start position
        self._ros_api.flash_mcqueen.set_position(self._current_position)

    def set_destination_reached(self):
        self._is_destination_reached = True
        # self._set_visited()

    def clear_queue(self):
        self._queue = []

    def start(self):
        self._state = StrategyState.START
        print('Set start')
        self._start()

    def run(self):
        if self._state == StrategyState.WAIT:
            pass
        elif self._state == StrategyState.START:
            print('START')
            print(time.time() - self._chrono)
            if self._is_start and (time.time() - self._chrono) > 0.5:
                self._is_start = False
                self._choose_next_cherry()
                self._queue.append(self._cherry_sequencer.run())
                self._go_to_destination()
                self._state = StrategyState.PICK_UP_CHERRIES
        elif self._state == StrategyState.PICK_UP_CHERRIES:
            # print('PICK_UP_CHERRIES')
            if self._is_destination_reached:
                if not self._queue:
                    move = self._cherry_sequencer.run()

                    if not move and self._cherry_sequencer.state == SequencerCherryState.WAIT:
                        self._set_visited()
                        self._choose_next_cherry()
                    elif move:
                        self._queue.append(move)

                self._go_to_destination()

        elif self._state == StrategyState.DROP_CHERRIES:
            pass
        elif self._state == StrategyState.GO_TO_PLATE:
            pass
        elif self._state == StrategyState.FINISH:
            pass
        else:
            self._state = StrategyState.WAIT

    def _start(self):
        self.reset()

        self._start = True
        # self._cross_sequence()
        # self._go_to_destination()

        # Start cherry
        self._cherry_sequencer.reset()

        # Create the cherry sequence
        if self._start_plate in LEFT_PLATES:
            self._cherries_to_visit = ['left', 'down',]
        else:
            self._cherries_to_visit = ['down', 'right']

        self._cherry_sequencer.cherry = self._cherries_to_visit[0]

    def _set_visited(self):
        if not self._current_destination:
            return

        # if self._current_destination.end_coord.x - 10.0 < self.current_position.x < self._current_destination.end_coord.x + 10.0 and \
        #     self._current_destination.end_coord.y - 10.0 < self.current_position.y < self._current_destination.end_coord.y+ 10.0:

        key = self._current_destination.key

        if key in self._map.plates:
            self._plates_visited.add(key)
        elif key in self._map.cherries:
            self._cherries_visited.add(key)

    def _choose_next_cherry(self):
        print("--- Choose next cherry ---")
        print(f"self._cherries_to_visit: {self._cherries_to_visit}")
        print(f"self._cherries_visited: {self._cherries_visited}")

        # Get available cherries
        cherry_available = [cherry for cherry in self._cherries_to_visit \
            if cherry not in self._cherries_visited]

        print(f"cherry_available: {cherry_available}")

        if not cherry_available:
            self._state = StrategyState.DROP_CHERRIES
            return

        self._cherry_sequencer.reset()
        self._cherry_sequencer.cherry = cherry_available[0]

    def _cross_sequence(self):
        CROSS_SEQUENCE = ['plate-3', 'plate-5', 'plate-4']

        map_center = Coordinate(x=self._map.width / 2, y=self._map.length / 2, angle=0.0)
        disp = Action(
            key='', 
            start_coord=self._current_position, 
            end_coord=map_center,
            displacement=RobotDisplacement.get_displacement_to_coordinate('a', self._current_position, map_center),
        )

        for plate in CROSS_SEQUENCE:
            if plate not in self._plates_visited:
                end_coord = Coordinate(x=self._map.plates[plate]['x_pos'], y=self._map.plates[plate]['y_pos'], angle=0.0)
                disp = Action(
                    key=plate,
                    start_coord=self._current_position,
                    displacement=RobotDisplacement.get_displacement_to_map_item(
                    plate,
                        self._current_position, 
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

        return RobotDisplacement.get_nearest_cherries(self._map, self._current_position, 
            on_side_cherry | self._cherries_visited)

    def _is_cherries_available(self):
        return len(self._cherries_visited)
