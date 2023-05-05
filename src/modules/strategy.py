import math
import time

from models.action import Action
from models.coordinate import Coordinate
from models.displacement import Displacement
from modules.cherry_sequencer import CherrySequencer, SequencerCherryState
from modules.dummy_sequencer import DummySequencer, DummySequencerState
from modules.dropout_cherry_sequencer import DropoutCherrySequencer, DropoutCherrySequencerState
from modules.emergency_stop import EmergencyStopDetector
from modules.obstacle_avoider import ObstacleAvoider
from modules.robot_displacement import RobotDisplacement, LEFT_PLATES, RIGHT_PLATES, LEFT_MAP_CHERRIES, RIGHT_MAP_CHERRIES
from modules.score_simulator import ScoreSimulator


MATCH_TIME  = 100.0  # s
DISGUISE_TIME = MATCH_TIME - 15.0  # s
END_MATCH = MATCH_TIME - 5.0


class StrategyState:
    WAIT = 0
    START = 1
    PICK_UP_CHERRIES = 2
    DROP_CHERRIES = 3
    GO_TO_PLATE = 4
    FINISH = 5
    DUMMY = 6
    OBSTACLE_AVOID = 7
    STOP_ROBOT = 8
    DISGUISE = 9


class Strategy:
    def __init__(self, ros_api, game_map, current_position, start_position, start_plate, lidar):
        self._ros_api = ros_api
        self._map = game_map
        self._queue = []
        self._is_start = False
        self._is_destination_reached = False
        self._state = StrategyState.WAIT
        self._mem_state = StrategyState.WAIT
        self._lidar = lidar
        self.score_simulator = ScoreSimulator()

        # Memorize the actions done during the game
        self._cherries_to_visit = []
        self._cherries_visited = set()
        self._plates_visited = set()

        # Store the current robot position
        self._current_position = current_position
        self._start_position = start_position
        self._current_destination = None

        # Store the start plate to adapt the strategy
        self._start_plate = start_plate

        # Store the time from the start of the game
        self._chrono = time.time()

        # Sequencer
        self._cherry_sequencer = CherrySequencer(self._ros_api, self._map, self._current_position, '')
        self._dummy_sequencer = DummySequencer(self._ros_api, self._map, self._current_position)
        self._dropout_cherry_sequencer = DropoutCherrySequencer(self._ros_api, 
            self._map, self._current_position, self._start_plate)
        self._obstacle_avoider = ObstacleAvoider(self._ros_api, self._current_position, None)

        
        self._mem_obstacle = False
        self._is_obstacle = False
        self._is_disguised = False
        self._in_game = False

    @property
    def current_position(self):
        return self._current_destination

    @property
    def start_plate(self):
        return self._start_plate

    @start_plate.setter
    def start_plate(self, start_plate):
        self._start_plate = start_plate
        self._dropout_cherry_sequencer.start_plate = start_plate

    @current_position.setter
    def current_position(self, current_position):
        self._current_position = current_position
        self._cherry_sequencer.current_position = current_position

    # @property
    # def score_simulator(self):
    #     return self._score_simulator

    def reset(self):
        self._chrono = time.time()
        self._is_destination_reached = False
        self._is_start = True
        self._is_disguised = False
        self._in_game = True
        self.clear_queue()
        self._cherries_visited.clear()
        self._plates_visited.clear()

        # Start cherry
        self._cherry_sequencer.reset()
        self._dropout_cherry_sequencer.reset()
        self.score_simulator.reset()

        # Set the start position
        # self._ros_api.flash_mcqueen.set_position(self._start_position)
        # self._ros_api.flash_mcqueen.set_rotation(math.radians(self._start_position.angle))

    def set_destination_reached(self):
        self._is_destination_reached = True
        # self._set_visited()

    def clear_queue(self):
        self._queue = []

    def start(self):
        self._state = StrategyState.START
        self._start_sequence()

    def run(self):
        self._is_obstacle = EmergencyStopDetector.detect_emergency_stop(
            self._lidar, self._current_position, self._map)

        if not self._disable_lidar() and self._is_obstacle and not self._mem_obstacle:
            print('--- Obstacle ---')
            print(self._lidar.distances)
            self._ros_api.flash_mcqueen.set_stop()
            self._mem_state = self._state
            # self._state = StrategyState.OBSTACLE_AVOID
            # self._obstacle_avoider.reset()
            # self._obstacle_avoider.destination = self._current_destination
        elif not self._is_obstacle and self._mem_obstacle and not self._is_disguised:
            print('recompute val')
            self._recompute_destination()
            self._go_to_destination()

        self._mem_obstacle = self._is_obstacle

        # Disguise after timeout
        if self._in_game and not self._is_disguised and (time.time() - self._chrono) > DISGUISE_TIME:
            self._is_disguised = True
            print('--- DISGUISE ---')
            self._state = StrategyState.DISGUISE
            self._ros_api.general_purpose.disguise()
            self._ros_api.flash_mcqueen.set_stop()

        if self._in_game and (time.time() - self._chrono) > END_MATCH:
            self._state =  StrategyState.STOP_ROBOT

        if self._state == StrategyState.WAIT:
            pass
        elif self._state == StrategyState.START:
            print('--- START---')
            if (time.time() - self._chrono) > 0.5:
                # self._choose_next_cherry()
                # self._queue.append(self._cherry_sequencer.run())
                # self._go_to_destination()
                print("Go here")
                # pass
                self._state = StrategyState.PICK_UP_CHERRIES
                # self._state = StrategyState.DUMMY
        elif self._state == StrategyState.PICK_UP_CHERRIES:
            # print('PICK_UP_CHERRIES')
            if self._is_destination_reached or self._is_start:
                self._is_start = False

                if not self._queue:
                    move = self._cherry_sequencer.run()

                    if not move and self._cherry_sequencer.state == SequencerCherryState.WAIT:
                        self._set_visited()
                        self._choose_next_cherry()
                    elif move:
                        self._queue.append(move)

                self._go_to_destination()

        elif self._state == StrategyState.DROP_CHERRIES:
            if self._is_destination_reached or self._is_start or self._dropout_cherry_sequencer.run_next_state:
                self._dropout_cherry_sequencer.run_next_state = False
                self._is_start = False
                move = self._dropout_cherry_sequencer.run()

                if not move and self._dropout_cherry_sequencer.state == DropoutCherrySequencerState.WAIT:
                    print("End of sequence for dropout")
                elif move:
                    self._queue.append(move)

                self._go_to_destination()
        elif self._state == StrategyState.GO_TO_PLATE:
            pass
        elif self._state == StrategyState.FINISH:
            pass
        elif self._state == StrategyState.DUMMY:
            if self._is_destination_reached or self._is_start:
                self._is_start = False
                move = self._dummy_sequencer.run()
                print(move)

                if not move:
                    print("End of dummy sequence")
                elif move:
                    self._queue.append(move)

                self._go_to_destination()
        elif self._state == StrategyState.OBSTACLE_AVOID:
            if self._is_destination_reached:
                move = self._obstacle_avoider.run()

                if not move:
                    self._state = self._mem_state
                    self._queue.append(self._obstacle_avoider.destination)
                else:
                    self._queue.append(move)
                self._go_to_destination()
        elif self._state == StrategyState.STOP_ROBOT:
            self._state = StrategyState.FINISH
            # self._ros_api.flash_mcqueen.set_stop()
            self._ros_api.general_purpose.end()
            # self._ros_api.general_purpose.set_display(self.score_simulator.score)
            self._in_game = False
            print('--- STOP ROBOT ---')
        elif self._state == StrategyState.DISGUISE:
            print('--- DISGUISE ---')
            pass
        else:
            self._state = StrategyState.WAIT

    def manage_limit_switches(self, limit_switches):
        if limit_switches.is_back_pressed():
            print("--- BACK Pressed ---")
            if self._state == StrategyState.PICK_UP_CHERRIES:
                print("--- BACK Pressed While picking cherries ---")
                print(f"self._cherry_sequencer.state {self._cherry_sequencer.state} {SequencerCherryState.GET_IN}")
                if self._cherry_sequencer.state == SequencerCherryState.GET_IN:
                    print("--- LIMIT SWITCH and GET_IN ---")
                    time.sleep(0.2)
                    if self._cherry_sequencer.cherry == 'down':
                        self._current_position.y = 0
                    else:
                        self._current_position.y = self._map.length

                    print(self._current_position)
                    self._ros_api.flash_mcqueen.set_position(self._current_position)
                    # self._ros_api.flash_mcqueen.set_stop()
                    self._queue.append(self._cherry_sequencer.run())
                    self._go_to_destination()
            elif self._state == StrategyState.DROP_CHERRIES:
                print("--- Pressed whil DROP cherries")
                print(self._dropout_cherry_sequencer.state)
                if self._dropout_cherry_sequencer.state == DropoutCherrySequencerState.GO_TO_BASKET:
                    print('GO TO BASKET')
                    self._ros_api.flash_mcqueen.set_stop()
                    self._dropout_cherry_sequencer.run_next_state = True
                    self._is_destination_reached = True
            else:
                print("--- STOP BECAUSE PRESS ---")
                self._ros_api.flash_mcqueen.set_stop()

    def _start_sequence(self):
        self.reset()

        self._start = True
        # self._cross_sequence()
        # self._go_to_destination()

        print("self._cherries_visited : ", self._cherries_visited)
        # Create the cherry sequence
        if self._start_plate in LEFT_PLATES:
            self._cherries_to_visit = ['left',]
        else:
            self._cherries_to_visit = ['right',]

        self._cherry_sequencer.cherry = self._cherries_to_visit[0]

    def _set_visited(self):
        if not self._current_destination:
            return

        key = self._current_destination.key

        if key in self._map.plates:
            self._plates_visited.add(key)
        elif key in self._map.cherries:
            self._cherries_visited.add(key)

    def _recompute_destination(self):
        if not self._current_destination:
            return

        mem_destination = self._current_destination
        dest = Action(
            key=self._current_destination.key,
            start_coord=self._current_position,
            end_coord=self._current_destination.end_coord,
            displacement=RobotDisplacement.get_displacement_to_coordinate(
                    self._current_destination.key,
                    self._current_position,
                    self._current_destination.end_coord,
            )
        )
        self._queue.append(dest)

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
            print(self._current_destination)
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

    def _disable_lidar(self):
        if 1450 < self._current_position.x < 2550 and 2450 < self._current_position.y < 3550:
            return True

        if -550 < self._current_position.x < 550 and 2450 < self._current_position.y < 3550:
            return True

        return False
        # return self._dropout_cherry_sequencer.state == DropoutCherrySequencerState.GO_TO_CENTER or
        #     self._dropout_cherry_sequencer.state == DropoutCherrySequencerState.GO_TO_CENTER or \
        #     self._dropout_cherry_sequencer.state == DropoutCherrySequencerState.GO_TO_BASKET or \
        #     self._dropout_cherry_sequencer.state == DropoutCherrySequencerState.OPEN_TANK or \
        #     self._dropout_cherry_sequencer.state == DropoutCherrySequencerState.TREMBLING or \
        #     self._dropout_cherry_sequencer.state == DropoutCherrySequencerState.BACKWARD
