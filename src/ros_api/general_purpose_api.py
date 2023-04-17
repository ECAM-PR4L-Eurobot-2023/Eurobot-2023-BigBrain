import eurobot2023.msg as msg
import ros_api.ros_tools as tools


class GeneralPurposeApi:
    def __init__(self):
        self.set_start_plate_callback = tools.default_callback
        self.start_callback = tools.default_callback

    def start(self):
        self._define_publishers()
        self._define_subscribers()

    def turn_on_fan(self, val):
        self._set_fan_on_pub.publish(val)

    def turn_off_fan(self):
        self._set_fan_off_pub.publish()

    def open_cherry_door(self):
        self._open_cherry_door_pub.publish()

    def close_cherry_door(self):
        self._close_cherry_door_pub.publish()

    def disguise(self):
        self._disguise_pub.publish()

    def set_display(self, value):
        self._set_display_pub.publish(value)

    def _define_publishers(self):
        self._set_display_pub = tools.create_publisher('set-display')
        self._set_fan_on_pub = tools.create_publisher('fan-on')
        self._set_fan_off_pub = tools.create_publisher('fan-off')
        self._open_cherry_door_pub = tools.create_publisher('open-cherry-door')
        self._close_cherry_door_pub = tools.create_publisher('close-cherry-door')
        self._disguise_pub = tools.create_publisher('robot-disguise')

    def _define_subscribers(self):
        self._set_start_plate_sub = tools.create_subscriber('start-plate', 
            self.set_start_plate_callback)
        self._start_sub = tools.create_subscriber('robot-start', self.start_callback)
