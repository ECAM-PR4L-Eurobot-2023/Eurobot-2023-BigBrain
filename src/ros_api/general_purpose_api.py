import eurobot2023.msg as msg
import ros_api.ros_tools as tools


class GeneralPurposeApi:
    def __init__(self):
        self.set_start_plate_callback = tools.default_callback

    def start(self):
        self._define_publishers()
        self._define_subscribers()

    def turn_on_fan(self, val):
        self._set_fan_on.publish(val)

    def turn_off_fan(self):
        self._set_fan_off.publish()

    def _define_publishers(self):
        self._set_display_pub = tools.create_publisher('set-display')
        self._set_fan_on = tools.create_publisher('fan-on')
        self._set_fan_off = tools.create_publisher('fan-off')

    def _define_subscribers(self):
        self._set_start_plate_sub = tools.create_subscriber('start-plate', 
            self.set_start_plate_callback)
