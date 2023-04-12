import ros_api.ros_tools as tools


class LidarApi:
    def __init__(self):
        self._on_lidar_data = tools.default_callback

    def start(self):
        self._define_publishers()
        self._define_subscribers()

    def start_scan(self):
        print('start scan')
        self._start_scan_pub.publish()

    def stop_scan(self):
        self._stop_scan_pub.publish()

    def set_precision(self, precision):
        self._set_precision_pub.publish(precision)

    def set_rate(self, rate):
        self._set_rate_pub.publish(rate)

    def _define_publishers(self):
        self._start_scan_pub = tools.create_publisher('start-scan')
        self._stop_scan_pub = tools.create_publisher('stop-scan')
        self._set_precision_pub = tools.create_publisher('set-precision')
        self._set_rate_pub = tools.create_publisher('set-rate')

    def _define_subscribers(self):
        self._lidar_data_sub = tools.create_subscriber('lidar-data', self._on_lidar_data)