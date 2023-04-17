import eurobot2023.msg as msg
import ros_api.ros_tools as tools


class KobeApi:
    def __init__(self):
        self.get_cherry_callback = tools.default_callback

    def start(self):
        self._define_publishers()
        self._define_subscribers()

    def request_cherry(self):
        self._request_cherry_pub.publish()

    def _define_publishers(self):
        self._request_cherry_pub = tools.create_publisher('request-cherry')

    def _define_subscribers(self):
        self._get_cherry_sub = tools.create_subscriber('get-cherry', self.get_cherry_callback)
