import rclpy
from rclpy.node import Node


heartbeat_period = 1

class FlakyDoorOpener(Node):

    def __init__(self):
        super().__init__('flaky_door_opener')
        self.log = self.get_logger()
        self.timer = self.create_timer(heartbeat_period, self.heartbeat)

    def heartbeat(self):
        self.log.info('heartbeat')

    def spin(self):
        rclpy.spin(self)


def main():
    rclpy.init()
    flaky_door_opener = FlakyDoorOpener()
    flaky_door_opener.spin()
    flaky_door_opener.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
