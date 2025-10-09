import rclpy
from rclpy.node import Node


heartbeat_period = 0.1

class BayesDoorController(Node):

    def __init__(self):
        super().__init__('bayes_door_controller')
        self.log = self.get_logger()
        self.timer = self.create_timer(heartbeat_period, self.heartbeat)

    def heartbeat(self):
        self.log.info('heartbeat')

    def spin(self):
        rclpy.spin(self)


def main():
    rclpy.init()
    bayes_door_controller = BayesDoorController()
    bayes_door_controller.spin()
    bayes_door_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
