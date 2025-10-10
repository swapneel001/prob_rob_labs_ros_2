import rclpy
import random
from rclpy.node import Node
from std_msgs.msg import Float64
from std_msgs.msg import Empty

max_torque = 5.0
required_open_time = 3

class FlakyDoorOpener(Node):

    def __init__(self):
        super().__init__('flaky_door_opener')
        self.log = self.get_logger()
        self.pub_torque = self.create_publisher(
            Float64, '/hinged_glass_door/torque', 1)
        self.sub_command = self.create_subscription(
            Empty, '/door_open', self.handle_command, 1)
        self.state = 'trying push'

    def handle_command(self, _):
        if self.state == 'trying push':
            self.log.info(f'state: {self.state}')
            self.torque = random.choice([1.0, 0.0, 0.0, 0.0, 0.0]) * max_torque
            if self.torque != 0.0:
                self.state = 'pushing'
                self.push_start_time = self.get_clock().now()
        elif self.state == 'pushing':
            self.log.info(f'state: {self.state}')
            elapsed_time = self.get_clock().now() - self.push_start_time
            if elapsed_time.nanoseconds / 1e9 > required_open_time:
                self.state = 'trying push'
        self.log.info(f'door open requested using torque {self.torque}')
        self.pub_torque.publish(Float64(data=self.torque))

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
