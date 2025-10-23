import rclpy
from rclpy.node import Node
from prob_rob_msgs.msg import Point2DArrayStamped
from sensor_msgs.msg import CameraInfo


heartbeat_period = 0.1

class LandmarkPositioner(Node):

    def __init__(self):
        super().__init__('landmark_positioner')
        self.log = self.get_logger()
        self.timer = self.create_timer(heartbeat_period, self.heartbeat)
        self.declare_parameter('landmark_color', 'cyan')
        self.landmark_color = self.get_parameter('landmark_color').get_parameter_value().string_value
        self.landmark_points_sub = self.create_subscription(
            Point2DArrayStamped,
            f'/vision_{self.landmark_color}/corners',
            self.landmark_positioning_callback,
            10
        )

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/camera_info',
            self.camera_info_callback,
            10
        )

    def heartbeat(self):
        self.log.info('heartbeat')

    def spin(self):
        rclpy.spin(self)


def main():
    rclpy.init()
    landmark_positioner = LandmarkPositioner()
    landmark_positioner.spin()
    landmark_positioner.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
