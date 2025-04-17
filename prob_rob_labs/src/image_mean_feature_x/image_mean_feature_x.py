import rclpy
from rclpy.node import Node


heartbeat_period = 1

class ImageMeanFeatureX(Node):

    def __init__(self):
        super().__init__('image_mean_feature_x')
        self.log = self.get_logger()
        self.timer = self.create_timer(heartbeat_period, self.heartbeat)

    def heartbeat(self):
        self.log.info('heartbeat')

    def spin(self):
        rclpy.spin(self)


def main():
    rclpy.init()
    image_mean_feature_x = ImageMeanFeatureX()
    image_mean_feature_x.spin()
    image_mean_feature_x.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
