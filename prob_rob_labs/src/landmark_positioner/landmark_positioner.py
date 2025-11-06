#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node

from prob_rob_msgs.msg import Point2DArrayStamped
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import Point  # to publish (distance, bearing)

heartbeat_period = 0.1


class LandmarkPositioner(Node):

    def __init__(self):
        super().__init__('landmark_positioner')
        self.log = self.get_logger()
        self.timer = self.create_timer(heartbeat_period, self.heartbeat)
        self.declare_parameter('landmark_color', 'cyan')
        self.declare_parameter('landmark_height', 0.5)  # meters
        self.landmark_color = self.get_parameter('landmark_color').get_parameter_value().string_value
        self.landmark_height = float(self.get_parameter('landmark_height').get_parameter_value().double_value)
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None
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
        topic_name = f'/vision_{self.landmark_color}/measurement'
        self.meas_pub = self.create_publisher(Point, topic_name, 10)
        self.log.info(f'Publishing landmark measurements on {topic_name}')


    def heartbeat(self):
        self.log.info('no landmark seeen, heartbeat')

    def camera_info_callback(self, msg: CameraInfo):
        # K = [fx 0 cx ; 0 fy cy ; 0 0 1]
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]

    def landmark_positioning_callback(self, msg: Point2DArrayStamped):
        if self.fx is None or self.fy is None or self.cx is None or self.cy is None:
            self.log.warn('No camera intrinsics yet, skipping measurement')
            return

        num_points = len(msg.points)
        if num_points == 0:
            #no landmark seen
            return

        xs = [p.x for p in msg.points]
        ys = [p.y for p in msg.points]

        min_y = min(ys)
        max_y = max(ys)
        height_pix = max_y - min_y

        if height_pix < 1.0:
            # too noisy / tiny detection
            return

        if num_points < 8:
            # rectangular
            min_x = min(xs)
            max_x = max(xs)
            x_sym = (max_x + min_x) / 2.0
        else:
            # cylindrical
            x_sym = sum(xs) / float(num_points)

        # bearing theta
        theta = math.atan((self.cx - x_sym) / self.fx)
        # distance 
        cos_th = math.cos(theta)
        if abs(cos_th) < 1e-3:
            return  # avoid div by zero
        d = self.landmark_height * self.fy / (height_pix * cos_th)

        out = Point()
        out.x = float(d)
        out.y = float(theta)
        out.z = 0.0
        self.meas_pub.publish(out)

        shape = 'rect' if num_points < 8 else 'cyl'
        self.log.info(f'{shape}: h_pix={height_pix:.1f}, x_sym={x_sym:.1f} -> d={d:.3f} m, th={theta:.3f} rad')

    def spin(self):
        rclpy.spin(self)


def main():
    rclpy.init()
    node = LandmarkPositioner()
    node.spin()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
