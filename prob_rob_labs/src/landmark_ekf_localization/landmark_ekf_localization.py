#!/usr/bin/env python3
import math
import json

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import CameraInfo
from prob_rob_msgs.msg import Point2DArrayStamped

path = 'src/prob_rob_labs_ros_2/prob_rob_labs/src/landmark_ekf_localization/landmark_map.json'


class LandmarkEkfLocalization(Node):
    """
    Initial EKF localization node for the measurement branch:

    - Loads landmark map from JSON (map_path parameter)
    - Builds color -> landmark lookup (correspondence mapping)
    - Subscribes to /vision_<color>/corners for all colors in the map
    - For each message, computes:
        * distance d
        * bearing theta
        * variances σ_d^2(d), σ_θ^2(θ) via the piecewise model
    """

    def __init__(self):
        super().__init__('landmark_ekf_localization')
        self.log = self.get_logger()

        self.declare_parameter('map_path', path)
        self.declare_parameter('landmark_height', 0.5)  # meters (known landmark height)

        map_path = self.get_parameter('map_path').get_parameter_value().string_value
        self.landmark_height = float(
            self.get_parameter('landmark_height').get_parameter_value().double_value
        )

        if not map_path:
            self.log.error('Parameter "map_path" is empty. Set it in the launch file.')
            raise RuntimeError('map_path parameter not set')
        try:
            with open(map_path, 'r') as f:
                map_data = json.load(f)
        except Exception as e:
            self.log.error(f'Failed to load map file "{map_path}": {e}')
            raise

        landmarks_list = map_data.get('landmarks', [])
        if not landmarks_list:
            self.log.error('No "landmarks" entry found in map file.')
            raise RuntimeError('Invalid map file format')

    
        self.landmarks_by_color = {}
        for lm in landmarks_list:
            color = lm['color']
            self.landmarks_by_color[color] = lm

        self.log.info(f'Loaded {len(self.landmarks_by_color)} landmarks from {map_path}')
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        self.camera_sub = self.create_subscription(
            CameraInfo,
            '/camera/camera_info',
            self.camera_callback,
            10
        )

        # Base variances
        self.var_d_base = 0.1467      
        self.var_theta_base = 0.000546  

        self.corner_subs = {}
        for color, landmark in self.landmarks_by_color.items():
            topic = f'/vision_{color}/corners'
            sub = self.create_subscription(
                Point2DArrayStamped,
                topic,
                lambda msg, c=color, lm=landmark: self.corner_callback(msg, c, lm),
                10
            )
            self.corner_subs[color] = sub
            self.log.info(f'Subscribed to corner topic: {topic}')

        self.log.info('EKF measurement initialization node is ready.')


    def camera_callback(self, msg: CameraInfo):
        # K = [fx 0 cx ; 0 fy cy ; 0 0 1]
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]

    def corner_callback(self, msg: Point2DArrayStamped, color: str, landmark: dict):
        if self.fx is None or self.fy is None or self.cx is None or self.cy is None:
            self.log.warn('No camera intrinsics yet, skipping measurement')
            return

        points = msg.points
        num_points = len(points)
        if num_points == 0:
            return 

        xs = [p.x for p in points]
        ys = [p.y for p in points]

        min_y = min(ys)
        max_y = max(ys)
        height_pix = max_y - min_y


        #using rectangular model when landmark has fewer points so far away, cylindrical when close and more points
        if num_points < 8:
            min_x = min(xs)
            max_x = max(xs)
            x_sym = 0.5 * (min_x + max_x)
            shape = 'rect'
        else:
            x_sym = sum(xs) / float(num_points)
            shape = 'cyl'

        # Bearing in camera frame
        theta = math.atan((self.cx - x_sym) / self.fx)

        cos_th = math.cos(theta)
        if abs(cos_th) < 1e-3:
            # too close to 90°, avoid numerical blow-up
            return

        # Distance from apparent height:
        # d = h * fy / (height_pix * cos(theta))
        d = self.landmark_height * self.fy / (height_pix * cos_th)

        var_d, var_theta = self.measurement_variance(d, theta)

        lm_pos = landmark['position']
        self.log.info(
            f'[{color}] {shape}: h_pix={height_pix:.1f}, x_sym={x_sym:.1f} '
            f'-> d={d:.3f} m, θ={theta:.3f} rad; '
            f'σ_d^2={var_d:.4f}, σ_θ^2={var_theta:.6f}; '
            f'landmark at map=({lm_pos["x"]:.2f}, {lm_pos["y"]:.2f})'
        )

    def measurement_variance(self, d: float, theta: float):
        '''
        piecewise variance model as in lab 5, define base variances for safe zone and blow them up when too close or too far'''
        outside_reliable = (d < 1.0) or (d > 10.0) or (abs(theta) > 0.6)

        factor = 3.0 if outside_reliable else 1.0

        var_d = factor * self.var_d_base
        var_theta = factor * self.var_theta_base

        return var_d, var_theta

    def spin(self):
        rclpy.spin(self)


def main(args=None):
    rclpy.init(args=args)
    node = LandmarkEkfLocalization()
    node.spin()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
