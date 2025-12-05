#!/usr/bin/env python3
import math
import json
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import CameraInfo
from prob_rob_msgs.msg import Point2DArrayStamped
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from tf_transformations import quaternion_from_euler, euler_from_quaternion

from tf2_ros import Buffer, TransformListener, TransformException

path = 'src/prob_rob_labs_ros_2/prob_rob_labs/config/landmarks_map.json'


class LandmarkEkfLocalization(Node):

    def __init__(self):
        super().__init__('landmark_ekf_localization')
        self.log = self.get_logger()

        self.declare_parameter('map_path', path)
        self.declare_parameter('landmark_height', 0.5)  # meters

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

        # color -> landmark dict (with "position", "landmark", etc.)
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

        # State: [x, y, theta]^T in base frame (e.g. base_footprint)
        self.state = np.zeros((3, 1))
        self.I = np.eye(3)
        self.Cov = 0.01 * self.I

        # Process noise from odometry twist covariance (v, w)
        # M is 2x2 covariance of [v, w]
        self.M = np.array([[1.0e-05, 0.0],
                           [0.0,      0.001]])

        # Jacobians for prediction step
        self.G = self.I.copy()        # 3x3
        self.V = np.zeros((3, 2))     #3x2

        # Measurement noise base variances (from Lab 5 CSV analysis)
        self.var_d_base = 0.1467     
        self.var_theta_base = 0.000546
        self.Q = np.zeros((2, 2))
        self.H = np.zeros((2, 3))
        self.H[1, 2] = -1.0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.T_base_to_camera = None    # TF message
        self.cam_offset = np.zeros(2)   # [tx, ty] camera position in base frame
        self.cam_yaw = 0.0              # yaw offset camera wrt base
        # poll TF until we get the transform once
        self.tf_timer = self.create_timer(1.0, self.get_camera_transform)

        self.system_time = None   # rclpy.time.Time associated with current state
        self.initialized = False  # becomes True after first measurement timestamp is set
        self.last_vel = None      # (v, w) from /odom

        self.odom_pub = self.create_publisher(Odometry, "/ekf_pose", 10)

        # Odometry for prediction
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # Cornerpoint topics for all colors
        self.corner_subs = {}
        for color, landmark in self.landmarks_by_color.items():
            topic = f'/vision_{color}/corners'
            sub = self.create_subscription(
                Point2DArrayStamped,
                topic,
                # capture color + landmark with lambda
                lambda msg, c=color, lm=landmark: self.corner_callback(msg, c, lm),
                10
            )
            self.corner_subs[color] = sub
            self.log.info(f'Subscribed to corner topic: {topic}')

        self.log.info('Landmark EKF localization node is ready.')

    @staticmethod
    def time_diff_sec(t_new, t_old):
        return (t_new.sec - t_old.sec) + 1e-9 * (t_new.nanosec - t_old.nanosec)

    @staticmethod
    def unwrap(angle):
        """Wrap angle to [-pi, pi]."""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    @staticmethod
    def to_quaternion(theta):
        """Convert yaw to geometry_msgs/Quaternion."""
        qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, theta)
        q = Quaternion()
        q.x = qx
        q.y = qy
        q.z = qz
        q.w = qw
        return q

    # quaternion -> yaw helper for TF
    def q2yaw(self, quat):
        roll, pitch, yaw = euler_from_quaternion(
            [quat.x, quat.y, quat.z, quat.w]
        )
        return yaw

   #get base_link -> camera_rgb_frame transform
    def get_camera_transform(self):
        if self.T_base_to_camera is not None:
            return

        try:
            tf_msg = self.tf_buffer.lookup_transform(
                'base_link',          # target frame
                'camera_rgb_frame',   # source frame
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
        except TransformException as ex:
            self.log.warn(f'Could not get base_link -> camera_rgb_frame transform yet: {ex}')
            return

        self.T_base_to_camera = tf_msg
        trans = tf_msg.transform.translation
        rot = tf_msg.transform.rotation

        self.cam_offset = np.array([trans.x, trans.y])
        self.cam_yaw = self.q2yaw(rot)

        self.log.info(
            f'Camera transform: tx={self.cam_offset[0]:.3f}, '
            f'ty={self.cam_offset[1]:.3f}, yaw={self.cam_yaw:.3f}'
        )
        self.tf_timer.cancel()

    def camera_callback(self, msg: CameraInfo):
        # K = [fx 0 cx; 0 fy cy; 0 0 1]
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]

    def odom_callback(self, msg: Odometry):
        v = msg.twist.twist.linear.x
        w = msg.twist.twist.angular.z
        self.last_vel = (v, w)

        if not self.initialized:
            return

        timestamp = msg.header.stamp
        dt = self.time_diff_sec(timestamp, self.system_time)
        if dt <= 0.0:
            # late or zero-time sample, ignore
            return

        # advance system time
        self.system_time = timestamp

        self.prediction(v, w, dt)
        self.publish_ekf_pose(timestamp)

    def prediction(self, v, w, dt):
        theta = self.state[2, 0]

        # small angular velocity → linear motion model
        if abs(w) < 0.01:
            # Jacobians
            self.G[0, 2] = -v * dt * math.sin(theta)
            self.G[1, 2] =  v * dt * math.cos(theta)

            self.V[:, :] = 0.0
            self.V[0, 0] = dt * math.cos(theta)
            self.V[1, 0] = dt * math.sin(theta)
            self.V[2, 1] = dt

            # State update (linear)
            self.state[0, 0] += v * dt * math.cos(theta)
            self.state[1, 0] += v * dt * math.sin(theta)
            self.state[2, 0] = self.unwrap(theta + w * dt)

        else:
            # Arc motion model
            self.G[0, 2] = -v / w * math.cos(theta) + v / w * math.cos(theta + w * dt)
            self.G[1, 2] = -v / w * math.sin(theta) + v / w * math.sin(theta + w * dt)

            self.V[:, :] = 0.0
            self.V[0, 0] = (-math.sin(theta) + math.sin(theta + w * dt)) / w
            self.V[0, 1] = (
                v * (math.sin(theta) - math.sin(theta + w * dt)) / (w ** 2)
                + v * math.cos(theta + w * dt) * dt / w
            )
            self.V[1, 0] = (math.cos(theta) - math.cos(theta + w * dt)) / w
            self.V[1, 1] = (
                -v * (math.cos(theta) - math.cos(theta + w * dt)) / (w ** 2)
                + v * math.sin(theta + w * dt) * dt / w
            )
            self.V[2, 1] = dt

            self.state[0, 0] += -v / w * math.sin(theta) + v / w * math.sin(theta + w * dt)
            self.state[1, 0] +=  v / w * math.cos(theta) - v / w * math.cos(theta + w * dt)
            self.state[2, 0] = self.unwrap(theta + w * dt)
        self.Cov = self.G @ self.Cov @ self.G.T + self.V @ self.M @ self.V.T

    def corner_callback(self, msg: Point2DArrayStamped, color: str, landmark: dict):
        # Need camera intrinsics first
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

        # Treat differently for rectangular vs cylindrical perception of landmarks based on number of corners
        if num_points < 8:
            min_x = min(xs)
            max_x = max(xs)
            x_sym = 0.5 * (min_x + max_x)
            shape = 'rect'
        else:
            x_sym = sum(xs) / float(num_points)
            shape = 'cyl'

        # Bearing in camera frame (we approximate as base frame bearing for this lab)
        theta_m = math.atan((self.cx - x_sym) / self.fx)

        cos_th = math.cos(theta_m)
        if abs(cos_th) < 1e-3:
            # too close to 90°, avoid numerical blow-up
            return

        d_m = self.landmark_height * self.fy / (height_pix * cos_th)

        var_d, var_theta = self.measurement_variance(d_m, theta_m)
        self.Q[0, 0] = var_d
        self.Q[1, 1] = var_theta

        lm_pos = landmark['position']

        timestamp = msg.header.stamp

        # First ever measurement: initialize time reference, skip update
        if not self.initialized:
            self.initialized = True
            self.system_time = timestamp
            self.log.info("EKF initialized with first measurement timestamp (state left at prior).")
            return

        dt = self.time_diff_sec(timestamp, self.system_time)
        if dt < 0.0:
            # late-arriving sample, discard
            self.log.warn("Received measurement with negative dt (late); discarding.")
            return

        if dt > 0.0 and self.last_vel is not None:
            v, w = self.last_vel
            self.prediction(v, w, dt)
        landmark_xy = (lm_pos["x"], lm_pos["y"])
        self.measurement_update(landmark_xy, d_m, theta_m)
        self.system_time = timestamp

        self.publish_ekf_pose(timestamp)

    def measurement_variance(self, d: float, theta: float):
        """
        Piecewise variance model from Lab 5:
        - base variances from CSV in "reliable region"
        - inflated by factor 3 otherwise
        Reliable region: 1 m <= d <= 10 m, |theta| <= 0.6 rad
        """
        outside_reliable = (d < 1.0) or (d > 10.0) or (abs(theta) > 0.6)
        factor = 3.0 if outside_reliable else 1.0
        var_d = factor * self.var_d_base
        var_theta = factor * self.var_theta_base
        return var_d, var_theta

    def measurement_update(self, landmark_xy, meas_range, meas_bearing):
        """
        Standard EKF range-bearing update to a known landmark.
        landmark_xy: (m_x, m_y) in map frame
        meas_range:  measured distance d_m
        meas_bearing: measured bearing theta_m
        """
        # if we don't yet know the camera transform, we cannot fuse properly
        if self.T_base_to_camera is None:
            return

        mx, my = landmark_xy

        # base state
        x = self.state[0, 0]
        y = self.state[1, 0]
        theta = self.state[2, 0]

        # camera pose in map frame: base pose + rotated offset
        tx, ty = self.cam_offset[0], self.cam_offset[1]
        x_cam = math.cos(theta) * tx - math.sin(theta) * ty + x
        y_cam = math.sin(theta) * tx + math.cos(theta) * ty + y
        theta_cam = theta + self.cam_yaw

        # expected measurement from camera frame
        dx = mx - x_cam
        dy = my - y_cam
        q = dx ** 2 + dy ** 2
        if q < 1e-9:
            return

        z1 = math.sqrt(q)
        z2 = self.unwrap(math.atan2(dy, dx) - theta_cam)

        # derivative of camera position wrt theta (how offset rotates)
        n_x = -math.sin(theta) * tx - math.cos(theta) * ty
        n_y =  math.cos(theta) * tx - math.sin(theta) * ty

        # Jacobian H with camera offset
        self.H[0, 0] = -dx / z1
        self.H[0, 1] = -dy / z1
        self.H[0, 2] = (-dx * n_x - dy * n_y) / z1

        self.H[1, 0] =  dy / q
        self.H[1, 1] = -dx / q
        self.H[1, 2] = (dy * n_x - dx * n_y) / q - 1.0

        # Innovation dz = z_meas - h(x)
        dz = np.array([
            [meas_range - z1],
            [self.unwrap(meas_bearing - z2)]
        ])

        # S = H P H^T + R (2x2)
        S = self.H @ self.Cov @ self.H.T + self.Q
        # K = P H^T S^{-1} (3x2)
        K = self.Cov @ self.H.T @ np.linalg.inv(S)

        self.state = self.state + K @ dz
        self.state[2, 0] = self.unwrap(self.state[2, 0])

        self.Cov = (self.I - K @ self.H) @ self.Cov

    def publish_ekf_pose(self, timestamp):
        odom_msg = Odometry()
        odom_msg.header.stamp = timestamp
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_footprint"

        odom_msg.pose.pose.position.x = float(self.state[0, 0])
        odom_msg.pose.pose.position.y = float(self.state[1, 0])
        odom_msg.pose.pose.orientation = self.to_quaternion(float(self.state[2, 0]))

        odom_msg.pose.covariance[0]  = float(self.Cov[0, 0])  # xx
        odom_msg.pose.covariance[1]  = float(self.Cov[0, 1])  # xy
        odom_msg.pose.covariance[5]  = float(self.Cov[0, 2])  # x-yaw
        odom_msg.pose.covariance[6]  = float(self.Cov[1, 0])  # yx
        odom_msg.pose.covariance[7]  = float(self.Cov[1, 1])  # yy
        odom_msg.pose.covariance[11] = float(self.Cov[1, 2])  # y-yaw
        odom_msg.pose.covariance[30] = float(self.Cov[2, 0])  # yaw-x
        odom_msg.pose.covariance[31] = float(self.Cov[2, 1])  # yaw-y
        odom_msg.pose.covariance[35] = float(self.Cov[2, 2])  # yaw-yaw

        self.odom_pub.publish(odom_msg)

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
