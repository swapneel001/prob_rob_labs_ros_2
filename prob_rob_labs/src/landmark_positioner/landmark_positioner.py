#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node

from prob_rob_msgs.msg import Point2DArrayStamped
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import PointStamped
from gazebo_msgs.msg import LinkStates  

import csv 
import os

heartbeat_period = 0.1


class LandmarkPositioner(Node):

    COLOR_TO_LINK ={
        'red'  : 'landmark_1::link',
        'green': 'landmark_2::link',
        'yellow': 'landmark_3::link',
        'magenta': 'landmark_4::link',
        'cyan' : 'landmark_5::link'
    }

    CAMERA_LINK = 'waffle_pi::camera_link'

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

        self.latest_link_states = None
        self.landmark_link_name = self.COLOR_TO_LINK.get(self.landmark_color, None)
        if self.landmark_link_name is None:
            self.log.error(f'Unknown landmark color: {self.landmark_color}')
            return

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

        self.link_states_sub = self.create_subscription(
            LinkStates,
            '/gazebo/link_states',
            self.link_states_callback,
            10
        )


        topic_name = f'/vision_{self.landmark_color}/measurement'
        self.meas_pub = self.create_publisher(PointStamped, topic_name, 10)
        self.log.info(f'Publishing landmark measurements on {topic_name}')

        err_topic = f'/vision_{self.landmark_color}/error'
        self.err_pub = self.create_publisher(PointStamped, err_topic, 10)
        self.log.info(f'Publishing landmark errors on {err_topic}')



    def heartbeat(self):
        self.log.info('no landmark seeen, heartbeat')

    def camera_info_callback(self, msg: CameraInfo):
        # K = [fx 0 cx ; 0 fy cy ; 0 0 1]
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]
    
    def link_states_callback(self, msg: LinkStates):
        self.latest_link_states = msg

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


        MAX_BEARING = 0.75
        if abs(theta) > MAX_BEARING:
            return
        #distance 
        cos_th = math.cos(theta)
        if abs(cos_th) < 1e-3:
            return  # avoid div by zero
        d = self.landmark_height * self.fy / (height_pix * cos_th)

        out = PointStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'waffle_pi::camera_link'
        out.point.x = float(d)
        out.point.y = float(theta)
        out.point.z = 0.0
        self.meas_pub.publish(out)

        shape = 'rect' if num_points < 8 else 'cyl'
        self.log.info(f'{shape}: h_pix={height_pix:.1f}, x_sym={x_sym:.1f} -> d={d:.3f} m, th={theta:.3f} rad')

        self.publish_error(stamp = out.header.stamp, measured_d = d, measured_theta = theta)

    def publish_error(self, stamp, measured_d, measured_theta):
        if self.latest_link_states is None:
            return

        if self.landmark_link_name not in self.latest_link_states.name:
            return

        ls = self.latest_link_states
        if self.CAMERA_LINK not in ls.name:
            return
        cam_idx = ls.name.index(self.CAMERA_LINK)
        cam_pose = ls.pose[cam_idx]
        cam_x = cam_pose.position.x
        cam_y = cam_pose.position.y
        cam_q = cam_pose.orientation
        cam_yaw = self.quaternion_to_yaw(cam_q.x, cam_q.y, cam_q.z, cam_q.w)
        if self.landmark_link_name not in ls.name:
            return
        lm_idx = ls.name.index(self.landmark_link_name)
        lm_pose = ls.pose[lm_idx]
        lm_x = lm_pose.position.x
        lm_y = lm_pose.position.y

        dx = lm_x - cam_x
        dy = lm_y - cam_y

        d_true = math.hypot(dx, dy)
        ang_w = math.atan2(dy, dx)
        theta_true = self.normalize_angle(ang_w - cam_yaw)
        err_d = measured_d - d_true
        err_theta = self.normalize_angle(measured_theta - theta_true)


        err_msg = PointStamped()
        err_msg.header.stamp = stamp
        err_msg.header.frame_id = self.CAMERA_LINK
        err_msg.point.x = float(err_d)
        err_msg.point.y = float(err_theta)
        err_msg.point.z = 0.0
        self.err_pub.publish(err_msg)

        self.log.info(f'Error: d_err={err_d:.3f} m, th_err={err_theta:.3f} rad')


        log_path = os.path.expanduser('~/measurement_errors.csv')
        file_exists = os.path.isfile(log_path)
        t = stamp.sec + stamp.nanosec * 1e-9
        with open(log_path, 'a', newline='') as f:
            writer = csv.writer(f)
            if not file_exists:
                writer.writerow(['timestamp',
                                 'd_meas', 'theta_meas',
                                 'd_true', 'theta_true',
                                 'd_err', 'theta_err'])
            writer.writerow([t,
                             measured_d, measured_theta,
                             d_true, theta_true,
                             err_d, err_theta])

    @staticmethod
    def normalize_angle(angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle
    
    @staticmethod
    def quaternion_to_yaw(x, y, z, w):
        # yaw (z-axis rotation)
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw

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
