#!/usr/bin/env python3
import os
import csv

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from message_filters import Subscriber, ApproximateTimeSynchronizer

heartbeat_period = 0.1


class ErrorReader(Node):

    def __init__(self):
        super().__init__('error_reader')
        self.log = self.get_logger()
        self.timer = self.create_timer(heartbeat_period, self.heartbeat)
        self.declare_parameter('landmark_color', 'cyan')
        self.declare_parameter('csv_path', '~/measurement_errors.csv')
        self.landmark_color = self.get_parameter('landmark_color').get_parameter_value().string_value
        raw_path = self.get_parameter('csv_path').get_parameter_value().string_value
        self.csv_path = os.path.expanduser(raw_path)
        meas_topic = f'/vision_{self.landmark_color}/measurement'
        err_topic = f'/vision_{self.landmark_color}/measurement_error'
        self.meas_sub = Subscriber(self, PointStamped, meas_topic)
        self.err_sub = Subscriber(self, PointStamped, err_topic)
        self.ts = ApproximateTimeSynchronizer(
            [self.meas_sub, self.err_sub],
            queue_size=50,
            slop=0.05,
            allow_headerless=False
        )
        self.ts.registerCallback(self.synced_cb)

        self.log.info(f'Sync-logging {self.landmark_color} to {self.csv_path}')
        self.log.info(f'Subscribed to {meas_topic} and {err_topic}')

    def heartbeat(self):
        self.log.info('heartbeat')

    def synced_cb(self, meas_msg: PointStamped, err_msg: PointStamped):
        self.log.info('Received synced measurement and error messages')
        d_meas = meas_msg.point.x
        th_meas = meas_msg.point.y
        d_err = err_msg.point.x
        th_err = err_msg.point.y

        # use the measurement timestamp
        stamp = meas_msg.header.stamp
        t = stamp.sec + stamp.nanosec * 1e-9

        # write to csv
        file_exists = os.path.isfile(self.csv_path)
        with open(self.csv_path, 'a', newline='') as f:
            writer = csv.writer(f)
            if not file_exists:
                writer.writerow(['timestamp', 'd_meas', 'theta_meas', 'd_err', 'theta_err'])
            writer.writerow([t, d_meas, th_meas, d_err, th_err])

        self.log.info(
            f'logged: t={t:.3f}, d={d_meas:.3f}, th={th_meas:.3f}, '
            f'd_err={d_err:.3f}, th_err={th_err:.3f}'
        )

    def spin(self):
        rclpy.spin(self)


def main():
    rclpy.init()
    node = ErrorReader()
    node.spin()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
