import math
import numpy as np
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, Quaternion
from sensor_msgs.msg import Imu, JointState
from nav_msgs.msg import Odometry
from message_filters import Subscriber, ApproximateTimeSynchronizer
from transforms3d.euler import euler2quat


def yaw_to_quaternion(yaw: float) -> Quaternion:
    qx, qy, qz, qw = euler2quat(0.0, 0.0, yaw)
    return Quaternion(x=qx, y=qy, z=qz, w=qw)


class EKFOdom(Node):
    def __init__(self):
        super().__init__('ekf_odom')

        self.R_WHEEL = 0.033      
        self.W_SEP = 0.1435      
        self.ROBOT_RADIUS = 0.5 * self.W_SEP
        self.SYNC_SLOP = 0.05    
        self.FIXED_FRAME = 'odom'
        self.CHILD_FRAME = 'base_footprint'

        self.last_cmd_lin = 0.0
        self.last_cmd_ang = 0.0
        self.create_subscription(Twist, '/cmd_vel', self.cmd_cb, 10)

        self.odom_pub = self.create_publisher(Odometry, '/ekf_odom', 10)

        self.imu_sub = Subscriber(self, Imu, '/imu')
        self.jnt_sub = Subscriber(self, JointState, '/joint_states')

        self.sync = ApproximateTimeSynchronizer(
            [self.imu_sub, self.jnt_sub],
            queue_size=50,
            slop=self.SYNC_SLOP,
            allow_headerless=False
        )
        self.sync.registerCallback(self.synced_cb)

        self.x = np.zeros((5, 1), dtype=float)   # [x, y, theta, v, omega]
        self.P = np.diag([1e-6, 1e-6, 1e-6, 1e-3, 1e-3])
        self.prev_time = None

        self.get_logger().info('EKF Odom Node (Increment 1) running.')

    def cmd_cb(self, msg: Twist):
        self.last_cmd_lin = float(msg.linear.x)
        self.last_cmd_ang = float(msg.angular.z)

    def synced_cb(self, imu_msg: Imu, jnt_msg: JointState):
        t = imu_msg.header.stamp.sec + 1e-9 * imu_msg.header.stamp.nanosec
        if self.prev_time is None:
            self.prev_time = t
            return
        dt = max(1e-6, t - self.prev_time)
        self.prev_time = t

        try:
            name_to_idx = {n: i for i, n in enumerate(jnt_msg.name)}
            wl = jnt_msg.velocity[name_to_idx['wheel_left_joint']]
            wr = jnt_msg.velocity[name_to_idx['wheel_right_joint']]
        except KeyError:
            self.get_logger().warn('Could not find wheel joint names in joint_states.')
            return

        v_meas = self.R_WHEEL * 0.5 * (wr + wl)
        w_meas = self.R_WHEEL * 0.5 / self.ROBOT_RADIUS * (wr - wl)
        odom = Odometry()
        odom.header.stamp = imu_msg.header.stamp
        odom.header.frame_id = self.FIXED_FRAME
        odom.child_frame_id = self.CHILD_FRAME

        theta = float(self.x[2, 0])  # 0 until we start integrating
        odom.pose.pose.orientation = yaw_to_quaternion(theta)

       # Pose covariance (row-major 6x6: x y z r p y)
        pose_cov = [0.0] * 36
        pose_cov[0]  = 1e-6   # var(x)
        pose_cov[7]  = 1e-6   # var(y)
        pose_cov[14] = 1e6    # var(z)  (unused in planar)
        pose_cov[21] = 1e6    # var(roll)
        pose_cov[28] = 1e6    # var(pitch)
        pose_cov[35] = 1e-2   # var(yaw)
        odom.pose.covariance = pose_cov

        odom.twist.twist.linear.x = v_meas
        odom.twist.twist.angular.z = w_meas
        # Twist covariance (row-major 6x6: vx vy vz wx wy wz)
        twist_cov = [0.0] * 36
        twist_cov[0]  = 1e-3  # var(vx)
        twist_cov[35] = 1e-3  # var(wz)
        odom.twist.covariance = twist_cov

        self.odom_pub.publish(odom)

        if int(t * 10) % 10 == 0:
            self.get_logger().info(
                f'dt={dt:.3f}s | v_meas={v_meas:.3f} m/s | w_meas={w_meas:.3f} rad/s'
            )


def main():
    rclpy.init()
    node = EKFOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
