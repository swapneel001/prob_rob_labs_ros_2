#!/usr/bin/env python3
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
        self.FIXED_FRAME = 'odom'
        self.CHILD_FRAME = 'base_footprint'
        self.SYNC_SLOP = 0.05     
        self.A_V = 0.921          
        self.G_V = 0.9            
        self.A_W = 0.701           
        self.G_W = 0.9       

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

        # state x = [x, y, theta, v, omega]^T
        self.x = np.zeros((5, 1), dtype=float)
        self.Sigma = np.diag([1e-6, 1e-6, 1e-6, 1e-3, 1e-3]).astype(float)

        # Jacobian G (refreshed each step)
        self.J = np.eye(5, dtype=float)
        self.B = np.zeros((5, 2), dtype=float)
        self.B[3, 0] = (1.0 - self.A_V)
        self.B[4, 1] = (1.0 - self.A_W)

        self.Sigma_u = np.array([[0.05, 0.0],
                                 [0.0,  0.05]], dtype=float)

        self.C = np.array([
            [0.0, 0.0, 0.0, 1.0 / self.R_WHEEL,  self.ROBOT_RADIUS / self.R_WHEEL],
            [0.0, 0.0, 0.0, 1.0 / self.R_WHEEL, -self.ROBOT_RADIUS / self.R_WHEEL],
            [0.0, 0.0, 0.0, 0.0,                 1.0]
        ], dtype=float)

        # Measurement noise sigma_z (encoders + gyro)
        # encoders variance guess ~5e-4, gyro from /imu ~4e-8
        self.Sigma_z = np.array([
            [5.0e-4, 0.0,    0.0],
            [0.0,    5.0e-4, 0.0],
            [0.0,    0.0,    4.0e-08]
        ], dtype=float)

        self.prev_time = None
        self.get_logger().info('EKF Odom Node (prediction + measurement) running.')

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
        except Exception:
            self.get_logger().warn('joint_states missing wheel_left_joint / wheel_right_joint')
            return
        wg = float(imu_msg.angular_velocity.z)

        z = np.array([[wr], [wl], [wg]], dtype=float)

        # EKF: prediction then measurement update
        self.predict(dt)
        self.update(z)

        # Publish odometry at the measurement time
        self.publish_odom(imu_msg.header.stamp)

    def predict(self, dt: float):
        x, y, th, v, w = self.x.flatten()
        u_v, u_w = self.last_cmd_lin, self.last_cmd_ang

        # State prediction
        x_next  = x  + v * math.cos(th) * dt
        y_next  = y  + v * math.sin(th) * dt
        th_next = th + w * dt
        v_next  = self.A_V * v + (1.0 - self.A_V) * self.G_V * u_v
        w_next  = self.A_W * w + (1.0 - self.A_W) * self.G_W * u_w
        self.x  = np.array([[x_next], [y_next], [th_next], [v_next], [w_next]], dtype=float)

        # Jacobian J
        self.J[:] = np.eye(5, dtype=float)
        self.J[0, 2] = -v * math.sin(th) * dt   
        self.J[0, 3] =  math.cos(th) * dt       
        self.J[1, 2] =  v * math.cos(th) * dt   
        self.J[1, 3] =  math.sin(th) * dt       
        self.J[2, 4] =  dt                      
        self.J[3, 3] =  self.A_V                
        self.J[4, 4] =  self.A_W                
        # Covariance propagation
        self.Sigma = self.G @ self.Sigma @ self.G.T + (self.B @ self.Sigma_u @ self.B.T)

    def update(self, z: np.ndarray):
        y = z - (self.C @ self.x)
        S = self.C @ self.Sigma @ self.C.T + self.Sigma_z
        K = self.Sigma @ self.C.T @ np.linalg.inv(S)
        # State & covariance update
        I = np.eye(5)
        self.Sigma = (I - K @ self.C) @ self.Sigma
        self.x[2, 0] = self.unwrap(self.x[2, 0])

    def publish_odom(self, stamp):
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.FIXED_FRAME
        odom.child_frame_id = self.CHILD_FRAME

        x, y, th, v, w = self.x.flatten()
        odom.pose.pose.position.x = float(x)
        odom.pose.pose.position.y = float(y)
        odom.pose.pose.orientation = yaw_to_quaternion(float(th))
        odom.twist.twist.linear.x  = float(v)
        odom.twist.twist.angular.z = float(w)

        pose_cov = [0.0] * 36
        pose_cov[0]  = float(self.Sigma[0, 0])  # var(x)
        pose_cov[7]  = float(self.Sigma[1, 1])  # var(y)
        pose_cov[35] = float(self.Sigma[2, 2])  # var(theta)
        pose_cov[1]  = float(self.Sigma[0, 1]); pose_cov[6]  = float(self.Sigma[1, 0])  
        pose_cov[5]  = float(self.Sigma[0, 2]); pose_cov[11] = float(self.Sigma[1, 2]) 
        pose_cov[30] = float(self.Sigma[2, 0]); pose_cov[31] = float(self.Sigma[2, 1]) 
        pose_cov[14] = pose_cov[21] = pose_cov[28] = 1e6
        odom.pose.covariance = pose_cov

        twist_cov = [0.0] * 36
        twist_cov[0]  = float(self.Sigma[3, 3])  # var(v)
        twist_cov[35] = float(self.Sigma[4, 4])  # var(ω)
        odom.twist.covariance = twist_cov

        self.odom_pub.publish(odom)

    def unwrap(self, angle):
        while angle >  math.pi: angle -= 2*math.pi
        while angle < -math.pi: angle += 2*math.pi
        return angle


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
