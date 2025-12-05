import rclpy
from rclpy.node import Node
import math
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from message_filters import Subscriber, ApproximateTimeSynchronizer


def quat_to_yaw(q: Quaternion) -> float:
    x, y, z, w = q.x, q.y, q.z, q.w
    # yaw (z-axis rotation)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def wrap_to_pi(angle: float) -> float:
    a = (angle + math.pi) % (2.0 * math.pi)
    if a < 0.0:
        a += 2.0 * math.pi
    return a - math.pi


heartbeat_period = 0.1

class LandmarkEkfError(Node):

    def __init__(self):
        super().__init__('landmark_ekf_error')

        self.gt_sub = Subscriber(self, PoseStamped, '/tb3/ground_truth/pose')
        self.ekf_sub = Subscriber(self, Odometry, '/ekf_pose')

        self.sync = ApproximateTimeSynchronizer(
            [self.gt_sub, self.ekf_sub],
            queue_size=50,
            slop=0.05,
            allow_headerless=False
        )
        self.sync.registerCallback(self.synced_cb)
        self.pos_err_pub = self.create_publisher(Float64, '/ekf_error/position', 10)
        self.yaw_err_pub = self.create_publisher(Float64, '/ekf_error/yaw_abs', 10)

        self.get_logger().info('ekf_error_plotter running.')

    def synced_cb(self, gt_pose_msg: PoseStamped, ekf_odom_msg: Odometry):
        gx = gt_pose_msg.pose.position.x
        gy = gt_pose_msg.pose.position.y
        ex = ekf_odom_msg.pose.pose.position.x
        ey = ekf_odom_msg.pose.pose.position.y

        pos_err = math.hypot(ex - gx, ey - gy)
        yaw_gt = quat_to_yaw(gt_pose_msg.pose.orientation)
        yaw_est = quat_to_yaw(ekf_odom_msg.pose.pose.orientation)
        yaw_diff = wrap_to_pi(yaw_est - yaw_gt)
        yaw_err_abs = abs(yaw_diff)

        self.pos_err_pub.publish(Float64(data=pos_err))
        self.yaw_err_pub.publish(Float64(data=yaw_err_abs))

    def heartbeat(self):
        self.log.info('heartbeat')

    def spin(self):
        rclpy.spin(self)


def main():
    rclpy.init()
    ekf_error_plotter = LandmarkEkfError()
    ekf_error_plotter.spin()
    ekf_error_plotter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
