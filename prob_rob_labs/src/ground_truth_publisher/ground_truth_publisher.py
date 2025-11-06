#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import PoseStamped, TwistStamped

class GroundTruthPublisher(Node):
    def __init__(self):
        super().__init__('ground_truth_publisher')

        self.declare_parameter('reference_frame', 'odom')
        self.reference_frame = self.get_parameter('reference_frame').get_parameter_value().string_value

        self.pose_pub  = self.create_publisher(PoseStamped,  '/tb3/ground_truth/pose', 10)
        self.twist_pub = self.create_publisher(TwistStamped, '/tb3/ground_truth/twist', 10)

        self.sub = self.create_subscription(
            LinkStates,
            '/gazebo/link_states',
            self.link_states_callback,
            10
        )

        self.link_name = 'waffle_pi::base_footprint'

    def link_states_callback(self, msg: LinkStates):
        ind = msg.name.index(self.link_name) 

        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = self.reference_frame
        pose_msg.pose = msg.pose[ind]
        self.pose_pub.publish(pose_msg)

        twist_msg = TwistStamped()
        twist_msg.header.stamp = stamp
        twist_msg.header.frame_id = self.reference_frame
        twist_msg.twist = msg.twist[ind]
        self.twist_pub.publish(twist_msg)

def main():
    rclpy.init()
    node = GroundTruthPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
