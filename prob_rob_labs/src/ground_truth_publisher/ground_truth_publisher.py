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

        self.landmark_publishers = {
            'landmark1::link' : self.create_publisher(PoseStamped, '/landmark1/ground_truth/pose', 10),
            'landmark2::link' : self.create_publisher(PoseStamped, '/landmark2/ground_truth/pose', 10),
            'landmark3::link' : self.create_publisher(PoseStamped, '/landmark3/ground_truth/pose', 10),
            'landmark4::link' : self.create_publisher(PoseStamped, '/landmark4/ground_truth/pose', 10),
            'landmark5::link' : self.create_publisher(PoseStamped, '/landmark5/ground_truth/pose', 10)
        }

        self.camera_link = 'waffle_pi::camera_link'
        self.camera_pose_pub = self.create_publisher(PoseStamped, '/tb3/camera/ground_truth/pose', 10)  
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
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        twist_msg.header.frame_id = self.reference_frame
        twist_msg.twist = msg.twist[ind]
        self.twist_pub.publish(twist_msg)

        for landmark_link, publisher in self.landmark_publishers.items():
            if landmark_link in msg.name:
                lm_ind = msg.name.index(landmark_link)
                lm_pose_msg = PoseStamped()
                lm_pose_msg.header.stamp = self.get_clock().now().to_msg()
                lm_pose_msg.header.frame_id = self.reference_frame
                lm_pose_msg.pose = msg.pose[lm_ind]
                publisher.publish(lm_pose_msg)

        if self.camera_link in msg.name:
            ci = msg.name.index(self.camera_link)
            camera_pose_msg = PoseStamped()
            camera_pose_msg.header.stamp = self.get_clock().now().to_msg()
            camera_pose_msg.header.frame_id = self.reference_frame
            camera_pose_msg.pose = msg.pose[ci]
            self.camera_pose_pub.publish(camera_pose_msg)

def main():
    rclpy.init()
    node = GroundTruthPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
