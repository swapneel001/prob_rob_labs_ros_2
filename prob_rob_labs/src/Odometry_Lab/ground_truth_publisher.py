import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import PoseStamped, TwistStamped
from rclpy.time import Time

class GroundTruthPublisher(Node):
    def __init__(self):
        super().__init__('ground_truth_publisher')
        
        
        self.subscription = self.create_subscription(
            LinkStates,
            '/gazebo/link_states',
            self.link_states_callback,
            10)
        
        
        self.pose_publisher = self.create_publisher(PoseStamped, '/tb3/ground_truth/pose', 10)
        self.twist_publisher = self.create_publisher(TwistStamped, '/tb3/ground_truth/twist', 10)
        
        
        self.robot_link_name = 'waffle_pi::base_footprint'
        self.get_logger().info(f"Publishing ground truth for link: {self.robot_link_name}")

    def link_states_callback(self, msg):
        try:
            
            index = msg.name.index(self.robot_link_name)
        except ValueError:
            
            return

        
        robot_pose = msg.pose[index]
        robot_twist = msg.twist[index]
        
        current_time = self.get_clock().now().to_msg()
        
        
        pose_stamped_msg = PoseStamped()
        pose_stamped_msg.header.stamp = current_time
        pose_stamped_msg.header.frame_id = 'odom'  
        pose_stamped_msg.pose = robot_pose
        
        
        twist_stamped_msg = TwistStamped()
        twist_stamped_msg.header.stamp = current_time
        twist_stamped_msg.header.frame_id = 'odom'
        twist_stamped_msg.twist = robot_twist
        
        
        self.pose_publisher.publish(pose_stamped_msg)
        self.twist_publisher.publish(twist_stamped_msg)

def main(args=None):
    rclpy.init(args=args)
    node = GroundTruthPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()