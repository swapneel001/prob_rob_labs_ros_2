import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import PoseStamped, TwistStamped
from rclpy.time import Time

class GroundTruthPublisher(Node):
    def __init__(self):
        super().__init__('ground_truth_publisher')
        
        # 订阅包含所有链接状态的话题
        self.subscription = self.create_subscription(
            LinkStates,
            '/gazebo/link_states',
            self.link_states_callback,
            10)
        
        # 创建两个发布者，用于发布提取出的位姿和速度
        self.pose_publisher = self.create_publisher(PoseStamped, '/tb3/ground_truth/pose', 10)
        self.twist_publisher = self.create_publisher(TwistStamped, '/tb3/ground_truth/twist', 10)
        
        # 我们要找的机器人本体链接的名称
        self.robot_link_name = 'waffle_pi::base_footprint'
        self.get_logger().info(f"Publishing ground truth for link: {self.robot_link_name}")

    def link_states_callback(self, msg):
        try:
            # 在消息的名称列表中找到我们想要的链接的索引
            index = msg.name.index(self.robot_link_name)
        except ValueError:
            # 如果没找到，就直接返回
            return

        # 根据索引提取对应的位姿和速度
        robot_pose = msg.pose[index]
        robot_twist = msg.twist[index]
        
        current_time = self.get_clock().now().to_msg()
        
        # 创建并填充 PoseStamped 消息
        pose_stamped_msg = PoseStamped()
        pose_stamped_msg.header.stamp = current_time
        pose_stamped_msg.header.frame_id = 'odom'  # 根据实验要求，参考系设为 odom
        pose_stamped_msg.pose = robot_pose
        
        # 创建并填充 TwistStamped 消息
        twist_stamped_msg = TwistStamped()
        twist_stamped_msg.header.stamp = current_time
        twist_stamped_msg.header.frame_id = 'odom'
        twist_stamped_msg.twist = robot_twist
        
        # 发布消息
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