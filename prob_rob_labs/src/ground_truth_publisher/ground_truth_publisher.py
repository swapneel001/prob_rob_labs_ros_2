import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import PoseStamped, TwistStamped
from rclpy.time import Time

heartbeat_period = 0.1  # control rate / "delay between successive pushes"

class GroundTruthPublisher(Node):

    def __init__(self):
        super().__init__('ground_truth_publisher')
        self.log = self.get_logger()
        self.timer = self.create_timer(heartbeat_period, self.heartbeat)
        self.gazebo_link_states = self.create_subscription(LinkStates, '/gazebo/link_states', self.link_states_callback, 10 )
        self.pose_publisher = self.create_publisher(PoseStamped, '/tb3/ground_truth/pose', 10)
        self.twist_publisher = self.create_publisher(TwistStamped, '/tb3/ground_truth/twist', 10)
        self.waffle_pi_footprint = 'waffle_pi::base_footprint' 


    def link_states_callback(self, msg):
        try:
            idx = msg.name.index(self.waffle_pi_footprint)
            self.current_pose = msg.pose[idx]
            self.current_twist = msg.twist[idx]
        except ValueError:
            self.log.error(f"Link '{self.waffle_pi_footprint}' not found in /gazebo/link_states")
            self.current_pose = None
            self.current_twist = None
        
        robot_pose = msg.pose[idx]
        robot_twist = msg.twist[idx]
        
    def spin(self):
        rclpy.spin(self)


def main():
    rclpy.init()
    ground_truth_publisher = GroundTruthPublisher()
    ground_truth_publisher.spin()
    ground_truth_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
