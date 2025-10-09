import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist

class MoveThroughDoorBayesian(Node):
    def __init__(self):
        super().__init__('move_through_door_bayesian')
        self.subscription = self.create_subscription(
            Float64, '/feature_mean', self.feature_mean_callback, 10)
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.control_loop)

      
        self.prob_z_open_given_x_open = 0.782   # P(z=open | x=open)
        self.prob_z_closed_given_x_open = 0.218  # P(z=closed | x=open)
        self.prob_z_open_given_x_closed = 0.017  # P(z=open | x=closed)
        self.prob_z_closed_given_x_closed = 0.983# P(z=closed | x=closed)

        
        self.belief_door_is_open = 0.5
        
        
        self.measurement_threshold = 235.0
        self.belief_threshold = 0.999      # [cite: 273]
        self.state = 'WAITING_FOR_DOOR'

    def feature_mean_callback(self, msg):
   
        measurement_is_open = (msg.data > self.measurement_threshold)
        
   
        if measurement_is_open:
            likelihood_open = self.prob_z_open_given_x_open
            likelihood_closed = self.prob_z_open_given_x_closed
        else:
            likelihood_open = self.prob_z_closed_given_x_open
            likelihood_closed = self.prob_z_closed_given_x_closed

   
        prior_belief_open = self.belief_door_is_open
        prior_belief_closed = 1.0 - self.belief_door_is_open
        
        
        bel_open_unnorm = likelihood_open * prior_belief_open
        bel_closed_unnorm = likelihood_closed * prior_belief_closed
        

        normalizer = bel_open_unnorm + bel_closed_unnorm
        if normalizer > 0.0:
            self.belief_door_is_open = bel_open_unnorm / normalizer
      
        self.get_logger().info(f'Belief P(open|z) = {self.belief_door_is_open:.6f}')

    def control_loop(self):
       
        if self.state == 'WAITING_FOR_DOOR':
            if self.belief_door_is_open > self.belief_threshold:
                self.get_logger().info('High confidence door is OPEN! Moving forward.')
                self.state = 'MOVING_FORWARD'
        
        elif self.state == 'MOVING_FORWARD':
            move_cmd = Twist()
            move_cmd.linear.x = 0.5
            self.publisher_.publish(move_cmd)

def main(args=None):
    rclpy.init(args=args)
    node = MoveThroughDoorBayesian()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()