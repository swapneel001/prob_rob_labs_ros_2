#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Empty
from geometry_msgs.msg import Twist

heartbeat_period = 0.1

class MoveThroughDoorBayesian(Node):
    def __init__(self):
        super().__init__('move_through_door_bayesian')
        self.log = self.get_logger()
        self.declare_parameter('robot_speed', 1.0)
        self.pub_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_door_open = self.create_publisher(Empty, '/door_open', 10)
        self.sub_feature_mean = self.create_subscription(Float64, '/feature_mean', self.check_feature_mean, 10)
        self.timer = self.create_timer(heartbeat_period, self.heartbeat)

        self.robot_speed = self.get_parameter('robot_speed').get_parameter_value().double_value
        self.feature_mean_value = 500.0

        self.measurement_threshold = 235.0
        self.belief_threshold = 0.99
        self.belief_door_is_open = 0.5

        self.p_zo_xo = 0.782
        self.p_zc_xo = 0.218
        self.p_zo_xc = 0.017
        self.p_zc_xc = 0.983

        self.p_open_given_open_cmd = 0.65
        self.state = 'OPENING'
        self.last_open_cmd_time = 0.0

    def heartbeat(self):
        now = time.time()

        if self.state == 'OPENING':
            if now - self.last_open_cmd_time >= 1.0:
                self.pub_door_open.publish(Empty())
                self.predict_open_cmd()
                self.last_open_cmd_time = now
            if self.belief_door_is_open >= self.belief_threshold:
                self.state = 'MOVING'
        elif self.state == 'MOVING':
            self.move_robot()

    def predict_open_cmd(self):
        p_open_prev = self.belief_door_is_open
        p_closed_prev = 1.0 - p_open_prev
        p_open = p_open_prev + self.p_open_given_open_cmd * p_closed_prev
        self.belief_door_is_open = min(max(p_open, 0.0), 1.0)

    def check_feature_mean(self, msg: Float64):
        self.feature_mean_value = msg.data
        z_open = (self.feature_mean_value < self.measurement_threshold)
        if z_open:
            like_open = self.p_zo_xo
            like_closed = self.p_zo_xc
        else:
            like_open = self.p_zc_xo
            like_closed = self.p_zc_xc

        prior_open = self.belief_door_is_open
        prior_closed = 1.0 - prior_open

        bel_open_unnorm = like_open * prior_open
        bel_closed_unnorm = like_closed * prior_closed
        norm = bel_open_unnorm + bel_closed_unnorm
        if norm > 0.0:
            self.belief_door_is_open = bel_open_unnorm / norm

    def move_robot(self):
        vel = Twist()
        vel.linear.x = self.robot_speed
        self.pub_vel.publish(vel)

    def spin(self):
        rclpy.spin(self)

def main():
    rclpy.init()
    node = MoveThroughDoorBayesian()
    node.spin()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
