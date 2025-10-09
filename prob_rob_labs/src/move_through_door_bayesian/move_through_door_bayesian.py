#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist

heartbeat_period = 0.1

class MoveThroughDoorBayesian(Node):

    def __init__(self):
        super().__init__('move_through_door_bayesian')
        self.log = self.get_logger()
        self.declare_parameter('robot_speed',1.0)
        self.pub_torque = self.create_publisher(Float64,'/hinged_glass_door/torque',1)
        self.pub_vel = self.create_publisher(Twist,'/cmd_vel',10)
        self.timer = self.create_timer(heartbeat_period, self.heartbeat)
        self.robot_speed = self.get_parameter('robot_speed').get_parameter_value().double_value
        self.sub_feature_mean = self.create_subscription(Float64,'/feature_mean',self.check_feature_mean,1)

        # kept variable names from your control node
        self.feature_mean_value = 500
        self.torque = 5.0
        self.heartbeat_counter = 0
        self.moving_started = False
        self.stop_started = False
        self.close_started = False

        # added bayesian fields
        self.measurement_threshold = 235.0
        self.belief_threshold = 0.999
        self.belief_door_is_open = 0.5
        self.p_zo_xo = 0.782
        self.p_zc_xo = 0.218
        self.p_zo_xc = 0.017
        self.p_zc_xc = 0.983

        # state machine
        self.state = 'OPENING'
        self.state_start = time.time()

    def heartbeat(self):
        now = time.time()

        if self.state == 'OPENING':
            self.open_door()
            self.log.info('Measuring door open probability')
            self.log.info(f'Bel(x=open)={self.belief_door_is_open:.3f}')
            if self.belief_door_is_open >= self.belief_threshold:
                self.moving_started = True
                self.state = 'MOVING'
                self.state_start = now
                self.move_robot()

        elif self.state == 'MOVING':
            self.log.info('Moving through door')
            self.move_robot()
            if now - self.state_start >= 5.0:
                self.stop_robot()
                self.stop_started = True
                self.state = 'STOPPED'
                self.state_start = now

        elif self.state == 'STOPPED':
            self.log.info('Stopped. Closing door.')
            self.state = 'CLOSING'
            self.close_door()
            

    def check_feature_mean(self, f):
        self.feature_mean_value = f.data
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
        normalizer = bel_open_unnorm + bel_closed_unnorm
        if normalizer > 0.0:
            self.belief_door_is_open = bel_open_unnorm / normalizer
            
    def open_door(self):
        self.log.info('Opening door')
        self.pub_torque.publish(Float64(data=self.torque))

    def close_door(self):
        self.pub_torque.publish(Float64(data=-self.torque))

    def move_robot(self):
        vel_msg = Twist()
        vel_msg.linear.x = self.robot_speed
        self.pub_vel.publish(vel_msg)

    def stop_robot(self):
        vel_msg = Twist()
        vel_msg.linear.x = 0.0
        self.pub_vel.publish(vel_msg)

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
