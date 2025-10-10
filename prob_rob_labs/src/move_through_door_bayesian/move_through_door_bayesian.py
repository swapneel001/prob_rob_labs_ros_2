#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Empty
from geometry_msgs.msg import Twist

heartbeat_period = 0.5  # control rate / "delay between successive pushes"

class MoveThroughDoorBayesian(Node):

    def __init__(self):
        super().__init__('move_through_door_bayesian')
        self.log = self.get_logger()
        self.declare_parameter('robot_speed', 1.0)
        self.pub_torque = self.create_publisher(Float64, '/hinged_glass_door/torque', 1)
        self.pub_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_door_open = self.create_publisher(Empty, '/door_open', 1)
        self.sub_feature_mean = self.create_subscription(Float64, '/feature_mean', self.check_feature_mean, 10)
        self.timer = self.create_timer(heartbeat_period, self.heartbeat)
        self.robot_speed = self.get_parameter('robot_speed').get_parameter_value().double_value

        self.feature_mean_value = 500.0
        self.torque = 5.0
        self.moving_started = False
        self.stop_started = False
        self.close_started = False

        self.measurement_threshold = 235.0
        self.belief_threshold = 0.999
        self.belief_door_is_open = 0.5
        self.p_zo_xo = 0.782
        self.p_zc_xo = 0.218
        self.p_zo_xc = 0.017
        self.p_zc_xc = 0.983

        self.p_open_given_open_cmd = 1.0
        self.p_open_given_closed_cmd = 0.65
        self.p_open_given_open_noop = 1.0
        self.p_open_given_closed_noop = 0.0

        self.state = 'OPENING'
        self.state_start = time.time()
        self.move_duration_sec = 5.0
        self.close_duration_sec = 3.0

        self.last_open_cmd_time = -1e9
        self.open_cmd_cooldown = 1.0
        self.last_action = 'NOOP'

        self.meas_seq = 0
        self.seq_at_last_push = -1

    def heartbeat(self):
        now = time.time()
        self.log.info(f'STATE={self.state}')

        if self.state == 'OPENING':
            can_push = (now - self.last_open_cmd_time >= self.open_cmd_cooldown) and (self.meas_seq > self.seq_at_last_push)

            if can_push:
                self.pub_door_open.publish(Empty())
                self.last_open_cmd_time = now
                self.seq_at_last_push = self.meas_seq
                self._predict(use_command=True)
                self.last_action = 'OPEN_CMD'
            else:
                self._predict(use_command=False)
                self.last_action = 'NOOP'

            self._update_from_latest_measurement()

            self.log.info(f'belief_open={self.belief_door_is_open:.6f} action={self.last_action} '
                          f'feature={self.feature_mean_value:.2f}')

            if self.belief_door_is_open >= self.belief_threshold:
                self.state = 'MOVING'
                self.state_start = now
                self.move_robot()
                self.log.info('High confidence door open — moving')

        elif self.state == 'MOVING':
            self.move_robot()
            if now - self.state_start >= self.move_duration_sec:
                self.stop_robot()
                self.state = 'CLOSING'
                self.state_start = now
                self.log.info('Passed through — closing door')

        elif self.state == 'CLOSING':
            self.close_door()
            if now - self.state_start >= self.close_duration_sec:
                self.state = 'DONE'
                self.stop_robot()
                self.log.info('Done')

        elif self.state == 'DONE':
            pass

    def _predict(self, use_command: bool):
        b = self.belief_door_is_open
        if use_command:
            p_oo = self.p_open_given_open_cmd
            p_oc = self.p_open_given_closed_cmd
        else:
            p_oo = self.p_open_given_open_noop
            p_oc = self.p_open_given_closed_noop
        self.belief_door_is_open = p_oo * b + p_oc * (1.0 - b)

    def _update_from_latest_measurement(self):
        z_open = (self.feature_mean_value < self.measurement_threshold)
        like_open = self.p_zo_xo if z_open else self.p_zc_xo
        like_closed = self.p_zo_xc if z_open else self.p_zc_xc
        prior_open = self.belief_door_is_open
        prior_closed = 1.0 - prior_open
        bel_open_unnorm = like_open * prior_open
        bel_closed_unnorm = like_closed * prior_closed
        denom = bel_open_unnorm + bel_closed_unnorm
        if denom > 0.0:
            self.belief_door_is_open = bel_open_unnorm / denom

    def check_feature_mean(self, f: Float64):
        self.feature_mean_value = f.data
        self.meas_seq += 1

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
