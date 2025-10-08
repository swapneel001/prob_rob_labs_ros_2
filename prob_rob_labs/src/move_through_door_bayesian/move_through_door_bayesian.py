import math
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

heartbeat_period = 0.05

class MoveThroughDoorBayesian(Node):

    def __init__(self):
        super().__init__('move_through_door_bayesian')
        self.log = self.get_logger()
        self.declare_parameter('threshold', 235.0)
        self.declare_parameter('measure_window_sec', 30.0)

        self.threshold = self.get_parameter('threshold').value
        self.measure_window_sec = self.get_parameter('measure_window_sec').value

        self.pub_torque = self.create_publisher(Float64, '/hinged_glass_door/torque', 1)
        self.sub_feature_mean = self.create_subscription(Float64, '/feature_mean', self.check_feature_mean, 10)

        self.torque = 5.0
        self.cmd_phase_sec = 3.0
        self.settle_sec = 2.0

        self.trials_total = 3
        self.trial_idx = 0

        self.state = 'close_cmd'
        self.state_start = None
        self.measure_active = False
        self.current_x_open = None

        self.below = 0
        self.above = 0
        self.xopen_n = 0
        self.xclosed_n = 0
        self.zopen_xopen = 0
        self.zclosed_xopen = 0
        self.zopen_xclosed = 0
        self.zclosed_xclosed = 0

        self.timer = self.create_timer(heartbeat_period, self.heartbeat)
        self.log.info(f'Threshold={self.threshold}, measure_window_sec={self.measure_window_sec}s, trials={self.trials_total}')

    def now(self):
        return time.time()

    def heartbeat(self):
        t = self.now()
        if self.state_start is None:
            self.state_start = t
        dt = t - self.state_start

        if self.state == 'close_cmd':
            self.pub_torque.publish(Float64(data=-self.torque))
            if dt >= self.cmd_phase_sec:
                self.state = 'close_settle'
                self.state_start = t

        elif self.state == 'close_settle':
            self.pub_torque.publish(Float64(data=-self.torque))
            if dt >= self.settle_sec:
                self.state = 'close_measure'
                self.state_start = t
                self.measure_active = True
                self.current_x_open = False

        elif self.state == 'close_measure':
            self.pub_torque.publish(Float64(data=-self.torque))
            if dt >= self.measure_window_sec:
                self.measure_active = False
                self.state = 'open_cmd'
                self.state_start = t

        elif self.state == 'open_cmd':
            self.pub_torque.publish(Float64(data=self.torque))
            if dt >= self.cmd_phase_sec:
                self.state = 'open_settle'
                self.state_start = t

        elif self.state == 'open_settle':
            self.pub_torque.publish(Float64(data=self.torque))
            if dt >= self.settle_sec:
                self.state = 'open_measure'
                self.state_start = t
                self.measure_active = True
                self.current_x_open = True

        elif self.state == 'open_measure':
            self.pub_torque.publish(Float64(data=self.torque))
            if dt >= self.measure_window_sec:
                self.measure_active = False
                self.trial_idx += 1
                if self.trial_idx < self.trials_total:
                    self.state = 'close_cmd'
                    self.state_start = t
                else:
                    self.report_results()
                    rclpy.shutdown()
                    return

    def check_feature_mean(self, msg):
        if not self.measure_active:
            return
        v = msg.data
        z_open = 1 if v < self.threshold else 0
        if z_open: self.below += 1
        else:      self.above += 1

        if self.current_x_open:
            self.xopen_n += 1
            if z_open: self.zopen_xopen += 1
            else:      self.zclosed_xopen += 1
        else:
            self.xclosed_n += 1
            if z_open: self.zopen_xclosed += 1
            else:      self.zclosed_xclosed += 1

    def report_results(self):
        def p(a,b): return (a/b) if b>0 else math.nan
        p_zo_xo = p(self.zopen_xopen, self.xopen_n)
        p_zc_xo = p(self.zclosed_xopen, self.xopen_n)
        p_zo_xc = p(self.zopen_xclosed, self.xclosed_n)
        p_zc_xc = p(self.zclosed_xclosed, self.xclosed_n)
        self.log.info(f'Counts: below={self.below}, above={self.above}')
        self.log.info(f'x=open   N={self.xopen_n}:   P(z=open|x=open)={p_zo_xo:.3f}, P(z=closed|x=open)={p_zc_xo:.3f}')
        self.log.info(f'x=closed N={self.xclosed_n}: P(z=open|x=closed)={p_zo_xc:.3f}, P(z=closed|x=closed)={p_zc_xc:.3f}')

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
