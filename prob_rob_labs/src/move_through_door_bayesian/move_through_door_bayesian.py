import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import time

heartbeat_period = 0.05

class MoveThroughDoorBayesian(Node):

    def __init__(self):
        super().__init__('move_through_door_bayesian')
        self.log = self.get_logger()
        self.declare_parameter('threshold', 235.0)
        self.declare_parameter('window_sec', 20.0)

        self.threshold = self.get_parameter('threshold').value
        self.window_sec = self.get_parameter('window_sec').value

        self.torque = 50.0
        self.phase_sec = 2
        self.ignore_sec = 0.2

        self.pub_torque = self.create_publisher(Float64, '/hinged_glass_door/torque', 1)
        self.sub_feature_mean = self.create_subscription(Float64, '/feature_mean', self.check_feature_mean, 10)

        self.start_time = None
        self.phase_start_time = None
        self.open_phase = True

        self.feature_mean_value = 0.0
        self.below = 0
        self.above = 0
        self.xopen_n = 0
        self.xclosed_n = 0
        self.zopen_xopen = 0
        self.zclosed_xopen = 0
        self.zopen_xclosed = 0
        self.zclosed_xclosed = 0

        self.timer = self.create_timer(heartbeat_period, self.heartbeat)
        self.log.info(f'Threshold={self.threshold}, window={self.window_sec}s')

    def now(self):
        return time.time()

    def heartbeat(self):
        now = self.now()
        if self.start_time is None:
            self.start_time = now
            self.phase_start_time = now

        if now - self.start_time >= self.window_sec:
            self.report_results()
            rclpy.shutdown()
            return

        if now - self.phase_start_time >= self.phase_sec:
            self.open_phase = not self.open_phase
            self.phase_start_time = now

        if self.open_phase:
            self.pub_torque.publish(Float64(data=self.torque))
        else:
            self.pub_torque.publish(Float64(data=-self.torque))

    def check_feature_mean(self, msg):
        now = self.now()
        if self.phase_start_time is None or (now - self.phase_start_time) < self.ignore_sec:
            return

        self.feature_mean_value = msg.data
        z_open = 1 if self.feature_mean_value < self.threshold else 0
        if z_open:
            self.below += 1
        else:
            self.above += 1

        if self.open_phase:
            self.xopen_n += 1
            if z_open:
                self.zopen_xopen += 1
            else:
                self.zclosed_xopen += 1
        else:
            self.xclosed_n += 1
            if z_open:
                self.zopen_xclosed += 1
            else:
                self.zclosed_xclosed += 1

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
