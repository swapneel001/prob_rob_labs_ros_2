import rclpy
from rclpy.node import Node
from prob_rob_msgs.msg import Point2DArrayStamped
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
import math


def euler_from_quaternion(x, y, z, w):
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    return math.atan2(t3, t4)

class EkfSlamNode(Node):
    def __init__(self):
        super().__init__('ekf_slam')

        # init EKF-SLAM parameters
        self.NUM_LANDMARKS = 5
        self.STATE_SIZE = 3 + 2 * self.NUM_LANDMARKS # 3(Robot) + 10(Landmarks) = 13
        
        # mu
        self.mu = np.zeros((self.STATE_SIZE, 1))
        
        # Sigma
        self.Sigma = np.eye(self.STATE_SIZE) * 1000.0  
        self.Sigma[0:3, 0:3] = np.zeros((3, 3))        

        self.seen_landmarks = [False] * self.NUM_LANDMARKS
        self.FX = 530.4669
        self.CX = 320.5
        self.REAL_LANDMARK_WIDTH = 0.20 # m

        # noise parameters
        self.R_noise = np.diag([0.3**2, 0.3**2]) #(Range, Bearing)0.15 before
        self.Q_noise = np.diag([0.01**2, 0.01**2, 0.01**2])
        #self.Q_noise = np.diag([0.1**2, 0.1**2, 0.05**2]) 

        # Variable to record the last frame time
        self.last_odom_time = None



        # odom subscription
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # vision subscriptions
        self.color_map = {'red': 0, 'green': 1, 'cyan': 2, 'yellow': 3, 'magenta': 4}
        for color, lid in self.color_map.items():
            topic = f'/vision_{color}/corners'
            self.create_subscription(Point2DArrayStamped, topic, 
                                     lambda msg, i=lid: self.landmark_callback(msg, i), 10)

        self.path_pub = self.create_publisher(Path, '/ekf_slam/path', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/ekf_slam/landmarks', 10)
    
        self.path_msg = Path()
        self.path_msg.header.frame_id = "odom" 

        self.get_logger().info("EKF-SLAM System Started!")

    
    def odom_callback(self, msg):
        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        
        # (v, omega)
        v = msg.twist.twist.linear.x
        omega = msg.twist.twist.angular.z

        if self.last_odom_time is None:
            self.last_odom_time = current_time
            return

        dt = current_time - self.last_odom_time
        self.last_odom_time = current_time
        
        
        if dt > 0.1: dt = 0.1

        # EKF Predict
        theta = self.mu[2, 0]
        

        if abs(omega) < 0.001: 
            self.mu[0, 0] += v * np.cos(theta) * dt
            self.mu[1, 0] += v * np.sin(theta) * dt
        else: 
            self.mu[0, 0] += (-v/omega * np.sin(theta) + v/omega * np.sin(theta + omega*dt))
            self.mu[1, 0] += ( v/omega * np.cos(theta) - v/omega * np.cos(theta + omega*dt))
        
        self.mu[2, 0] += omega * dt
        self.mu[2, 0] = (self.mu[2, 0] + np.pi) % (2 * np.pi) - np.pi

       
        G = np.eye(self.STATE_SIZE)
        G[0, 2] = -v * np.sin(theta) * dt
        G[1, 2] =  v * np.cos(theta) * dt

        
        Q_full = np.zeros((self.STATE_SIZE, self.STATE_SIZE))
        Q_full[0:3, 0:3] = self.Q_noise 

        self.Sigma = G @ self.Sigma @ G.T + Q_full

        
        self.publish_visualization()

    def landmark_callback(self, msg, lid):
        if len(msg.points) == 0: return
        
        # Pixel -> Range/Bearing
        xs = [p.x for p in msg.points]
        w_pixel = max(xs) - min(xs)

        if w_pixel < 40.0: return 

        u_curr = sum(xs) / len(xs)
        r_meas = (self.FX * self.REAL_LANDMARK_WIDTH) / w_pixel
        phi_meas = np.arctan2((self.CX - u_curr), self.FX)
        
        if r_meas > 4.5: 
            return 
        
        # --- EKF Update ---
        idx = 3 + 2 * lid 

        # Case A 
        if not self.seen_landmarks[lid]:
            
            if r_meas > 2.5: 
                return
            
            
            theta = self.mu[2, 0]
            lx = self.mu[0, 0] + r_meas * np.cos(theta + phi_meas)
            ly = self.mu[1, 0] + r_meas * np.sin(theta + phi_meas)
            
            self.mu[idx, 0] = lx
            self.mu[idx+1, 0] = ly
            self.seen_landmarks[lid] = True
            
            self.get_logger().info(f"Initialized Landmark {lid} at ({lx:.2f}, {ly:.2f})")
            return

        # Case B
        lx = self.mu[idx, 0]
        ly = self.mu[idx+1, 0]
        dx = lx - self.mu[0, 0]
        dy = ly - self.mu[1, 0]
        q = dx**2 + dy**2
        r_hat = np.sqrt(q)
        phi_hat = np.arctan2(dy, dx) - self.mu[2, 0]
        phi_hat = (phi_hat + np.pi) % (2 * np.pi) - np.pi 

        H = np.zeros((2, self.STATE_SIZE))
     
        H[0, 0] = -dx/r_hat; H[0, 1] = -dy/r_hat; H[0, 2] = 0
        H[1, 0] = dy/q;      H[1, 1] = -dx/q;     H[1, 2] = -1
        
        H[0, idx] = dx/r_hat; H[0, idx+1] = dy/r_hat
        H[1, idx] = -dy/q;    H[1, idx+1] = dx/q

        
        S = H @ self.Sigma @ H.T + self.R_noise
        K = self.Sigma @ H.T @ np.linalg.inv(S)

        
        z_diff = np.array([[r_meas - r_hat], [phi_meas - phi_hat]])
        z_diff[1, 0] = (z_diff[1, 0] + np.pi) % (2 * np.pi) - np.pi 

        self.mu = self.mu + K @ z_diff
        self.Sigma = (np.eye(self.STATE_SIZE) - K @ H) @ self.Sigma

    
    def publish_visualization(self):
        
        pose = PoseStamped()
        pose.header.frame_id = "odom"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(self.mu[0, 0])
        pose.pose.position.y = float(self.mu[1, 0])
        self.path_msg.poses.append(pose)
        self.path_pub.publish(self.path_msg)

        
        marker_array = MarkerArray()
        for i in range(self.NUM_LANDMARKS):
            if self.seen_landmarks[i]:
                marker = Marker()
                marker.header.frame_id = "odom"
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.id = i
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD
                marker.scale.x = 0.2; marker.scale.y = 0.2; marker.scale.z = 0.2
                marker.color.a = 1.0; marker.color.g = 1.0 
                
                idx = 3 + 2 * i
                marker.pose.position.x = float(self.mu[idx, 0])
                marker.pose.position.y = float(self.mu[idx+1, 0])
                marker_array.markers.append(marker)
        
        self.marker_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = EkfSlamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()