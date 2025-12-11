import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from gazebo_msgs.msg import LinkStates 
import matplotlib.pyplot as plt
import numpy as np
import math

class SlamEvaluator(Node):
    def __init__(self):
        super().__init__('slam_evaluator')
        
        # LinkStates ,get ground truth
        self.create_subscription(LinkStates, '/gazebo/link_states', self.gt_callback, 10)
        
        # ros2 topic echo /gazebo/link_states
        self.target_link_name = 'waffle_pi::base_footprint' 
        
       
        self.create_subscription(Path, '/ekf_slam/path', self.est_callback, 10)

        
        self.gt_history_x = []
        self.gt_history_y = []
        self.est_history_x = []
        self.est_history_y = []
        self.errors = []
        self.timestamps = []

        self.current_gt_pose = None
        self.get_logger().info(f"SLAM Evaluator Started. Tracking link: {self.target_link_name}")

    def gt_callback(self, msg):
        try:
            
            idx = msg.name.index(self.target_link_name)
            self.current_gt_pose = msg.pose[idx]
        except ValueError:
            
            pass 

    def est_callback(self, msg):
        if self.current_gt_pose is None:
            return

        
        if len(msg.poses) == 0: return
        latest_pose = msg.poses[-1].pose

        est_x = latest_pose.position.x
        est_y = latest_pose.position.y

        
        gt_x = self.current_gt_pose.position.x
        gt_y = self.current_gt_pose.position.y

        
        error = math.sqrt((est_x - gt_x)**2 + (est_y - gt_y)**2)

        
        self.gt_history_x.append(gt_x)
        self.gt_history_y.append(gt_y)
        self.est_history_x.append(est_x)
        self.est_history_y.append(est_y)
        self.errors.append(error)
        self.timestamps.append(self.get_clock().now().nanoseconds / 1e9)

        
        if len(self.errors) % 50 == 0:
            rmse = np.sqrt(np.mean(np.array(self.errors)**2))
            self.get_logger().info(f"Current RMSE: {rmse:.4f} m | Frame: {len(self.errors)}")

    def plot_results(self):
        self.get_logger().info("Generating Evaluation Plots...")
        
        if len(self.errors) == 0:
            print("No data collected.")
            return

        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))

        
        if len(self.gt_history_x) > 0 and len(self.est_history_x) > 0:
            
            gt_x0, gt_y0 = self.gt_history_x[0], self.gt_history_y[0]
            gt_x_aligned = [x - gt_x0 for x in self.gt_history_x]
            gt_y_aligned = [y - gt_y0 for y in self.gt_history_y]
            
            
            est_x0, est_y0 = self.est_history_x[0], self.est_history_y[0]
            est_x_aligned = [x - est_x0 for x in self.est_history_x]
            est_y_aligned = [y - est_y0 for y in self.est_history_y]
            
            
            aligned_errors = []
            min_len = min(len(gt_x_aligned), len(est_x_aligned))
            for i in range(min_len):
                e = math.sqrt((est_x_aligned[i] - gt_x_aligned[i])**2 + 
                              (est_y_aligned[i] - gt_y_aligned[i])**2)
                aligned_errors.append(e)

            
            ax1.plot(gt_x_aligned, gt_y_aligned, 'k--', label='Ground Truth', linewidth=2)
            ax1.plot(est_x_aligned, est_y_aligned, 'b-', label='EKF Estimate', linewidth=1)
            
            
            start_time = self.timestamps[0]
            rel_time = [t - start_time for t in self.timestamps[:min_len]]
            ax2.plot(rel_time, aligned_errors, 'r-')
            
            
            rmse = np.sqrt(np.mean(np.array(aligned_errors)**2))
            plt.suptitle(f"Final RMSE (Aligned): {rmse:.4f} m")

        ax1.set_title("Trajectory Comparison")
        ax1.set_xlabel("X [m]")
        ax1.set_ylabel("Y [m]")
        ax1.legend()
        ax1.grid(True)
        ax1.axis('equal')

        ax2.set_title("Position Error over Time")
        ax2.set_xlabel("Time [s]")
        ax2.set_ylabel("Error [m]")
        ax2.grid(True)
        
        plt.show()

def main(args=None):
    rclpy.init(args=args)
    node = SlamEvaluator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.plot_results()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()