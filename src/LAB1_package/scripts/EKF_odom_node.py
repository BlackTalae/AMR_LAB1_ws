#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Quaternion, TransformStamped, PoseStamped
import tf2_ros
import numpy as np
import matplotlib.pyplot as plt

class EKF_Comparison_Node(Node):
    def __init__(self):
        super().__init__('EKF_Comparison_Node')

        # --- 1. Parameters ---
        self.R, self.L = 0.066, 0.16
        
        # --- 2. States ---
        # Pure Odometry State (Wheel only)
        self.x_raw, self.y_raw, self.th_raw = 0.0, 0.0, 0.0
        
        # EKF State [x, y, theta]^T
        self.state_ekf = np.zeros((3, 1))
        self.P = np.diag([0.1, 0.1, 0.01])
        self.Q = np.diag([0.01, 0.01, 0.02])
        self.R_mat = np.array([[0.001]]) 

        self.last_left_pos, self.last_right_pos = None, None
        self.last_time = self.get_clock().now()

        # --- 3. Publishers ---
        # Raw Odometry (Red Path in Rviz)
        self.path_raw_pub = self.create_publisher(Path, '/path_raw', 10)
        self.path_raw_msg = Path()
        self.path_raw_msg.header.frame_id = 'map'
        
        # EKF Odometry (Green Path in Rviz)
        self.path_ekf_pub = self.create_publisher(Path, '/path_ekf', 10)
        self.path_ekf_msg = Path()
        self.path_ekf_msg.header.frame_id = 'map'

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # History for Matplotlib
        self.hist_raw_x, self.hist_raw_y = [], []
        self.hist_ekf_x, self.hist_ekf_y = [], []

        # --- 4. Subs ---
        self.create_subscription(Imu, 'imu', self.imu_callback, 10)
        self.create_subscription(JointState, 'joint_states', self.joint_states_callback, 10)

    def euler_from_quaternion(self, q):
        x, y, z, w = q.x, q.y, q.z, q.w
        return np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))

    def joint_states_callback(self, msg: JointState):
        if len(msg.position) < 2: return
        curr_time = self.get_clock().now()
        dt = (curr_time - self.last_time).nanoseconds / 1e9
        if dt <= 0: return

        if self.last_left_pos is not None:
            # Kinematics calculation
            d_l = (msg.position[0] - self.last_left_pos) * self.R
            d_r = (msg.position[1] - self.last_right_pos) * self.R
            d_c = (d_l + d_r) / 2.0
            d_th = (d_r - d_l) / self.L
            v = d_c / dt
            omega = d_th / dt

            # A. Update Pure Odometry (No Fusion)
            self.x_raw += d_c * np.cos(self.th_raw + d_th/2.0)
            self.y_raw += d_c * np.sin(self.th_raw + d_th/2.0)
            self.th_raw += d_th

            # B. EKF Prediction Step
            th_ekf = self.state_ekf[2, 0]
            self.state_ekf[0, 0] += v * np.cos(th_ekf) * dt
            self.state_ekf[1, 0] += v * np.sin(th_ekf) * dt
            self.state_ekf[2, 0] += omega * dt

            G = np.array([[1, 0, -v*np.sin(th_ekf)*dt], [0, 1, v*np.cos(th_ekf)*dt], [0, 0, 1]])
            self.P = G @ self.P @ G.T + self.Q

            # Record Data
            self.hist_raw_x.append(self.x_raw); self.hist_raw_y.append(self.y_raw)
            self.hist_ekf_x.append(self.state_ekf[0,0]); self.hist_ekf_y.append(self.state_ekf[1,0])
            
            self.publish_paths(msg.header.stamp)

        self.last_left_pos, self.last_right_pos = msg.position[0], msg.position[1]
        self.last_time = curr_time

    def imu_callback(self, msg: Imu):
        # EKF Update Step (IMU Corrects Heading)
        z = np.array([[self.euler_from_quaternion(msg.orientation)]])
        H = np.array([[0, 0, 1]])
        y = z - (H @ self.state_ekf)
        y[0, 0] = (y[0, 0] + np.pi) % (2 * np.pi) - np.pi # Norm Angle
        
        S = H @ self.P @ H.T + self.R_mat
        K = self.P @ H.T @ np.linalg.inv(S)
        self.state_ekf = self.state_ekf + K @ y
        self.P = (np.eye(3) - K @ H) @ self.P

    def publish_paths(self, stamp):
        # Publish Raw Path (Red)
        self.path_raw_msg.poses.append(self.create_pose(self.x_raw, self.y_raw, self.th_raw, stamp))
        self.path_raw_pub.publish(self.path_raw_msg)
        
        # Publish EKF Path (Green)
        self.path_ekf_msg.poses.append(self.create_pose(self.state_ekf[0,0], self.state_ekf[1,0], self.state_ekf[2,0], stamp))
        self.path_ekf_pub.publish(self.path_ekf_msg)

        # TF Broadcaster follows EKF (The most accurate)
        t = TransformStamped()
        t.header.stamp = stamp; t.header.frame_id = 'odom'; t.child_frame_id = 'base_link'
        t.transform.translation.x = self.state_ekf[0,0]
        t.transform.translation.y = self.state_ekf[1,0]
        t.transform.rotation.z = np.sin(self.state_ekf[2,0]/2); t.transform.rotation.w = np.cos(self.state_ekf[2,0]/2)
        self.tf_broadcaster.sendTransform(t)

    def create_pose(self, x, y, th, stamp):
        p = PoseStamped()
        p.header.stamp = stamp; p.header.frame_id = 'map'
        p.pose.position.x = x; p.pose.position.y = y
        p.pose.orientation.z = np.sin(th/2); p.pose.orientation.w = np.cos(th/2)
        return p

    def plot_results(self):
        plt.figure(figsize=(10, 10))
        plt.plot(self.hist_raw_x, self.hist_raw_y, 'r--', label='Pure Wheel Odometry (Drifting)')
        plt.plot(self.hist_ekf_x, self.hist_ekf_y, 'g-', label='EKF Fusion (Wheel + IMU)')
        plt.title('Comparison: Wheel Odom vs EKF Fusion'); plt.legend(); plt.axis('equal'); plt.grid(True); plt.show()

def main():
    rclpy.init(); node = EKF_Comparison_Node()
    try: rclpy.spin(node)
    except KeyboardInterrupt: node.plot_results()
    finally: node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__': main()