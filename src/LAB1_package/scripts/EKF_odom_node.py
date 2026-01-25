#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from nav_msgs.msg import Path
from geometry_msgs.msg import TransformStamped, PoseStamped
import tf2_ros
import numpy as np
import matplotlib.pyplot as plt

class EKF_SLAM_Comparison(Node):
    def __init__(self):
        super().__init__('ekf_comparison_node')

        # --- 1. EKF Parameters (อ้างอิง PythonRobotics) ---
        self.state_ekf = np.zeros((3, 1))      # State: [x, y, yaw]^T
        self.P = np.eye(3)                      # Covariance Matrix
        
        # Noise Matrices (ปรับแต่งตามความแรงของสัญญาณ)
        self.Q = np.diag([0.1, 0.1, np.deg2rad(1.0)])**2  # Process noise
        self.R_mat = np.diag([np.deg2rad(1.0)])**2         # Observation noise (IMU)

        # Robot Physical Params
        self.R, self.L = 0.066, 0.16
        self.x_raw, self.y_raw, self.th_raw = 0.0, 0.0, 0.0
        
        # --- 2. ROS 2 Utilities ---
        self.last_left_pos, self.last_right_pos = None, None
        self.last_time = None
        
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.path_raw_pub = self.create_publisher(Path, '/path_raw', 10)
        self.path_ekf_pub = self.create_publisher(Path, '/path_ekf', 10)
        self.path_raw_msg = Path(header={'frame_id': 'map'})
        self.path_ekf_msg = Path(header={'frame_id': 'map'})

        # History for Visualization
        self.hist_raw = {"x": [], "y": []}
        self.hist_ekf = {"x": [], "y": []}

        # Subscriptions
        self.create_subscription(Imu, 'imu', self.imu_callback, 10)
        self.create_subscription(JointState, 'joint_states', self.joint_states_callback, 10)

    # ==========================================
    # EKF CORE FUNCTIONS (Based on PythonRobotics)
    # ==========================================

    def motion_model(self, x, u, dt):
        """ g(x, u) - ระบบการเคลื่อนที่ของหุ่นยนต์ """
        F = np.array([[1, 0, 0],
                      [0, 1, 0],
                      [0, 0, 1]])
        
        B = np.array([[dt * np.cos(x[2, 0]), 0],
                      [dt * np.sin(x[2, 0]), 0],
                      [0, dt]])
        
        return F @ x + B @ u

    def observation_model(self, x):
        """ h(x) - ระบบการวัด (เราวัดแค่ Yaw จาก IMU) """
        H = np.array([[0, 0, 1]])
        return H @ x

    def jacobian_f(self, x, u, dt):
        """ G_t - Jacobian ของ Motion Model เทียบกับ State """
        yaw = x[2, 0]
        v = u[0, 0]
        return np.array([
            [1.0, 0.0, -dt * v * np.sin(yaw)],
            [0.0, 1.0,  dt * v * np.cos(yaw)],
            [0.0, 0.0, 1.0]
        ])

    def ekf_estimation(self, u, z, dt):
        """ ขั้นตอนรวม Predict และ Correct """
        # --- Predict ---
        x_pred = self.motion_model(self.state_ekf, u, dt)
        j_f = self.jacobian_f(self.state_ekf, u, dt)
        self.P = j_f @ self.P @ j_f.T + self.Q

        # --- Update ---
        if z is not None:
            z_pred = self.observation_model(x_pred)
            y = z - z_pred # Innovation
            y[0, 0] = (y[0, 0] + np.pi) % (2 * np.pi) - np.pi # Normalize yaw
            
            H = np.array([[0, 0, 1]]) # Jacobian ของ Observation Model
            S = H @ self.P @ H.T + self.R_mat
            K = self.P @ H.T @ np.linalg.inv(S) # Kalman Gain
            
            x_pred = x_pred + K @ y
            self.P = (np.eye(len(x_pred)) - K @ H) @ self.P
        
        self.state_ekf = x_pred

    # ==========================================
    # CALLBACKS & UTILITIES (จัดการข้อมูล ROS)
    # ==========================================

    def joint_states_callback(self, msg: JointState):
        if len(msg.position) < 2: return
        t_now = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        
        if self.last_time is None:
            self.last_time, self.last_left_pos, self.last_right_pos = t_now, msg.position[0], msg.position[1]
            return

        dt = t_now - self.last_time
        if dt <= 0: return

        # 1. Kinematics for Raw & EKF
        d_l = (msg.position[0] - self.last_left_pos) * self.R
        d_r = (msg.position[1] - self.last_right_pos) * self.R
        v, omega = ((d_l + d_r) / 2.0) / dt, ((d_r - d_l) / self.L) / dt
        u = np.array([[v], [omega]])

        # 2. Raw Odometry Calculation
        d_c, d_th = (d_l + d_r) / 2.0, (d_r - d_l) / self.L
        self.x_raw += d_c * np.cos(self.th_raw + d_th/2.0)
        self.y_raw += d_c * np.sin(self.th_raw + d_th/2.0)
        self.th_raw += d_th

        # 3. EKF Predict
        self.ekf_estimation(u, None, dt)

        # Record & Publish
        self.update_history_and_publish(msg.header.stamp)
        self.last_left_pos, self.last_right_pos, self.last_time = msg.position[0], msg.position[1], t_now

    def imu_callback(self, msg: Imu):
        q = msg.orientation
        yaw = np.arctan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        z = np.array([[yaw]])
        # Update EKF with IMU observation
        self.ekf_estimation(np.array([[0.0], [0.0]]), z, 0.0)

    def update_history_and_publish(self, stamp):
        self.hist_raw["x"].append(self.x_raw); self.hist_raw["y"].append(self.y_raw)
        self.hist_ekf["x"].append(self.state_ekf[0,0]); self.hist_ekf["y"].append(self.state_ekf[1,0])
        
        # Publish Paths to Rviz
        self.path_raw_msg.poses.append(self.create_pose(self.x_raw, self.y_raw, self.th_raw, stamp))
        self.path_ekf_msg.poses.append(self.create_pose(self.state_ekf[0,0], self.state_ekf[1,0], self.state_ekf[2,0], stamp))
        self.path_raw_pub.publish(self.path_raw_msg); self.path_ekf_pub.publish(self.path_ekf_msg)

    def create_pose(self, x, y, th, stamp):
        p = PoseStamped()
        p.header.stamp, p.header.frame_id = stamp, 'map'
        p.pose.position.x, p.pose.position.y = x, y
        p.pose.orientation.z, p.pose.orientation.w = np.sin(th/2), np.cos(th/2)
        return p

def main():
    rclpy.init()
    node = EKF_SLAM_Comparison()
    try: 
        rclpy.spin(node)
    except KeyboardInterrupt: 
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__': main()