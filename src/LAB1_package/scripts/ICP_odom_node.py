#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState, LaserScan
from nav_msgs.msg import Path, OccupancyGrid
from geometry_msgs.msg import TransformStamped, PoseStamped
import tf2_ros
import numpy as np
from scipy.spatial import KDTree
from rclpy.qos import QoSProfile, ReliabilityPolicy
import struct

class EKF_ICP_Node(Node):
    def __init__(self):
        super().__init__('ekf_icp_node')

        # --- 1. SETUP PARAMETERS (TurtleBot3 Burger) ---
        self.R, self.L = 0.033, 0.16  
        self.Q = np.diag([0.05, 0.05, np.deg2rad(1.0)])**2 
        self.R_imu = np.diag([np.deg2rad(0.5)])**2
        
        # --- 2. EKF-SLAM STATES & GRID MAP CONFIG ---
        self.state_ekf = np.zeros((3, 1))      
        self.P = np.eye(3) * 0.1   

        self.landmarks = None                   
        self.map_res = 0.05      
        self.map_size = 200      
        
        # Buffers
        self.latest_imu_yaw = None
        self.last_left_pos, self.last_right_pos = None, None
        self.last_time = None

        # --- 3. ROS COMMUNICATION ---
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.path_pub = self.create_publisher(Path, '/path_slam', 10)
        self.path_msg = Path()
        self.path_msg.header.frame_id = 'odom'  
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', 10)

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)
        self.create_subscription(Imu, 'imu', self.imu_callback, 10)
        self.create_subscription(JointState, 'joint_states', self.joint_states_callback, 10)
        self.create_subscription(LaserScan, 'scan', self.scan_callback, qos)

    # ============================================================
    # THE UNIFIED EKF-SLAM PROCESSOR (หัวใจสำคัญ)
    # ============================================================

    def run_ekf_slam_iteration(self, u=None, imu_yaw=None, scan_msg=None, dt=0.0):
        """ 
        ฟังก์ชันเดียวที่ทำหน้าที่ประมวลผล SLAM: Predict -> Update
        """
        # --- 1. PREDICTION (Motion Model) ---
        if u is not None and dt > 0:
            v, omega = u[0, 0], u[1, 0]
            yaw = self.state_ekf[2, 0]
            
            # Predict State
            self.state_ekf[0, 0] += v * np.cos(yaw) * dt
            self.state_ekf[1, 0] += v * np.sin(yaw) * dt
            self.state_ekf[2, 0] += omega * dt
            
            # Predict Covariance P
            G = np.array([[1, 0, -dt*v*np.sin(yaw)], [0, 1, dt*v*np.cos(yaw)], [0, 0, 1]])
            self.P = G @ self.P @ G.T + self.Q

        # --- 2. UPDATE STEP 1: IMU (Angular correction) ---
        if imu_yaw is not None:
            z_imu = imu_yaw - self.state_ekf[2, 0]
            z_imu = (z_imu + np.pi) % (2 * np.pi) - np.pi # Normalize angle
            self.apply_kalman_gain(np.array([[z_imu]]), np.array([[0, 0, 1]]), self.R_imu)

        # --- 3. UPDATE STEP 2: LIDAR/ICP (Position correction) ---
        if scan_msg is not None:
            curr_pts_local = self.scan_to_points(scan_msg)
            curr_pts_global = self.transform_points(curr_pts_local, self.state_ekf)

            if self.landmarks is not None:
                # Data Association using ICP-style Matching
                tree = KDTree(self.landmarks)
                distances, indices = tree.query(curr_pts_global)
                
                gate_threshold = 0.15 + np.sqrt(np.trace(self.P[:2,:2])) # Adaptive gating
                valid = distances < gate_threshold
                
                if np.any(valid):
                    # Compute SVD for correction
                    s_pts, t_pts = curr_pts_global[valid], self.landmarks[indices[valid]]
                    mu_s, mu_t = np.mean(s_pts, axis=0), np.mean(t_pts, axis=0)
                    H_mat = (s_pts - mu_s).T @ (t_pts - mu_t)
                    U, _, Vt = np.linalg.svd(H_mat)
                    R_mat = Vt.T @ U.T
                    t_vec = mu_t - R_mat @ mu_s
                    
                    innovation = np.array([[t_vec[0]], [t_vec[1]], [np.arctan2(R_mat[1,0], R_mat[0,0])]])
                    self.apply_kalman_gain(innovation, np.eye(3), self.Q * 0.2) # Update Position

                # Mapping: Add new discovered points
                new_pts = curr_pts_global[~valid]
                if len(new_pts) > 0:
                    self.landmarks = np.vstack((self.landmarks, new_pts))
                    if len(self.landmarks) > 4000: self.landmarks = self.landmarks[::2]
            else:
                self.landmarks = curr_pts_global

    def apply_kalman_gain(self, innovation, H, R_noise):
        """ ฟังก์ชันสำหรับคำนวณและอัปเดต Kalman Gain """
        S = H @ self.P @ H.T + R_noise
        K = self.P @ H.T @ np.linalg.inv(S)
        self.state_ekf += K @ innovation
        self.P = (np.eye(H.shape[1]) - K @ H) @ self.P

    # ============================================================
    # ROS CALLBACKS (Clean and Minimal)
    # ============================================================

    def joint_states_callback(self, msg: JointState):
        t_now = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.last_time is None:
            self.last_time, self.last_left_pos, self.last_right_pos = t_now, msg.position[0], msg.position[1]
            return

        dt = t_now - self.last_time
        u = np.array([[( (msg.position[0]-self.last_left_pos)*self.R + (msg.position[1]-self.last_right_pos)*self.R ) / 2.0 / dt],
                      [( (msg.position[1]-self.last_right_pos)*self.R - (msg.position[0]-self.last_left_pos)*self.R ) / self.L / dt]])
        
        # รันเฉพาะ Prediction
        self.run_ekf_slam_iteration(u=u, dt=dt)
        
        self.last_left_pos, self.last_right_pos, self.last_time = msg.position[0], msg.position[1], t_now
        self.publish_tf(msg.header.stamp)

    def scan_callback(self, msg: LaserScan):
        # รัน Update (IMU ก่อนแล้วค่อย ICP)
        self.run_ekf_slam_iteration(imu_yaw=self.latest_imu_yaw, scan_msg=msg)
        self.latest_imu_yaw = None 
        self.publish_data(msg.header.stamp)

    def imu_callback(self, msg: Imu):
        q = msg.orientation
        current_yaw = np.arctan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))

        if not hasattr(self, 'imu_offset'):
            self.imu_offset = current_yaw # เก็บค่ามุมแรกสุดไว้

        self.latest_imu_yaw = current_yaw - self.imu_offset

    # ============================================================
    # UTILITIES & PUBLISHERS
    # ============================================================

    def scan_to_points(self, scan_msg):
        ranges = np.array(scan_msg.ranges)
        angles = np.linspace(scan_msg.angle_min, scan_msg.angle_max, len(ranges))
        valid = (ranges > scan_msg.range_min) & (ranges < scan_msg.range_max)
        # 29mm LiDAR Forward Offset
        return np.vstack((ranges[valid]*np.cos(angles[valid]) + 0.029, ranges[valid]*np.sin(angles[valid]))).T

    def transform_points(self, points, pose):
        th = pose[2,0]
        rot = np.array([[np.cos(th), -np.sin(th)], [np.sin(th), np.cos(th)]])
        return (rot @ points.T).T + pose[:2, 0]

    def create_occupancy_grid(self, points, stamp):
        msg = OccupancyGrid()
        msg.header.stamp = stamp
        msg.header.frame_id = 'odom'
        msg.info.resolution, msg.info.width, msg.info.height = self.map_res, self.map_size, self.map_size
        msg.info.origin.position.x, msg.info.origin.position.y = -(self.map_size * self.map_res) / 2, -(self.map_size * self.map_res) / 2
        grid_data = np.full((self.map_size * self.map_size), -1, dtype=np.int8)
        for p in points:
            ix = int((p[0] - msg.info.origin.position.x) / self.map_res)
            iy = int((p[1] - msg.info.origin.position.y) / self.map_res)
            if 0 <= ix < self.map_size and 0 <= iy < self.map_size: grid_data[iy * self.map_size + ix] = 100
        msg.data = grid_data.tolist()
        return msg

    def publish_data(self, stamp):
        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = 'odom'       
        pose.pose.position.x, pose.pose.position.y = self.state_ekf[0,0], self.state_ekf[1,0]
        pose.pose.orientation.z, pose.pose.orientation.w = np.sin(self.state_ekf[2,0]/2), np.cos(self.state_ekf[2,0]/2)
        self.path_msg.poses.append(pose)
        self.path_pub.publish(self.path_msg)
        if self.landmarks is not None: self.map_pub.publish(self.create_occupancy_grid(self.landmarks, stamp))

    def publish_tf(self, stamp):
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'  
        t.transform.translation.x, t.transform.translation.y = self.state_ekf[0,0], self.state_ekf[1,0]
        t.transform.rotation.z, t.transform.rotation.w = np.sin(self.state_ekf[2,0]/2), np.cos(self.state_ekf[2,0]/2)
        self.tf_broadcaster.sendTransform(t)

def main():
    rclpy.init(); node = EKF_ICP_Node()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__': main()