#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState, LaserScan
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Quaternion, TransformStamped, PoseStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy
import tf2_ros
import numpy as np

class EKF_Odom_Node(Node):
    def __init__(self):
        super().__init__('ekf_odom_node')

        # EKF States & Noise
        self.state_ekf = np.zeros((3, 1))      # [x, y, yaw]^T
        self.P = np.eye(3)                      
        self.Q = np.diag([0.5, 0.5, np.deg2rad(0.001)])**2  
        self.R_mat = np.diag([np.deg2rad(0.1)])**2         

        self.imu_offset = None

        # Shared Buffer
        self.latest_imu_yaw = None
        self.latest_imu_gyro_z = 0.0

        # Robot Params
        self.R, self.L = 0.033, 0.16
        self.last_left_pos, self.last_right_pos = None, None
        self.last_time = None
        
        # ROS 2 Publishers & TF
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.path_ekf_pub = self.create_publisher(Path, '/path_ekf', 10)
        
        # Path Base Frame ( Run with SLAM:'map' | Run without SLAM:'odom' )
        self.path_base_frame = 'map'
        
        # Creating Path
        self.path_ekf_msg = Path()
        self.path_ekf_msg.header.frame_id = self.path_base_frame

        # Subscriptions
        self.create_subscription(Imu, 'imu', self.imu_callback, 10)
        self.create_subscription(JointState, 'joint_states', self.joint_states_callback, 10)

        # Visualize by lidar
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)
        self.create_subscription(LaserScan, 'scan', self.scan_callback, qos)
        self.scan_pub = self.create_publisher(LaserScan, '/visualize_scan', 10)

    # ==========================================
    # EKF CORE FUNCTION (The Only Entry Point)
    # ==========================================

    def ekf_estimation(self, u, z, dt):
        """ Combine Predict and Update steps in one function following Bayes Filter """
        # --- Predict Step ---
        yaw = self.state_ekf[2, 0]
        v, omega = u[0, 0], u[1, 0]
        
        # State Prediction: f(x, u)
        self.state_ekf[0, 0] += v * np.cos(yaw) * dt
        self.state_ekf[1, 0] += v * np.sin(yaw) * dt
        self.state_ekf[2, 0] += omega * dt

        # Covariance Prediction: G P G^T + Q
        G = np.array([
            [1.0, 0.0, -dt * v * np.sin(yaw)],
            [0.0, 1.0,  dt * v * np.cos(yaw)],
            [0.0, 0.0, 1.0]
        ])
        self.P = G @ self.P @ G.T + self.Q

        # --- Update Step (Correct) ---
        if z is not None:
            H = np.array([[0, 0, 1]]) # เราวัดแค่ yaw
            y = z - (H @ self.state_ekf)
            y[0, 0] = (y[0, 0] + np.pi) % (2 * np.pi) - np.pi # Normalize Angle
            
            S = H @ self.P @ H.T + self.R_mat
            K = self.P @ H.T @ np.linalg.inv(S) # Kalman Gain
            
            self.state_ekf = self.state_ekf + K @ y
            self.P = (np.eye(3) - K @ H) @ self.P

    # ==========================================
    # CALLBACKS
    # ==========================================

    def scan_callback(self, msg: LaserScan):
        scan_to_viz = msg
        scan_to_viz.header.frame_id = 'base_link'
        self.scan_pub.publish(scan_to_viz)

    def imu_callback(self, msg: Imu):

        # Omega
        self.latest_imu_gyro_z = msg.angular_velocity.z

        # Theta
        q = msg.orientation # Quaternion
        raw_yaw = np.arctan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

        if self.imu_offset is None:
            self.imu_offset = raw_yaw # เก็บมุมแรกไว้เป็นค่าอ้างอิง

        # ลบค่าเริ่มต้นออกเพื่อให้เริ่มที่ 0 เสมอ
        relative_yaw = raw_yaw - self.imu_offset
        self.latest_imu_yaw = np.array([[relative_yaw]])

    def joint_states_callback(self, msg: JointState):
        """ Run EKF every time the wheel moves """
        if len(msg.position) < 2: return
        t_now = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        
        if self.last_time is None:
            self.last_time, self.last_left_pos, self.last_right_pos = t_now, msg.position[0], msg.position[1]
            return

        dt = t_now - self.last_time
        if dt <= 0: return

        # 1. Calculate robot speed (Control Input: u)
        d_l = (msg.position[0] - self.last_left_pos) * self.R
        d_r = (msg.position[1] - self.last_right_pos) * self.R
        v = ((d_l + d_r) / 2.0) / dt
        omega = ((d_r - d_l) / self.L) / dt # self.latest_imu_gyro_z
        u = np.array([[v], [omega]])

        # 2. Run EKF (Estimate position and correct with IMU)
        self.ekf_estimation(u, self.latest_imu_yaw, dt)
        
        # 3. Clear IMU Buffer : Prevent using the same data
        self.latest_imu_yaw = None

        # 4. Save and Send Data
        self.publish_ekf_data(msg.header.stamp)
        self.last_left_pos, self.last_right_pos, self.last_time = msg.position[0], msg.position[1], t_now

    # ==========================================
    # PUBLISHERS
    # ==========================================

    def publish_ekf_data(self, stamp):
        x, y, th = self.state_ekf[0,0], self.state_ekf[1,0], self.state_ekf[2,0]
        
        # Publish Path
        pose = PoseStamped()
        pose.header.stamp, pose.header.frame_id = stamp, 'odom'
        pose.pose.position.x, pose.pose.position.y = x, y
        pose.pose.orientation.z, pose.pose.orientation.w = np.sin(th/2), np.cos(th/2)
        
        self.path_ekf_msg.poses.append(pose)
        self.path_ekf_pub.publish(self.path_ekf_msg)

        # Broadcast TF (odom -> base_link)
        t = TransformStamped()
        t.header.stamp, t.header.frame_id = stamp, 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x, t.transform.translation.y = x, y
        t.transform.rotation.z, t.transform.rotation.w = np.sin(th/2), np.cos(th/2)
        self.tf_broadcaster.sendTransform(t)

def main():
    rclpy.init()
    node = EKF_Odom_Node()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__': main()