#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState, LaserScan
from nav_msgs.msg import Path
from geometry_msgs.msg import TransformStamped, PoseStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy
import tf2_ros
import numpy as np
from scipy.spatial import KDTree

# --- ICP Parameters ---
EPS = 0.0001        # Convergence threshold for ICP
MAX_ITER = 20       # Maximum iterations for ICP to ensure real-time performance

class EKF_ICP_Refined_Node(Node):
    """
    ROS 2 Node that performs sensor fusion using an Extended Kalman Filter (EKF)
    and refines the odometry using Iterative Closest Point (ICP) on LiDAR scans.
    """
    def __init__(self):
        super().__init__('ekf_icp_refined_node')

        # --- EKF Parameters ---
        self.state_ekf = np.zeros((3, 1))      # [x, y, theta]
        self.P = np.eye(3)                      
        self.Q = np.diag([0.5, 0.5, np.deg2rad(0.001)])**2  
        self.R_mat = np.diag([np.deg2rad(0.1)])**2         
        self.imu_offset = None
        self.latest_imu_yaw = None

        # --- ICP Odometry States ---
        self.x_lidar, self.y_lidar, self.th_lidar = 0.0, 0.0, 0.0
        self.prev_scan_world = None # Store previous scan in World frame for Target
        
        # Robot Params
        self.R, self.L = 0.033, 0.16
        self.last_left_pos, self.last_right_pos, self.last_time = None, None, None
        
        # Publishers & TF
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.path_lidar_pub = self.create_publisher(Path, '/path_lidar_refined', 10)
        self.path_lidar_msg = Path()
        self.path_lidar_msg.header.frame_id = 'odom'

        # Subscriptions
        self.create_subscription(Imu, 'imu', self.imu_callback, 10)
        self.create_subscription(JointState, 'joint_states', self.joint_states_callback, 10)
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)
        self.create_subscription(LaserScan, 'scan', self.scan_callback, qos)

    # ==========================================
    # CORE ICP FUNCTIONS
    # ==========================================
    def icp_matching(self, previous_points, current_points):

        H = None

        dError, preError, count = np.inf, np.inf, 0

        while dError >= EPS:

            count += 1

            indexes, error = self.determine_correspondences(previous_points, current_points)
            Rt, Tt = self.svd_motion_estimation(previous_points[:, indexes], current_points)
            current_points = (Rt @ current_points) + Tt[:, np.newaxis]
            
            dError = preError - error

            if dError < 0 or MAX_ITER <= count: break 

            preError = error

            H = self.update_homogeneous_matrix(H, Rt, Tt)

            if dError <= EPS: break

        R = np.array(H[0:-1, 0:-1]); T = np.array(H[0:-1, -1])

        return R, T

    def update_homogeneous_matrix(self, Hin, R, T):

        r_size = R.shape[0]
        H = np.zeros((r_size + 1, r_size + 1))

        H[0:r_size, 0:r_size] = R
        H[0:r_size, r_size] = T
        H[r_size, r_size] = 1.0

        return H if Hin is None else Hin @ H

    # def nearest_neighbor_association(self, previous_points, current_points):
    #     delta_points = previous_points - current_points
    #     error = sum(np.linalg.norm(delta_points, axis=0))
    #     d = np.linalg.norm(np.repeat(current_points, previous_points.shape[1], axis=1)
    #                        - np.tile(previous_points, (1, current_points.shape[1])), axis=0)
    #     indexes = np.argmin(d.reshape(current_points.shape[1], previous_points.shape[1]), axis=1)
    #     return indexes, error

    def determine_correspondences(self, previous_points, current_points):
        """
        Use KDTree to find the nearest neighbors between two frames
        - previous_points: Target points (2, N)
        - current_points: Source points (2, M)
        """
        # 1. Create a Tree from the previous frame points (Target)
        tree = KDTree(previous_points.T)
        
        # 2. Find the nearest neighbors for each point in current_points (Source)
        distances, indexes = tree.query(current_points.T)
        
        # 3. Calculate the total error (mean or sum of distances)
        error = np.sum(distances)

        return indexes, error

    def svd_motion_estimation(self, previous_points, current_points):
        pm, cm = np.mean(previous_points, axis=1), np.mean(current_points, axis=1)
        
        p_shift, c_shift = previous_points - pm[:, np.newaxis], current_points - cm[:, np.newaxis]
        
        W = c_shift @ p_shift.T
        u, s, vh = np.linalg.svd(W)
        
        R = (u @ vh).T
        t = pm - (R @ cm)
        
        return R, t

    # ==========================================
    # SCAN CALLBACK (Integration Point)
    # ==========================================
    def scan_callback(self, msg: LaserScan):

        # 1. Convert raw scan to Point Cloud (Local Frame)
        ranges = np.array(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))
        valid = (ranges > msg.range_min) & (ranges < msg.range_max) & (ranges > 0.15)
        valid[::5] = valid[::5] 
        curr_pts_local = np.vstack((ranges[valid]*np.cos(angles[valid]), ranges[valid]*np.sin(angles[valid])))

        if curr_pts_local.shape[1] < 10: return

        # 2. Convert to World Frame by using EKF (Initial Guess)
        tx, ty, tth = self.state_ekf[0,0], self.state_ekf[1,0], self.state_ekf[2,0]
        rot = np.array([[np.cos(tth), -np.sin(tth)], [np.sin(tth), np.cos(tth)]])
        curr_pts_world = (rot @ curr_pts_local) + np.array([[tx], [ty]])

        if self.prev_scan_world is not None:
            # 3. Use ICP Matching to align the points from EKF to the previous scan
            # previous_points = previous ICP points in world frame
            # current_points = current scan points in world frame
            R_refined, T_refined = self.icp_matching(self.prev_scan_world, curr_pts_world)
            
            # 4. Update the LiDAR Odometry from the ICP result
            self.x_lidar += T_refined[0]
            self.y_lidar += T_refined[1]
            self.th_lidar += np.arctan2(R_refined[1, 0], R_refined[0, 0])
            
            self.publish_lidar_refined(msg.header.stamp)

        # store current scan for next time step Target
        self.prev_scan_world = curr_pts_world

    # ==========================================
    # EKF FUNCTIONS
    # ==========================================
    def ekf_estimation(self, u, z, dt):

        yaw = self.state_ekf[2, 0]
        v, omega = u[0, 0], u[1, 0]

        self.state_ekf[0, 0] += v * np.cos(yaw) * dt
        self.state_ekf[1, 0] += v * np.sin(yaw) * dt
        self.state_ekf[2, 0] += omega * dt

        G = np.array([[1.0, 0.0, -dt*v*np.sin(yaw)], [0.0, 1.0, dt*v*np.cos(yaw)], [0.0, 0.0, 1.0]])
        self.P = G @ self.P @ G.T + self.Q

        if z is not None:
            H = np.array([[0, 0, 1]])
            y = z - (H @ self.state_ekf)
            y[0, 0] = (y[0, 0] + np.pi) % (2 * np.pi) - np.pi

            S = H @ self.P @ H.T + self.R_mat
            K = self.P @ H.T @ np.linalg.inv(S)

            self.state_ekf += K @ y
            self.P = (np.eye(3) - K @ H) @ self.P

    def joint_states_callback(self, msg: JointState):
        if len(msg.position) < 2: return
        t_now = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        
        if self.last_time is None:
            self.last_time, self.last_left_pos, self.last_right_pos = t_now, msg.position[0], msg.position[1]
            return
        
        dt = t_now - self.last_time
        if dt <= 0: return
        
        d_l, d_r = (msg.position[0]-self.last_left_pos)*self.R, (msg.position[1]-self.last_right_pos)*self.R
        u = np.array([[(d_l+d_r)/2/dt], [(d_r-d_l)/self.L/dt]])
        
        self.ekf_estimation(u, self.latest_imu_yaw, dt)
        
        self.latest_imu_yaw = None
        
        self.last_left_pos, self.last_right_pos, self.last_time = msg.position[0], msg.position[1], t_now

    def imu_callback(self, msg: Imu):
        q = msg.orientation
        raw_yaw = np.arctan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y**2 + q.z**2))
        
        if self.imu_offset is None: self.imu_offset = raw_yaw
        
        self.latest_imu_yaw = np.array([[ (raw_yaw - self.imu_offset + np.pi)%(2*np.pi) - np.pi ]])

    # ==========================================
    # PUBLISH FUNCTIONS
    # ==========================================
    def publish_lidar_refined(self, stamp):
        
        pose = PoseStamped()
        pose.header.stamp, pose.header.frame_id = stamp, 'odom'
        pose.pose.position.x, pose.pose.position.y = self.x_lidar, self.y_lidar
        pose.pose.orientation.z, pose.pose.orientation.w = np.sin(self.th_lidar/2), np.cos(self.th_lidar/2)
        self.path_lidar_msg.header.stamp = stamp
        self.path_lidar_msg.poses.append(pose)
        self.path_lidar_pub.publish(self.path_lidar_msg)

        # Send TF for Rviz to show robot moving according to ICP
        t = TransformStamped()
        t.header.stamp, t.header.frame_id = stamp, 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x, t.transform.translation.y = self.x_lidar, self.y_lidar
        t.transform.rotation.z, t.transform.rotation.w = np.sin(self.th_lidar/2), np.cos(self.th_lidar/2)
        self.tf_broadcaster.sendTransform(t)

def main():
    rclpy.init(); node = EKF_ICP_Refined_Node()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__': main()