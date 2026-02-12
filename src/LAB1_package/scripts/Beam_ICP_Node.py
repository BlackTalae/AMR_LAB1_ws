#!/usr/bin/python3

from LAB1_package.dummy_module import dummy_function, dummy_var
import rclpy
from rclpy.node import Node
import numpy as np

class Beam_ICP_Node(Node):
    def __init__(self):
        super().__init__('beam_icp_node')

    def icp_scan_to_map(self, scan_points, init_x, init_y, init_theta):
        # 1. Initialize
        x, y, theta = init_x, init_y, init_theta
        
        # 2. Run ICP
        for point in scan_points:
            # 2.1. Find Closest Point
            
            # 2.2. Update Pose
            
        # SVD on original scan points (body frame) vs map points (odom frame)
        src_m = scan_points[valid]
        tgt_m = self.local_map_points[indices[valid]]

        src_c = np.mean(src_m, axis=0)
        tgt_c = np.mean(tgt_m, axis=0)

        W = (tgt_m - tgt_c).T @ (src_m - src_c)
        U, _, Vt = np.linalg.svd(W)

        d = np.linalg.det(U @ Vt)
        R_opt = U @ np.diag([1, 1, np.sign(d)]) @ Vt
        t_opt = tgt_c - R_opt @ src_c

        theta = np.arctan2(R_opt[1, 0], R_opt[0, 0])
        x = t_opt[0]
        y = t_opt[1]
        
        return x, y, theta, True

    def scan_callback(self, msg):
        current_time = self.get_clock().now()

        if self.start_time is None:
            self.start_time = current_time
            self.publish_odom_and_path(current_time)

        self.scan_count += 1

        current_points = self.scan_to_pointcloud(msg)
        if len(current_points) < self.min_correspondences:
            return

        if self.latest_ekf_odom is None:
            return

        ekf_x, ekf_y, ekf_theta = self.extract_pose(self.latest_ekf_odom)

        # Initialize
        if self.prev_ekf_x is None:
            self.prev_ekf_x = ekf_x
            self.prev_ekf_y = ekf_y
            self.prev_ekf_theta = ekf_theta

            # Add first scan to local map
            odom_points = self.transform_to_odom(current_points, self.x, self.y, self.theta)
            self.local_map_scans.append(odom_points)
            self.local_map_dirty = True
            self.last_kf_x = self.x
            self.last_kf_y = self.y
            self.last_kf_theta = self.theta
            self.keyframe_count = 1
            return

        # Step 1: Apply EKF delta (always - this is our verified baseline)
        ekf_dx = ekf_x - self.prev_ekf_x
        ekf_dy = ekf_y - self.prev_ekf_y
        ekf_dtheta = np.arctan2(
            np.sin(ekf_theta - self.prev_ekf_theta),
            np.cos(ekf_theta - self.prev_ekf_theta))

        self.x += ekf_dx
        self.y += ekf_dy
        self.theta += ekf_dtheta
        self.theta = np.arctan2(np.sin(self.theta), np.cos(self.theta))

        # Step 2: Check if keyframe - run ICP against local map
        dist_from_kf = np.sqrt((self.x - self.last_kf_x)**2 + (self.y - self.last_kf_y)**2)
        angle_from_kf = abs(np.arctan2(
            np.sin(self.theta - self.last_kf_theta),
            np.cos(self.theta - self.last_kf_theta)))

        if dist_from_kf > self.keyframe_dist_thresh or \
        angle_from_kf > self.keyframe_angle_thresh:

            if len(self.local_map_scans) >= 3: # need enough map
                # Build/update local map KDTree
                if self.local_map_dirty:
                    self.rebuild_local_map()

                if self.local_map_tree is not None:
                    # Subsample current scan
                    if len(current_points) > self.max_scan_points:
                        idx = np.random.choice(len(current_points), self.max_scan_points, replace=False)
                        pts_icp = current_points[idx]
                    else:
                        pts_icp = current_points

                    # ICP: match current scan (in body frame) against local map (in odom frame)
                    try:
                        refined_x, refined_y, refined_theta, success = \
                            self.icp_scan_to_map(pts_icp, self.x, self.y, self.theta)

                        if success:
                            corr_t = np.sqrt((refined_x - self.x)**2 + (refined_y - self.y)**2)
                            corr_r = abs(np.arctan2(
                                np.sin(refined_theta - self.theta),
                                np.cos(refined_theta - self.theta)))

                            if corr_t < self.max_translation_correction and \
                            corr_r < self.max_rotation_correction:
                                self.x = refined_x
                                self.y = refined_y
                                self.theta = refined_theta
                                self.icp_success_count += 1
                            else:
                                self.fallback_count += 1
                        else:
                            self.fallback_count += 1
                    except Exception as e:
                        self.get_logger().error(f'ICP err: {e}', throttle_duration_sec=5.0)
                        self.fallback_count += 1

            # Add current scan to local map (transformed to odom frame)
            odom_points = self.transform_to_odom(current_points, self.x, self.y, self.theta)
            self.local_map_scans.append(odom_points)
            self.local_map_dirty = True

            self.last_kf_x = self.x
            self.last_kf_y = self.y
            self.last_kf_theta = self.theta
            self.keyframe_count += 1

        # Save trajectory
        timestamp = current_time.nanoseconds / 1e9
        self.trajectory.append([timestamp, self.x, self.y, self.theta])
        if self.first_msg_time is None:
            self.first_msg_time = timestamp
        self.last_msg_time = timestamp

        self.publish_odom_and_path(current_time)

        self.update_count += 1
        interval = 10 if self.update_count <= 50 else 50
        if self.update_count % interval == 0:
            elapsed = (current_time - self.start_time).nanoseconds / 1e9
            diff = np.sqrt((self.x - ekf_x)**2 + (self.y - ekf_y)**2)
            self.get_logger().info(
                f'[{elapsed:.1f}s] #{self.update_count} | '
                f'ICP: ({self.x:.3f}, {self.y:.3f}) θ={np.degrees(self.theta):.1f} | '
                f'EKF: ({ekf_x:.3f}, {ekf_y:.3f}) θ={np.degrees(ekf_theta):.1f} | '
                f'diff: {diff:.3f}m | '
                f'ok: {self.icp_success_count} fb: {self.fallback_count} '
                f'kf: {self.keyframe_count} map: {len(self.local_map_scans)}')

        self.prev_ekf_x = ekf_x
        self.prev_ekf_y = ekf_y
        self.prev_ekf_theta = ekf_theta

    # --- Utilities ---

    def transform_to_odom(self, points, x, y, theta):
        """Transform body-frame points to odom frame."""
        c = np.cos(theta)
        s = np.sin(theta)
        return np.column_stack([
            points[:, 0] * c - points[:, 1] * s + x,
            points[:, 0] * s + points[:, 1] * c + y
        ])

    def rebuild_local_map(self):
        """Merge all keyframe scans and downsample via voxel grid."""
        if len(self.local_map_scans) == 0:
            self.local_map_points = None
            self.local_map_tree = None
            return

        all_points = np.vstack(list(self.local_map_scans))

        if self.local_map_voxel_size > 0:
            all_points = self.voxel_downsample(all_points, self.local_map_voxel_size)

        self.local_map_points = all_points
        self.local_map_tree = KDTree(all_points)
        self.local_map_dirty = False

    def voxel_downsample(self, points, voxel_size):
        """Simple 2D voxel grid downsampling."""
        voxel_indices = np.floor(points / voxel_size).astype(int)
        voxel_dict = {}
        for i in range(len(points)):
            key = (voxel_indices[i, 0], voxel_indices[i, 1])
            if key not in voxel_dict:
                voxel_dict[key] = []
            voxel_dict[key].append(points[i])
        downsampled = np.array([np.mean(pts, axis=0) for pts in voxel_dict.values()])
        return downsampled

    def scan_to_pointcloud(self, scan_msg):
        ranges = np.array(scan_msg.ranges, dtype=np.float64)
        angles = scan_msg.angle_min + np.arange(len(ranges)) * scan_msg.angle_increment
        valid = (ranges > scan_msg.range_min) & (ranges < scan_msg.range_max) \
                & np.isfinite(ranges) & (ranges > 0.12)
        ranges, angles = ranges[valid], angles[valid]
        if len(ranges) == 0:
            return np.empty((0, 2))
        return np.column_stack([ranges * np.cos(angles), ranges * np.sin(angles)])

    def extract_pose(self, odom_msg):
        x = odom_msg.pose.pose.position.x
        y = odom_msg.pose.pose.position.y
        q = odom_msg.pose.pose.orientation
        # Assuming tf_translations is available in the scope
        _, _, yaw = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        return x, y, yaw

def main(args=None):
    rclpy.init(args=args)
    node = Beam_ICP_Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
