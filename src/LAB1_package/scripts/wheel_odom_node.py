#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, LaserScan
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Quaternion, TransformStamped, PoseStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy
import tf2_ros
import numpy as np

class Wheel_odom_node(Node):
    def __init__(self):
        super().__init__('Wheel_odom_node')

        # --- Parameters ---
        self.declare_parameter('mode', 'all') 
        self.mode = self.get_parameter('mode').get_parameter_value().string_value

        self.R = 0.033
        self.L = 0.16
        
        self.x_p, self.y_p, self.th_p = 0.0, 0.0, 0.0
        self.x_v, self.y_v, self.th_v = 0.0, 0.0, 0.0
        
        self.last_left_pos = None
        self.last_right_pos = None
        self.last_time = None        

        # --- Publishers ---
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # Publisher สำหรับ Visualize LiDAR Scans
        self.scan_pub = self.create_publisher(LaserScan, '/visualize_scan', 10)

        if self.mode in ['position', 'all']:
            self.path_p_pub = self.create_publisher(Path, '/path_position', 10)
            self.path_p_msg = Path()
            self.path_p_msg.header.frame_id = 'odom'
            
        if self.mode in ['velocity', 'all']:
            self.path_v_pub = self.create_publisher(Path, '/path_velocity', 10)
            self.path_v_msg = Path()
            self.path_v_msg.header.frame_id = 'odom'

        # --- Subscriptions ---
        self.create_subscription(JointState, 'joint_states', self.joint_states_callback, 10)

        # Visualize by lidar
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)
        self.create_subscription(LaserScan, 'scan', self.scan_callback, qos)

        self.get_logger().info(f'--- Wheel Odom Node started in mode: {self.mode} ---')

    def scan_callback(self, msg: LaserScan):

        scan_to_viz = msg
        scan_to_viz.header.frame_id = 'base_link'
        self.scan_pub.publish(scan_to_viz)

    def joint_states_callback(self, msg: JointState):
        if len(msg.position) < 2 or len(msg.velocity) < 2:
            return

        # t_now = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        t_now = self.get_clock().now().seconds_nanoseconds()[0] + \
                      self.get_clock().now().seconds_nanoseconds()[1] * 1e-9
        if self.last_time is None:
            self.last_time = t_now
            self.last_left_pos, self.last_right_pos = msg.position[0], msg.position[1]
            return

        dt = t_now - self.last_time
        self.last_time = t_now 

        if self.mode in ['position', 'all']:
            d_l = (msg.position[0] - self.last_left_pos) * self.R
            d_r = (msg.position[1] - self.last_right_pos) * self.R
            d_c = (d_l + d_r) / 2.0
            d_th = (d_r - d_l) / self.L
            self.x_p += d_c * np.cos(self.th_p + d_th / 2.0)
            self.y_p += d_c * np.sin(self.th_p + d_th / 2.0)
            self.th_p += d_th

        if self.mode in ['velocity', 'all']:
            v = (msg.velocity[0] + msg.velocity[1]) / 2.0
            w = (msg.velocity[1] - msg.velocity[0]) / self.L
            self.x_v += v * np.cos(self.th_v) * dt
            self.y_v += v * np.sin(self.th_v) * dt
            self.th_v += w * dt

        self.last_left_pos, self.last_right_pos = msg.position[0], msg.position[1]
        self.publish_data(msg.header.stamp)

    def get_quat(self, yaw):
        q = Quaternion()
        q.z, q.w = np.sin(yaw / 2.0), np.cos(yaw / 2.0)
        q.x, q.y = 0.0, 0.0
        return q

    def publish_data(self, stamp):
        if self.mode in ['position', 'all']:
            q = self.get_quat(self.th_p)
            self.send_path(self.path_p_pub, self.path_p_msg, self.x_p, self.y_p, q, stamp)
            self.send_tf(self.x_p, self.y_p, q, stamp)

        if self.mode in ['velocity', 'all']:
            q = self.get_quat(self.th_v)
            self.send_path(self.path_v_pub, self.path_v_msg, self.x_v, self.y_v, q, stamp)
            if self.mode == 'velocity':
                self.send_tf(self.x_v, self.y_v, q, stamp)

    def send_path(self, pub, msg, x, y, q, stamp):
        pose = PoseStamped()
        pose.header.stamp, pose.header.frame_id = stamp, 'odom'
        pose.pose.position.x, pose.pose.position.y = x, y
        pose.pose.orientation = q
        msg.poses.append(pose)
        msg.header.stamp = stamp
        pub.publish(msg)

    def send_tf(self, x, y, q, stamp):
        t = TransformStamped()
        t.header.stamp, t.header.frame_id, t.child_frame_id = stamp, 'odom', 'base_link'
        t.transform.translation.x, t.transform.translation.y = x, y
        t.transform.translation.z = 0.0
        t.transform.rotation = q
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = Wheel_odom_node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__=='__main__':
    main()