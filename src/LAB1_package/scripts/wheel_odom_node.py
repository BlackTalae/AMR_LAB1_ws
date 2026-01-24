#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Quaternion, TransformStamped, PoseStamped
import tf2_ros
import numpy as np
import matplotlib.pyplot as plt

class Wheel_odom_node(Node):
    def __init__(self):
        super().__init__('Wheel_odom_node')

        # Robot Parameters
        self.R = 0.066  # wheel radius (meters)
        self.L = 0.16   # wheel base (meters) 
        
        # Initital State [x, y, theta]
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        self.last_left_pos = None
        self.last_right_pos = None

        self.history_x = []
        self.history_y = []

        # --- ส่วนการตั้งค่าสำหรับ Rviz2 ---
        self.odom_pub = self.create_publisher(Odometry, '/raw_odom', 10)
        self.path_pub = self.create_publisher(Path, '/robot_path', 10) # Publisher สำหรับเส้น Path
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # สร้างตัวแปรเก็บ Path
        self.path_msg = Path()
        self.path_msg.header.frame_id = 'map'
        # --------------------------------

        self.create_subscription(Imu, 'imu', self.imu_callback, 10)
        self.create_subscription(JointState, 'joint_states', self.joint_states_callback, 10)

        self.timer = self.create_timer(0.05, self.timer_callback)

    def joint_states_callback(self, msg: JointState):
        if len(msg.position) < 2:
            return

        current_left_pos = msg.position[0]
        current_right_pos = msg.position[1]

        if self.last_left_pos is not None:
            d_left = (current_left_pos - self.last_left_pos) * self.R
            d_right = (current_right_pos - self.last_right_pos) * self.R
            
            d_center = (d_left + d_right) / 2.0
            d_theta = (d_right - d_left) / self.L

            self.x += d_center * np.cos(self.theta + d_theta/2.0)
            self.y += d_center * np.sin(self.theta + d_theta/2.0)
            self.theta += d_theta

            self.history_x.append(self.x)
            self.history_y.append(self.y)

            # ส่งข้อมูลไป Rviz2 (รวมทั้ง Odom, TF และ Path)
            self.broadcast_rviz(msg.header.stamp)

        self.last_left_pos = current_left_pos
        self.last_right_pos = current_right_pos

    def broadcast_rviz(self, stamp):
        # 1. แปลง Yaw (theta) เป็น Quaternion
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = np.sin(self.theta / 2.0)
        q.w = np.cos(self.theta / 2.0)

        # 2. ส่ง TF Transform (odom -> base_link)
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation = q
        self.tf_broadcaster.sendTransform(t)

        # 3. ส่ง Odometry Message
        odom_msg = Odometry()
        odom_msg.header.stamp = stamp
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.orientation = q
        self.odom_pub.publish(odom_msg)

        # 4. ส่ง Path Message (เพิ่มจุดใหม่เข้าไปในเส้นทาง)
        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = 'map'
        pose.pose.position.x = self.x
        pose.pose.position.y = self.y
        pose.pose.orientation = q
        
        self.path_msg.header.stamp = stamp
        self.path_msg.poses.append(pose)
        self.path_pub.publish(self.path_msg)

    def imu_callback(self, msg: Imu):
        pass

    def timer_callback(self):
        pass
        # self.get_logger().info(f'Current Pose: x={self.x:.3f}, y={self.y:.3f}, theta={self.theta:.3f}')

    def plot_trajectory(self):
        plt.figure(figsize=(8, 8))
        plt.plot(self.history_x, self.history_y, label='Robot Path (Matplotlib)')
        plt.xlabel('X (meters)')
        plt.ylabel('Y (meters)')
        plt.title('Robot 2D Trajectory from Joint States')
        plt.legend()
        plt.axis('equal')
        plt.grid(True)
        plt.show()

def main(args=None):
    rclpy.init(args=args)
    node = Wheel_odom_node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Closing... Plotting graph.')
        node.plot_trajectory()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__=='__main__':
    main()