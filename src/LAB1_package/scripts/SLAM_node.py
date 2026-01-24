#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import tf2_ros
import numpy as np

class SLAMNode(Node):
    def __init__(self):
        super().__init__('slam_node')

        # 1. Static TF สำหรับ LiDAR
        self.tf_static_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        self.make_static_transform(0.029, 0.0, 0.192, 0.0, 'base_link', 'base_scan')

        # 2. ส่วนการรับค่าจาก SLAM และส่ง Path ไป Rviz
        # โดยปกติ slam_toolbox จะส่งค่ามาที่ /pose (ชนิด PoseWithCovarianceStamped)
        self.slam_sub = self.create_subscription(
            PoseWithCovarianceStamped, 
            '/pose', 
            self.slam_pose_callback, 
            10)
        
        # Publisher สำหรับแสดงเส้นทาง (Path) ใน Rviz
        self.slam_path_pub = self.create_publisher(Path, '/slam_path', 10)
        self.slam_path = Path()
        self.slam_path.header.frame_id = 'map' # SLAM อ้างอิงกับ map เสมอ

        self.get_logger().info('SLAM Path Node Started')

    def make_static_transform(self, x, y, z, yaw, parent_frame, child_frame):
        t = TransformStamped()
        # ใช้เวลาปัจจุบันของ Sim/Bag
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = parent_frame
        t.child_frame_id = child_frame
        t.transform.translation.x = float(x)
        t.transform.translation.y = float(y)
        t.transform.translation.z = float(z)
        t.transform.rotation.z = np.sin(yaw / 2.0)
        t.transform.rotation.w = np.cos(yaw / 2.0)
        self.tf_static_broadcaster.sendTransform(t)

    def slam_pose_callback(self, msg):
        """ รับค่าจาก SLAM แล้วสร้างเป็น Path ส่งไป Rviz """
        # สร้าง PoseStamped สำหรับใส่ใน Path
        pose_stamped = PoseStamped()
        pose_stamped.header = msg.header
        pose_stamped.pose = msg.pose.pose

        # เพิ่มตำแหน่งลงในลิสต์ Path
        self.slam_path.poses.append(pose_stamped)
        self.slam_path.header.stamp = msg.header.stamp

        # ส่ง Path ออกไป
        self.slam_path_pub.publish(self.slam_path)
        
        # Log ดูค่า x, y เพื่อความแน่ใจ
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.get_logger().info(f'SLAM Pose: x={x:.3f}, y={y:.3f}')

def main(args=None):
    rclpy.init(args=args)
    node = SLAMNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()