#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Quaternion, TransformStamped, PoseStamped
import tf2_ros
import numpy as np
import matplotlib.pyplot as plt

class Wheel_odom_node(Node):
    def __init__(self):
        super().__init__('Wheel_odom_node')

        # Robot Parameters
        self.R = 0.033  # wheel radius (meters)
        self.L = 0.16   # wheel base (meters) 
        
        # Robot State [x, y, theta]
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        # Last wheel positions
        self.last_left_pos = None
        self.last_right_pos = None

        # History for plotting path at the end
        self.history_x = []
        self.history_y = []

        # --- Rviz2 Setting ---
        self.odom_pub = self.create_publisher(Odometry, '/wheel_odom', 10)
        self.path_pub = self.create_publisher(Path, '/wheel_odom_path', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Path Base Frame ( Run with SLAM: 'map' | Run without SLAM: 'odom' )
        self.path_base_frame = 'odom'
        
        # Creating Path
        self.path_msg = Path()
        self.path_msg.header.frame_id = self.path_base_frame
        # --------------------------------

        # Subscription
        self.create_subscription(JointState, 'joint_states', self.joint_states_callback, 10)

    def joint_states_callback(self, msg: JointState):

        # Check if we have both left and right wheel positions
        if len(msg.position) < 2:
            return

        # Assign current wheel positions
        current_left_pos = msg.position[0]
        current_right_pos = msg.position[1]

        if self.last_left_pos is not None:

            # Calculate odometry
            self.wheel_odom_calculate(current_left_pos, current_right_pos)

            # Add current position to history for plotting path at the end
            self.history_x.append(self.x)
            self.history_y.append(self.y)

            # Send data to Rviz2 (Odom, TF and Path)
            self.broadcast_rviz(msg.header.stamp)

        # Update last wheel positions
        self.last_left_pos = current_left_pos
        self.last_right_pos = current_right_pos

    # Calculate diff-drive odometry
    # Ref: https://automaticaddison.com/calculating-wheel-odometry-for-a-differential-drive-robot/
    def wheel_odom_calculate(self, current_left_pos, current_right_pos):

        d_left = (current_left_pos - self.last_left_pos) * self.R
        d_right = (current_right_pos - self.last_right_pos) * self.R
        
        d_center = (d_left + d_right) / 2.0
        d_theta = (d_right - d_left) / self.L

        self.x += d_center * np.cos(self.theta + d_theta / 2.0)
        self.y += d_center * np.sin(self.theta + d_theta / 2.0)
        self.theta += d_theta

    def broadcast_rviz(self, stamp):

        # 1. Convert Yaw (theta) to Quaternion
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = np.sin(self.theta / 2.0)
        q.w = np.cos(self.theta / 2.0)

        # 2. Send TF Transform (odom -> base_link)
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation = q
        self.tf_broadcaster.sendTransform(t)

        # 3. Send Odometry Message
        odom_msg = Odometry()
        odom_msg.header.stamp = stamp
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.orientation = q
        self.odom_pub.publish(odom_msg)

        # 4. Send Path Message
        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = self.path_base_frame
        pose.pose.position.x = self.x
        pose.pose.position.y = self.y
        pose.pose.orientation = q
        
        self.path_msg.header.stamp = stamp
        self.path_msg.poses.append(pose)
        self.path_pub.publish(self.path_msg)

    def plot_trajectory(self):
        plt.figure(figsize=(8, 8))
        plt.plot(self.history_x, self.history_y, label='Robot Path')
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