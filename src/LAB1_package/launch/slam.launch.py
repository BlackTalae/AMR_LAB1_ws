import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    return LaunchDescription([

        # 1. Wheel Odom Node
        # Node(
        #     package='LAB1_package',
        #     executable='wheel_odom_node.py',
        #     name='wheel_odom_node',
        #     parameters=[{'use_sim_time': use_sim_time}]
        # ),

        # 2. EKF Odom Node
        Node(
            package='LAB1_package',
            executable='EKF_odom_node.py',
            name='EKF_odom_node',
            parameters=[{'use_sim_time': use_sim_time}]
        ),

        # 3. Static TF Node (LiDAR Offset 192mm)
        Node(
            package='LAB1_package',
            executable='SLAM_node.py',
            name='SLAM_node',
            parameters=[{'use_sim_time': use_sim_time}]
        ),
    
        # 4. Slam Toolbox (เรียกใช้ผ่าน Node โดยตรงเพื่อตั้งค่าพารามิเตอร์)
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'odom_frame': 'odom',
                'base_frame': 'base_link',
                'scan_topic': '/scan',
                'mode': 'mapping'
            }]
        )
    ])