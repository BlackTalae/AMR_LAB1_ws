import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import ExecuteProcess, TimerAction

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    dataset_no = 2
    dataset_name = f"fibo_floor3_seq0{dataset_no}"

    return LaunchDescription([

        # 3. RViz2
        ExecuteProcess(
            cmd=['ros2', 'run', 'rviz2', 'rviz2', '-d', '/home/talae/AMR_LAB1_ws/src/LAB1_package/config/slam_config.rviz'],
            output='screen'
        ),
        
        # 0. Rosbag
        TimerAction(
            period=1.0,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'bag', 'play', '/home/talae/AMR_LAB1_ws/src/LAB1_package/dataset/' + dataset_name + '/', '--rate', '1.0'],
                    output='screen'
                ),
            ]
        ),

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
            parameters=[
                '/home/talae/AMR_LAB1_ws/src/LAB1_package/config/slam_params.yaml',
                {'use_sim_time': use_sim_time}
            ]
        ),
    ])