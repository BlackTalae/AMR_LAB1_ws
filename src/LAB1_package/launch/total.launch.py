import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import ExecuteProcess, TimerAction

def generate_launch_description():

    dataset_no = 0
    dataset_name = f"fibo_floor3_seq0{dataset_no}"

    return LaunchDescription([

        # 1. RViz2
        ExecuteProcess(
            cmd=['ros2', 'run', 'rviz2', 'rviz2', '-d', '/home/talae/AMR_LAB1_ws/src/LAB1_package/config/total_config.rviz'],
            output='screen'
        ),
        
        # 2. Rosbag
        TimerAction(
            period=1.0,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'bag', 'play', '/home/talae/AMR_LAB1_ws/src/LAB1_package/dataset/' + dataset_name + '/', '--rate', '1.0'],
                    output='screen'
                ),
            ]
        ),

        # 3. Wheel Odom Node
        Node(
            package='LAB1_package',
            executable='wheel_odom_node.py',
            name='wheel_odom_node',
            parameters=[{'mode': 'position'}]
        ),

        # 4. KEN EKF Odom + ICP Odom Node
        Node(
            package='LAB1_package',
            executable='KEN_ICP_node.py',
            name='KEN_ICP_node',
        ),

        # 5. SLAM Node
        Node(
            package='LAB1_package',
            executable='SLAM_node.py',
            name='SLAM_node',
        ),
    
        # 6. Slam Toolbox
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                '/home/talae/AMR_LAB1_ws/src/LAB1_package/config/slam_params.yaml',
            ]
        ),

        # 7. Eval Node
        Node(
            package='LAB1_package',
            executable='eval_node.py',
            name='eval_node',
        ),
        
 
    ])