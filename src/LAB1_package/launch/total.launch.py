import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import ExecuteProcess, TimerAction

def generate_launch_description():

    dataset_no = 2
    dataset_name = f"fibo_floor3_seq0{dataset_no}"
    user_name = "aitthikit"

    # 7. Slam Toolbox
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            f'/home/{user_name}/AMR_LAB1_ws/src/LAB1_package/config/toolbox_params.yaml',
        ]
    )

    slam_toolbox_launch = TimerAction(
        period=1.0,
        actions=[slam_toolbox_node]
    )

    return LaunchDescription([

        # 1. RViz2
        ExecuteProcess(
            cmd=['ros2', 'run', 'rviz2', 'rviz2', '-d', f'/home/{user_name}/AMR_LAB1_ws/src/LAB1_package/config/total_config.rviz'],
            output='screen'
        ),
        
        # 2. Rosbag
        TimerAction(
            period=1.0,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'bag', 'play', f'/home/{user_name}/AMR_LAB1_ws/src/LAB1_package/dataset/' + dataset_name + '/', '--rate', '1.0'],
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
            executable='KEN_ICP_2_node.py',
            name='KEN_ICP_2_node',
        ),

        # 5. EKF for slam_toolbox
        Node(
            package='LAB1_package',
            executable='EKF_odom_node.py',
            name='EKF_odom_node',
        ),

        # 6. SLAM Node
        Node(
            package='LAB1_package',
            executable='SLAM_node.py',
            name='SLAM_node',
        ),

        # 8. Eval Node
        # Node(
        #     package='LAB1_package',
        #     executable='eval_node.py',
        #     name='eval_node',
        # ),

        # 7. slam_toolbox
        slam_toolbox_launch,

    ])