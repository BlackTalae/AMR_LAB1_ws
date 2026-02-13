from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction

def generate_launch_description():

    dataset_no = 2
    dataset_name = f"fibo_floor3_seq0{dataset_no}"

    return LaunchDescription([

        # 1. Node Wheel Odometry
        # Node(
        #     package='LAB1_package',
        #     executable='wheel_odom_node.py',
        #     name='wheel_odom_node',
        #     parameters=[{'mode': 'position'}, {'use_sim_time': True}]
        # ),

        # Node(
        #     package='LAB1_package',
        #     executable='EKF_odom_node.py',
        #     name='EKF_odom_node',
        # ),

        Node(
            package='LAB1_package',
            executable='KEN_ICP_node.py',
            name='KEN_ICP_node',
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
        
        # 3. RViz2
        ExecuteProcess(
            cmd=['ros2', 'run', 'rviz2', 'rviz2', '-d', '/home/talae/AMR_LAB1_ws/src/LAB1_package/config/config.rviz'],
            output='screen'
        ),
    ])