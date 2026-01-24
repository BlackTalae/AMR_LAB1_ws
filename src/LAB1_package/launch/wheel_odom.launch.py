from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([

        # 1. Node EKF ที่คุณเขียนเอง
        Node(
            package='LAB1_package',
            executable='wheel_odom_node',
            name='wheel_odom_node',
            parameters=[{'wheel_radius': 0.05, 'wheel_base': 0.2}]
        ),
        
        # 2. เล่นไฟล์ Rosbag (ใส่ path ให้ถูก)
        ExecuteProcess(
            cmd=['ros2', 'bag', 'play', 'src/LAB1_package/datasets/fibo_floor3_seq00/fibo_floor3_seq00.db3'],
            output='screen'
        ),
        
    ])