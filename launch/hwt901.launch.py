import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
# Thêm 3 thư viện này để dùng tham số Launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

def generate_launch_description():
    pkg_share = get_package_share_directory('hwt901')
    
    # Khai báo đường dẫn động (Không bị fix cứng tên user)
    imu_config_path = os.path.join(pkg_share, 'config', 'imu_config.yaml')
    urdf_file_path = os.path.join(pkg_share, 'urdf', 'imu_fusion_lidar.urdf')
    rviz_file_path = os.path.join(pkg_share, 'rviz', 'hwt901.rviz')
    
    with open(urdf_file_path, 'r') as infp:
        robot_description_config = infp.read()

    # --- 1. Khai báo tham số Launch ---
    # Tên tham số là 'use_rviz', mặc định là 'false' (tắt)
    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='Bật (true) hoặc Tắt (false) RViz2'
    )

    # --- 2. Đọc giá trị tham số ---
    use_rviz = LaunchConfiguration('use_rviz')

    return LaunchDescription([
        # Nhớ đưa khai báo argument vào LaunchDescription
        declare_use_rviz_cmd, 

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'robot_description': robot_description_config
            }]
        ),
        
        Node(
            package="hwt901",
            executable="imu_node",
            name="hwt901_publisher",    
            parameters=[imu_config_path], 
            output="screen"
        ),

        # --- 3. Thêm điều kiện (condition) vào Node RViz ---
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_file_path], 
            output='screen',
            condition=IfCondition(use_rviz) # Nếu use_rviz = true thì mới chạy Node này
        )
    ])