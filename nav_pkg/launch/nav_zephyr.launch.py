import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Share directories
    nav_share = get_package_share_directory('nav_pkg')
    drone_sim_share = get_package_share_directory('drone_sim')
    bridge_config = os.path.join(nav_share, 'config', 'gazebo_ros2_bridge.yaml')

    return LaunchDescription([
        # Gazebo <-> ROS 2 bridge
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='ros_gz_bridge',
            parameters=[bridge_config],
            output='screen'
        ),

        # Din RTAB-Map-baserede zig-zag-controller
        Node(
            package='nav_pkg',
            executable='rtabmap_nav',   # matcher target-navnet i CMakeLists.txt
            name='rtabmap_nav',
            output='screen',
            parameters=[
                {'pose_topic': '/rtabmap/odom'},  # RTAB-Map's odometri-topic
                {'cmd_vel_topic': '/cmd_vel'},
                {'takeoff_duration_s': 3.0},
                {'vz_takeoff': 1.0},
                {'vx_forward': 1.0},
                {'vy_amplitude': 1.0},
                {'zig_period_s': 4.0},
                {'use_sim_time': True}
            ],
        ),
    ])
