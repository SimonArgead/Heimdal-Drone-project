import os
import xacro
import rclpy
from launch import LaunchDescription
from launch.actions import ExecuteProcess, OpaqueFunction, RegisterEventHandler, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.event_handlers import OnProcessStart

def wait_for_clock(context):
    from rosgraph_msgs.msg import Clock
    rclpy.init()
    node = rclpy.create_node('wait_for_clock_node')
    clock_ready = False

    def cb(msg):
        nonlocal clock_ready
        clock_ready = True

    sub = node.create_subscription(Clock, '/clock', cb, 10)
    node.get_logger().info("Venter på /clock fra Gazebo...")

    end_time = node.get_clock().now() + rclpy.time.Duration(seconds=60)
    while rclpy.ok() and not clock_ready and node.get_clock().now() < end_time:
        rclpy.spin_once(node, timeout_sec=0.1)

    node.destroy_node()
    rclpy.shutdown()

    if not clock_ready:
        raise RuntimeError("Modtog aldrig /clock – Gazebo startede ikke korrekt")

def generate_launch_description():
    home_dir = os.getenv("HOME")
    world_path = os.path.join(home_dir, "ros2_ws", "src", "drone_sim", "worlds", "empty.world")
    xacro_path = os.path.join(home_dir, "ros2_ws", "src", "drone_sim", "urdf", "zephyr_delta_wing.xacro")

    # Parse xacro → URDF string
    doc = xacro.parse(open(xacro_path))
    xacro.process_doc(doc, mappings={
        'prefix': '',
        'namespace': 'zephyr',
        'pkg': 'package://drone_sim',
    })
    robot_desc = doc.toxml()

    # Gem URDF til /tmp
    urdf_tmp_path = "/tmp/zephyr_delta_wing.urdf"
    with open(urdf_tmp_path, "w") as f:
        f.write(robot_desc)

    # Start Gazebo
    gz_sim = ExecuteProcess(
        cmd=["gz", "sim", "-r", world_path],
        output="screen"
    )

    # Bridge /clock fra Gazebo til ROS 2
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['--config-file', os.path.join(home_dir, 'ros2_ws', 'src', 'drone_sim', 'config', 'gz_bridge.yaml')],
        output='screen'
    )

    # Robot State Publisher
    state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='zephyr_state_publisher',
        parameters=[
            {'robot_description': robot_desc},
            {'use_sim_time': True},
        ],
        output='screen'
    )

    # Spawn dronen i Gazebo
    spawn_model_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                os.path.dirname(__import__('importlib.util').util.find_spec('ros_gz_sim').origin),
                '..', 'launch', 'ros_gz_spawn_model.launch.py'
            )
        ),
        launch_arguments={
            'world': 'sonoma_raceway',
            'file': urdf_tmp_path,
            'entity_name': 'zephyr_drone',
            'x': '0', 'y': '0', 'z': '10'
        }.items()
    )

    # Stereo Odometry
    stereo_odom = Node(
        package='rtabmap_odom',
        executable='stereo_odometry',
        name='stereo_odometry',
        output='screen',
        parameters=[{
            'frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'publish_tf': True,
            'approx_sync': True,
            'use_sim_time': True
        }],
        remappings=[
            ('/odom', '/rtabmap/odom'),
            ('left/image_rect', '/zephyr/camera_left/image_rect'),
            ('right/image_rect', '/zephyr/camera_right/image_rect'),
            ('left/camera_info', '/zephyr/camera_left/camera_info'),
            ('right/camera_info', '/zephyr/camera_right/camera_info')
        ]
    )

    # MAVROS2 node til ArduPilot SITL   
    mavros_node = Node(
        package='mavros',
        executable='mavros_node',
        name='mavros',
        output='screen',
        arguments=[
            '--ros-args',
            '-p', 'use_sim_time:=true',
            '-p', 'fcu_urls:=[udp://:14550@]',
            '-p', 'gcs_urls:=[]',
            '-p', 'uas_urls:=[]'
        ]
    )

    # Start MAVROS først efter clock er klar
    start_mavros_handler = RegisterEventHandler(
        OnProcessStart(
            target_action=clock_bridge,
            on_start=[
                OpaqueFunction(function=wait_for_clock),
                mavros_node
            ]
        )
    )


    # Start clock_bridge 2 sekunder efter Gazebo er startet
    clock_bridge_handler = RegisterEventHandler(
        OnProcessStart(
            target_action=gz_sim,
            on_start=[
                TimerAction(period=2.0, actions=[clock_bridge])
            ]
        )
    )

    # Vent på clock før vi spawner modellen
    wait_for_clock_handler = RegisterEventHandler(
        OnProcessStart(
            target_action=clock_bridge,
            on_start=[
                OpaqueFunction(function=wait_for_clock),
                spawn_model_launch
            ]
        )
    )

    return LaunchDescription([
        gz_sim,
        clock_bridge_handler,
        wait_for_clock_handler,
        state_pub,
        stereo_odom,
        start_mavros_handler
    ])
