import os
import socket
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController


def _find_free_port(start_port=1234, max_tries=100):
    for port in range(start_port, start_port + max_tries):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            if sock.connect_ex(('127.0.0.1', port)) != 0:
                return port
    return start_port


def generate_launch_description():
    package_dir = get_package_share_directory('corgi_sim')
    launch_user = os.environ.get('USER') or os.environ.get('USERNAME') or 'root'
    webots_port = str(_find_free_port(start_port=int(os.environ.get('WEBOTS_PORT', '1234'))))

    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='realtime',
        description='Webots startup mode: realtime, pause, fast'
    )
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='Corgi_ABAD_Upsidedown.wbt',
        description='World file name under corgi_sim/worlds'
    )

    world_path = PathJoinSubstitution([
        package_dir,
        'worlds',
        LaunchConfiguration('world')
    ])

    webots = WebotsLauncher(
        world=world_path,
        gui=True,
        ros2_supervisor=False,
        mode=LaunchConfiguration('mode'),
        port=webots_port
    )

    robot_driver = WebotsController(
        robot_name='CorgiRobotABAD',
        port=webots_port,
        parameters=[
            {
                'robot_description': os.path.join(package_dir, 'resource', 'corgi.urdf')
            }
        ],
        respawn=True
    )

    control_panel = Node(
        package='corgi_panel',
        executable='corgi_control_panel',
        parameters=[{
            'use_sim_time': True,
            'enable_custom_sequence': True,
        }],
        output='screen'
    )

    return LaunchDescription([
        mode_arg,
        world_arg,
        launch.actions.SetEnvironmentVariable(name='USER', value=launch_user),
        launch.actions.SetEnvironmentVariable(name='USERNAME', value=launch_user),
        webots,
        robot_driver,
        control_panel,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        ),
    ])
