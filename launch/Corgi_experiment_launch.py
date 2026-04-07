import os
import socket
import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
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
    
    # --- Launch Arguments ---
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='realtime',
        description='Webots startup mode: realtime, pause, fast'
    )
    
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='Corgi_ABAD_experiment.wbt',
        description='Choose the world file name'
    )

    # 1. 使用實驗專用 world (Solid 有 DEF SUPPORT_BOX 可供 Supervisor 移除)
    world_path = launch.substitutions.PathJoinSubstitution([
        package_dir, 'worlds', LaunchConfiguration('world')
    ])

    # 2. 啟動 Webots — mode 預設 realtime (不停頓)
    webots = WebotsLauncher(
        world=world_path,
        gui=False,
        ros2_supervisor=False,
        mode=LaunchConfiguration('mode'),
        port=webots_port
    )

    # 3. 啟動機器人控制器 (ROS 2 Bridge)
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

    # *** 不啟動 corgi_panel — 實驗 / headless 模式不需要 GUI ***

    return LaunchDescription([
        mode_arg,
        world_arg,
        launch.actions.SetEnvironmentVariable(name='USER', value=launch_user),
        launch.actions.SetEnvironmentVariable(name='USERNAME', value=launch_user),
        # 告知 driver 跳過初始 PAUSE 並啟用箱子移除邏輯
        launch.actions.SetEnvironmentVariable(name='CORGI_EXPERIMENT_MODE', value='1'),
        webots,
        robot_driver,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])
