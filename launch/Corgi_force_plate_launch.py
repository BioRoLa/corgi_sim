import os
import socket
import launch
from launch import LaunchDescription
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
    
    # 1. 設定 Webots 世界檔路徑
    world_path = os.path.join(package_dir, 'worlds', "IFS_force_plate_Proto" + ".wbt") # corgi_origin // IFS_Proto // Corgi_ABAD

    # 2. 啟動 Webots
    webots = WebotsLauncher(
        world=world_path,
        ros2_supervisor=False,
        port=webots_port
    )

    # 3. 啟動機器人控制器 (CorgiRobot)
    robot_driver = WebotsController(
        robot_name='CorgiRobotABAD', # 必須對應 PROTO 的 name
        port=webots_port,
        parameters=[
            {
                'robot_description': os.path.join(package_dir, 'resource', 'corgi.urdf')
            }
        ],
        respawn=True    #maintain connection if Webots restarts
    )

    # 4. 啟動 Force Plate 控制器
    # 注意：在 .wbt 檔案中，你的 Force Plate 物件必須將 controller 設為 <extern>
    # 且 name 必須設為 'Force Plate'
    force_plate_driver = WebotsController(
        robot_name='Force Plate', # 對應 .wbt 中 ForcePlate 的 name
        parameters=[
            {
                'robot_description': os.path.join(package_dir, 'resource', 'force_plate.urdf')
            }
        ],
        respawn=True
    )

    # 5. 啟動 Corgi 控制面板
    control_panel = Node(
        package='corgi_panel',
        executable='corgi_control_panel',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([
        launch.actions.SetEnvironmentVariable(name='USER', value=launch_user),
        launch.actions.SetEnvironmentVariable(name='USERNAME', value=launch_user),
        webots,
        robot_driver,
        force_plate_driver,
        control_panel,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])