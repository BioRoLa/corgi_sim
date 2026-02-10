import os
import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController

def generate_launch_description():
    package_dir = get_package_share_directory('corgi_sim')
    
    # 1. 設定 Webots 世界檔路徑
    world_path = os.path.join(package_dir, 'worlds', "IFS_Proto" + ".wbt") # corgi_origin // IFS_Proto

    # 2. 啟動 Webots
    webots = WebotsLauncher(
        world=world_path,
        ros2_supervisor=False
    )

    # 3. 啟動機器人控制器 (CorgiRobot)
    robot_driver = WebotsController(
        robot_name='CorgiRobot', # 必須對應 PROTO 的 name
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