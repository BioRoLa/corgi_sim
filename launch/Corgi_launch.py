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
    # 假設您的世界檔還在私人 Repo，您可以直接指向絕對路徑 (開發階段方便)
    # 或者把它複製到 package 的 worlds 資料夾
    # 這裡我們先用一個變數，請修改為您真實的 .wbt 路徑
    world_path = os.path.join(package_dir, 'worlds', "IFS_Proto.wbt")

    # 2. 啟動 Webots
    webots = WebotsLauncher(
        world=world_path,
        ros2_supervisor=False
    )

    # 3. 啟動機器人控制器 (ROS 2 Bridge)
    robot_driver = WebotsController(
        robot_name='CorgiRobot', # 必須對應 PROTO 的 name
        parameters=[
            {
                'robot_description': os.path.join(package_dir, 'resource', 'corgi.urdf')
            }
        ],
        respawn=True    #maintain connection if Webots restarts
    )

    # 4. 啟動 corgi_data_recorder
    data_recorder = Node(
        package='corgi_data_recorder',
        executable='corgi_data_recorder',
        name='corgi_data_recorder',
        output='screen'
    )

    return LaunchDescription([
        webots,
        robot_driver,
        data_recorder,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])