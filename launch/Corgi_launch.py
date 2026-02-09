import os
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController

def generate_launch_description():
    package_dir = get_package_share_directory('corgi_sim')
    
    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # 1. 設定 Webots 世界檔路徑
    world_path = os.path.join(package_dir, 'worlds', 'IFS_Proto.wbt')

    # 2. 啟動 Webots
    webots = WebotsLauncher(
        world=world_path,
        ros2_supervisor=True,  # Enable supervisor for full control
        mode='realtime'  # Can be 'pause', 'realtime', or 'fast'
    )

    # 3. 啟動機器人控制器 (Corgi Driver - Webots Plugin)
    robot_driver = WebotsController(
        robot_name='CorgiRobot', # 必須對應 PROTO 的 name
        parameters=[
            {
                'robot_description': os.path.join(package_dir, 'resource', 'corgi.urdf'),
                'use_sim_time': use_sim_time,
            }
        ],
        respawn=True    # maintain connection if Webots restarts
    )
    
    # 4. 啟動 Corgi 控制面板
    control_panel = Node(
        package='corgi_panel',
        executable='corgi_control_panel',
        name='corgi_control_panel',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Webots) clock'
        ),
        
        # Nodes
        webots,
        robot_driver,
        control_panel,
        
        # Event handlers
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=Shutdown())],
            )
        )
    ])