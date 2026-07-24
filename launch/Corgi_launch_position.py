"""Thin wrapper around Corgi_launch.py that forces motor_mode='position'.

用途：驗證軌跡規劃（IK/FK/gait timing）本身是否正確，排除馬達扭矩控制的動態誤差。
不重複任何 Webots/Node 設定，僅 include 主 launch 檔並帶入 motor_mode 參數。
"""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    package_dir = get_package_share_directory('corgi_sim')
    main_launch = os.path.join(package_dir, 'launch', 'Corgi_launch.py')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(main_launch),
            launch_arguments={'motor_mode': 'position'}.items(),
        )
    ])
