"""启动抓取链：夹爪驱动 + 抓取规划 + 取放执行。

注意: grasp_executor 通过 move_group 标准接口规划运动，需先起 MoveIt
      （`bin_picking_bringup moveit.launch.py` 或总 launch）。本文件仅调试抓取链本身；
      未起 MoveIt 时给 grasp_executor 传 simulate:=true 只打印动作序列。
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('bin_picking_grasp')
    params = os.path.join(pkg_share, 'config', 'grasp_params.yaml')

    return LaunchDescription([
        Node(package='bin_picking_grasp', executable='gripper_driver',
             name='gripper_driver', output='screen', parameters=[params]),
        Node(package='bin_picking_grasp', executable='grasp_planner',
             name='grasp_planner', output='screen', parameters=[params]),
        Node(package='bin_picking_grasp', executable='grasp_executor',
             name='grasp_executor', output='screen', parameters=[params]),
    ])
