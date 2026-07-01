"""启动抓取链：夹爪驱动 + 抓取规划 + 取放执行。

注意: grasp_executor 依赖 MoveIt2(moveit_py)，通常需配合 UR 的 MoveIt launch 一起跑，
      推荐用 bin_picking_bringup 的总 launch。本文件用于单独调试抓取链。
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
