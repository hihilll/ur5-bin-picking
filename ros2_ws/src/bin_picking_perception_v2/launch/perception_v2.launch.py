"""单独启动阶段二感知客户端（调试用）。

前置：推理服务已在运行（可用 --fake 先测通链路）:
  python3 inference/server.py --fake        # 无 GPU/模型时返回假结果
  python3 inference/server.py               # 正式推理

用法:
  ros2 launch bin_picking_perception_v2 perception_v2.launch.py
  # 只起某一个:
  ros2 launch bin_picking_perception_v2 perception_v2.launch.py \
      enable_sam6d:=true enable_inhand:=false enable_grasp:=false
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('bin_picking_perception_v2')
    params = os.path.join(pkg_share, 'config', 'perception_v2_params.yaml')

    args = [
        DeclareLaunchArgument('enable_sam6d', default_value='true'),
        DeclareLaunchArgument('enable_inhand', default_value='false'),
        DeclareLaunchArgument('enable_grasp', default_value='false'),
    ]

    sam6d = Node(
        package='bin_picking_perception_v2', executable='sam6d_client',
        name='sam6d_client', output='screen', parameters=[params],
        condition=IfCondition(LaunchConfiguration('enable_sam6d')))

    inhand = Node(
        package='bin_picking_perception_v2', executable='foundationpose_client',
        name='foundationpose_client', output='screen', parameters=[params],
        condition=IfCondition(LaunchConfiguration('enable_inhand')))

    grasp = Node(
        package='bin_picking_perception_v2', executable='grasp_client',
        name='grasp_client', output='screen', parameters=[params],
        condition=IfCondition(LaunchConfiguration('enable_grasp')))

    return LaunchDescription(args + [sam6d, inhand, grasp])
