"""单独启动感知节点（用于阶段二调试）。

用法:
  ros2 launch bin_picking_perception perception.launch.py \
      cad_model_path:=/abs/path/to/part.stl
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('bin_picking_perception')
    params_file = os.path.join(pkg_share, 'config', 'perception_params.yaml')

    cad_arg = DeclareLaunchArgument(
        'cad_model_path', default_value='',
        description='CAD 模型绝对路径 (.stl/.obj/.ply)')

    perception = Node(
        package='bin_picking_perception',
        executable='perception_node',
        name='perception_node',
        output='screen',
        parameters=[
            params_file,
            {'cad_model_path': LaunchConfiguration('cad_model_path')},
        ],
    )

    return LaunchDescription([cad_arg, perception])
