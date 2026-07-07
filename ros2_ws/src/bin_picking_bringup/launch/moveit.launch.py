"""MoveIt2 启动（UR5 + 沃姆 EPGC-50 夹爪 组合模型）——阶段3 MoveIt 集成。

替代官方 ur_moveit_config/launch/ur_moveit.launch.py：
  官方 launch 的 SRDF 固定为裸 UR，无法让 MoveIt 感知夹爪几何（自碰撞/避障）
  与指尖 TCP(gripper_grasp_tcp)。本 launch 用 bin_picking_description 的
  UR5+夹爪 URDF/SRDF 起 move_group，其余参数与官方 humble 版一致
  （组装逻辑见 bin_picking_bringup/moveit_params.py）。

前置：ur_control.launch.py 已在跑，且 description_file 也用带夹爪的
  ur5_with_gripper_control.xacro（system.launch.py / sim.launch.py 已配好），
  否则 /joint_states 齐全但 TF 里没有夹爪连杆。

单独用法:
  ros2 launch bin_picking_bringup moveit.launch.py                    # 真机
  ros2 launch bin_picking_bringup moveit.launch.py launch_rviz:=false # 不开 RViz
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node

from bin_picking_bringup.moveit_params import (
    move_group_parameters, rviz_parameters)


def _launch_setup(context, *args, **kwargs):
    ur_type = context.launch_configurations['ur_type']
    launch_rviz = context.launch_configurations['launch_rviz'].lower() == 'true'
    use_sim_time = (
        context.launch_configurations['use_sim_time'].lower() == 'true')

    nodes = [Node(
        package='moveit_ros_move_group',
        executable='move_group',
        name='move_group',
        output='screen',
        parameters=move_group_parameters(
            ur_type=ur_type, use_sim_time=use_sim_time),
    )]

    if launch_rviz:
        rviz_config = os.path.join(
            get_package_share_directory('bin_picking_bringup'),
            'rviz', 'moveit.rviz')
        nodes.append(Node(
            package='rviz2', executable='rviz2', name='rviz2_moveit',
            output='log', arguments=['-d', rviz_config],
            parameters=rviz_parameters(ur_type=ur_type)))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('ur_type', default_value='ur5'),
        DeclareLaunchArgument('launch_rviz', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        OpaqueFunction(function=_launch_setup),
    ])
