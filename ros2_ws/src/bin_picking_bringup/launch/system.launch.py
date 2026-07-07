"""系统总启动：相机 + UR5 + MoveIt2 + 手眼TF + 感知 + 抓取。

用法示例:
  ros2 launch bin_picking_bringup system.launch.py \
      robot_ip:=192.168.0.11 \
      cad_model_path:=/home/tao/ros2_ws/src/bin_picking_description/meshes/part.stl

各组件可用 enable_* 参数单独开关，便于分阶段调试。
手眼标定外参(base_link -> camera_link)从 config/handeye_result.yaml 读取，
首次运行前请先用 easy_handeye2 标定并把结果填进去（见 docs）。
"""

import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            GroupAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _load_handeye(bringup_share):
    """读取手眼标定外参；缺省给一个明显占位值。"""
    path = os.path.join(bringup_share, 'config', 'handeye_result.yaml')
    default = ['0.5', '0.0', '0.8', '0.0', '1.0', '0.0', '0.0',
               'base_link', 'camera_link']
    try:
        with open(path) as f:
            d = yaml.safe_load(f)['handeye']
        return [str(d['x']), str(d['y']), str(d['z']),
                str(d['qx']), str(d['qy']), str(d['qz']), str(d['qw']),
                str(d['parent_frame']), str(d['child_frame'])]
    except Exception:
        return default


def generate_launch_description():
    bringup_share = get_package_share_directory('bin_picking_bringup')
    perception_share = get_package_share_directory('bin_picking_perception')
    grasp_share = get_package_share_directory('bin_picking_grasp')

    args = [
        DeclareLaunchArgument('ur_type', default_value='ur5'),
        DeclareLaunchArgument('robot_ip', default_value='192.168.0.11'),
        # RTDE 握手超时(ms)。网络延迟大时驱动会因握手超时 SIGABRT，放宽到 500 兜底。
        DeclareLaunchArgument('rtde_config_package_timeout', default_value='500'),
        DeclareLaunchArgument('cad_model_path', default_value=''),
        DeclareLaunchArgument('enable_robot', default_value='true'),
        DeclareLaunchArgument('enable_camera', default_value='true'),
        DeclareLaunchArgument('enable_moveit', default_value='true'),
        DeclareLaunchArgument('enable_perception', default_value='true'),
        DeclareLaunchArgument('enable_grasp', default_value='true'),
    ]

    # --- UR5 驱动（描述文件换成 UR5+夹爪组合，让 TF/robot_description 含夹爪连杆）---
    ur_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('ur_robot_driver'), 'launch', 'ur_control.launch.py'])),
        launch_arguments={
            'ur_type': LaunchConfiguration('ur_type'),
            'robot_ip': LaunchConfiguration('robot_ip'),
            'rtde_config_package_timeout': LaunchConfiguration(
                'rtde_config_package_timeout'),
            'description_package': 'bin_picking_description',
            'description_file': 'ur5_with_gripper_control.xacro',
            'launch_rviz': 'false',
        }.items(),
        condition=IfCondition(LaunchConfiguration('enable_robot')))

    # --- MoveIt2（UR5+夹爪组合模型，自建 move_group + RViz）---
    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('bin_picking_bringup'), 'launch', 'moveit.launch.py'])),
        launch_arguments={
            'ur_type': LaunchConfiguration('ur_type'),
            'launch_rviz': 'true',
        }.items(),
        condition=IfCondition(LaunchConfiguration('enable_moveit')))

    # --- Gemini2 相机 ---
    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('orbbec_camera'), 'launch', 'gemini2.launch.py'])),
        condition=IfCondition(LaunchConfiguration('enable_camera')))

    # --- 手眼外参 静态 TF (base_link -> camera_link) ---
    handeye = _load_handeye(bringup_share)
    static_tf_camera = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='handeye_static_tf',
        arguments=['--x', handeye[0], '--y', handeye[1], '--z', handeye[2],
                   '--qx', handeye[3], '--qy', handeye[4], '--qz', handeye[5],
                   '--qw', handeye[6],
                   '--frame-id', handeye[7], '--child-frame-id', handeye[8]],
        condition=IfCondition(LaunchConfiguration('enable_camera')))

    # --- 感知 ---
    perception = Node(
        package='bin_picking_perception', executable='perception_node',
        name='perception_node', output='screen',
        parameters=[
            os.path.join(perception_share, 'config', 'perception_params.yaml'),
            {'cad_model_path': LaunchConfiguration('cad_model_path')},
        ],
        condition=IfCondition(LaunchConfiguration('enable_perception')))

    # --- 抓取链 ---
    grasp_params = os.path.join(grasp_share, 'config', 'grasp_params.yaml')
    grasp_group = GroupAction([
        Node(package='bin_picking_grasp', executable='gripper_driver',
             name='gripper_driver', output='screen', parameters=[grasp_params]),
        Node(package='bin_picking_grasp', executable='grasp_planner',
             name='grasp_planner', output='screen', parameters=[grasp_params]),
        Node(package='bin_picking_grasp', executable='inhand_estimator',
             name='inhand_estimator', output='screen', parameters=[grasp_params]),
        Node(package='bin_picking_grasp', executable='grasp_executor',
             name='grasp_executor', output='screen', parameters=[grasp_params]),
        Node(package='bin_picking_grasp', executable='pick_loop',
             name='pick_loop', output='screen', parameters=[grasp_params]),
    ], condition=IfCondition(LaunchConfiguration('enable_grasp')))

    return LaunchDescription(args + [
        ur_control, moveit, camera, static_tf_camera,
        perception, grasp_group,
    ])
