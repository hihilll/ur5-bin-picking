"""阶段二系统总启动：相机 + UR5 + MoveIt2 + 手眼TF + 大模型感知 + 抓取。

与 system.launch.py 的区别：感知层换成大模型推理客户端
（bin_picking_perception_v2），执行链（executor/pick_loop/夹爪）完全复用。

前置：推理服务已在另一终端/容器运行（见 docs/08）:
  python3 inference/server.py --fake      # 无模型时先测链路
  python3 inference/server.py --cad /abs/part.stl --templates ... --grasp-ckpt ...

两条抓取路线（**二选一**，都发 /grasp_candidates，同时开会互相覆盖）:
  A. 位姿路线(默认): sam6d_client -> /detected_objects -> grasp_planner
     零件位姿已知 -> 用 CAD 标注抓取点，可控制抓哪个面（利于后续装配）
  B. 学习抓取路线:  grasp_client 直接出 /grasp_candidates
     enable_pose_route:=false enable_learned_grasp:=true
     杂乱堆叠更鲁棒，但不保证抓取面向

在手补偿（阶段4，路线 A/B 均可叠加）:
  enable_inhand:=true 时起 foundationpose_client + inhand_estimator，
  inhand_estimator 的 /detected_objects 已 remap 到 /detected_objects_inhand，
  grasp_executor 的 enable_inhand 参数同步打开。

用法示例:
  ros2 launch bin_picking_bringup system_v2.launch.py robot_ip:=192.168.0.11
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
    v2_share = get_package_share_directory('bin_picking_perception_v2')
    grasp_share = get_package_share_directory('bin_picking_grasp')

    args = [
        DeclareLaunchArgument('ur_type', default_value='ur5'),
        DeclareLaunchArgument('robot_ip', default_value='192.168.0.11'),
        # RTDE 握手超时(ms)。网络延迟大时驱动会因握手超时 SIGABRT，放宽到 500 兜底。
        DeclareLaunchArgument('rtde_config_package_timeout', default_value='500'),
        DeclareLaunchArgument('enable_robot', default_value='true'),
        DeclareLaunchArgument('enable_camera', default_value='true'),
        DeclareLaunchArgument('enable_moveit', default_value='true'),
        DeclareLaunchArgument('enable_grasp', default_value='true'),
        # 抓取路线二选一
        DeclareLaunchArgument('enable_pose_route', default_value='true'),
        DeclareLaunchArgument('enable_learned_grasp', default_value='false'),
        # 阶段4 在手补偿
        DeclareLaunchArgument('enable_inhand', default_value='false'),
        # 相机时间戳用主机时钟。默认 device 时钟落后系统几百秒，点云经 TF 转基座系
        # 时必报 extrapolation（手眼标定阶段真机已踩坑）。
        DeclareLaunchArgument('camera_time_domain', default_value='system'),
    ]

    # --- UR5 驱动 ---
    ur_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('ur_robot_driver'), 'launch', 'ur_control.launch.py'])),
        launch_arguments={
            'ur_type': LaunchConfiguration('ur_type'),
            'robot_ip': LaunchConfiguration('robot_ip'),
            'rtde_config_package_timeout': LaunchConfiguration(
                'rtde_config_package_timeout'),
            'launch_rviz': 'false',
        }.items(),
        condition=IfCondition(LaunchConfiguration('enable_robot')))

    # --- MoveIt2 (UR) ---
    ur_moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('ur_moveit_config'), 'launch', 'ur_moveit.launch.py'])),
        launch_arguments={
            'ur_type': LaunchConfiguration('ur_type'),
            'launch_rviz': 'true',
        }.items(),
        condition=IfCondition(LaunchConfiguration('enable_moveit')))

    # --- Gemini2 相机 ---
    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('orbbec_camera'), 'launch', 'gemini2.launch.py'])),
        launch_arguments={
            'time_domain': LaunchConfiguration('camera_time_domain'),
        }.items(),
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

    # --- 感知 v2 ---
    v2_params = os.path.join(v2_share, 'config', 'perception_v2_params.yaml')
    grasp_params = os.path.join(grasp_share, 'config', 'grasp_params.yaml')

    # 路线 A: SAM-6D 位姿 + CAD 标注抓取规划
    pose_route = GroupAction([
        Node(package='bin_picking_perception_v2', executable='sam6d_client',
             name='sam6d_client', output='screen', parameters=[v2_params]),
        Node(package='bin_picking_grasp', executable='grasp_planner',
             name='grasp_planner', output='screen', parameters=[grasp_params]),
    ], condition=IfCondition(LaunchConfiguration('enable_pose_route')))

    # 路线 B: 学习型抓取直接出候选
    learned_grasp = Node(
        package='bin_picking_perception_v2', executable='grasp_client',
        name='grasp_client', output='screen', parameters=[v2_params],
        condition=IfCondition(LaunchConfiguration('enable_learned_grasp')))

    # 阶段4: FoundationPose 在手 + inhand_estimator(输入 remap 到在手话题)
    inhand_group = GroupAction([
        Node(package='bin_picking_perception_v2',
             executable='foundationpose_client',
             name='foundationpose_client', output='screen',
             parameters=[v2_params]),
        Node(package='bin_picking_grasp', executable='inhand_estimator',
             name='inhand_estimator', output='screen',
             parameters=[grasp_params],
             remappings=[('/detected_objects', '/detected_objects_inhand')]),
    ], condition=IfCondition(LaunchConfiguration('enable_inhand')))

    # --- 执行链（与 system.launch 相同）---
    exec_group = GroupAction([
        Node(package='bin_picking_grasp', executable='gripper_driver',
             name='gripper_driver', output='screen', parameters=[grasp_params]),
        Node(package='bin_picking_grasp', executable='grasp_executor',
             name='grasp_executor', output='screen',
             parameters=[grasp_params,
                         {'enable_inhand':
                          LaunchConfiguration('enable_inhand')}]),
        Node(package='bin_picking_grasp', executable='pick_loop',
             name='pick_loop', output='screen', parameters=[grasp_params]),
    ], condition=IfCondition(LaunchConfiguration('enable_grasp')))

    return LaunchDescription(args + [
        ur_control, ur_moveit, camera, static_tf_camera,
        pose_route, learned_grasp, inhand_group, exec_group,
    ])
