"""无模型抓取测试总启动（不依赖 CAD）。

用途：手头只有螺丝/螺母等小物件、没有 CAD 模型时，跑通
  相机 -> 手眼TF -> 无模型抓取 -> MoveIt2 -> 夹爪 整条闭环。

与 system.launch.py 的区别：
  用 `model_free_grasp`(点云直接算俯视抓取) 替代 `perception_node + grasp_planner`，
  其余(UR驱动/MoveIt/相机/手眼TF/夹爪/执行器/状态机)完全一致。两条链路互不影响。

用法示例:
  ros2 launch bin_picking_bringup model_free_test.launch.py \
      robot_ip:=192.168.0.11

分阶段调试:
  只看抓取候选(不动机械臂): enable_robot:=false enable_moveit:=false
    然后在 RViz 里看 /grasp_markers 是否对准物体。
  确认无误后再开 robot/moveit，调用 /pick_place/run 或 /bin_picking/start。
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
        DeclareLaunchArgument('rtde_config_package_timeout', default_value='500'),
        DeclareLaunchArgument('enable_robot', default_value='true'),
        DeclareLaunchArgument('enable_camera', default_value='true'),
        DeclareLaunchArgument('enable_moveit', default_value='true'),
        DeclareLaunchArgument('enable_grasp', default_value='true'),
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

    # --- 无模型抓取（替代 perception_node + grasp_planner）---
    model_free = Node(
        package='bin_picking_perception', executable='model_free_grasp',
        name='model_free_grasp', output='screen',
        parameters=[os.path.join(
            perception_share, 'config', 'model_free_params.yaml')])

    # --- 抓取执行链（与 system.launch 相同，去掉 grasp_planner）---
    grasp_params = os.path.join(grasp_share, 'config', 'grasp_params.yaml')
    grasp_group = GroupAction([
        Node(package='bin_picking_grasp', executable='gripper_driver',
             name='gripper_driver', output='screen', parameters=[grasp_params]),
        Node(package='bin_picking_grasp', executable='grasp_executor',
             name='grasp_executor', output='screen', parameters=[grasp_params]),
        Node(package='bin_picking_grasp', executable='pick_loop',
             name='pick_loop', output='screen', parameters=[grasp_params]),
    ], condition=IfCondition(LaunchConfiguration('enable_grasp')))

    return LaunchDescription(args + [
        ur_control, ur_moveit, camera, static_tf_camera,
        model_free, grasp_group,
    ])
