"""仿真启动：UR5 用 fake hardware（无真机）+ MoveIt2 + 夹爪模拟 + 抓取链。

用于在没有真实机械臂/相机时验证运动与取放逻辑：
  - UR 驱动用 use_fake_hardware:=true，不需要真实 robot_ip
  - 夹爪 simulate:=true（只打印）
  - 相机/感知默认关闭；可用 publish_test_grasp 手动喂一个抓取候选

用法:
  ros2 launch bin_picking_bringup sim.launch.py
  # 另开终端喂一个测试抓取，再触发执行:
  ros2 run bin_picking_grasp publish_test_grasp
  ros2 service call /pick_place/run std_srvs/srv/Trigger {}
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    grasp_share = get_package_share_directory('bin_picking_grasp')
    grasp_params = os.path.join(grasp_share, 'config', 'grasp_params.yaml')

    args = [
        DeclareLaunchArgument('ur_type', default_value='ur5'),
        DeclareLaunchArgument('enable_grasp', default_value='true'),
    ]

    # UR5 fake hardware（描述文件换成 UR5+夹爪组合，让 TF/robot_description 含夹爪连杆）
    ur_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('ur_robot_driver'), 'launch', 'ur_control.launch.py'])),
        launch_arguments={
            'ur_type': LaunchConfiguration('ur_type'),
            'robot_ip': 'yyy.yyy.yyy.yyy',     # fake 模式忽略
            'use_fake_hardware': 'true',
            'description_package': 'bin_picking_description',
            'description_file': 'ur5_with_gripper_control.xacro',
            # UR 本体配置 yaml 仍从 ur_description 读：description_package 换成本包后，
            # 这些参数的默认路径会跟着指向本包(config/ur5/*.yaml 不存在)，导致 xacro 报
            # "No such file: visual_parameters.yaml"。显式指回 ur_description。
            'joint_limit_params': PathJoinSubstitution([FindPackageShare('ur_description'),
                'config', LaunchConfiguration('ur_type'), 'joint_limits.yaml']),
            'kinematics_params': PathJoinSubstitution([FindPackageShare('ur_description'),
                'config', LaunchConfiguration('ur_type'), 'default_kinematics.yaml']),
            'physical_params': PathJoinSubstitution([FindPackageShare('ur_description'),
                'config', LaunchConfiguration('ur_type'), 'physical_parameters.yaml']),
            'visual_params': PathJoinSubstitution([FindPackageShare('ur_description'),
                'config', LaunchConfiguration('ur_type'), 'visual_parameters.yaml']),
            'launch_rviz': 'false',
            'initial_joint_controller': 'scaled_joint_trajectory_controller',
        }.items())

    # MoveIt2 + RViz（UR5+夹爪组合模型，自建 move_group）
    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('bin_picking_bringup'), 'launch', 'moveit.launch.py'])),
        launch_arguments={
            'ur_type': LaunchConfiguration('ur_type'),
            'launch_rviz': 'true',
        }.items())

    # 夹爪（模拟）+ 执行器
    gripper = Node(
        package='bin_picking_grasp', executable='gripper_driver',
        name='gripper_driver', output='screen',
        parameters=[grasp_params, {'simulate': True}],
        condition=IfCondition(LaunchConfiguration('enable_grasp')))
    executor = Node(
        package='bin_picking_grasp', executable='grasp_executor',
        name='grasp_executor', output='screen', parameters=[grasp_params],
        condition=IfCondition(LaunchConfiguration('enable_grasp')))

    return LaunchDescription(args + [ur_control, moveit, gripper, executor])
