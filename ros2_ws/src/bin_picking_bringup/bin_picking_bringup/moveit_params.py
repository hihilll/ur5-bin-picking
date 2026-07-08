"""MoveIt2 参数组装（阶段3 MoveIt 集成）。

为什么自己组装而不用 ur_moveit_config/launch/ur_moveit.launch.py：
  官方 launch 把 SRDF/kinematics 固定到 ur_moveit_config 包，无法换成
  "UR5+夹爪"的组合模型。这里按官方 humble launch 的同一套逻辑组参数，
  仅把 URDF/SRDF 换成 bin_picking_description 的组合文件，其余
  (kinematics/joint_limits/ompl_planning) 仍从 ur_moveit_config 读取，
  读不到时用内置兜底值（与官方 humble 内容一致）。

供 launch/moveit.launch.py 使用；grasp_executor 本身不再需要这些参数
（它已改为纯 action/service 客户端，见 grasp_executor.py 顶部说明）。
"""

from __future__ import annotations

import os

import yaml
from ament_index_python.packages import get_package_share_directory

UR_JOINTS = [
    'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
    'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint',
]

# 与 ur_moveit_config(humble) ur_moveit.launch.py 相同的请求适配器链
_REQUEST_ADAPTERS = (
    'default_planner_request_adapters/AddTimeOptimalParameterization '
    'default_planner_request_adapters/FixWorkspaceBounds '
    'default_planner_request_adapters/FixStartStateBounds '
    'default_planner_request_adapters/FixStartStateCollision '
    'default_planner_request_adapters/FixStartStatePathConstraints'
)


def _unwrap_ros_params(data):
    """ur_moveit_config 里部分 yaml 是 ROS2 参数文件格式(顶层 '/**:' ->
    'ros__parameters:')，需剥到里层裸内容才能作 MoveIt 参数用；
    其余是裸 yaml(如 joint_limits/controllers/ompl_planning)，原样返回。

    ⚠️ 关键：kinematics.yaml 是带包裹格式，不剥的话 move_group 收不到
    robot_description_kinematics，ur_manipulator 组就没有 IK 求解器，
    位姿目标会 'Unable to construct goal representation'（关节目标不受影响，
    故 RViz 手动拖动 Plan 能过、执行器发的位姿目标过不了）。"""
    if isinstance(data, dict):
        for k in ('/**', '/*', 'move_group', '/move_group'):
            v = data.get(k)
            if isinstance(v, dict) and 'ros__parameters' in v:
                return v['ros__parameters']
        if 'ros__parameters' in data:
            return data['ros__parameters']
    return data


def _load_yaml(package: str, rel_path: str):
    """读包内 yaml（自动剥 ROS2 参数文件包裹）；不存在/解析失败返回 None。"""
    try:
        path = os.path.join(get_package_share_directory(package), rel_path)
        with open(path) as f:
            return _unwrap_ros_params(yaml.safe_load(f))
    except Exception:  # noqa: BLE001
        return None


def robot_description(ur_type: str = 'ur5') -> dict:
    """UR5+夹爪 URDF（含 ros2_control 段；对 move_group 只用几何/关节部分）。"""
    import xacro
    path = os.path.join(
        get_package_share_directory('bin_picking_description'),
        'urdf', 'ur5_with_gripper_control.xacro')
    doc = xacro.process_file(
        path, mappings={'ur_type': ur_type, 'name': 'ur'})
    return {'robot_description': doc.toxml()}


def robot_description_semantic() -> dict:
    """UR5+夹爪 SRDF。"""
    import xacro
    path = os.path.join(
        get_package_share_directory('bin_picking_description'),
        'srdf', 'ur5_with_gripper.srdf.xacro')
    doc = xacro.process_file(path, mappings={'name': 'ur', 'prefix': ''})
    return {'robot_description_semantic': doc.toxml()}


def robot_description_kinematics() -> dict:
    kin = _load_yaml('ur_moveit_config', 'config/kinematics.yaml')
    # 剥掉 ros__parameters 后仍含一层 'robot_description_kinematics'，取其值，
    # 使返回结构为 {'robot_description_kinematics': {'ur_manipulator': {...}}}
    if isinstance(kin, dict) and 'robot_description_kinematics' in kin:
        kin = kin['robot_description_kinematics']
    if not kin:  # 兜底：KDL（与官方一致）
        kin = {'ur_manipulator': {
            'kinematics_solver': 'kdl_kinematics_plugin/KDLKinematicsPlugin',
            'kinematics_solver_search_resolution': 0.005,
            'kinematics_solver_timeout': 0.005,
        }}
    return {'robot_description_kinematics': kin}


def robot_description_planning() -> dict:
    """关节加速度限制（UR 固件不带加速度限制，MoveIt 时间参数化需要）。"""
    limits = _load_yaml('ur_moveit_config', 'config/joint_limits.yaml')
    if limits is None:  # 兜底：与官方 humble joint_limits.yaml 相同
        limits = {'joint_limits': {j: {
            'has_acceleration_limits': True, 'max_acceleration': 5.0,
        } for j in UR_JOINTS}}
    return {'robot_description_planning': limits}


def ompl_pipeline() -> dict:
    """OMPL 规划管线。humble 的 move_group 默认管线命名空间是 'move_group'，
    与官方 ur_moveit.launch.py 保持同一写法。"""
    cfg = {
        'planning_plugin': 'ompl_interface/OMPLPlanner',
        'request_adapters': _REQUEST_ADAPTERS,
        'start_state_max_bounds_error': 0.1,
    }
    ompl_yaml = _load_yaml('ur_moveit_config', 'config/ompl_planning.yaml')
    if ompl_yaml:
        cfg.update(ompl_yaml)
    return {'move_group': cfg}


def controllers() -> dict:
    """MoveIt -> ros2_control 控制器映射（与官方 ur_moveit_config 同）。

    两个轨迹控制器都列出、scaled 为默认。真机与 UR fake hardware 默认
    激活的都是 scaled_joint_trajectory_controller，故此配置两者通用；
    若改用 joint_trajectory_controller，把它的 default 设 true 即可。
    优先读 ur_moveit_config/config/controllers.yaml，读不到用内置兜底。
    """
    cm = {'moveit_controller_manager':
          'moveit_simple_controller_manager/MoveItSimpleControllerManager'}
    yaml_cfg = _load_yaml('ur_moveit_config', 'config/controllers.yaml')
    if yaml_cfg is None:  # 兜底：与官方 controllers.yaml 一致
        yaml_cfg = {
            'controller_names': [
                'scaled_joint_trajectory_controller',
                'joint_trajectory_controller'],
            'scaled_joint_trajectory_controller': {
                'action_ns': 'follow_joint_trajectory',
                'type': 'FollowJointTrajectory',
                'default': True, 'joints': UR_JOINTS},
            'joint_trajectory_controller': {
                'action_ns': 'follow_joint_trajectory',
                'type': 'FollowJointTrajectory',
                'default': False, 'joints': UR_JOINTS},
        }
    cm['moveit_simple_controller_manager'] = yaml_cfg
    return cm


def trajectory_execution() -> dict:
    return {
        'moveit_manage_controllers': False,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.01,
        # 与官方一致：UR 有速度缩放，执行时长不做硬监控
        'trajectory_execution.execution_duration_monitoring': False,
    }


def planning_scene_monitor() -> dict:
    return {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }


def move_group_parameters(ur_type: str = 'ur5',
                          use_sim_time: bool = False) -> list:
    """move_group 节点的完整参数列表。"""
    return [
        robot_description(ur_type),
        robot_description_semantic(),
        {'publish_robot_description_semantic': True},
        robot_description_kinematics(),
        robot_description_planning(),
        ompl_pipeline(),
        trajectory_execution(),
        controllers(),
        planning_scene_monitor(),
        {'use_sim_time': use_sim_time},
    ]


def rviz_parameters(ur_type: str = 'ur5') -> list:
    """RViz MotionPlanning 插件所需参数（与官方 launch 相同的子集）。"""
    return [
        robot_description(ur_type),
        robot_description_semantic(),
        robot_description_kinematics(),
        robot_description_planning(),
        ompl_pipeline(),
    ]
