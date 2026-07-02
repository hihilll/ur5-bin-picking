"""阶段1 手眼标定启动（Eye-to-Hand / easy_handeye2）。

布局：相机固定俯视料框（不装在手上），**标定板贴在机械臂末端法兰**，
机器人带着标定板在相机视野里摆多个姿态，easy_handeye2 解出
  base_link -> camera_link  外参（正是我们要填进 handeye_result.yaml 的量）。

依赖（在 Ubuntu 上装）:
  sudo apt install ros-humble-easy-handeye2 ros-humble-aruco-ros
  （aruco_ros 用来检测标定板并发布 相机->marker 的 TF；也可换 charuco/apriltag）

数据链:
  orbbec 相机 --image/camera_info--> aruco_ros(single) --TF: camera_link->marker-->
  UR 驱动 --TF: base_link->tool0-->  easy_handeye2 采样求解

前置（本 launch 默认不重复起，假定它们已在别的终端跑）:
  ros2 launch orbbec_camera gemini2.launch.py
  ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5 robot_ip:=<真实IP>
  （UR 示教器跑 External Control）
若想让本 launch 顺带把相机/机器人一起起来，加 enable_camera:=true / enable_robot:=true。

用法:
  ros2 launch bin_picking_bringup calibrate_handeye.launch.py \
      marker_id:=26 marker_size:=0.06

标定完成后（在 easy_handeye2 GUI 里 Save）:
  python3 scripts/handeye_to_yaml.py            # 把 .calib 转成 handeye_result.yaml
然后 colcon build 让新外参生效。详见 docs/05_手眼标定.md。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    args = [
        # ---- 标定基本设置 ----
        DeclareLaunchArgument('name', default_value='handeye_ur5_gemini2',
                              description='标定名（easy_handeye2 存档名）'),
        # Eye-to-Hand（相机固定）对应 easy_handeye2 的 eye_on_base
        DeclareLaunchArgument('calibration_type', default_value='eye_on_base'),
        DeclareLaunchArgument('robot_base_frame', default_value='base_link'),
        DeclareLaunchArgument('robot_effector_frame', default_value='tool0'),
        # 相机侧参考系。设成 camera_link，标定直接得到 base_link->camera_link，
        # 与 handeye_result.yaml 对齐，且不与相机驱动内部 TF 冲突。
        DeclareLaunchArgument('camera_link_frame', default_value='camera_link'),
        # 相机图像所在的光学系（aruco 在此系里算 marker 位姿，再经 TF 转到 camera_link）
        DeclareLaunchArgument('camera_optical_frame',
                              default_value='camera_color_optical_frame'),
        DeclareLaunchArgument('marker_frame', default_value='handeye_marker'),

        # ---- 标定板（ArUco 单码）----
        DeclareLaunchArgument('marker_id', default_value='26'),
        DeclareLaunchArgument('marker_size', default_value='0.06',
                              description='ArUco 边长(m)，务必与实际打印尺寸一致'),
        DeclareLaunchArgument('image_topic',
                              default_value='/camera/color/image_raw'),
        DeclareLaunchArgument('camera_info_topic',
                              default_value='/camera/color/camera_info'),

        # ---- 可选：顺带起相机 / 机器人 ----
        DeclareLaunchArgument('enable_camera', default_value='false'),
        DeclareLaunchArgument('enable_robot', default_value='false'),
        DeclareLaunchArgument('ur_type', default_value='ur5'),
        DeclareLaunchArgument('robot_ip', default_value='192.168.0.11'),
        # RTDE 握手超时(ms)。网络延迟大时防止驱动因握手超时 SIGABRT。
        DeclareLaunchArgument('rtde_config_package_timeout', default_value='500'),
    ]

    # --- (可选) 相机 ---
    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('orbbec_camera'), 'launch', 'gemini2.launch.py'])),
        condition=IfCondition(LaunchConfiguration('enable_camera')))

    # --- (可选) UR5 驱动 ---
    ur_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('ur_robot_driver'), 'launch',
            'ur_control.launch.py'])),
        launch_arguments={
            'ur_type': LaunchConfiguration('ur_type'),
            'robot_ip': LaunchConfiguration('robot_ip'),
            'rtde_config_package_timeout': LaunchConfiguration(
                'rtde_config_package_timeout'),
            'launch_rviz': 'false',
        }.items(),
        condition=IfCondition(LaunchConfiguration('enable_robot')))

    # --- 标定板检测：aruco_ros 单码 ---
    # reference_frame=camera_link 让发布的 marker TF 直接挂在 camera_link 下，
    # 于是 easy_handeye2 求得的就是 base_link->camera_link。
    aruco = Node(
        package='aruco_ros', executable='single', name='aruco_single',
        output='screen',
        parameters=[{
            # 显式指定类型：launch 参数默认是字符串，aruco 需要 int/double
            'marker_id': ParameterValue(
                LaunchConfiguration('marker_id'), value_type=int),
            'marker_size': ParameterValue(
                LaunchConfiguration('marker_size'), value_type=float),
            'reference_frame': LaunchConfiguration('camera_link_frame'),
            'camera_frame': LaunchConfiguration('camera_optical_frame'),
            'marker_frame': LaunchConfiguration('marker_frame'),
        }],
        remappings=[
            ('/camera_info', LaunchConfiguration('camera_info_topic')),
            ('/image', LaunchConfiguration('image_topic')),
        ])

    # --- easy_handeye2 标定服务 + GUI ---
    handeye = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('easy_handeye2'), 'launch',
            'calibrate.launch.py'])),
        launch_arguments={
            'name': LaunchConfiguration('name'),
            'calibration_type': LaunchConfiguration('calibration_type'),
            'robot_base_frame': LaunchConfiguration('robot_base_frame'),
            'robot_effector_frame': LaunchConfiguration('robot_effector_frame'),
            'tracking_base_frame': LaunchConfiguration('camera_link_frame'),
            'tracking_marker_frame': LaunchConfiguration('marker_frame'),
        }.items())

    return LaunchDescription(args + [camera, ur_control, aruco, handeye])
