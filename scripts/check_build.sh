#!/usr/bin/env bash
# 一键编译自检（在 Ubuntu 上、ros2_ws 上一级目录或 ros2_ws 内运行均可）
# 用法: bash scripts/check_build.sh
set -e

echo "==== [1/6] 定位工作空间 ===="
if [ -d "ros2_ws/src" ]; then
  WS="$(cd ros2_ws && pwd)"
elif [ -d "src" ]; then
  WS="$(pwd)"
else
  echo "✗ 找不到 ros2_ws/src 或 src，请在项目根或 ros2_ws 内运行"; exit 1
fi
echo "工作空间: $WS"

echo "==== [2/6] source ROS2 ===="
source /opt/ros/humble/setup.bash
echo "ROS_DISTRO=$ROS_DISTRO"

echo "==== [3/6] 安装 Python 第三方依赖 ===="
pip install -q open3d opencv-contrib-python numpy scipy pymodbus pyserial || \
  echo "⚠ pip 安装有问题，请手动检查"

echo "==== [4/6] rosdep + colcon build ===="
cd "$WS"
rosdep install --from-paths src --ignore-src -r -y || echo "⚠ rosdep 部分失败，继续"
colcon build --symlink-install
source install/setup.bash

echo "==== [5/6] 检查包与接口 ===="
echo "-- 本项目包 --"
ros2 pkg list | grep bin_picking || { echo "✗ 未发现 bin_picking 包"; exit 1; }
echo "-- 自定义接口 --"
ros2 interface show bin_picking_interfaces/msg/ObjectPose > /dev/null && echo "✓ ObjectPose"
ros2 interface show bin_picking_interfaces/msg/GraspCandidate > /dev/null && echo "✓ GraspCandidate"
ros2 interface show bin_picking_interfaces/srv/SetGripper > /dev/null && echo "✓ SetGripper"

echo "-- MoveIt 组合模型（UR5+夹爪 URDF/SRDF）--"
DESC="$(ros2 pkg prefix bin_picking_description)/share/bin_picking_description"
xacro "$DESC/urdf/ur5_with_gripper_control.xacro" ur_type:=ur5 > /tmp/ur5g.urdf \
  && check_urdf /tmp/ur5g.urdf > /dev/null && echo "✓ ur5_with_gripper_control.xacro" \
  || echo "✗ 组合 URDF 处理失败（检查 ur_description 是否安装）"
xacro "$DESC/srdf/ur5_with_gripper.srdf.xacro" name:=ur prefix:= > /tmp/ur5g.srdf \
  && echo "✓ ur5_with_gripper.srdf.xacro" \
  || echo "✗ 组合 SRDF 处理失败（检查 ur_moveit_config 是否安装）"

echo "==== [6/6] 节点自检（模拟模式，3秒后退出） ===="
timeout 3 ros2 run bin_picking_grasp gripper_driver --ros-args -p simulate:=true || true
# 执行器纯逻辑模式（不连 MoveIt，只打印动作序列）
timeout 3 ros2 run bin_picking_grasp grasp_executor --ros-args -p simulate:=true || true
echo ""
echo "✅ 自检完成。若以上无 ✗ 报错，代码与依赖就绪。"
echo "   下一步可跑仿真: ros2 launch bin_picking_bringup sim.launch.py"
