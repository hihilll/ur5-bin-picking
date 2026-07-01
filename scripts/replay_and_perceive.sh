#!/usr/bin/env bash
# 回放录好的点云 rosbag 并（可选）启动感知节点，离线反复调参（阶段S.4）。
# 无需相机/机器人，纯靠录包驱动感知，改 perception_params.yaml 后重跑即可对比。
#
# 用法:
#   bash scripts/replay_and_perceive.sh <bag目录> [CAD路径]
# 例:
#   # 只回放（自己另开终端跑感知）:
#   bash scripts/replay_and_perceive.sh bin_scene_01
#   # 回放 + 自动起感知:
#   bash scripts/replay_and_perceive.sh bin_scene_01 /abs/part.stl
#
# 回放循环播放(--loop)，方便边看 RViz 边调参。Ctrl-C 结束。
set -e

BAG="${1:?用法: replay_and_perceive.sh <bag目录> [CAD路径]}"
CAD="${2:-}"

source /opt/ros/humble/setup.bash
if [ -f install/setup.bash ]; then source install/setup.bash; fi
if [ -f ros2_ws/install/setup.bash ]; then source ros2_ws/install/setup.bash; fi

echo "==== 回放 rosbag: $BAG (循环) ===="
ros2 bag play "$BAG" --loop &
BAG_PID=$!
trap 'kill $BAG_PID 2>/dev/null || true' EXIT

if [ -n "$CAD" ]; then
  echo "==== 启动感知节点 (cad=$CAD) ===="
  echo "另开终端可看: ros2 topic echo /detected_objects ; 或 RViz 加 /detected_markers"
  ros2 launch bin_picking_perception perception.launch.py "cad_model_path:=$CAD"
else
  echo "仅回放。另开终端启动感知，例如："
  echo "  ros2 launch bin_picking_perception perception.launch.py cad_model_path:=/abs/part.stl"
  echo "（Ctrl-C 结束回放）"
  wait $BAG_PID
fi
