#!/usr/bin/env bash
# 录制真实点云到 rosbag，供离线反复调感知（阶段S.4）。
# 在 Ubuntu 上、相机已启动时运行。
#
# 用法:
#   bash scripts/record_pointcloud.sh [输出名] [点云话题]
# 例:
#   ros2 launch orbbec_camera gemini2.launch.py      # 另一个终端先起相机
#   bash scripts/record_pointcloud.sh bin_scene_01
#
# 录完 Ctrl-C 停止，生成目录 <输出名>/（含 .db3）。
set -e

OUT="${1:-bin_scene_$(date +%Y%m%d_%H%M%S)}"
CLOUD_TOPIC="${2:-/camera/depth_registered/points}"

# 一并录彩色/内参/TF，便于回放时其他节点也能用
TOPICS=(
  "$CLOUD_TOPIC"
  /camera/color/image_raw
  /camera/color/camera_info
  /tf
  /tf_static
)

echo "==== 录制 rosbag ===="
echo "输出目录: $OUT"
echo "话题: ${TOPICS[*]}"
echo "（Ctrl-C 停止录制）"
source /opt/ros/humble/setup.bash
ros2 bag record -o "$OUT" "${TOPICS[@]}"
