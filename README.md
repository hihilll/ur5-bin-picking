# UR5 无序抓取（Bin Picking）项目

UR5 机械臂 + 奥比中光 Gemini2 相机，从料框中抓取无序混摆的 3D 打印零件，放到指定位置（含高精度定向装配）。

> 完整技术方案见 [`方案设计.md`](方案设计.md)。

## 开发工作流

- **本机（Windows）**：编写、组织 ROS2 代码。**不要**在此尝试 `colcon` / `ros2` 命令。
- **目标机（Ubuntu 22.04 + ROS2 Humble）**：拷贝 `ros2_ws/` 过去，`colcon build` 并连接真实硬件测试。

## 目录结构

```
robotarm/
├── 方案设计.md                # 总体技术方案（三阶段路线）
├── README.md                  # 本文件
├── docs/                      # 分阶段文档
│   ├── 00_开发计划.md         # 总体计划 + 各步骤是否依赖硬件
│   ├── 01_环境搭建.md         # 阶段0：Ubuntu + ROS2 环境搭建
│   ├── 02_运行说明.md         # 各节点启动命令 + 待填参数
│   ├── 03_测试步骤.md         # 自底向上详细测试流程
│   ├── 04_夹爪URDF集成.md     # 沃姆夹爪装到 UR5
│   ├── 05_手眼标定.md         # 阶段1 标定完整流程
│   ├── 06_操作手册.md         # 从编译到跑通的命令速查（含标定）
│   ├── 07_无模型抓取测试.md   # 无 CAD 抓螺丝螺母（model_free_grasp）
│   ├── 08_阶段二大模型接入.md # SAM-6D/FoundationPose/学习抓取 集成指南
│   ├── 09_整机测试教程.md     # ★硬件安装接线 + 全流程测试命令一站式教程
│   └── 10_移动操作方向调研.md # 法奥FR+小车 新方向：文献调研与推进路径
├── inference/                 # 阶段二推理服务（conda/Docker 独立环境，非 ROS）
│   ├── server.py              # ZMQ 服务入口（--fake 假模式可先通链路）
│   └── wrappers/              # sam6d / foundationpose / grasp 模型包装（TODO 待集成）
├── scripts/
│   ├── check_build.sh         # 一键编译自检（Ubuntu）
│   ├── offline_pose_test.py   # 离线感知/配准测试（Windows 可跑）
│   ├── annotate_grasp.py      # CAD 抓取点标注 -> grasp_annotations（Windows 可跑）
│   ├── handeye_to_yaml.py     # easy_handeye2 结果 -> handeye_result.yaml
│   ├── record_pointcloud.sh   # 录点云 rosbag（Ubuntu）
│   └── replay_and_perceive.sh # 回放 rosbag + 感知，离线调参（Ubuntu）
└── ros2_ws/                   # ROS2 colcon 工作空间（拷到 Ubuntu 构建）
    └── src/
        ├── bin_picking_interfaces/   # 自定义消息/服务
        ├── bin_picking_perception/   # 阶段一感知：PPF+ICP 6D 位姿估计
        ├── bin_picking_perception_v2/# 阶段二感知：大模型推理客户端（ZMQ→inference/）
        ├── bin_picking_grasp/        # 夹爪驱动 + 抓取规划 + MoveIt2 执行 + 在手补偿 + 状态机
        ├── bin_picking_bringup/      # 总启动 launch + 仿真 + RViz + 手眼外参
        └── bin_picking_description/  # 料框场景 + 零件 CAD + 夹爪 URDF
```

## 构建（在 Ubuntu 上）

```bash
cd ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## 阶段进度

> 图例：✅ 代码已写（待真机测试） · ⬜ 未开始

- ⬜ 阶段 0：环境搭建（文档就绪 `docs/01_环境搭建.md`，需在 Ubuntu 执行）
- ✅ 阶段 1：手眼标定（标定 launch + 结果转换脚本 + 精度验证节点，`docs/05_手眼标定.md`；剩真机标定操作）
- ✅ 阶段 2：感知节点（PPF+ICP 位姿估计）— `bin_picking_perception`
- ✅ 阶段 3：抓取规划 + MoveIt2（RRTConnect 自由移动 + 笛卡尔直线接近 + 料框碰撞避障）
- ✅ 阶段 4：在手位姿重估计 + 放置补偿（`inhand_estimator` + 执行器补偿，`enable_inhand` 开关）
- ✅ 阶段 5：主状态机 + 鲁棒性（`pick_loop`：循环/清空判定/失败重试）
- ✅ 夹爪 URDF：宏 + UR5 组合（`ur5_with_gripper.xacro`，定义 TCP）；待填实测尺寸并集成进 MoveIt SRDF
- ✅ 工具：CAD 抓取点标注（`scripts/annotate_grasp.py`，Win 可跑）、录包回放（`scripts/*.sh`）
- 🔶 阶段二（大模型感知）：**骨架已写**（客户端/服务/launch/文档），模型本体待在
  Ubuntu+GPU 上按 `docs/08_阶段二大模型接入.md` 集成（wrapper 内为 TODO）

> 标 ✅ 表示**代码已写、待真机测试**。剩余硬件门控项见 `docs/00_开发计划.md`。
> 仿真验证：`ros2 launch bin_picking_bringup sim.launch.py`（无真机跑通逻辑）。
> 运行/测试见 `docs/02_运行说明.md`、`docs/03_测试步骤.md`。
