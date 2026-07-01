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
│   └── 04_夹爪URDF集成.md     # 沃姆夹爪装到 UR5
├── scripts/
│   ├── check_build.sh         # 一键编译自检（Ubuntu）
│   └── offline_pose_test.py   # 离线感知/配准测试（Windows 可跑）
└── ros2_ws/                   # ROS2 colcon 工作空间（拷到 Ubuntu 构建）
    └── src/
        ├── bin_picking_interfaces/   # 自定义消息/服务
        ├── bin_picking_perception/   # 感知：PPF+ICP 6D 位姿估计
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
- 🚧 夹爪 URDF：参数化占位（`worm_epgc50.xacro`），待填实测尺寸并集成进 MoveIt

> 标 ✅ 表示**代码已写、待真机测试**。剩余硬件门控项见 `docs/00_开发计划.md`。
> 仿真验证：`ros2 launch bin_picking_bringup sim.launch.py`（无真机跑通逻辑）。
> 运行/测试见 `docs/02_运行说明.md`、`docs/03_测试步骤.md`。
