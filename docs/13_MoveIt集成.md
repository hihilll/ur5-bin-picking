# 13 MoveIt2 集成（阶段3 联调）

> 2026-07 完成。把 UR5+沃姆夹爪作为一个整体交给 MoveIt2 规划：
> 组合 URDF/SRDF（含夹爪几何、指尖 TCP、自碰撞白名单）、自建 `move_group` 启动、
> 取放执行器改用 MoveIt **标准 action/service 接口**（不再依赖 `moveit_py`）。

## 为什么这样做

1. **`moveit_py` 在 Humble apt 版 MoveIt 里没有**（它是 MoveIt 2.7/Iron 才引入的）。
   旧 `grasp_executor` 用 `from moveit.planning import MoveItPy`，真机上 `import` 必失败、
   静默进"只打印"模拟模式——看起来在跑其实没动。现改为纯 `rclpy` 客户端，
   直接调 `move_group` 的标准接口，**任何 MoveIt2 版本都可用**：
   - 自由规划+执行：`/move_action`（`moveit_msgs/action/MoveGroup`）
   - 笛卡尔直线：`/compute_cartesian_path`（`srv`）+ `/execute_trajectory`（`action`）
   - 碰撞场景：`/apply_planning_scene`（`srv`）

2. **官方 `ur_moveit.launch.py` 只认裸 UR**：它的 SRDF 固定在 `ur_moveit_config`，
   MoveIt 看不到夹爪几何、也没有指尖 TCP。所以自建 `moveit.launch.py`，
   URDF/SRDF 换成"UR5+夹爪"组合，其余参数（kinematics/joint_limits/ompl）
   仍与官方 Humble 版一致（组装逻辑见 `bin_picking_bringup/moveit_params.py`，
   读不到官方包时用内置兜底值）。

## 新增/改动文件

| 文件 | 作用 |
|---|---|
| `bin_picking_description/urdf/ur5_with_gripper_control.xacro` | 驱动用描述文件：include 官方 `ur.urdf.xacro`（含 ros2_control）+ 挂夹爪宏。给 `ur_control.launch.py` 的 `description_file` |
| `bin_picking_description/srdf/ur5_with_gripper.srdf.xacro` | SRDF：官方 `ur_srdf` 宏 + 夹爪组/末端执行器/手指 passive/夹爪与手腕 allow-collision |
| `bin_picking_bringup/bin_picking_bringup/moveit_params.py` | 组装 `move_group`/RViz 的全部参数（URDF/SRDF/kinematics/joint_limits/ompl/controllers/…） |
| `bin_picking_bringup/launch/moveit.launch.py` | 起 `move_group`（+可选 RViz MotionPlanning） |
| `bin_picking_bringup/rviz/moveit.rviz` | RViz 配置（MotionPlanning + `/grasp_markers`） |
| `bin_picking_grasp/grasp_executor.py` | **重写**：标准接口客户端，限速/退化/超时/取消齐全 |
| `bin_picking_grasp/gripper_driver.py` | 新增向 `/joint_states` 发布手指开度（MoveIt 碰撞检测/RViz 用） |

三个总 launch（`system` / `sim` / `model_free_test`）已改为：
① 驱动 `description_package:=bin_picking_description description_file:=ur5_with_gripper_control.xacro`，
② 用自建 `moveit.launch.py` 代替官方 `ur_moveit.launch.py`。

## 关键参数（`grasp_params.yaml` 的 `grasp_executor`）

- `tcp_link: gripper_grasp_tcp` —— **抓取位姿现在以夹爪指尖为 TCP**（旧值 `tool0` 其实是错的，
  抓取点应是夹持中心而非法兰）。规划组仍是 `ur_manipulator`（base→tool0），
  MoveIt 对"固连在链末端之后的连杆"可直接求 IK / 笛卡尔，无需改规划组。
- `max_velocity_scaling` / `max_acceleration_scaling`：**真机首次务必设小（0.1~0.2）**，
  确认轨迹无误再调大。笛卡尔段 Humble 的服务固定按全速做时间参数化、请求端无缩放字段，
  执行器对返回轨迹做等效时间拉伸达到同样限速。
- `goal_position_tolerance` / `goal_orientation_tolerance`：位姿目标容差。
- `simulate: true`：不连 MoveIt，只打印每个动作（纯逻辑联调用）。
- 料框碰撞盒 `bin_center/bin_size/...`：启动后经 `/apply_planning_scene` 加入，做避障。

## 验证步骤

### A. 仿真（无真机，验证运动与取放逻辑）
```bash
ros2 launch bin_picking_bringup sim.launch.py
# 另开终端：喂一个测试抓取，触发一次取放
ros2 run bin_picking_grasp publish_test_grasp
ros2 service call /pick_place/run std_srvs/srv/Trigger {}
```
RViz 里应看到：UR5 **带夹爪** 模型、规划出轨迹、机械臂动起来走到抓取/放置位。
（`sim.launch` 用 UR fake hardware + 夹爪 `simulate`；手指开合随 `set_gripper` 变化。）

### B. 先只验 MoveIt 规划（RViz 手动拖动）
```bash
ros2 launch bin_picking_bringup sim.launch.py enable_grasp:=false
# RViz MotionPlanning 面板：拖动橙色交互球设目标 → Plan → Execute
```

### C. 真机干跑（真臂低速，先不放零件）
```bash
ros2 launch bin_picking_bringup system.launch.py robot_ip:=192.168.0.11 \
    cad_model_path:=/abs/.../part.stl
# 示教器先跑 External Control 程序。确认 grasp_params 的 max_*_scaling 很小。
ros2 service call /pick_place/run std_srvs/srv/Trigger {}
```

## 真机门控 / 待核对

- **[C1] 夹爪安装偏置**：`ur5_with_gripper_control.xacro` 与 `ur5_with_gripper.xacro`
  里 `<origin xyz="0 0 0.012">`（转接板厚）需按实物量，两文件保持一致。
  夹爪本体/指尺寸在 `worm_epgc50.xacro`（`docs/04`）。
- **[C2] kinematics 一致性**：`move_group` 与驱动默认都用 `ur_description` 出厂 kinematics，
  二者**一致**。若日后用 `calibration_correction` 提取了本台 UR5 的标定文件，
  需给两侧都传 `kinematics_params`（驱动的 `kinematics_params_file` +
  `moveit_params.robot_description()` 加 mapping），否则规划模型与真机有 mm 级偏差
  （手眼标定+抓取偏置通常能吸收，装配级精度才需要处理）。
- **[C3] fake vs 真机控制器**：两个轨迹控制器都注册、`scaled_joint_trajectory_controller`
  为默认；真机与 UR fake hardware 默认激活的都是它，故通用。
- **[C4] 规划组名**：`name:=ur` → 规划组 `ur_manipulator`，与 `grasp_params.yaml` 一致。
  若给 UR 加 `tf_prefix`，SRDF/参数需同步带前缀。
